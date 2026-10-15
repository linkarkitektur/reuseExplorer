// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/EdgeEditor.hpp>
#include <rux_qt/FrameBrowser.hpp>
#include <rux_qt/FrameImageLoader.hpp>
#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/widgets.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>
#include <reusex/core/label_semantics.hpp>

#include <QAbstractSpinBox>
#include <QApplication>
#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QListView>
#include <QLocale>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QScrollBar>
#include <QSlider>
#include <QSpinBox>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace rux::qt {
namespace {

QString qs(const std::string &s) { return QString::fromStdString(s); }

QString dec(double v, int decimals) {
  return qs(format_decimal_da(v, decimals));
}

/// Danish date and time with milliseconds, local time: "10. mar. 2026
/// 07:37:50,856".
QString format_time(double epoch_s) {
  if (epoch_s < 0)
    return "Ukendt";
  const auto ms = static_cast<qint64>(std::llround(epoch_s * 1000.0));
  const QDateTime dt = QDateTime::fromMSecsSinceEpoch(ms);
  const QLocale da(QLocale::Danish, QLocale::Denmark);
  return da.toString(dt.date(), "d. MMM yyyy") + " " +
         dt.time().toString("HH:mm:ss") + "," + dt.time().toString("zzz");
}

/// Linear blend of two token colours.
QRgb mix(const QColor &a, const QColor &b, double t) {
  QColor c;
  c.setRgbF(static_cast<float>(a.redF() + (b.redF() - a.redF()) * t),
            static_cast<float>(a.greenF() + (b.greenF() - a.greenF()) * t),
            static_cast<float>(a.blueF() + (b.blueF() - a.blueF()) * t));
  return c.rgb();
}

/// Index 0 of an indexed plane: no data, so the canvas shows through.
constexpr QRgb kNoData = 0; // fully transparent ARGB

int label_slots() {
  bool ok = false;
  const int n = theme().value("--label-count").toInt(&ok);
  return ok && n > 0 ? n : 1;
}

QString label_token(int class_id) {
  return QString("--label-%1").arg(class_id % label_slots());
}

QPushButton *segment(const QString &text, const char *position) {
  auto *b = new QPushButton(NavItem::escape_mnemonic(text));
  b->setObjectName("segment");
  b->setCheckable(true);
  b->setCursor(Qt::PointingHandCursor);
  b->setProperty("position", position);
  return b;
}

QPushButton *button(const QString &text, const char *kind) {
  auto *b = new QPushButton(NavItem::escape_mnemonic(text));
  b->setProperty("kind", kind);
  b->setCursor(Qt::PointingHandCursor);
  return b;
}

/// True when the widget keeps the arrow keys for its own cursor.
bool is_text_field(const QWidget *w) {
  if (!w)
    return false;
  if (qobject_cast<const QLineEdit *>(w) ||
      qobject_cast<const QAbstractSpinBox *>(w))
    return true;
  if (const auto *c = qobject_cast<const QComboBox *>(w))
    return c->isEditable();
  return false;
}

} // namespace

QVector<QRgb> depth_colour_table() {
  // Far = a dim tint over the canvas, near = the chrome's light ink. Index 0
  // (no return) is transparent, so the canvas shows through.
  const Theme &t = theme();
  const QColor canvas = t.color("--color-canvas");
  const QColor ink = t.color("--color-on-chrome");
  QVector<QRgb> table(256, kNoData);
  for (int i = 1; i < 256; ++i)
    table[i] = mix(canvas, ink, 0.12 + 0.88 * (i - 1) / 254.0);
  return table;
}

// ---------------------------------------------------------------- ImagePane --

ImagePane::ImagePane(QWidget *parent) : QWidget(parent) {
  setObjectName("imagePane");
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setMinimumSize(theme().px("--space-7") * 2, theme().px("--space-7") * 2);
  rebuild_tables();
  connect(&theme(), &Theme::changed, this, [this] {
    rebuild_tables();
    update();
  });
}

QSize ImagePane::sizeHint() const {
  return {theme().px("--layout-panel-width"),
          theme().px("--layout-panel-width") * 4 / 3};
}

void ImagePane::rebuild_tables() {
  const Theme &t = theme();
  depth_table_ = depth_colour_table();
  // ARKit confidence low / medium / high.
  conf_levels_table_.assign(256, kNoData);
  conf_levels_table_[1] = t.color("--color-status-failed").rgb();
  conf_levels_table_[2] = t.color("--label-3").rgb();
  conf_levels_table_[3] = t.color("--color-status-succeeded").rgb();
  label_table_.assign(256, kNoData);
  const int n = label_slots();
  for (int i = 1; i < 256; ++i)
    label_table_[i] = t.color(QString("--label-%1").arg((i - 1) % n)).rgba();
}

void ImagePane::set_frame(std::shared_ptr<const DecodedFrame> frame) {
  frame_ = std::move(frame);
  loading_ = false;
  message_.clear();
  update();
}

void ImagePane::set_layer(FrameLayer layer) {
  layer_ = layer;
  update();
}

void ImagePane::set_overlay(bool on, int opacity_percent) {
  overlay_ = on;
  opacity_ = opacity_percent;
  update();
}

void ImagePane::set_loading(bool loading) {
  loading_ = loading;
  update();
}

void ImagePane::set_message(const QString &text) {
  message_ = text;
  frame_.reset();
  update();
}

void ImagePane::paintEvent(QPaintEvent *) {
  const Theme &t = theme();
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  const qreal radius = t.px("--radius-md");
  QPainterPath clip;
  clip.addRoundedRect(QRectF(rect()), radius, radius);
  p.setClipPath(clip);
  p.fillRect(rect(), t.color("--color-canvas"));

  auto centred_text = [&](const QString &text) {
    p.setPen(t.color("--color-on-chrome-muted"));
    p.setFont(t.font(FontRole::sans, "--font-size-sm"));
    p.drawText(rect().adjusted(t.px("--space-4"), 0, -t.px("--space-4"), 0),
               Qt::AlignCenter | Qt::TextWordWrap, text);
  };

  if (!frame_) {
    centred_text(!message_.isEmpty() ? message_
                 : loading_          ? QString("Henter billedet …")
                                     : QString());
    return;
  }
  if (!frame_->error.isEmpty()) {
    centred_text("Billedet kunne ikke læses.\n" + frame_->error);
    return;
  }

  // The plane under the overlay, in its own colours.
  QImage base;
  QString caption;
  switch (layer_) {
  case FrameLayer::color:
    base = frame_->color;
    caption = "Farve";
    break;
  case FrameLayer::depth:
    base = frame_->depth;
    if (!base.isNull()) {
      base.setColorTable(depth_table_);
      caption =
          QString("Dybde · %1–%2 m")
              .arg(dec(frame_->depth_min_m, 2), dec(frame_->depth_max_m, 2));
    }
    break;
  case FrameLayer::confidence:
    base = frame_->confidence;
    if (!base.isNull()) {
      base.setColorTable(frame_->confidence_levels ? conf_levels_table_
                                                   : depth_table_);
      caption = frame_->confidence_levels ? "Konfidens · lav, middel, høj"
                                          : "Konfidens";
    }
    break;
  }

  // Fit the image's aspect: the colour frame decides it (depth and
  // confidence are lower-resolution captures of the same view).
  QSize aspect = !frame_->color.isNull() ? frame_->color.size() : base.size();
  if (aspect.isEmpty() && !frame_->labels.isNull())
    aspect = frame_->labels.size();
  if (aspect.isEmpty()) {
    centred_text("Billedet har ingen data.");
    return;
  }
  const QSize fitted = aspect.scaled(size(), Qt::KeepAspectRatio);
  const QRect target(
      QPoint((width() - fitted.width()) / 2, (height() - fitted.height()) / 2),
      fitted);

  if (!base.isNull()) {
    p.setRenderHint(QPainter::SmoothPixmapTransform, true);
    p.drawImage(target, base);
  } else {
    p.setPen(t.color("--color-on-chrome-muted"));
    p.setFont(t.font(FontRole::sans, "--font-size-sm"));
    const QString what = layer_ == FrameLayer::color   ? "farvebillede"
                         : layer_ == FrameLayer::depth ? "dybdebillede"
                                                       : "konfidensbillede";
    p.drawText(target, Qt::AlignCenter, QString("Intet %1").arg(what));
  }

  if (overlay_ && !frame_->labels.isNull() && opacity_ > 0) {
    QImage labels = frame_->labels;
    labels.setColorTable(label_table_);
    // Nearest-neighbour: blending two classes' colours would invent a third.
    p.setRenderHint(QPainter::SmoothPixmapTransform, false);
    p.setOpacity(opacity_ / 100.0);
    p.drawImage(target, labels);
    p.setOpacity(1.0);
  }

  // Caption chip, bottom left, on a scrim so it reads over any image —
  // but not over a thumbnail it would mostly cover (Rediger's pair images
  // yield their height to the 3D overlay; its header names the layer).
  const QFont chip_font =
      t.font(FontRole::sans, "--font-size-xs", "--font-weight-medium");
  const QFontMetrics fm(chip_font);
  const QRect text_rect(0, 0,
                        fm.horizontalAdvance(caption) + 2 * t.px("--space-2"),
                        fm.height() + t.px("--space-1") * 2);
  if (!caption.isEmpty() && target.height() >= 4 * text_rect.height() &&
      target.width() >= 2 * text_rect.width()) {
    p.setFont(chip_font);
    const int pad = t.px("--space-2");
    const QRect chip = text_rect.translated(
        target.left() + pad, target.bottom() - pad - text_rect.height());
    QColor scrim = t.color("--color-scrim");
    scrim.setAlphaF(0.72);
    p.setPen(Qt::NoPen);
    p.setBrush(scrim);
    p.drawRoundedRect(chip, t.px("--radius-sm"), t.px("--radius-sm"));
    p.setPen(t.color("--color-on-chrome"));
    p.drawText(chip, Qt::AlignCenter, caption);
  }
  if (loading_) {
    QColor veil = t.color("--color-canvas");
    veil.setAlphaF(0.45);
    p.fillRect(target, veil);
  }
}

// ---------------------------------------------------------------- FrameSide --

FrameSide::FrameSide(const QString &letter, QWidget *parent) : QFrame(parent) {
  setObjectName("frameSide");
  const Theme &t = theme();
  auto *v = new QVBoxLayout(this);
  const int pad = t.px("--space-3");
  v->setContentsMargins(pad, pad, pad, pad);
  v->setSpacing(t.px("--space-2"));

  auto *head = new QHBoxLayout;
  head->setSpacing(t.px("--space-2"));
  auto *badge = new QLabel(letter);
  badge->setObjectName("sideBadge");
  badge->setProperty("side", letter);
  badge->setAlignment(Qt::AlignCenter);
  head->addWidget(badge);
  id_ = new QSpinBox;
  id_->setObjectName("frameId");
  id_->setButtonSymbols(QAbstractSpinBox::NoButtons);
  id_->setToolTip("Billedets id — skriv et id og tryk Enter");
  id_->setKeyboardTracking(false);
  head->addWidget(id_);
  position_ = new QLabel;
  position_->setObjectName("sidePosition");
  head->addWidget(position_, 1);
  prev_ = button("‹", "ghost");
  prev_->setObjectName("stepButton");
  prev_->setToolTip(letter == "A" ? "Forrige billede (←)"
                                  : "Forrige billede (Shift+←)");
  next_ = button("›", "ghost");
  next_->setObjectName("stepButton");
  next_->setToolTip(letter == "A" ? "Næste billede (→)"
                                  : "Næste billede (Shift+→)");
  head->addWidget(prev_);
  head->addWidget(next_);
  v->addLayout(head);

  slider_ = new QSlider(Qt::Horizontal);
  slider_->setObjectName("frameSlider");
  slider_->setProperty("side", letter);
  slider_->setFocusPolicy(Qt::NoFocus); // the browser owns ←/→
  v->addWidget(slider_);

  image_ = new ImagePane;
  v->addWidget(image_, 1);

  legend_ = new QLabel;
  legend_->setObjectName("legendLine");
  legend_->setWordWrap(true);
  legend_->setTextFormat(Qt::RichText);
  v->addWidget(legend_);

  meta_box_ = new QVBoxLayout;
  meta_box_->setContentsMargins(0, 0, 0, 0);
  v->addLayout(meta_box_);

  connect(slider_, &QSlider::valueChanged, this, [this](int i) {
    if (i != index_)
      emit index_requested(i);
  });
  connect(prev_, &QPushButton::clicked, this, [this] {
    if (index_ > 0)
      emit index_requested(index_ - 1);
  });
  connect(next_, &QPushButton::clicked, this, [this] {
    if (index_ + 1 < static_cast<int>(ids_.size()))
      emit index_requested(index_ + 1);
  });
  connect(id_, &QSpinBox::valueChanged, this, [this](int id) {
    if (ids_.empty() || (index_ >= 0 && ids_[index_] == id))
      return;
    // Nearest frame: ids need not be contiguous.
    const auto it = std::lower_bound(ids_.begin(), ids_.end(), id);
    int i = static_cast<int>(it - ids_.begin());
    if (it == ids_.end() || (it != ids_.begin() && *it - id > id - *(it - 1)))
      i = std::max(0, i - 1);
    emit index_requested(i);
  });
  connect(&theme(), &Theme::changed, this, &FrameSide::refresh_legend);
  set_index(-1);
}

void FrameSide::set_ids(const std::vector<int> &ids) {
  ids_ = ids;
  const QSignalBlocker b1(slider_), b2(id_);
  slider_->setRange(0, std::max(0, static_cast<int>(ids_.size()) - 1));
  slider_->setPageStep(std::max(1, static_cast<int>(ids_.size()) / 20));
  id_->setRange(ids_.empty() ? 0 : ids_.front(),
                ids_.empty() ? 0 : ids_.back());
  id_->setFixedWidth(QFontMetrics(id_->font())
                         .horizontalAdvance(QString::number(id_->maximum())) +
                     theme().px("--space-5"));
  const bool any = !ids_.empty();
  slider_->setEnabled(ids_.size() > 1);
  id_->setEnabled(any);
}

void FrameSide::set_index(int index) {
  index_ = index;
  const QSignalBlocker b1(slider_), b2(id_);
  const int n = static_cast<int>(ids_.size());
  if (index < 0 || index >= n) {
    position_->setText(QString());
    prev_->setEnabled(false);
    next_->setEnabled(false);
    return;
  }
  slider_->setValue(index);
  id_->setValue(ids_[index]);
  position_->setText(QString("%1 af %2")
                         .arg(format_count(index + 1))
                         .arg(format_count(static_cast<qulonglong>(n))));
  prev_->setEnabled(index > 0);
  next_->setEnabled(index + 1 < n);
}

void FrameSide::set_frame(std::shared_ptr<const DecodedFrame> frame) {
  frame_ = frame;
  image_->set_frame(std::move(frame));
  refresh_legend();
}

void FrameSide::set_loading(bool loading) { image_->set_loading(loading); }
void FrameSide::set_layer(FrameLayer layer) { image_->set_layer(layer); }

void FrameSide::set_overlay(bool on, int opacity_percent) {
  overlay_ = on;
  image_->set_overlay(on, opacity_percent);
  refresh_legend();
}

void FrameSide::set_message(const QString &text) {
  frame_.reset();
  image_->set_message(text);
  refresh_legend();
}

void FrameSide::refresh_legend() {
  // The classes in this frame, largest first, as coloured chips. Rich text
  // takes the colour value itself, read from the token on every theme.
  if (!frame_ || frame_->labels.isNull()) {
    legend_->setText(frame_ && overlay_ ? QString("Ingen mærkater i billedet")
                                        : QString());
    legend_->setVisible(frame_ && overlay_);
    return;
  }
  legend_->setVisible(overlay_);
  if (!overlay_)
    return;
  const Theme &t = theme();
  QStringList parts;
  const int shown = 6;
  int i = 0;
  for (const auto &[cls, px] : frame_->label_pixels) {
    if (i++ == shown)
      break;
    QString name = QString("klasse %1").arg(cls);
    if (names_) {
      if (auto it = names_->find(cls); it != names_->end())
        name = QString::fromStdString(it->second);
    }
    const double pct = 100.0 * px / std::max(1, frame_->label_total);
    parts << QString("<span style=\"color:%1\">■</span>&nbsp;%2&nbsp;"
                     "<span style=\"color:%3\">%4&nbsp;%</span>")
                 .arg(t.color(label_token(cls)).name(),
                      name.toHtmlEscaped().replace(' ', "&nbsp;"),
                      t.color("--color-text-faint").name(),
                      dec(pct, pct < 10 ? 1 : 0));
  }
  if (static_cast<int>(frame_->label_pixels.size()) > shown)
    parts << QString("<span style=\"color:%1\">+%2</span>")
                 .arg(t.color("--color-text-faint").name())
                 .arg(frame_->label_pixels.size() - shown);
  legend_->setText(parts.join("&nbsp;&nbsp;&nbsp;"));
}

void FrameSide::set_metadata(const QVector<std::pair<QString, QString>> &rows) {
  if (meta_) {
    meta_->hide(); // a widget waiting for deleteLater still paints
    meta_->deleteLater();
  }
  meta_ = new PropertyList;
  for (const auto &[k, v] : rows)
    meta_->add(k, v);
  meta_box_->addWidget(meta_);
}

void FrameSide::mousePressEvent(QMouseEvent *e) {
  emit activated();
  QFrame::mousePressEvent(e);
}

// ----------------------------------------------------------- FilmstripModel --

FilmstripModel::FilmstripModel(FrameImageLoader &loader, QObject *parent)
    : QAbstractListModel(parent), loader_(loader) {
  connect(&loader_, &FrameImageLoader::thumbnail_ready, this,
          &FilmstripModel::thumbnail_ready);
}

void FilmstripModel::set_frames(std::vector<int> ids, QSet<int> segmented) {
  beginResetModel();
  ids_ = std::move(ids);
  segmented_ = std::move(segmented);
  endResetModel();
}

void FilmstripModel::set_marks(int a_id, int b_id) {
  const int old_a = a_, old_b = b_;
  a_ = a_id;
  b_ = b_id;
  for (int id : {old_a, old_b, a_, b_}) {
    const auto it = std::lower_bound(ids_.begin(), ids_.end(), id);
    if (it != ids_.end() && *it == id) {
      const QModelIndex i = index(static_cast<int>(it - ids_.begin()));
      emit dataChanged(i, i, {MarkRole});
    }
  }
}

void FilmstripModel::set_degrees(QHash<int, int> degrees) {
  degrees_ = std::move(degrees);
  if (!ids_.empty())
    emit dataChanged(index(0), index(rowCount() - 1), {DegreeRole});
}

int FilmstripModel::rowCount(const QModelIndex &parent) const {
  return parent.isValid() ? 0 : static_cast<int>(ids_.size());
}

QVariant FilmstripModel::data(const QModelIndex &index, int role) const {
  if (!index.isValid() || index.row() >= rowCount())
    return {};
  const int id = ids_[static_cast<std::size_t>(index.row())];
  switch (role) {
  case IdRole:
    return id;
  case ThumbRole: {
    QImage img = loader_.thumbnail(id);
    if (img.isNull())
      const_cast<FrameImageLoader &>(loader_).request_thumbnail(id);
    return img;
  }
  case MarkRole:
    return id == a_ && id == b_ ? QString("AB")
           : id == a_           ? QString("A")
           : id == b_           ? QString("B")
                                : QString();
  case SegmentedRole:
    return segmented_.contains(id);
  case DegreeRole:
    return degrees_.value(id);
  case Qt::ToolTipRole: {
    QStringList t{QString("Billede %1").arg(id)};
    if (segmented_.contains(id))
      t << "segmenteret";
    if (const int d = degrees_.value(id))
      t << (d == 1 ? QString("1 kant") : QString("%1 kanter").arg(d));
    return t.join(" · ") + "\nKlik: A · Shift+klik: B";
  }
  default:
    return {};
  }
}

void FilmstripModel::thumbnail_ready(int id) {
  const auto it = std::lower_bound(ids_.begin(), ids_.end(), id);
  if (it == ids_.end() || *it != id)
    return;
  const QModelIndex i = index(static_cast<int>(it - ids_.begin()));
  emit dataChanged(i, i, {ThumbRole});
}

// -------------------------------------------------------- FilmstripDelegate --

int FilmstripDelegate::thumb_height() {
  return theme().px("--space-7") + theme().px("--space-6");
}

QSize FilmstripDelegate::sizeHint(const QStyleOptionViewItem &,
                                  const QModelIndex &) const {
  const int h = thumb_height();
  return {h * 3 / 4 + theme().px("--space-2"),
          h + theme().px("--space-2") + theme().px("--space-4")};
}

void FilmstripDelegate::paint(QPainter *p, const QStyleOptionViewItem &option,
                              const QModelIndex &index) const {
  const Theme &t = theme();
  p->save();
  p->setRenderHint(QPainter::Antialiasing);
  p->setRenderHint(QPainter::SmoothPixmapTransform);
  const int gap = t.px("--space-1");
  const int h = thumb_height();
  QRect thumb = option.rect.adjusted(gap, gap, -gap, 0);
  thumb.setHeight(h);
  const qreal r = t.px("--radius-sm");

  QPainterPath clip;
  clip.addRoundedRect(QRectF(thumb), r, r);
  p->fillPath(clip, t.color("--color-canvas"));
  const QImage img = index.data(FilmstripModel::ThumbRole).value<QImage>();
  if (img.width() > 1) {
    const QSize fitted = img.size().scaled(thumb.size(), Qt::KeepAspectRatio);
    const QRect target(thumb.center().x() - fitted.width() / 2,
                       thumb.center().y() - fitted.height() / 2, fitted.width(),
                       fitted.height());
    p->setClipPath(clip);
    p->drawImage(target, img);
    p->setClipping(false);
  }

  // A / B: an accent frame for A, a strong neutral one for B, and a chip.
  const QString mark = index.data(FilmstripModel::MarkRole).toString();
  if (!mark.isEmpty()) {
    const QColor c = mark.startsWith('A') ? t.color("--color-accent")
                                          : t.color("--color-text");
    p->setPen(QPen(c, 2));
    p->setBrush(Qt::NoBrush);
    p->drawRoundedRect(QRectF(thumb).adjusted(1, 1, -1, -1), r, r);
    const QFont f =
        t.font(FontRole::sans, "--font-size-2xs", "--font-weight-bold");
    p->setFont(f);
    const int cw = QFontMetrics(f).horizontalAdvance(mark) + 2 * gap * 2;
    const QRect chip(thumb.left() + gap, thumb.top() + gap, cw,
                     QFontMetrics(f).height() + gap);
    p->setPen(Qt::NoPen);
    p->setBrush(c);
    p->drawRoundedRect(chip, r, r);
    p->setPen(mark.startsWith('A') ? t.color("--color-on-accent")
                                   : t.color("--color-surface"));
    p->drawText(chip, Qt::AlignCenter, mark);
  } else if (option.state & QStyle::State_MouseOver) {
    p->setPen(QPen(t.color("--color-border-strong"), 1));
    p->setBrush(Qt::NoBrush);
    p->drawRoundedRect(QRectF(thumb).adjusted(0.5, 0.5, -0.5, -0.5), r, r);
  }

  // Under the thumbnail: the id, and dots for segmentation and graph edges.
  const QRect foot(thumb.left(), thumb.bottom() + gap, thumb.width(),
                   t.px("--space-4"));
  p->setFont(t.font(FontRole::mono, "--font-size-2xs"));
  p->setPen(mark.isEmpty() ? t.color("--color-text-faint")
                           : t.color("--color-text"));
  p->drawText(foot, Qt::AlignLeft | Qt::AlignVCenter,
              QString::number(index.data(FilmstripModel::IdRole).toInt()));
  const int dot = gap + gap / 2 + 1;
  int x = foot.right() - dot;
  const int cy = foot.center().y();
  p->setPen(Qt::NoPen);
  if (index.data(FilmstripModel::DegreeRole).toInt() > 0) {
    p->setBrush(t.color("--color-accent"));
    p->drawEllipse(QPoint(x, cy), dot / 2 + 1, dot / 2 + 1);
    x -= dot + gap;
  }
  if (index.data(FilmstripModel::SegmentedRole).toBool()) {
    p->setBrush(t.color("--color-status-succeeded"));
    p->drawEllipse(QPoint(x, cy), dot / 2 + 1, dot / 2 + 1);
  }
  p->restore();
}

// ------------------------------------------------------------- FrameBrowser --

FrameBrowser::FrameBrowser(ProjectSession &session, EdgeEditor &editor,
                           FrameImageLoader &loader, QWidget *parent)
    : QWidget(parent), session_(session), editor_(editor), loader_(loader) {
  setObjectName("frameBrowser");
  setFocusPolicy(Qt::StrongFocus);
  const Theme &t = theme();
  auto *outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  outer->setSpacing(0);

  // ---- toolbar: layer, label overlay, opacity, key hint
  auto *bar = new QFrame;
  bar->setObjectName("dbToolbar");
  auto *bl = new QHBoxLayout(bar);
  bl->setContentsMargins(t.px("--space-4"), t.px("--space-2"),
                         t.px("--space-4"), t.px("--space-2"));
  bl->setSpacing(t.px("--space-3"));
  auto *title = new CapsLabel("Billeder", "panelTitle");
  bl->addWidget(title);
  subtitle_ = new QLabel;
  subtitle_->setObjectName("toolbarMeta");
  bl->addWidget(subtitle_);
  bl->addStretch(1);

  auto *seg = new QWidget;
  seg->setObjectName("segmented");
  auto *sl = new QHBoxLayout(seg);
  sl->setContentsMargins(0, 0, 0, 0);
  sl->setSpacing(0);
  layers_ = new QButtonGroup(this);
  layers_->setExclusive(true);
  const char *pos[] = {"first", "middle", "last"};
  const QString names[] = {"Farve", "Dybde", "Konfidens"};
  for (int i = 0; i < 3; ++i) {
    auto *b = segment(names[i], pos[i]);
    b->setFocusPolicy(Qt::NoFocus);
    layers_->addButton(b, i);
    sl->addWidget(b);
  }
  layers_->button(0)->setChecked(true);
  bl->addWidget(seg);

  overlay_box_ = new QCheckBox("Mærkater");
  overlay_box_->setChecked(true);
  overlay_box_->setFocusPolicy(Qt::NoFocus);
  overlay_box_->setToolTip("Vis segmenteringen oven på billedet");
  bl->addWidget(overlay_box_);
  opacity_slider_ = new QSlider(Qt::Horizontal);
  opacity_slider_->setObjectName("opacitySlider");
  opacity_slider_->setRange(0, 100);
  opacity_slider_->setValue(opacity_);
  opacity_slider_->setFixedWidth(t.px("--space-7") * 2);
  opacity_slider_->setFocusPolicy(Qt::NoFocus);
  opacity_slider_->setToolTip("Mærkaternes dækkevne");
  bl->addWidget(opacity_slider_);
  outer->addWidget(bar);

  // ---- body: A | B, pair strip, filmstrip
  body_ = new QWidget;
  body_->setObjectName("dbBody");
  auto *bv = new QVBoxLayout(body_);
  const int m = t.px("--space-4");
  bv->setContentsMargins(m, t.px("--space-3"), m, t.px("--space-3"));
  bv->setSpacing(t.px("--space-3"));
  auto *sides = new QHBoxLayout;
  sides->setSpacing(t.px("--space-3"));
  side_a_ = new FrameSide("A");
  side_b_ = new FrameSide("B");
  side_a_->set_label_names(&label_names_);
  side_b_->set_label_names(&label_names_);
  sides->addWidget(side_a_, 1);
  sides->addWidget(side_b_, 1);
  bv->addLayout(sides, 1);

  film_model_ = new FilmstripModel(loader_, this);
  film_ = new QListView;
  film_->setObjectName("filmstrip");
  film_->setModel(film_model_);
  film_->setItemDelegate(new FilmstripDelegate(film_));
  film_->setViewMode(QListView::IconMode);
  film_->setFlow(QListView::LeftToRight);
  film_->setWrapping(false);
  film_->setMovement(QListView::Static);
  film_->setUniformItemSizes(true);
  film_->setSelectionMode(QAbstractItemView::NoSelection);
  film_->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
  film_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  film_->setMouseTracking(true);
  film_->setFocusPolicy(Qt::NoFocus);
  film_->setFrameShape(QFrame::NoFrame);
  const QSize cell = FilmstripDelegate().sizeHint({}, {});
  film_->setFixedHeight(cell.height() + t.px("--space-3"));
  bv->addWidget(film_);
  outer->addWidget(body_, 1);

  empty_ = new QLabel;
  empty_->setObjectName("emptyText");
  empty_->setAlignment(Qt::AlignCenter);
  empty_->setWordWrap(true);
  outer->addWidget(empty_, 1);

  // ---- wiring
  connect(layers_, &QButtonGroup::idClicked, this, [this](int id) {
    layer_ = static_cast<FrameLayer>(id);
    apply_view();
  });
  connect(overlay_box_, &QCheckBox::toggled, this, [this](bool on) {
    overlay_ = on;
    opacity_slider_->setEnabled(on);
    apply_view();
  });
  connect(opacity_slider_, &QSlider::valueChanged, this, [this](int v) {
    opacity_ = v;
    apply_view();
  });
  connect(side_a_, &FrameSide::index_requested, this, [this](int i) {
    if (pair_.set_a_index(i))
      sync_side(false);
  });
  connect(side_b_, &FrameSide::index_requested, this, [this](int i) {
    if (pair_.set_b_index(i))
      sync_side(true);
  });
  connect(side_a_, &FrameSide::activated, this, [this] {
    selection_is_b_ = false;
    emit selection_changed(frame_selection(false));
  });
  connect(side_b_, &FrameSide::activated, this, [this] {
    selection_is_b_ = true;
    emit selection_changed(frame_selection(true));
  });
  connect(film_, &QListView::clicked, this, [this](const QModelIndex &i) {
    const int id = i.data(FilmstripModel::IdRole).toInt();
    if (QApplication::keyboardModifiers() & Qt::ShiftModifier)
      set_b(id);
    else
      set_a(id);
    setFocus();
  });
  connect(&loader_, &FrameImageLoader::frame_ready, this,
          &FrameBrowser::frame_ready);
  connect(&editor_, &EdgeEditor::changed, this,
          &FrameBrowser::refresh_strip_marks);
  qApp->installEventFilter(this);
  apply_view();
  reload();
}

FrameBrowser::~FrameBrowser() { qApp->removeEventFilter(this); }

void FrameBrowser::reload() {
  label_names_.clear();
  segmented_.clear();
  std::vector<int> ids;
  reusex::ProjectDB *db = session_.db();
  if (db) {
    try {
      ids = db->sensor_frame_ids();
      for (int id : db->segmentation_image_ids())
        segmented_.insert(id);
      if (db->has_point_cloud("labels"))
        label_names_ = db->label_definitions("labels");
    } catch (const std::exception &e) {
      empty_->setText(QString("Billederne kunne ikke læses.\n%1")
                          .arg(QString::fromUtf8(e.what())));
    }
  }
  pair_.reset(ids);
  side_a_->set_ids(pair_.ids());
  side_b_->set_ids(pair_.ids());
  film_model_->set_frames(pair_.ids(), segmented_);
  loader_.set_project(db ? session_.path() : QString());
  loader_.set_label_slots(label_slots());
  loader_.set_thumbnail_size(static_cast<int>(
      FilmstripDelegate::thumb_height() * devicePixelRatioF()));

  const bool any = !pair_.empty();
  body_->setVisible(any);
  empty_->setVisible(!any);
  if (!db)
    empty_->setText("Åbn et projekt for at se dets billeder.");
  else if (!any && empty_->text().isEmpty())
    empty_->setText("Projektet har ingen billeder endnu.\nImportér en "
                    "scanning med rux import rtabmap (eller arkitscenes, "
                    "mushroom).");
  subtitle_->setText(
      any ? QString("%1 billeder · %2 segmenteret")
                .arg(format_count(static_cast<qulonglong>(pair_.size())))
                .arg(format_count(static_cast<qulonglong>(segmented_.size())))
          : QString());
  refresh_strip_marks();
  sync_sides();
}

void FrameBrowser::show_a_near(int id) {
  const int i = pair_.nearest_index(id);
  if (i >= 0 && pair_.set_a_index(i))
    sync_side(false);
}

void FrameBrowser::set_a(int id) {
  if (pair_.set_a_id(id))
    sync_side(false);
}

void FrameBrowser::set_b(int id) {
  if (pair_.set_b_id(id))
    sync_side(true);
}

void FrameBrowser::apply_view() {
  for (FrameSide *s : {side_a_, side_b_}) {
    s->set_layer(layer_);
    s->set_overlay(overlay_, opacity_);
  }
}

void FrameBrowser::sync_sides() {
  sync_side(false);
  sync_side(true);
}

void FrameBrowser::sync_side(bool side_b) {
  FrameSide *side = side_b ? side_b_ : side_a_;
  const int index = side_b ? pair_.b_index() : pair_.a_index();
  const int id = side_b ? pair_.b_id() : pair_.a_id();
  side->set_index(index);
  if (id < 0) {
    side->set_message(QString());
    side->set_metadata({});
  } else {
    side->set_metadata(metadata(id));
    if (auto f = loader_.frame(id)) {
      side->set_frame(f);
    } else {
      side->set_loading(true);
      loader_.request(id);
    }
  }
  film_model_->set_marks(pair_.a_id(), pair_.b_id());
  emit pair_changed(pair_.a_id(), pair_.b_id());
  if (!side_b && index >= 0)
    film_->scrollTo(film_model_->index(index),
                    QAbstractItemView::EnsureVisible);
  if (side_b == selection_is_b_ || id < 0)
    emit selection_changed(frame_selection(selection_is_b_));
}

void FrameBrowser::frame_ready(int id) {
  auto f = loader_.frame(id);
  if (!f)
    return;
  if (id == pair_.a_id())
    side_a_->set_frame(f);
  if (id == pair_.b_id())
    side_b_->set_frame(f);
  if (id == (selection_is_b_ ? pair_.b_id() : pair_.a_id()))
    emit selection_changed(frame_selection(selection_is_b_));
}

void FrameBrowser::refresh_strip_marks() {
  // One pass over the edges (a project may have tens of thousands).
  QHash<int, int> degrees;
  for (const auto &[frame, n] : editor_.edits().degrees())
    degrees.insert(frame, n);
  // Frames whose edges are all rejected (stored or staged) show a 0 mark.
  for (const auto &v : editor_.edits().effective())
    for (int f : {v.edge.key.from, v.edge.key.to})
      if (!degrees.contains(f))
        degrees.insert(f, 0);
  film_model_->set_degrees(std::move(degrees));
}

QVector<std::pair<QString, QString>> FrameBrowser::metadata(int id) const {
  QVector<std::pair<QString, QString>> rows;
  const reusex::ProjectDB *db = session_.db();
  if (!db)
    return rows;
  try {
    rows.push_back({"Tid", format_time(db->sensor_frame_timestamp(id))});
    if (db->has_sensor_frame_pose(id)) {
      const auto s = summarize_pose(db->sensor_frame_pose(id));
      rows.push_back({"Position", QString("%1  %2  %3 m")
                                      .arg(dec(s.t[0], 2), dec(s.t[1], 2),
                                           dec(s.t[2], 2))});
      rows.push_back(
          {"Retning", QString("%1°  %2°  %3°")
                          .arg(dec(s.yaw_deg, 1), dec(s.pitch_deg, 1),
                               dec(s.roll_deg, 1))});
    } else {
      rows.push_back({"Pose", "Ingen gyldig pose"});
    }
    const auto k = db->sensor_frame_intrinsics(id);
    if (k.fx > 0)
      rows.push_back(
          {"Kamera", QString("f %1 · c %2 / %3")
                         .arg(dec(k.fx, 1), dec(k.cx, 1), dec(k.cy, 1))});
  } catch (const std::exception &e) {
    rows.push_back({"Fejl", QString::fromUtf8(e.what())});
  }
  return rows;
}

Selection FrameBrowser::frame_selection(bool side_b) const {
  Selection s;
  const int id = side_b ? pair_.b_id() : pair_.a_id();
  const reusex::ProjectDB *db = session_.db();
  if (id < 0 || !db)
    return s;
  const int index = side_b ? pair_.b_index() : pair_.a_index();
  s.kind = side_b ? "Billede B" : "Billede A";
  s.title = QString("Billede %1").arg(id);
  s.subtitle = QString("Nr. %1 af %2")
                   .arg(format_count(static_cast<qulonglong>(index + 1)))
                   .arg(format_count(static_cast<qulonglong>(pair_.size())));
  try {
    const bool posed = db->has_sensor_frame_pose(id);
    s.pill = posed ? (segmented_.contains(id) ? "Segmenteret" : "Med pose")
                   : "Ingen pose";
    s.pill_tone = posed ? "good" : "warn";

    SelectionSection when{"Tid", {}, {}};
    const double ts = db->sensor_frame_timestamp(id);
    if (ts >= 0) {
      const auto ms = static_cast<qint64>(std::llround(ts * 1000.0));
      const QDateTime dt = QDateTime::fromMSecsSinceEpoch(ms);
      when.rows.push_back({"Dato", QLocale(QLocale::Danish, QLocale::Denmark)
                                       .toString(dt.date(), "d. MMM yyyy")});
      when.rows.push_back({"Klokken", dt.time().toString("HH:mm:ss") + "," +
                                          dt.time().toString("zzz")});
    } else {
      when.rows.push_back({"Optaget", "Ukendt", SelectionRow::Style::value});
    }
    if (ts >= 0)
      when.rows.push_back({"Epoke", dec(ts, 3) + " s"});
    s.sections.push_back(when);

    SelectionSection pose{"Pose", {}, {}};
    const auto m = db->sensor_frame_pose(id);
    if (posed) {
      const auto ps = summarize_pose(m);
      pose.rows.push_back({"x", dec(ps.t[0], 3) + " m"});
      pose.rows.push_back({"y", dec(ps.t[1], 3) + " m"});
      pose.rows.push_back({"z", dec(ps.t[2], 3) + " m"});
      pose.rows.push_back({"Yaw", dec(ps.yaw_deg, 2) + "°"});
      pose.rows.push_back({"Pitch", dec(ps.pitch_deg, 2) + "°"});
      pose.rows.push_back({"Roll", dec(ps.roll_deg, 2) + "°"});
    } else {
      pose.rows.push_back(
          {"Status", "Ingen gyldig pose", SelectionRow::Style::value});
    }
    QStringList lines;
    for (int r = 0; r < 4; ++r) {
      QStringList cells;
      for (int c = 0; c < 4; ++c)
        cells << dec(m[r * 4 + c], 4).rightJustified(8);
      lines << cells.join(" ");
    }
    pose.block = lines.join('\n');
    s.sections.push_back(pose);

    const auto k = db->sensor_frame_intrinsics(id);
    SelectionSection cam{"Kamera", {}, {}};
    cam.rows.push_back({"fx", dec(k.fx, 2)});
    cam.rows.push_back({"fy", dec(k.fy, 2)});
    cam.rows.push_back({"cx", dec(k.cx, 2)});
    cam.rows.push_back({"cy", dec(k.cy, 2)});
    cam.rows.push_back(
        {"Opløsning", QString("%1 × %2").arg(k.width).arg(k.height)});
    s.sections.push_back(cam);

    SelectionSection data{"Data", {}, {}};
    if (auto f = loader_.frame(id)) {
      data.rows.push_back({"Farve", f->color.isNull()
                                        ? QString("—")
                                        : QString("%1 × %2")
                                              .arg(f->color.width())
                                              .arg(f->color.height())});
      data.rows.push_back({"Dybde", f->depth.isNull()
                                        ? QString("—")
                                        : QString("%1 × %2")
                                              .arg(f->depth.width())
                                              .arg(f->depth.height())});
      if (!f->depth.isNull())
        data.rows.push_back(
            {"Område", QString("%1–%2 m").arg(dec(f->depth_min_m, 2),
                                              dec(f->depth_max_m, 2))});
      data.rows.push_back({"Konf.", f->confidence.isNull()
                                        ? QString("—")
                                        : QString("%1 × %2")
                                              .arg(f->confidence.width())
                                              .arg(f->confidence.height())});
    }
    const int degree = editor_.edits().degree(id);
    data.rows.push_back({"Kanter", QString::number(degree)});
    s.sections.push_back(data);

    if (auto f = loader_.frame(id); f && !f->label_pixels.empty()) {
      SelectionSection labels{"Mærkater", {}, {}};
      for (const auto &[cls, px] : f->label_pixels) {
        // `cls` is the image's API class; the "labels" definitions (and the
        // "Klasse N" names the 3D legend and survey use) are keyed by point
        // label = API class + 1 (STANDARDS §3.1).
        const int point_label =
            static_cast<int>(reusex::core::api_to_point_label(cls));
        QString name = QString("klasse %1").arg(point_label);
        if (auto it = label_names_.find(point_label); it != label_names_.end())
          name = QString::fromStdString(it->second);
        SelectionRow row{
            name, dec(100.0 * px / std::max(1, f->label_total), 1) + " %",
            SelectionRow::Style::name, label_token(cls)};
        labels.rows.push_back(row);
      }
      s.sections.push_back(labels);
    }
  } catch (const std::exception &e) {
    s.sections.push_back({"Fejl", {}, QString::fromUtf8(e.what())});
  }
  return s;
}

void FrameBrowser::showEvent(QShowEvent *e) {
  QWidget::showEvent(e);
  setFocus();
}

bool FrameBrowser::eventFilter(QObject *watched, QEvent *event) {
  if (event->type() != QEvent::KeyPress || !isVisible() || pair_.empty())
    return false;
  // Only while the focus is in the browser (or nowhere): the project tree
  // and the tables keep their own arrow keys.
  QWidget *focus = QApplication::focusWidget();
  if (focus && focus != this && !isAncestorOf(focus))
    return false;
  if (watched != focus && !(focus == nullptr && watched == window()))
    return false;
  auto *k = static_cast<QKeyEvent *>(event);
  const ArrowKey arrow = k->key() == Qt::Key_Left    ? ArrowKey::left
                         : k->key() == Qt::Key_Right ? ArrowKey::right
                                                     : ArrowKey::other;
  const BrowserKey action = browser_key(
      arrow, k->modifiers() & Qt::ShiftModifier, is_text_field(focus));
  const int step = browser_step(k->modifiers() & Qt::ControlModifier);
  switch (action) {
  case BrowserKey::none:
    return false;
  case BrowserKey::a_prev:
    if (pair_.step_a(-step))
      sync_side(false);
    break;
  case BrowserKey::a_next:
    if (pair_.step_a(step))
      sync_side(false);
    break;
  case BrowserKey::b_prev:
    if (pair_.step_b(-step))
      sync_side(true);
    break;
  case BrowserKey::b_next:
    if (pair_.step_b(step))
      sync_side(true);
    break;
  }
  return true;
}

} // namespace rux::qt
