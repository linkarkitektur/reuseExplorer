// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/PanoramaViewer.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/background.hpp>
#include <rux_qt/database_logic.hpp>
#include <rux_qt/graph_view.hpp>
#include <rux_qt/widgets.hpp>

#include <reusex/core/ProjectDB.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <QApplication>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QPointer>
#include <QResizeEvent>
#include <QStackedLayout>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <thread>

namespace rux::qt {

// ------------------------------------------------------------ PanoramaView --

PanoramaView::PanoramaView(QWidget *parent) : QGraphicsView(parent) {
  setObjectName("panoramaView");
  setDragMode(QGraphicsView::ScrollHandDrag);
  setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
  setResizeAnchor(QGraphicsView::AnchorViewCenter);
  setRenderHint(QPainter::SmoothPixmapTransform);
  setFrameShape(QFrame::NoFrame);
  setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setFocusPolicy(Qt::StrongFocus);
}

void PanoramaView::fit() {
  if (!scene() || scene()->itemsBoundingRect().isEmpty())
    return;
  fitInView(scene()->itemsBoundingRect(), Qt::KeepAspectRatio);
  user_view_ = false;
}

void PanoramaView::wheelEvent(QWheelEvent *e) {
  e->accept();
  const QRectF r = scene() ? scene()->itemsBoundingRect() : QRectF();
  if (r.isEmpty())
    return;
  // Image pixels per screen pixel: in to 8x magnification, out to the fit.
  const double upp = 1.0 / std::max(transform().m11(), 1e-12);
  ZoomLimits z;
  z.min_units_per_px = 0.125;
  z.max_units_per_px = std::max(r.width() / std::max(viewport()->width(), 1),
                                r.height() / std::max(viewport()->height(), 1));
  z.max_units_per_px = std::max(z.max_units_per_px, z.min_units_per_px);
  const double f =
      clamp_zoom_factor(upp, std::pow(1.0015, e->angleDelta().y()), z);
  if (f != 1.0)
    scale(f, f);
  user_view_ = true;
}

void PanoramaView::keyPressEvent(QKeyEvent *e) {
  if (e->key() == Qt::Key_F && e->modifiers() == Qt::NoModifier) {
    fit();
    return;
  }
  QGraphicsView::keyPressEvent(e);
}

void PanoramaView::resizeEvent(QResizeEvent *e) {
  QGraphicsView::resizeEvent(e);
  if (!user_view_)
    fit();
}

// ---------------------------------------------------------- PanoramaViewer --

PanoramaViewer::PanoramaViewer(QWidget *parent) : QWidget(parent) {
  setObjectName("panoramaViewer");
  const Theme &t = theme();
  auto *v = new QVBoxLayout(this);
  v->setContentsMargins(0, 0, 0, 0);
  v->setSpacing(0);

  auto *bar = new QFrame;
  bar->setObjectName("dbToolbar");
  auto *bl = new QHBoxLayout(bar);
  bl->setContentsMargins(t.px("--space-4"), t.px("--space-2"),
                         t.px("--space-4"), t.px("--space-2"));
  bl->setSpacing(t.px("--space-3"));
  bl->addWidget(new CapsLabel("Panorama", "panelTitle"));
  caption_ = new QLabel;
  caption_->setObjectName("tableName");
  bl->addWidget(caption_);
  size_ = new QLabel;
  size_->setObjectName("toolbarMeta");
  bl->addWidget(size_);
  bl->addStretch(1);
  auto *hint = new QLabel("Hjul: zoom · træk: panorér · F: tilpas");
  hint->setObjectName("toolbarMeta");
  bl->addWidget(hint);
  v->addWidget(bar);

  auto *host = new QWidget;
  stack_ = new QStackedLayout(host);
  view_ = new PanoramaView;
  view_->setScene(new QGraphicsScene(view_));
  view_->setBackgroundBrush(t.color("--color-canvas"));
  item_ = view_->scene()->addPixmap(QPixmap());
  item_->setTransformationMode(Qt::SmoothTransformation);
  stack_->addWidget(view_);
  message_ = new QLabel;
  message_->setObjectName("viewportEmpty");
  message_->setAlignment(Qt::AlignCenter);
  message_->setWordWrap(true);
  stack_->addWidget(message_);
  v->addWidget(host, 1);

  connect(&theme(), &Theme::changed, this, [this] {
    view_->setBackgroundBrush(theme().color("--color-canvas"));
  });
  show_message("Vælg et panorama i træet.");
}

void PanoramaViewer::show_message(const QString &text) {
  message_->setText(text);
  stack_->setCurrentWidget(message_);
}

void PanoramaViewer::clear() {
  ++generation_;
  loading_ = false;
  item_->setPixmap(QPixmap());
  view_->scene()->setSceneRect(QRectF());
  caption_->clear();
  size_->clear();
  show_message("Vælg et panorama i træet.");
}

void PanoramaViewer::show_panorama(const QString &project, int id,
                                   const QString &caption) {
  const unsigned gen = ++generation_;
  loading_ = true;
  caption_->setText(caption);
  size_->clear();
  item_->setPixmap(QPixmap());
  show_message("Henter panorama …");
  const std::string path = project.toStdString();
  auto work = std::make_shared<BackgroundWork>();
  QPointer<PanoramaViewer> guard(this);
  std::thread([path, id, gen, guard, work]() mutable {
    QImage img;
    QSize stored;
    QString error;
    try {
      // Its own read-only connection: the session's belongs to the GUI
      // thread.
      reusex::ProjectDB db(path, /*readOnly=*/true);
      cv::Mat bgr = db.panoramic_image(id);
      if (bgr.empty())
        throw std::runtime_error("billedet er tomt eller kan ikke afkodes");
      if (bgr.depth() != CV_8U)
        throw std::runtime_error("ukendt pixelformat");
      stored = QSize(bgr.cols, bgr.rows);
      const ImageSize s = fit_within(bgr.cols, bgr.rows, kPanoramaMaxSide);
      if (s.width != bgr.cols || s.height != bgr.rows)
        cv::resize(bgr, bgr, cv::Size(s.width, s.height), 0, 0, cv::INTER_AREA);
      cv::Mat rgb;
      switch (bgr.channels()) {
      case 1:
        cv::cvtColor(bgr, rgb, cv::COLOR_GRAY2RGB);
        break;
      case 4:
        cv::cvtColor(bgr, rgb, cv::COLOR_BGRA2RGB);
        break;
      default:
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
      }
      img = QImage(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step),
                   QImage::Format_RGB888)
                .copy();
    } catch (const std::exception &e) {
      error = QString::fromUtf8(e.what());
    }
    QMetaObject::invokeMethod(
        qApp,
        [guard, gen, img, stored, error] {
          if (guard)
            guard->loaded(gen, img, stored, error);
        },
        Qt::QueuedConnection);
    work.reset(); // last: counted until the thread has let go of everything
  }).detach();
}

void PanoramaViewer::loaded(unsigned generation, const QImage &image,
                            QSize stored, const QString &error) {
  if (generation != generation_)
    return; // a newer panorama was asked for (or the viewer was cleared)
  loading_ = false;
  if (!error.isEmpty() || image.isNull()) {
    show_message(QString("Panoramaet kunne ikke læses.\n%1").arg(error));
    return;
  }
  size_->setText(
      QString("%1 × %2 px")
          .arg(format_count(static_cast<qulonglong>(stored.width())),
               format_count(static_cast<qulonglong>(stored.height()))));
  item_->setPixmap(QPixmap::fromImage(image));
  view_->scene()->setSceneRect(item_->boundingRect());
  stack_->setCurrentWidget(view_);
  view_->fit();
}

} // namespace rux::qt
