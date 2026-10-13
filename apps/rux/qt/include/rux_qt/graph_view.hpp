// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Qt-free geometry of the Rediger graph canvas (spec §5.2): how far the view
// may zoom, how a 3D pose lands on the chosen view plane after the view yaw,
// the heading tick, the "Auto" yaw, the yaw's text in project_settings, and
// which of A/B a node click fills. Standard library only (rux_qt_core).

#include <array>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace rux::qt {

// ----------------------------------------------------------------- zoom --

/// The closest zoom: one millimetre per screen pixel.
inline constexpr double kMinUnitsPerPx = 0.001;

struct ZoomLimits {
  double min_units_per_px = kMinUnitsPerPx;
  double max_units_per_px = 1.0;
};

/// Limits for a graph @p content_extent metres across shown in a viewport
/// @p viewport_px pixels across: in to kMinUnitsPerPx, out to four graphs
/// (at least 4 m) across the viewport.
ZoomLimits zoom_limits(double content_extent, int viewport_px);

/// The scale factor to apply for a requested @p factor (> 1 zooms in) at
/// @p units_per_px, clamped so the result stays within @p limits. A view
/// already past a limit (a very wide viewport fits the graph by its height)
/// is never pushed further out, and moves back as asked — no snap, and never
/// the opposite way to the wheel. 1 when either input is not positive.
double clamp_zoom_factor(double units_per_px, double factor,
                         const ZoomLimits &limits);

/// fit_avoiding_corner keeps at least this share of the plain fit's scale.
inline constexpr double kMinCornerFitRatio = 0.5;
/// A content rect in scene units: centre and size (scene y points down).
struct FitRect {
  double cx = 0.0;
  double cy = 0.0;
  double w = 0.0;
  double h = 0.0;
};
/// What fit() applies: pixels per scene unit and the scene point to centre.
struct FitFrame {
  double scale = 1.0;
  double cx = 0.0;
  double cy = 0.0;
};
/// Fit @p content into a @p vw x @p vh px viewport whose lower-left
/// @p rw x @p rh px corner is taken (the legend card): the content goes
/// above the corner or right of it, whichever keeps the larger scale (above
/// on a tie), centred in what is left. A corner that leaves no room, or
/// would cost more than half the plain fit's scale, is ignored. A viewport or
/// content without extent gives scale 1 at the content's centre.
FitFrame fit_avoiding_corner(const FitRect &content, double vw, double vh,
                             double rw, double rh);

// ----------------------------------------------------------- projection --

/// Which two axes the canvas shows: xy = plan, xz = seen from -y (x right,
/// z up), yz = seen from +x (y right, z up).
enum class ViewPlane { xy, xz, yz };
std::string_view view_plane_key(ViewPlane plane); ///< "xy" | "xz" | "yz"
std::optional<ViewPlane> parse_view_plane(std::string_view key);

struct ViewProjection {
  ViewPlane plane = ViewPlane::xy;
  /// Display rotation of the model about world z, degrees, counter-clockwise
  /// seen from above (project_settings view.yaw_deg).
  double yaw_deg = 0.0;
};

struct Vec2 {
  double x = 0.0;
  double y = 0.0; ///< up (the canvas negates it for the screen)
};

/// World point -> canvas coordinates: rotate by the yaw, then pick the plane.
Vec2 project_point(const ViewProjection &view, double x, double y, double z);

/// The camera's forward (optical +z) axis in world coordinates: the rotation
/// of @p world_pose (sensor_frames.transform) times that of
/// @p local_transform (the optical frame in the pose frame), applied to z.
std::array<double, 3>
camera_forward(const std::array<double, 16> &world_pose,
               const std::array<double, 16> &local_transform);

struct Heading {
  Vec2 dir;             ///< unit length when visible
  bool visible = false; ///< false when the camera looks along the view axis
};

/// Direction of @p forward on the canvas. Hidden when less than a quarter of
/// it lies in the view plane (it would be a meaningless stub).
Heading heading_in_view(const ViewProjection &view,
                        const std::array<double, 3> &forward);

/// The coordinate project_point drops (the one along the view axis), in
/// the yaw-rotated frame: z in plan, the rotated y for xz, x for yz.
double view_depth(const ViewProjection &view, double x, double y, double z);

/// Where the world point that canvas point @p p shows under @p view lands
/// once the yaw is @p new_yaw_deg — so a yaw change can keep the view's
/// centre. The point is lifted with @p depth on the dropped axis (see
/// view_depth; irrelevant in plan view).
Vec2 reproject_after_yaw(const ViewProjection &view, double new_yaw_deg, Vec2 p,
                         double depth = 0.0);

/// Median distance between consecutive points of the same scan (@p scans
/// per point, or empty for one scan); 0 with no such pair. The mean of the
/// middle two for an even count.
double median_step(const std::vector<Vec2> &points,
                   const std::vector<int> &scans);

/// Heading ticks only once consecutive nodes are at least @p tick_px apart
/// on screen (at a fit zoom of a dense graph they would be fur). True for a
/// step of 0 (one node, or all in one spot) or a degenerate zoom.
bool heading_ticks_visible(double median_step_units, double units_per_px,
                           double tick_px);

// ------------------------------------------------------------------ yaw --

/// The yaw (degrees, in (-45, 45]) that turns the dominant horizontal
/// direction of @p forwards onto the screen's x axis. Directions are folded
/// mod 90° (perpendicular corridors are one axis system) and weighted by how
/// horizontal they are; ones more than 60° off horizontal are ignored. 0
/// when nothing is horizontal.
double auto_yaw_deg(const std::vector<std::array<double, 3>> &forwards);

/// @p deg wrapped into (-180, 180]; 0 for a non-finite value.
double normalize_yaw_deg(double deg);

/// project_settings key of the view yaw (spec D7).
inline constexpr std::string_view kViewYawSetting = "view.yaw_deg";
/// Locale-independent parse of a stored yaw ('.' decimal), normalised;
/// nullopt for anything that is not exactly one finite number.
std::optional<double> parse_yaw_setting(std::string_view text);
/// The stored text: normalised, one decimal, '.' separator ("-12.5").
std::string format_yaw_setting(double deg);

// --------------------------------------------------------------- slots --

enum class Slot { a, b };

/// Which of the pair a node click fills (spec §5.2): A, then B, alternating;
/// Shift forces B. A frame is never both A and B.
class SlotPicker {
    public:
  Slot next() const { return next_; }
  int a() const { return a_; }
  int b() const { return b_; }
  bool complete() const { return a_ >= 0 && b_ >= 0; }
  /// A click on frame @p id: the slot it filled, or nullopt (no change) when
  /// @p id is already in the other slot.
  std::optional<Slot> click(int id, bool force_b);
  /// Both at once (an edge or a list row); the next click is A again.
  void set_pair(int a, int b);
  void clear();
  /// After a reload of the same project: keep the pair while every chosen
  /// frame is still among @p frame_ids, else clear it. True when kept.
  bool forget_missing(const std::vector<int> &frame_ids);

    private:
  int a_ = -1;
  int b_ = -1;
  Slot next_ = Slot::a;
};

} // namespace rux::qt
