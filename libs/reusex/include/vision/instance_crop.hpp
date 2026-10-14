// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The pinhole bbox projection now lives in core (core/frame_projection.hpp)
// so sync_survey and apply_mask_selection can use it; vision keeps its names.

#include "reusex/core/frame_projection.hpp"

namespace reusex::vision {
using core::project_points_to_box;
using core::ProjectedBox;
} // namespace reusex::vision
