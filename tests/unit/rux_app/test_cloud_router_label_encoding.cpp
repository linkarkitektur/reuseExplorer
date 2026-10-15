// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux set clouds.labels` writes the default semantic cloud, so it stamps
// "label_encoding": "api_plus_one" (STANDARDS §3.1, reusex-z1v.2.1); any other
// label cloud keeps whatever encoding its author gave it.

#include <catch2/catch_test_macros.hpp>

#include <database/cloud_router.hpp>
#include <database/path_parser.hpp>
#include <database/resource_router.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/label_encoding.hpp>
#include <reusex/types/point_types.hpp>

#include "../../support/temp_path.hpp"

#include <memory>
#include <string>
#include <vector>

using namespace rux::database;

TEST_CASE("CloudRouterSet_SemanticLabelCloud_StampsLabelEncoding",
          "[rux_app][label_encoding]") {
  reusex::test_support::TempPath tmp("test_cloud_router_label_encoding");
  auto db = std::make_shared<reusex::ProjectDB>(tmp.path);
  reusex::CloudL c;
  c.resize(3);
  c[0].label = 0;
  c[1].label = 1;
  c[2].label = 2;
  db->save_point_cloud("labels", c);
  db->save_point_cloud("other", c);

  CloudRouter router(db);
  for (const char *name : {"labels", "other"}) {
    const auto pcd = router.get({PathComponent(ComponentType::Item, name)});
    router.set({PathComponent(ComponentType::Item, name)}, pcd);
  }

  CHECK(reusex::core::has_api_plus_one_encoding(
      db->point_cloud_provenance("labels").second));
  CHECK_FALSE(reusex::core::has_api_plus_one_encoding(
      db->point_cloud_provenance("other").second));
  CHECK(db->point_cloud_label("labels")->at(2).label == 2u);
}
