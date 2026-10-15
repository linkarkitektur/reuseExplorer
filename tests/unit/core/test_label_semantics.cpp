// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/label_semantics.hpp>

#include <opencv2/core.hpp>

#include <stdexcept>
#include <string>

using namespace reusex::core;

TEST_CASE("LabelSemanticsConstants_EncodingContract_MatchDefinedValues",
          "[label_semantics]") {
  REQUIRE(kUnlabeled == 0u);
  REQUIRE(kBackgroundApi == -1);
  REQUIRE(kMaxStorableLabel == 65534);
}

TEST_CASE("IsValidLabel_UnlabeledAndValidInputs_DistinguishesCorrectly",
          "[label_semantics]") {
  REQUIRE_FALSE(is_valid_label(0u));
  REQUIRE(is_valid_label(1u));
  REQUIRE(is_valid_label(65534u));
  REQUIRE(is_valid_label(65535u));
  REQUIRE(is_valid_label(4000000000u));
}

TEST_CASE("LabelToIndex_ValidAndUnlabeledInput_ConvertsOrThrows",
          "[label_semantics]") {
  REQUIRE(label_to_index(1u) == 0u);
  REQUIRE(label_to_index(2u) == 1u);
  REQUIRE(label_to_index(65534u) == 65533u);
  REQUIRE_THROWS_AS(label_to_index(0u), std::out_of_range);
}

TEST_CASE("ApiPointLabelConversion_VariousInputs_RoundTripsCorrectly",
          "[label_semantics]") {
  SECTION("background maps to unlabeled and back") {
    REQUIRE(api_to_point_label(kBackgroundApi) == kUnlabeled);
    REQUIRE(point_label_to_api(kUnlabeled) == kBackgroundApi);
  }

  SECTION("API class 0 is a real class, not unlabeled") {
    // STANDARDS §3.1: point label = API class + 1. API class 0 (the first
    // prompt class) used to map to 0 == kUnlabeled and was silently dropped.
    REQUIRE(api_to_point_label(0) == 1u);
    REQUIRE(is_valid_label(api_to_point_label(0)));
    REQUIRE(point_label_to_api(1u) == 0);
  }

  SECTION("valid class ids shift by one") {
    REQUIRE(api_to_point_label(5) == 6u);
    REQUIRE(point_label_to_api(6u) == 5);
  }

  SECTION("round-trip through point labels for valid API classes") {
    for (int a : {0, 1, 5, 65533, 65534})
      REQUIRE(point_label_to_api(api_to_point_label(a)) == a);
  }

  SECTION("label encoding marker") {
    REQUIRE(std::string(kLabelEncodingKey) == "label_encoding");
    REQUIRE(std::string(kLabelEncodingApiPlusOne) == "api_plus_one");
  }

  SECTION("negative non-sentinel throws") {
    REQUIRE_THROWS_AS(api_to_point_label(-2), std::out_of_range);
  }

  SECTION("out-of-contract point label throws instead of wrapping") {
    REQUIRE_THROWS_AS(point_label_to_api(0xFFFFFFFFu), std::out_of_range);
  }

  SECTION("round-trip through api for valid labels") {
    for (uint32_t p : {1u, 2u, 100u, 65534u, 65535u}) {
      REQUIRE(api_to_point_label(point_label_to_api(p)) == p);
    }
  }
}

TEST_CASE("ApiStorageConversion_BoundaryAndOverflowLabels_RoundTripsOrThrows",
          "[label_semantics]") {
  SECTION("background") {
    REQUIRE(api_to_storage(kBackgroundApi) == 0u);
    REQUIRE(storage_to_api(0u) == kBackgroundApi);
  }

  SECTION("boundary labels round-trip") {
    // API label L stores as L+1; background (-1) stores as 0.
    REQUIRE(api_to_storage(0) == 1u);
    REQUIRE(storage_to_api(1u) == 0);

    REQUIRE(api_to_storage(1) == 2u);
    REQUIRE(storage_to_api(2u) == 1);

    REQUIRE(api_to_storage(65534) == 65535u);
    REQUIRE(storage_to_api(65535u) == 65534);
  }

  SECTION("overflow throws instead of wrapping") {
    REQUIRE_THROWS_AS(api_to_storage(65535), std::out_of_range);
    REQUIRE_THROWS_AS(api_to_storage(100000), std::out_of_range);
  }

  SECTION("below-background throws") {
    REQUIRE_THROWS_AS(api_to_storage(-2), std::out_of_range);
  }
}

TEST_CASE("ApiStorageMatConversion_BoundaryLabels_RoundTripsCorrectly",
          "[label_semantics]") {
  // Build an API image (CV_32S) with background and boundary labels.
  cv::Mat api(2, 2, CV_32SC1);
  api.at<int>(0, 0) = kBackgroundApi; // -1 background
  api.at<int>(0, 1) = 0;              // class 0
  api.at<int>(1, 0) = 1;              // class 1
  api.at<int>(1, 1) = 65534;          // max storable

  cv::Mat storage = api_mat_to_storage(api);
  REQUIRE(storage.type() == CV_16UC1);
  REQUIRE(storage.at<uint16_t>(0, 0) == 0u);
  REQUIRE(storage.at<uint16_t>(0, 1) == 1u);
  REQUIRE(storage.at<uint16_t>(1, 0) == 2u);
  REQUIRE(storage.at<uint16_t>(1, 1) == 65535u);

  cv::Mat api_rt = storage_mat_to_api(storage);
  REQUIRE(api_rt.type() == CV_32SC1);
  REQUIRE(api_rt.at<int>(0, 0) == kBackgroundApi);
  REQUIRE(api_rt.at<int>(0, 1) == 0);
  REQUIRE(api_rt.at<int>(1, 0) == 1);
  REQUIRE(api_rt.at<int>(1, 1) == 65534);
}

TEST_CASE("ApiMatToStorage_OverflowingLabel_Throws", "[label_semantics]") {
  cv::Mat api(1, 2, CV_32SC1);
  api.at<int>(0, 0) = 1;
  api.at<int>(0, 1) = 65535; // one past the storable max -> must throw
  REQUIRE_THROWS_AS(api_mat_to_storage(api), std::out_of_range);
}

TEST_CASE("MatLabelConverters_EmptyOrWrongTypeInput_ThrowsOrReturnsEmpty",
          "[label_semantics]") {
  SECTION("empty storage returns empty") {
    REQUIRE(storage_mat_to_api(cv::Mat()).empty());
  }
  SECTION("empty api throws") {
    REQUIRE_THROWS_AS(api_mat_to_storage(cv::Mat()), std::invalid_argument);
  }
  SECTION("wrong storage type throws") {
    cv::Mat wrong(2, 2, CV_8UC1, cv::Scalar(0));
    REQUIRE_THROWS_AS(storage_mat_to_api(wrong), std::invalid_argument);
  }
  SECTION("wrong api type throws") {
    cv::Mat wrong(2, 2, CV_32FC1, cv::Scalar(0));
    REQUIRE_THROWS_AS(api_mat_to_storage(wrong), std::invalid_argument);
  }
}

// ── majority_label (reusex-z1v.2.2) ───────────────────────────────────────
// The one vote rule shared by `rux create project` (votes across frames) and
// `rux create clouds` (votes inside a voxel): kUnlabeled never votes, the
// highest count wins, an exact tie goes to the lowest label.

TEST_CASE("MajorityLabel_NoVotes_ReturnsUnlabeled", "[label_semantics]") {
  REQUIRE(majority_label({}) == kUnlabeled);
}

TEST_CASE("MajorityLabel_OnlyUnlabeledVotes_ReturnsUnlabeled",
          "[label_semantics]") {
  REQUIRE(majority_label({{kUnlabeled, 12}}) == kUnlabeled);
}

TEST_CASE("MajorityLabel_UnlabeledOutnumbersAClass_ClassWins",
          "[label_semantics]") {
  // Background never overwrites a class, however many frames saw background.
  REQUIRE(majority_label({{kUnlabeled, 9}, {4u, 1}}) == 4u);
}

TEST_CASE("MajorityLabel_TwoVersusOne_MajorityWins", "[label_semantics]") {
  REQUIRE(majority_label({{6u, 1}, {4u, 2}}) == 4u);
  REQUIRE(majority_label({{4u, 1}, {6u, 2}}) == 6u);
}

TEST_CASE("MajorityLabel_ExactTie_LowestLabelWinsInAnyOrder",
          "[label_semantics]") {
  REQUIRE(majority_label({{10u, 3}, {3u, 3}}) == 3u);
  REQUIRE(majority_label({{3u, 3}, {10u, 3}}) == 3u);
  REQUIRE(majority_label({{kUnlabeled, 3}, {10u, 3}, {7u, 3}}) == 7u);
}

TEST_CASE("MajorityLabel_RepeatedLabelEntries_AreSummed", "[label_semantics]") {
  // Callers may hand in one entry per vote rather than pre-aggregated counts.
  REQUIRE(majority_label({{5u, 1}, {2u, 1}, {5u, 1}}) == 5u);
}

TEST_CASE("MajorityLabel_NonPositiveCounts_DoNotVote", "[label_semantics]") {
  REQUIRE(majority_label({{5u, 0}, {2u, -3}}) == kUnlabeled);
}
