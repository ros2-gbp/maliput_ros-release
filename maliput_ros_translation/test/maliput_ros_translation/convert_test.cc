// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include "maliput_ros_translation/convert.h"

#include <optional>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>
#include <maliput/test_utilities/mock.h>

namespace maliput_ros_translation {
namespace test {
namespace {

using ::testing::Return;
using ::testing::ReturnRef;

/// @brief Google mock maliput::api::RoadGeometry.
class RoadGeometryMock final : public maliput::api::RoadGeometry {
 public:
  MOCK_METHOD(maliput::api::RoadGeometryId, do_id, (), (const));
  MOCK_METHOD(int, do_num_junctions, (), (const));
  MOCK_METHOD(const maliput::api::Junction*, do_junction, (int), (const));
  MOCK_METHOD(int, do_num_branch_points, (), (const));
  MOCK_METHOD(const maliput::api::BranchPoint*, do_branch_point, (int), (const));
  MOCK_METHOD(const maliput::api::RoadGeometry::IdIndex&, DoById, (), (const));
  MOCK_METHOD(maliput::api::RoadPositionResult, DoToRoadPosition,
              (const maliput::api::InertialPosition&, const std::optional<maliput::api::RoadPosition>&), (const));
  MOCK_METHOD(std::vector<maliput::api::RoadPositionResult>, DoFindRoadPositions,
              (const maliput::api::InertialPosition&, double), (const));
  MOCK_METHOD(double, do_linear_tolerance, (), (const));
  MOCK_METHOD(double, do_angular_tolerance, (), (const));
  MOCK_METHOD(double, do_scale_length, (), (const));
  MOCK_METHOD(std::vector<maliput::api::InertialPosition>, DoSampleAheadWaypoints,
              (const maliput::api::LaneSRoute&, double), (const));
  MOCK_METHOD(maliput::math::Vector3, do_inertial_to_backend_frame_translation, (), (const));
};

/// @brief Google mock maliput::api::BranchPoint.
class BranchPointMock final : public maliput::api::BranchPoint {
 public:
  MOCK_METHOD(maliput::api::BranchPointId, do_id, (), (const));
  MOCK_METHOD(const maliput::api::RoadGeometry*, do_road_geometry, (), (const));
  MOCK_METHOD(const maliput::api::LaneEndSet*, DoGetConfluentBranches, (const maliput::api::LaneEnd& end), (const));
  MOCK_METHOD(const maliput::api::LaneEndSet*, DoGetOngoingBranches, (const maliput::api::LaneEnd& end), (const));
  MOCK_METHOD(std::optional<maliput::api::LaneEnd>, DoGetDefaultBranch, (const maliput::api::LaneEnd& end), (const));
  MOCK_METHOD(const maliput::api::LaneEndSet*, DoGetASide, (), (const));
  MOCK_METHOD(const maliput::api::LaneEndSet*, DoGetBSide, (), (const));
};

/// @brief Google mock maliput::api::Lane.
class LaneMock final : public maliput::api::Lane {
 public:
  MOCK_METHOD(maliput::api::LaneId, do_id, (), (const));
  MOCK_METHOD(int, do_index, (), (const));
  MOCK_METHOD(const maliput::api::Segment*, do_segment, (), (const));
  MOCK_METHOD(const maliput::api::Lane*, do_to_left, (), (const));
  MOCK_METHOD(const maliput::api::Lane*, do_to_right, (), (const));
  MOCK_METHOD(double, do_length, (), (const));
  MOCK_METHOD(const maliput::api::BranchPoint*, DoGetBranchPoint, (const maliput::api::LaneEnd::Which), (const));
  MOCK_METHOD(std::optional<maliput::api::LaneEnd>, DoGetDefaultBranch, (const maliput::api::LaneEnd::Which), (const));
  MOCK_METHOD(maliput::api::RBounds, do_lane_bounds, (double), (const));
  MOCK_METHOD(maliput::api::RBounds, do_segment_bounds, (double), (const));
  MOCK_METHOD(maliput::api::HBounds, do_elevation_bounds, (double, double), (const));
  MOCK_METHOD(maliput::api::InertialPosition, DoToInertialPosition, (const maliput::api::LanePosition&), (const));
  MOCK_METHOD(maliput::api::Rotation, DoGetOrientation, (const maliput::api::LanePosition&), (const));
  MOCK_METHOD(maliput::api::LanePosition, DoEvalMotionDerivatives,
              (const maliput::api::LanePosition&, const maliput::api::IsoLaneVelocity&), (const));
  MOCK_METHOD(maliput::api::LanePositionResult, DoToLanePosition, (const maliput::api::InertialPosition&), (const));
  MOCK_METHOD(maliput::api::LanePositionResult, DoToSegmentPosition, (const maliput::api::InertialPosition&), (const));
  MOCK_METHOD(const maliput::api::LaneEndSet*, DoGetConfluentBranches, (const maliput::api::LaneEnd::Which), (const));
  MOCK_METHOD(const maliput::api::LaneEndSet*, DoGetOngoingBranches, (const maliput::api::LaneEnd::Which), (const));
};

/// @brief Google mock maliput::api::LaneEndSet.
class LaneEndSetMock final : public maliput::api::LaneEndSet {
 public:
  MOCK_METHOD(int, do_size, (), (const));
  MOCK_METHOD(const maliput::api::LaneEnd&, do_get, (int), (const));
};

/// @brief Google mock maliput::api::SegmentMock.
class SegmentMock final : public maliput::api::Segment {
 public:
  MOCK_METHOD(maliput::api::SegmentId, do_id, (), (const));
  MOCK_METHOD(const maliput::api::Junction*, do_junction, (), (const));
  MOCK_METHOD(int, do_num_lanes, (), (const));
  MOCK_METHOD(const maliput::api::Lane*, do_lane, (int), (const));
};

/// @brief Google mock maliput::api::JunctionMock.
class JunctionMock final : public maliput::api::Junction {
 public:
  MOCK_METHOD(maliput::api::JunctionId, do_id, (), (const));
  MOCK_METHOD(const maliput::api::RoadGeometry*, do_road_geometry, (), (const));
  MOCK_METHOD(int, do_num_segments, (), (const));
  MOCK_METHOD(const maliput::api::Segment*, do_segment, (int), (const));
};

/// @brief Google mock maliput::api::RoadGeometry::IdIndex.
class IdIndexMock final : public maliput::api::RoadGeometry::IdIndex {
 public:
  MOCK_METHOD(const maliput::api::Lane*, DoGetLane, (const maliput::api::LaneId&), (const));
  MOCK_METHOD((const std::unordered_map<maliput::api::LaneId, const maliput::api::Lane*>&), DoGetLanes, (), (const));
  MOCK_METHOD(const maliput::api::Segment*, DoGetSegment, (const maliput::api::SegmentId&), (const));
  MOCK_METHOD(const maliput::api::Junction*, DoGetJunction, (const maliput::api::JunctionId&), (const));
  MOCK_METHOD(const maliput::api::BranchPoint*, DoGetBranchPoint, (const maliput::api::BranchPointId&), (const));
};

class IdsToRosMessageTest : public ::testing::Test {
 public:
  const maliput::api::BranchPointId kBranchPointId{"branch_point_id"};
  const maliput::api::LaneId kLaneId{"lane_id"};
  const maliput::api::JunctionId kJunctionId{"junction_id"};
  const maliput::api::RoadGeometryId kRoadGeometryId{"road_geometry_id"};
  const maliput::api::SegmentId kSegmentId{"segment_id"};
};

TEST_F(IdsToRosMessageTest, BranchPointId) {
  const maliput_ros_interfaces::msg::BranchPointId dut = ToRosMessage(kBranchPointId);
  ASSERT_EQ(dut.id, kBranchPointId.string());
}

TEST_F(IdsToRosMessageTest, LaneId) {
  const maliput_ros_interfaces::msg::LaneId dut = ToRosMessage(kLaneId);
  ASSERT_EQ(dut.id, kLaneId.string());
}

TEST_F(IdsToRosMessageTest, JunctionId) {
  const maliput_ros_interfaces::msg::JunctionId dut = ToRosMessage(kJunctionId);
  ASSERT_EQ(dut.id, kJunctionId.string());
}

TEST_F(IdsToRosMessageTest, RoadGeometryId) {
  const maliput_ros_interfaces::msg::RoadGeometryId dut = ToRosMessage(kRoadGeometryId);
  ASSERT_EQ(dut.id, kRoadGeometryId.string());
}

TEST_F(IdsToRosMessageTest, kSegmentId) {
  const maliput_ros_interfaces::msg::SegmentId dut = ToRosMessage(kSegmentId);
  ASSERT_EQ(dut.id, kSegmentId.string());
}

class LaneEndToRosMessageTest : public ::testing::Test {
 public:
  const maliput::api::LaneId kLaneId{"lane_id"};
  const std::string kNullPtrLaneId{""};
};

TEST_F(LaneEndToRosMessageTest, LaneEndAtStart) {
  LaneMock lane;
  EXPECT_CALL(lane, do_id()).WillRepeatedly(Return(kLaneId));
  const maliput::api::LaneEnd lane_end(&lane, maliput::api::LaneEnd::Which::kStart);

  const maliput_ros_interfaces::msg::LaneEnd dut = ToRosMessage(lane_end);

  ASSERT_EQ(dut.lane_id.id, kLaneId.string());
  ASSERT_EQ(dut.end, maliput_ros_interfaces::msg::LaneEnd::WHICHSTART);
}

TEST_F(LaneEndToRosMessageTest, LaneEndAtFinish) {
  LaneMock lane;
  EXPECT_CALL(lane, do_id()).WillRepeatedly(Return(kLaneId));
  const maliput::api::LaneEnd lane_end(&lane, maliput::api::LaneEnd::Which::kFinish);

  const maliput_ros_interfaces::msg::LaneEnd dut = ToRosMessage(lane_end);

  ASSERT_EQ(dut.lane_id.id, kLaneId.string());
  ASSERT_EQ(dut.end, maliput_ros_interfaces::msg::LaneEnd::WHICHFINISH);
}

TEST_F(LaneEndToRosMessageTest, NullLane) {
  const maliput_ros_interfaces::msg::LaneEnd dut = ToRosMessage(maliput::api::LaneEnd());

  ASSERT_EQ(dut.lane_id.id, kNullPtrLaneId);
}

GTEST_TEST(LaneEndSetToRosMessageTest, ValidLaneEndSet) {
  static constexpr int kSize{2};
  const maliput::api::LaneId kLaneIdA{"lane_id_a"};
  const maliput::api::LaneId kLaneIdB{"lane_id_b"};
  const maliput::api::LaneEnd::Which kWhichA{maliput::api::LaneEnd::Which::kStart};
  const maliput::api::LaneEnd::Which kWhichB{maliput::api::LaneEnd::Which::kFinish};
  LaneMock lane_a;
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  LaneMock lane_b;
  EXPECT_CALL(lane_b, do_id()).WillRepeatedly(Return(kLaneIdB));
  const maliput::api::LaneEnd lane_end_a(&lane_a, kWhichA);
  const maliput::api::LaneEnd lane_end_b(&lane_b, kWhichB);
  LaneEndSetMock lane_end_set;
  EXPECT_CALL(lane_end_set, do_size()).WillRepeatedly(Return(kSize));
  EXPECT_CALL(lane_end_set, do_get(0)).WillRepeatedly(ReturnRef(lane_end_a));
  EXPECT_CALL(lane_end_set, do_get(1)).WillRepeatedly(ReturnRef(lane_end_b));

  const maliput_ros_interfaces::msg::LaneEndSet dut = ToRosMessage(&lane_end_set);

  ASSERT_EQ(dut.lane_ends.size(), static_cast<size_t>(kSize));
  ASSERT_EQ(dut.lane_ends[0].lane_id.id, kLaneIdA.string());
  ASSERT_EQ(dut.lane_ends[0].end, maliput_ros_interfaces::msg::LaneEnd::WHICHSTART);
  ASSERT_EQ(dut.lane_ends[1].lane_id.id, kLaneIdB.string());
  ASSERT_EQ(dut.lane_ends[1].end, maliput_ros_interfaces::msg::LaneEnd::WHICHFINISH);
}

GTEST_TEST(LaneEndSetToRosMessageTest, NullptrLaneEndSet) {
  const maliput_ros_interfaces::msg::LaneEndSet dut =
      ToRosMessage(static_cast<const maliput::api::LaneEndSet*>(nullptr));

  ASSERT_EQ(dut.lane_ends.size(), static_cast<size_t>(0));
}

class BranchPointToRosMessageTest : public ::testing::Test {
 public:
  const maliput::api::LaneId kLaneIdA{"lane_id_a"};
  const maliput::api::LaneId kLaneIdB{"lane_id_b"};
  const maliput::api::LaneEnd::Which kWhichA{maliput::api::LaneEnd::Which::kStart};
  const maliput::api::LaneEnd::Which kWhichB{maliput::api::LaneEnd::Which::kFinish};
  const maliput::api::RoadGeometryId kRoadGeometryId{"kRoadGeometryId"};
  const maliput::api::BranchPointId kBranchPointId{"branch_point_id"};
  const std::string kNullBranchPointId{""};
};

TEST_F(BranchPointToRosMessageTest, ValidBranchPoint) {
  LaneMock lane_a;
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  LaneMock lane_b;
  EXPECT_CALL(lane_b, do_id()).WillRepeatedly(Return(kLaneIdB));
  const maliput::api::LaneEnd lane_end_a(&lane_a, kWhichA);
  const maliput::api::LaneEnd lane_end_b(&lane_b, kWhichB);
  LaneEndSetMock lane_end_set_a;
  EXPECT_CALL(lane_end_set_a, do_size()).WillRepeatedly(Return(1));
  EXPECT_CALL(lane_end_set_a, do_get(0)).WillRepeatedly(ReturnRef(lane_end_a));
  LaneEndSetMock lane_end_set_b;
  EXPECT_CALL(lane_end_set_b, do_size()).WillRepeatedly(Return(1));
  EXPECT_CALL(lane_end_set_b, do_get(0)).WillRepeatedly(ReturnRef(lane_end_b));
  RoadGeometryMock road_geometry;
  EXPECT_CALL(road_geometry, do_id()).WillRepeatedly(Return(kRoadGeometryId));
  BranchPointMock branch_point;
  EXPECT_CALL(branch_point, do_id()).WillRepeatedly(Return(kBranchPointId));
  EXPECT_CALL(branch_point, do_road_geometry()).WillRepeatedly(Return(&road_geometry));
  EXPECT_CALL(branch_point, DoGetASide()).WillRepeatedly(Return(&lane_end_set_a));
  EXPECT_CALL(branch_point, DoGetBSide()).WillRepeatedly(Return(&lane_end_set_b));

  const maliput_ros_interfaces::msg::BranchPoint dut = ToRosMessage(&branch_point);

  ASSERT_EQ(dut.id.id, kBranchPointId.string());
  ASSERT_EQ(dut.road_geometry_id.id, kRoadGeometryId.string());
  ASSERT_EQ(dut.a_side.lane_ends.size(), static_cast<size_t>(1));
  ASSERT_EQ(dut.a_side.lane_ends[0].lane_id.id, kLaneIdA.string());
  ASSERT_EQ(dut.a_side.lane_ends[0].end, maliput_ros_interfaces::msg::LaneEnd::WHICHSTART);
  ASSERT_EQ(dut.b_side.lane_ends.size(), static_cast<size_t>(1));
  ASSERT_EQ(dut.b_side.lane_ends[0].lane_id.id, kLaneIdB.string());
  ASSERT_EQ(dut.b_side.lane_ends[0].end, maliput_ros_interfaces::msg::LaneEnd::WHICHFINISH);
}

TEST_F(BranchPointToRosMessageTest, BranchPointWithEmptyBSide) {
  LaneMock lane_a;
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  const maliput::api::LaneEnd lane_end_a(&lane_a, kWhichA);
  LaneEndSetMock lane_end_set_a;
  EXPECT_CALL(lane_end_set_a, do_size()).WillRepeatedly(Return(1));
  EXPECT_CALL(lane_end_set_a, do_get(0)).WillRepeatedly(ReturnRef(lane_end_a));
  RoadGeometryMock road_geometry;
  EXPECT_CALL(road_geometry, do_id()).WillRepeatedly(Return(kRoadGeometryId));
  BranchPointMock branch_point;
  EXPECT_CALL(branch_point, do_id()).WillRepeatedly(Return(kBranchPointId));
  EXPECT_CALL(branch_point, do_road_geometry()).WillRepeatedly(Return(&road_geometry));
  EXPECT_CALL(branch_point, DoGetASide()).WillRepeatedly(Return(&lane_end_set_a));
  EXPECT_CALL(branch_point, DoGetBSide()).WillRepeatedly(Return(static_cast<const maliput::api::LaneEndSet*>(nullptr)));

  const maliput_ros_interfaces::msg::BranchPoint dut = ToRosMessage(&branch_point);

  ASSERT_EQ(dut.id.id, kBranchPointId.string());
  ASSERT_EQ(dut.road_geometry_id.id, kRoadGeometryId.string());
  ASSERT_EQ(dut.a_side.lane_ends.size(), static_cast<size_t>(1));
  ASSERT_EQ(dut.a_side.lane_ends[0].lane_id.id, kLaneIdA.string());
  ASSERT_EQ(dut.a_side.lane_ends[0].end, maliput_ros_interfaces::msg::LaneEnd::WHICHSTART);
  ASSERT_EQ(dut.b_side.lane_ends.size(), static_cast<size_t>(0));
}

TEST_F(BranchPointToRosMessageTest, NullBranchPoint) {
  const maliput_ros_interfaces::msg::BranchPoint dut =
      ToRosMessage(static_cast<const maliput::api::BranchPoint*>(nullptr));

  ASSERT_EQ(dut.id.id, kNullBranchPointId);
}

GTEST_TEST(SegmentToRosMessageTest, ValidSegment) {
  static constexpr int kSize{2};
  const maliput::api::LaneId kLaneIdA{"lane_id_a"};
  const maliput::api::LaneId kLaneIdB{"lane_id_b"};
  const maliput::api::JunctionId kJunctionId{"junction_id"};
  const maliput::api::SegmentId kSegmentId{"segment_id"};
  LaneMock lane_a;
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  LaneMock lane_b;
  EXPECT_CALL(lane_b, do_id()).WillRepeatedly(Return(kLaneIdB));
  JunctionMock junction;
  EXPECT_CALL(junction, do_id()).WillRepeatedly(Return(kJunctionId));
  SegmentMock segment;
  EXPECT_CALL(segment, do_id()).WillRepeatedly(Return(kSegmentId));
  EXPECT_CALL(segment, do_junction()).WillRepeatedly(Return(&junction));
  EXPECT_CALL(segment, do_num_lanes()).WillRepeatedly(Return(kSize));
  EXPECT_CALL(segment, do_lane(0)).WillRepeatedly(Return(&lane_a));
  EXPECT_CALL(segment, do_lane(1)).WillRepeatedly(Return(&lane_b));

  const maliput_ros_interfaces::msg::Segment dut = ToRosMessage(&segment);

  ASSERT_EQ(dut.id.id, kSegmentId.string());
  ASSERT_EQ(dut.junction_id.id, kJunctionId.string());
  ASSERT_EQ(dut.lane_ids.size(), static_cast<size_t>(kSize));
  ASSERT_EQ(dut.lane_ids[0].id, kLaneIdA.string());
  ASSERT_EQ(dut.lane_ids[1].id, kLaneIdB.string());
}

GTEST_TEST(SegmentToRosMessageTest, NullSegment) {
  const std::string kNullSegmentId{""};

  const maliput_ros_interfaces::msg::Segment dut = ToRosMessage(static_cast<const maliput::api::Segment*>(nullptr));

  ASSERT_EQ(dut.id.id, kNullSegmentId);
}

GTEST_TEST(JunctionToRosMessageTest, ValidJunction) {
  static constexpr int kSize{2};
  const maliput::api::SegmentId kSgmentIdA{"segment_id_a"};
  const maliput::api::SegmentId kSgmentIdB{"segment_id_b"};
  const maliput::api::RoadGeometryId kRoadGeometryId{"road_geometry_id"};
  const maliput::api::JunctionId kJunctionId{"junction_id"};
  SegmentMock segment_a;
  EXPECT_CALL(segment_a, do_id()).WillRepeatedly(Return(kSgmentIdA));
  SegmentMock segment_b;
  EXPECT_CALL(segment_b, do_id()).WillRepeatedly(Return(kSgmentIdB));
  RoadGeometryMock road_geometry;
  EXPECT_CALL(road_geometry, do_id()).WillRepeatedly(Return(kRoadGeometryId));
  JunctionMock junction;
  EXPECT_CALL(junction, do_id()).WillRepeatedly(Return(kJunctionId));
  EXPECT_CALL(junction, do_road_geometry()).WillRepeatedly(Return(&road_geometry));
  EXPECT_CALL(junction, do_num_segments()).WillRepeatedly(Return(kSize));
  EXPECT_CALL(junction, do_segment(0)).WillRepeatedly(Return(&segment_a));
  EXPECT_CALL(junction, do_segment(1)).WillRepeatedly(Return(&segment_b));

  const maliput_ros_interfaces::msg::Junction dut = ToRosMessage(&junction);

  ASSERT_EQ(dut.id.id, kJunctionId.string());
  ASSERT_EQ(dut.road_geometry_id.id, kRoadGeometryId.string());
  ASSERT_EQ(dut.segment_ids.size(), static_cast<size_t>(kSize));
  ASSERT_EQ(dut.segment_ids[0].id, kSgmentIdA.string());
  ASSERT_EQ(dut.segment_ids[1].id, kSgmentIdB.string());
}

GTEST_TEST(JunctionToRosMessageTest, NullJunction) {
  const std::string kNullJunctionId{""};

  const maliput_ros_interfaces::msg::Junction dut = ToRosMessage(static_cast<const maliput::api::Junction*>(nullptr));

  ASSERT_EQ(dut.id.id, kNullJunctionId);
}

GTEST_TEST(RoadGeometryToRosMessageTest, ValidRoadGeometry) {
  static constexpr int kSizeJunctions{2};
  static constexpr int kSizeBranchPoints{2};
  static constexpr double kLinearTolerance{1e-12};
  static constexpr double kAngularTolerance{5e-12};
  static constexpr double kScaleLength{1.};
  const maliput::api::JunctionId kJunctionIdA{"junction_id_a"};
  const maliput::api::JunctionId kJunctionIdB{"junction_id_b"};
  const maliput::api::BranchPointId kBranchPointIdA{"branch_point_id_a"};
  const maliput::api::BranchPointId kBranchPointIdB{"branch_point_id_b"};
  const maliput::math::Vector3 kInertialToBackendFrameTranslation{1., 2., 3.};
  const maliput::api::RoadGeometryId kRoadGeometryId{"road_geometry_id"};
  JunctionMock junction_a;
  EXPECT_CALL(junction_a, do_id()).WillRepeatedly(Return(kJunctionIdA));
  JunctionMock junction_b;
  EXPECT_CALL(junction_b, do_id()).WillRepeatedly(Return(kJunctionIdB));
  BranchPointMock branch_point_a;
  EXPECT_CALL(branch_point_a, do_id()).WillRepeatedly(Return(kBranchPointIdA));
  BranchPointMock branch_point_b;
  EXPECT_CALL(branch_point_b, do_id()).WillRepeatedly(Return(kBranchPointIdB));
  RoadGeometryMock road_geometry;
  EXPECT_CALL(road_geometry, do_id()).WillRepeatedly(Return(kRoadGeometryId));
  EXPECT_CALL(road_geometry, do_linear_tolerance()).WillRepeatedly(Return(kLinearTolerance));
  EXPECT_CALL(road_geometry, do_angular_tolerance()).WillRepeatedly(Return(kAngularTolerance));
  EXPECT_CALL(road_geometry, do_scale_length()).WillRepeatedly(Return(kScaleLength));
  EXPECT_CALL(road_geometry, do_inertial_to_backend_frame_translation())
      .WillRepeatedly(Return(kInertialToBackendFrameTranslation));
  EXPECT_CALL(road_geometry, do_num_junctions()).WillRepeatedly(Return(kSizeJunctions));
  EXPECT_CALL(road_geometry, do_num_branch_points()).WillRepeatedly(Return(kSizeBranchPoints));
  EXPECT_CALL(road_geometry, do_junction(0)).WillRepeatedly(Return(&junction_a));
  EXPECT_CALL(road_geometry, do_junction(1)).WillRepeatedly(Return(&junction_b));
  EXPECT_CALL(road_geometry, do_branch_point(0)).WillRepeatedly(Return(&branch_point_a));
  EXPECT_CALL(road_geometry, do_branch_point(1)).WillRepeatedly(Return(&branch_point_b));

  const maliput_ros_interfaces::msg::RoadGeometry dut = ToRosMessage(&road_geometry);

  ASSERT_EQ(dut.id.id, kRoadGeometryId.string());
  ASSERT_EQ(dut.linear_tolerance, kLinearTolerance);
  ASSERT_EQ(dut.angular_tolerance, kAngularTolerance);
  ASSERT_EQ(dut.scale_length, kScaleLength);
  ASSERT_EQ(dut.inertial_to_backend_frame_translation.x, kInertialToBackendFrameTranslation.x());
  ASSERT_EQ(dut.inertial_to_backend_frame_translation.y, kInertialToBackendFrameTranslation.y());
  ASSERT_EQ(dut.inertial_to_backend_frame_translation.z, kInertialToBackendFrameTranslation.z());
  ASSERT_EQ(dut.junction_ids.size(), static_cast<size_t>(kSizeJunctions));
  ASSERT_EQ(dut.junction_ids[0].id, kJunctionIdA.string());
  ASSERT_EQ(dut.junction_ids[1].id, kJunctionIdB.string());
  ASSERT_EQ(dut.branch_point_ids.size(), static_cast<size_t>(kSizeJunctions));
  ASSERT_EQ(dut.branch_point_ids[0].id, kBranchPointIdA.string());
  ASSERT_EQ(dut.branch_point_ids[1].id, kBranchPointIdB.string());
}

GTEST_TEST(RoadGeometryToRosMessageTest, NullRoadGeometry) {
  const std::string kNullRoadGeometryId{""};

  const maliput_ros_interfaces::msg::RoadGeometry dut =
      ToRosMessage(static_cast<const maliput::api::RoadGeometry*>(nullptr));

  ASSERT_EQ(dut.id.id, kNullRoadGeometryId);
}

class LaneToRosMessageTest : public ::testing::Test {
 public:
  static constexpr int kIndex{3};
  static constexpr double kLaneLength{123.456};
  const maliput::api::BranchPointId kStartBranchPointId{"start_branch_point_id"};
  const maliput::api::BranchPointId kFinishBranchPointId{"finish_branch_point_id"};
  const maliput::api::SegmentId kSegmentId{"segment_id"};
  const maliput::api::LaneId kLeftLaneId{"left_lane_id"};
  const maliput::api::LaneId kRightLaneId{"right_lane_id"};
  const maliput::api::LaneId kDefaultStartLaneId{"default_start_lane_id"};
  const maliput::api::LaneEnd::Which kDefaultStartWhich{maliput::api::LaneEnd::Which::kStart};
  const maliput::api::LaneId kDefaultFinishLaneId{"default_finish_lane_id"};
  const maliput::api::LaneEnd::Which kDefaultFinishWhich{maliput::api::LaneEnd::Which::kFinish};
  const maliput::api::LaneId kLaneId{"lane_id"};
  const std::string kNullLaneId{""};

  void SetUp() override {
    EXPECT_CALL(segment, do_id()).WillRepeatedly(Return(kSegmentId));
    EXPECT_CALL(left_lane, do_id()).WillRepeatedly(Return(kLeftLaneId));
    EXPECT_CALL(right_lane, do_id()).WillRepeatedly(Return(kRightLaneId));
    EXPECT_CALL(default_start_lane, do_id()).WillRepeatedly(Return(kDefaultStartLaneId));
    EXPECT_CALL(default_finish_lane, do_id()).WillRepeatedly(Return(kDefaultFinishLaneId));
    EXPECT_CALL(start_branch_point, do_id()).WillRepeatedly(Return(kStartBranchPointId));
    EXPECT_CALL(finish_branch_point, do_id()).WillRepeatedly(Return(kFinishBranchPointId));
    EXPECT_CALL(lane, do_id()).WillRepeatedly(Return(kLaneId));
    EXPECT_CALL(lane, do_segment()).WillRepeatedly(Return(&segment));
    EXPECT_CALL(lane, do_index()).WillRepeatedly(Return(kIndex));
    EXPECT_CALL(lane, do_to_left()).WillRepeatedly(Return(&left_lane));
    EXPECT_CALL(lane, do_to_right()).WillRepeatedly(Return(&right_lane));
    EXPECT_CALL(lane, do_length()).WillRepeatedly(Return(kLaneLength));
    EXPECT_CALL(lane, DoGetBranchPoint(maliput::api::LaneEnd::Which::kStart))
        .WillRepeatedly(Return(&start_branch_point));
    EXPECT_CALL(lane, DoGetBranchPoint(maliput::api::LaneEnd::Which::kFinish))
        .WillRepeatedly(Return(&finish_branch_point));
  }

  SegmentMock segment;
  LaneMock left_lane;
  LaneMock right_lane;
  LaneMock default_start_lane;
  LaneMock default_finish_lane;
  BranchPointMock start_branch_point;
  BranchPointMock finish_branch_point;
  LaneMock lane;
};

TEST_F(LaneToRosMessageTest, FullLane) {
  const std::optional<maliput::api::LaneEnd> default_branch_start =
      maliput::api::LaneEnd{&default_start_lane, kDefaultStartWhich};
  const std::optional<maliput::api::LaneEnd> default_branch_finish =
      maliput::api::LaneEnd{&default_finish_lane, kDefaultFinishWhich};
  EXPECT_CALL(lane, DoGetDefaultBranch(maliput::api::LaneEnd::Which::kStart))
      .WillRepeatedly(Return(default_branch_start));
  EXPECT_CALL(lane, DoGetDefaultBranch(maliput::api::LaneEnd::Which::kFinish))
      .WillRepeatedly(Return(default_branch_finish));

  const maliput_ros_interfaces::msg::Lane dut = ToRosMessage(&lane);

  ASSERT_EQ(dut.id.id, kLaneId.string());
  ASSERT_EQ(dut.segment_id.id, kSegmentId.string());
  ASSERT_EQ(dut.index, kIndex);
  ASSERT_EQ(dut.left_lane.id, kLeftLaneId.string());
  ASSERT_EQ(dut.right_lane.id, kRightLaneId.string());
  ASSERT_EQ(dut.length, kLaneLength);
  ASSERT_EQ(dut.start_branch_point_id.id, kStartBranchPointId.string());
  ASSERT_EQ(dut.finish_branch_point_id.id, kFinishBranchPointId.string());
  ASSERT_EQ(dut.default_start_branch.lane_id.id, kDefaultStartLaneId.string());
  ASSERT_EQ(dut.default_start_branch.end, maliput_ros_interfaces::msg::LaneEnd::WHICHSTART);
  ASSERT_EQ(dut.default_finish_branch.lane_id.id, kDefaultFinishLaneId.string());
  ASSERT_EQ(dut.default_finish_branch.end, maliput_ros_interfaces::msg::LaneEnd::WHICHFINISH);
}

TEST_F(LaneToRosMessageTest, WithoutDefaultStartBranch) {
  const std::optional<maliput::api::LaneEnd> default_branch_start{};
  const std::optional<maliput::api::LaneEnd> default_branch_finish =
      maliput::api::LaneEnd{&default_finish_lane, kDefaultFinishWhich};
  EXPECT_CALL(lane, DoGetDefaultBranch(maliput::api::LaneEnd::Which::kStart))
      .WillRepeatedly(Return(default_branch_start));
  EXPECT_CALL(lane, DoGetDefaultBranch(maliput::api::LaneEnd::Which::kFinish))
      .WillRepeatedly(Return(default_branch_finish));

  const maliput_ros_interfaces::msg::Lane dut = ToRosMessage(&lane);

  ASSERT_EQ(dut.id.id, kLaneId.string());
  ASSERT_EQ(dut.segment_id.id, kSegmentId.string());
  ASSERT_EQ(dut.index, kIndex);
  ASSERT_EQ(dut.left_lane.id, kLeftLaneId.string());
  ASSERT_EQ(dut.right_lane.id, kRightLaneId.string());
  ASSERT_EQ(dut.length, kLaneLength);
  ASSERT_EQ(dut.start_branch_point_id.id, kStartBranchPointId.string());
  ASSERT_EQ(dut.finish_branch_point_id.id, kFinishBranchPointId.string());
  ASSERT_EQ(dut.default_start_branch.lane_id.id, kNullLaneId);
  ASSERT_EQ(dut.default_finish_branch.lane_id.id, kDefaultFinishLaneId.string());
  ASSERT_EQ(dut.default_finish_branch.end, maliput_ros_interfaces::msg::LaneEnd::WHICHFINISH);
}

TEST_F(LaneToRosMessageTest, WithoutDefaultFinishBranch) {
  const std::optional<maliput::api::LaneEnd> default_branch_start =
      maliput::api::LaneEnd{&default_start_lane, kDefaultStartWhich};
  const std::optional<maliput::api::LaneEnd> default_branch_finish{};
  EXPECT_CALL(lane, DoGetDefaultBranch(maliput::api::LaneEnd::Which::kStart))
      .WillRepeatedly(Return(default_branch_start));
  EXPECT_CALL(lane, DoGetDefaultBranch(maliput::api::LaneEnd::Which::kFinish))
      .WillRepeatedly(Return(default_branch_finish));

  const maliput_ros_interfaces::msg::Lane dut = ToRosMessage(&lane);

  ASSERT_EQ(dut.id.id, kLaneId.string());
  ASSERT_EQ(dut.segment_id.id, kSegmentId.string());
  ASSERT_EQ(dut.index, kIndex);
  ASSERT_EQ(dut.left_lane.id, kLeftLaneId.string());
  ASSERT_EQ(dut.right_lane.id, kRightLaneId.string());
  ASSERT_EQ(dut.length, kLaneLength);
  ASSERT_EQ(dut.start_branch_point_id.id, kStartBranchPointId.string());
  ASSERT_EQ(dut.finish_branch_point_id.id, kFinishBranchPointId.string());
  ASSERT_EQ(dut.default_start_branch.lane_id.id, kDefaultStartLaneId.string());
  ASSERT_EQ(dut.default_start_branch.end, maliput_ros_interfaces::msg::LaneEnd::WHICHSTART);
  ASSERT_EQ(dut.default_finish_branch.lane_id.id, kNullLaneId);
}

TEST_F(LaneToRosMessageTest, NullLane) {
  const maliput_ros_interfaces::msg::Lane dut = ToRosMessage(static_cast<const maliput::api::Lane*>(nullptr));

  ASSERT_EQ(dut.id.id, kNullLaneId);
}

GTEST_TEST(BranchPointIdFromRosMessage, ValidateConversion) {
  const std::string kBranchPointId{"branch_point_id"};
  maliput_ros_interfaces::msg::BranchPointId msg;
  msg.id = kBranchPointId;

  const maliput::api::BranchPointId dut = FromRosMessage(msg);

  ASSERT_EQ(dut.string(), kBranchPointId);
}

GTEST_TEST(JunctionIdFromRosMessage, ValidateConversion) {
  const std::string kJunctionId{"junction_id"};
  maliput_ros_interfaces::msg::JunctionId msg;
  msg.id = kJunctionId;

  const maliput::api::JunctionId dut = FromRosMessage(msg);

  ASSERT_EQ(dut.string(), kJunctionId);
}

GTEST_TEST(LaneIdFromRosMessage, ValidateConversion) {
  const std::string kLaneId{"lane_id"};
  maliput_ros_interfaces::msg::LaneId msg;
  msg.id = kLaneId;

  const maliput::api::LaneId dut = FromRosMessage(msg);

  ASSERT_EQ(dut.string(), kLaneId);
}

GTEST_TEST(RoadGeometryIdFromRosMessage, ValidateConversion) {
  const std::string kRoadGeometryId{"road_geometry_id"};
  maliput_ros_interfaces::msg::RoadGeometryId msg;
  msg.id = kRoadGeometryId;

  const maliput::api::RoadGeometryId dut = FromRosMessage(msg);

  ASSERT_EQ(dut.string(), kRoadGeometryId);
}

GTEST_TEST(SegmentIdFromRosMessage, ValidateConversion) {
  const std::string kSegmentId{"segment_id"};
  maliput_ros_interfaces::msg::SegmentId msg;
  msg.id = kSegmentId;

  const maliput::api::SegmentId dut = FromRosMessage(msg);

  ASSERT_EQ(dut.string(), kSegmentId);
}

GTEST_TEST(InertialPositionToRosMessage, ValidateConversion) {
  const maliput::api::InertialPosition kPosition{1., 2., 3.};

  const maliput_ros_interfaces::msg::InertialPosition dut = ToRosMessage(kPosition);

  ASSERT_EQ(dut.x, kPosition.x());
  ASSERT_EQ(dut.y, kPosition.y());
  ASSERT_EQ(dut.z, kPosition.z());
}

GTEST_TEST(LanePositionToRosMessage, ValidateConversion) {
  const maliput::api::LanePosition kPosition{1., 2., 3.};

  const maliput_ros_interfaces::msg::LanePosition dut = ToRosMessage(kPosition);

  ASSERT_EQ(dut.s, kPosition.s());
  ASSERT_EQ(dut.r, kPosition.r());
  ASSERT_EQ(dut.h, kPosition.h());
}

GTEST_TEST(RoadPositionToRosMessage, ValidateConversion) {
  const maliput::api::LaneId kLaneId{"lane_id"};
  LaneMock lane;
  EXPECT_CALL(lane, do_id()).WillRepeatedly(Return(kLaneId));
  const maliput::api::LanePosition kPosition{1., 2., 3.};
  const maliput::api::RoadPosition kRoadPosition{&lane, kPosition};

  const maliput_ros_interfaces::msg::RoadPosition dut = ToRosMessage(kRoadPosition);

  ASSERT_EQ(dut.lane_id.id, kLaneId.string());
  ASSERT_EQ(dut.pos.s, kPosition.s());
  ASSERT_EQ(dut.pos.r, kPosition.r());
  ASSERT_EQ(dut.pos.h, kPosition.h());
}

GTEST_TEST(RoadPositionToRosMessage, ValidateConversionWhenLaneIsNullptr) {
  const maliput::api::RoadPosition kRoadPosition{};

  const maliput_ros_interfaces::msg::RoadPosition dut = ToRosMessage(kRoadPosition);

  ASSERT_TRUE(dut.lane_id.id.empty());
}

GTEST_TEST(RoadPositionResultToRosMessage, ValidateConversion) {
  const maliput::api::LaneId kLaneId{"lane_id"};
  LaneMock lane;
  EXPECT_CALL(lane, do_id()).WillRepeatedly(Return(kLaneId));
  const maliput::api::LanePosition kLanePosition{1., 2., 3.};
  const maliput::api::RoadPosition kRoadPosition{&lane, kLanePosition};
  const maliput::api::InertialPosition kInertialPosition{4., 5., 6.};
  constexpr const double kDistance{7.};
  const maliput::api::RoadPositionResult kPosition{kRoadPosition, kInertialPosition, kDistance};

  const maliput_ros_interfaces::msg::RoadPositionResult dut = ToRosMessage(kPosition);

  ASSERT_EQ(dut.road_position.lane_id.id, kLaneId.string());
  ASSERT_EQ(dut.road_position.pos.s, kLanePosition.s());
  ASSERT_EQ(dut.road_position.pos.r, kLanePosition.r());
  ASSERT_EQ(dut.road_position.pos.h, kLanePosition.h());
  ASSERT_EQ(dut.nearest_position.x, kInertialPosition.x());
  ASSERT_EQ(dut.nearest_position.y, kInertialPosition.y());
  ASSERT_EQ(dut.nearest_position.z, kInertialPosition.z());
  ASSERT_EQ(dut.distance, kDistance);
}

GTEST_TEST(InertialPositionFromRosMessage, ValidateConversion) {
  maliput_ros_interfaces::msg::InertialPosition position;
  position.x = 1.;
  position.y = 2.;
  position.z = 3.;

  const maliput::api::InertialPosition dut = FromRosMessage(position);

  ASSERT_EQ(dut.x(), position.x);
  ASSERT_EQ(dut.y(), position.y);
  ASSERT_EQ(dut.z(), position.z);
}

GTEST_TEST(LanePositionFromRosMessage, ValidateConversion) {
  maliput_ros_interfaces::msg::LanePosition position;
  position.s = 1.;
  position.r = 2.;
  position.h = 3.;

  const maliput::api::LanePosition dut = FromRosMessage(position);

  ASSERT_EQ(dut.s(), position.s);
  ASSERT_EQ(dut.r(), position.r);
  ASSERT_EQ(dut.h(), position.h);
}

GTEST_TEST(RoadPositionFromRosMessage, ValidateConversion) {
  const maliput::api::LanePosition kLanePosition(1., 2., 3.);
  const maliput::api::LaneId kLaneId{"lane_id"};
  LaneMock lane;
  EXPECT_CALL(lane, do_id()).WillRepeatedly(Return(kLaneId));
  IdIndexMock id_index;
  EXPECT_CALL(id_index, DoGetLane(kLaneId)).WillRepeatedly(Return(&lane));
  RoadGeometryMock road_geometry;
  EXPECT_CALL(road_geometry, DoById()).WillRepeatedly(ReturnRef(id_index));
  maliput_ros_interfaces::msg::RoadPosition road_position;
  road_position.lane_id = ToRosMessage(kLaneId);
  road_position.pos = ToRosMessage(kLanePosition);

  const maliput::api::RoadPosition dut = FromRosMessage(&road_geometry, road_position);

  ASSERT_EQ(dut.lane, &lane);
  ASSERT_EQ(dut.pos.s(), kLanePosition.s());
  ASSERT_EQ(dut.pos.r(), kLanePosition.r());
  ASSERT_EQ(dut.pos.h(), kLanePosition.h());
}

GTEST_TEST(RoadPositionFromRosMessage, PassingNullRoadGeometryThrows) {
  maliput_ros_interfaces::msg::RoadPosition road_position;

  ASSERT_THROW({ FromRosMessage(nullptr, road_position); }, maliput::common::assertion_error);
}

GTEST_TEST(RoadPositionFromRosMessage, EmptyLaneIdYieldsDefaultConstructedRoadPosition) {
  RoadGeometryMock road_geometry;
  maliput_ros_interfaces::msg::RoadPosition road_position;

  const maliput::api::RoadPosition dut = FromRosMessage(&road_geometry, road_position);

  ASSERT_EQ(dut.lane, nullptr);
  ASSERT_EQ(dut.pos.s(), 0.);
  ASSERT_EQ(dut.pos.r(), 0.);
  ASSERT_EQ(dut.pos.h(), 0.);
}

GTEST_TEST(QuaternionToRosMessage, ValidateConversion) {
  // A quaternion that rotates x->y, y->z, z->x...
  const maliput::math::Quaternion kQuaternion =
      maliput::math::Quaternion(M_PI * 2. / 3., maliput::math::Vector3(1.0, 1.0, 1.0).normalized());

  const maliput_ros_interfaces::msg::Rotation dut = ToRosMessage(maliput::api::Rotation::FromQuat(kQuaternion));

  ASSERT_EQ(dut.x, kQuaternion.x());
  ASSERT_EQ(dut.y, kQuaternion.y());
  ASSERT_EQ(dut.z, kQuaternion.z());
  ASSERT_EQ(dut.w, kQuaternion.w());
}

GTEST_TEST(IsoLaneVelocityToRosMessage, ValidateConversion) {
  const maliput::api::IsoLaneVelocity kVelocity{1., 2., 3.};

  const maliput_ros_interfaces::msg::IsoLaneVelocity dut = ToRosMessage(kVelocity);

  ASSERT_EQ(dut.sigma_v, kVelocity.sigma_v);
  ASSERT_EQ(dut.rho_v, kVelocity.rho_v);
  ASSERT_EQ(dut.eta_v, kVelocity.eta_v);
}

GTEST_TEST(IsoLaneVelocityFromRosMessage, ValidateConversion) {
  maliput_ros_interfaces::msg::IsoLaneVelocity velocity;
  velocity.sigma_v = 1.;
  velocity.rho_v = 2.;
  velocity.eta_v = 3.;

  const maliput::api::IsoLaneVelocity dut = FromRosMessage(velocity);

  ASSERT_EQ(dut.sigma_v, velocity.sigma_v);
  ASSERT_EQ(dut.rho_v, velocity.rho_v);
  ASSERT_EQ(dut.eta_v, velocity.eta_v);
}

GTEST_TEST(HBoundsToRosMessage, ValidateConversion) {
  const maliput::api::HBounds kHBounds{-1., 2.};

  const maliput_ros_interfaces::msg::HBounds dut = ToRosMessage(kHBounds);

  ASSERT_EQ(dut.min, kHBounds.min());
  ASSERT_EQ(dut.max, kHBounds.max());
}

GTEST_TEST(HBoundsFromRosMessage, ValidateConversion) {
  maliput_ros_interfaces::msg::HBounds h_bounds;
  h_bounds.min = -1.;
  h_bounds.max = 2.;

  const maliput::api::HBounds dut = FromRosMessage(h_bounds);

  ASSERT_EQ(dut.min(), h_bounds.min);
  ASSERT_EQ(dut.max(), h_bounds.max);
}

GTEST_TEST(RBoundsToRosMessage, ValidateConversion) {
  const maliput::api::RBounds kRBounds{-1., 2.};

  const maliput_ros_interfaces::msg::RBounds dut = ToRosMessage(kRBounds);

  ASSERT_EQ(dut.min, kRBounds.min());
  ASSERT_EQ(dut.max, kRBounds.max());
}

GTEST_TEST(RBoundsFromRosMessage, ValidateConversion) {
  maliput_ros_interfaces::msg::RBounds r_bounds;
  r_bounds.min = -1.;
  r_bounds.max = 2.;

  const maliput::api::RBounds dut = FromRosMessage(r_bounds);

  ASSERT_EQ(dut.min(), r_bounds.min);
  ASSERT_EQ(dut.max(), r_bounds.max);
}

GTEST_TEST(SRangeToRosMessage, ValidateConversion) {
  const maliput::api::SRange kSRange{1., 2.};

  const maliput_ros_interfaces::msg::SRange dut = ToRosMessage(kSRange);

  ASSERT_EQ(dut.s0, kSRange.s0());
  ASSERT_EQ(dut.s1, kSRange.s1());
  ASSERT_EQ(dut.size, kSRange.size());
  ASSERT_EQ(dut.with_s, kSRange.WithS());
}

GTEST_TEST(SRangeFromRosMessage, ValidateConversion) {
  maliput_ros_interfaces::msg::SRange s_range;
  s_range.s0 = 1.;
  s_range.s1 = 2.;
  s_range.size = 1.;
  s_range.with_s = true;

  const maliput::api::SRange dut = FromRosMessage(s_range);

  ASSERT_EQ(dut.s0(), s_range.s0);
  ASSERT_EQ(dut.s1(), s_range.s1);
  ASSERT_EQ(dut.size(), s_range.size);
  ASSERT_EQ(dut.WithS(), s_range.with_s);
}

GTEST_TEST(LaneSRangeToRosMessage, ValidateConversion) {
  const maliput::api::SRange kSRange{1., 2.};
  const maliput::api::LaneId kLaneId{"lane_id"};
  const maliput::api::LaneSRange kLaneSRange{kLaneId, kSRange};

  const maliput_ros_interfaces::msg::LaneSRange dut = ToRosMessage(kLaneSRange);

  ASSERT_EQ(dut.lane_id.id, kLaneId.string());
  ASSERT_EQ(dut.s_range.s0, kSRange.s0());
  ASSERT_EQ(dut.s_range.s1, kSRange.s1());
  ASSERT_EQ(dut.s_range.size, kSRange.size());
  ASSERT_EQ(dut.s_range.with_s, kSRange.WithS());
}

GTEST_TEST(LaneSRangeFromRosMessage, ValidateConversion) {
  maliput_ros_interfaces::msg::SRange s_range;
  s_range.s0 = 1.;
  s_range.s1 = 2.;
  s_range.size = 1.;
  s_range.with_s = true;
  maliput_ros_interfaces::msg::LaneSRange lane_s_range;
  lane_s_range.lane_id.id = "lane_id";
  lane_s_range.s_range = s_range;

  const maliput::api::LaneSRange dut = FromRosMessage(lane_s_range);

  ASSERT_EQ(dut.lane_id().string(), "lane_id");
  ASSERT_EQ(dut.s_range().s0(), s_range.s0);
  ASSERT_EQ(dut.s_range().s1(), s_range.s1);
  ASSERT_EQ(dut.s_range().size(), s_range.size);
  ASSERT_EQ(dut.s_range().WithS(), s_range.with_s);
}

GTEST_TEST(LaneSRouteToRosMessage, ValidateConversion) {
  const maliput::api::SRange kSRange{1., 2.};
  const maliput::api::LaneId kLaneId{"lane_id"};
  const maliput::api::LaneSRange kLaneSRange{kLaneId, kSRange};
  const maliput::api::LaneSRoute kLaneSRoute{{kLaneSRange}};

  const maliput_ros_interfaces::msg::LaneSRoute dut = ToRosMessage(kLaneSRoute);

  ASSERT_EQ(dut.ranges.size(), 1u);
  ASSERT_EQ(dut.ranges[0].lane_id.id, kLaneId.string());
  ASSERT_EQ(dut.ranges[0].s_range.s0, kSRange.s0());
  ASSERT_EQ(dut.ranges[0].s_range.s1, kSRange.s1());
  ASSERT_EQ(dut.ranges[0].s_range.size, kSRange.size());
  ASSERT_EQ(dut.ranges[0].s_range.with_s, kSRange.WithS());
}

GTEST_TEST(LaneSRouteFromRosMessage, ValidateConversion) {
  maliput_ros_interfaces::msg::SRange s_range;
  s_range.s0 = 1.;
  s_range.s1 = 2.;
  s_range.size = 1.;
  s_range.with_s = true;
  maliput_ros_interfaces::msg::LaneSRange lane_s_range;
  lane_s_range.lane_id.id = "lane_id";
  lane_s_range.s_range = s_range;
  maliput_ros_interfaces::msg::LaneSRoute lane_s_route;
  lane_s_route.ranges.push_back(lane_s_range);

  const maliput::api::LaneSRoute dut = FromRosMessage(lane_s_route);

  ASSERT_EQ(dut.ranges().size(), 1u);
  ASSERT_EQ(dut.ranges()[0].lane_id().string(), "lane_id");
  ASSERT_EQ(dut.ranges()[0].s_range().s0(), s_range.s0);
  ASSERT_EQ(dut.ranges()[0].s_range().s1(), s_range.s1);
  ASSERT_EQ(dut.ranges()[0].s_range().size(), s_range.size);
  ASSERT_EQ(dut.ranges()[0].s_range().WithS(), s_range.with_s);
}

}  // namespace
}  // namespace test
}  // namespace maliput_ros_translation
