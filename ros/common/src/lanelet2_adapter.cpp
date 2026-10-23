// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "frenet_planner_ros/lanelet2_adapter.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <cmath>
#include <stdexcept>

namespace frenet_planner::ros_common {

struct Lanelet2Map::Impl {
  lanelet::LaneletMapPtr map;
  lanelet::traffic_rules::TrafficRulesPtr rules;
  lanelet::routing::RoutingGraphUPtr graph;
};

namespace {

template <typename LineString>
std::vector<Vec2> points(const LineString& ls) {
  std::vector<Vec2> out;
  out.reserve(ls.size());
  for (const auto& p : ls) out.push_back({p.x(), p.y()});
  return out;
}

LineType lineType(const lanelet::ConstLineString3d& ls) {
  const std::string type = ls.attributeOr(lanelet::AttributeName::Type, "");
  const std::string sub = ls.attributeOr(lanelet::AttributeName::Subtype, "");
  if (type == "road_border" || type == "curbstone" || type == "guard_rail" || type == "wall") {
    return LineType::kRoadEdge;
  }
  if (type == "virtual") return LineType::kVirtual;
  if (sub.find("dashed") != std::string::npos) return LineType::kDashed;
  if (sub.find("solid") != std::string::npos) return LineType::kSolid;
  return LineType::kUnknown;
}

int toId(lanelet::Id id) { return static_cast<int>(id); }

}  // namespace

Lanelet2Map::Lanelet2Map(const Lanelet2Options& o) : impl_(std::make_unique<Impl>()) {
  lanelet::ErrorMessages errors;
  const lanelet::Origin origin({o.origin_lat, o.origin_lon});
  if (o.projector == "local") {
    lanelet::projection::LocalCartesianProjector projector(origin);
    impl_->map = lanelet::load(o.path, projector, &errors);
  } else {
    lanelet::projection::UtmProjector projector(origin);
    impl_->map = lanelet::load(o.path, projector, &errors);
  }
  if (!impl_->map || impl_->map->laneletLayer.empty()) {
    throw std::runtime_error("cannot load Lanelet2 map '" + o.path + "'");
  }
  const std::string location = o.location == "de" ? lanelet::Locations::Germany : o.location;
  impl_->rules =
      lanelet::traffic_rules::TrafficRulesFactory::create(location, lanelet::Participants::Vehicle);
  impl_->graph = lanelet::routing::RoutingGraph::build(*impl_->map, *impl_->rules);

  for (const auto& ll : impl_->map->laneletLayer) {
    if (!impl_->rules->canPass(ll)) continue;  // not drivable for vehicles
    MapLane lane;
    lane.id = toId(ll.id());
    lane.centerline = points(ll.centerline2d());
    lane.left = points(ll.leftBound2d());
    lane.right = points(ll.rightBound2d());
    lane.left_type = lineType(ll.leftBound());
    lane.right_type = lineType(ll.rightBound());
    for (const auto& next : impl_->graph->following(ll, false)) lane.successors.push_back(toId(next.id()));
    for (const auto& prev : impl_->graph->previous(ll, false)) lane.predecessors.push_back(toId(prev.id()));
    // left()/right() are lane-changeable neighbours; adjacent*() are not.
    if (const auto left = impl_->graph->left(ll)) {
      lane.left_neighbor = toId(left->id());
      lane.can_change_left = true;
    } else if (const auto adj = impl_->graph->adjacentLeft(ll)) {
      lane.left_neighbor = toId(adj->id());
    }
    if (const auto right = impl_->graph->right(ll)) {
      lane.right_neighbor = toId(right->id());
      lane.can_change_right = true;
    } else if (const auto adj = impl_->graph->adjacentRight(ll)) {
      lane.right_neighbor = toId(adj->id());
    }
    lane.speed_limit = impl_->rules->speedLimit(ll).speedLimit.value();
    if (lane.centerline.size() < 2 || lane.left.size() < 2 || lane.right.size() < 2) continue;

    // Stop lines of traffic lights (inactive until a state message says otherwise).
    for (const auto& tl : ll.regulatoryElementsAs<lanelet::TrafficLight>()) {
      const auto stop = tl->stopLine();
      if (!stop || stop->size() < 2) continue;
      const double mx = 0.5 * (stop->front().x() + stop->back().x());
      const double my = 0.5 * (stop->front().y() + stop->back().y());
      const double s = projectOnPolyline(lane.centerline, mx, my).s;
      road_map_.addStopLine({toId(tl->id()), lane.id, s, false});
    }
    road_map_.addLane(std::move(lane));
  }
  road_map_.finalize();
}

Lanelet2Map::~Lanelet2Map() = default;

std::vector<int> Lanelet2Map::route(double from_x, double from_y, double to_x, double to_y) const {
  const auto nearest = [this](double x, double y) {
    return lanelet::geometry::findNearest(impl_->map->laneletLayer, lanelet::BasicPoint2d(x, y), 1);
  };
  const auto from = nearest(from_x, from_y);
  const auto to = nearest(to_x, to_y);
  if (from.empty() || to.empty()) return {};
  const auto path = impl_->graph->shortestPath(from.front().second, to.front().second);
  std::vector<int> ids;
  if (path) {
    for (const auto& ll : *path) ids.push_back(toId(ll.id()));
  }
  return ids;
}

}  // namespace frenet_planner::ros_common
