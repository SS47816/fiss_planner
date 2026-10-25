// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
//
// Python bindings of the fiss_planner core (nanobind).
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/vector.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>

#include "fiss_planner/collision/box2d.hpp"
#include "fiss_planner/common/types.hpp"
#include "fiss_planner/control/controller.hpp"
#include "fiss_planner/geometry/reference_line.hpp"
#include "fiss_planner/map/road_map.hpp"
#include "fiss_planner/map/route_corridor.hpp"
#include "fiss_planner/planning/local_planner.hpp"
#include "fiss_planner/planning/planner.hpp"
#include "fiss_planner/prediction/lane_following.hpp"
#include "fiss_planner/smoothing/piecewise_jerk.hpp"
#include "fiss_planner/smoothing/smoother.hpp"

namespace nb = nanobind;
using namespace nb::literals;
using namespace fiss_planner;

namespace {

using Array1 = nb::ndarray<nb::numpy, double, nb::ndim<1>>;
using Array2 = nb::ndarray<nb::numpy, double, nb::shape<-1, 2>>;
using InPoints = nb::ndarray<const double, nb::shape<-1, 2>, nb::c_contig, nb::device::cpu>;

// Arrays are returned as owned Python objects so that property getters (which
// default to the reference_internal policy) do not try to re-own them.
nb::object toNumpy(const std::vector<double>& v) {
  auto* data = new double[v.size()];
  std::copy(v.begin(), v.end(), data);
  nb::capsule owner(data, [](void* p) noexcept { delete[] static_cast<double*>(p); });
  return nb::cast(Array1(data, {v.size()}, owner), nb::rv_policy::move);
}

nb::object pointsToNumpy(const std::vector<Vec2>& pts) {
  auto* data = new double[2 * pts.size()];
  for (size_t i = 0; i < pts.size(); ++i) {
    data[2 * i] = pts[i].x;
    data[2 * i + 1] = pts[i].y;
  }
  nb::capsule owner(data, [](void* p) noexcept { delete[] static_cast<double*>(p); });
  return nb::cast(Array2(data, {pts.size(), 2}, owner), nb::rv_policy::move);
}

std::vector<Vec2> pointsFromNumpy(const InPoints& a) {
  std::vector<Vec2> out(a.shape(0));
  const double* d = a.data();
  for (size_t i = 0; i < out.size(); ++i) out[i] = {d[2 * i], d[2 * i + 1]};
  return out;
}

std::optional<Vec2> toVec2(const std::optional<std::pair<double, double>>& p) {
  if (!p) return std::nullopt;
  return Vec2{p->first, p->second};
}

const char* rejectionName(Rejection r) {
  switch (r) {
    case Rejection::kOk:
      return "ok";
    case Rejection::kInvalid:
      return "invalid";
    case Rejection::kSpeed:
      return "speed";
    case Rejection::kAccel:
      return "accel";
    case Rejection::kCurvature:
      return "curvature";
    case Rejection::kLatAccel:
      return "lat_accel";
    case Rejection::kOffRoad:
      return "off_road";
    case Rejection::kCollision:
      return "collision";
    case Rejection::kSafetyDistance:
      return "safety_distance";
    case Rejection::kCount:
      break;
  }
  return "unknown";
}

#define TRAJ_ARRAY(name) def_prop_ro(#name, [](const Trajectory& t) { return toNumpy(t.name); })

}  // namespace

NB_MODULE(_core, m) {
  m.doc() = "Sampling-based Frenet trajectory planners (FOP, FOP+, FISS, FISS+)";

  // --- basic types ---------------------------------------------------------
  nb::class_<EgoState>(m, "EgoState")
      .def(nb::init<>())
      .def(
          "__init__",
          [](EgoState* s, double x, double y, double yaw, double v, double a, double kappa) {
            new (s) EgoState{x, y, yaw, v, a, kappa};
          },
          "x"_a, "y"_a, "yaw"_a, "v"_a = 0.0, "a"_a = 0.0, "kappa"_a = 0.0)
      .def_rw("x", &EgoState::x)
      .def_rw("y", &EgoState::y)
      .def_rw("yaw", &EgoState::yaw)
      .def_rw("v", &EgoState::v)
      .def_rw("a", &EgoState::a)
      .def_rw("kappa", &EgoState::kappa)
      .def("__repr__", [](const EgoState& s) {
        std::ostringstream os;
        os << "EgoState(x=" << s.x << ", y=" << s.y << ", yaw=" << s.yaw << ", v=" << s.v << ", a=" << s.a
           << ", kappa=" << s.kappa << ")";
        return os.str();
      });

  nb::class_<FrenetState>(m, "FrenetState")
      .def(nb::init<>())
      .def(
          "__init__",
          [](FrenetState* f, double s, double s_d, double s_dd, double d, double d_d, double d_dd) {
            new (f) FrenetState{s, s_d, s_dd, d, d_d, d_dd};
          },
          "s"_a, "s_d"_a = 0.0, "s_dd"_a = 0.0, "d"_a = 0.0, "d_d"_a = 0.0, "d_dd"_a = 0.0)
      .def_rw("s", &FrenetState::s)
      .def_rw("s_d", &FrenetState::s_d)
      .def_rw("s_dd", &FrenetState::s_dd)
      .def_rw("d", &FrenetState::d)
      .def_rw("d_d", &FrenetState::d_d)
      .def_rw("d_dd", &FrenetState::d_dd)
      .def("__repr__", [](const FrenetState& f) {
        std::ostringstream os;
        os << "FrenetState(s=" << f.s << ", s_d=" << f.s_d << ", s_dd=" << f.s_dd << ", d=" << f.d
           << ", d_d=" << f.d_d << ", d_dd=" << f.d_dd << ")";
        return os.str();
      });

  nb::class_<EndState>(m, "EndState")
      .def(nb::init<>())
      .def_rw("d", &EndState::d)
      .def_rw("s_d", &EndState::s_d)
      .def_rw("T", &EndState::T);

  nb::class_<VehicleParams>(m, "VehicleParams")
      .def(nb::init<>())
      .def_rw("length", &VehicleParams::length)
      .def_rw("width", &VehicleParams::width)
      .def_rw("wheelbase", &VehicleParams::wheelbase)
      .def_rw("rear_axle_to_center", &VehicleParams::rear_axle_to_center)
      .def_rw("max_speed", &VehicleParams::max_speed)
      .def_rw("max_accel", &VehicleParams::max_accel)
      .def_rw("max_decel", &VehicleParams::max_decel)
      .def_rw("max_steer", &VehicleParams::max_steer)
      .def_rw("max_steer_rate", &VehicleParams::max_steer_rate)
      .def_rw("max_lat_accel", &VehicleParams::max_lat_accel)
      .def("max_curvature", &VehicleParams::maxCurvature);

  // --- geometry ------------------------------------------------------------
  nb::class_<RefPoint>(m, "RefPoint")
      .def_ro("s", &RefPoint::s)
      .def_ro("x", &RefPoint::x)
      .def_ro("y", &RefPoint::y)
      .def_ro("yaw", &RefPoint::yaw)
      .def_ro("kappa", &RefPoint::kappa)
      .def_ro("dkappa", &RefPoint::dkappa);

  nb::class_<Projection>(m, "Projection").def_ro("s", &Projection::s).def_ro("d", &Projection::d);

  nb::class_<ReferenceLine>(m, "ReferenceLine")
      .def(nb::init<>())
      .def(nb::init<const std::vector<double>&, const std::vector<double>&, double>(), "x"_a, "y"_a,
           "resample_ds"_a = 1.0)
      .def_prop_ro("length", &ReferenceLine::length)
      .def("empty", &ReferenceLine::empty)
      .def("at", &ReferenceLine::at, "s"_a)
      .def("project", &ReferenceLine::project, "x"_a, "y"_a, "s_hint"_a = NAN)
      .def("to_frenet", &ReferenceLine::toFrenet, "state"_a, "s_hint"_a = NAN)
      .def("to_cartesian", &ReferenceLine::toCartesian, "frenet"_a)
      .def(
          "sample",
          [](const ReferenceLine& r, double ds) {
            const auto pts = r.sample(ds);
            auto* data = new double[pts.size() * 5];
            for (size_t i = 0; i < pts.size(); ++i) {
              data[5 * i] = pts[i].s;
              data[5 * i + 1] = pts[i].x;
              data[5 * i + 2] = pts[i].y;
              data[5 * i + 3] = pts[i].yaw;
              data[5 * i + 4] = pts[i].kappa;
            }
            nb::capsule owner(data, [](void* p) noexcept { delete[] static_cast<double*>(p); });
            return nb::ndarray<nb::numpy, double, nb::shape<-1, 5>>(data, {pts.size(), 5}, owner);
          },
          "ds"_a = 1.0, "Sample the line as an (N, 5) array of [s, x, y, yaw, kappa].");
  m.def(
      "smooth_polyline",
      [](std::vector<double> x, std::vector<double> y, double length, double tolerance, double ds) {
        smoothPolyline(x, y, length, tolerance, ds);
        return std::make_pair(x, y);
      },
      "x"_a, "y"_a, "length"_a, "tolerance"_a, "ds"_a = 1.0,
      "Resample a polyline every `ds` and smooth it within `tolerance`; returns (x, y).");

  nb::class_<Box2d>(m, "Box2d")
      .def(nb::init<>())
      .def(
          "__init__",
          [](Box2d* b, double cx, double cy, double yaw, double length, double width) {
            new (b) Box2d{cx, cy, yaw, length, width};
          },
          "cx"_a, "cy"_a, "yaw"_a, "length"_a, "width"_a)
      .def_rw("cx", &Box2d::cx)
      .def_rw("cy", &Box2d::cy)
      .def_rw("yaw", &Box2d::yaw)
      .def_rw("length", &Box2d::length)
      .def_rw("width", &Box2d::width)
      .def("corners", [](const Box2d& b) {
        const auto c = b.corners();
        return pointsToNumpy(std::vector<Vec2>(c.begin(), c.end()));
      });
  m.def("overlaps", &overlaps, "a"_a, "b"_a);

  // --- obstacles -----------------------------------------------------------
  nb::class_<ObstacleState>(m, "ObstacleState")
      .def(nb::init<>())
      .def(
          "__init__",
          [](ObstacleState* o, double t, double x, double y, double yaw, double v) {
            new (o) ObstacleState{t, x, y, yaw, v};
          },
          "t"_a, "x"_a, "y"_a, "yaw"_a, "v"_a = 0.0)
      .def_rw("t", &ObstacleState::t)
      .def_rw("x", &ObstacleState::x)
      .def_rw("y", &ObstacleState::y)
      .def_rw("yaw", &ObstacleState::yaw)
      .def_rw("v", &ObstacleState::v);

  nb::class_<Obstacle>(m, "Obstacle")
      .def(nb::init<>())
      .def_rw("id", &Obstacle::id)
      .def_rw("length", &Obstacle::length)
      .def_rw("width", &Obstacle::width)
      .def_rw("is_static", &Obstacle::is_static)
      .def_rw("prediction", &Obstacle::prediction)
      .def("box_at", &Obstacle::boxAt, "t"_a)
      .def_static(
          "from_arrays",
          [](int id, double length, double width, const std::vector<double>& t, const std::vector<double>& x,
             const std::vector<double>& y, const std::vector<double>& yaw, bool is_static) {
            if (t.size() != x.size() || x.size() != y.size() || y.size() != yaw.size()) {
              throw std::invalid_argument("Obstacle.from_arrays: arrays must have equal length");
            }
            Obstacle ob;
            ob.id = id;
            ob.length = length;
            ob.width = width;
            ob.is_static = is_static;
            ob.prediction.reserve(t.size());
            for (size_t i = 0; i < t.size(); ++i) ob.prediction.push_back({t[i], x[i], y[i], yaw[i], 0.0});
            return ob;
          },
          "id"_a, "length"_a, "width"_a, "t"_a, "x"_a, "y"_a, "yaw"_a, "is_static"_a = false);
  m.def("predict_constant_velocity", &predictConstantVelocity, "id"_a, "x"_a, "y"_a, "yaw"_a, "v"_a,
        "length"_a, "width"_a, "horizon"_a, "dt"_a);

  // --- corridor ------------------------------------------------------------
  nb::class_<LaneBand>(m, "LaneBand")
      .def(nb::init<>())
      .def_rw("offset", &LaneBand::offset)
      .def_rw("lane_id", &LaneBand::lane_id)
      .def_rw("change_allowed", &LaneBand::change_allowed)
      .def_rw("extra_cost", &LaneBand::extra_cost)
      .def_rw("urgency", &LaneBand::urgency)
      .def_rw("speed_limit", &LaneBand::speed_limit)
      .def_rw("center", &LaneBand::center)
      .def_rw("left", &LaneBand::left)
      .def_rw("right", &LaneBand::right);

  nb::class_<Corridor>(m, "Corridor")
      .def(nb::init<>())
      .def(nb::init<std::vector<double>>(), "stations"_a)
      .def_static("uniform", &Corridor::uniform, "s_start"_a, "s_end"_a, "d_right"_a, "d_left"_a,
                  "ds"_a = 1.0)
      .def_static("parallel_lanes", &Corridor::parallelLanes, "s_start"_a, "s_end"_a, "lane_width"_a,
                  "num_left"_a, "num_right"_a, "ds"_a = 1.0)
      .def("add_lane", &Corridor::addLane, "lane"_a)
      .def_prop_ro("stations", [](const Corridor& c) { return toNumpy(c.stations()); })
      .def_prop_ro("lanes", [](const Corridor& c) { return c.lanes(); })
      .def("set_change_allowed",
           [](Corridor& c, int offset, bool allowed) {
             for (auto& l : c.lanes())
               if (l.offset == offset) l.change_allowed = allowed;
           })
      .def("road_bounds", &Corridor::roadBounds, "s"_a)
      .def("empty", &Corridor::empty);

  // --- configuration -------------------------------------------------------
  nb::enum_<PlannerType>(m, "PlannerType")
      .value("FOP", PlannerType::kFop)
      .value("FOP_PLUS", PlannerType::kFopPlus)
      .value("FISS", PlannerType::kFiss)
      .value("FISS_PLUS", PlannerType::kFissPlus);
  m.def("planner_type_from_string", &plannerTypeFromString, "name"_a);
  m.def("planner_type_to_string", [](PlannerType t) { return toString(t); }, "type"_a);

  nb::class_<SamplingConfig>(m, "SamplingConfig")
      .def(nb::init<>())
      .def_rw("num_width", &SamplingConfig::num_width)
      .def_rw("num_speed", &SamplingConfig::num_speed)
      .def_rw("num_t", &SamplingConfig::num_t)
      .def_rw("min_t", &SamplingConfig::min_t)
      .def_rw("max_t", &SamplingConfig::max_t)
      .def_rw("dt", &SamplingConfig::dt)
      .def_rw("min_speed", &SamplingConfig::min_speed)
      .def_rw("lane_margin", &SamplingConfig::lane_margin);
  nb::class_<CostConfig>(m, "CostConfig")
      .def(nb::init<>())
      .def_rw("w_time", &CostConfig::w_time)
      .def_rw("w_speed", &CostConfig::w_speed)
      .def_rw("w_accel", &CostConfig::w_accel)
      .def_rw("w_jerk", &CostConfig::w_jerk)
      .def_rw("w_lateral", &CostConfig::w_lateral)
      .def_rw("w_lane_change", &CostConfig::w_lane_change)
      .def_rw("w_heuristic", &CostConfig::w_heuristic)
      .def_rw("w_headway", &CostConfig::w_headway)
      .def_rw("desired_gap", &CostConfig::desired_gap)
      .def_rw("desired_time_headway", &CostConfig::desired_time_headway)
      .def_rw("w_ttc", &CostConfig::w_ttc)
      .def_rw("desired_ttc", &CostConfig::desired_ttc)
      .def_rw("w_clearance", &CostConfig::w_clearance)
      .def_rw("desired_clearance", &CostConfig::desired_clearance)
      .def_rw("follow_time", &CostConfig::follow_time);
  nb::class_<ConstraintConfig>(m, "ConstraintConfig")
      .def(nb::init<>())
      .def_rw("check_curvature", &ConstraintConfig::check_curvature)
      .def_rw("check_lat_accel", &ConstraintConfig::check_lat_accel)
      .def_rw("check_road_bounds", &ConstraintConfig::check_road_bounds)
      .def_rw("curvature_check_min_speed", &ConstraintConfig::curvature_check_min_speed);
  nb::class_<CollisionConfig>(m, "CollisionConfig")
      .def(nb::init<>())
      .def_rw("margin_lon", &CollisionConfig::margin_lon)
      .def_rw("margin_lat", &CollisionConfig::margin_lat)
      .def_rw("stride", &CollisionConfig::stride)
      .def_rw("ignore_rear_in_lane", &CollisionConfig::ignore_rear_in_lane);
  nb::class_<SafetyConfig>(m, "SafetyConfig")
      .def(nb::init<>())
      .def_rw("enabled", &SafetyConfig::enabled)
      .def_rw("min_gap", &SafetyConfig::min_gap)
      .def_rw("min_time_headway", &SafetyConfig::min_time_headway)
      .def_rw("min_ttc", &SafetyConfig::min_ttc)
      .def_rw("reaction_time", &SafetyConfig::reaction_time)
      .def_rw("crossing_time_gap", &SafetyConfig::crossing_time_gap);
  nb::class_<FissPlusConfig>(m, "FissPlusConfig")
      .def(nb::init<>())
      .def_rw("refine", &FissPlusConfig::refine)
      .def_rw("max_refine_iters", &FissPlusConfig::max_refine_iters)
      .def_rw("decay", &FissPlusConfig::decay)
      .def_rw("time_budget", &FissPlusConfig::time_budget);
  nb::class_<PlannerConfig>(m, "PlannerConfig")
      .def(nb::init<>())
      .def_rw("type", &PlannerConfig::type)
      .def_rw("sampling", &PlannerConfig::sampling)
      .def_rw("cost", &PlannerConfig::cost)
      .def_rw("constraints", &PlannerConfig::constraints)
      .def_rw("collision", &PlannerConfig::collision)
      .def_rw("safety", &PlannerConfig::safety)
      .def_rw("fiss_plus", &PlannerConfig::fiss_plus)
      .def_rw("num_threads", &PlannerConfig::num_threads)
      .def_rw("keep_candidates", &PlannerConfig::keep_candidates)
      .def("validate", &PlannerConfig::validate);
  m.def(
      "load_planner_config",
      [](const std::string& path) {
        VehicleParams v;
        auto c = loadPlannerConfig(path, &v);
        return std::make_pair(c, v);
      },
      "path"_a, "Load (PlannerConfig, VehicleParams) from a YAML file.");
  m.def(
      "parse_planner_config",
      [](const std::string& text) {
        VehicleParams v;
        auto c = parsePlannerConfig(text, &v);
        return std::make_pair(c, v);
      },
      "yaml"_a, "Parse (PlannerConfig, VehicleParams) from a YAML string.");

  // --- results -------------------------------------------------------------
  nb::enum_<Rejection>(m, "Rejection")
      .value("OK", Rejection::kOk)
      .value("INVALID", Rejection::kInvalid)
      .value("SPEED", Rejection::kSpeed)
      .value("ACCEL", Rejection::kAccel)
      .value("CURVATURE", Rejection::kCurvature)
      .value("LAT_ACCEL", Rejection::kLatAccel)
      .value("OFF_ROAD", Rejection::kOffRoad)
      .value("COLLISION", Rejection::kCollision)
      .value("SAFETY_DISTANCE", Rejection::kSafetyDistance);

  nb::class_<CostTerms>(m, "CostTerms")
      .def_ro("time", &CostTerms::time)
      .def_ro("speed", &CostTerms::speed)
      .def_ro("accel", &CostTerms::accel)
      .def_ro("jerk", &CostTerms::jerk)
      .def_ro("lateral", &CostTerms::lateral)
      .def_ro("lane_change", &CostTerms::lane_change)
      .def_ro("proximity", &CostTerms::proximity)
      .def("total", &CostTerms::total);

  nb::class_<Trajectory>(m, "Trajectory")
      .def(nb::init<>())
      .def_ro("end", &Trajectory::end)
      .def_ro("idx", &Trajectory::idx)
      .def_ro("lane", &Trajectory::lane)
      .def_ro("cost", &Trajectory::cost)
      .def_ro("is_cartesian", &Trajectory::is_cartesian)
      .def_ro("rejection", &Trajectory::rejection)
      .def_ro("blocking_id", &Trajectory::blocking_id)
      .def_ro("blocking_t", &Trajectory::blocking_t)
      .def("__len__", &Trajectory::size)
      .def("total_cost", &Trajectory::totalCost)
      .def("cartesian_state_at", &Trajectory::cartesianStateAt, "i"_a)
      .def("frenet_state_at", &Trajectory::frenetStateAt, "i"_a)
      .def("sample", &sampleTrajectory, "t"_a, "Cartesian state at time t (interpolated).")
      .TRAJ_ARRAY(t)
      .TRAJ_ARRAY(s)
      .TRAJ_ARRAY(s_d)
      .TRAJ_ARRAY(s_dd)
      .TRAJ_ARRAY(s_ddd)
      .TRAJ_ARRAY(d)
      .TRAJ_ARRAY(d_d)
      .TRAJ_ARRAY(d_dd)
      .TRAJ_ARRAY(d_ddd)
      .TRAJ_ARRAY(x)
      .TRAJ_ARRAY(y)
      .TRAJ_ARRAY(yaw)
      .TRAJ_ARRAY(v)
      .TRAJ_ARRAY(a)
      .TRAJ_ARRAY(kappa);

  nb::class_<LeadGap>(m, "LeadGap")
      .def(nb::init<>())
      .def_ro("id", &LeadGap::id)
      .def_ro("gap", &LeadGap::gap)
      .def_ro("time_headway", &LeadGap::time_headway)
      .def_ro("ttc", &LeadGap::ttc)
      .def_ro("speed", &LeadGap::speed);

  nb::class_<PlanStats>(m, "PlanStats")
      .def(nb::init<>())
      .def_ro("num_generated", &PlanStats::num_generated)
      .def_ro("num_validated", &PlanStats::num_validated)
      .def_ro("num_collision_checks", &PlanStats::num_collision_checks)
      .def_ro("num_iterations", &PlanStats::num_iterations)
      .def_ro("time_ms", &PlanStats::time_ms)
      .def_ro("blocking_id", &PlanStats::blocking_id)
      .def_ro("blocking_t", &PlanStats::blocking_t)
      .def_ro("lead", &PlanStats::lead)
      .def_prop_ro("rejections", [](const PlanStats& s) {
        nb::dict d;
        for (int i = 1; i < static_cast<int>(Rejection::kCount); ++i) {
          d[rejectionName(static_cast<Rejection>(i))] = s.rejections[static_cast<size_t>(i)];
        }
        return d;
      });

  nb::class_<PlanResult>(m, "PlanResult")
      .def_ro("success", &PlanResult::success)
      .def_ro("trajectory", &PlanResult::trajectory)
      .def_ro("stats", &PlanResult::stats)
      .def_ro("candidates", &PlanResult::candidates);

  nb::class_<Planner>(m, "Planner")
      .def_prop_ro("type", &Planner::type)
      .def_prop_ro("config", &Planner::config)
      .def_prop_ro("vehicle", &Planner::vehicle)
      .def("set_config", &Planner::setConfig, "config"_a)
      .def("reset", &Planner::reset)
      .def(
          "plan",
          [](Planner& p, const FrenetState& start, const ReferenceLine& ref, const Corridor& corridor,
             const std::vector<Obstacle>& obstacles, double target_speed) {
            PlanningInput in;
            in.start = start;
            in.reference = &ref;
            in.corridor = &corridor;
            in.obstacles = &obstacles;
            in.target_speed = target_speed;
            nb::gil_scoped_release release;
            return p.plan(in);
          },
          "start"_a, "reference"_a, "corridor"_a, "obstacles"_a = std::vector<Obstacle>{}, "target_speed"_a);
  m.def("make_planner", &makePlanner, "config"_a, "vehicle"_a = VehicleParams{});

  // --- map -----------------------------------------------------------------
  nb::enum_<LineType>(m, "LineType")
      .value("UNKNOWN", LineType::kUnknown)
      .value("DASHED", LineType::kDashed)
      .value("SOLID", LineType::kSolid)
      .value("ROAD_EDGE", LineType::kRoadEdge)
      .value("VIRTUAL", LineType::kVirtual);

  nb::class_<MapLane>(m, "MapLane")
      .def(nb::init<>())
      .def_rw("id", &MapLane::id)
      .def_prop_rw(
          "centerline", [](const MapLane& l) { return pointsToNumpy(l.centerline); },
          [](MapLane& l, const InPoints& a) { l.centerline = pointsFromNumpy(a); })
      .def_prop_rw(
          "left", [](const MapLane& l) { return pointsToNumpy(l.left); },
          [](MapLane& l, const InPoints& a) { l.left = pointsFromNumpy(a); })
      .def_prop_rw(
          "right", [](const MapLane& l) { return pointsToNumpy(l.right); },
          [](MapLane& l, const InPoints& a) { l.right = pointsFromNumpy(a); })
      .def_rw("left_type", &MapLane::left_type)
      .def_rw("right_type", &MapLane::right_type)
      .def_rw("left_neighbor", &MapLane::left_neighbor)
      .def_rw("right_neighbor", &MapLane::right_neighbor)
      .def_rw("can_change_left", &MapLane::can_change_left)
      .def_rw("can_change_right", &MapLane::can_change_right)
      .def_rw("successors", &MapLane::successors)
      .def_rw("predecessors", &MapLane::predecessors)
      .def_rw("speed_limit", &MapLane::speed_limit)
      .def("length", &MapLane::length);

  nb::class_<StopLine>(m, "StopLine")
      .def(nb::init<>())
      .def(
          "__init__",
          [](StopLine* s, int id, int lane_id, double sl, bool active) {
            new (s) StopLine{id, lane_id, sl, active};
          },
          "id"_a, "lane_id"_a, "s"_a, "active"_a = true)
      .def_rw("id", &StopLine::id)
      .def_rw("lane_id", &StopLine::lane_id)
      .def_rw("s", &StopLine::s)
      .def_rw("active", &StopLine::active);

  nb::class_<RoadMap>(m, "RoadMap")
      .def(nb::init<>())
      .def("add_lane", &RoadMap::addLane, "lane"_a)
      .def("add_stop_line", &RoadMap::addStopLine, "stop_line"_a)
      .def("set_stop_line_active", &RoadMap::setStopLineActive, "id"_a, "active"_a)
      .def("finalize", &RoadMap::finalize)
      .def(
          "lane",
          [](const RoadMap& m, int id) -> std::optional<MapLane> {
            const MapLane* l = m.lane(id);
            return l ? std::optional<MapLane>(*l) : std::nullopt;
          },
          "id"_a)
      .def("lane_ids",
           [](const RoadMap& m) {
             std::vector<int> ids;
             for (const auto& [id, _] : m.lanes()) ids.push_back(id);
             std::sort(ids.begin(), ids.end());
             return ids;
           })
      .def_prop_ro("stop_lines", &RoadMap::stopLines)
      .def(
          "locate",
          [](const RoadMap& m, double x, double y, double yaw, double max_dist) {
            return m.locate(x, y, yaw, nullptr, max_dist);
          },
          "x"_a, "y"_a, "yaw"_a, "max_dist"_a = 5.0);

  nb::class_<LaneFollowingConfig>(m, "LaneFollowingConfig")
      .def(nb::init<>())
      .def_rw("max_heading_diff", &LaneFollowingConfig::max_heading_diff)
      .def_rw("max_lane_distance", &LaneFollowingConfig::max_lane_distance)
      .def_rw("min_speed", &LaneFollowingConfig::min_speed)
      .def_rw("lateral_time_constant", &LaneFollowingConfig::lateral_time_constant);
  m.def("predict_lane_following", &predictLaneFollowing, "map"_a, "id"_a, "x"_a, "y"_a, "yaw"_a, "v"_a,
        "length"_a, "width"_a, "horizon"_a, "dt"_a, "config"_a = LaneFollowingConfig{},
        nb::call_guard<nb::gil_scoped_release>());
  m.def(
      "complete_predictions",
      [](std::vector<Obstacle> obstacles, const std::string& mode, const RoadMap* map, double horizon,
         double dt) {
        completePredictions(obstacles, predictionModeFromString(mode), map, horizon, dt);
        return obstacles;
      },
      "obstacles"_a, "mode"_a = "lane_following", "map"_a = nb::none(), "horizon"_a = 5.5, "dt"_a = 0.1,
      "Obstacles with predictions for those that carry only their current state "
      "(mode: as_given | constant_velocity | lane_following).");

  nb::class_<RouteCorridorConfig>(m, "RouteCorridorConfig")
      .def(nb::init<>())
      .def_rw("lookahead", &RouteCorridorConfig::lookahead)
      .def_rw("lookbehind", &RouteCorridorConfig::lookbehind)
      .def_rw("station_ds", &RouteCorridorConfig::station_ds)
      .def_rw("smoothing_length", &RouteCorridorConfig::smoothing_length)
      .def_rw("smoothing_tolerance", &RouteCorridorConfig::smoothing_tolerance)
      .def_rw("max_adjacent_lanes", &RouteCorridorConfig::max_adjacent_lanes)
      .def_rw("w_route_urgency", &RouteCorridorConfig::w_route_urgency)
      .def_rw("urgency_distance", &RouteCorridorConfig::urgency_distance)
      .def_rw("urgency_time", &RouteCorridorConfig::urgency_time);

  nb::class_<RouteCorridor>(m, "RouteCorridor")
      .def_ro("reference", &RouteCorridor::reference)
      .def_ro("corridor", &RouteCorridor::corridor)
      .def_ro("ego_chain", &RouteCorridor::ego_chain)
      .def_ro("ego_lane", &RouteCorridor::ego_lane)
      .def_ro("ego_s", &RouteCorridor::ego_s)
      .def_ro("speed_limit", &RouteCorridor::speed_limit)
      .def_ro("stop_lines", &RouteCorridor::stop_lines);
  m.def(
      "build_route_corridor",
      [](const RoadMap& map, const std::vector<int>& route, const EgoState& ego,
         const RouteCorridorConfig& cfg, int previous_ego_lane,
         std::optional<std::pair<double, double>> goal) {
        return buildRouteCorridor(map, route, ego, cfg, previous_ego_lane, toVec2(goal));
      },
      "map"_a, "route"_a, "ego"_a, "config"_a = RouteCorridorConfig{}, "previous_ego_lane"_a = -1,
      "goal"_a = nb::none());

  // --- pipeline ------------------------------------------------------------
  nb::class_<StitchingConfig>(m, "StitchingConfig")
      .def(nb::init<>())
      .def_rw("enabled", &StitchingConfig::enabled)
      .def_rw("latency", &StitchingConfig::latency)
      .def_rw("max_position_error", &StitchingConfig::max_position_error)
      .def_rw("max_heading_error", &StitchingConfig::max_heading_error);
  nb::class_<PathModeConfig>(m, "PathModeConfig")
      .def(nb::init<>())
      .def_rw("lane_width", &PathModeConfig::lane_width)
      .def_rw("num_left_lanes", &PathModeConfig::num_left_lanes)
      .def_rw("num_right_lanes", &PathModeConfig::num_right_lanes)
      .def_rw("allow_left", &PathModeConfig::allow_left)
      .def_rw("allow_right", &PathModeConfig::allow_right);
  m.def("has_optimizers", &hasOptimizers, "Whether the OSQP smoother and MPC are available.");
  nb::class_<SmootherConfig>(m, "SmootherConfig")
      .def(nb::init<>())
      .def_rw("enabled", &SmootherConfig::enabled)
      .def_rw("margin_lat", &SmootherConfig::margin_lat)
      .def_rw("margin_lon", &SmootherConfig::margin_lon)
      .def_rw("w_lat_ref", &SmootherConfig::w_lat_ref)
      .def_rw("w_lat_target", &SmootherConfig::w_lat_target)
      .def_rw("w_lat_accel", &SmootherConfig::w_lat_accel)
      .def_rw("w_lat_jerk", &SmootherConfig::w_lat_jerk)
      .def_rw("w_lon_ref", &SmootherConfig::w_lon_ref)
      .def_rw("w_lon_speed_ref", &SmootherConfig::w_lon_speed_ref)
      .def_rw("w_lon_speed", &SmootherConfig::w_lon_speed)
      .def_rw("w_lon_accel", &SmootherConfig::w_lon_accel)
      .def_rw("w_lon_jerk", &SmootherConfig::w_lon_jerk);

  // --- control -------------------------------------------------------------
  nb::class_<ControlCommand>(m, "ControlCommand")
      .def(nb::init<>())
      .def(
          "__init__",
          [](ControlCommand* c, double steer, double accel) { new (c) ControlCommand{steer, accel}; },
          "steer"_a, "accel"_a)
      .def_rw("steer", &ControlCommand::steer)
      .def_rw("accel", &ControlCommand::accel);
  nb::class_<KinematicBicycle>(m, "KinematicBicycle")
      .def(nb::init<const VehicleParams&, const EgoState&, double>(), "vehicle"_a, "initial"_a,
           "steer"_a = 0.0)
      .def("step", &KinematicBicycle::step, "command"_a, "dt"_a)
      .def_prop_ro("state", &KinematicBicycle::state)
      .def_prop_ro("steer", &KinematicBicycle::steer);
  nb::class_<Controller>(m, "Controller")
      .def("compute", &Controller::compute, "ego"_a, "trajectory"_a, "t"_a)
      .def("reset", &Controller::reset)
      .def_prop_ro("name", &Controller::name);
  nb::class_<StanleyPidConfig>(m, "StanleyPidConfig")
      .def(nb::init<>())
      .def_rw("k_heading", &StanleyPidConfig::k_heading)
      .def_rw("k_cross", &StanleyPidConfig::k_cross)
      .def_rw("k_soft", &StanleyPidConfig::k_soft)
      .def_rw("kp", &StanleyPidConfig::kp)
      .def_rw("ki", &StanleyPidConfig::ki)
      .def_rw("kd", &StanleyPidConfig::kd)
      .def_rw("dt", &StanleyPidConfig::dt);
  nb::class_<StanleyPidController, Controller>(m, "StanleyPidController")
      .def(nb::init<const VehicleParams&, const StanleyPidConfig&>(), "vehicle"_a,
           "config"_a = StanleyPidConfig{});
  nb::class_<MpcConfig>(m, "MpcConfig")
      .def(nb::init<>())
      .def_rw("horizon", &MpcConfig::horizon)
      .def_rw("dt", &MpcConfig::dt)
      .def_rw("q_lat", &MpcConfig::q_lat)
      .def_rw("q_heading", &MpcConfig::q_heading)
      .def_rw("q_lon", &MpcConfig::q_lon)
      .def_rw("q_speed", &MpcConfig::q_speed)
      .def_rw("r_steer", &MpcConfig::r_steer)
      .def_rw("r_accel", &MpcConfig::r_accel)
      .def_rw("rd_steer", &MpcConfig::rd_steer)
      .def_rw("rd_accel", &MpcConfig::rd_accel);
  nb::class_<MpcController, Controller>(m, "MpcController")
      .def(nb::init<const VehicleParams&, const MpcConfig&>(), "vehicle"_a, "config"_a = MpcConfig{})
      .def("last_solve_ok", &MpcController::lastSolveOk);
  m.def("make_controller", &makeController, "name"_a, "vehicle"_a = VehicleParams{});

  nb::class_<LocalPlannerConfig>(m, "LocalPlannerConfig")
      .def(nb::init<>())
      .def_rw("planner", &LocalPlannerConfig::planner)
      .def_rw("route", &LocalPlannerConfig::route)
      .def_rw("stitching", &LocalPlannerConfig::stitching)
      .def_rw("path", &LocalPlannerConfig::path)
      .def_rw("smoother", &LocalPlannerConfig::smoother)
      .def_rw("desired_speed", &LocalPlannerConfig::desired_speed)
      .def_rw("emergency_decel", &LocalPlannerConfig::emergency_decel)
      .def_rw("degraded_horizon", &LocalPlannerConfig::degraded_horizon)
      .def_rw("recovery_decel", &LocalPlannerConfig::recovery_decel)
      .def_rw("use_speed_limit", &LocalPlannerConfig::use_speed_limit)
      .def_rw("use_curvature_speed_limit", &LocalPlannerConfig::use_curvature_speed_limit);

  nb::class_<LocalPlanOutput>(m, "LocalPlanOutput")
      .def_ro("success", &LocalPlanOutput::success)
      .def_ro("fallback", &LocalPlanOutput::fallback)
      .def_ro("emergency", &LocalPlanOutput::emergency)
      .def_ro("degraded", &LocalPlanOutput::degraded)
      .def_ro("stitched", &LocalPlanOutput::stitched)
      .def_ro("smoothed", &LocalPlanOutput::smoothed)
      .def_ro("trajectory", &LocalPlanOutput::trajectory)
      .def_ro("stats", &LocalPlanOutput::stats)
      .def_ro("candidates", &LocalPlanOutput::candidates)
      .def_prop_ro("reference",
                   [](const LocalPlanOutput& o) { return o.reference ? *o.reference : ReferenceLine(); })
      .def_ro("corridor", &LocalPlanOutput::corridor)
      .def_ro("start", &LocalPlanOutput::start)
      .def_ro("target_speed", &LocalPlanOutput::target_speed)
      .def_ro("ego_lane", &LocalPlanOutput::ego_lane)
      .def_ro("virtual_obstacles", &LocalPlanOutput::virtual_obstacles);

  nb::class_<LocalPlanner>(m, "LocalPlanner")
      .def(nb::init<const LocalPlannerConfig&, const VehicleParams&>(), "config"_a = LocalPlannerConfig{},
           "vehicle"_a = VehicleParams{})
      .def(
          "set_map",
          [](LocalPlanner& lp, const RoadMap& map, std::vector<int> route,
             std::optional<std::pair<double, double>> goal) {
            lp.setMap(std::make_shared<RoadMap>(map), std::move(route), toVec2(goal));
          },
          "map"_a, "route"_a = std::vector<int>{}, "goal"_a = nb::none(),
          "Plan along `route` (lane ids in driving order); `goal` (x, y) in the last route lane is where "
          "its lane changes must be done.")
      .def("set_reference_path", &LocalPlanner::setReferencePath, "x"_a, "y"_a)
      .def("request_config", &LocalPlanner::requestConfig, "config"_a)
      .def("request_planner_type", &LocalPlanner::requestPlannerType, "type"_a)
      .def("set_stop_line_active", &LocalPlanner::setStopLineActive, "stop_line_id"_a, "active"_a)
      .def("reset", &LocalPlanner::reset)
      .def(
          "step",
          [](LocalPlanner& lp, double t, const EgoState& ego, const std::vector<Obstacle>& obstacles,
             double desired_speed) {
            nb::gil_scoped_release release;
            return lp.step(t, ego, obstacles, desired_speed);
          },
          "t"_a, "ego"_a, "obstacles"_a = std::vector<Obstacle>{}, "desired_speed"_a = -1.0)
      .def_prop_ro("config", &LocalPlanner::config)
      .def_prop_ro("planner_type", [](const LocalPlanner& lp) { return lp.planner().type(); });
}
