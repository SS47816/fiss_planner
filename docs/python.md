# Python

```bash
pip install fiss-planner               # bindings, smoother and MPC included
pip install "fiss-planner[commonroad]" # CommonRoad adapter and benchmark
```

## A single planning call

```python
import numpy as np
import fiss_planner as fp

x = np.arange(0.0, 300.0, 5.0)
ref = fp.ReferenceLine(x, np.zeros_like(x))
lanes = fp.Corridor.parallel_lanes(0.0, ref.length, 3.5, 1, 0)
obstacles = [fp.predict_constant_velocity(1, 45.0, 0.0, 0.0, 0.0, 4.5, 1.9, horizon=6.0, dt=0.1)]

cfg = fp.make_config("fiss_plus", num_width=5)
cfg.cost.w_lane_change = 3.0
planner = fp.make_planner(cfg, fp.VehicleParams())
res = planner.plan(fp.FrenetState(s=10.0, s_d=10.0), ref, lanes, obstacles, target_speed=12.0)
traj = res.trajectory            # numpy arrays: traj.t, traj.x, traj.y, traj.yaw, traj.v, traj.kappa, traj.s, traj.d …
print(res.stats.rejections)      # why candidates were rejected
print(res.stats.lead.gap, res.stats.lead.ttc, res.stats.blocking_id)  # vehicle ahead; what blocked
```

Keeping distance is configured on the same object (see [Concepts](concepts.md#keeping-distance)):

```python
cfg.cost.desired_time_headway = 2.0   # comfort: car following and proximity cost
cfg.safety.min_ttc = 2.0              # hard envelope ("virtual bumper")
```

## The full pipeline

```python
lp = fp.LocalPlanner(fp.LocalPlannerConfig(), fp.VehicleParams())
lp.set_map(road_map, route_lane_ids, goal=(gx, gy))  # or lp.set_reference_path(xs, ys)
out = lp.step(t, fp.EgoState(x, y, yaw, v), obstacles)
lp.request_planner_type(fp.PlannerType.FOP)  # takes effect next cycle
```

`road_map` is an `fp.RoadMap` of `fp.MapLane`s; adapters build it from CommonRoad
(`fiss_planner.commonroad.road_map_from_lanelet_network`) and CARLA
(`fiss_planner.carla.adapter.road_map_from_carla`). The route lists lane ids in driving order and
ends in the destination lane; the optional goal, a point in that lane, is where the route's lane
changes must be done (see [route urgency](concepts.md#corridor-and-lanes)).

## Configuration files and profiles

Every `LocalPlannerConfig` setting has a YAML key named after its field, the same as the ROS 2
parameter: the scalars (`desired_speed`, `degraded_horizon`, ...) at the top level, then the
sections `planner`, `sampling`, `cost`, `constraints`, `collision`, `safety`, `fiss_plus`, `route`,
`stitching`, `path`, `behavior`, `free_space`, `smoother` and `vehicle`. Missing keys keep their
defaults and unknown keys raise an error.

```python
cfg, vehicle = fp.load_local_planner_config("my_config.yaml", vehicle)  # fields not in the file keep their value
print(fp.to_yaml(cfg))                                                   # every setting; reads back exactly
```

A *profile* is such a file with every setting pinned, tuned for one benchmark or simulator. The
shipped profiles are `commonroad` and `carla_scenarios` (see [Benchmarks](benchmarks.md#profiles-splits-and-tuning)):

```python
cfg, vehicle = fp.profiles.load("carla_scenarios", vehicle)
cfg, vehicle = fp.profiles.override(cfg, vehicle, {"cost.w_lane_change": 3.0, "desired_speed": 12.0})
fp.profiles.config_hash(cfg, vehicle)  # fingerprint the benchmarks record with every result
```

## Free space (parking)

With `cfg.free_space.enabled`, a `LocalPlanner` on a map that finds the vehicle at low speed outside
every lane of its corridor (parked on a strip, on a shoulder) plans a Hybrid A* path, forwards and
backwards, across the map's lanes and areas back into its lane, then hands over to the lane
planner. Add the drivable space outside the lanes with `road_map.add_area(polygon)`. While it drives
free space, `out.free_space` is set, `out.free_space_path` holds the path and the trajectory's speed
is negative where it reverses. The search is also available on its own:

```python
grid = fp.OccupancyGrid(cx, cy, 50.0, 0.1)   # starts occupied
grid.mark_free(lane_polygon)                 # (N, 2) arrays
grid.mark_occupied(obstacle.box_at(0.0))
grid.compute_clearance()
path = fp.plan_hybrid_astar(grid, fp.FreeSpacePose(x, y, yaw), goal, vehicle, fp.FreeSpaceConfig())
traj = fp.free_space_trajectory(path, v0, fp.FreeSpaceConfig(), 0.1)
```

## Prediction

```python
ob = fp.predict_lane_following(road_map, obstacle_id, x, y, yaw, v, length, width, horizon=5.5, dt=0.1)
obstacles = fp.complete_predictions(obstacles, "lane_following", road_map)  # single-state obstacles only
```

## Recording for Foxglove

```python
from fiss_planner.viz import SceneRecorder, obstacle_dict   # pip install foxglove-sdk

rec = SceneRecorder("run.mcap")              # or SceneRecorder(live=True) for ws://localhost:8765
rec.lanes((road_map.lane(i).left, road_map.lane(i).right) for i in road_map.lane_ids())
rec.frame(t, ego, vehicle, [obstacle_dict(o) for o in obstacles], out, "fiss_plus")
rec.close()
```

See [Foxglove](foxglove.md).

## Control

```python
ctrl = fp.make_controller("mpc", vehicle)       # or "stanley_pid"
car = fp.KinematicBicycle(vehicle, ego)
car.step(ctrl.compute(car.state, out.trajectory, t_since_plan), 0.05)
```
