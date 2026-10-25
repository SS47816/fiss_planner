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
