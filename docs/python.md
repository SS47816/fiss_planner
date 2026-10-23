# Python

```bash
pip install frenet-planner               # bindings, smoother and MPC included
pip install "frenet-planner[commonroad]" # CommonRoad adapter and benchmark
```

## A single planning call

```python
import numpy as np
import frenet_planner as fp

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
```

## The full pipeline

```python
lp = fp.LocalPlanner(fp.LocalPlannerConfig(), fp.VehicleParams())
lp.set_map(road_map, route_lane_ids)       # or lp.set_reference_path(xs, ys)
out = lp.step(t, fp.EgoState(x, y, yaw, v), obstacles)
lp.request_planner_type(fp.PlannerType.FOP)  # takes effect next cycle
```

`road_map` is an `fp.RoadMap` of `fp.MapLane`s; adapters build it from CommonRoad
(`frenet_planner.commonroad.road_map_from_lanelet_network`) and CARLA
(`frenet_planner.carla.adapter.road_map_from_carla`).

## Control

```python
ctrl = fp.make_controller("mpc", vehicle)       # or "stanley_pid"
car = fp.KinematicBicycle(vehicle, ego)
car.step(ctrl.compute(car.state, out.trajectory, t_since_plan), 0.05)
```
