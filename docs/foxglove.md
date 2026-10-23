# Inspecting runs in Foxglove

Every CARLA and CommonRoad run can be written to an [MCAP](https://mcap.dev) file, or streamed
live, and opened in [Foxglove](https://foxglove.dev) or the open-source
[Lichtblick](https://github.com/lichtblick-suite/lichtblick). Scrub through a run frame by frame and
see what the planner saw: every candidate with its cost rank, the obstacle that blocked the cheapest
candidate, the predicted paths, gap / headway / time to collision to the vehicle ahead, planning
times and (in CARLA) the camera.

The recordings of the demo runs are attached to the GitHub release
([carla_cut_in.mcap, carla_traffic.mcap, commonroad_zip.mcap](https://github.com/SS47816/frenet_planner/releases)).

## Open a recording

1. Download a `.mcap` file and the layout
   [`docs/foxglove/frenet_planner_layout.json`](https://github.com/SS47816/frenet_planner/blob/main/docs/foxglove/frenet_planner_layout.json).
2. Open the file in Foxglove (desktop app or app.foxglove.dev; a free account is required) or
   Lichtblick.
3. Import the layout (*Layouts → Import from file*).

The layout shows a 3D view following the ego, the camera, plots of speed vs. target, headway and
TTC, planning time and trajectory count, and the raw planner statistics.

| Topic | Content |
|---|---|
| `/map/lanes` | lane boundaries |
| `/ego` | ego box and heading |
| `/obstacles` | obstacle boxes and predicted paths; vehicle ahead yellow, blocking obstacle orange |
| `/plan/trajectory` | executed plan |
| `/plan/candidates` | explored candidates, cheapest brightest; rejected ones faint |
| `/plan/stats` | JSON: planner, mode (plan / fallback / degraded / emergency), timings, counts, gap, headway, TTC, rejections |
| `/tf` | `map → ego` |
| `/camera/image` | JPEG chase camera (CARLA) |

## Record your own

```bash
pip install "frenet-planner[foxglove,commonroad]"
# CommonRoad scenario
python -m frenet_planner.commonroad.demo ZAM_Zip-1_50_T-1.xml --mcap zip.mcap
# CARLA with the chase camera (start the simulator first)
python -m frenet_planner.carla.record --scenario cut_in --mcap cut_in.mcap
# CARLA batch runs, one file per run
python -m frenet_planner.carla.closed_loop --scenario traffic --mcap recordings/
```

Add `--foxglove-live` to the CARLA commands and connect Foxglove to `ws://localhost:8765` to watch a
run while it drives. For the ROS nodes, record the topics with `ros2 bag record -s mcap` (see
[ROS](ros.md#visualization-and-recording)).
