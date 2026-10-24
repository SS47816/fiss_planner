# FISS Planner

> [!IMPORTANT]
> **This project now lives in [SS47816/frenet_planner](https://github.com/SS47816/frenet_planner).**

FISS (RA-L 2022) is maintained in **frenet_planner**, together with FOP, FOP+ and the other FISS variants: one
C++17 core with Python bindings (`pip install frenet-planner`), ROS 2 / ROS 1 nodes, Lanelet2 / CommonRoad /
CARLA support and a reproducible benchmark. Select it with `planner.type: fiss`.

## Paper version

The code used in the paper is kept at the tag
[`paper-ral2022`](https://github.com/SS47816/fiss_planner/tree/paper-ral2022):

```bash
git clone --branch paper-ral2022 https://github.com/SS47816/fiss_planner.git
```

## Citation

```bibtex
@article{Sun_FISS_2022,
  author={Sun, Shuo and Liu, Zhiyang and Yin, Huan and Ang, Marcelo H.},
  journal={IEEE Robotics and Automation Letters},
  title={FISS: A Trajectory Planning Framework Using Fast Iterative Search and Sampling Strategy for Autonomous Driving},
  year={2022},
  volume={7},
  number={4},
  pages={9985-9992},
  doi={10.1109/LRA.2022.3191940}
}
```
