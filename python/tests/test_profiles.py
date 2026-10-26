import fiss_planner as fp
import pytest
from fiss_planner.commonroad.benchmark import Options, make_config


def test_shipped_profiles_load():
    assert {"commonroad", "carla_scenarios"} <= set(fp.profiles.names())
    for name in fp.profiles.names():
        cfg, _ = fp.profiles.load(name)
        cfg.validate()
    carla, _ = fp.profiles.load("carla_scenarios")
    assert carla.planner.collision.ignore_rear_in_lane
    assert carla.stitching.latency == pytest.approx(0.05)
    with pytest.raises(ValueError):
        fp.profiles.load("no_such_profile")


def test_profile_keeps_vehicle_fields_it_does_not_set():
    vehicle = fp.VehicleParams()
    vehicle.length = 4.25
    _, v = fp.profiles.load("commonroad", vehicle)
    assert v.length == 4.25


def test_override_and_yaml_round_trip():
    cfg, vehicle = fp.profiles.load("commonroad")
    cfg2, vehicle2 = fp.profiles.override(
        cfg,
        vehicle,
        {
            "cost.w_speed": 2.5,
            "route.urgency_time": 8.0,
            "desired_speed": 12.0,
            "smoother.enabled": True,
            "planner.type": "fiss",
            "vehicle.max_decel": 6.5,
        },
    )
    assert cfg2.planner.cost.w_speed == 2.5
    assert cfg2.route.urgency_time == 8.0
    assert cfg2.desired_speed == 12.0
    assert cfg2.smoother.enabled
    assert cfg2.planner.type == fp.PlannerType.FISS
    assert vehicle2.max_decel == 6.5
    assert cfg2.planner.cost.w_jerk == cfg.planner.cost.w_jerk  # untouched keys keep the base
    assert cfg.planner.cost.w_speed != 2.5  # the base is not modified
    text = fp.profiles.to_yaml(cfg2, vehicle2)
    cfg3, vehicle3 = fp.parse_local_planner_config(text)
    assert fp.profiles.to_yaml(cfg3, vehicle3) == text
    with pytest.raises(ValueError):
        fp.profiles.override(cfg, vehicle, {"cost.w_sped": 1.0})
    with pytest.raises(ValueError):
        fp.profiles.override(cfg, vehicle, {"route.station_ds": 0.0})


def test_config_hash_tracks_planning_settings_only():
    cfg, vehicle = fp.profiles.load("commonroad")
    h = fp.profiles.config_hash(cfg, vehicle)
    cfg.planner.keep_candidates = True
    cfg.planner.num_threads = 4
    assert fp.profiles.config_hash(cfg, vehicle) == h
    cfg.planner.cost.w_lane_change += 1.0
    assert fp.profiles.config_hash(cfg, vehicle) != h


def test_benchmark_options_override_the_profile():
    cfg, _ = make_config(Options())
    default = fp.LocalPlannerConfig()
    assert fp.to_yaml(cfg) == fp.to_yaml(default)  # the commonroad profile pins the defaults
    cfg, _ = make_config(Options(planner="fop", samples=(3, 4, 2), adjacent_lanes=0, smoother=True))
    assert cfg.planner.type == fp.PlannerType.FOP
    assert (cfg.planner.sampling.num_width, cfg.planner.sampling.num_speed, cfg.planner.sampling.num_t) == (
        3,
        4,
        2,
    )
    assert cfg.route.max_adjacent_lanes == 0
    assert cfg.smoother.enabled
    cfg, _ = make_config(Options(profile="carla_scenarios"))
    assert cfg.planner.collision.ignore_rear_in_lane
