import pytest
from fiss_planner.carla import adapter


class FakeWaypoint:
    """A waypoint `s` along a straight lane of road 1 that is `length` long and
    continues into road 2 unless it is a dead end."""

    def __init__(self, s, length, dead_end, road_id=1):
        self.s, self.length, self.dead_end, self.road_id = s, length, dead_end, road_id

    def next(self, d):
        s = self.s + d
        if s <= self.length:
            return [FakeWaypoint(s, self.length, self.dead_end)]
        return [] if self.dead_end else [FakeWaypoint(s - self.length, 10.0, False, road_id=2)]

    def next_until_lane_end(self, d):
        if self.dead_end:
            raise RuntimeError("CARLA crashes here")
        lane, w = [], self
        while w.s + d <= self.length:
            w = w.next(d)[0]
            lane.append(w)
        return lane + [FakeWaypoint(self.length, self.length, False)]


def test_lane_walk_reaches_a_dead_end():
    lane = adapter._next_until_lane_end(FakeWaypoint(0.0, 9.0, dead_end=True), 2.0)
    assert [w.s for w in lane[:-1]] == pytest.approx([2.0, 4.0, 6.0, 8.0])
    assert lane[-1].s == pytest.approx(9.0, abs=0.06)


def test_lane_walk_matches_carla_where_the_lane_continues():
    lane = adapter._next_until_lane_end(FakeWaypoint(0.0, 9.0, dead_end=False), 2.0)
    assert [w.s for w in lane] == pytest.approx([2.0, 4.0, 6.0, 8.0, 9.0])
