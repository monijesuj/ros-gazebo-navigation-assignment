import math
from types import SimpleNamespace
import numpy as np
from robot_demo.sensing import RayMap, PositionFilter, depth_distance


def test_mapping_begins_unknown():
    mapper = RayMap([0, 4, 0, 4], 0.1)
    assert np.all(mapper.occupancy() == -1)


def test_ray_clears_free_space_and_preserves_unknown_behind_hit():
    mapper = RayMap([0, 4, 0, 4], 0.1)
    mapper.update((0.5, 1.5), 0, [1.0], 0, 0, 0.12, 3)
    data = mapper.occupancy()
    assert data[15, 10] < 50
    assert data[15, 15] >= 65
    assert data[15, 20] == -1


def test_subsequent_clear_rays_remove_obstacle():
    mapper = RayMap([0, 4, 0, 4], 0.1)
    mapper.update((0.5, 1.5), 0, [1.0], 0, 0, 0.12, 3)
    for _ in range(4):
        mapper.update((0.5, 1.5), 0, [float("inf")], 0, 0, 0.12, 3)
    assert mapper.occupancy()[15, 15] < 50


def test_invalid_ray_leaves_unknown_and_max_range_is_not_obstacle():
    mapper = RayMap([0, 4, 0, 4], 0.1)
    mapper.update((0.5, 1.5), 0, [float("nan")], 0, 0, 0.12, 3)
    assert np.all(mapper.occupancy() == -1)
    mapper.update((0.5, 1.5), 0, [3.0], 0, 0, 0.12, 3)
    assert np.max(mapper.occupancy()) < 50


def test_prediction_rotates_wheel_increments_into_imu_heading():
    position = PositionFilter()
    position.gps((3.0, 4.0))
    position.yaw = math.pi / 2
    position.predict(0, 0, 0)
    position.predict(1, 0, 0)
    assert np.allclose(position.xy, (3, 5))


def test_gps_rejects_outlier_and_heading_crosses_pi():
    position = PositionFilter()
    position.gps((0, 0))
    assert not position.gps((100, 100))
    assert position.xy == (0, 0)
    position.yaw = math.pi - 0.01
    position.heading(-math.pi + 0.01)
    assert abs(abs(position.yaw) - math.pi) < 1e-5


def test_gps_corrects_wheel_slip():
    position = PositionFilter()
    position.gps((0, 0))
    position.predict(0, 0, 0)
    position.predict(1, 0, 0)
    assert position.gps((0.8, 0))
    assert abs(position.xy[0] - 0.8) < 0.01


def test_depth_16bit_big_endian_and_padded_rows():
    data = np.full((10, 12), 1200, dtype=">u2")
    image = SimpleNamespace(
        width=10,
        height=10,
        step=24,
        encoding="16UC1",
        is_bigendian=True,
        data=data.tobytes(),
    )
    assert abs(depth_distance(image) - 1.2) < 1e-5


def test_depth_float_excludes_invalid_values():
    data = np.full((10, 10), 0.75, dtype="<f4")
    data[3, 3] = float("nan")
    data[3, 4] = 0
    image = SimpleNamespace(
        width=10,
        height=10,
        step=40,
        encoding="32FC1",
        is_bigendian=False,
        data=data.tobytes(),
    )
    assert depth_distance(image) == 0.75
    image.data = b"bad"
    assert depth_distance(image) is None


def test_guard_margin_exceeds_braking_plus_observation_latency():
    from pathlib import Path
    import json
    from robot_demo.sensing import STOP_MARGIN

    profiles = json.loads(
        (Path(__file__).parents[1] / "config/robots.json").read_text()
    )
    for profile in profiles.values():
        speed = profile["max_speed"]
        assert STOP_MARGIN > speed**2 / (2 * 0.5) + speed * 0.15
