"""Safety and hysteresis contracts for switching a live detour."""
import math

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.msg import Costmap
from nav2_msgs.srv import IsPathValid

from smartcar_tools.path_guard import PathGuard


@pytest.fixture
def guard():
    rclpy.init()
    node = PathGuard()
    transform = TransformStamped()
    transform.header.frame_id = 'odom'
    transform.child_frame_id = 'map'
    transform.transform.rotation.w = 1.0
    node.buffer.set_transform_static(transform, 'test')
    grid = Costmap()
    grid.header.frame_id = 'odom'
    grid.metadata.size_x = grid.metadata.size_y = 40
    grid.metadata.resolution = 0.1
    grid.metadata.origin.position.x = grid.metadata.origin.position.y = -2.0
    grid.data = [0] * 1600
    node.on_costmap(grid)
    yield node
    node.destroy_node()
    rclpy.shutdown()


def request(*points):
    value = IsPathValid.Request()
    value.path.header.frame_id = 'map'
    for x, y in points:
        p = PoseStamped()
        p.pose.position.x = x
        p.pose.position.y = y
        value.path.poses.append(p)
    return value


def check(guard, path):
    return guard.check(path, IsPathValid.Response()).is_valid


def test_blocked_segment_is_confirmed_then_cleared(guard, monkeypatch):
    now = [0.0]
    monkeypatch.setattr('smartcar_tools.path_guard.time.monotonic', lambda: now[0])
    guard.on_costmap(guard.costmap)
    # Both path endpoints are free; the wall lies strictly between them.
    guard.costmap.data[20 * 40 + 20] = 254
    path = request((-0.5, 0.05), (0.5, 0.05))
    assert check(guard, path)  # A single raster update does not flip the route.
    now[0] = 0.7
    assert not check(guard, path)
    guard.costmap.data[20 * 40 + 20] = 0
    guard.on_costmap(guard.costmap)
    assert check(guard, path)


def test_rotated_map_frame_checks_the_correct_obstacle(guard):
    transform = TransformStamped()
    transform.header.frame_id = 'odom'
    transform.child_frame_id = 'map'
    transform.transform.rotation.z = math.sin(math.pi / 4)
    transform.transform.rotation.w = math.cos(math.pi / 4)
    guard.buffer.set_transform_static(transform, 'test')
    guard.confirmation_time = 0.0
    guard.costmap.data[23 * 40 + 20] = 253
    assert not check(guard, request((0.1, -0.05), (0.6, -0.05)))
    assert check(guard, request((0.1, 0.5), (0.6, 0.5)))


def test_unavailable_data_cannot_validate_a_path(guard):
    guard.received_at -= guard.stale_timeout + 1.0
    assert not check(guard, request((0.0, 0.0), (0.1, 0.0)))
    guard.on_costmap(guard.costmap)
    assert not check(guard, request((4.0, 0.0)))
