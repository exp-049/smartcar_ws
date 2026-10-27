from pathlib import Path

import pytest

from smartcar_hal.launch_support import require_executable, validate_hal


def test_missing_driver_fails_with_protocol_handoff_message():
    template = Path(__file__).parents[1] / 'config' / 'hal.template.yaml'
    with pytest.raises(RuntimeError, match='HAL 尚未配置'):
        validate_hal('smartcar_hal', '', template)


def test_unconfigured_template_cannot_be_started():
    template = Path(__file__).parents[1] / 'config' / 'hal.template.yaml'
    with pytest.raises(RuntimeError, match='HAL 尚未配置'):
        validate_hal('smartcar_hal', 'udp_hal', template)


def test_agent_or_driver_must_be_a_ros_executable():
    with pytest.raises(RuntimeError, match='ROS package'):
        require_executable('package_that_does_not_exist', 'driver')
