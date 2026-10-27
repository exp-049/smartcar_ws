from pathlib import Path

import pytest
import yaml
from launch import LaunchContext

from smartcar_bringup.runtime import agent_arguments, validate_calibration


ROOT = Path(__file__).parents[3]
DESCRIPTION = ROOT / 'src' / 'smartcar_description' / 'config'


def context(**values):
    launch_context = LaunchContext()
    launch_context.launch_configurations.update(values)
    return launch_context


def test_nominal_config_is_accepted_for_sim():
    config = validate_calibration(DESCRIPTION / 'nominal.yaml', 'sim')
    assert config['model']['wheel_radius'] == 0.024


def test_unmeasured_template_is_rejected_for_real():
    with pytest.raises(RuntimeError, match='requires measured calibration'):
        validate_calibration(DESCRIPTION / 'real.template.yaml', 'real')


def test_measured_robot_requires_provenance_and_valid_footprint(tmp_path):
    config = yaml.safe_load((DESCRIPTION / 'real.template.yaml').read_text())
    config['metadata'].update(status='measured', calibration_version='car01-v1',
                              measurement_method='tape_and_turn_test', source_data='calibration-bag')
    config['device_calibration'] = {
        'chassis_firmware_version': 'fw-a', 'chassis_calibration_version': 'drive-v1',
        'imu_firmware_version': 'fw-b', 'imu_calibration_version': 'imu-v1',
    }
    path = tmp_path / 'car01.yaml'
    path.write_text(yaml.safe_dump(config))
    assert validate_calibration(path, 'real')['metadata']['robot_id'] == 'car01'
    config['model']['footprint_radius'] = 0.01
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(RuntimeError, match='physical model envelope'):
        validate_calibration(path, 'real')


def test_agent_udp_and_serial_arguments_are_explicit():
    assert agent_arguments(context(agent_transport='udp4', agent_port='8888')) == ['udp4', '-p', '8888']
    assert agent_arguments(context(agent_transport='serial', agent_device='/dev/ttyUSB0',
                                   agent_baudrate='115200')) == ['serial', '--dev', '/dev/ttyUSB0', '-b', '115200']
    with pytest.raises(RuntimeError, match='agent_port'):
        agent_arguments(context(agent_transport='udp4', agent_port='0'))
