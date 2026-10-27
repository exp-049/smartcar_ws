"""Validate the selected sensor HAL executable and its parameters before launch."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, PackageNotFoundError
import yaml


def require_executable(package, executable):
    """Resolve a ROS libexec program, without falling back to unrelated binaries."""
    if not package or not executable or Path(executable).name != executable:
        raise RuntimeError('A ROS driver package and executable name are required')
    try:
        path = Path(get_package_prefix(package)) / 'lib' / package / executable
    except PackageNotFoundError as exc:
        raise RuntimeError(f'ROS package {package!r} is not installed; source its overlay') from exc
    if not path.is_file() or not os.access(path, os.X_OK):
        raise RuntimeError(f'ROS executable is unavailable: {package}/{executable}')
    return str(path)


def validate_hal(package, executable, params_file):
    """Validate the integration point, not an invented UDP wire protocol."""
    path = Path(params_file)
    if not path.is_file():
        raise RuntimeError(f'HAL parameter file does not exist: {params_file}')
    with path.open() as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or not config:
        raise RuntimeError('HAL parameter file must contain ROS node parameters')
    for entry in config.values():
        params = entry.get('ros__parameters', {}) if isinstance(entry, dict) else {}
        if params.get('integration_template', False):
            raise RuntimeError(
                'HAL 尚未配置: 参数文件仍是模板; provide real HAL parameters; '
                'set start_hal:=false when an external HAL already publishes the public sensor topics.')
    if not executable:
        raise RuntimeError(
            'HAL 尚未配置: supply an installed hal_package and hal_executable; or set '
            'start_hal:=false when an external HAL is already running.')
    require_executable(package, executable)
    return str(path)
