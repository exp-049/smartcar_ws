import math

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from smartcar_tools.system_health import valid_sensor


def timestamp(msg):
    msg.header.stamp.sec = 1


def test_sensor_health_checks_public_frames_and_content():
    scan = LaserScan()
    timestamp(scan)
    scan.header.frame_id = 'laser_link'
    scan.angle_increment = 0.01
    scan.range_min = 0.12
    scan.range_max = 8.0
    scan.ranges = [math.inf, 1.0]
    assert valid_sensor('scan', scan)
    scan.header.frame_id = 'base_link'
    assert not valid_sensor('scan', scan)

    imu = Imu()
    timestamp(imu)
    imu.header.frame_id = 'imu_link'
    imu.orientation_covariance[0] = -1.0
    assert valid_sensor('imu', imu)
    imu.angular_velocity.z = float('nan')
    assert not valid_sensor('imu', imu)

    odom = Odometry()
    timestamp(odom)
    odom.header.frame_id = 'chassis_odom'
    odom.child_frame_id = 'base_link'
    odom.pose.pose.orientation.w = 1.0
    assert valid_sensor('odom', odom)
    odom.child_frame_id = 'wrong_frame'
    assert not valid_sensor('odom', odom)
