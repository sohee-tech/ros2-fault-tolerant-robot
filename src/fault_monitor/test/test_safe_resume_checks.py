"""Safe Resume check functions (no ROS graph needed)."""
import math

from fault_monitor.safe_resume_gate import forward_clearance, pose_problem, required_clearance, scan_quality
from geometry_msgs.msg import PoseWithCovarianceStamped

INC = 2 * math.pi / 360


def scan(front=math.inf, others=math.inf):
    r = [others] * 360
    for i in list(range(0, 5)) + list(range(355, 360)):
        r[i] = front
    return r


def clearance(ranges):
    return forward_clearance(ranges, 0.0, INC, 0.12, 0.155, -0.032)


def pose(xy=0.01, yaw=0.01, frame='map'):
    m = PoseWithCovarianceStamped()
    m.header.frame_id = frame
    m.pose.pose.orientation.w = 1.0
    cov = [0.0] * 36
    cov[0], cov[7], cov[35] = xy, xy, yaw
    m.pose.covariance = cov
    return m


def test_required_clearance_terms():
    assert math.isclose(required_clearance(0.2, 0.25, 0.0, 0.5, 0.1, 0.105), 0.295)
    assert math.isclose(required_clearance(0.0, 0.25, 0.7, 0.5, 0.1, 0.105), 0.205)
    assert required_clearance(0.2, 0.25, 0.5, 0.5, 0.1, 0.105) > 0.295


def test_clearance_sees_obstacle_ahead_only():
    assert math.isclose(clearance(scan(front=0.25))[0], 0.25 - 0.032, abs_tol=0.01)
    assert clearance(scan())[0] == math.inf
    back = [math.inf] * 360
    back[180] = 0.2
    assert clearance(back)[0] == math.inf


def test_below_range_min_is_an_obstacle_not_free_space():
    assert clearance(scan(front=0.05))[0] < 0.1
    assert clearance(scan(front=-math.inf))[0] < 0.1


def test_nan_ahead_is_reported():
    assert clearance(scan(front=math.nan))[1] > 0.1


def test_scan_quality():
    assert scan_quality([math.nan] * 360, 0.12, 3.5) == (0.0, 0)
    assert scan_quality([], 0.12, 3.5) == (0.0, 0)
    assert scan_quality([1.0] * 360, 0.12, 3.5) == (1.0, 360)
    assert scan_quality([math.inf] * 360, 0.12, 3.5)[1] == 0


def test_pose_problem():
    assert pose_problem(pose(), 'map', 0.1, 0.15) == ''
    assert 'xy covariance' in pose_problem(pose(xy=1.0), 'map', 0.1, 0.15)
    assert 'yaw covariance' in pose_problem(pose(yaw=1.0), 'map', 0.1, 0.15)
    assert 'not finite' in pose_problem(pose(xy=math.nan), 'map', 0.1, 0.15)
    assert 'frame' in pose_problem(pose(frame='odom'), 'map', 0.1, 0.15)
