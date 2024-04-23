from scipy.spatial.transform import Rotation as Rot
from geometry_msgs.msg import Pose

def angle_from_quaternion(q):
    
    # Needs an orientation from the pose message
    r = Rot.from_quat([q.x, q.y, q.z, q.w])
    return r.as_euler('zyx')[0]

def get_pose_from_position(position, orientation):
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = 0.0
    q = Rot.from_euler('zyx', [orientation, 0, 0]).as_quat()
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose