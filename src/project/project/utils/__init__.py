from scipy.spatial.transform import Rotation as Rot

def angle_from_quaternion(q):
    
    # Needs an orientation from the pose message
    r = Rot.from_quat([q.x, q.y, q.z, q.w])
    return r.as_euler('zyx')[0]