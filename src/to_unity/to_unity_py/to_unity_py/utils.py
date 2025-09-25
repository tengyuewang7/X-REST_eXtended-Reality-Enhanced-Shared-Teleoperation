import torch
from scipy.spatial.transform import Rotation as R

def quat_conjugate(a):
    shape = a.shape
    a = a.reshape(-1, 4)
    return torch.cat((-a[:, :3], a[:, -1:]), dim=-1).view(shape)

def quat_mul(a, b):
    shape = a.shape
    a = a.reshape(-1, 4)
    b = b.reshape(-1, 4)

    x1, y1, z1, w1 = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    x2, y2, z2, w2 = b[:, 0], b[:, 1], b[:, 2], b[:, 3]
    ww = (z1 + x1) * (x2 + y2)
    yy = (w1 - y1) * (w2 + z2)
    zz = (w1 + y1) * (w2 - z2)
    xx = ww + yy + zz
    qq = 0.5 * (xx + (z1 - x1) * (x2 - y2))
    w = qq - ww + (z1 - y1) * (y2 - z2)
    x = qq - xx + (x1 + w1) * (x2 + w2)
    y = qq - yy + (w1 - x1) * (y2 + z2)
    z = qq - zz + (z1 + y1) * (w2 - x2)

    quat = torch.stack([x, y, z, w], dim=-1).view(shape)

    return quat

def orientation_error(desired, current):
    cc = quat_conjugate(current)
    q_r = quat_mul(desired, cc)
    return q_r[0:3] * torch.sign(q_r[3]) 

def ros_pose_to_tensor(pose): 
    # [px, py, pz, qx, qy, qz, qw]
    return torch.tensor([pose.position.x, pose.position.y, pose.position.z, 
                         pose.orientation.x, pose.orientation.y, pose.orientation.z, 
                         pose.orientation.w], dtype=torch.float) 

def up_in_z(origianl_pose, l_z):
    pose = origianl_pose.clone()
    rot = R.from_quat(origianl_pose[3:]).as_matrix() 
    pose[0] -= l_z * rot[0, 2] 
    pose[1] -= l_z * rot[1, 2] 
    pose[2] -= l_z * rot[2, 2] 
    return pose
   