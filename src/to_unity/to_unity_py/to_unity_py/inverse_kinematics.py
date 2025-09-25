import numpy as np
import torch
import time 
import copy 
from scipy.spatial.transform import Rotation as R
from forward_kinematics import ForwardKinematics


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



class InverseKinematics():
    def __init__(self): 
        self.damping = 0.05 

        self.flange_pose = None
        self.current_q = torch.tensor([0.0, -0.698132, 0.0, 1.570796, 0.0, 0.698132, 0.0], dtype=torch.float) 
        self.current_q = torch.tensor([-5.4173e-10, -1.1241e+00,  1.0339e-09,  1.5603e+00, -9.6400e-11, 2.6843e+00, -8.8768e-10], dtype=torch.float) 
        self.target_pose = torch.tensor([0.6888, -0.11, 0.291, 0, 0.707, 0, 0.707], dtype=torch.float)  
        self.fk = ForwardKinematics()
        self.j_eef = []

        self.run()

    def run(self): 
        for _ in range(100):
            self.current_q[0] = 0.0
            self.current_q[2] = 0.0
            self.current_q[4] = 0.0
            self.current_q[6] = 0.0
            self.fk.update_link_transformations(self.current_q) 
            flange_pose_trans = self.fk.get_transformation(8) 
            self.j_eef = self.fk.get_Jacobian(8, flange_pose_trans)
            rot = torch.tensor(R.from_matrix(flange_pose_trans[0:3, 0:3]).as_quat(), dtype=torch.float)
            self.flange_pose = torch.cat([flange_pose_trans[0:3, 3], rot], -1)

            pos_err = self.target_pose[0:3] - self.flange_pose[0:3] 
            quat_err = orientation_error(self.target_pose[3:7], rot)
            dpose = torch.cat([pos_err, quat_err], -1)

            u = self.control_ik(dpose) 
            self.current_q += u

            print(self.flange_pose)


    def control_ik(self, dpose):
        # solve damped least squares
        j_eef_T = torch.transpose(self.j_eef, 0, 1)
        lmbda = torch.eye(6) * (self.damping ** 2)
        u = (j_eef_T @ torch.inverse(self.j_eef @ j_eef_T + lmbda) @ dpose)
        return u



def main(args=None):
    ik = InverseKinematics()
    print("========")
    print(ik.current_q)

    # quat1 = torch.tensor([0, 0, 0, 1])
    # quat2 = torch.tensor([0, 0, 0.4472, 0.8944])
    # print(orientation_error(quat1, quat2)) 
    # print(orientation_error(quat2, quat1)) 


    # t1 = R.from_quat(quat1).as_matrix()
    # t2 = R.from_quat(quat2).as_matrix()
    # t = t1.T @ t2
    # print(t1)
    # print(t2)
    # print(t)
    # print(R.from_matrix(t).as_euler())




if __name__ == '__main__':
    main()
