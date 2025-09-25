import numpy as np
import torch
import copy 
from scipy.spatial.transform import Rotation as R

class ForwardKinematics():
    def __init__(self):
        theta1, theta2, theta3, theta4, theta5, theta6, theta7 = 0, 0, 0, 0, 0, 0, 0
        DH_table = [[0, 0.365, 0, -np.pi],
                    [0, 0.065, -np.pi/2, 0], 
                    [0, 0.395, np.pi/2, 0], 
                    [0, 0.055, np.pi/2, np.pi], 
                    [0, 0.385, np.pi/2, np.pi], 
                    [0, 0.1, -np.pi/2, np.pi/2],
                    [0.11, 0, np.pi/2, np.pi/2], 
                    [0, 0.136, 0, -np.pi/2]]
        self.dof_num = 7 
        self.T_one_by_one = torch.empty((8, 4, 4))
        self.T_link = torch.empty((9, 4, 4)) 
        self.jaco = torch.zeros((6, 7))
        
    def update_one_by_one_transformations(self, thetas): 
        [theta1, theta2, theta3, theta4, theta5, theta6, theta7] = thetas 

        self.T_one_by_one[0] = torch.tensor([[-np.cos(theta1), np.sin(theta1), 0, 0 ], 
                            [-np.sin(theta1), -np.cos(theta1), 0, 0 ], 
                            [0, 0, 1, 0.365 ],
                            [0, 0, 0, 1 ]], dtype=torch.float) 

        self.T_one_by_one[1] = torch.tensor([[np.cos(theta2), -np.sin(theta2), 0, 0 ], 
                            [0, 0, 1, 0.065 ], 
                            [-np.sin(theta2), -np.cos(theta2), 0, 0 ],
                            [0, 0, 0, 1 ]], dtype=torch.float) 

        self.T_one_by_one[2] = torch.tensor([[np.cos(theta3), -np.sin(theta3), 0, 0 ], 
                            [0, 0, -1, -0.395 ], 
                            [np.sin(theta3), np.cos(theta3), 0, 0 ],
                            [0, 0, 0, 1 ]], dtype=torch.float) 

        self.T_one_by_one[3] = torch.tensor([[-np.cos(theta4), np.sin(theta4), 0, -0.02 ], 
                            [0, 0, -1, -0.055 ], 
                            [-np.sin(theta4), -np.cos(theta4), 0, 0 ],
                            [0, 0, 0, 1 ]], dtype=torch.float) 

        self.T_one_by_one[4] = torch.tensor([[-np.cos(theta5), np.sin(theta5), 0, -0.02 ], 
                            [0, 0, -1, -0.385 ], 
                            [-np.sin(theta5), -np.cos(theta5), 0, 0 ],
                            [0, 0, 0, 1 ]], dtype=torch.float) 

        self.T_one_by_one[5] = torch.tensor([[np.sin(theta6), np.cos(theta6), 0, 0 ], 
                            [0, 0, 1, 0.1 ], 
                            [np.cos(theta6), -np.sin(theta6), 0, 0 ],
                            [0, 0, 0, 1 ]], dtype=torch.float) 

        self.T_one_by_one[6] = torch.tensor([[np.sin(theta7), np.cos(theta7), 0, 0.1118], 
                            [0, 0, -1, 0 ], 
                            [-np.cos(theta7), np.sin(theta7), 0, 0 ],
                            [0, 0, 0, 1 ]], dtype=torch.float) 

        self.T_one_by_one[7] = torch.tensor([[0, 1, 0, 0 ], 
                                [-1, 0, 0, 0 ], 
                                [0, 0, 1, 0.132 ],
                                [0, 0, 0, 1 ]], dtype=torch.float) 
        

    def update_link_transformations(self, thetas):
        self.update_one_by_one_transformations(thetas)
        T_temp = torch.eye(4, dtype=torch.float)
        self.T_link[0] = torch.eye(4, dtype=torch.float)
        for i in range(len(self.T_one_by_one)): 
            self.T_link[i+1] = self.T_link[i] @ self.T_one_by_one[i]

    def get_transformation(self, link_id, length=1): 
        # update link transformations before running this method 
        # link_id starts from 1. Flange id is 8 with length=1. 
        trans = self.T_link[link_id]
        if (length != 1): 
            v = trans[0:3, 3] - self.T_link[link_id-1][0:3, 3]
            trans[0:3, 3] -= v * (1 - length)
        return trans 
    
    def get_point_on_link(self, link_id, length=1):
        v = (1 - length) * self.T_link[link_id - 1, 0:3, 3] + length * self.T_link[link_id, 0:3, 3]
        return v
    
    # def get_Jacobian(self, link_id, trans): 
    #     # update link transformations before running this method 
        
    #     jaco = torch.zeros((6, 7)) 
    #     t_f = trans[0:3, 3]
    #     for i in range(link_id-1): 
    #         z =  self.T_link[i+1][0:3, 2]
    #         t = t_f - self.T_link[i+1][0:3, 3]
    #         jaco[0:3, i] = torch.cross(z, t) 
    #         jaco[3:6, i] = z
    #     return jaco 
    
    def get_Jacobian(self, link_id, point): 
        # update link transformations before running this method 
        jaco = torch.zeros((6, 7)) 
        z = self.T_link[1:link_id, 0:3, 2] 
        t = point.unsqueeze(0) - self.T_link[1:link_id, 0:3, 3] 
        jaco[0:3, :link_id-1] = torch.transpose(torch.cross(z, t, dim=1), 0, 1)
        jaco[3:6, :link_id-1] = torch.transpose(z, 0, 1)

        return jaco
    
    def get_closest_on_links(self, point): 
        # Find the closest point to the obstacle point on the links 
        vector_1 = self.T_link[1:-1, 0:3, 3] - point
        vector_2 = self.T_link[2:, 0:3, 3] - point

        l = torch.sum(vector_1 * (vector_1 - vector_2), dim=1) / (torch.norm(vector_2 - vector_1, dim=1) ** 2)
        l = torch.clamp(l, min=0, max=1) 

        v = (1 - l).unsqueeze(1) * vector_1 + l.unsqueeze(1) * vector_2 
        min_index = torch.argmin(torch.norm(v, dim=1))

        return v[min_index], min_index+2, l[min_index]



def main(args=None):
    fk = ForwardKinematics()
    thetas = np.radians([0.13, -40.12, -0.04, 89.99, -0.01, 39.39, -0.04])
    thetas = [0.6, -0.698132, 0.0, 1.570796, 0.0, 0.698132, 0.0]


    import time
    t = time.time()
    for i in range(1000):
        fk.update_link_transformations(torch.tensor(thetas, dtype=torch.float)) 
        # for T in fk.T_link:
        #     print(T[0:3, 3])
        #     print()

        obstacle  = torch.tensor([0.688, 0.0, 0.29], dtype=torch.float) 
        _, link_id, length = fk.get_closest_on_links(obstacle)
        point = fk.get_point_on_link(link_id, length)
        # print(point)


        j = fk.get_Jacobian(link_id, point)
        # print(j)
    print(time.time() - t)



    # trans = fk.get_transformation(8)
    # print(trans)
    # print(R.from_matrix(trans[0:3, 0:3]).as_quat())

    
    # j = fk.get_Jacobian(8, trans)
    # print(j.T)
    

if __name__ == '__main__':
    main()
        