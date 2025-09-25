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
        self.device = 'cpu'
        self.dof_num = 7 
        self.T_one_by_one = torch.empty((8, 4, 4), dtype=torch.float, device=self.device) 
        self.T_link = torch.empty((8, 4, 4), dtype=torch.float, device=self.device) 
        self.jaco_eff = torch.zeros((6, 7), dtype=torch.float, device=self.device)
        self.pdist = torch.nn.PairwiseDistance(p=2) 
        self.thetas = torch.tensor([0.0] * 7, dtype=torch.float, device=self.device) 
        self.obstacle_points = torch.tensor([[0.7, 0.0, 0.4], 
                                             [0.5, 0.0, 0.4],
                                             [0.2, 0.0, 1.1]], dtype=torch.float, device=self.device)

        link_dict = {"base_link": 0, "link1": 1, "link2": 2, "link3": 3, "link4": 4, "link5": 5, "link6": 6, "link7": 7, "flange": 8}



        # self.obstacle = torch.tensor([[0.6888, -0.01, 0.291], [0.7, 0.01, 0.3], 
        #                             [0.7, 0.02, 0.25], [0.67, -0.03, 0.3], 
        #                             [0.7, 0.02, 0.25], [0.67, -0.03, 0.3], 
        #                             [0.7, 0.02, 0.25], [0.67, -0.03, 0.3], 
        #                             [0.7, 0.02, 0.25], [0.67, -0.03, 0.3], 
        #                             [0.7, 0.02, 0.25], [0.67, -0.03, 0.3], 
        #                             [0.7, 0.02, 0.25], [0.67, -0.03, 0.3], 
        #                             [0.7, 0.02, 0.25], [0.67, -0.03, 0.3], 
        #                             [0.7, 0.02, 0.25], [0.67, -0.03, 0.3], 
        #                             [0.7, 0.02, 0.25], [0.67, -0.03, 0.3], ], dtype=torch.float)

    def update(self, thetas): 
        self.thetas = torch.tensor(thetas, dtype=torch.float, device=self.device).clone().detach()
        self.update_one_by_one_transformations()
        self.update_link_transformations()
        self.update_jaco_eff()

    def update_one_by_one_transformations(self): 
        cos_thetas = torch.cos(self.thetas)
        sin_thetas = torch.sin(self.thetas)

        self.T_one_by_one[0] = torch.tensor([[-cos_thetas[0], sin_thetas[0], 0, 0 ], 
                                             [-sin_thetas[0], -cos_thetas[0], 0, 0 ], 
                                             [0, 0, 1, 0.365 ], 
                                             [0, 0, 0, 1 ]])

        self.T_one_by_one[1] = torch.tensor([[cos_thetas[1], -sin_thetas[1], 0, 0 ], 
                                             [0, 0, 1, 0.065 ], 
                                             [-sin_thetas[1], -cos_thetas[1], 0, 0 ],
                                             [0, 0, 0, 1 ]]) 

        self.T_one_by_one[2] = torch.tensor([[cos_thetas[2], -sin_thetas[2], 0, 0 ], 
                                             [0, 0, -1, -0.395 ], 
                                             [sin_thetas[2], cos_thetas[2], 0, 0 ],
                                             [0, 0, 0, 1 ]]) 

        self.T_one_by_one[3] = torch.tensor([[-cos_thetas[3], sin_thetas[3], 0, -0.02 ], 
                                             [0, 0, -1, -0.055 ], 
                                             [-sin_thetas[3], -cos_thetas[3], 0, 0 ],
                                             [0, 0, 0, 1 ]]) 

        self.T_one_by_one[4] = torch.tensor([[-cos_thetas[4], sin_thetas[4], 0, -0.02 ], 
                                             [0, 0, -1, -0.385 ], 
                                             [-sin_thetas[4], -cos_thetas[4], 0, 0 ], 
                                             [0, 0, 0, 1 ]]) 

        self.T_one_by_one[5] = torch.tensor([[sin_thetas[5], cos_thetas[5], 0, 0 ], 
                                             [0, 0, 1, 0.1 ], 
                                             [cos_thetas[5], -sin_thetas[5], 0, 0 ],
                                             [0, 0, 0, 1 ]]) 

        self.T_one_by_one[6] = torch.tensor([[sin_thetas[6], cos_thetas[6], 0, 0.11], #0.1118 
                                             [0, 0, -1, 0 ], 
                                             [-cos_thetas[6], sin_thetas[6], 0, 0 ],
                                             [0, 0, 0, 1 ]]) 

        self.T_one_by_one[7] = torch.tensor([[0, 1, 0, 0 ], 
                                             [-1, 0, 0, 0 ], 
                                             [0, 0, 1, 0.132 ],
                                             [0, 0, 0, 1 ]]) 
        

    def update_link_transformations(self):
        self.T_link[0] = self.T_one_by_one[0]
        for i in range(1, len(self.T_one_by_one)): 
            self.T_link[i] = self.T_link[i-1] @ self.T_one_by_one[i]
    
    def update_jaco_eff(self):
        z = self.T_link[:-1, 0:3, 2] 
        t = self.T_link[-1, 0:3, 3] - self.T_link[:-1, 0:3, 3] 

        self.jaco_eff[0:3, :] = torch.transpose(torch.cross(z, t, dim=1), 0, 1)
        self.jaco_eff[3:6, :] = torch.transpose(z, 0, 1)

    def closest_to_links(self):
        vector_1 = self.T_link[0:-1, 0:3, 3] - self.obstacle_points.unsqueeze(1)
        vector_2 = self.T_link[1: , 0:3, 3] - self.obstacle_points.unsqueeze(1)

        l = torch.sum(vector_1 * (vector_1 - vector_2), dim=2) / (torch.norm(vector_2 - vector_1, dim=2) ** 2)
        l = torch.clamp(l, min=0, max=1) 
        v = (1 - l).unsqueeze(-1) * vector_1 + l.unsqueeze(-1) * vector_2 
        norm_v = torch.norm(v, dim=-1)
        min_index = torch.argmin(norm_v) 

        obstcle_index = min_index // (self.T_link.shape[0] - 1)
        joint_index = min_index % (self.T_link.shape[0] - 1)

        # print(obstcle_index)
        # print(joint_index)

        if norm_v[obstcle_index, joint_index] > 0.1 * 100:
            return False, None, None
        else: 
            l_min = l[obstcle_index, joint_index]
            v_min = v[obstcle_index, joint_index]
            z = self.T_link[0:joint_index+1, 0:3, 2]  
            # print("z", z)
            point = v_min + self.obstacle_points[obstcle_index]
            t = point - self.T_link[0:joint_index+1, 0:3, 3] 

            # print("t", t)


            j = torch.zeros((3, 7), dtype=torch.float, device=self.device) 
            j[:, :joint_index+1] = torch.transpose(torch.cross(z, t, dim=1), 0, 1)

            return True, v_min.unsqueeze(-1), j

    # def get_Jacobian(self, link_id, point): 
    #     # update link transformations before running this method 
    #     jaco = torch.zeros((6, 7)) 
    #     z = self.T_link[1:link_id, 0:3, 2] 
    #     t = point.unsqueeze(0) - self.T_link[1:link_id, 0:3, 3] 
    #     jaco[0:3, :link_id-1] = torch.transpose(torch.cross(z, t, dim=1), 0, 1)
    #     jaco[3:6, :link_id-1] = torch.transpose(z, 0, 1)

    #     return jaco

    # def get_transformation(self, link_id, length=1): 
    #     # update link transformations before running this method 
    #     # link_id starts from 1. Flange id is 8 with length=1. 
    #     trans = self.T_link[link_id]
    #     if (length != 1): 
    #         v = trans[0:3, 3] - self.T_link[link_id-1][0:3, 3]
    #         trans[0:3, 3] -= v * (1 - length)
    #     return trans 
    
    # def get_point_on_link(self, link_id, length=1):
    #     v = (1 - length) * self.T_link[link_id - 1, 0:3, 3] + length * self.T_link[link_id, 0:3, 3]
    #     return v
    
    # def get_Jacobian(self, link_id, trans): 
    #     # update link transformations before running this method 
        
    #     jaco = torch.zeros((6, 7)) 
    #     t_f = trans[0:3, 3]
    #     for i
    #  in range(link_id-1): 
    #         z =  self.T_link[i+1][0:3, 2]
    #         t = t_f - self.T_link[i+1][0:3, 3]
    #         jaco[0:3, i] = torch.cross(z, t) 
    #         jaco[3:6, i] = z
    #     return jaco 
    
    # def get_closest_on_links(self, points): 
    #     # Find the closest point to the obstacle point on the links, return vextor, link_id, length on the link  
    #     dis = self.pdist(self.T_link[1:, 0:3, 3].unsqueeze(1), points.unsqueeze(0))
    #     min_distance_index = torch.argmin(dis)
    #     point = points[min_distance_index % points.size(0)]

    #     vector_1 = self.T_link[1:-1, 0:3, 3] - point
    #     vector_2 = self.T_link[2:, 0:3, 3] - point

    #     l = torch.sum(vector_1 * (vector_1 - vector_2), dim=1) / (torch.norm(vector_2 - vector_1, dim=1) ** 2)
    #     l = torch.clamp(l, min=0, max=1) 

    #     v = (1 - l).unsqueeze(1) * vector_1 + l.unsqueeze(1) * vector_2 
    #     norm_v = torch.norm(v, dim=1)
    #     min_index = torch.argmin(norm_v)


    #     return v[min_index], min_index+2, l[min_index]
    #     # return min_v, min_link_id, min_l
    
    # def test(self):
    #     a = self.T_link[1:, 0:3, 3]
    #     dis = self.pdist(a.unsqueeze(1), self.obstacle.unsqueeze(0))
    #     print(torch.min(dis))
    #     min_distance_index = torch.argmin(dis)
    #     print(min_distance_index)
    #     print(142 // self.obstacle.size(0))
    #     print(142 % self.obstacle.size(0))




def main(args=None):
    fk = ForwardKinematics()
    thetas = np.radians([0.13, -40.12, -0.04, 89.99, -0.01, 39.39, -0.04])
    thetas = [0.6, -0.698132, 0.0, 1.570796, 0.0, 0.698132, 0.0]
    thetas = [0.0] * 7

    import time
    t1 = time.time()
    for _ in range(1000000):
        fk.update([0.0] * 7) 
        # print(fk.T_one_by_one)
        # print(fk.T_link)
        # print(fk.jaco_eff)
        fk.closest_to_links()
        # print(fk.closest_to_links())
    print(time.time() - t1)

    # import time
    # import random
    # t = time.time()
    # fk.obstacle_points = torch.rand(10, 3, dtype=torch.float, device=fk.device)
    # for i in range(1000):
    #     # thetas = torch.rand(7)
    #     fk.update([0.23] * 7)
    #     fk.closest_to_links()
    # print((time.time() - t))
        

if __name__ == '__main__':
    main()
        
