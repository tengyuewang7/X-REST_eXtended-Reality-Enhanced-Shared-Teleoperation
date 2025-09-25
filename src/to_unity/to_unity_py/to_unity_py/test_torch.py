import numpy as np
import torch
import copy 
from scipy.spatial.transform import Rotation as R


def main(args=None): 
        M_PI = torch.pi
        joint_points = torch.tensor([[0.0, 0.365, 0.0], 
			[0.0, 0.065, -M_PI/2.0],
			[0.0, 0.395, M_PI/2.0],
			[-0.02, 0.055, M_PI/2],
			[-0.02, 0.385, M_PI/2],
			[0.0, 0.1, -M_PI/2],
			[0.1118, 0.0, M_PI/2],
			[0.0, 0.132, 0.0]], dtype=torch.float)
        obstalce = torch.tensor([[0.8354,  0.4677,  0.1615],
                                  [0.0336,  0.0637,  0.0021]], dtype=torch.float)
        vector_1 = joint_points[0:-1] - obstalce.unsqueeze(1)
        vector_2 = joint_points[1:] - obstalce.unsqueeze(1)

        l = torch.sum(vector_1 * (vector_1 - vector_2), dim=2) / (torch.norm(vector_2 - vector_1, dim=2) ** 2)
        l = torch.clamp(l, min=0, max=1) 
        v = (1 - l).unsqueeze(-1) * vector_1 + l.unsqueeze(-1) * vector_2 
        norm_v = torch.norm(v, dim=-1)
        min_index = torch.argmin(norm_v) 

        obstcle_index = min_index // (8 - 1)
        joint_index = min_index % (8 - 1)

        print(norm_v)
        print(obstcle_index)
        print(joint_index)


        if norm_v[obstcle_index, joint_index] > 0.1:
             pass
            # return False, None, None
        else: 
            l_min = l[obstcle_index, joint_index]
            v_min = v[obstcle_index, joint_index]
            z = joint_points[0:joint_index+1]  
            point = v_min + obstalce[obstcle_index]
            t = point - joint_points[0:joint_index+1]

            j = torch.zeros((3, 7), dtype=torch.float) 
            j[:, :joint_index+1] = torch.transpose(torch.cross(z, t, dim=1), 0, 1)
            # return True, v_min.unsqueeze(-1), j
        

if __name__ == '__main__':
    main()
        
