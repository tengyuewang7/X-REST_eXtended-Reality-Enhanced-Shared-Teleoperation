from isaacgym import gymapi
from isaacgym import gymutil
from isaacgym import gymtorch
from isaacgym.torch_utils import *

import math
import numpy as np
import torch
import random
import time

from forward_kinematics import ForwardKinematics 


def quat_axis(q, axis=0):
    basis_vec = torch.zeros(q.shape[0], 3, device=q.device)
    basis_vec[:, axis] = 1
    return quat_rotate(q, basis_vec)


def orientation_error(desired, current):
    cc = quat_conjugate(current)
    q_r = quat_mul(desired, cc)
    return q_r[:, 0:3] * torch.sign(q_r[:, 3]).unsqueeze(-1)


def get_random_new_goal():
    goal_pos = torch.tensor([np.random.uniform(0.2, 0.8), 
                         np.random.uniform(-0.3, 0.3), 
                         np.random.uniform(0.2, 0.8)], dtype=torch.float, device='cuda:0').unsqueeze(0)
    # goal_rot = torch.tensor([np.random.uniform(-1, 1), 
    #                         np.random.uniform(-1, 1), 
    #                         np.random.uniform(-1, 1), 
    #                         np.random.uniform(-1, 1)], dtype=torch.float, device='cuda:0').unsqueeze(0)
    # goal_rot /= torch.norm(goal_rot, dim=1, keepdim=True) 
    goal_rot = torch.tensor([0, 1, 0, 0], dtype=torch.float, device='cuda:0').unsqueeze(0)
    return goal_pos, goal_rot

def get_a_new_goal(i, device):
    if (i % 2):
        y = -0.4
    else:
        y = 0.4
    goal_pos = torch.tensor([0.7, y, 0.3], dtype=torch.float, device=device).unsqueeze(0)
    goal_rot = torch.tensor([0.0, 1.0, 0.0, 0.0], dtype=torch.float, device=device).unsqueeze(0)
    return goal_pos, goal_rot


def print_asset_info(gym, asset, name):
    print("======== Asset info %s: ========" % (name))
    num_bodies = gym.get_asset_rigid_body_count(asset)
    num_joints = gym.get_asset_joint_count(asset)
    num_dofs = gym.get_asset_dof_count(asset)
    print("Got %d bodies, %d joints, and %d DOFs" %
          (num_bodies, num_joints, num_dofs))

    # Iterate through bodies
    print("Bodies:")
    for i in range(num_bodies):
        name = gym.get_asset_rigid_body_name(asset, i)
        print(" %2d: '%s'" % (i, name))

    # Iterate through joints
    print("Joints:")
    for i in range(num_joints):
        name = gym.get_asset_joint_name(asset, i)
        type = gym.get_asset_joint_type(asset, i)
        type_name = gym.get_joint_type_string(type)
        print(" %2d: '%s' (%s)" % (i, name, type_name))

    # iterate through degrees of freedom (DOFs)
    print("DOFs:")
    for i in range(num_dofs):
        name = gym.get_asset_dof_name(asset, i)
        type = gym.get_asset_dof_type(asset, i)
        type_name = gym.get_dof_type_string(type)
        print(" %2d: '%s' (%s)" % (i, name, type_name))
    print("=================================")

# set random seed
np.random.seed(42)

torch.set_printoptions(precision=4, sci_mode=False)

class RizonIK():
    def __init__(self):

        # IK params
        self.damping = 0.05

        # acquire gym interface
        self.gym = gymapi.acquire_gym()

        # parse arguments
        # Add custom arguments
        custom_parameters = [{"name": "--num_envs", "type": int, "default": 1, "help": "Number of environments to create"},]
        args = gymutil.parse_arguments(
            description="Rizon Jacobian Inverse Kinematics (IK)",
            custom_parameters=custom_parameters,
        )

        # set torch device
        self.device = args.sim_device if args.use_gpu_pipeline else 'cpu'

        # configure sim
        sim_params = gymapi.SimParams()
        sim_params.up_axis = gymapi.UP_AXIS_Z
        sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.8)
        sim_params.dt = 1.0 / 100.0
        sim_params.substeps = 2
        sim_params.use_gpu_pipeline = args.use_gpu_pipeline

        if args.physics_engine == gymapi.SIM_PHYSX:
            sim_params.physx.solver_type = 1
            sim_params.physx.num_position_iterations = 8
            sim_params.physx.num_velocity_iterations = 4
            sim_params.physx.rest_offset = 0.0
            sim_params.physx.contact_offset = 0.001
            sim_params.physx.friction_offset_threshold = 0.001
            sim_params.physx.friction_correlation_distance = 0.0005
            sim_params.physx.num_threads = args.num_threads
            sim_params.physx.use_gpu = args.use_gpu
        else:
            raise Exception("This example can only be used with PhysX")

        # create sim
        self.sim = self.gym.create_sim(args.compute_device_id, args.graphics_device_id, args.physics_engine, sim_params)
        if self.sim is None:
            raise Exception("Failed to create sim")

        asset_root = "/home/rvc/isaacgym/assets"

        # load rizon asset
        rizon_asset_file = "urdf/rizon4_description_modified/robots/flexiv_rizon4_kinematics.urdf"
        asset_options = gymapi.AssetOptions()
        asset_options.armature = 0.01
        asset_options.fix_base_link = True
        asset_options.disable_gravity = True
        asset_options.flip_visual_attachments = False
        self.rizon_asset = self.gym.load_asset(self.sim, asset_root, rizon_asset_file, asset_options)

        # configure rizon dofs
        self.rizon_dof_props = self.gym.get_asset_dof_properties(self.rizon_asset)
        self.rizon_dof_props["driveMode"].fill(gymapi.DOF_MODE_POS)
        self.rizon_dof_props["stiffness"].fill(400.0)
        self.rizon_dof_props["damping"].fill(40.0)

        # default dof states and position targets
        rizon_num_dofs = self.gym.get_asset_dof_count(self.rizon_asset)
        self.default_dof_pos = np.array([-0.5, -0.698132, 0.0, 1.570796, 0.0, 0.698132, 0.0], dtype=np.float32)

        # self.default_dof_pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)


        self.default_dof_state = np.zeros(rizon_num_dofs, gymapi.DofState.dtype)
        self.default_dof_state["pos"] = self.default_dof_pos

        # send to torch
        default_dof_pos_tensor = to_torch(self.default_dof_pos, device=self.device)

        print_asset_info(self.gym, self.rizon_asset, "rizon")

        # create sphere asset
        asset_options = gymapi.AssetOptions()
        asset_options.fix_base_link = True
        self.sphere_asset = self.gym.create_sphere(self.sim, 0.02, asset_options)

        # add ground plane
        plane_params = gymapi.PlaneParams()
        plane_params.normal = gymapi.Vec3(0, 0, 1)
        self.gym.add_ground(self.sim, plane_params)

        self.num_envs = args.num_envs
        self.envs = [] 
        self.rizon_handles = []
        self.flange_idxs = []
        self.joint_idxs = []

        self.create_envs()

        self.start_time = time.time()
        self.obstacle_points = torch.tensor([[0.7, 0.0, 0.4], 
                                             [0.5, 0.0, 0.4]], 
                                             dtype=torch.float, device=self.device)
        self.obstacle_points = torch.tensor([[1000, 0.0, 0.4]], 
                                            dtype=torch.float, device=self.device)
        self.loop_count = 0

        self.fk = ForwardKinematics()
        self.fk.obstacle_points = self.obstacle_points

        self.saved_data = torch.empty((0, 13), device=self.device)
        self.vel_data = torch.empty((0, 7))
        self.u_data = torch.empty((0, 8))

    def create_envs(self):

        # configure env grid
        spacing = 1.0
        env_lower = gymapi.Vec3(-spacing, -spacing, 0.0)
        env_upper = gymapi.Vec3(spacing, spacing, spacing)
        num_per_row = int(math.sqrt(self.num_envs))

        for i in range(self.num_envs):
            # create env
            env = self.gym.create_env(self.sim, env_lower, env_upper, num_per_row)
            self.envs.append(env)

            # # add sphere
            # sphere_pose = gymapi.Transform()
            # sphere_pose.p.x = 0.7
            # sphere_pose.p.y = 0.0
            # sphere_pose.p.z = 0.4 
            # sphere_handle = self.gym.create_actor(env, self.sphere_asset, sphere_pose, "sphere_1", i, 2)
            # color = gymapi.Vec3(0, 0, 1)
            # self.gym.set_rigid_body_color(env, sphere_handle, 0, gymapi.MESH_VISUAL_AND_COLLISION, color)
            # sphere_pose.p.x = 0.5 
            # sphere_handle = self.gym.create_actor(env, self.sphere_asset, sphere_pose, "sphere_2", i, 2)
            # self.gym.set_rigid_body_color(env, sphere_handle, 0, gymapi.MESH_VISUAL_AND_COLLISION, color)

            # add rizon
            rizon_pose = gymapi.Transform()
            rizon_pose.p = gymapi.Vec3(0, 0, 0)
            rizon_handle = self.gym.create_actor(env, self.rizon_asset, rizon_pose, "rizon", i, 0)
            self.rizon_handles.append(rizon_handle)

            # set dof properties
            self.gym.set_actor_dof_properties(env, rizon_handle, self.rizon_dof_props)

            # set initial dof states
            self.gym.set_actor_dof_states(env, rizon_handle, self.default_dof_state, gymapi.STATE_ALL)

            # set initial position targets
            self.gym.set_actor_dof_position_targets(env, rizon_handle, self.default_dof_pos)

            # get link index of rizon flange, which we will use as end effector
            rizon_link_dict = self.gym.get_asset_rigid_body_dict(self.rizon_asset)
            rizon_flange_index = rizon_link_dict["flange"]

            rizon_joint_dict = self.gym.get_asset_joint_dict(self.rizon_asset)

        # create viewer
        self.viewer = self.gym.create_viewer(self.sim, gymapi.CameraProperties())
        if self.viewer is None:
            raise Exception("Failed to create viewer")
        
        # point camera at middle env
        cam_pos = gymapi.Vec3(4, 3, 2)
        cam_target = gymapi.Vec3(-4, -3, 0)
        middle_env = self.envs[self.num_envs // 2 + num_per_row // 2]
        self.gym.viewer_camera_look_at(self.viewer, middle_env, cam_pos, cam_target)

        # ==== prepare tensors =====
        # from now on, we will use the tensor API that can run on CPU or GPU
        self.gym.prepare_sim(self.sim)

                           
        # get global index of flange in rigid body state tensor
        link1_idx = self.gym.find_actor_rigid_body_index(self.envs[0], rizon_handle, "link1", gymapi.DOMAIN_SIM) 
        link2_idx = self.gym.find_actor_rigid_body_index(self.envs[0], rizon_handle, "link2", gymapi.DOMAIN_SIM) 
        link3_idx = self.gym.find_actor_rigid_body_index(self.envs[0], rizon_handle, "link3", gymapi.DOMAIN_SIM) 
        link4_idx = self.gym.find_actor_rigid_body_index(self.envs[0], rizon_handle, "link4", gymapi.DOMAIN_SIM) 
        link5_idx = self.gym.find_actor_rigid_body_index(self.envs[0], rizon_handle, "link5", gymapi.DOMAIN_SIM) 
        link6_idx = self.gym.find_actor_rigid_body_index(self.envs[0], rizon_handle, "link6", gymapi.DOMAIN_SIM) 
        link7_idx = self.gym.find_actor_rigid_body_index(self.envs[0], rizon_handle, "link7", gymapi.DOMAIN_SIM) 
        flange_idx = self.gym.find_actor_rigid_body_index(self.envs[0], rizon_handle, "flange", gymapi.DOMAIN_SIM) 

        self.joint_idxs = [link1_idx, link2_idx, link3_idx, link4_idx, link5_idx, link6_idx, link7_idx, flange_idx]
        
        # get jacobian tensor
        # for fixed-base rizon, tensor has shape (num envs, 8, 6, 7), 8 joints; 6 = 3(pos) + 3(rot); 7 DOFs(motors)
        _jacobian = self.gym.acquire_jacobian_tensor(self.sim, "rizon")
        self.jacobian = gymtorch.wrap_tensor(_jacobian)[0] # shape: [8, 6, 7], joint 1-7 + flange

        # jacobian entries corresponding to rizon flange
        self.j_eef = self.jacobian[-1, :, :]  # last one is the end-effector, shape: [6, 7]

        # get rigid body state tensor
        # position([0:3]), rotation([3:7]), linear velocity([7:10]), and angular velocity([10:13])
        _rb_states = self.gym.acquire_rigid_body_state_tensor(self.sim)  
        self.rb_states = gymtorch.wrap_tensor(_rb_states) # shape: [11, 13]
        self.joint_points = self.rb_states[self.joint_idxs[0]:self.joint_idxs[-1]+1, :3]

        # get dof state tensor
        _dof_states = self.gym.acquire_dof_state_tensor(self.sim)
        dof_states = gymtorch.wrap_tensor(_dof_states)
        self.dof_pos = dof_states[:, 0].view(7, 1)  # shape: [7, 1]
        self.dof_vel = dof_states[:, 1].view(7, 1)  # shape: [7, 1]

        # Set action tensors
        self.pos_action = torch.zeros_like(self.dof_pos).view(self.num_envs, 7)

        self.i = 0
        self.goal_pos, self.goal_rot = self.get_a_new_goal()
        self.i += 1

    def update(self): 

        # step the physics
        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)

        # refresh tensors
        self.gym.refresh_rigid_body_state_tensor(self.sim)
        self.gym.refresh_dof_state_tensor(self.sim)
        self.gym.refresh_jacobian_tensors(self.sim)

        flange_pos = self.rb_states[self.joint_idxs[-1], :3]
        flange_rot = self.rb_states[self.joint_idxs[-1], 3:7]

        self.goal_pos =  torch.tensor([0.6, -0.11, 0.291], dtype=torch.float, device=self.device).unsqueeze(0)
        t = time.time()- self.start_time
        self.goal_pos[0, 0] = (0.1) * np.cos(1.0 * t) + (0.6) 
        self.goal_pos[0, 1] = (0.1) * np.sin(1.0 * t) + (-0.11) 
        self.goal_rot = torch.tensor([0.0, 1.0, 0.0, 0.0], dtype=torch.float, device=self.device).unsqueeze(0)

        # compute position and orientation error
        pos_err = (self.goal_pos - flange_pos).view(1, 3)
        orn_err = orientation_error(self.goal_rot, flange_rot.unsqueeze(0))
        dpose = torch.cat([pos_err, orn_err], -1).squeeze(0).unsqueeze(-1)  # shape: [6, 1]

        self.save_data(dpose)

        # if (torch.norm(dpose) < 0.01):
        #     self.goal_pos, self.goal_rot = self.get_a_new_goal()
        #     self.i += 1
        #     print("Arrive")

        # u = self.my_control_2(dpose)
        # time1 = time.time()
        # for i in range(10000):
        u = self.controller(dpose)
        # print((time.time() - time1)/10000)
        
        self.pos_action[0] = (self.dof_pos + u / 1.0).squeeze(-1)

        # self.loop_count += 1
        # if (self.loop_count == 100):
        #     print(time.time() - self.start_time)
        #     self.start_time = time.time()
        #     self.loop_count = 0

        # Deploy actions
        self.gym.set_dof_position_target_tensor(self.sim, gymtorch.unwrap_tensor(self.pos_action))
        
        # self.gym.set_dof_velocity_target_tensor(self.sim, gymtorch.unwrap_tensor(u))

        # update viewer
        self.gym.step_graphics(self.sim)
        self.gym.draw_viewer(self.viewer, self.sim, False)
        self.gym.sync_frame_time(self.sim)

    def controller(self, dpose):
        # solve damped least squares

        j_eef_T = torch.transpose(self.j_eef, 0, 1)
        lmbda = torch.eye(6, device=self.device) * (self.damping ** 2) 
        is_close, v_min, j_0 = self.closest_to_links() 
        u3 = 0.0
        if (is_close): 
            v_min = v_min * ((1.0 / torch.norm(v_min) - 0.0 /0.1)** 2)
            j_0_T = torch.transpose(j_0, 0, 1)
            u3 = j_0_T @ v_min / 10.0

        u1 = j_eef_T @ torch.inverse(self.j_eef @ j_eef_T + lmbda) @ dpose  

        u = u1 + u3


        if (torch.abs(torch.max(u)) > 0.2):
            u = u / torch.abs(torch.max(u)) * 0.2
        # print(u1) 
        # print(u3)
        return u

    def closest_to_links(self):
        vector_1 = self.joint_points[0:-1] - self.obstacle_points.unsqueeze(1)
        vector_2 = self.joint_points[1:] - self.obstacle_points.unsqueeze(1)

        l = torch.sum(vector_1 * (vector_1 - vector_2), dim=2) / (torch.norm(vector_2 - vector_1, dim=2) ** 2)
        l = torch.clamp(l, min=0, max=1) 
        v = (1 - l).unsqueeze(-1) * vector_1 + l.unsqueeze(-1) * vector_2 
        norm_v = torch.norm(v, dim=-1)
        min_index = torch.argmin(norm_v) 
        obstcle_index = min_index // (self.joint_points.shape[0] - 1)
        joint_index = min_index % (self.joint_points.shape[0] - 1)

        if norm_v[obstcle_index, joint_index] > 0.1:
            return False, None, None
        else: 
            l_min = l[obstcle_index, joint_index]
            v_min = v[obstcle_index, joint_index]
            j = (1 - l_min) * self.jacobian[joint_index][0:3] + l_min * self.jacobian[joint_index+1][0:3]

            return True, v_min.unsqueeze(-1), j

    def save_data(self, dpose):

        new_tensor = torch.cat((torch.tensor([time.time() - self.start_time], dtype=torch.float, device=self.device), dpose.squeeze(-1)))
        new_tensor = torch.cat((new_tensor, self.goal_pos[0, 0].unsqueeze(0)))

        print(new_tensor)
        print(self.rb_states[self.joint_idxs[-1], 0].unsqueeze(0))


        new_tensor = torch.cat((new_tensor, self.rb_states[self.joint_idxs[-1], 0].unsqueeze(0))) 
        new_tensor = torch.cat((new_tensor, self.goal_pos[0, 1].unsqueeze(0)))
        new_tensor = torch.cat((new_tensor, self.rb_states[self.joint_idxs[-1], 1].unsqueeze(0))) 
        new_tensor = torch.cat((new_tensor, self.goal_pos[0, 2].unsqueeze(0)))
        new_tensor = torch.cat((new_tensor, self.rb_states[self.joint_idxs[-1], 2].unsqueeze(0))) 
        self.saved_data = torch.cat((self.saved_data, new_tensor.unsqueeze(0)), 0)
        # vel = torch.tensor(self.current_dq, dtype=torch.float).unsqueeze(0)
        # self.vel_data = torch.cat((self.vel_data, vel), 0)

        # u = self.control_ik(dpose, is_slow=is_slow, good_rot=good_rot) 
        # if (is_stopped):
        #     u = torch.zeros(u.shape)

        # u_data = torch.empty((1, 8))
        # u_data[0, :7] = u
        # u_data[0, 7] = self.th
        # # u_data = torch.cat((u.unsqueeze(0), torch.tensor(self.th, dtype=torch.float)).unsqueeze(0), -1)
        # self.u_data = torch.cat((self.u_data, u_data), 0)
    
    def get_a_new_goal(self):
        if (self.i % 2):
            y = -0.4
        else:
            y = 0.4
        goal_pos = torch.tensor([0.7, y, 0.3], dtype=torch.float, device=self.device).unsqueeze(0)
        goal_rot = torch.tensor([0.0, 1.0, 0.0, 0.0], dtype=torch.float, device=self.device).unsqueeze(0)
        return goal_pos, goal_rot
    
    def on_shutdown(self):

        torch.save(self.saved_data, 'issac_pos.pt')

        print("On shutdown ... ")


def main(): 
    try: 
        sim = RizonIK() 

        while not sim.gym.query_viewer_has_closed(sim.viewer): 

            sim.update()

    finally:
        sim.on_shutdown()
        print("Done")
        sim.gym.destroy_viewer(sim.viewer) 
        sim.gym.destroy_sim(sim.sim)


if __name__ == '__main__': 
    main()
