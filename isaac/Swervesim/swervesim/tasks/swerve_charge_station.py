# Copyright (c) 2018-2022, NVIDIA Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from swervesim.tasks.base.rl_task import RLTask
from swervesim.robots.articulations.swerve import Swerve
from swervesim.robots.articulations.views.swerve_view import SwerveView
from swervesim.robots.articulations.views.charge_station_view import ChargeStationView

from swervesim.tasks.utils.usd_utils import set_drive
from omni.isaac.core.objects import DynamicSphere
from omni.isaac.core.articulations import Articulation


from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.torch.rotations import *
from omni.isaac.core.prims import RigidPrimView
from omni.isaac.core.utils.stage import add_reference_to_stage



import numpy as np
import torch
import math
import itertools


class Swerve_Charge_Station_Task(RLTask):
    def __init__(
        self,
        name,
        sim_config,
        env,
        offset=None
    ) -> None:
        # sets up the sim
        self._sim_config = sim_config
        self._cfg = sim_config.config
        self._task_cfg = sim_config.task_config

        # limits max velocity of wheels and axles
        self.velocity_limit = 10*math.pi

        self.dt = 1 / 10
        self.dt_total = 0.0
        self.max_episode_length_s = self._task_cfg["env"]["episodeLength_s"]
        self._max_episode_length = int(
            self.max_episode_length_s / self.dt)
        self.Kp = self._task_cfg["env"]["control"]["stiffness"]
        self.Kd = self._task_cfg["env"]["control"]["damping"]

        # for key in self.rew_scales.keys():
        #     self.rew_scales[key] *= self.dt

        self._num_envs = self._task_cfg["env"]["numEnvs"]
        self._swerve_translation = torch.tensor([0.0, 0.0, 0.0])
        self._env_spacing = self._task_cfg["env"]["envSpacing"]
        # Number of data points the policy is recieving
        self._num_observations = 37
        # Number of data points the policy is producing
        self._num_actions = 8
        # starting position of the swerve module
        self.swerve_position = torch.tensor([0, 0, 0])
        # starting position of the target
        self._chargestation_part_1_position = torch.tensor([1, 1, 0])


        self.swerve_initia_pos = []
        RLTask.__init__(self, name, env)

        self.target_positions = torch.zeros(
            (self._num_envs, 3), device=self._device, dtype=torch.float32)  # xyx of target position
        self.target_rotation = torch.zeros(
            (self._num_envs, 4), device=self._device, dtype=torch.float32) #ypr
        self.target_positions[:, 1] = 1

        return

    # Adds all of the items to the stage
    def set_up_scene(self, scene) -> None:
        # Adds USD of swerve to stage
        self.get_swerve()
        # Adds ball to stage
        self.get_charge_station()
        super().set_up_scene(scene)
        # Sets up articluation controller for swerve
        self._swerve = SwerveView(
            prim_paths_expr="/World/envs/.*/swerve", name="swerveview")
        self._charge_station= ChargeStationView(prim_paths_expr="/World/envs/.*/ChargeStation", name="ChargeStation_1_view")
        # Allows for position tracking of targets
        # Adds everything to the scene
        scene.add(self._swerve)
        for axle in self._swerve._axle:
            scene.add(axle)
        for wheel in self._swerve._wheel:
            scene.add(wheel)
        scene.add(self._swerve._base)
        scene.add(self._charge_station)
        # print("scene set up")

        return

    def get_swerve(self):
        # Adds swerve to env_0 and adds articulation controller
        swerve = Swerve(self.default_zero_env_path + "/swerve",
                        "swerve", self._swerve_translation)
        self._sim_config.apply_articulation_settings("swerve", get_prim_at_path(
            swerve.prim_path), self._sim_config.parse_actor_config("swerve"))

    def get_charge_station(self):
        chargestation = "/root/edna/isaac/assets/ChargeStation-Copy/Assembly-1.usd"
        add_reference_to_stage(chargestation, self.default_zero_env_path+"/ChargeStation")
        charge_station_1 = Articulation(prim_path=self.default_zero_env_path+"/ChargeStation",name="ChargeStation")
        self._sim_config.apply_articulation_settings("ChargeStation", get_prim_at_path(
            charge_station_1.prim_path), self._sim_config.parse_actor_config("ChargeStation"))

    def get_observations(self) -> dict:
        # Gets various positions and velocties to observations
        self.root_pos, self.root_rot = self._swerve.get_world_poses(clone=False)
        self.joint_velocities = self._swerve.get_joint_velocities()
        self.joint_positions = self._swerve.get_joint_positions()
        self.charge_station_pos, self.charge_station_rot = self._charge_station.get_world_poses(clone=False)
        self.chargestation_vertices = torch.zeros(
            (self._num_envs, 8), device=self._device, dtype=torch.float32) #ypr
        charge_station_pos = self.charge_station_pos - self._env_pos

        for i in range(len(self.charge_station_pos)):
            w=self.charge_station_rot[i][0]
            x=self.charge_station_rot[i][1]
            y=self.charge_station_rot[i][2]
            z=self.charge_station_rot[i][3]
            # print(f"x:{x} y:{y} z:{z} w:{w}")
            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            angle = math.atan2(siny_cosp, cosy_cosp)
            self.chargestation_vertices[i][0] , self.chargestation_vertices[i][1] = findB(charge_station_pos[i][0],charge_station_pos[i][1],angle) 
            self.chargestation_vertices[i][2] , self.chargestation_vertices[i][3] = findB(charge_station_pos[i][0],charge_station_pos[i][1],math.pi-angle) 
            self.chargestation_vertices[i][4] , self.chargestation_vertices[i][5] = findB(charge_station_pos[i][0],charge_station_pos[i][1],angle+math.pi) 
            self.chargestation_vertices[i][6] , self.chargestation_vertices[i][7] = findB(charge_station_pos[i][0],charge_station_pos[i][1],(2*math.pi)-angle)  

        self.root_velocities = self._swerve.get_velocities(clone=False)
        root_positions = self.root_pos - self._env_pos
        root_quats = self.root_rot
        root_linvels = self.root_velocities[:, :3]
        root_angvels = self.root_velocities[:, 3:]
        self.obs_buf[..., 0:3] = (root_positions) / 3
        self.obs_buf[..., 3:7] = root_quats
        self.obs_buf[..., 7:10] = root_linvels / 2
        self.obs_buf[..., 10:13] = root_angvels / math.pi
        self.obs_buf[..., 13:21] = self.joint_velocities
        self.obs_buf[..., 21:29] = self.joint_positions
        self.obs_buf[..., 29:37] = self.chargestation_vertices
        

        # Should not exceed observation ssize declared earlier
        # An observation is created for each swerve in each environment
        observations = {
            self._swerve.name: {
                "obs_buf": self.obs_buf
            }
        }
        return observations

    def pre_physics_step(self, actions) -> None:
        # This is what sets the action for swerve

        reset_env_ids = self.reset_buf.nonzero(as_tuple=False).squeeze(-1)
        if len(reset_env_ids) > 0:
            self.reset_idx(reset_env_ids)

        set_target_ids = (self.progress_buf % 500 == 0).nonzero(
            as_tuple=False).squeeze(-1)
        if len(set_target_ids) > 0:
            self.set_targets(set_target_ids)

        self.actions[:] = actions.clone().to(self._device)
        # Sets velocity for each wheel and axle within the velocity limits
        linear_x_cmd = torch.clamp(
            actions[:, 0:1] * self.velocity_limit, -self.velocity_limit, self.velocity_limit)
        linear_y_cmd = torch.clamp(
            actions[:, 1:2] * self.velocity_limit, -self.velocity_limit, self.velocity_limit)
        angular_cmd = torch.clamp(
            actions[:, 2:3] * self.velocity_limit, -self.velocity_limit, self.velocity_limit)
        x_offset = 0.7366
        radius = 0.1016
        actionlist = []
        for i in range(self.num_envs):
            action = []

        #    Compute Wheel Velocities and Positions
            a = linear_x_cmd[i] - angular_cmd[i] * x_offset / 2

            b = linear_x_cmd[i] + angular_cmd[i] * x_offset / 2

            c = linear_y_cmd[i] - angular_cmd[i] * x_offset / 2

            d = linear_y_cmd[i] + angular_cmd[i] * x_offset / 2

        #   get current wheel positions
            
            front_left_current_pos = (
                (self._swerve.get_joint_positions()[i][0]))
            front_right_current_pos = (
                (self._swerve.get_joint_positions()[i][1]))
            rear_left_current_pos = (
                (self._swerve.get_joint_positions()[i][2]))
            rear_right_current_pos = (
                (self._swerve.get_joint_positions()[i][3]))
 
            front_left_velocity = (
                math.sqrt(math.pow(b, 2) + math.pow(d, 2)))*(1/(radius*math.pi))
            front_right_velocity = (
                math.sqrt(math.pow(b, 2) + math.pow(c, 2)))*(1/(radius*math.pi))
            rear_left_velocity = (
                math.sqrt(math.pow(a, 2) + math.pow(d, 2)))*(1/(radius*math.pi))
            rear_right_velocity = (
                math.sqrt(math.pow(a, 2) + math.pow(c, 2)))*(1/(radius*math.pi))

            front_left_position = math.atan2(b, d)
            front_right_position = math.atan2(b, c)
            rear_left_position = math.atan2(a, d)
            rear_right_position = math.atan2(a, c)


        #   optimization
            
            front_left_position, front_left_velocity = simplifiy_angle(
                front_left_current_pos, front_left_position, front_left_velocity)
            front_right_position, front_right_velocity = simplifiy_angle(
                front_right_current_pos, front_right_position, front_right_velocity)
            rear_left_position, rear_left_velocity = simplifiy_angle(
                rear_left_current_pos, rear_left_position, rear_left_velocity)
            rear_right_position, rear_right_velocity = simplifiy_angle(
                rear_right_current_pos, rear_right_position, rear_right_velocity)

        #   Set Wheel Positions
        #   Has a 1 degree tolerance. Turns clockwise if less than, counter clockwise if greater than
            # if (i == 1):
                # print(f"front_left_position:{front_left_position}")
                # print(f"rear_left_position:{rear_left_position}")
            action.append(calculate_turn_velocity(front_left_current_pos, front_left_position))
            action.append(calculate_turn_velocity(front_right_current_pos, front_right_position))
            action.append(calculate_turn_velocity(rear_left_current_pos, rear_left_position))
            action.append(calculate_turn_velocity(rear_right_current_pos, rear_right_position))
            
            sortlist=[front_left_velocity, front_right_velocity, rear_left_velocity, rear_right_velocity]
            maxs = abs(max(sortlist, key=abs))
            if (maxs < 0.5):
                for num in sortlist:
                    action.append(0.0)
            else:
                for num in sortlist:
                    if (maxs != 0 and abs(maxs) > 10):
                        # scales down velocty to max of 10 radians
                        num = (num/abs(maxs))*10
                        # print(num)
                    action.append(num)
            # print(len(action))
            actionlist.append(action)
        # Sets robots velocities
        self._swerve.set_joint_velocities(torch.FloatTensor(actionlist))

    def reset_idx(self, env_ids):
        # print("line 211")
        # For when the environment resets. This is great for randomization and increases the chances of a successful policy in the real world
        num_resets = len(env_ids)
        # Turns the wheels and axles -pi to pi radians
        self.dof_pos[env_ids, 1] = torch_rand_float(
            -math.pi, math.pi, (num_resets, 1), device=self._device).squeeze()
        self.dof_pos[env_ids, 3] = torch_rand_float(
            -math.pi, math.pi, (num_resets, 1), device=self._device).squeeze()
        self.dof_vel[env_ids, :] = 0

        root_pos = self.initial_root_pos.clone()
        root_pos[env_ids, 0] += torch_rand_float(-0.5, 0.5,
                                                 (num_resets, 1), device=self._device).view(-1)
        root_pos[env_ids, 1] += torch_rand_float(-0.5, 0.5,
                                                 (num_resets, 1), device=self._device).view(-1)
        root_pos[env_ids, 2] += torch_rand_float(
            0, 0, (num_resets, 1), device=self._device).view(-1)
        root_velocities = self.root_velocities.clone()
        root_velocities[env_ids] = 0

        # apply resets
        self._swerve.set_joint_positions(
            self.dof_pos[env_ids], indices=env_ids)
        self._swerve.set_joint_velocities(
            self.dof_vel[env_ids], indices=env_ids)

        self._swerve.set_world_poses(
            root_pos[env_ids], self.initial_root_rot[env_ids].clone(), indices=env_ids)
        self._swerve.set_velocities(root_velocities[env_ids], indices=env_ids)

        # bookkeeping
        self.reset_buf[env_ids] = 0
        self.progress_buf[env_ids] = 0
        # print("line 249")

    def post_reset(self):
        # print("line 252")
        self.root_pos, self.root_rot = self._swerve.get_world_poses()
        self.root_velocities = self._swerve.get_velocities()

        self.dof_pos = self._swerve.get_joint_positions()
        self.dof_vel = self._swerve.get_joint_velocities()

        self.initial_charge_station_pos, self.initial_charge_station_rot = self._charge_station.get_world_poses()
        self.initial_root_pos, self.initial_root_rot = self.root_pos.clone(), self.root_rot.clone()

        # initialize some data used later on
        self.extras = {}
        self.actions = torch.zeros(
            self._num_envs, self.num_actions, dtype=torch.float, device=self._device, requires_grad=False
        )
        self.last_dof_vel = torch.zeros(
            (self._num_envs, 12), dtype=torch.float, device=self._device, requires_grad=False)
        self.last_actions = torch.zeros(
            self._num_envs, self.num_actions, dtype=torch.float, device=self._device, requires_grad=False)

        self.time_out_buf = torch.zeros_like(self.reset_buf)

        # randomize all envs
        indices = torch.arange(
            self._swerve.count, dtype=torch.int64, device=self._device)
        self.reset_idx(indices)

    def set_targets(self, env_ids):
        num_sets = len(env_ids)
        envs_long = env_ids.long()
        # set target position randomly with x, y in (-8, 8)
        self.target_positions[envs_long, 0:2] = torch.rand(
            (num_sets, 2), device=self._device) * 8 - 1
        self.target_rotation[envs_long, 0]= 0
        self.target_rotation[envs_long, 1]= torch.rand((num_sets), device=self._device) 
        # self.target_rotation = self.target_rotation[envs_long]+self.initial_charge_station_rot[envs_long]
        self.target_rotation[envs_long, 2]= 1
        self.target_rotation[envs_long, 3] = 0#
       
        self.target_positions[envs_long,2] = 0
        # print(self.target_positions)

        # shift the target up so it visually aligns better
        charge_station_pos = self.target_positions[envs_long] + self._env_pos[envs_long]
        # print(self.initial_charge_station_rot)
        # print(self.target_rotation)
        self._charge_station.set_world_poses(
            charge_station_pos[:, 0:3], self.target_rotation[envs_long].clone(), indices=env_ids)

    def calculate_metrics(self) -> None:
        # self.dt_total += self.dt
        root_positions = self.root_pos - self._env_pos
        # distance to target
        target_dist = torch.sqrt(torch.square(
            self.target_positions - root_positions).sum(-1))
        charge_station_score = in_charge_station(self.chargestation_vertices,self._swerve.get_axle_positions(self.num_envs))
        # print(f"shape_cs:{charge_station_score.shape}")
        balance_reward = torch.mul(self._charge_station.if_balanced(self.num_envs)[0],charge_station_score[:])*100
        # print(f"shape_balance:{balance_reward.shape}")
        # print(charge_station_score.tolist())
        pos_reward = 1.0 / (1.0 + 2.5 * target_dist * target_dist)
        self.target_dist = target_dist
        self.root_positions = root_positions
        self.root_position_reward = self.rew_buf
        # rewards for moving away form starting point
        for i in range(len(self.root_position_reward)):
            self.root_position_reward[i] = sum(root_positions[i][0:3])

        numerator = (self.root_position_reward*pos_reward + balance_reward)
        # print(f"shape_numerator:{numerator.shape}")
        self.rew_buf[:] = numerator

    def is_done(self) -> None:
        # print("line 312")
        # These are the dying constaints. It dies if it is going in the wrong direction or starts flying
        ones = torch.ones_like(self.reset_buf)
        die = torch.zeros_like(self.reset_buf)
        die = torch.where(self.target_dist > 20.0, ones, die)
        die = torch.where(self.root_positions[..., 2] > 0.5, ones, die)

        # resets due to episode length
        self.reset_buf[:] = torch.where(
            self.progress_buf >= self._max_episode_length - 1, ones, die)
        # print("line 316")


def simplifiy_angle(current_pos, turn_pos, velocity):
    while (abs(current_pos - turn_pos) > math.pi / 2):
        if(turn_pos>current_pos):
            turn_pos -= math.pi
        else:
            turn_pos += math.pi
        velocity *= -1
    return turn_pos, velocity


def calculate_turn_velocity(current_pos, turn_position):
    turningspeed = 5.0
    setspeed = 0.0
    if (current_pos > turn_position+(math.pi/90) or current_pos < turn_position-(math.pi/90)):
        setspeed = abs(turn_position-current_pos)/(math.pi/9)
        if (setspeed > turningspeed):
            setspeed = turningspeed
        if (turn_position < current_pos):
            setspeed *= -1
    return setspeed
def findB(Cx,Cy, angle_change,angle_init=0.463647609,r=1.363107039084):
    L= angle_init*r
    if(angle_change < 0):
        angle_init -= angle_change
    else:
        angle_init += angle_change
    Bx = Cx + r*math.cos(angle_init)
    By = Cy + r*math.sin(angle_init)
    return Bx, By
def in_charge_station(charge_station_verticies,axle_position):
    if_in_chargestation = torch.zeros_like(axle_position)
    # for i in range(len(charge_station_verticies)):
    #     axle_in_chargestation = True
    #     for j in range(len(axle_position)):
    #         if(not check_point(charge_station_verticies[i],axle_position[j])):
    #             axle_in_chargestation = False
    #     if_in_chargestation.add(axle_in_chargestation)
    # if_in_chargestation = [check_point(charge_station_verticies[i],axle_position[j], i) for i in range(len(charge_station_verticies)) for j in range(len(axle_position))]
    

    
    # charge_station_verticies_flat = torch.FloatTensor(sum(charge_station_verticies.tolist(),[]))
    # axle_position_flat = torch.FloatTensor(sum(axle_position.tolist(),[]))
    # for i, j in itertools.product(range(len(charge_station_verticies), 4), range(len(axle_position), 12)):
    #     if_in_chargestation.add(check_point(charge_station_verticies_flat[i:i+4],axle_position_flat[j:j+12]))
    #     print(i, j)
    # for i in range(len(charge_station_verticies)):
    #     if_in_chargestation.add(np.vectorize(check_point)(charge_station_verticies[i].tolist(), axle_position[i].tolist()))
    # if_in_chargestation = torch.from_numpy(if_in_chargestation)

    # for i in range(len(charge_station_verticies)):
    #     if_in_chargestation.add(check_point(charge_station_verticies[i],axle_position[i]))

    if_in_chargestation = [check_point(charge_station_verticies[i],axle_position[i]) for i in range(len(charge_station_verticies))]
    return torch.FloatTensor(if_in_chargestation).cuda()

def check_point(r,m):
    def dot(a, b):
        return a[0]*b[0] + a[1]*b[1]
    
    # print(len(m))
    for i in range(4):
        AB = [r[3]-r[1],r[2]-r[0]]
        AM = [m[3*i]-r[1],m[3*i + 1]-r[0]]
        BC = [r[5]-r[3],r[4]-r[2]]
        BM = [m[3*i]-r[3],m[3*i + 1]-r[2]]
        dotABAM = dot(AB, AM)
        dotABAB = dot(AB, AB)
        dotBCBC = dot(BC, BC)
        dotBCBM = dot(BC, BM)
        out = 0 <= dotABAM and dotABAM <= dotABAB and 0 <= dotBCBM and dotBCBM <= dotBCBC
        if out == False:
            return 0.0
    return 1.0
