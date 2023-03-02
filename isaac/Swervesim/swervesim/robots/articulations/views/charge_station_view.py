
from typing import Optional

from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.prims import RigidPrimView

import math
import torch

class ChargeStationView(ArticulationView):
    def __init__(
        self,
        prim_paths_expr: str,
        name: Optional[str] = "ChargeStationView",
    ) -> None:
        """[summary]
        """

        super().__init__(
            prim_paths_expr=prim_paths_expr,
            name=name,
            reset_xform_properties=False
        )

        self.chargestation_base = RigidPrimView(prim_paths_expr="/World/envs/.*/ChargeStation/Assembly_1/Part_1", name="base_view", reset_xform_properties=False)
        # self.top = RigidPrimView(prim_paths_expr="/World/envs/.*/Root/Rapid_React_Field", name="field_view", reset_xform_properties=False)
        # self.red_ball_1 = RigidPrimView(prim_paths_expr="/World/envs/.*/Root/Rapid_React_Field/Group_1/Tennis_Ball___Red_08", name="redball[1]", reset_xform_properties=False)
        # self.red_ball_2 = RigidPrimView(prim_paths_expr="/World/envs/.*/Root/Rapid_React_Field/Group_1/Tennis_Ball___Red_09", name="redball[2]", reset_xform_properties=False)
        # self.red_ball_3 = RigidPrimView(prim_paths_expr="/World/envs/.*/Root/Rapid_React_Field/Group_1/Tennis_Ball___Red_10", name="redball[3]", reset_xform_properties=False)
        # self.red_ball_4 = RigidPrimView(prim_paths_expr="/World/envs/.*/Root/Rapid_React_Field/Group_1/Tennis_Ball___Red_11", name="redball[4]", reset_xform_properties=False)
        # self.red_ball_5 = RigidPrimView(prim_paths_expr="/World/envs/.*/Root/Rapid_React_Field/Group_1/Tennis_Ball___Red_12", name="redball[5]", reset_xform_properties=False)
        # self.red_ball_6 = RigidPrimView(prim_paths_expr="/World/envs/.*/Root/Rapid_React_Field/Group_1/Tennis_Ball___Red_13", name="redball[6]", reset_xform_properties=False)
        # self.red_ball_7 = RigidPrimView(prim_paths_expr="/World/envs/.*/Root/Rapid_React_Field/Group_1/Tennis_Ball___Red_14", name="redball[7]", reset_xform_properties=False)
        # self.red_ball_8 = RigidPrimView(prim_paths_expr="/World/envs/.*/Root/Rapid_React_Field/Group_1/Tennis_Ball___Red_15", name="redball[8]", reset_xform_properties=False)
        # self.blue_ball_1 = RigidPrimView(prim_paths_expr="/World/envs/.*/Root/Rapid_React_Field/Group_1/Tennis_Ball___Blue_08", name="blueball[1]", reset_xform_properties=False)
        # self.blue_ball_2 = RigidPrimView(prim_paths_expr="/World/envs/.*/Root/Rapid_React_Field/Group_1/Tennis_Ball___Blue_09", name="blueball[2]", reset_xform_properties=False)
        # self.blue_ball_3 = RigidPrimView(prim_paths_expr="/World/envs/.*/Root/Rapid_React_Field/Group_1/Tennis_Ball___Blue_10", name="blueball[3]", reset_xform_properties=False)
        # self.blue_ball_4 = RigidPrimView(prim_paths_expr="/World/envs/.*/Root/Rapid_React_Field/Group_1/Tennis_Ball___Blue_11", name="blueball[4]", reset_xform_properties=False)
        # self.blue_ball_5 = RigidPrimView(prim_paths_expr="/World/envs/.*/Root/Rapid_React_Field/Group_1/Tennis_Ball___Blue_12", name="blueball[5]", reset_xform_properties=False)
        # self.blue_ball_6 = RigidPrimView(prim_paths_expr="/World/envs/.*/Root/Rapid_React_Field/Group_1/Tennis_Ball___Blue_13", name="blueball[6]", reset_xform_properties=False)
        # self.blue_ball_7 = RigidPrimView(prim_paths_expr="/World/envs/.*/Root/Rapid_React_Field/Group_1/Tennis_Ball___Blue_14", name="blueball[7]", reset_xform_properties=False)
        # self.blue_ball_8 = RigidPrimView(prim_paths_expr="/World/envs/.*/Root/Rapid_React_Field/Group_1/Tennis_Ball___Blue_15", name="blueball[8]", reset_xform_properties=False)
        # self.goal =  RigidPrimView(prim_paths_expr="/World/envs/.*/Root/Rapid_React_Field/Group_1/THE_HUB_GE_22300_01/GE_22434", name="goal[1]", reset_xform_properties=False)
    def if_balanced(self, num_envs, ):
        self.base_pose, self.base_orientation = self.chargestation_base.get_world_poses()
        tolerance = 0.03
        output_roll = torch.zeros(num_envs, 1, dtype=torch.float32).cuda()
        for i in range(len(self.base_orientation)):
            w=self.base_orientation[i][0]
            x=self.base_orientation[i][1]
            y=self.base_orientation[i][2]
            z=self.base_orientation[i][3]
            roll = math.atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z)
            if roll <= tolerance:
                output_roll[i] = 1.0
            else:
                output_roll[i] = 0.0

        print(f"x:{x} y:{y} z:{z} w:{w}")
        return output_roll
        