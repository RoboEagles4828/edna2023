from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.graph.core as og
import omni.usd
from omni.isaac.edna.base_sample import BaseSample
from omni.isaac.urdf import _urdf
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils import prims
from omni.isaac.core.prims import GeometryPrim
from omni.isaac.core_nodes.scripts.utils import set_target_prims
# from omni.kit.viewport_legacy import get_default_viewport_window
# from omni.isaac.sensor import IMUSensor
from pxr import UsdPhysics, UsdShade, Sdf, Gf
import omni.kit.commands
import os
import numpy as np
import math
from random import randint, choice
import carb

NAMESPACE = f"{os.environ.get('ROS_NAMESPACE')}" if 'ROS_NAMESPACE' in os.environ else 'default'

def set_drive_params(drive, stiffness, damping, max_force):
    drive.GetStiffnessAttr().Set(stiffness)
    drive.GetDampingAttr().Set(damping)
    if(max_force != 0.0):
        drive.GetMaxForceAttr().Set(max_force)
    return

def add_physics_material_to_prim(prim, materialPath):
    bindingAPI = UsdShade.MaterialBindingAPI.Apply(prim)
    materialPrim = UsdShade.Material(materialPath)
    bindingAPI.Bind(materialPrim, UsdShade.Tokens.weakerThanDescendants, "physics")

class ImportBot(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self.game_piece_list = []

        return


    def set_friction(self, robot_prim_path):
        mtl_created_list = []
        # Create a new material using OmniGlass.mdl
        omni.kit.commands.execute(
            "CreateAndBindMdlMaterialFromLibrary",
            mdl_name="OmniPBR.mdl",
            mtl_name="Rubber",
            mtl_created_list=mtl_created_list,
        )
        # Get reference to created material
        stage = omni.usd.get_context().get_stage()
        mtl_prim = stage.GetPrimAtPath(mtl_created_list[0])

        friction_material = UsdPhysics.MaterialAPI.Apply(mtl_prim)
        friction_material.CreateDynamicFrictionAttr(1.0)
        friction_material.CreateStaticFrictionAttr(1.0)

        front_left_wheel_prim = stage.GetPrimAtPath(f"{robot_prim_path}/front_left_wheel_link/collisions")
        front_right_wheel_prim = stage.GetPrimAtPath(f"{robot_prim_path}/front_right_wheel_link/collisions")
        rear_left_wheel_prim = stage.GetPrimAtPath(f"{robot_prim_path}/rear_left_wheel_link/collisions")
        rear_right_wheel_prim = stage.GetPrimAtPath(f"{robot_prim_path}/rear_right_wheel_link/collisions")

        add_physics_material_to_prim(front_left_wheel_prim, mtl_prim)
        add_physics_material_to_prim(front_right_wheel_prim, mtl_prim)
        add_physics_material_to_prim(rear_left_wheel_prim, mtl_prim)
        add_physics_material_to_prim(rear_right_wheel_prim, mtl_prim)

    def setup_scene(self):
        world = self.get_world()
        world.get_physics_context().enable_gpu_dynamics(False)
        world.scene.add_default_ground_plane()
        self.setup_field()
        # self.setup_perspective_cam()
        self.setup_world_action_graph()
        return
    def add_game_piece(self):
        print("asdklj;f;asdjdfdl;asdjfkl;asdjfkl;asdjfkl;asfjkla;sdjkflka;k")
        self.extension_path = os.path.abspath(__file__)
        self.project_root_path = os.path.abspath(os.path.join(self.extension_path, "../../../../../../.."))
        cone = os.path.join(self.project_root_path, "assets/2023_field_cpu/parts/GE-23700_JFH.usd")
        cube = os.path.join(self.project_root_path, "assets/2023_field_cpu/parts/GE-23701_JFL.usd")
        add_reference_to_stage(cone, "/World/Cone_1")
        substation_empty = [True, True, True, True]
        for game_piece in self.game_piece_list:
            pose, orienation = game_piece.get_world_pose()
            print(pose)
            if(pose[0]<8.17+0.25 and pose[0]>8.17-0.25):
                if(pose[1]<-2.65+0.25 and pose[1]>-2.65-0.25):
                    substation_empty[0]=False
                    print("substation_empty[0]=False")
                elif pose[1]<-3.65+0.25 and pose[1]>-3.65-0.25:
                    substation_empty[1]=False
                    print("substation_empty[1]=False")

            elif (pose[0]<-8.17+0.25 and pose[0]>-8.17-0.25):
                if(pose[1]<-2.65+0.25 and pose[1]>-2.65-0.25):
                    substation_empty[2]=False
                    print("substation_empty[2]=False")

                elif pose[1]<-3.65+0.25 and pose[1]>-3.65-0.25:
                    substation_empty[3]=False
                    print("substation_empty[3]=False")
        for i in range(4):
            if substation_empty[i]:
                num = (int(len(self.game_piece_list)+1)/2)
                if(i==0):
                    position = [8.17, -2.65, 0.0]
                elif(i==1):
                    position = [-8.17, -2.65, 0.0]
                elif(i==2):
                    position = [8.17, -3.65, 0.0]
                else:
                    position = [-8.17, -3.65, 0.0]
                if choice([True, False]):
                    print("yes")
                    name = "/World/Cube_"+str(num)
                    view = "cube_"+str(num)+"_view"
                    add_reference_to_stage(cube, "/World/Cube_"+str(num))
                    self.game_piece_list.append(GeometryPrim(name, view, position=position))
                else:
                    print("yesss")
                    name = "/World/Cone_"+str(num)
                    view = "cone_"+str(num)+"_view" 
                    add_reference_to_stage(cone, "/World/Cone_"+str((int(len(self.game_piece_list)+1)/2)))
                    self.game_piece_list.append(GeometryPrim(name, view, position=position))

            

            

        return
   
    def setup_field(self):
        world = self.get_world()
        self.extension_path = os.path.abspath(__file__)
        self.project_root_path = os.path.abspath(os.path.join(self.extension_path, "../../../../../../.."))
        field = os.path.join(self.project_root_path, "assets/2023_field_cpu/FE-2023.usd")
        add_reference_to_stage(usd_path=field,prim_path="/World/Field")
        cone = os.path.join(self.project_root_path, "assets/2023_field_cpu/parts/GE-23700_JFH.usd")
        cube = os.path.join(self.project_root_path, "assets/2023_field_cpu/parts/GE-23701_JFL.usd")
        chargestation = os.path.join(self.project_root_path, "assets/ChargeStation-Copy/Assembly-1.usd")
        substation = os.path.join(self.project_root_path, "assets/te-23060/te_23060.usd")
        add_reference_to_stage(chargestation, "/World/ChargeStation_1")
        add_reference_to_stage(chargestation, "/World/ChargeStation_2") 
        add_reference_to_stage(substation, "/World/Substation_1")
        add_reference_to_stage(substation, "/World/Substation_2")
        add_reference_to_stage(substation, "/World/Substation_3")
        add_reference_to_stage(substation, "/World/Substation_4")
        add_reference_to_stage(cone, "/World/Cone_1")
        add_reference_to_stage(cone, "/World/Cone_2")
        add_reference_to_stage(cone, "/World/Cone_3")
        add_reference_to_stage(cone, "/World/Cone_4")
        add_reference_to_stage(cone, "/World/Cone_5")
        add_reference_to_stage(cone, "/World/Cone_6")
        # add_reference_to_stage(cone, "/World/Cone_7")
        # add_reference_to_stage(cone,  "/World/Cone_8")
        self.game_piece_list.append( GeometryPrim("/World/Cone_1","cone_1_view",position=np.array([1.20298,-0.56861,0.0])))
        self.game_piece_list.append( GeometryPrim("/World/Cone_2","cone_2_view",position=np.array([1.20298,3.08899,0.0])))
        self.game_piece_list.append( GeometryPrim("/World/Cone_3","cone_3_view",position=np.array([-1.20298,-0.56861,0.0])))
        self.game_piece_list.append( GeometryPrim("/World/Cone_4","cone_4_view",position=np.array([-1.20298,3.08899,0.0])))
        self.game_piece_list.append( GeometryPrim("/World/Cone_5","cone_5_view",position=np.array([-8.17,-2.65,1.15])))
        self.game_piece_list.append( GeometryPrim("/World/Cone_6","cone_6_view",position=np.array([8.17,-2.65,1.15])))
        chargestation_1 = GeometryPrim("/World/ChargeStation_1","chargestation_1_view",position=np.array([-4.20298,-0.56861,0.0]))
        chargestation_2 = GeometryPrim("/World/ChargeStation_2","chargesation_2_view",position=np.array([4.20298,0.56861,0.0]))
        substation_1 = GeometryPrim("/World/Substation_1","substation_1_view",position=np.array([-8,-3.4,0.0]))
        substation_2 = GeometryPrim("/World/Substation_2","substation_2_view",position=np.array([8,-3.4,0.0]))
        substation_3 = GeometryPrim("/World/Substation_3","substation_3_view",position=np.array([-8,-2.2,0.0]))
        substation_4 = GeometryPrim("/World/Substation_4","substation_4_view",position=np.array([8,-2.2,0.0]))


        add_reference_to_stage(cube, "/World/Cube_1")
        add_reference_to_stage(cube, "/World/Cube_2")
        add_reference_to_stage(cube, "/World/Cube_3")
        add_reference_to_stage(cube, "/World/Cube_4")
        add_reference_to_stage(cube, "/World/Cube_5")
        add_reference_to_stage(cube, "/World/Cube_6")
        # add_reference_to_stage(cube, "/World/Cube_7")
        # add_reference_to_stage(cube, "/World/Cube_8")
        self.game_piece_list.append( GeometryPrim("/World/Cube_1","cube_1_view",position=np.array([1.20298,0.65059,0.121])))
        self.game_piece_list.append( GeometryPrim("/World/Cube_2","cube_2_view",position=np.array([1.20298,1.86979,0.121])))
        self.game_piece_list.append( GeometryPrim("/World/Cube_3","cube_3_view",position=np.array([-1.20298,0.65059,0.121])))
        self.game_piece_list.append( GeometryPrim("/World/Cube_4","cube_4_view",position=np.array([-1.20298,1.86979,0.121])))
        self.game_piece_list.append( GeometryPrim("/World/Cube_5","cube_5_view",position=np.array([-8.17,-3.65,1.15])))
        self.game_piece_list.append( GeometryPrim("/World/Cube_6","cube_6_view",position=np.array([8.17,-3.65,1.15])))


    async def setup_post_load(self):
        self._world = self.get_world()
        # self._world.get_physics_context().enable_gpu_dynamics(True)
        self.robot_name = "edna"
        self.extension_path = os.path.abspath(__file__)
        self.project_root_path = os.path.abspath(os.path.join(self.extension_path, "../../../../../../.."))
        self.path_to_urdf = os.path.join(self.extension_path, "../../../../../../../..", "src/edna_description/urdf/edna.urdf")
        carb.log_info(self.path_to_urdf)

        self._robot_prim_path = self.import_robot(self.path_to_urdf)


        if self._robot_prim_path is None:
            print("Error: failed to import robot")
            return
        
        self._robot_prim = self._world.scene.add(
            Robot(prim_path=self._robot_prim_path, name=self.robot_name, position=np.array([0.0, 0.0, 0.3]), orientation=np.array([0.0, 0.0, 0.0, 1.0]))
        )
        
        self.configure_robot(self._robot_prim_path)
        return
    
    def import_robot(self, urdf_path):
        import_config = _urdf.ImportConfig()
        import_config.merge_fixed_joints = False
        import_config.fix_base = False
        import_config.make_default_prim = True
        import_config.self_collision = False
        import_config.create_physics_scene = False
        import_config.import_inertia_tensor = True
        import_config.default_drive_strength = 1047.19751
        import_config.default_position_drive_damping = 52.35988
        import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY
        import_config.distance_scale = 1.0
        import_config.density = 0.0
        result, prim_path = omni.kit.commands.execute( "URDFParseAndImportFile", 
            urdf_path=urdf_path,
            import_config=import_config)

        if result:
            return prim_path
        return None

    
    def configure_robot(self, robot_prim_path):
        w_sides = ['left', 'right']
        l_sides = ['front', 'back']
        stage = self._world.stage
        chassis_name = f"swerve_chassis_link"

       
        front_left_axle = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/{chassis_name}/front_left_axle_joint"), "angular")
        front_right_axle = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/{chassis_name}/front_right_axle_joint"), "angular")
        rear_left_axle = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/{chassis_name}/rear_left_axle_joint"), "angular")
        rear_right_axle = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/{chassis_name}/rear_right_axle_joint"), "angular")
        front_left_wheel = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/front_left_axle_link/front_left_wheel_joint"), "angular")
        front_right_wheel = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/front_right_axle_link/front_right_wheel_joint"), "angular")
        rear_left_wheel = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/rear_left_axle_link/rear_left_wheel_joint"), "angular")
        rear_right_wheel = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/rear_right_axle_link/rear_right_wheel_joint"), "angular")
        arm_roller_bar_joint = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/arm_elevator_leg_link/arm_roller_bar_joint"), "linear")
        elevator_center_joint = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/elevator_outer_1_link/elevator_center_joint"), "linear")
        elevator_outer_2_joint = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/elevator_center_link/elevator_outer_2_joint"), "linear")
        elevator_outer_1_joint = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/arm_back_leg_link/elevator_outer_1_joint"), "angular")
        top_gripper_left_arm_joint = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/top_gripper_bar_link/top_gripper_left_arm_joint"), "angular")
        top_gripper_right_arm_joint = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/top_gripper_bar_link/top_gripper_right_arm_joint"), "angular")
        top_slider_joint = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/elevator_outer_2_link/top_slider_joint"), "linear")
        bottom_intake_joint = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/arm_elevator_leg_link/bottom_intake_joint"), "angular")
        
        set_drive_params(front_left_axle, 1, 1000, 98.0)
        set_drive_params(front_right_axle, 1, 1000, 98.0)
        set_drive_params(rear_left_axle, 1, 1000, 98.0)
        set_drive_params(rear_right_axle, 1, 1000, 98.0)       
        set_drive_params(front_left_wheel, 1, 1000, 98.0)
        set_drive_params(front_right_wheel, 1, 1000, 98.0)
        set_drive_params(rear_left_wheel, 1, 1000, 98.0)
        set_drive_params(rear_right_wheel, 1, 1000, 98.0)
        set_drive_params(arm_roller_bar_joint, 10000000, 100000, 98.0)
        set_drive_params(elevator_center_joint, 10000000, 100000, 98.0)
        set_drive_params(elevator_outer_1_joint, 10000000, 100000, 2000.0)
        set_drive_params(elevator_outer_2_joint, 10000000, 100000, 98.0)
        set_drive_params(top_gripper_left_arm_joint, 10000000, 100000, 98.0)
        set_drive_params(top_gripper_right_arm_joint, 10000000, 100000, 98.0)
        set_drive_params(top_slider_joint, 10000000, 100000, 98.0)
        set_drive_params(bottom_intake_joint, 10000000, 100000, 98.0)
        
        # self.create_lidar(robot_prim_path)
        self.create_imu(robot_prim_path)
        self.create_depth_camera(robot_prim_path)
        self.setup_camera_action_graph(robot_prim_path)
        self.setup_imu_action_graph(robot_prim_path)
        self.setup_robot_action_graph(robot_prim_path)  
        self.set_friction(robot_prim_path)
        return

    def create_lidar(self, robot_prim_path):
        lidar_parent = f"{robot_prim_path}/lidar_link"
        lidar_path = "/lidar"
        self.lidar_prim_path = lidar_parent + lidar_path
        result, prim = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path=lidar_path,
            parent=lidar_parent,
            min_range=0.4,
            max_range=25.0,
            draw_points=False,
            draw_lines=True,
            horizontal_fov=360.0,
            vertical_fov=30.0,
            horizontal_resolution=0.4,
            vertical_resolution=4.0,
            rotation_rate=0.0,
            high_lod=False,
            yaw_offset=0.0,
            enable_semantics=False
        )
        return

    def create_imu(self, robot_prim_path):
        imu_parent = f"{robot_prim_path}/zed2i_imu_link"
        imu_path = "/imu"
        self.imu_prim_path = imu_parent + imu_path
        result, prim = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path=imu_path,
            parent=imu_parent,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            visualize=False,
        )
        return        
    
    def create_depth_camera(self, robot_prim_path):
        self.depth_left_camera_path = f"{robot_prim_path}/zed2i_right_camera_isaac_frame/left_cam"
        self.depth_right_camera_path = f"{robot_prim_path}/zed2i_right_camera_isaac_frame/right_cam"
        self.left_camera = prims.create_prim(
            prim_path=self.depth_left_camera_path,
            prim_type="Camera",
            attributes={
                "focusDistance": 1,
                "focalLength": 24,
                "horizontalAperture": 20.955,
                "verticalAperture": 15.2908,
                "clippingRange": (0.1, 1000000),
                "clippingPlanes": np.array([1.0, 0.0, 1.0, 1.0]),
            },
        )


        self.right_camera = prims.create_prim(
            prim_path=self.depth_right_camera_path,
            prim_type="Camera",
            attributes={
                "focusDistance": 1,
                "focalLength": 24,
                "horizontalAperture": 20.955,
                "verticalAperture": 15.2908,
                "clippingRange": (0.1, 1000000),
                "clippingPlanes": np.array([1.0, 0.0, 1.0, 1.0]),
            },
        )
        return

    def setup_world_action_graph(self):
        og.Controller.edit(
            {"graph_path": "/globalclock", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                    ("Context.outputs:context", "PublishClock.inputs:context"),
                    ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ],
            }
        )
        return  
    
    def setup_camera_action_graph(self, robot_prim_path):
        camera_graph = "{}/camera_sensor_graph".format(robot_prim_path)
        enable_left_cam = False
        enable_right_cam = False
        rgbType = "RgbType"
        infoType = "InfoType"
        depthType = "DepthType"
        depthPclType = "DepthPclType"

        def createCamType(side, name, typeNode, topic):
            return {
                "create": [
                    (f"{side}CamHelper{name}", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ],
                "connect": [
                    (f"{side}CamViewProduct.outputs:renderProductPath", f"{side}CamHelper{name}.inputs:renderProductPath"),
                    (f"{side}CamSet.outputs:execOut", f"{side}CamHelper{name}.inputs:execIn"),
                    (f"{typeNode}.inputs:value", f"{side}CamHelper{name}.inputs:type"),
                ],
                "setvalues": [
                    (f"{side}CamHelper{name}.inputs:topicName", f"{side.lower()}/{topic}"),
                    (f"{side}CamHelper{name}.inputs:frameId", f"{NAMESPACE}/zed2i_{side.lower()}_camera_frame"),
                    (f"{side}CamHelper{name}.inputs:nodeNamespace", f"/{NAMESPACE}"),
                ]
            }

        def getCamNodes(side, enable):
            camNodes = {
                "create": [
                    (f"{side}CamBranch", "omni.graph.action.Branch"),
                    (f"{side}CamCreateViewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
                    (f"{side}CamViewportResolution", "omni.isaac.core_nodes.IsaacSetViewportResolution"),
                    (f"{side}CamViewProduct", "omni.isaac.core_nodes.IsaacGetViewportRenderProduct"),
                    (f"{side}CamSet", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
                ],
                "connect": [
                    ("OnPlaybackTick.outputs:tick", f"{side}CamBranch.inputs:execIn"),
                    (f"{side}CamBranch.outputs:execTrue", f"{side}CamCreateViewport.inputs:execIn"),
                    (f"{side}CamCreateViewport.outputs:execOut", f"{side}CamViewportResolution.inputs:execIn"),
                    (f"{side}CamCreateViewport.outputs:viewport", f"{side}CamViewportResolution.inputs:viewport"),
                    (f"{side}CamCreateViewport.outputs:viewport", f"{side}CamViewProduct.inputs:viewport"),
                    (f"{side}CamViewportResolution.outputs:execOut", f"{side}CamViewProduct.inputs:execIn"),
                    (f"{side}CamViewProduct.outputs:execOut", f"{side}CamSet.inputs:execIn"),
                    (f"{side}CamViewProduct.outputs:renderProductPath", f"{side}CamSet.inputs:renderProductPath"),
                ],
                "setvalues": [
                    (f"{side}CamBranch.inputs:condition", enable),
                    (f"{side}CamCreateViewport.inputs:name", f"{side}Cam"),
                    (f"{side}CamViewportResolution.inputs:width", 640),
                    (f"{side}CamViewportResolution.inputs:height", 360),
                ]
            }
            rgbNodes = createCamType(side, "RGB", rgbType, "rgb")
            infoNodes = createCamType(side, "Info", infoType, "camera_info")
            depthNodes = createCamType(side, "Depth", depthType, "depth")
            depthPClNodes = createCamType(side, "DepthPcl", depthPclType, "depth_pcl")
            camNodes["create"] += rgbNodes["create"] + infoNodes["create"] + depthNodes["create"] + depthPClNodes["create"]
            camNodes["connect"] += rgbNodes["connect"] + infoNodes["connect"] + depthNodes["connect"] + depthPClNodes["connect"]
            camNodes["setvalues"] += rgbNodes["setvalues"] + infoNodes["setvalues"] + depthNodes["setvalues"] + depthPClNodes["setvalues"]
            return camNodes

        leftCamNodes = getCamNodes("Left", enable_left_cam)
        rightCamNodes = getCamNodes("Right", enable_right_cam)
        og.Controller.edit(
            {"graph_path": camera_graph, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    (rgbType, "omni.graph.nodes.ConstantToken"),
                    (infoType, "omni.graph.nodes.ConstantToken"),
                    (depthType, "omni.graph.nodes.ConstantToken"),
                    (depthPclType, "omni.graph.nodes.ConstantToken"),
                ] + leftCamNodes["create"] + rightCamNodes["create"],

                og.Controller.Keys.CONNECT: leftCamNodes["connect"] + rightCamNodes["connect"],

                og.Controller.Keys.SET_VALUES: [
                    (f"{rgbType}.inputs:value", "rgb"),
                    (f"{infoType}.inputs:value", "camera_info"),
                    (f"{depthType}.inputs:value", "depth"),
                    (f"{depthPclType}.inputs:value", "depth_pcl"),
                ] + leftCamNodes["setvalues"] + rightCamNodes["setvalues"],
            }
        )
        set_target_prims(primPath=f"{camera_graph}/RightCamSet", targetPrimPaths=[self.depth_right_camera_path], inputName="inputs:cameraPrim")
        set_target_prims(primPath=f"{camera_graph}/LeftCamSet", targetPrimPaths=[self.depth_left_camera_path], inputName="inputs:cameraPrim")
        return


    def setup_imu_action_graph(self, robot_prim_path):
        sensor_graph = "{}/imu_sensor_graph".format(robot_prim_path)
        swerve_link = "{}/swerve_chassis_link".format(robot_prim_path)
        lidar_link = "{}/lidar_link/lidar".format(robot_prim_path)

        og.Controller.edit(
            {"graph_path": sensor_graph, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    # General Nodes
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("SimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                    # Odometry Nodes
                    ("ComputeOdometry", "omni.isaac.core_nodes.IsaacComputeOdometry"),
                    ("PublishOdometry", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
                    ("RawOdomTransform", "omni.isaac.ros2_bridge.ROS2PublishRawTransformTree"),
                    # LiDAR Nodes
                    # ("ReadLidar", "omni.isaac.range_sensor.IsaacReadLidarBeams"),
                    # ("PublishLidar", "omni.isaac.ros2_bridge.ROS2PublishLaserScan"),
                    # IMU Nodes
                    ("IsaacReadImu", "omni.isaac.sensor.IsaacReadIMU"),
                    ("PublishImu", "omni.isaac.ros2_bridge.ROS2PublishImu"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("PublishOdometry.inputs:nodeNamespace", f"/{NAMESPACE}"),
                    # ("PublishLidar.inputs:nodeNamespace", f"/{NAMESPACE}"),
                    ("PublishImu.inputs:nodeNamespace", f"/{NAMESPACE}"), 
                    # ("PublishLidar.inputs:frameId", f"{NAMESPACE}/lidar_link"),
                    ("RawOdomTransform.inputs:childFrameId", f"{NAMESPACE}/base_link"),
                    ("RawOdomTransform.inputs:parentFrameId", f"{NAMESPACE}/zed/odom"),
                    ("PublishOdometry.inputs:chassisFrameId", f"{NAMESPACE}/base_link"),
                    ("PublishOdometry.inputs:odomFrameId", f"{NAMESPACE}/odom"),
                    ("PublishImu.inputs:frameId", f"{NAMESPACE}/zed2i_imu_link"),
                    ("PublishOdometry.inputs:topicName", "zed/odom")
                ],
                og.Controller.Keys.CONNECT: [
                    # Odometry Connections
                    ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "RawOdomTransform.inputs:execIn"),
                    ("ComputeOdometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
                    ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
                    ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
                    ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
                    ("ComputeOdometry.outputs:orientation", "RawOdomTransform.inputs:rotation"),
                    ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
                    ("ComputeOdometry.outputs:position", "RawOdomTransform.inputs:translation"),
                    ("Context.outputs:context", "PublishOdometry.inputs:context"),
                    ("Context.outputs:context", "RawOdomTransform.inputs:context"),
                    ("SimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
                    ("SimTime.outputs:simulationTime", "RawOdomTransform.inputs:timeStamp"),

                    # LiDAR Connections
                    # ("OnPlaybackTick.outputs:tick", "ReadLidar.inputs:execIn"),
                    # ("ReadLidar.outputs:execOut", "PublishLidar.inputs:execIn"),
                    # ("Context.outputs:context", "PublishLidar.inputs:context"),
                    # ("SimTime.outputs:simulationTime", "PublishLidar.inputs:timeStamp"),
                    
                    # ("ReadLidar.outputs:azimuthRange", "PublishLidar.inputs:azimuthRange"),
                    # ("ReadLidar.outputs:depthRange", "PublishLidar.inputs:depthRange"),
                    # ("ReadLidar.outputs:horizontalFov", "PublishLidar.inputs:horizontalFov"),
                    # ("ReadLidar.outputs:horizontalResolution", "PublishLidar.inputs:horizontalResolution"),
                    # ("ReadLidar.outputs:intensitiesData", "PublishLidar.inputs:intensitiesData"),
                    # ("ReadLidar.outputs:linearDepthData", "PublishLidar.inputs:linearDepthData"),
                    # ("ReadLidar.outputs:numCols", "PublishLidar.inputs:numCols"),
                    # ("ReadLidar.outputs:numRows", "PublishLidar.inputs:numRows"),
                    # ("ReadLidar.outputs:rotationRate", "PublishLidar.inputs:rotationRate"),
                    
                    # IMU Connections
                    ("OnPlaybackTick.outputs:tick", "IsaacReadImu.inputs:execIn"),
                    ("IsaacReadImu.outputs:execOut", "PublishImu.inputs:execIn"),
                    ("Context.outputs:context", "PublishImu.inputs:context"),
                    ("SimTime.outputs:simulationTime", "PublishImu.inputs:timeStamp"),

                    ("IsaacReadImu.outputs:angVel", "PublishImu.inputs:angularVelocity"),
                    ("IsaacReadImu.outputs:linAcc", "PublishImu.inputs:linearAcceleration"),
                    ("IsaacReadImu.outputs:orientation", "PublishImu.inputs:orientation"),
                ],
            }
        )
        # Setup target prims for the Odometry and the Lidar
        set_target_prims(primPath=f"{sensor_graph}/ComputeOdometry", targetPrimPaths=[swerve_link], inputName="inputs:chassisPrim")
        set_target_prims(primPath=f"{sensor_graph}/ComputeOdometry", targetPrimPaths=[swerve_link], inputName="inputs:chassisPrim") 
        set_target_prims(primPath=f"{sensor_graph}/IsaacReadImu", targetPrimPaths=[self.imu_prim_path], inputName="inputs:imuPrim")
        return

    def setup_robot_action_graph(self, robot_prim_path):
        robot_controller_path = f"{robot_prim_path}/ros_interface_controller"
        og.Controller.edit(
            {"graph_path": robot_controller_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                    ("SubscribeJointState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
                    ("articulation_controller", "omni.isaac.core_nodes.IsaacArticulationController"),
                    
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("PublishJointState.inputs:topicName", "isaac_joint_states"),
                    ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
                    ("articulation_controller.inputs:usePath", False),
                    ("SubscribeJointState.inputs:nodeNamespace", f"/{NAMESPACE}"),
                    ("PublishJointState.inputs:nodeNamespace", f"/{NAMESPACE}"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                   
                    ("OnPlaybackTick.outputs:tick", "articulation_controller.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                    ("Context.outputs:context", "PublishJointState.inputs:context"),
                    ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                    ("SubscribeJointState.outputs:jointNames", "articulation_controller.inputs:jointNames"),
                    ("SubscribeJointState.outputs:velocityCommand", "articulation_controller.inputs:velocityCommand"),
                    ("SubscribeJointState.outputs:positionCommand", "articulation_controller.inputs:positionCommand"),
                ],
            }
        )

        set_target_prims(primPath=f"{robot_controller_path}/articulation_controller", targetPrimPaths=[robot_prim_path])
        set_target_prims(primPath=f"{robot_controller_path}/PublishJointState", targetPrimPaths=[robot_prim_path])
        return

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        return
    
    async def setup_post_clear(self):
        return
    
    def world_cleanup(self):
        carb.log_info(f"Removing {self.robot_name}")
        if self._world is not None:
            self._world.scene.remove_object(self.robot_name)
        return
