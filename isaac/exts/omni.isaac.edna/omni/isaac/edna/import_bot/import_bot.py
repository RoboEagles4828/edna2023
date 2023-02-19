from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.graph.core as og
import omni.usd
from omni.isaac.edna.base_sample import BaseSample
from omni.isaac.urdf import _urdf
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils import prims
from omni.isaac.core.prims import GeometryPrim
from omni.isaac.core_nodes.scripts.utils import set_target_prims
from omni.kit.viewport_legacy import get_default_viewport_window
from omni.isaac.sensor import IMUSensor
from pxr import UsdPhysics
import omni.kit.commands
import os
import numpy as np
import math
import carb

def set_drive_params(drive, stiffness, damping, max_force):
    drive.GetStiffnessAttr().Set(stiffness)
    drive.GetDampingAttr().Set(damping)
    if(max_force != 0.0):
        drive.GetMaxForceAttr().Set(max_force)
    return

class ImportBot(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    

    def setup_scene(self):
        world = self.get_world()
        world.get_physics_context().enable_gpu_dynamics(False)
        world.scene.add_default_ground_plane()
        self.setup_field()
        # self.setup_perspective_cam()
        self.setup_world_action_graph()
        return
   
    def setup_field(self):
        world = self.get_world()
        self.extension_path = os.path.abspath(__file__)
        self.project_root_path = os.path.abspath(os.path.join(self.extension_path, "../../../../../../.."))
        field = os.path.join(self.project_root_path, "assets/2023_field/FE-2023.usd")
        add_reference_to_stage(usd_path=field,prim_path="/World/Field")
        cone = os.path.join(self.project_root_path, "assets/2023_field/parts/cone_without_deformable_body.usd")
        cube = os.path.join(self.project_root_path, "assets/2023_field/parts/cube_without_deformable_body.usd")
        # chargestation = os.path.join(self.project_root_path, "assets/2023_field/charge_station.usd")
        # add_reference_to_stage(chargestation, "/World/ChargeStation_1")
        # add_reference_to_stage(chargestation, "/World/ChargeStation_2") 
        add_reference_to_stage(cone, "/World/Cone_1")
        add_reference_to_stage(cone, "/World/Cone_2")
        add_reference_to_stage(cone, "/World/Cone_3")
        add_reference_to_stage(cone, "/World/Cone_4")
        # add_reference_to_stage(cone, "/World/Cone_5")
        # add_reference_to_stage(cone, "/World/Cone_6")
        # add_reference_to_stage(cone, "/World/Cone_7")
        # add_reference_to_stage(cone, "/World/Cone_8")
        cone_1 = GeometryPrim("/World/Cone_1","cone_1_view",position=np.array([1.20298,-0.56861,0.0]))
        cone_2 = GeometryPrim("/World/Cone_2","cone_2_view",position=np.array([1.20298,3.08899,0.0]))
        cone_3 = GeometryPrim("/World/Cone_3","cone_3_view",position=np.array([-1.20298,-0.56861,0.0]))
        cone_4 = GeometryPrim("/World/Cone_4","cone_4_view",position=np.array([-1.20298,3.08899,0.0]))
        # chargestation_1 = GeometryPrim("/World/ChargeStation_1","cone_3_view",position=np.array([-4.20298,-0.56861,0.0]))
        # chargestation_2 = GeometryPrim("/World/ChargeStation_2","cone_4_view",position=np.array([4.20298,0.56861,0.0]))
        

        add_reference_to_stage(cube, "/World/Cube_1")
        add_reference_to_stage(cube, "/World/Cube_2")
        add_reference_to_stage(cube, "/World/Cube_3")
        add_reference_to_stage(cube, "/World/Cube_4")
        # add_reference_to_stage(cube, "/World/Cube_5")
        # add_reference_to_stage(cube, "/World/Cube_6")
        # add_reference_to_stage(cube, "/World/Cube_7")
        # add_reference_to_stage(cube, "/World/Cube_8")
        cube_1 = GeometryPrim("/World/Cube_1","cube_1_view",position=np.array([1.20298,0.65059,0.121]))
        cube_2 = GeometryPrim("/World/Cube_2","cube_2_view",position=np.array([1.20298,1.86979,0.121]))
        cube_3 = GeometryPrim("/World/Cube_3","cube_3_view",position=np.array([-1.20298,0.65059,0.121]))
        cube_4 = GeometryPrim("/World/Cube_4","cube_4_view",position=np.array([-1.20298,1.86979,0.121]))

    async def setup_post_load(self):
        self._world = self.get_world()
        # self._world.get_physics_context().enable_gpu_dynamics(True)
        self.robot_name = "Swerve"
        self.extension_path = os.path.abspath(__file__)
        self.project_root_path = os.path.abspath(os.path.join(self.extension_path, "../../../../../../.."))
        self.path_to_urdf = os.path.join(self.extension_path, "../../../../../../../..", "src/edna_description/urdf/edna.urdf")
        carb.log_info(self.path_to_urdf)

        self._robot_prim_path = self.import_robot(self.path_to_urdf)


        if self._robot_prim_path is None:
            print("Error: failed to import robot")
            return
        
        self._robot_prim = self._world.scene.add(
            Robot(prim_path=self._robot_prim_path, name=self.robot_name, position=np.array([0.0, 0.0, 0.3]))
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
        chassis_name = "swerve_chassis_link"

       
        front_left_axle = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/{chassis_name}/front_left_axle_joint"), "angular")
        front_right_axle = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/{chassis_name}/front_right_axle_joint"), "angular")
        rear_left_axle = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/{chassis_name}/rear_left_axle_joint"), "angular")
        rear_right_axle = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/{chassis_name}/rear_right_axle_joint"), "angular")
        front_left_wheel = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/front_left_axle_link/front_left_wheel_joint"), "angular")
        front_right_wheel = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/front_right_axle_link/front_right_wheel_joint"), "angular")
        rear_left_wheel = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/rear_left_axle_link/rear_left_wheel_joint"), "angular")
        rear_right_wheel = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/rear_right_axle_link/rear_right_wheel_joint"), "angular")
        # base = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/base_link/swerve_chassis_joint"), "angular")
        # set_drive_params(front_left_axle, 10000000.0, 100000.0, 98.0)
        # set_drive_params(front_right_axle, 10000000.0, 100000.0, 98.0)
        # set_drive_params(rear_left_axle, 10000000.0, 100000.0, 98.0)
        # set_drive_params(rear_right_axle, 10000000.0, 100000.0, 98.0)
        set_drive_params(front_left_axle, 1, 1000, 98.0)
        set_drive_params(front_right_axle, 1, 1000, 98.0)
        set_drive_params(rear_left_axle, 1, 1000, 98.0)
        set_drive_params(rear_right_axle, 1, 1000, 98.0)       
        set_drive_params(front_left_wheel, 1, 1000, 98.0)
        set_drive_params(front_right_wheel, 1, 1000, 98.0)
        set_drive_params(rear_left_wheel, 1, 1000, 98.0)
        set_drive_params(rear_right_wheel, 1, 1000, 98.0)
        # set_drive_params(base,1,1000,98.0)
        #self.create_lidar(robot_prim_path)
        #self.create_depth_camera()
        self.setup_imu_action_graph(robot_prim_path)
        self.setup_robot_action_graph(robot_prim_path)
        return

    def create_lidar(self, robot_prim_path):
        lidar_parent = "{}/lidar_link".format(robot_prim_path)
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
    
    def create_depth_camera(self):
        self.depth_left_camera_path = f"{self._robot_prim_path}/zed_left_camera_frame/left_cam"
        self.depth_right_camera_path = f"{self._robot_prim_path}/zed_right_camera_frame/right_cam"
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
    
    def setup_imu_action_graph(self, robot_prim_path):
        imu_graph = "{}/sensor_graph".format(robot_prim_path)
        swerve_link = "{}/swerve_chassis_link".format(robot_prim_path)

        og.Controller.edit(
            {"graph_path": imu_graph, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("SimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("ComputeOdometry", "omni.isaac.core_nodes.IsaacComputeOdometry"),
                    ("PublishOdometry", "omni.isaac.ros2_bridge.ROS2PublishOdometry")
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                    ("ComputeOdometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
                    # ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
                    # ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
                    ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
                    # ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
                    ("Context.outputs:context", "PublishOdometry.inputs:context"),
                    ("SimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp")
                ],
            }
        )
        set_target_prims(primPath=f"{imu_graph}/ComputeOdometry", targetPrimPaths=[swerve_link], inputName="inputs:chassisPrim")
        return
    
    def setup_perspective_cam(self):
        # Get the Viewport and the Default Camera
        viewport_window = get_default_viewport_window()
        camera = self.get_world().stage.GetPrimAtPath(viewport_window.get_active_camera())

        # Get Default Cam Values
        camAttributes = {}
        camOrientation = None
        camTranslation = None
        for att in camera.GetAttributes():
            name = att.GetName()
            if not (name.startswith('omni') or name.startswith('xform')):
                camAttributes[att.GetName()] = att.Get()
            elif name == 'xformOp:orient':
                convertedQuat = [att.Get().GetReal()] + list(att.Get().GetImaginary())
                camOrientation = np.array(convertedQuat)
            elif name == 'xformOp:translate':
                camTranslation = np.array(list(att.Get()))

        # Modify what we want
        camAttributes["clippingRange"] = (0.1, 1000000)
        camAttributes["clippingPlanes"] = np.array([1.0, 0.0, 1.0, 1.0])

        # Create a new camera with desired values
        cam_path = "/World/PerspectiveCam"
        prims.create_prim(
            prim_path=cam_path,
            prim_type="Camera",
            translation=camTranslation,
            orientation=camOrientation,
            attributes=camAttributes,
        )

        # Use the camera for our viewport
        viewport_window.set_active_camera(cam_path)
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
                    # ("SubscribeJointState.outputs:positionCommand", "articulation_controller.inputs:positionCommand"),
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
        self._world.scene.remove_object(self.robot_name)
        return
