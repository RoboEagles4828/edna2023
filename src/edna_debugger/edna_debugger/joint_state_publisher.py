# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Standard Python imports
import argparse
import math
import sys
import time
import xml.dom.minidom

# ROS 2 imports
import rclpy
import rclpy.node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import sensor_msgs.msg
import std_msgs.msg

POSITION_JOINTS = {
    'arm_roller_bar_joint',
    'elevator_center_joint',
    'elevator_outer_1_joint',
    'elevator_outer_2_joint',
    'top_gripper_right_arm_joint',
    'top_gripper_left_arm_joint',
    'top_slider_joint',
    'bottom_intake_joint',
    'front_left_axle_joint',
    'front_right_axle_joint',
    'rear_left_axle_joint',
    'rear_right_axle_joint',
}
    
VELOCITY_JOINTS = [
    'front_left_wheel_joint',
    'front_right_wheel_joint',
    'rear_left_wheel_joint',
    'rear_right_wheel_joint',
]

class JointStatePublisher(rclpy.node.Node):
    def get_param(self, name):
        return self.get_parameter(name).value

    def _init_joint(self, minval, maxval, zeroval):
        joint = {'min': minval, 'max': maxval, 'zero': zeroval}
        if self.pub_def_positions:
            joint['position'] = zeroval
        if self.pub_def_vels:
            joint['velocity'] = 0.0
        if self.pub_def_efforts:
            joint['effort'] = 0.0
        return joint

    def init_collada(self, robot):
        robot = robot.getElementsByTagName('kinematics_model')[0].getElementsByTagName('technique_common')[0]
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                name = child.getAttribute('name')
                if child.getElementsByTagName('revolute'):
                    joint = child.getElementsByTagName('revolute')[0]
                else:
                    self.get_logger().warn('Unknown joint type %s', child)
                    continue

                if joint:
                    limit = joint.getElementsByTagName('limits')[0]
                    minval = float(limit.getElementsByTagName('min')[0].childNodes[0].nodeValue)
                    maxval = float(limit.getElementsByTagName('max')[0].childNodes[0].nodeValue)
                if minval == maxval:  # this is a fixed joint
                    continue

                self.joint_list.append(name)
                minval *= math.pi/180.0
                maxval *= math.pi/180.0
                self.free_joints_sub[name] = self._init_joint(minval, maxval, 0.0)
                self.free_joints_pub[name] = self.free_joints_sub[name]

    def init_urdf(self, robot):
        robot = robot.getElementsByTagName('robot')[0]
        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype in ['fixed', 'floating', 'planar']:
                    continue
                name = child.getAttribute('name')
                self.joint_list.append(name)
                if jtype == 'continuous': 
                    if name in VELOCITY_JOINTS:
                        minval = -39.4 # this is not a good number i pulled it out of thin air
                        maxval = 39.4  # please someone calculate the actual thing using the circumference of the wheels and that stuff
                    else:
                        minval = -math.pi
                        maxval = math.pi
                else:
                    try:
                        limit = child.getElementsByTagName('limit')[0]
                        minval = float(limit.getAttribute('lower'))
                        maxval = float(limit.getAttribute('upper'))
                    except:
                        self.get_logger().warn('%s is not fixed, nor continuous, but limits are not specified!' % name)
                        continue

                safety_tags = child.getElementsByTagName('safety_controller')
                if self.use_small and len(safety_tags) == 1:
                    tag = safety_tags[0]
                    if tag.hasAttribute('soft_lower_limit'):
                        minval = max(minval, float(tag.getAttribute('soft_lower_limit')))
                    if tag.hasAttribute('soft_upper_limit'):
                        maxval = min(maxval, float(tag.getAttribute('soft_upper_limit')))

                mimic_tags = child.getElementsByTagName('mimic')
                if self.use_mimic and len(mimic_tags) == 1:
                    tag = mimic_tags[0]
                    entry = {'parent': tag.getAttribute('joint')}
                    if tag.hasAttribute('multiplier'):
                        entry['factor'] = float(tag.getAttribute('multiplier'))
                    if tag.hasAttribute('offset'):
                        entry['offset'] = float(tag.getAttribute('offset'))

                    self.dependent_joints[name] = entry
                    continue

                if name in self.dependent_joints:
                    continue

                if self.zeros and name in self.zeros:
                    zeroval = self.zeros[name]
                elif minval > 0 or maxval < 0:
                    zeroval = (maxval + minval)/2
                else:
                    zeroval = 0

                joint = self._init_joint(minval, maxval, zeroval)

                if jtype == 'continuous':
                    joint['continuous'] = True
                self.free_joints_sub[name] = joint
                self.free_joints_pub[name] = joint.copy()

    def configure_robot(self, description):
        self.get_logger().debug('Got description, configuring robot')
        try:
            robot = xml.dom.minidom.parseString(description)
        except xml.parsers.expat.ExpatError:
            # If the description fails to parse for some reason, print an error
            # and get out of here without doing further work.  If we were
            # already running with a description, we'll continue running with
            # that older one.
            self.get_logger().warn('Invalid robot_description given, ignoring')
            return

        # Make sure to clear out the old joints so we don't get duplicate joints
        # on a new robot description.
        self.free_joints_sub = {}
        self.free_joints_pub = {}
        self.joint_list = [] # for maintaining the original order of the joints

        if robot.getElementsByTagName('COLLADA'):
            self.init_collada(robot)
        else:
            self.init_urdf(robot)

        if self.robot_description_update_cb is not None:
            self.robot_description_update_cb()

    def parse_dependent_joints(self):
        dj = {}
        dependent_joints = self.get_parameters_by_prefix('dependent_joints')
        # get_parameters_by_prefix returned a dictionary of keynames like
        # 'head.parent', 'head.offset', etc. that map to rclpy Parameter values.
        # The rest of the code assumes that the dependent_joints dictionary is
        # a map of name -> dict['parent': parent, factor: factor, offset: offset],
        # where both factor and offset are optional.  Thus we parse the values we
        # got into that structure.
        for name, param in dependent_joints.items():
            # First split on the dots; there should be one and exactly one dot
            split = name.split('.')
            if len(split) != 2:
                raise Exception("Invalid dependent_joint name '%s'" % (name))
            newkey = split[0]
            newvalue = split[1]
            if newvalue not in ['parent', 'factor', 'offset']:
                raise Exception("Invalid dependent_joint name '%s' (allowed values are 'parent', 'factor', and 'offset')" % (newvalue))
            if newkey in dj:
                dj[newkey].update({newvalue: param.value})
            else:
                dj[newkey] = {newvalue: param.value}

        # Now ensure that there is at least a 'parent' in all keys
        for name,outdict in dj.items():
            if outdict.get('parent', None) is None:
                raise Exception('All dependent_joints must at least have a parent')

        return dj

    def declare_ros_parameter(self, name, default, descriptor):
        # When the automatically_declare_parameters_from_overrides parameter to
        # rclpy.create_node() is True, then any parameters passed in on the
        # command-line are automatically declared.  In that case, calling
        # node.declare_parameter() will raise an exception.  However, in the case
        # where a parameter is *not* overridden, we still want to declare it
        # (so it shows up in "ros2 param list", for instance).  Thus we always do
        # a declaration and just ignore ParameterAlreadyDeclaredException.

        try:
            self.declare_parameter(name, default, descriptor)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

    def __init__(self, urdf_file):
        super().__init__('joint_state_publisher', automatically_declare_parameters_from_overrides=True)

        self.declare_ros_parameter('publish_default_efforts', False, ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.declare_ros_parameter('publish_default_positions', True, ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.declare_ros_parameter('publish_default_velocities', False, ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.declare_ros_parameter('rate', 10, ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_ros_parameter('source_list', [], ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY))
        self.declare_ros_parameter('use_mimic_tags', True, ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.declare_ros_parameter('use_smallest_joint_limits', True, ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.declare_ros_parameter('delta', 0.0, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        # In theory we would also declare 'dependent_joints' and 'zeros' here.
        # Since rclpy doesn't support maps natively, though, we just end up
        # letting 'automatically_declare_parameters_from_overrides' declare
        # any parameters for us.

        self.free_joints_sub = {}
        self.free_joints_pub = {}

        self.joint_list = [] # for maintaining the original order of the joints
        self.dependent_joints = self.parse_dependent_joints()
        self.use_mimic = self.get_param('use_mimic_tags')
        self.use_small = self.get_param('use_smallest_joint_limits')

        zeros = self.get_parameters_by_prefix('zeros')
        # get_parameters_by_prefix() returns a map of name -> Parameter
        # structures, but self.zeros is expected to be a list of name -> float;
        # fix that here.
        self.zeros = {k:v.value for (k, v) in zeros.items()}

        self.pub_def_positions = self.get_param('publish_default_positions')
        self.pub_def_vels = self.get_param('publish_default_velocities')
        self.pub_def_efforts = self.get_param('publish_default_efforts')

        self.robot_description_update_cb = None


        if urdf_file is not None:
            # If we were given a URDF file on the command-line, use that.
            with open(urdf_file, 'r') as infp:
                description = infp.read()
            self.configure_robot(description)
        else:
            # Otherwise, subscribe to the '/robot_description' topic and wait
            # for a callback there
            self.get_logger().info('Waiting for robot_description to be published on the robot_description topic...')
            self.create_subscription(std_msgs.msg.String, 'robot_description',
                                     lambda msg: self.configure_robot(msg.data),
                                     rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL))

        self.delta = self.get_param('delta')

        source_list = self.get_param('source_list')
        self.sources = []
        # for source in source_list:
        #     self.sources.append(self.create_subscription(sensor_msgs.msg.JointState, source, self.source_cb, 10))

        # The source_update_cb will be called at the end of self.source_cb.
        # The main purpose is to allow external observers (such as the
        # joint_state_publisher_gui) to be notified when things are updated.
        self.source_update_cb = None


        # Override topic name here
        self.pub_pos = self.create_publisher(std_msgs.msg.Float64MultiArray, 'forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(std_msgs.msg.Float64MultiArray, 'forward_velocity_controller/commands', 10)
        self.create_subscription(sensor_msgs.msg.JointState, 'joint_states', self.source_cb, 10)
        self.timer = self.create_timer(1.0 / self.get_param('rate'), self.timer_callback)

    def source_cb(self, msg):
        # self.get_logger().info("ran source callback")
        for i in range(len(msg.name)):
            name = msg.name[i]
            if name not in self.free_joints_sub:
                continue

            if msg.position:
                position = msg.position[i]
            else:
                position = None
            if msg.velocity:
                velocity = msg.velocity[i]
            else:
                velocity = None
            if msg.effort:
                effort = msg.effort[i]
            else:
                effort = None

            joint = self.free_joints_sub[name]
            if position is not None:
                joint['position'] = position
            if velocity is not None:
                joint['velocity'] = velocity
            if effort is not None:
                joint['effort'] = effort

        
        if self.source_update_cb is not None:
            self.source_update_cb()

    def set_source_update_cb(self, user_cb):
        self.source_update_cb = user_cb

    def set_robot_description_update_cb(self, user_cb):
        self.robot_description_update_cb = user_cb

    def timer_callback(self):
        # Publish Joint States
        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.delta > 0:
            self.update(self.delta)

        # Initialize msg.position, msg.velocity, and msg.effort.
        has_position = False
        has_velocity = False
        has_effort = False
        for name, joint in self.free_joints_pub.items():
            if not has_position and 'position' in joint:
                has_position = True
            if not has_velocity and 'velocity' in joint:
                has_velocity = True
            if not has_effort and 'effort' in joint:
                has_effort = True
        num_joints = (len(self.free_joints_pub.items()) +
                      len(self.dependent_joints.items()))
        if has_position:
            msg.position = []
        if has_velocity:
            msg.velocity = []
        if has_effort:
            msg.effort = num_joints * [0.0]
        
        for i, name in enumerate(self.joint_list):
            msg.name.append(str(name))
            joint = None

            # Add Free Joint
            if name in self.free_joints_pub:
                joint = self.free_joints_pub[name]
                factor = 1
                offset = 0
            # Add Dependent Joint
            elif name in self.dependent_joints:
                param = self.dependent_joints[name]
                parent = param['parent']
                factor = param.get('factor', 1.0)
                offset = param.get('offset', 0.0)
                # Handle recursive mimic chain
                recursive_mimic_chain_joints = [name]
                while parent in self.dependent_joints:
                    if parent in recursive_mimic_chain_joints:
                        error_message = 'Found an infinite recursive mimic chain'
                        self.get_logger().error(f'{error_message}: {recursive_mimic_chain_joints + [parent]}')
                        sys.exit(1)
                    recursive_mimic_chain_joints.append(parent)
                    param = self.dependent_joints[parent]
                    parent = param['parent']
                    offset += factor * param.get('offset', 0)
                    factor *= param.get('factor', 1)
                joint = self.free_joints_pub[parent]

            # the issue here is that its setting [i] either to the joint, or its skipping i in the list when we need it to be the next value.
            # For some reason, [list].append() ends up putting way too many empty values that come from nowhere into the list.
            # The best thing to do here would be having two separate iterators which only increment if the variable has a position or a velocity.
            if has_position and 'position' in joint and name in POSITION_JOINTS:
                msg.position.append(float(joint['position']) * factor + offset)
            if has_velocity and 'velocity' in joint and name in VELOCITY_JOINTS:
                msg.velocity.append(float(joint['velocity']) * factor)
            if has_effort and 'effort' in joint:
                msg.effort[i] = float(joint['effort'])
            

        # Only publish non-empty messages
        if not (msg.name or msg.position or msg.velocity or msg.effort):
            return

        pos_msg = std_msgs.msg.Float64MultiArray()
        vel_msg = std_msgs.msg.Float64MultiArray()

        pos_msg.data = msg.position
        vel_msg.data = msg.velocity

        self.pub_pos.publish(pos_msg)
        self.pub_vel.publish(vel_msg)

    def update(self, delta):
        for name, joint in self.free_joints_sub.items():
            forward = joint.get('forward', True)
            if forward:
                joint['position'] += delta
                if joint['position'] > joint['max']:
                    if joint.get('continuous', False):
                        joint['position'] = joint['min']
                    else:
                        joint['position'] = joint['max']
                        joint['forward'] = not forward
            else:
                joint['position'] -= delta
                if joint['position'] < joint['min']:
                    joint['position'] = joint['min']
                    joint['forward'] = not forward


def main():
    # Initialize rclpy with the command-line arguments
    rclpy.init()

    # Strip off the ROS 2-specific command-line arguments
    stripped_args = rclpy.utilities.remove_ros_args(args=sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument('urdf_file', help='URDF file to use', nargs='?', default=None)

    # Parse the remaining arguments, noting that the passed-in args must *not*
    # contain the name of the program.
    parsed_args = parser.parse_args(args=stripped_args[1:])

    jsp = JointStatePublisher(parsed_args.urdf_file)

    try:
        rclpy.spin(jsp)
    except KeyboardInterrupt:
        pass

    jsp.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
