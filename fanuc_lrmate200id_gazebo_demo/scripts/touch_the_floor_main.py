#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import sys

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import WrenchStamped
import moveit_commander
from moveit_commander import conversions
import rospy
import tf
import tf2_geometry_msgs
import tf2_ros

import termcolor


def loginfo(msg):
    rospy.loginfo('[{}] {}'.format(rospy.get_name(),
                                   termcolor.colored(msg, 'green')))


class MainController(object):

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.eef_ft = None
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.manipulator = self.robot.manipulator
        self.manipulator.remember_joint_values('reset_pose', [
            -8.919513305727378e-06,
            0.02127825324544297,
            -0.24378089125464353,
            -4.526926321801739e-06,
            0.2012009032703368,
            7.030738657931579e-09
        ])  # inititial pose on Gazebo
        self.pub = rospy.Publisher('~output', WrenchStamped, queue_size=1)
        self.sub_ft_sensor = rospy.Subscriber(
            '/ft_sensor', WrenchStamped, self._cb_ft_sensor, queue_size=1)

    def _cb_ft_sensor(self, msg):
        force = msg.wrench.force
        torque = msg.wrench.torque
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame='world',
                source_frame=msg.header.frame_id,
                time=msg.header.stamp,
                timeout=rospy.Duration(2.0),
            )
        except Exception:
            link_pose = self.robot._r.get_link_pose(msg.header.frame_id)
            transform = TransformStamped(
                header=msg.header,
                transform=conversions.list_to_transform(link_pose),
            )
            transform.header.frame_id = self.robot.get_planning_frame()
        force = tf2_geometry_msgs.do_transform_vector3(
            Vector3Stamped(header=msg.header, vector=force), transform).vector
        torque = tf2_geometry_msgs.do_transform_vector3(
            Vector3Stamped(header=msg.header, vector=torque), transform).vector
        self.eef_ft = Wrench(force=force, torque=torque)

    def go_down_to_floor(self, reactive=True):
        # # add plane under the robot
        # from geometry_msgs.msg import PoseStamped
        # p = PoseStamped()
        # p.header.frame_id = self.robot.get_planning_frame()
        # p.pose.orientation.w = 1
        # self.scene.add_box('floor', p, (1, 1, 0.01))
        # rospy.sleep(1)

        # rotate the end effector to the ground
        loginfo('Initializing to go down to floor')
        self.manipulator.clear_pose_targets()
        pose = self.manipulator.get_current_pose().pose
        pose.orientation = Quaternion(
            *tf.transformations.quaternion_from_euler(-3.14, 0, 0))
        self.manipulator.set_pose_target(pose)
        self.manipulator.go(wait=True)
        rospy.sleep(3)

        loginfo('Going down')
        self.manipulator.clear_pose_targets()
        pose.position.z = -1.0
        self.manipulator.clear_pose_targets()
        plan, fraction = self.manipulator.compute_cartesian_path(
            [pose], 0.01, 0.0)
        self.manipulator.execute(plan, wait=False)
        while True:
            force_z = self.eef_ft.force.z
            loginfo('Force in z axis: {}'.format(force_z))
            if reactive and force_z > 0:
                loginfo('Touched the floor')
                self.manipulator.stop()
                self.manipulator.clear_pose_targets()
                pose = self.manipulator.get_current_pose().pose
                loginfo("Approximate floor's z is: {}".format(pose.position.z))
                pose.position.z += 0.01
                plan, fraction = self.manipulator.compute_cartesian_path(
                    [pose], 0.01, 0.0)
                self.manipulator.execute(plan, wait=True)
                break
            rospy.sleep(0.01)


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('touch_floor')
    c = MainController()
    rospy.sleep(1)
    reactive = rospy.get_param('~reactive', True)
    loginfo('Reactive Planning: {}'.format(reactive))
    c.go_down_to_floor(reactive=reactive)
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException():
        pass
