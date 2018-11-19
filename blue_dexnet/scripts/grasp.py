#!/usr/bin/env python
import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import sys
import actionlib
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import PyKDL as kdl
import kdl_parser_py.urdf as kdl_parser
from trac_ik_python.trac_ik_wrap import TRAC_IK
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal,
)


def cheb_points(n, k):
    # return float(k)/float(n)
    xk = np.cos( (2.0 * k  - 1.0) / (2.0 * n) * np.pi )
    return (-xk + 1.0) / 2.0

class GripperClient(object):
    def __init__(self):
        self._client = actionlib.SimpleActionClient(
            "blue_controllers/gripper_controller/gripper_cmd",
            GripperCommandAction,
        )
        # Wait 10 Seconds for the gripper action server to start or exit
        if not self._client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Exiting - Gripper Action Server Not Found")
            rospy.signal_shutdown("Action Server not found")
            sys.exit(1)

    def command(self, position, effort):
        goal = GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = effort
        self._client.send_goal(goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=0.1):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))
        return self._client.get_result()

    def call_grip(self, pos):
        # commands a position 0 = open, 1 = close
        position = -1.5 * pos
        self.command(position, 2.5)
        self.wait()

class BlueIK:
    def _setup(self):
        # set up chebychev points
        self.resolution = 100
        self.duration = 2

        # build tf listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # load in ros parameters
        self.baselink = rospy.get_param("blue_hardware/baselink")
        self.endlink = rospy.get_param("blue_hardware/endlink")
        urdf = rospy.get_param("/robot_description")
        flag, self.tree = kdl_parser.treeFromString(urdf)

        # build kinematic chain and fk and jacobian solvers
        chain_ee = self.tree.getChain(self.baselink, self.endlink)
        self.fk_ee = kdl.ChainFkSolverPos_recursive(chain_ee)
        self.jac_ee = kdl.ChainJntToJacSolver(chain_ee)

        # building robot joint state
        self.num_joints = chain_ee.getNrOfJoints()

        self.joint_names = rospy.get_param("blue_hardware/joint_names")
        self.joint_names = self.joint_names[:self.num_joints]
        if self.debug:
          rospy.loginfo(self.joint_names)

        self.joints = kdl.JntArray(self.num_joints)

        # todo make seperate in abstract class
        self.ik = TRAC_IK(self.baselink,
                     self.endlink,
                     urdf,
                     0.01,
                     1e-4,
                    "Distance")
                    # "Manipulation2")
                    # "Manipulation1")

    def publish_ik_sol(self, target, joints):
        # target is a target pose object
        # joints is the starting joint seed
        seed_state = []
        for j in joints:
            seed_state.append(j)

        result = self.ik.CartToJnt(seed_state,
                               target.position.x, target.position.y, target.position.z,
                               target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w, )
        if not len(result) == self.num_joints:
            return
        self.command_to_joint_state(result)

    def command_to_joint_state(self, joints):
        start_pos = np.array(self.joints).copy()
        end_pos = np.array(joints).copy()
        joint_diff = end_pos - start_pos
        max_joint_diff = np.linalg.norm(joint_diff, np.inf)
        for j in range(self.resolution + 1):
            step = start_pos + (end_pos - start_pos) * cheb_points(self.resolution, j)
            self._pub_js_msg(np.array(step))
            rospy.sleep( np.abs(max_joint_diff) * duration * 1.0 / resolution)

    def _pub_js_msg(self, joints):
        # joints is an array of 7 joint angles
        msg = Float64MultiArray()
        msg.data = joints
        if self.debug:
            pass
            rospy.logerr("ik result")
            rospy.logerr(result)
        self.command_pub.publish(msg)
        self.command_pub_ctc.publish(msg)


    def update_joints(self, joint_msg):
        if self.first:
            return

        temp_joints = kdl.JntArray(self.num_joints)
        for i, n in enumerate(self.joint_names):
            index = joint_msg.name.index(n)
            temp_joints[i] = joint_msg.position[index]
        self.joints = temp_joints

    def __init__(self, debug=False):
        self.debug = debug
        if self.debug:
            self.debug_count = 0

        self._setup()

        self.target_pos = Pose()
        self.first = True

        self.command_pub = rospy.Publisher("blue_controllers/joint_position_controller/command", Float64MultiArray, queue_size=1)
        self.command_pub_ctc = rospy.Publisher("blue_controllers/joint_ctc/command", Float64MultiArray, queue_size=1)
        rospy.Subscriber("/joint_states", JointState, self.update_joints)

def main():
    rospy.init_node("blue_ik")
    b = BlueIK(debug=True)
    home = [0, 0, 0, 0, 0, 0, 0]
    drop_off = [0, 0, 0, 0, 0, 0, 0]
    gc = GripperClient()

    # TODO init prime sense
    # TODO create grasp service client
    rate = rospy.Rate(0.05)
    while not rospy.is_shutdow():
        b.command_to_joint_state(home)
        # TODO read in image from primesense
        # TODO make service call
        b.publish_ik_sol(from_service_call_PoseStamped message)
        gc.call_grip(1.0):
        b.command_to_joint_state(drop_off)
        gc.call_grip(0.0):

if __name__ == "__main__":
    main()
