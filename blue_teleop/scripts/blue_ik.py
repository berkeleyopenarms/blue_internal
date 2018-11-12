#!/usr/bin/env python
import rospy
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

class BlueIK:
    def _setup(self):
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
        msg = Float64MultiArray()
        msg.data = result
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

        ## TODO make publishing the result seperately
        self.publish_ik_sol(self.target_pose, self.joints)


    def command_callback(self, msg):
        trans = self.tfBuffer.lookup_transform(self.baselink, msg.header.frame_id, rospy.Time())
        msg = tf2_geometry_msgs.do_transform_pose(msg, trans);

        temp_target = Pose()

        temp_target.position.x = msg.pose.position.x # x
        temp_target.position.y = msg.pose.position.y # y
        temp_target.position.z = msg.pose.position.z # z

        temp_target.orientation.x = msg.pose.orientation.x # qx
        temp_target.orientation.y = msg.pose.orientation.y # qy
        temp_target.orientation.z = msg.pose.orientation.z # qz
        temp_target.orientation.w = msg.pose.orientation.w # qw

        self.target_pose = temp_target

        if self.first:
            self.first = False

    def __init__(self, debug=False):
        rospy.init_node("blue_ik")
        self.debug = debug
        if self.debug:
            self.debug_count = 0

        self._setup()

        self.target_pos = Pose()
        self.first = True

        rospy.Subscriber("pose_target/command", PoseStamped, self.command_callback)
        self.command_pub = rospy.Publisher("blue_controllers/joint_position_controller/command", Float64MultiArray, queue_size=1)
        self.command_pub_ctc = rospy.Publisher("blue_controllers/joint_ctc/command", Float64MultiArray, queue_size=1)
        rospy.Subscriber("/joint_states", JointState, self.update_joints)

def main():
    b = BlueIK(debug=True)
    rospy.spin()

if __name__ == "__main__":
    main()
