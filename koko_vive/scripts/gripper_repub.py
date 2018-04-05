import rospy

import actionlib

from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal,
)

class GripperClient(object):
    def __init__(self, gripper):
        ns = 'koko_controllers/' + 'gripper_controller/'
        self._client = actionlib.SimpleActionClient(
            ns + "gripper_action",
            GripperCommandAction,
        )
        self._goal = GripperCommandGoal()

        # Wait 10 Seconds for the gripper action server to start or exit
        if not self._client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Exiting - %s Gripper Action Server Not Found" %
                         (gripper.capitalize(),))
            rospy.signal_shutdown("Action Server not found")
            sys.exit(1)
        self.clear()
        rospy.Subscriber("/right_trigger", Float32, self.goal_callback, queue_size=1)

    def command(self, position, effort):
        self._goal.command.position = position
        self._goal.command.max_effort = effort
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=5.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))
        return self._client.get_result()

    def goal_callback(self, msg):
        self._goal.command.position = msg.data
        self._client.send_goal(self._goal)
        gc.wait()

    def set_effort(self, eff):
        self._goal.command.max_effort = eff

    def clear(self):
        self._goal = GripperCommandGoal()


def main():
    rospy.init_node("gripper_action_client")
    gc = GripperClient(gripper)
    gc.set_effort(effort=1.5)
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        r.sleep()

if __name__ == "__main__":
    main()
