# !/usr/bin/env python
import rospy
import sys
import time
import math
import baxter_interface
import ik_solver

from tpb.msg import tpb_command
from tpb.msg import tpb_reset
from tpb.msg import tpb_done

position_increment_cap = 0.25
angle_increment_cap = 3.1415/6


def mm_clamp(my_value, min_value, max_value):
    return max(min(my_value, max_value), min_value)


class TPBRobotMover:

    def __init__(self, side):
        self.side = side
        node_name = 'tpb_'+side + '_rm'
        rospy.init_node(node_name, anonymous=False)
        self.r = rospy.Rate(1)  # 20hz
        # enable the robot
        self.rs = baxter_interface.RobotEnable()
        self.rs.enable()

        # arm interface
        self.limb = baxter_interface.Limb(side)
        self.gripper = baxter_interface.Gripper(side)
        speed_p_name = side+'_arm_speed'
        arm_speed = rospy.get_param(speed_p_name, 2.5)
        self.limb.set_joint_position_speed(arm_speed)

        # subscriptions and
        command_sub_name = "/tpb_"+side+"_vt/tpb_command"
        self.command_sub = rospy.Subscriber(command_sub_name, tpb_command, self.command_callback, queue_size=10)
        reset_sub_name = "/tpb_"+side+"_vt/reset_command"
        self.reset_sub = rospy.Subscriber(reset_sub_name, tpb_reset, self.reset_callback, queue_size=10)

        # publications
        status_pub_name = "/"+node_name+"/tpb_status"
        self.status_pub = rospy.Publisher(status_pub_name, tpb_done, queue_size=10)

        # not sure why this is needed, but if it is absent, then the vote tracker will never receive the status pub
        time.sleep(1.0)

        self.home_position()

        self.pose = self.limb.endpoint_pose()
        self.pos = self.pose.popitem()
        self.orient = self.pose.popitem()
        self.delta = [0] * 4
        self.button = 0
        self.last_button = 0
        self.wrist_angle = 0  # radians
        self.new_command = False
        self.new_reset_command = False

    def disable(self):
        self.rs.disable()

    def home_position(self):
        self.limb.move_to_neutral()
        t = tpb_done()
        t.done = True
        self.status_pub.publish(t)

    def command_callback(self, data):
        # store received new deltas, convert to m and rad
        self.delta[0] = data.delta_x / 100.0
        self.delta[1] = data.delta_y / 100.0
        self.delta[2] = data.delta_z / 100.0
        self.delta[3] = math.radians(data.delta_wrist)
        self.button = data.gripper_state
        self.new_command = True

    def reset_callback(self, data):
        if data.reset is True:
            self.home_position()

    def move(self):
        # only try to move if there is a requested delta
        if not all(v == 0 for v in self.delta):
            xd = mm_clamp(self.delta[0], -1 * position_increment_cap, position_increment_cap)
            yd = mm_clamp(self.delta[1], -1 * position_increment_cap, position_increment_cap)
            zd = mm_clamp(self.delta[2], -1 * position_increment_cap, position_increment_cap)
            wa = mm_clamp(self.delta[3], -1 * angle_increment_cap, angle_increment_cap)

            new_end_point = baxter_interface.Limb.Point(self.pos[1].x + xd, self.pos[1].y + yd, self.pos[1].z + zd)
            limb_joints = ik_solver.ik_solve(self.side, new_end_point, self.orient[1])

            if limb_joints is not -1:
                print "Moving Limb To: "
                print new_end_point
                print time.time()
                print limb_joints
                # replace wrist with user commanded
                self.wrist_angle += wa
                wrist_joint_string = self.side+'_w2'
                limb_joints[wrist_joint_string] = self.wrist_angle
                self.limb.move_to_joint_positions(limb_joints)
                print time.time()

    def run(self):
        # main loop
        while not rospy.is_shutdown():
            print "rm: run iteration"
            # if the limb needs to be moved, move it
            if self.new_command is True:
                # do the actual move.  This returns when the physical move is complete
                print "New Command!!"
                self.move()
                # publish a message informing subscribers that the move is done
                t = tpb_done()
                t.done = True
                self.status_pub.publish(t)
                # reset variables
                self.delta = [0] * 4
                self.new_command = False

            # modify gripper state if necessary
            if self.last_button != self.button:
                if self.button == 0:
                    self.gripper.close()
                else:
                    self.gripper.open()
            self.last_button = self.button

            # sleep for the remainder of the loop time
            self.r.sleep()