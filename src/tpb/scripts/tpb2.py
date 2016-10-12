#!/usr/bin/env python
import rospy
import sys
import time
import math
import random
import baxter_interface
import ik_solver
import twitch

# how far each position command will make the gripper move (in meters)
default_position_increment = 0.10
# wrist angle change command default (degrees)
default_angle_increment = 10.0
# how long (in sec) to wait between loop executions, longer will result in a less personalized democracy
loop_sleep_period = 0.10

right_min_x = 0.2
right_min_z = -0.3
left_min_x = 0.2
left_min_z = -0.3

position_increment_cap = 0.25


def clamp(my_value, min_value, max_value):
    return max(min(my_value, max_value), min_value)


class sn_baxter:
    def __init__(self):
        # initialize new ros node
        rospy.init_node('tpb_baxter', anonymous=False)
        # enable the robot
        self.rs = baxter_interface.RobotEnable()
        self.rs.enable()

        # right arm
        self.right = baxter_interface.Limb('right')
        self.right_gripper = baxter_interface.Gripper('right')
        # set arm movement speed to maximum
        self.right.set_joint_position_speed(2.5)
        self.right_pose = self.right.endpoint_pose()
        self.right_pos = self.right_pose.popitem()
        self.right_orient = self.right_pose.popitem()

        # left arm
        self.left = baxter_interface.Limb('left')
        self.left_gripper = baxter_interface.Gripper('left')
        # set arm movement speed to maximum
        self.left.set_joint_position_speed(2.5)
        self.left_pose = self.left.endpoint_pose()
        self.left_pos = self.left_pose.popitem()
        self.left_orient = self.left_pose.popitem()

        # only position for now, no orientation change
        self.right_delta_pos = [0] * 3
        self.left_delta_pos = [0] * 3
        self.left_delta_wrist = 0
        self.right_delta_wrist = 0

        self.right_wrist_angle = 0
        self.left_wrist_angle = 0

        # start them open at 0
        self.right_button = 0
        self.left_button = 0

        # Connect to twitch
        self.t = twitch.Twitch();

        # for official
        #username = "twitchplaysbaxter"
        #key = "oauth:4cmfkqt4jwy63oqq0wiqflisd74s1n"

        # not official
        username = "btgibbons"
        key = "oauth:04parj4fmlbfnego06bktuhchtx493";

        self.t.twitch_connect(username, key);

        self.home_position()

    def run(self):

        while 1:
            # Get twitch messages since last time and process them
            new_messages = self.t.twitch_recieve_messages();
            if new_messages:

                # get the current endpoint poses
                self.right_pose = self.right.endpoint_pose()
                self.right_pos = self.right_pose.popitem()
                self.right_orient = self.right_pose.popitem()

                self.left_pose = self.left.endpoint_pose()
                self.left_pos = self.left_pose.popitem()
                self.left_orient = self.left_pose.popitem()

                # reset movement sums so they'll work democratically
                self.right_delta_pos = [0] * 3
                self.left_delta_pos = [0] * 3

                self.left_delta_wrist = 0
                self.right_delta_wrist = 0

                # reset button sums so they can be used democratically
                self.right_button = 0
                self.left_button = 0

                for message in new_messages:
                    msg = message['message'].lower()
                    username = message['username'].lower()

                    if msg == "!reset" and username == "btgibbons":
                        self.home_position()
                        break

                    if msg[0] != "!":
                        print "message not starting with !"
                        print msg[0]
                        break

                    # break the message up after removing the !
                    msg = msg[1:]
                    msg_split = msg.split()

                    print msg
                    print msg_split

                    if len(msg_split) < 2 or len(msg_split) > 3:
                        print "ill formed message"
                        break

                    increment = default_position_increment
                    if len(msg_split) == 3 and msg_split[2].isdigit():
                        increment = float(msg_split[2]) / 100.0
                        print increment

                    if msg_split[0] in ["r", "rr", "right"]:
                        print "right detected"
                        if msg_split[1] in ["f", "forward"]:
                            self.right_delta_pos[0] += increment
                        elif msg_split[1] in ["b", "back"]:
                            self.right_delta_pos[0] -= increment
                        elif msg_split[1] in ["l", "left"]:
                            self.right_delta_pos[1] += increment
                        elif msg_split[1] in ["r", "right"]:
                            self.right_delta_pos[1] -= increment
                        elif msg_split[1] in ["u", "up"]:
                            self.right_delta_pos[2] += increment
                        elif msg_split[1] in ["d", "down"]:
                            self.right_delta_pos[2] -= increment

                        elif msg_split[1] in ["cw", "clockwise"]:
                            self.right_delta_wrist += default_angle_increment
                        elif msg_split[1] in ["ccw", "counterclockwise"]:
                            self.right_delta_wrist -= default_angle_increment

                        elif msg_split[1] in ["c", "close"]:
                            self.right_button += 1
                        elif msg_split[1] in ["o", "open"]:
                            self.right_button -= 1

                    elif msg_split[0] in ["l", "ll", "left"]:
                        print "left detected"
                        if msg_split[1] in ["f", "forward"]:
                            self.left_delta_pos[0] += increment
                        elif msg_split[1] in ["b", "back"]:
                            self.left_delta_pos[0] -= increment
                        elif msg_split[1] in ["l", "left"]:
                            self.left_delta_pos[1] += increment
                        elif msg_split[1] in ["r", "right"]:
                            self.left_delta_pos[1] -= increment
                        elif msg_split[1] in ["u", "up"]:
                            self.left_delta_pos[2] += increment
                        elif msg_split[1] in ["d", "down"]:
                            self.left_delta_pos[2] -= increment

                        elif msg_split[1] in ["cw", "clockwise"]:
                            self.left_delta_wrist += default_angle_increment
                        elif msg_split[1] in ["ccw", "counterclockwise"]:
                            self.left_delta_wrist -= default_angle_increment

                        elif msg_split[1] in ["c", "close"]:
                            self.left_button += 1
                        elif msg_split[1] in["o", "open"]:
                            self.left_button -= 1

                    else:
                        print "unrecognized command!"
                        break
                        # end of the for new messages

                # right arm movement
                if not all(v == 0 for v in self.right_delta_pos) or self.right_delta_wrist != 0:
                    rxd = clamp(self.right_delta_pos[0], -1 * position_increment_cap, position_increment_cap)
                    ryd = clamp(self.right_delta_pos[1], -1 * position_increment_cap, position_increment_cap)
                    rzd = clamp(self.right_delta_pos[2], -1 * position_increment_cap, position_increment_cap)

                    new_r_point = baxter_interface.Limb.Point(self.right_pos[1].x + rxd, self.right_pos[1].y + ryd, self.right_pos[1].z + rzd)
                    right_limb_joints = ik_solver.ik_solve('right', new_r_point, self.right_orient[1])

                    if right_limb_joints is not -1:
                        print "Moving Right To: "
                        print new_r_point
                        print time.time()
                        print right_limb_joints
                        # replace wrist with user commanded
                        self.right_wrist_angle += self.right_delta_wrist
                        right_limb_joints["right_w2"] = math.radians(self.right_wrist_angle)
                        self.right.move_to_joint_positions(right_limb_joints)
                        print time.time()

                # left arm movement
                if not all(v == 0 for v in self.left_delta_pos) or self.left_delta_wrist != 0:
                    lxd = clamp(self.left_delta_pos[0], -1 * position_increment_cap, position_increment_cap)
                    lyd = clamp(self.left_delta_pos[1], -1 * position_increment_cap, position_increment_cap)
                    lzd = clamp(self.left_delta_pos[2], -1 * position_increment_cap, position_increment_cap)

                    new_l_point = baxter_interface.Limb.Point(self.left_pos[1].x + lxd, self.left_pos[1].y + lyd, self.left_pos[1].z + lzd)
                    left_limb_joints = ik_solver.ik_solve('left', new_l_point, self.left_orient[1])
                    if left_limb_joints is not -1:
                        print "Moving Left To: "
                        print new_l_point
                        print time.time()
                        print left_limb_joints
                        # replace wrist with user commanded
                        self.left_wrist_angle += self.left_delta_wrist
                        left_limb_joints["left_w2"] = math.radians(self.left_wrist_angle)
                        self.left.move_to_joint_positions(left_limb_joints)
                        print time.time()

                # Check gripper tallies and respond accordingly
                if self.right_button > 0:
                    self.right_gripper.close()
                elif self.right_button < 0:
                    self.right_gripper.open()
                    # else 0 do nothing, right gripper remains the same

                if self.left_button > 0:
                    self.left_gripper.close()
                elif self.left_button < 0:
                    self.left_gripper.open()
                    # else 0 do nothing, left gripper remains the same
            else:
                print "No new messages"

    def home_position(self):
        print "Homing"
        self.right.move_to_neutral()
        self.left.move_to_neutral()

    def disable(self):
        self.rs.disable()


if __name__ == '__main__':
    s = sn_baxter()
    time.sleep(5.0)
    s.run()
    s.disable()
