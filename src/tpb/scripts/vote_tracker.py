# !/usr/bin/env python
import rospy
import sys
import time
import math
import random
import twitch

from tpb.msg import tpb_command
from tpb.msg import tpb_reset
from tpb.msg import tpb_done

default_position_increment = 10  # cm
default_angle_increment = 15  # deg


class TPBVoteTracker:

    def __init__(self, side):
        self.side = side
        node_name = "tpb_"+side+"_vt"

        self.s_print("Initializing")
        rospy.init_node(node_name, anonymous=True)
        self.r = rospy.Rate(1) # 20hz

        command_string = "/"+node_name+"/tpb_command"
        self.command_pub = rospy.Publisher(command_string, tpb_command, queue_size=10)
        reset_string = "/"+node_name+"/reset_command"
        self.reset_pub = rospy.Publisher(reset_string, tpb_reset, queue_size=10)

        status_string = "/tpb_"+side+"_rm/tpb_status"
        self.status_sub = rospy.Subscriber(status_string, tpb_done, self.tpb_status_callback, queue_size=10)

        self.move_done = False

        # Connect to twitch
        self.t = twitch.Twitch()

        # for official
        # username = "twitchplaysbaxter"
        # key = "oauth:4cmfkqt4jwy63oqq0wiqflisd74s1n"

        # not official
        username = "btgibbons"
        key = "oauth:04parj4fmlbfnego06bktuhchtx493";

        self.t.twitch_connect(username, key);

        # 3DOF position end effector changing
        self.delta_pos = [0] * 3
        # final wrist joint angle changing
        self.delta_wrist = 0
        # close and open the grippers
        self.button = 0
        print "VT: Vote Tracker Initialized"

    def s_print(self, s):
        print self.side+"_VT: ", s

    # Callback for receipt of the done moving message
    def tpb_status_callback(self, tpbd):
        print "VT:Got TPD state"
        if tpbd.done is True:
            self.move_done = True
        print "VT:Gpt TPD state done"

    # resets all of the global tracking variables to 0
    def reset_tracker_variables(self):
        print "VT:resetting tracker variables"
        # 3DOF position end effector changing
        self.delta_pos = [0] * 3
        # final wrist joint angle changing
        self.delta_wrist = 0
        # close and open the grippers
        self.button = 0
        print "VT:tracker variables reset"

    # return a
    def validate_and_split(self, msg):
        print "VT:validating and splitting"
        ret = [False]

        if msg[0] != "!":
            print "missing !"
            return ret

        # break the message up after removing the !
        msg = msg[1:]
        msg_split = msg.split()

        if len(msg_split) < 2 or len(msg_split) > 3:
            print "ill formed message"
            return ret

        if len(msg_split) == 3:
            if msg_split[2].isdigit():
                msg_split = float(msg_split[2])
            else:
                print "third arg is non-numeric"
                return ret
        else:
            if "w" not in msg_split[1]:
                msg_split.append(default_position_increment)
            else:
                msg_split.append(default_angle_increment)

        # we got this far, we've broken it down and its legit.
        ret[0] = True
        ret.append(msg_split)
        print "VT:validated and split"
        return ret

    def send_home(self):
        print "VT:sending home"
        reset_msg = tpb_reset()
        reset_msg.reset = true
        self.reset_pub.publish(reset_msg)
        print "VT:sent home"

    def run(self):

        # main loop time
        while not rospy.is_shutdown():
            print "VT:loop iteration"
            # get all of the twitch votes since last loop
            new_messages = self.t.twitch_recieve_messages()

            if new_messages:
                print "VT:Got new messages"
                # analyze them all
                for message in new_messages:
                    print "VT:analysing a message"
                    msg = message['message'].lower()
                    username = message['username'].lower()
                    msg_split = self.validate_and_split(msg)
                    if not msg_split[0]:
                        # skip this loop iteration, crap message
                        break
                    else:
                        msg_split = msg_split[1]
                        # check for reset
                        if msg_split[0] == "reset" and username == "btgibbons":
                            # TODO double check this isn't an issue
                            send_home()
                        else:
                            if msg_split[0] in [self.side[0], ""+self.side[0]+self.side[0], self.side]:
                                print "side detected"
                                if msg_split[1] in ["f", "forward"]:
                                    self.delta_pos[0] += msg_split[2]
                                elif msg_split[1] in ["b", "back"]:
                                    self.delta_pos[0] -= msg_split[2]
                                elif msg_split[1] in ["l", "left"]:
                                    self.delta_pos[1] += msg_split[2]
                                elif msg_split[1] in ["r", "right"]:
                                    self.delta_pos[1] -= msg_split[2]
                                elif msg_split[1] in ["u", "up"]:
                                    self.delta_pos[2] += msg_split[2]
                                elif msg_split[1] in ["d", "down"]:
                                    self.delta_pos[2] -= msg_split[2]

                                elif msg_split[1] in ["cw", "clockwise"]:
                                    self.delta_wrist += msg_split[2]
                                elif msg_split[1] in ["ccw", "counterclockwise"]:
                                    self.delta_wrist -= msg_split[2]

                                elif msg_split[1] in ["c", "close"]:
                                    self.button += 1
                                elif msg_split[1] in ["o", "open"]:
                                    self.button -= 1

            # Now that all of the messages have been tallied, form appropriate command messages and send them
            if self.move_done is True:
                print "VT:move done is true"
                command = tpb_command()
                command.delta_x = self.delta_pos[0]
                command.delta_y = self.delta_pos[1]
                command.delta_z = self.delta_pos[2]
                command.delta_wrist = self.delta_wrist
                if self.button > 0:
                    command.gripper_state = 1
                else:
                    command.gripper_state = 0

                print "VT:publishing command"
                self.command_pub.publish(command)

                # reset in prep for the next call
                self.reset_tracker_variables()
                print "VT:setting move done to false"
                self.move_done = False

            # sleep off the remainder of the frequency duration
            print "VT:sleeping off the remainder"
            self.r.sleep()
