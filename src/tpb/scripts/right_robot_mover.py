# !/usr/bin/env python

import robot_mover

if __name__ == '__main__':
    tpbrm = robot_mover.TPBRobotMover("right")
    tpbrm.run()
    tpbrm.disable()
