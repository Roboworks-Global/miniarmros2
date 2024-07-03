#!/usr/bin/env python
# coding=utf-8
# Copyright (c) 2011, Willow Garage, Inc.
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
#  * Neither the name of {copyright_holder} nor the names of its
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
#
# Author: Darby Lim

import os
import select
import sys
import rclpy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1


msg = """
Control your stepperArm.
---------------------------
q/a : Arm_A_Joint increase/decrease 1 step(rad)
w/s : Arm_B_Joint increase/decrease 1 step(rad)
e/d : Arm_C_Joint increase/decrease 1 step(rad)
r/f : Arm_end_point foward/back 1 step(mm)
t/g : Arm_end_point up/down 1 step(mm)
y/h : Arm_hand tighten/loosen 1 step(rad)

Control Your carrrrrrrrrr!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

z/x : increase/decrease max speeds by 10%
c/v : increase/decrease only linear speed by 10%
b/n : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly
m : switch to OmniMode/CommonMode

CTRL-C to quit
"""
e = """
Communications Failed
"""
#键值对应移动/转向方向
moveBindings = {
        'i':( 1, 0),
        'o':( 1,-1),
        'j':( 0, 1),
        'l':( 0,-1),
        'u':( 1, 1),
        ',':(-1, 0),
        '.':(-1, 1),
        'm':(-1,-1),
           }

#键值对应速度增量
speedBindings={
        'z':(1.1,1.1),
        'x':(0.9,0.9),
        'c':(1.1,1),
        'v':(0.9,1),
        'b':(1,  1.1),
        'n':(1,  0.9),
          }

armJointBindings={
    'q': (0, 0.1),
    'a': (0, -0.1),
    'w': (1, 0.1),
    's': (1, -0.1),
    'e': (2, 0),
    'd': (2, 0),
}

armPositionBindings={
    'r': (0, 0),
    'f': (0, 0),
    't': (1, 0),
    'g': (1, 0),
    'y': (2, 0),
    'h': (2, 0),
}

#获取键值函数
speed = 0.2 #默认移动速度 m/s
turn  = 1   #默认转向速度 rad/
def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

#以字符串格式返回当前速度
def print_vels(speed, turn):
    print('currently:\tspeed {0}\t turn {1} '.format(
        speed,
        turn))

def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('wheeltec_keyboard')
    pub = node.create_publisher(Twist, 'cmd_vel', qos)
    arm_pub = node.create_publisher(JointState, 'joint_states', qos)

    speed = 0.2 #默认移动速度 m/s
    turn  = 1.0   #默认转向速度 rad/
    x      = 0.0   #前进后退方向
    th     = 0.0   #转向/横向移动方向
    count  = 0.0   #键值不再范围计数
    target_speed = 0.0 #前进后退目标速度
    target_turn  = 0.0 #转向目标速度
    target_HorizonMove = 0.0 #横向移动目标速度
    control_speed = 0.0 #前进后退实际控制速度
    control_turn  = 0.0 #转向实际控制速度
    control_HorizonMove = 0.0 #横向移动实际控制速度
    Omni = 0

    arm_state = JointState()
    twist = Twist()

    try:
        print(msg)
        print(print_vels(speed, turn))
        while(1):
            key = get_key(settings)
            #切换是否为全向移动模式，全向轮/麦轮小车可以加入全向移动模式
            if key=='m':               
                Omni=~Omni
                if Omni: 
                    print("Switch to OmniMode")
                    moveBindings['.']=[-1,-1]
                    moveBindings['m']=[-1, 1]
                else:
                    print("Switch to CommonMode")
                    moveBindings['.']=[-1, 1]
                    moveBindings['m']=[-1,-1]
            
            #判断键值是否在移动/转向方向键值内
            if key in moveBindings.keys():
                x  = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0

            #判断键值是否在速度增量键值内
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn  = turn  * speedBindings[key][1]
                count = 0
                print(print_vels(speed,turn)) #速度发生变化，打印出来

            #空键值/'k',相关变量置0
            elif key == ' ' or key == 'k' :
                x  = 0
                th = 0.0
                control_speed = 0.0
                control_turn  = 0.0
                HorizonMove   = 0.0

            elif key in armJointBindings.keys():
                JointA = 


            #长期识别到不明键值，相关变量置0
            else:
                count = count + 1
                if count > 4:
                    x  = 0
                    th = 0.0
                if (key == '\x03'):
                    break

           #根据速度与方向计算目标速度
            target_speed = speed * x
            target_turn  = turn * th
            target_HorizonMove = speed*th

            #平滑控制，计算前进后退实际控制速度
            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.1 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.1 )
            else:
                control_speed = target_speed

            #平滑控制，计算转向实际控制速度
            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.5 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.5 )
            else:
                control_turn = target_turn

            #平滑控制，计算横向移动实际控制速度
            if target_HorizonMove > control_HorizonMove:
                control_HorizonMove = min( target_HorizonMove, control_HorizonMove + 0.1 )
            elif target_HorizonMove < control_HorizonMove:
                control_HorizonMove = max( target_HorizonMove, control_HorizonMove - 0.1 )
            else:
                control_HorizonMove = target_HorizonMove
         
            if Omni==0:
                twist.linear.x  = control_speed
                twist.angular.z = control_turn
            else:
                twist.linear.x  = control_speed
                twist.linear.y = control_HorizonMove
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = 0.0
            
            pub.publish(twist)
            arm_pub.publish(arm_state)

    except Exception as e:
        print(e)

    finally:
        pub.publish(twist)
        arm_pub.publish(arm_state)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
