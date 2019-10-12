#!/usr/bin/env python3
from __future__ import print_function
import sys
print(sys.version)
import pygame
import time
import roslib
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Bool

def EntradaDeTeclado():
    pygame.init()
    screen_size = (400, 200)
    screen = pygame.display.set_mode(screen_size)
    pub_L_command = rospy.Publisher('keyboard_L_command', Int8, queue_size=10)
    pub_R_command = rospy.Publisher('keyboard_R_command', Int8, queue_size=10)
    pub_break_command = rospy.Publisher('break_command', Bool, queue_size=10)

    rospy.init_node('keyboard_pygame_control', anonymous=True)

    rate = rospy.Rate(25)
    shotdown = False;

    while not shotdown:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                shotdown = True;
        if rospy.is_shutdown():
            shotdown = True;

        pressed = pygame.key.get_pressed()
        if pressed[pygame.K_ESCAPE]:
            shotdown = True;

        dutycycle_L = 0
        dutycycle_R = 0

        if pressed[pygame.K_w]:
            dutycycle_L = 63
            dutycycle_R = 63

        if pressed[pygame.K_a]:
            dutycycle_L = dutycycle_L + 63

        if pressed[pygame.K_d]:
            dutycycle_R = dutycycle_R + 63

        if pressed[pygame.K_s]:
            dutycycle_L = dutycycle_L - 63
            dutycycle_R = dutycycle_R - 63

        if pressed[pygame.K_LEFT]:
            dutycycle_R = 126
            dutycycle_L = -126
        if pressed[pygame.K_RIGHT]:
            dutycycle_R = -126
            dutycycle_L = 126
        if pressed[pygame.K_UP]:
            dutycycle_R = 126
            dutycycle_L = 126
        if pressed[pygame.K_DOWN]:
            dutycycle_R = -126
            dutycycle_L = -126

        if pressed[pygame.K_SPACE]:
            break_command = True
        else:
        	break_command = False

        pub_L_command.publish(int(dutycycle_L))
        pub_R_command.publish(int(dutycycle_R))
        pub_break_command.publish(break_command)

        print(dutycycle_L, dutycycle_R)

        rate.sleep()

    print("end")

if __name__ == '__main__':
    try:
        EntradaDeTeclado()
    except rospy.ROSInterruptException:
        pass
