#!/usr/bin/env python3

import sys
import logger_minimal
import os
import signal
import rospy
import time
import pygame
from std_msgs.msg import Bool

# Callback to handle SIGINT and SIGTERM
def shutdown_callback(_1, _2):
    # Log, then shut down remaining
    logger.log('STOP')
    rospy.signal_shutdown("User requested shutdown.")
    pygame.quit()
    sys.exit(0)

if __name__ == '__main__':
    # Setup callbacks, ROS and logging
    signal.signal(signal.SIGINT, shutdown_callback)
    signal.signal(signal.SIGTERM, shutdown_callback)
    rospy.init_node('sim', anonymous=True)
    chair_pub = rospy.Publisher("/bluetooth/chair", Bool, queue_size=1)
    r = rospy.Rate(1)
    sitting = True
    state_changed = True
    autorun = False
    auto_time_stand = [20, 10, 40, 5, 30, 15, 15]
    start_time = time.time()
    auto_iter = 0

    logger = logger_minimal.Logger('sim')
    logger.log('START', True)

    pygame.init()
    x_size = 300
    y_size = 300
    window = pygame.display.set_mode((x_size,y_size))
    font = pygame.font.SysFont(None, 20)
    text1 = font.render("s toggles sit/stand", True, (255, 255, 255))
    textRect1 = text1.get_rect()
    textRect1.center = (x_size // 2, (y_size // 5))
    text2 = font.render("a toggles autorun", True, (255, 255, 255))
    textRect2 = text2.get_rect()
    textRect2.center = (x_size // 2, (y_size // 5)*2)

    windowclock = pygame.time.Clock()

    
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                logger.log('STOP')
                rospy.signal_shutdown("User requested shutdown.")
                pygame.quit()
                sys.exit(0)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_s:
                    sitting = not sitting
                    state_changed = True
                    logger.log("Sitting is {}".format(sitting))
                elif event.key == pygame.K_a:
                    autorun = not autorun
                    state_changed = True
                    logger.log("Automatic change is {}".format(autorun))
        
        if autorun:
            if time.time() - start_time > auto_time_stand[auto_iter]*60:
                start_time = time.time()
                auto_iter = auto_iter + 1
                if auto_iter > len(auto_time_stand)-1:
                    auto_iter = 0
                sitting = not sitting
                state_changed = True

        chair_pub.publish(sitting)
        if state_changed:
            text3 = font.render("Sitting: {}".format(sitting), True, (255, 255, 255))
            textRect3 = text3.get_rect()
            textRect3.center = (x_size // 2, (y_size // 5)*3)
            text4 = font.render("Auto Mode: {}".format(autorun), True, (255, 255, 255))
            textRect4 = text4.get_rect()
            textRect4.center = (x_size // 2, (y_size // 5)*4)
            window.fill((0,0,0))
            window.blit(text1, textRect1)
            window.blit(text2, textRect2)
            window.blit(text3, textRect3)
            window.blit(text4, textRect4)
            pygame.display.update()
            state_changed = False
        #windowclock.tick
        r.sleep()
