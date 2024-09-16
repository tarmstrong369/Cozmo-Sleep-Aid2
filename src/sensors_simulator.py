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
    key_pub = rospy.Publisher("/serial/keyboard", Bool, queue_size=1)
    btn_pub = rospy.Publisher("/serial/button", Bool, queue_size=1)
    chair_pub = rospy.Publisher("/bluetooth/chair", Bool, queue_size=1)
    r = rospy.Rate(1)
    sitting = True
    typing = True
    state_changed = True
    button_pressed = False
    button_time = time.time()
    autorun = False
    auto_time_sit = [20, 10, 40, 5, 30, 15, 15]
    auto_time_key = [20, 10, 40, 5, 30, 15, 15]
    sit_time = time.time()
    key_time = time.time()
    sit_iter = 0
    key_iter = 0

    logger = logger_minimal.Logger('sim')
    logger.log('START', True)

    pygame.init()
    x_size = 300
    y_size = 300
    window = pygame.display.set_mode((x_size,y_size))
    font = pygame.font.SysFont(None, 20)
    text1 = font.render("k toggles keyboard active", True, (255, 255, 255))
    textRect1 = text1.get_rect()
    textRect1.center = (x_size // 2, y_size // 9)
    text2 = font.render("s toggles sit/stand", True, (255, 255, 255))
    textRect2 = text2.get_rect()
    textRect2.center = (x_size // 2, (y_size // 9)*2 )
    text3 = font.render("b flags button press", True, (255, 255, 255))
    textRect3 = text3.get_rect()
    textRect3.center = (x_size // 2, (y_size // 9)*3 )
    text4 = font.render("a toggles auto mode", True, (255, 255, 255))
    textRect4 = text4.get_rect()
    textRect4.center = (x_size // 2, (y_size // 9)*4 )
    textBtn = font.render("Button pressed!", True, (0,255,0))
    textRectBtn = textBtn.get_rect()
    textRectBtn.center = (x_size // 2, (y_size // 9)*8 )

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
                if event.key == pygame.K_b:
                    btn_pub.publish(True)
                    button_pressed = True
                    state_changed = True
                    button_time = time.time()
                    logger.log("Button pressed")
                if event.key == pygame.K_k:
                    typing = not typing
                    state_changed = True
                    logger.log("Typing is {}".format(typing))
                if event.key == pygame.K_a:
                    autorun = not autorun
                    state_changed = True
                    logger.log("Automatic change is {}".format(autorun))
        
        if autorun:
            if time.time() - sit_time > auto_time_sit[sit_iter]*60:
                sit_time = time.time()
                sit_iter = sit_iter + 1
                if sit_iter > len(auto_time_sit)-1:
                    sit_iter = 0
                sitting = not sitting
                state_changed = True
            if time.time() - key_time > auto_time_key[key_iter]*60:
                key_time = time.time()
                key_iter = key_iter + 1
                if key_iter > len(auto_time_key)-1:
                    key_iter = 0
                typing = not typing
                state_changed = True
        if button_pressed:
            if time.time() - button_time > 5:
                button_pressed = False
                state_changed = True

        if typing:
            key_pub.publish(True)
        chair_pub.publish(sitting)
        if state_changed:
            text5 = font.render("Sitting: {}".format(sitting), True, (255, 255, 255))
            textRect5 = text5.get_rect()
            textRect5.center = (x_size // 2, (y_size // 9)*5)
            text6 = font.render("Typing: {}".format(typing), True, (255, 255, 255))
            textRect6 = text6.get_rect()
            textRect6.center = (x_size // 2, (y_size // 9)*6)
            text7 = font.render("Auto Mode: {}".format(autorun), True, (255, 255, 255))
            textRect7 = text7.get_rect()
            textRect7.center = (x_size // 2, (y_size // 9)*7)
            window.fill((0,0,0))
            window.blit(text1, textRect1)
            window.blit(text2, textRect2)
            window.blit(text3, textRect3)
            window.blit(text4, textRect4)
            window.blit(text5, textRect5)
            window.blit(text6, textRect6)
            window.blit(text7, textRect7)
            if button_pressed:
                window.blit(textBtn, textRectBtn)
            pygame.display.update()
            state_changed = False
        #windowclock.tick
        r.sleep()
