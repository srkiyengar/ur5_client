__author__ = 'srkiyengar'

import pygame
from datetime import datetime
import reflex
import screen_print as sp
import time



METHOD = 1

POS_ERROR = 20


# Define some colors
BLACK    = (   0,   0,   0)
WHITE    = ( 255, 255, 255)

SCAN_RATE = 20                    #1 (one) second divided by scan rate is the loop checking if the main program should stop



class gripper():

    def __init__(self):

        # Set up a logger with output level set to debug; Add the handler to the logger

        #Create Palm object
        self.palm = reflex.reflex_sf() # Reflex object ready

        for i in range(1,5,1):
            lowest_position = self.palm.finger[i]["lower_limit"]
            highest_position = self.palm.finger[i]["upper_limit"]
            init_position = self.palm.finger[i]["initial_position"]

            calibrate = False

            if (i == 1 or i == 3):
                a = lowest_position - POS_ERROR
                b= highest_position + POS_ERROR
                if a >= init_position or init_position >= b:
                    print('Servo {} Initial Position {} not between Lower Limit {} and Upper Limit {}'.format(\
                        i,init_position,lowest_position,highest_position))
                    #calibrate = 1
            elif (i == 2):
                a = lowest_position + POS_ERROR
                b = highest_position - POS_ERROR
                if a <= init_position or init_position <= b:
                    print('Servo {} Initial Position {} not between Lower Limit {} and Upper Limit {}'.format(\
                        i,init_position,lowest_position,highest_position))
                    #calibrate = 1

            # calibration is a must after every start.

        pygame.init()
        # Set the width and height of the screen [width,height]
        size = [500, 700]
        screen = pygame.display.set_mode(size)
        pygame.display.set_caption("Reflex_SF Calibration")

        # Used to manage how fast the screen updates
        clock = pygame.time.Clock()

        # for print in Pygame screen object
        textPrint = sp.TextPrint()

        my_key_controller = reflex.key_reflex_controller(self.palm)

        calibrate = False


        key_ring={}
        key_ring['301']= 0  # 301 is Caps lock. This will be displayed in the screen  Caps lock = 1 + keys are the command set
        key_pressed = 0     # key press and release will happen one after another
        key_released = 0

        file_ring={}        # to make sure we close any file created only if it is not closed

        method = METHOD
        # Calibration
        while calibrate is False:
            screen.fill(WHITE)
            textPrint.reset()

            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    key_pressed = event.key
                    print("Key Ascii Value {} Pressed".format(key_pressed))
                    key_ring[str(key_pressed)] = 1
                    if key_ring['301'] == 1:    # Caps lock is 1
                        if my_key_controller.set_key_press(key_pressed) == 1:
                            calibrate = True
                elif event.type == pygame.KEYUP:
                    key_released = event.key
                    print("Key Ascii Value {} Released".format(key_released))
                    key_ring[str(key_released)] = 0
                else:
                    pass # ignoring other non-logitech joystick event types

            textPrint.Screenprint(screen, "Caps Lock should be 1 to accept any of the keys")
            textPrint.Yspace()
            textPrint.Yspace()
            textPrint.Screenprint(screen,"Caps Lock Key set to {}".format(key_ring['301']))
            textPrint.Yspace()
            textPrint.Screenprint(screen, "Finger 1 - Press 'q' to move up")
            textPrint.Yspace()
            textPrint.Screenprint(screen, "Finger 1 - Press 'a' to move down")
            textPrint.Yspace()
            textPrint.Screenprint(screen, "Finger 2 - Press 'w' to move up")
            textPrint.Yspace()
            textPrint.Screenprint(screen, "Finger 2 - Press 's' to move down")
            textPrint.Yspace()
            textPrint.Screenprint(screen, "Finger 3 - Press 'e' to move up")
            textPrint.Yspace()
            textPrint.Screenprint(screen, "Finger 3 - Press 'd' to move down")
            textPrint.Yspace()
            textPrint.Screenprint(screen, "Pre Shape - Press 'r' to move closer")
            textPrint.Yspace()
            textPrint.Screenprint(screen, "Pre Shape - Press 'f' to move away")
            textPrint.Yspace()

            textPrint.Screenprint(screen, "Press 'c' when calibration is complete")

            # Go ahead and update the screen with what we've drawn.
            pygame.display.flip()

            # Limit to 20 frames per second OR 50 ms scan rate - 1000/20 = 50 ms Both display and checking of Joystick;
            clock.tick(SCAN_RATE)

        # Calibration completed
        pygame.quit()
        return

    def move_to_start(self):

        self.palm.move_to_lower_limits()
        gp_servo = self.palm.read_palm_servo_positions()
        print(gp_servo)
        return



if __name__ == "__main__":

    a = gripper()
    b =5



