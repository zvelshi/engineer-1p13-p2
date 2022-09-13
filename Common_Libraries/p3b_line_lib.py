"""
This code represents the library that students will reference to complete DESIGN STUDIO activities for Project 3
All code is currently set up to run on a raspberry pi ONLY.

Libaries were split into two due to the backend operation of the simulation. P3a_lib.py shall be used in the lab and
P3b_lib.py in the design studio.

During the design studio, it is assumed that students have already gone through the activity of determining the bin ID
and bottle mass. Given the variability in solutions that students may come up with, the intent of this library is to
first provide them with bin IDs and bottle masses as that was already been evaluated, and to ensure that all students have similar starting points.

Items to mention to students
1. Rotation limits base +/- 175 deg, shoulder +/- 90 deg, elbow +90 -80 deg, wrist +/-170, gripper 0(open)-1(close)
2. P3a_lib.py shall be used in the lab and P3b_lib.py in the design studio
"""

# Import all required libraries
import sys
sys.path.append('../')

#import numpy as np
import time
import os
import math

import RPi.GPIO as GPIO

from Common_Libraries.QBot2e_Lib import *
from Common_Libraries.quanser_image_lib import * # Has CV and numpy imported already

GPIO_left = 18
GPIO_right = 27

loop_counter = 0
servo_speed = 0.15
interval = 0.2

# Servo Table Constants
# Container properties
empty_plastic_container = 9.25 # empty mass of plastic container in g
empty_metal_container = 15.0 # empty mass of metal container in g
empty_paper_container = 10.0 # empty mass of paper container in g

# QBot Constants
wheel_to_wheel_distance = 0.235
camera_bumper_depth = 0.156
row = 385
col = 319




class qbot:
    def __init__(self, speed):
        self.bot = QBot2e()  # Initialize qbot hardware
        self.bot.reset()

        self.max_speed = 100
        self.speed = speed
        self.turn = 0

        # Camera image number
        #self.camera_image = 0.0

        # initialize variable for line following capabilities
        self.lost_line = 0

      

    def forward_time(self, time):
        distance = self.speed * time
        self.bot.move_time(distance, 0, time)

    def forward_distance(self, distance):
        time = distance / self.speed
        self.bot.move_time(distance, 0, time)

        # WORK IN PROGRESS: Waiting for Quanser depth implementation

    def travel_forward(self, threshold):
        d = self.bot.depth()

        while threshold < d:
            self.bot.set_velocity([self.speed, self.speed])
            print("Depth (m): ", d)
            d = self.bot.depth()
            # time.sleep(0.05)

    def rotate(self, degree):
        time = 1.0
        rad = math.radians(
            -degree)  # To ensure clockwise rotation for a positive degree input similar to the simulation.
        self.bot.move_time(0, rad, time)

    # WIP: NEED QUANSER INPUT
    # Read and return how far the Q-Bot is from an object e.g. the walls.
    def depth(self):  # changed from depth since it does not work!!
        ##
        ##        depth_frame = self.camera_image.get_depth_frame()
        ##        print(depth_frame)
        ##        camera_depth = depth_frame[row][col]
        ##        print(camera_depth)
        # Place requirement to initialize camera before you run this code.
        # camera_depth = depth_frame[row][col]
        # print(camera_depth)

        # camera_depth = depth_frame[row][col][1]
        # d_meters = 9.44*camera_depth/255

        # d = round(d_meters - camera_bumper_depth,3)
        # return d
        return depth_frame  # this is the entire image

    def stop(self):
        self.bot.halt()

    # Takes an input file from the modelling sub-team that contains time and angle data.
    # Recommended file type is a .txt file without headers i.e. string characters identifying
    # the time and angle columns. It is assumed that the first column is time (s) and the
    # second column is the angle (deg)
    def process_file(self, filename):
        rotation_time = []
        rotation = []
        with open(filename, "r") as f:
            for line in f:
                line = line.strip()
                line_pair = line.split("\t")  # assuming translation and rotation coordinates are spearated by \t

                rotation_time.append(float(line_pair[0]))
                rotation.append(float(line_pair[1]))
        return rotation_time, rotation

    # -----------------------------------------------------------------------------------------------
    # Used for internal calculations to reset the box's position and rotation.
    # Do not include in the library documentation
    def reset_box(self):
        self.x, self.y, self.z, self.x_r, self.y_r, self.z_r = [0] * 6
        self.bot._set_box_attitude()

    # ------------------------------------------------------------------------------------------------------------------------
    # DON'T INCLUDE IN THE LIBRARY DESCRIPTION.

    # Used to calculate the length, which is used to evaluate the distance
    def dotproduct(self, v1, v2):
        return sum((a * b) for a, b in zip(v1, v2))

    # Same as above
    def length(self, v):
        return math.sqrt(self.dotproduct(v, v))
 
    def line_following_sensors(self):
        print(GPIO.input(GPIO_left))
    # Returns IR sensor readings at the bottom of the Q-Bot. 1 = yellow line reading, 0 = no yellow line readings
    # Use redesigned to create a follow line function. Suggest to position the camera at 21.5 degrees.
##    def line_following_sensors(self):
##        # Get RGB color
##        self.img_RGB = self.camera_image.get_RGB_frame()
##
##        # get image binary. This is a cropped version of the image
##        self.img_binary = crop_rect(self.img_RGB, [280, 313], [448, 480])
##        # self.show_camera_image() # used to display the Kinect's image
##
##        left_color = self.img_binary[10, 0]  # BGR color
##        right_color = self.img_binary[10, 32]  # BGR color
##
##        if left_color[1] >= 150 and left_color[2] >= 150:  # for a yellow line
##            left_ir_sensor_reading = 1  # reflects all the light
##        else:
##            left_ir_sensor_reading = 0
##
##        if right_color[1] >= 150 and right_color[2] >= 150:  # for a yellow line
##            right_ir_sensor_reading = 1  # reflects all the light
##        else:
##            right_ir_sensor_reading = 0
##
##        ir_reading = [left_ir_sensor_reading, right_ir_sensor_reading]
##
##        return ir_reading

    # Sets the speed of the left and right wheel. input is a list containing the left wheel speed at index 0 and right wheel speed at index 1.
    def set_wheel_speed(self, speeds):
        if type(speeds) == list:
            self.bot.set_velocity([speeds[1], speeds[
                0]])  # This is done such that the referenced wheels are the same in the simulation and hardware
        else:
            print("Invalid input. Enter the left and right wheel speed as a list. e.g. [left_speed, right_speed].")

    def initialize_camera(self):
        self.camera_image = Kinect()

        self.img_RGB = cv2.imread('DefaultImage.jpg') #np.zeros((480, 640, 3), dtype=np.uint8)  # initial image
        self.img_binary = crop_rect(self.img_RGB, [280, 313], [448, 480])
        self.show_camera_image()

    # WIP: NEED Quanser input. Using this method and then calling the Kinect() class after yields an error
    # Note: This method is not necessary, but want to put it in here for completion.
    def stop_camera(self):
        self.camera_image.halt()

    def show_camera_image(self):
        cv2.startWindowThread()
        cv2.namedWindow('Camera Image 33 x 32', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Camera Image 33 x 32', self.img_binary)

    def follow_line(self,speed):
        # Initialize camera
        #self.camera_image = CameraUI()

        #image_buffer = self.bot.get_new_RGB()
        self.img_RGB = self.camera_image.get_RGB_frame()
        image_bin = hue_threshold(self.img_RGB, 60, 34, 360)

        #image_cropped = crop_rect(image_bin, [0,640], [0,480])
        #self.img_binary = crop_rect(self.img_RGB, [280,313],[448,480])
        #self.img_binary = crop_rect(image_bin, [280,313],[448,480])
        self.img_binary = crop_rect(image_bin, [0,640],[0,480])
        self.show_camera_image()

        
        line_ctr = extract_line_ctr(self.img_binary)
        
        max_speed = speed
        qbot_speed = 0
        turn = 0
        print(line_ctr)
        
        if line_ctr != -1:
            self.lost_line = 0

            # Normalize the position of the line in the range (-1, 1)
            #err = (320 - line_ctr) / 320 # Takes the curve path around the track
            err = (300 - line_ctr) / 300 # From testing in the real environment. Print the max line_ctr when on the line
            # Calculate the offset for turning
            turn = err * 0.5
            # Slow down as the line approaches the edge of the FOV
            qbot_speed = max_speed * (1 - abs(err))

        else:
            # Stop the robot if the line is not found for 5 frames
            self.lost_line += 1
            if self.lost_line > 5 and max_speed != 0 :
                self.stop()
                print("Cannot find line, QBot stopped...")

        delta = turn * 0.235  # QBOT_DIAMETER
        left = qbot_speed - delta
        right = qbot_speed + delta
        velocity = [left,right]
        #self.bot.set_velocity(velocity)
        return self.lost_line, velocity
# -----------------------------------------------------------------------------------------------
# Class used for internal use only. It was used to be used in conjuction with the sensors on the Q-Bot.
# Leave it out of the documentation

class bins:
    def __init__(self):

        self.metal_bin = smartbox_sim(QIL, 1)
        self.paper_bin = smartbox_sim(QIL, 2)
        self.plastic_bin = smartbox_sim(QIL, 3)
        self.garbage_bin = smartbox_sim(QIL, 4)

    # Returns the position of the specified bin.
    def bin_position(self,bin_id):
        if bin_id == "Bin01":
            [self.bin_position_x, self.bin_position_y, self.bin_position_z] = self.metal_bin.get_position()
        elif bin_id == "Bin02":
            [self.bin_position_x, self.bin_position_y, self.bin_position_z] = self.paper_bin.get_position()
        elif bin_id == "Bin03":
            [self.bin_position_x, self.bin_position_y, self.bin_position_z] = self.plastic_bin.get_position()
        elif bin_id == "Bin04":
            [self.bin_position_x, self.bin_position_y, self.bin_position_z] = self.garbage_bin.get_position()

        #print(self.bin_position_x, self.bin_position_y, self.bin_position_z)
        return self.bin_position_x, self.bin_position_y, self.bin_position_z

    # Returns the properties of the specified bin.
    def bin_properties(self,bin_id):
        if bin_id == "Bin01":
            self.r, self.g, self.b, self.metallic, self.roughness = self.metal_bin.get_surface_properties()
        elif bin_id == "Bin02":
            self.r, self.g, self.b, self.metallic, self.roughness = self.paper_bin.get_surface_properties()
        elif bin_id == "Bin03":
            self.r, self.g, self.b, self.metallic, self.roughness = self.plastic_bin.get_surface_properties()
        elif bin_id == "Bin04":
            self.r, self.g, self.b, self.metallic, self.roughness = self.garbage_bin.get_surface_properties()

        return self.r, self.g, self.b, self.metallic, self.roughness

# -----------------------------------------------------------------------------------------------
