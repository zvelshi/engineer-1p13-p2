# This code represents the library that students will reference to complete activities in for Project 0
#
# All code is currently set up to run on a raspberry pi ONLY.

import math
import sys
import time
sys.path.append('../')

from Common_Libraries.QBot2e_Lib import *
from Common_Libraries.quanser_image_lib import * # Has CV and numpy imported already

#from quanser.multimedia import (Video3D, Video3DStreamType, ImageFormat, ImageDataType)

wheel_to_wheel_distance = 0.235
camera_bumper_depth = 0.156
row = 385
col = 319

class qbot:
 
    def __init__(self, speed):
        self.bot = QBot2e() # Initialize qbot hardware
        self.bot.reset()
        
        self.max_speed = 100
        self.speed = speed
        self.turn = 0
        
    def forward_time(self, time):
        distance = self.speed * time
        self.bot.move_time(distance,0,time)

    def forward_distance(self,distance):
            time = distance/self.speed
            self.bot.move_time(distance,0,time)

# WORK IN PROGRESS: Waiting for Quanser depth implementation
    def travel_forward(self,threshold):
        d = self.bot.depth()

        while threshold < d:
            self.bot.set_velocity([self.speed,self.speed])
            print("Depth (m): ",d)
            d = self.bot.depth()
            #time.sleep(0.05)

        # Stop QBot
        self.bot.halt()

    def rotate(self, degree):
        time = 1.0
        rad = math.radians(-degree) # To ensure clockwise rotation for a positive degree input similar to the simulation.
        self.bot.move_time(0,rad,time)

    # WIP: NEED QUANSER INPUT        
    # Read and return how far the Q-Bot is from an object e.g. the walls.
    def depth(self): # changed from depth since it does not work!!
##
##        depth_frame = self.camera_image.get_depth_frame()
##        print(depth_frame)
##        camera_depth = depth_frame[row][col]
##        print(camera_depth)
        #Place requirement to initialize camera before you run this code.
        #camera_depth = depth_frame[row][col]
        #print(camera_depth)

        #camera_depth = depth_frame[row][col][1]
        #d_meters = 9.44*camera_depth/255

        #d = round(d_meters - camera_bumper_depth,3)
        #return d
        return depth_frame # this is the entire image

    def stop(self):
        self.bot.halt()

    # Returns IR sensor readings at the bottom of the Q-Bot. 1 = yellow line reading, 0 = no yellow line readings
    # Use readigns to create a follow line function. Suggest to position the camera at 21.5 degrees.
    def line_following_sensors(self):
        # Get RGB color
        self.img_RGB = self.camera_image.get_RGB_frame()

        #get image binary. This is a cropped version of the image
        self.img_binary = crop_rect(self.img_RGB,[280,313],[448,480])
        #self.show_camera_image() # used to display the Kinect's image

        left_color = self.img_binary[10,0] #BGR color
        right_color = self.img_binary[10,32] #BGR color

        if left_color[1] >= 150 and left_color[2] >= 150: # for a yellow line
            left_ir_sensor_reading = 1 # reflects all the light
        else:
            left_ir_sensor_reading = 0

        if right_color[1] >= 150 and right_color[2] >= 150: # for a yellow line
            right_ir_sensor_reading = 1 # reflects all the light
        else:
            right_ir_sensor_reading = 0

        ir_reading = [left_ir_sensor_reading,right_ir_sensor_reading]
        
        return ir_reading
            
    # Sets the speed of the left and right wheel. input is a list containing the left wheel speed at index 0 and right wheel speed at index 1.
    def set_wheel_speed(self,speeds):
        if type(speeds) == list:
            self.bot.set_velocity([speeds[1],speeds[0]]) # This is done such that the referenced wheels are the same in the simulation and hardware
        else:
            print("Invalid input. Enter the left and right wheel speed as a list. e.g. [left_speed, right_speed].")

    def initialize_camera(self):
        self.camera_image = Kinect()

        self.img_RGB = np.zeros((480,640,3), dtype = np.uint8) #initial image
        self.img_binary = crop_rect(self.img_RGB,[280,313],[448,480])
        #self.show_camera_image()

    #WIP: NEED Quanser input. Using this method and then calling the Kinect() class after yields an error
    # Note: This method is not necessary, but want to put it in here for completion.
    def stop_camera(self):
        self.camera_image.halt()

    def show_camera_image(self):
         cv2.startWindowThread()
         cv2.namedWindow('Camera Image 33 x 32', cv2.WINDOW_AUTOSIZE)
         cv2.imshow('Camera Image 33 x 32', self.img_binary)

# WIP: Quanser input required here. Should be provided in the QBot2e_Lib.py file.
class depth_sensor:
 # Define class-level variables
    _image_width = 640
    _image_height = 480
    _image_rate = 10
    _depth_buffer = None
    _stream_index = 0
    _depth_capture = None
    _depth_stream = None
    _status = 0

    _measure = None
# Initialize Kinect
    def __init__(self, ID = "0", rate = 10):
        print ("Initializing Kinect")
        if rate != 10:
            self._image_rate = rate
        self._depth_buffer = self.placeholder_image()
        self._measure = 0 # Martha Added
        rcv_frame = False
        fail_count = 0
        while rcv_frame == False:
            if self._depth_stream != None:
                self.halt()
            try:
                self._depth_capture = Video3D(ID)
                print("Created Video3D instance")
                self._depth_stream = self._depth_capture.stream_open(Video3DStreamType.DEPTH, self._stream_index,
                    self._image_rate, self._image_width, self._image_height,
                    ImageFormat.ROW_MAJOR_GREYSCALE, ImageDataType.UINT8)
                print("Opened RGB stream")
                self._depth_capture.start_streaming()
            except GenericError as ex:
                print("ERROR: " + ex.get_error_message())

            print("Waiting for Kinect...")
            t_start = time.monotonic()
            while (time.monotonic() - t_start) < 5:
                frame = self._depth_stream.get_frame()
                if frame != None:
                    rcv_frame = True
                    self._status = 1
                    break
            fail_count += 1
            if fail_count > 2:
                print("too many tries, aborting")
                break
            
        print ("Kinect Initialized")

    #Get an RGB frame from the Kinect
    def get_depth_frame(self):
        frame = self._depth_stream.get_frame()
        if frame != None:
            frame.get_data(self._depth_buffer)
            frame.release()
       
        return self._depth_buffer

    def get_depth_meters(self):
        
        frame = self._depth_stream.get_frame()
        if frame != None:
            frame.get_meters(self._depth_buffer)
            frame.release()
        return self._measure
        
    #Return the status of the Kinect (are frames being received)
    def get_status(self):
        return self._status

    #Return a placeholder image
    def placeholder_image(self):
        return cv2.imread('robot.jpg')
