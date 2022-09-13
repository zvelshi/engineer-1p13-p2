### Quanser Interactive Labs Connection Code. Do Not Edit. ###

import time
import sys
sys.path.append('../')

from Common_Libraries.p2_sim_lib import *

import os
from Common_Libraries.repeating_timer_lib import repeating_timer

def update_sim ():
    try:
        arm.ping()
    except Exception as error_update_sim:
        print (error_update_sim)

arm = qarm()
update_thread = repeating_timer(2, update_sim)

### Implementation Below ###

import random # Import Random Function For Use Of Randomizing Container Order Later On In The Program.

'''
Name: main
Purpose: Initializes the simulation sequence for the 6 containers, generates a list of integer spawn values within a range of 1-6 each appearing only once, repeats the Q-Arm movement cycle 6 times and then promptly terminates the program.
Inputs: None.
Outputs: None.
Author(s): Zacharia Velshi (velshiz), Tarnveer Takhtar (takhtart), Noel Altali (altaln1)
'''
def main():
    muscle_threshold = 0.8                          # EMG Sensor Threshhold
    gripper_travel_close = 30                       # Default Gripper Closing Degree Movement Value
    gripper_travel_open = -30                       # Default Gripper Opening Degree Movement Value
    cycle_complete = False                          # Identifies Whether A Single Cycle Is Fully Complete (Identify, Pick-up, Open/Close Drawer, Place, Return)
    ctrl_gripper_state = True                       # Identifies Whether The Gripper Is Currently In The Open (True) Or Closed (False) Position - Default Value Is Open (True)
    intermediate_position = [0.406, 0.0, 0.484]     # Intermediate Position For The Q-arm To Move To Between The Pickup Location And Final Autoclave Location
    pickup_position = [0.483, 0.0, 0.030]           # Initial Pickup Position For The Q-arm To Move To When Fetching The Non-Sterilized Containers
    arm_home_position = (0.406, 0.0, 0.483)         # Cartesian Coordinates Of The arm.home() Position Of The Q-arm (In A Tuple So That It Matches The Format For What Is Returned By The arm.effector_postion() Function, Utilized In The Code Later On) 


    # Spawn Value List - Creates A Randomized List (Utilizing Random Library) To Shuffle The Spawn Order Of The Containers (Shuffled List Of Numbers 1-6 Which Corresponds To The Container Values)
    spawn_values = [1,2,3,4,5,6]
    random.shuffle(spawn_values)
    print("Container Spawn Order:", spawn_values)


    # For Loop That Runs A Total Of 6 Cycles For Each Of The 6 Containers The Q-Arm Will Be Depositing
    for i in range (len(spawn_values)): 
        arm.spawn_cage(spawn_values[i])                                                                                                            # Spawns Cage (From Shuffled List)
        dropoff_location = identify_autoclave_bin(spawn_values[i])                                                                                 # Generates Coordinates Of Destination Of Container
        move_end_effector(muscle_threshold, arm_home_position, pickup_position, dropoff_location, intermediate_position, cycle_complete)           # Q-arm Moves To Pickup Location
        ctrl_gripper_state = actuate_grippers(muscle_threshold, ctrl_gripper_state, gripper_travel_close, gripper_travel_open, cycle_complete)     # Q-arm Grabs Container
        modify_drawer(muscle_threshold, spawn_values[i], True, cycle_complete)                                                                     # Checks To See If Container Is Large, If So Opens The Drawer
        move_end_effector(muscle_threshold, arm_home_position, pickup_position, dropoff_location, intermediate_position, cycle_complete)           # Q-arm Moves To Dropoff Location
        ctrl_gripper_state = actuate_grippers(muscle_threshold, ctrl_gripper_state, gripper_travel_close, gripper_travel_open, cycle_complete)     # Q-arm Releases Container
        modify_drawer(muscle_threshold, spawn_values[i], False, cycle_complete)                                                                    # Checks To See If Container Is Large, If So Closes The Drawer
        arm.home()                                                                                                                                 # Q-arm Returns To Home Position
       

'''
Name: move_end_effector
Purpose: Moves the virtual Q-Arm end effector between home and drop-off location based on the input data from the emulated muscle sensors.
    If the left muscle sensor is less than the threshold value, and the right muscle sensor is greater than the threshold value, the arm will move first to home position to clear any obsticles then to the destination dropoff location.
Inputs: muscle_threshold (a float value that indicates our chosen muscle sensor threshold to active), arm_home_position (XYZ coordinates of ), pickup_position (XYZ coordinates that move the arm to the position of where the container will be picked up from), intermediate_position (Secondary position XYZ coordinates that occur after the intial pickup of the autoclave and prior to the final drop off position), dropoff_location (a list of float type values indicating the x, y and z autoclave drop-off coordinates), cycle_complete (Holds a boolean value that indicates if a looping cycle of a function has ended).
Outputs: None.
Author: Zacharia Velshi (velshiz)
'''
def move_end_effector(muscle_threshold, arm_home_position, pickup_position, dropoff_location, intermediate_position, cycle_complete):
    print("Move Effector Cycle Initialized")

    while not cycle_complete:
        left = float(arm.emg_left())                # Emulator Muscle Sensor - Left
        right = float(arm.emg_right())              # Emulator Muscle Sensor - Right

        if left < muscle_threshold and right > muscle_threshold:
            if arm.effector_position() == arm_home_position:
                arm.move_arm(pickup_position[0],pickup_position[1],pickup_position[2])                         # Move To Pick-Up Postion
                cycle_complete = True                                                                          # Ends While Loop In The Function
            else:
                arm.move_arm(intermediate_position[0],intermediate_position[1],intermediate_position[2])       # Move To Intermediate Postion
                time.sleep(1.5)
                arm.move_arm(dropoff_location[0], dropoff_location[1], dropoff_location[2])                    # Move To Drop-Off Postion
                cycle_complete = True                                                                          # Ends While Loop In The Function
                

'''
Name: identify_autoclave_bin
Purpose: Identifies the autoclave bin destination coordinates given a spawn cage value.
Inputs: spawn_value (integer - current container's spawn cage value).
Outputs: list (a list of float type coordinates for a corresponding autoclave drop-off location).
Author: Tarnveer Takhtar (takhtart)
'''
def identify_autoclave_bin(spawn_value):
    autoclave_locations = [
    [-0.592, 0.236, 0.353],     # Small Red     ID: 1
    [0.0, -0.638, 0.353],       # Small Green   ID: 2
    [0.0, 0.638, 0.353],        # Small Blue    ID: 3
    [-0.355,0.147,0.304],       # Large Red     ID: 4
    [0.0,-0.384,0.304],         # Large Green   ID: 5
    [0.0,0.384,0.304]]          # Large Blue    ID: 6
    
    return autoclave_locations[spawn_value-1] # Returns Location Of Container Drop-Off Point

'''
Name: modify_drawer
Purpose: Either opens or closes a given autoclave's drawer for a large container to be deposited depending on muscle sensor inputs.
    If the left and right muscle sensors are greater than the threshold value, and the spawn value is greater than 4 (indicitive of a large container) then the corresponding autoclave container will be modified (opened/closed).
Inputs: spawn_value (integer - current container's spawn cage value), modifier (boolean - to indicate whether to open (True) or close (False) a drawer), muscle_threshold (a float value that indicates our chosen muscle sensor threshold to active), cycle_complete (Holds a boolean value that indicates if a looping cycle of a function has ended).
Outputs: None.
Author(s): Zacharia Velshi (velshiz), Tarnveer Takhtar (takhtart), Noel Altali (altaln1)
'''
def modify_drawer(muscle_threshold, spawn_value, modifier, cycle_complete):
    print("Modify Drawer Cycle Initialized")
    
    #Large Container IDs
    red_large_id = 4
    green_large_id = 5
    blue_large_id = 6

    while not cycle_complete and spawn_value in (red_large_id,green_large_id,blue_large_id):
        left = float(arm.emg_left())                                                            # Emulator Muscle Sensor - Left
        right = float(arm.emg_right())                                                          # Emulator Muscle Sensor - Right
        if left > muscle_threshold and right > muscle_threshold:
            if spawn_value == red_large_id:
                arm.open_red_autoclave(modifier)                                                # Opens/Closes Bin Based On Inputed Boolean Modifier
                cycle_complete = True                                                           # Ends While Loop In The Function
            elif spawn_value == green_large_id:
                arm.open_green_autoclave(modifier)                                              # Opens/Closes Bin Based On Inputed Boolean Modifier
                cycle_complete = True                                                           # Ends While Loop In The Function
            elif spawn_value == blue_large_id:
                arm.open_blue_autoclave(modifier)                                               # Opens/Closes Bin Based On Inputed Boolean Modifier
                cycle_complete = True                                                           # Ends While Loop In The Function
'''     
Name: actuate_grippers
Purpose: Either opens or closes the Q-Arm's grippers by referencing the global state of the Q-Arm's grippers and depending on muscle sensor inputs.
    If the left muscle sensor is greater than the threshold value and the right muslce sensor is less than the threshold value, then the grippers will be either opened or closed depending on the global gripper state variable
Inputs:  muscle_threshold (a float value that indicates our chosen muscle sensor threshold to active), ctrl_gripper_state (a boolean value which indicates whether or not the grippers are currently open (True) or closed (False), gripper_travel_close (integer value relating to the degrees of movement the gripper makes when closing), gripper_travel_open (integer value relating to the degrees of movement the gripper makes when opening), cycle_complete (Holds a boolean value that indicates if a looping cycle of a function has ended).
Outputs: Boolean (which updates the ctrl_gripper_state variable indicating whether or not the grippers are currently open (True) or closed (False))
Author(s): Zacharia Velshi (velshiz), Tarnveer Takhtar (takhtart), Noel Altali (altaln1)
'''
def actuate_grippers(muscle_threshold, ctrl_gripper_state, gripper_travel_close, gripper_travel_open, cycle_complete):
    print("Gripper Cycle Initialized")

    while not cycle_complete:
        left = float(arm.emg_left())                                        # Emulator Muscle Sensor - Left
        right = float(arm.emg_right())                                      # Emulator Muscle Sensor - Right
        if left > muscle_threshold and right < muscle_threshold:
            if ctrl_gripper_state:
                print("Closing Grippers")
                arm.control_gripper(gripper_travel_close)                   # Closes Gripper To Set Amount Of Degrees
                time.sleep(1)
                cycle_complete = True                                       # Ends While Loop In The Function
                return False                                                # Updates ctrl_gripper_state Variable to False (To Indicate Gripper Is Currently In The Closed Position)
            else:
                print("Opening Grippers")
                arm.control_gripper(gripper_travel_open)                    # Opens Gripper To Set Amount Of Degrees
                time.sleep(1)
                cycle_complete = True                                       # Ends While Loop In The Function
                return True                                                 # Updates ctrl_gripper_state Variable To True (To Indicate Gripper Is Currently In The Open Position)

main()