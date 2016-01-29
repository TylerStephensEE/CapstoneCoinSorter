import threading
import queue
import cv2
import os
import numpy as np

#testing security on github#
################################################################################
################################################################################

########## FUNCTION DEFINITIONS ##########

##### Retrieve Item from queue #####
'''
INPUT:
q  # This will be the queue with which you want to retrieve an item from

OUTPUT:
item  # The item that was retreived from the queue

DESCRIPTION:
This preforms the same function as get_nowait() but will incorparate the 
ability to handle exceptions when a get_nowait() is called but the queue is 
empty. If the queue is empty the ouput item is 'None'
'''
def retrieve_nowait(q):
    try:
        item = q.get_nowait()
    except:
        item = None
    return item

################################################################################
################################################################################

########## CLASS DEFINITIONS ##########

##### Servo Class for moving servos #####
'''
INPUT:
servo_name  # Name of the servo, can be any string you want to differentiate this servo from another

PWM_channel  # Channel that the servo is connected to on Adafruit PCA9685 Servo Board can take on values from 0-15

tick_range  # list of 2 values that correspond to the degree_range values

degree_range  # list of 2 values that specify the min and max angles this servo is allowed to travel through

DESCRIPTION:
Constructor of Servo instance. After servo has been created it will move the servo to the midway point between
the degree_range values and pause for 0.5 seconds, This is important because it sets up the current_pos angle and
tick value for the servo to use if smooth_move_to is called
'''
class servo():

    def __init__(self, servo_name, PWM_channel, tick_range, degree_range):
        self.name = servo_name
        self.channel = PWM_channel
        self.tick_range = tick_range
        self.degree_range = degree_range
    
        self.move_to((self.degree_range[0]+self.degree_range[1])/2)  # Set initial Position to mid point of degree range
        time.sleep(0.5)  # pause for 0.5 seconds to allow for servo to reach mid point


    def move_to(self,angle_value):
    '''
    INPUT:
    angle_value  # value for the angle you want the servo to travel to as long as
                   it is within the degree_range

    DESCRIPTION:
    Given a angle_value this will calculate the required tick value for the PWM driver
    and move the servo to that value. If the angle_value given as input is not within
    the degree_range then the servo will stay at its current position
    '''

        # Determining linear function for converting angle_value to tick_value for PWM
        tick_diff = self.tick_range[1]-self.tick_range[0]                    # rise 
        degree_diff = float(abs(self.degree_range[0]-self.degree_range[1]))  # run
        zero_tick = (self.tick_range[1]+self.tick_range[0])/2                # intercept
        tick_value = int(((tick_diff/degree_diff)*angle_value)+zero_tick)    # y = mx+b

        # Is the desired angle_value within the servos range
        if angle_value <= self.degree_range[1] and angle_value >= self.degree_range[0]:
            # Move Servo to specified angle value
            pwm.setPWM(self.channel,0,tick_value)

            # Current position value in tick and angle
            self.current_pos_tick = tick_value
            self.current_pos_angle = angle_value

        else:
            print('angle_value not within range: Staying at current position')


    def smooth_move_to(self,angle_value):
    '''
    INPUT:
    angle_value # value for the angle you want the servo to travel to as long as
                  it is within the degree_range. Can also be thought of as the
                  final angle value.

    DESCRIPTION:
    Given and angle_value this will move the servo from its initial position to the desired
    final value given by angle_value. However it will move the servo in such a manner to
    control its speed and acceleration so that both are smooth functions preventing quick
    jerky motion of the servo.

    The position vs. time function that describes the smooth motion of the servo is x(t) = 1/(1+exp(-t)),
    this is known as the sigmoid function. By taking derivatives, W.R.T. time, the velocity and acceleration
    curves can be found too. 
    '''

        # 20 point vector for mapping angle_value for controlling the acceleration (arbitrarily chosen length of list)
        sigmoid_mapping_vector = [0.0011, 0.0023, 0.0046, 0.0094, 0.0191, 0.0384, 0.0755, 0.1431, 0.2547, 0.4115,
                                  0.5885, 0.7453, 0.8569, 0.9245, 0.9616, 0.9809, 0.9906, 0.9954, 0.9977, 0.9989]

        # Time delay vector to smooth motion of servo. Last value is 0 because the servo has reached its final position therefore
        # no need to delay  
        time_delay = [0.0200, 0.0250, 0.0300, 0.0350, 0.0400, 0.0500, 0.0600, 0.0700, 0.0800, 0.0900,
                      0.1000, 0.0900, 0.0800, 0.0700, 0.0600, 0.0500, 0.0400, 0.0350, 0.0300, 0.0000]

        # Is the desired angle_value within the servos range
        if angle_value <= self.degree_range[1] and angle_value >= self.degree_range[0]:
            # Obtain angle delta
            angle_diff = angle_value-self.current_pos_angle
      
            # Map change in angle position onto sigmoid function
            angle_vector = map(lambda x: x*angle_diff+self.current_pos_angle, sigmoid_mapping_vector)
      
            # Move servo to angle_value smoothly
            count = 0
            for i in angle_vector:
                self.move_to(i)
                time.sleep(time_delay[count])
                count += 1

        else:
            print('angle_value not within range: Staying at current position')

################################################################################
################################################################################

##### TASK DESCRIPTIONS #####
'''
Define functions to execute in threads. These functions are individual tasks 
that operate the stages of the Coin Sorter. There are a total of 5 tasks 
needed to run the Coin Sorter.

1) Conveyor Task
2) Photo Analysis Task
3) LED Matrix Task
4) Track Task
5) Robot Arm Task
'''

########## TASK FUNCTION DEFINITIONS ##########


##### Conveyor Task #####
'''
INPUT:
shutdown_bit  # A global variable that will only change value when a push 
                button is pressed to shutdown the machine.
			   
input_coin  # A sensor to know when a coin falls onto conveyor.

QUEUES: (The queues this task will interact with)
photo_queue  # is to hold the photos of the coins. conveyor_task will only be
               placing values at the head of the queue.


DESCRIPTION:
Runs infinitely until shutdown_bit is True. During normal operation, when 
shutdown_bit is False, the function will test shutdown_bit find it False,
because this is normal operation, test if a new coin has been placed on the 
conveyor, if False step conveyor forward and repeat from begining. If True 
step conveyor forward, take photo of coin and place photo in photo_queue, then 
repeat from begining. When the shutdown_bit is True a timer is started and 
normal operation continues until the timer is exceded. The timer is restarted 
everytime a new coin is added on the conveyor, therefore only after "T" 
seconds of continuous time without new coins being added to the conveyor will 
cause shutdown of the task.

See flowchart "conveyor_task" for more information
'''
def conveyor_task(shutdown_bit, input_coin):
    while True:
	    if shutdown_bit == True:
	        # run code to shutdown conveyor_task
        else:
	        # run normal operation code
    return

###############################################################################


##### Photo Analysis Task #####
'''
QUEUES: (The queues this task will interact with)
photo_queue  # A queue that is to hold the photo of the coins. 
               photo_analysis_queue will remove the photos from the tail of the
			   queue.

coin_queue  # A queue that is to hold the letter of the coins. 
              photo_analysis_task will place the letters at the head of the 
			  coin_queue.


DESCRIPTION:
Determines if a photo was taken, if not then re-check continually. When it
determines a photo was taken, retreive the photo from the photo_queue analyze
the photo and determine its denomination, then place the corresponding letter
at the head of the coin_queue, and repeat from beginning.

See flowchart "photo_analysis_task" for  more information
'''
def photo_analysis_task():
    while robot_task.isAlive() # run until robot task is joined
    
    return
	
###############################################################################			


##### LED Matrix Task #####
'''
QUEUES: (The queues this task will interact with)
coin_queue  # A queue that is to hold the letter of the coins. led_matrix_task
              will only read items from the head of the queue. Will have to
			  make another queue that is a copy of the coin_queue so this task
			  can read only, since reading in queue is impossible as far as I 
			  know. My current idea for the copy of coin_queue is to only hold
			  1 item in it at most.
			  

OUTPUT:
None, only reads queue
May need to have LED pin outputs so the LED matrix stays on when task is not 
running


DESCRIPTION:
Checks if coin_queue is empty if it is re-check continually. If the queue is 
not empty read  the value at the head of the queue, then display the 
corresponding letter for the coin on the LED matrix then repeat from beginning.

see flowchart "led_matrix_task" for more information
'''
def led_matrix_task():
    while robot_task.isAlive()  # run until robot task is joined
        if coin_queue has item IN it # this line is not code
		    # from input from head of queue light up correct LED's on the Matrix
    return
	
###############################################################################			


##### Track Task #####
'''
INPUT:
ir_track_sensor  # A sensor that will be placed at the beginning of the track
                   to identify if a coin has passed.

OUTPUT:
may need to output PWM signal value so servo stays in constant position
This may not be necessary if no PWM signal doesn't change servo position

DESCRIPTION:
Checks if the coin passed the junction if it did not re-check continually. 
Depending on the pattern chosen the servo will only switch sides if the correct
number of coins has passed the sensor.

see flowchart "track_task" for more information
'''
def track_task(ir_track_sensor):
    while robot_task.isAlive()  # run until robot task is joined
	    if coin passed sensor:  # this line is not code
		    # flip servo to previous position
    return
	
###############################################################################
	

##### Robot Arm Task #####
'''
QUEUES: (The queues this task will interact with)
coin_queue  # A queue that is to hold the letter of the coins. robot_task will
              take items from the tail of the queue.


OUTPUT:
may need to output PWM signal value for all the servos so they stay in a 
constant position. This may not be necessary if no PWM singla doesn't change
servo position and servos apply torque with no PWM signal

DESCRIPTION:
Runs infinitely until conveyor_task has stopped. 
During normal operation, when conveyor_task is still running, the function will
test  conveyor_task find it running, because this is normal operation, test if 
a coin has been arrived to be sorted,if False repeat from begining. If True 
take coin value from tail of coin_queue pick up that coin and place it in the 
corresponding bin then repeat from begining. When the conveyor_task has stopped
a timer is started and normal operation continues until the timer is exceded. 
The timer is restarted everytime a new coin has arrived for sorting, therefore 
only after "T" seconds of continuous time without new coins arriving, to be 
sorted, will cause a shutdown of the task.

see flowchart "robot_task" for more information
'''
def robot_task()

########## FUNCTION DEFINITIONS ##########

	##### Robot Arm Servo Angles #####
	'''
	INPUT:
	coin_letter  # This will be the letter of the coin that is ready to be sorted.

	OUTPUT:
	may need to output PWM signal value for all the servos so they stay in a 
	constant position. This may not be necessary if no PWM singla doesn't change
	servo position and servos apply torque with no PWM signal

	DESCRIPTION:
	With the inputted coin_letter this will return the servo angle values for the
	arm to be in position above the coin_letter bin. coin_letter is the key to the
	dictionary which will contain a 1x4 row vector where each column 'i' represents
	the corresponding servos angle. 
	'''
	def robot_arm_position(coin_letter):
	    servo_angles = {'P': [0,1,2,3],
	                    'N': [4,5,6,7],
	                    'D': [8,9,10,11],
	                    'Q': [12,13,14,15],
	                    'L': [16,17,18,19],
	                    'T': [20,21,22,23],
	                    'U': [24,25,26,27]}
    
		return servo_angles[coin_letter]
    	# each of the values in the dictionary list will be the servo angle
    	# ex) for P 0 = servo_0 angle, 1 = servo_1 angle ...
    	# The servo angles must be converted from inverse kinematics definition
    	# to servo angle definition. ie) the zeros may be defined differently 

################################################################################

##### Main Task #####
	while True:
	    if conveyor_task.isAlive() 
            # run code to pick up coin
            # robot_arm_position(coin_letter) function
            # run code to drop coin
            # run code to return to initial position			
        else:
	        # run code to shutdown robot_task
    return
	
###############################################################################
###############################################################################

