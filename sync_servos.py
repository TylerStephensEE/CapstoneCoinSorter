from threading import Thread
import threading
from random import random
import Queue
import time
from Adafruit_PWM_Servo_Driver import PWM
import RPi.GPIO as GPIO


##### Functions #####
def retrieve_nowait(q):
    try:
        item = q.get_nowait()
    except:
        item = None
    return item  


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
    servo_angles = {'P': [-90,-90,-90,-90],
                    'N': [-60,-60,-60,-60],
                    'D': [-30,-30,-30,-30],
                    'Q': [0,0,0,0],
                    'L': [30,30,30,30],
                    'T': [60,60,60,60],
                    'U': [90,90,90,90],}
    
    return servo_angles[coin_letter]
    	# each of the values in the dictionary list will be the servo angle
    	# ex) for P 0 = servo_0 angle, 1 = servo_1 angle ...
    	# The servo angles must be converted from inverse kinematics definition
    	# to servo angle definition. ie) the zeros may be defined differently 
    	# None is added to handel retrieveing item from empty queue, this should not happen


##### Servo Class for moving servos #####
class servo():

  def __init__(self, servo_name, PWM_channel, tick_range, degree_range):
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
      
################################################################################################################################



##### Servo Class for Synchronizing Servos ###### 
'''
INPUT:
num_servos  # The number of servos that are to be syncrhonized with eachother
DESCRIPTION:
Constructor of servo_threads instance. Initializes important information to
initial value. coin_denom is the denomination of the coin that is being
sorted. servo_done is a counter that keeps track of the number of servos
that have completed thier loop. IR_count is a counter for the number
of coins that have brokent the IR sensor beam. run is a boolean value
that is True while the tasks are running and only set to False once the
tasks have completed, ie the pragram is being shutdown.
'''
class sync_servos():
    def __init__(self, num_servos):
        self.num_servos = num_servos
        self.coin_angles = None
        self.servo_done = 0
        self.run = True


    def run_servos(self):
        '''
        DESCRIPTION:
        Will only start performing actions once the IR sensor has been tripped.
        When this occurs a coin denomination will be retrieved from the coin_queue.
        The previous event of servo_complete will be set to False because the
        current loop servos cannot be completed if the loop hasn't started yet.
        The loop is then started with servo_start.set(), this sets the event value
        to True. Then the funciton will wait until all the servos have completed
        their loop, this is specified by the servo_complete event which is set to
        True inside the ServoComplete function.    
        '''
        while self.run:
            if IRsensor.IR_counter > 0:
                self.coin_angles = robot_arm_position(retrieve_nowait(coin_queue)) # retrieve coin denomination from queue and convert to angle vector
                servo_complete.clear() # clear previous set() of servo_complete
                servo_pause.clear() # clear previus set() to prevent servos from looping 
                servo_start.set() # set() so the servos can run
                servo_complete.wait() # wait for all the servos to be completed


    def stop_servos(self):
        '''
        DESCRIPTION:
        Will only start performing actions once all the servos have completed thier
        current loop. When this happens the servo_done value is reset back to 0, to
        signify that 0 servos have done thier loop. The IR_count value will be
        decremented by 1 because one coin has been sorted and then the
        servo_complete will be set to True allowing ServoStart to finish its loop.
        '''
        while self.run:
            if self.servo_done == self.num_servos:
                servo_start.clear() # prevent the servo event from running until next time 
                self.servo_done = 0 # clear the completed servos
                servo_pause.set() # set() to get servos to next loop
                IRsensor.IR_counter -= 1 ### NEEDS TO BE MADE GENERAL. Maybe add coin_sensor() object to input of function
                servo_complete.set() # set() so run_servos() may continue

                
    def servos(self, servo_num,servo_object):
        '''
        INPUT:
        thread_num  # The number of the servo in the range of [0,num_servos-1]
    
        DESCRIPTION:
        Will only start performing actions once the servo_start event is set to
        True by the ServoStart function. Once the function is started it will
        travel to servo_num's specified angle to pick up the coin, then travel
        to servo_num's specified angle to have the arm above the proper bin,
        travel to servo_num's specified angle to drop the coin, and then
        travel back the the initial position by having the servo go to servo_num's
        specified angle for the inital position.
        '''
        while self.run:
            if servo_start.isSet():
                # This code needs to be replaced with code to pick up the coin
                # move the arm to the bin, drop the coin, and return back to
                # the initial position
                print ' {0}_{1} '.format(servo_num,self.coin_angles[servo_num])
                servo_object.smooth_move_to(self.coin_angles[servo_num])
                self.servo_done += 1
                servo_pause.wait()

################################################################################

'''
Kyle add a description for the class coin_sensor()
'''
class coin_sensor():
    # A class to initialize the coin sensors and to create interupts when the beam is broken
    gpio_coin_sensor = 0
    coin_still_there = False
    IR_counter = 0

    def __init__(self, sensor_gpio):
        self.gpio_coin_sensor = sensor_gpio
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_coin_sensor, GPIO.IN, GPIO.PUD_DOWN)

        def look_for_coin(channel):
        '''
        Kyle add information about INPUT:
        Kyle add a description for this function below
        '''
            if GPIO.input(channel):
                if self.coin_still_there == False:
                    self.IR_counter += 1
                    self.coin_still_there = True
            else:
                self.coin_still_there = False

        GPIO.add_event_detect(self.gpio_coin_sensor, GPIO.BOTH, callback=look_for_coin)

#############################################################################
# Set PWM frequency and I2C address
pwm = PWM(0x40)
pwm.setPWMFreq(60)

# create queue items
items = ['P','N','D','Q','L','T','U','P','N','D','Q','L','T','U']

# create events 
servo_start = threading.Event() # is set to False initially
servo_complete = threading.Event() # is set to False initially
servo_pause = threading.Event() # is set to False initially

# Create the coin queue
coin_queue = Queue.Queue()

# fill coin_queue
for letter in items:
    coin_queue.put_nowait(letter)

# create object
IRsensor = coin_sensor(17)

# Create servos objects
servo_object_0 = servo('HS311', 3, [170,595], [-90,90])
servo_object_1 = servo('HS311', 7, [170,615], [-90,90])
servo_object_2 = servo('SG90', 11, [200,650], [-90,90])
servo_object_3 = servo('SG90', 15, [200,650], [-90,90])

# initiate sync class
ServoThreads = sync_servos(4)

# create the threads
start_thread = Thread(target = ServoThreads.run_servos)
complete_thread = Thread(target = ServoThreads.stop_servos)
servo_0 = Thread(target = ServoThreads.servos, args=(0,servo_object_0,))
servo_1 = Thread(target = ServoThreads.servos, args=(1,servo_object_1,))
servo_2 = Thread(target = ServoThreads.servos, args=(2,servo_object_2,))
servo_3 = Thread(target = ServoThreads.servos, args=(3,servo_object_3,))

# start threads
start_thread.start()
complete_thread.start()
servo_0.start()
servo_1.start()
servo_2.start()
servo_3.start()

# join threads
start_thread.join()
complete_thread.join()
servo_0.join()
servo_1.join()
servo_2.join()
servo_3.join()

GPIO.cleanup()
