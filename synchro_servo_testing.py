from threading import Thread
import threading
from random import random
import Queue
import time


def retrieve_nowait(q):
    try:
        item = q.get_nowait()
    except:
        item = None
    return item  

# create queue items
items = ['A','B','C','D','E','F','G','H','I','J']


################################################################################
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
class servo_threads():
    def __init__(self, num_servos):
        self.num_servos = num_servos
        self.coin_denom = None
        self.servo_done = 0
        self.IR_count = 0
        self.run = True


    def ServoStart(self):
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
            if self.IR_count > 0:
                self.coin_denom = retrieve_nowait(coin_queue) # retrieve coin denomination from queue
                servo_complete.clear() # clear previous set() of servo_complete
                servo_pause.clear() # clear previus set() to prevent servos from looping 
                servo_start.set() # set() so the servos can run
                servo_complete.wait() # wait for all the servos to be completed


    def ServoComplete(self):
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
                self.IR_count -= 1
                servo_complete.set() # set() so ServoStart() may continue

                
    def servo(self, servo_num):
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
                print '{0}{1} '.format(servo_num,self.coin_denom)
                time.sleep(0.1)
                #print '{0} dropping coin'.format(thread_num)
                time.sleep(0.1)
                #print '{0} moving back'.format(thread_num)
                time.sleep(0.1)
                #print '{0} arrived back'.format(thread_num)
                self.servo_done += 1
                servo_pause.wait()

    # this is temporary and is used only to simulate machine
    # may be replaced or completely deleted.
    def IR_sensor(self):
        time.sleep(random()/20)
        count = 0
        while True:
            rand = random()
            if rand >= 0.95 and count <= 9:
                count += 1
                self.IR_count += 1
            elif count > 9 and self.IR_count == 0:
                self.run = False
                break
            else:
                time.sleep(random()/20)
################################################################################

# create events 
servo_start = threading.Event() # is set to False
servo_complete = threading.Event() # is set to False
servo_pause = threading.Event() # is set to False

# Create the coin queue
coin_queue = Queue.Queue()

# fill coin_queue
for letter in items:
    coin_queue.put_nowait(letter)

# initiate class
ServoThreads = servo_threads(4)


# create the threads
IR_thread = Thread(target = ServoThreads.IR_sensor)
start_thread = Thread(target = ServoThreads.ServoStart)
complete_thread = Thread(target = ServoThreads.ServoComplete)
servo_0 = Thread(target = ServoThreads.servo, args=(0,))
servo_1 = Thread(target = ServoThreads.servo, args=(1,))
servo_2 = Thread(target = ServoThreads.servo, args=(2,))
servo_3 = Thread(target = ServoThreads.servo, args=(3,))

# start threads
IR_thread.start()
start_thread.start()
complete_thread.start()
servo_0.start()
servo_1.start()
servo_2.start()
servo_3.start()

# join threads
IR_thread.join()
start_thread.join()
complete_thread.join()
servo_0.join()
servo_1.join()
servo_2.join()
servo_3.join()
