from threading import Thread
from random import random
import Queue
import time


# define function to get stuff out of queue
def retrieve_nowait(q):
    try:
        item = q.get_nowait()
    except:
        item = None
    return item  

# create queue items
items = ['A','B','C','D','E','F','G','H','I','J']

# initialize 1 queue for each servo
q0 = Queue.Queue()
q1 = Queue.Queue()
q2 = Queue.Queue()
q3 = Queue.Queue()

# define function that will place all the items into the queues
def put_in_q():
    for letter in items:
        q0.put_nowait(letter)
        q1.put_nowait(letter)
        q2.put_nowait(letter)
        q3.put_nowait(letter)

###############################################

# synchro class used to sync servo threads
class synchro():
    
    def __init__(self):
        self.IR = 0
        
        self.letter_0 = None
        self.letter_1 = None
        self.letter_2 = None
        self.letter_3 = None

# randomly trips the IR sensor to act as if coin broke sensor beam
def IR_sensor():
    time.sleep(random()/20)
    count = 0
    while True:
        rand = random()
        if rand >= 0.8 and count <= 9:
            count = count +1
            synchro.IR += 1
            #print rand
        elif count > 9:
            break
        else:
            time.sleep(random()/20)


# all the servo thread functions
# Each servo will print its number along with the letter from the queue that it retrieved, ie 0 A 0.
# Because the threads are synchronized all 4 servos should print their numbers and letters around the same time and 
# all 4 servos should print the same letter in succession before the next letter is printed.
def servo_0():
    time.sleep(random()/20) # random wait time to simulate real world start conditions
    while IR.is_alive():
        if synchro.IR > 0:
            synchro.letter_0 = retrieve_nowait(q0)
            while True:
                if synchro.letter_0 == synchro.letter_1 and synchro.letter_2 == synchro.letter_3 and synchro.letter_0 == synchro.letter_2:
                    synchro.IR -= 1
                    print 0, synchro.letter_0, 0
                    break
                
def servo_1():
    time.sleep(random()/20) # random wait time to simulate real world start conditions
    while IR.is_alive():
        if synchro.IR:
            synchro.letter_1 = retrieve_nowait(q1)
            while True:
                if synchro.letter_0 == synchro.letter_1 and synchro.letter_2 == synchro.letter_3 and synchro.letter_0 == synchro.letter_2:
                    #synchro.IR = False
                    print 1, synchro.letter_1, 1
                    break

def servo_2():
    time.sleep(random()/20) # random wait time to simulate real world start conditions
    while IR.is_alive():
        if synchro.IR:
            synchro.letter_2 = retrieve_nowait(q2)
            while True:
                if synchro.letter_0 == synchro.letter_1 and synchro.letter_2 == synchro.letter_3 and synchro.letter_0 == synchro.letter_2:
                    #synchro.IR = False
                    print 2, synchro.letter_2, 2
                    break

def servo_3():
    time.sleep(random()/20) # random wait time to simulate real world start conditions
    while IR.is_alive():
        if synchro.IR:
            synchro.letter_3 = retrieve_nowait(q3)
            while True:
                if synchro.letter_0 == synchro.letter_1 and synchro.letter_2 == synchro.letter_3 and synchro.letter_0 == synchro.letter_2:
                    #synchro.IR = False
                    print 3, synchro.letter_3, 3
                    break

#######################################################

# fill all the queues
put_in_q()

# initiate synchro class
synchro = synchro()

# create threads
IR = Thread(target = IR_sensor)
servo0 = Thread(target = servo_0)
servo1 = Thread(target = servo_1)
servo2 = Thread(target = servo_2)
servo3 = Thread(target = servo_3)

# start threads
IR.start()
servo0.start()
servo1.start()
servo2.start()
servo3.start()

# join threads
IR.join()
servo0.join()
servo1.join()
servo2.join()
servo3.join()
