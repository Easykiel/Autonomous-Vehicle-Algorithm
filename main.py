#
# Authors: Ezekiel Protacio, Devin Pon, Karli Ching
# Description: Detects white lines to control an Autonomous car
#

import sensor, image, time, math, pyb
from pyb import UART, Pin, Timer

#Constants to change
max_speed = 400 # Desired Max Speed
min_speed = 300 # Desired Max Speed
steering_gain = 10  # Steering Sensitivity
cruise_speed = 5 # Cruise Gain
kp = .8   # P term of the PID
ki = 0.01   # I term of the PID
kd = 10.0    # D term of the PID

# Initialize STATE to OFF
ON = 1
OFF = 0
BLUETOOTH = 10
STATE = OFF

steer_direction = 1   # Change to 1 if Steers in wrong direction
steering_center = 80  # Servo Center
set_angle = 90 # Going Straight
direction = False #false for left, true for right
offcourse = False #false for on course, true for offcourse

# initialize the terms to zero
old_error = 0
measured_angle = 0
p_term = 0
i_term = 0
d_term = 0
motor_counter = 0
counter = 0 #counts number of finish lines
old_time = pyb.millis()

# Tracks a White line
GRAYSCALE_THRESHOLD = [(230, 255)]

# Each roi is (x, y, w, h). The line detection algorithm will try to find the
# centroid of the largest blob in each roi. The x position of the centroids
# will then be averaged with different weights where the most weight is assigned
# to the roi near the bottom of the image and less to the next roi and so on.
ROIS = [ # [ROI, weight]
    (0, 100, 160, 20, 0.2), # You'll need to tweak the weights for your app
    (0,  40, 160, 40, 0.5), # depending on how your robot is setup.
    (0,   0, 160, 20, 0.3)
]

# Compute the weight divisor (we're computing this so you don't have to make weights add to 1).
weight_sum = 0
for r in ROIS: weight_sum += r[4] # r[4] is the roi weight.

# Camera setup...
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # use grayscale.
sensor.set_framesize(sensor.QQVGA) # use QQVGA for speed.
sensor.skip_frames(time = 2000) # Let new settings take affect.
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock() # Tracks FPS.
led1 = pyb.LED(1) # Red LED = 1
led2 = pyb.LED(2) # Green LED = 2
led3 = pyb.LED(3) # Blue LED = 3

#Setup GPIO Pins for Motor Input
inA = Pin('P0', Pin.OUT_PP)
enA = Pin('P1', Pin.OUT_PP)
inB = Pin('P3', Pin.OUT_PP)
enB = Pin('P2', Pin.OUT_PP)

#Set EN to LOW Initially
enA.low()
enB.low()
inA.low()
inB.low()

#Setup UART
uart = UART(3, 115200, timeout_char=1000)

#Send Ok command to Bluetooth
uart.write('AT\r\n')
print(uart.readline())

#Initialize PWM & Motor Duty Cycle
pwm = 1400
motor = 0
mil = 1000000

# Timer Setup for Servo
tim = Timer(2, freq=300) # Frequency in Hz
t1prescaler = tim.source_freq()/(tim.prescaler() + 1)
div = int(t1prescaler * (pwm/mil))
ch1 = tim.channel(1, Timer.PWM, pin=Pin("P6"), pulse_width=div)

# Timer Setup for Motor
tim2 = Timer(4, freq=1000) # Frequency in Hz
t2prescaler = tim2.source_freq()/(tim2.prescaler() + 1)
div2 = int(t2prescaler * (motor/mil))
ch2 = tim2.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width=div2)


Forward = False
Reverse = False
BrakeVCC = False
BrakeGND = False

#Sets board to go Forward
def forward():
    global enA, enB, inA, inB, Forward, Reverse, BrakeVCC, BrakeGND
    Forward = True
    Reverse = False
    BrakeVCC = False
    BrakeGND = False
    enA.high()
    enB.high()
    inA.high()
    inB.low()

#Sets board to go Reverse
def reverse():
    global enA, enB, inA, inB, Forward, Reverse, BrakeVCC, brakeGND
    Forward = False
    Reverse = True
    BrakeVCC = False
    BrakeGND = False
    enA.high()
    enB.high()
    inA.low()
    inB.high()

#Sets board to Break to VCC
def brakeVCC():
    global enA, enB, inA, inB, Forward, Reverse, BrakeVCC, BrakeGND
    Forward = False
    Reverse = False
    BrakeVCC = True
    BrakeGND = False
    enA.high()
    enB.high()
    inA.high()
    inB.high()

#Sets board to Break to GND
def brakeGND():
    global enA, enB, inA, inB, Forward, Reverse, BrakeVCC, BrakeGND
    Forward = False
    Reverse = False
    BrakeVCC = False
    BrakeGND = True
    enA.high()
    enB.high()
    inA.low()
    inB.low()

#Sets board to Break to GND
def turnOFF():
    global enA, enB, inA, inB
    enA.low()
    enB.low()
    inA.low()
    inB.low()

#For Constraining Values to a Certain Range
def constrain(value, min, max):
    if value < min :
        return min
    if value > max :
        return max
    else:
        return value

#Updates PID Terms Based on Error
def update_pid():
    #Get Global Variables
    global old_time, old_error, measured_angle, set_angle
    global p_term, i_term, d_term

    now = pyb.millis() #Gets current time
    dt = now - old_time #Calculates dt
    error = set_angle - measured_angle #Calculates Error
    de = error - old_error #Calculates de

    p_term = kp * error #Calculates P Term
    i_term += ki * error #Calculates I Term
    i_term = constrain(i_term, 0, 100) #Constrains form 0 - 100
    d_term = (de / dt) * kd #Calculates D Term
    #print("d_term = %d i_ term = %d\n" % (d_term, i_term))

    old_error = error #Makes Current Error to Old
    output = steer_direction * (p_term + i_term + d_term) #Updates Correction Output
    output = constrain(output, -50, 50) #Constrains it to -50 - 50 degrees
    return output

#Original Finding Blobs Example
def findingBlobs():
    global old_time, pwm, motor, old_error, measured_angle, set_angle, STATE, d_term, counter, motor_counter, direction, offcourse
    global Forward, Reverse, BrakeVCC, BrakeGND

    img = sensor.snapshot() # Take a picture and return the image.
    centroid_sum = 0 #Calculates Centroid Sum

    for r in ROIS:
        blobs = img.find_blobs(GRAYSCALE_THRESHOLD, roi=r[0:4], merge=True) # r[0:4] is roi tuple.

        if blobs:
            offcourse = False
            # Find the blob with the most pixels.
            center_blob = min(blobs, key=lambda b: abs(b.cx() - 80))

            #uart.write("d_term: %d\n" % (d_term))
            #if(center_blob.w() <= 35):
            # Draw a rect around the center blob.
            img.draw_rectangle(center_blob.rect(), color=(0,0,0))
            img.draw_cross(center_blob.cx(),
                           center_blob.cy(), color=(0,0,0))

            centroid_sum += center_blob.cx() * r[4] # r[4] is the roi weight.

            if(r == ROIS[2]): #Finds Left and Right Finish Line $$WIP$$
                if(center_blob.cx() <= 5):
                    brakeVCC()
                    motor = 300
                    pwm = 1700
                    #uart.write("left drift\n")
                elif(center_blob.cx() >= 155):
                    brakeVCC()
                    motor = 300
                    pwm = 1000
                    #uart.write("right drift\n")
                else:
                    forward()

            elif(r == ROIS[1]):
                left_blob = min(blobs, key=lambda b: abs(b.cx() - 40))
                right_blob = min(blobs, key=lambda b: abs(b.cx() - 120))

                # Draw a rect around the left blob.
                img.draw_rectangle(left_blob.rect(), color=(0,0,0))
                img.draw_cross(left_blob.cx(),
                               left_blob.cy(), color=(0,0,0))

                # Draw a rect around the right blob.
                img.draw_rectangle(right_blob.rect(), color=(0,0,0))
                img.draw_cross(right_blob.cx(),
                               right_blob.cy(), color=(0,0,0))

                left_rect = left_blob.rect()
                right_rect = right_blob.rect()

                if ((abs(right_rect[0] - left_rect[0]) >= 10) and abs(right_rect[1] - left_rect[1]) <= 10
                        and right_rect[3] <= 15 and left_rect[3] <= 15):
                    if(counter == 1):
                        uart.write("stopping\n")
                        pwm = 1400
                        reverse()
                        motor = 900
                        pyb.delay(500)
                        STATE = OFF
                    else:
                        uart.write("starting\n")
                        counter = 1
                        pyb.delay(200)

        else:
            offcourse = True

    center_pos = (centroid_sum / weight_sum) # Determine center of line.
    deflection_angle = 0
    deflection_angle = -math.atan((center_pos-80)/60)
    deflection_angle = math.degrees(deflection_angle) #Finds Deflection Angle

    now = pyb.millis() #Looks at current time

    if  now > old_time + 1.0 :  # time has passed since last measurement
        measured_angle = deflection_angle + 90
        steer_angle = update_pid()   # send error off to the PID to get a correction command
        old_time = now  # reset clock
        #print(str(measured_angle) + ', ' + str(set_angle) + ', ' + str(steer_angle))

    if(not offcourse):
        pwm = 1400 - (steer_angle * steering_gain) #Calculates Servo PWM
        motor = constrain(max_speed - (abs(steer_angle) * cruise_speed), min_speed, max_speed)

        if(pwm > 1400):
            direction = False
        else:
            direction = True
    else:
        if(direction): #correct to the right
            pwm = 900
        else:  #correct to the left
            pwm = 1600

        motor = 300

def bluetoothControl(char):
    global pwm, motor
    #Base Steering and Motor Speed on Bluetooth Command
    if(char == 49 or char == 97): #To the Left
        pwm = 1000
        motor = max_speed - 100
    elif(char == 50 or char == 115):
        pwm = 1100
        motor = max_speed - 75
    elif(char == 51 or char == 100):
        pwm = 1200
        motor = max_speed - 50
    elif(char == 52 or char == 102):
        pwm = 1300
        motor = max_speed - 25
    elif(char == 53 or char == 103): # Center Position
        pwm = 1400
        motor = max_speed
    elif(char == 54 or char == 104):
        pwm = 1500
        motor = max_speed - 25
    elif(char == 55 or char == 106):
        pwm = 1600
        motor = max_speed - 50
    elif(char == 56 or char == 107):
        pwm = 1700
        motor = max_speed - 75
    elif(char == 57 or char == 108): #To the Right
        pwm = 1800
        motor = max_speed - 100


while(True):
    #Prints to UART and Terminal
    clock.tick() # Track elapsed milliseconds between snapshots().

    if(uart.any()): #Checks for UART
        char = uart.readchar() #Grab Bluetooth Command

        if(char == 113): #Press Q to turn ON Auto Mode
            STATE = ON
            forward()

        if(char == 101): #Press E to turn OFF Car
            STATE = OFF

        if(char == 98): #Press B to turn to Bluetooth Mode
            STATE = BLUETOOTH


    #State Machine
    if(STATE == OFF): #Turn Car OFF
        led1.on() #Turn Red ON
        led2.off()
        led3.off()
        brakeGND()
        counter = 0
        pwm = 1400
        motor = 1500

    elif(STATE == ON): #Turns on Auto Mode
        led1.off()
        led2.on() #Turn Green ON
        led3.off()
        findingBlobs() # Find Blobs
        #forward()

    elif(STATE == BLUETOOTH):
        led1.off()
        led2.off() #Turn Green ON
        led3.on() #Turn Blue ON
        bluetoothControl(char)

    # Calculate PWM for Servo
    div = int(t1prescaler * (pwm/mil))
    ch1.pulse_width(div)

    # Calculate PWM for Motor
    div2 = int(t2prescaler * (motor/mil))
    ch2.pulse_width(div2)

    #uart.write("PWM = %d\n" % pwm)
    #print("PWM = %.2f" % pwm)
    #uart.write("Motor = %d\n\n" % motor)
    #print("Motor = %d" % motor)
    #print("STATE = %.2f\n\n" % STATE)
    #print("%.2f fps\n" % clock.fps()) # Displays FPS on Terminal
    #uart.write("%d fps\n" % clock.fps()) # Displays on UART
