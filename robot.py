import picamera
import time
import sys
from threading import Thread, Event
import serial
import serial.tools.list_ports
import os
# import numpy


# CAMERA
# SERIAL
VERSION = "v0.0.0"
IMAGE_INTERVAL = 1
SERIAL_INTERVAL = 0.001
ACTION_INTERVAL = 0.5
IMAGE_PATH = "/home/pi/robot/imgs"
LOG_LEVEL = 1;                    #0=debug, 1=info, 3=warn, 4=error
ULTRASONICSENSOR_DATA = {}        # Distance values. One list per bucket
ULTRASONICSENSOR_TIMESTAMPS = {}  # Timestamps. One list per bucket
MAX_HISTORY = 100                 # Max amt of measurements. Albeit imgs, ultrasonics
HISTORY_PURGE_TO = 50             # When max reached, purge to this amount


# Thread handling interval and image capturing
class ImageThread(Thread):
  def __init__(self, event):
    Thread.__init__(self)
    self.stopped = event

  def run(self):
    while not self.stopped.wait(IMAGE_INTERVAL):
      capture_image()


# Thread handling serial data receival
class SerialReceivalThread(Thread):
  def __init__(self, event):
    Thread.__init__(self)
    self.stopped = event

  def run(self):
    while not self.stopped.wait(SERIAL_INTERVAL):
      read_serial()


# Thread handling robot action
class RobotActionThread(Thread):
  def __init__(self, event):
    Thread.__init__(self)
    self.stopped = event

  def run(self):
    while not self.stopped.wait(ACTION_INTERVAL):
      calculate_robot_action()


# Init method
def init():
  global CAMERA
  global SERIAL
  global LOG_LEVEL

  # Init camera
  # CAMERA = picamera.PiCamera()
  # # CAMERA.crop = (0.0, 0.25, 1, 0.75)
  # CAMERA.resolution = (720, 480)
  # CAMERA.brightness = 70

  # Init sensor data array

  # Init serial
  log(0, "Retrieving Arduino Port")
  arduino_port = ""
  ports = serial.tools.list_ports.comports()
  for port in ports:
    if port.description == "Arduino Uno":
      arduino_port = port.device
  log(0, "Arduino found at {0}".format(arduino_port))

  log(0, "Initiating Serial communication")
  SERIAL = serial.Serial( #TODO
      port=arduino_port,
      baudrate=9600,
      parity=serial.PARITY_ODD,
      stopbits=serial.STOPBITS_TWO,
      bytesize=serial.SEVENBITS
  )

  try:  
    log(0, "Serial comm init success")
    SERIAL.isOpen()
    #TODO when SERIAL.close()
  except Exception as e:
    log(3, "Serial connection could not be established. Reason: {0}".format(e))
    #TODO do some kind of simple handshake with robot
    exit()

  # Init threads
  log(0, "Starting img, serial, action threads")

  # Image capturing
  # imageThreadStopFlag = Event()
  # imageThread = ImageThread(imageThreadStopFlag)
  # imageThread.start()
  # #imageThreadStopFlag.set()    # this will stop the timer in the thread

  # Serial communication receiving end
  serialReceivalThreadStopFlag = Event()
  serialReceivalThread = SerialReceivalThread(serialReceivalThreadStopFlag)
  serialReceivalThread.start()
  #serialReceivalThreadStopFlag.set()    # this will stop the timer in the thread

  # Robot action calculating thread
  robotActionThreadStopFlag = Event()
  robotActionThread = RobotActionThread(robotActionThreadStopFlag)
  robotActionThread.start()
  #robotActionThreadStopFlag.set()    # this will stop the timer in the thread
  log(0, "Init finished")

def send_robot_drive(velocity, rotation):
  """ Send drive instructions to robot
  """
  # M (int)velocity (int)rotation
  #TODO verify correctness of input
  log(0, "Asking robot to set motor drive to: v{0} r{1}".format(velocity, rotation))
  send_robot_instruction("M {0} {1}".format(velocity, rotation));


def send_robot_sensor_aim(angle):
  """ Send sensor aim instructions to robot
  """
  # S (int)angle
  #TODO verify correctness of input
  log(0, "Asking robot to aim sensor at {0} degrees".format(angle))
  send_robot_instruction("S {0}".format(angle));


def send_robot_instruction(instruction):
  """ Send instructions to robot
  Uses USART over USB
  """
  try:
    SERIAL.flushOutput()    # Empty output buffer
    SERIAL.write(instruction)
  except Exception as e:
    log(4, "Failed writing instruction to serial".format(e))


def read_serial():
  """ Robot sends instructions over USB over USART
  Read serial will read the last received line
  Calls add_ultrasonic_sensor_measurement for further processing

  Format:
  USM (int)angle (int)measuredDistanceCM
  """
  log(0, "Waiting for serial input")
  buffer_string = ''
  while True:
    buffer_string = buffer_string + SERIAL.read(SERIAL.inWaiting())
    if '\n' in buffer_string:
      lines = buffer_string.split('\n') # Guaranteed to have at least 2 entries
      line = lines[-2]
      log(0, "Serial: {0}".format(line))
      
      parts = line.split(" ")
      if len(parts) == 4 and parts[0] == "USM":
        # Parse ultrasonic measurement
        log(0, "Serial: {0}".format(line))
        add_ultrasonic_sensor_measurement(parts[1], parts[2], parts[3])
      else:
        log(1, "Did not parse line. Format not recognised. {0} {1}".format(line, parts[0]))

      break;


def add_ultrasonic_sensor_measurement(timestamp, bucket, value):
  """ Adds given measurement to data structures
  TODO further explanation
  """
  log(0, "Reveived USM for t={0}, b={1}, v={2}".format(timestamp, bucket, value))

  # If bucket does not exist, create new ones
  # Note: Also overwrites when one dict does not contain. Should not happen
  if bucket not in ULTRASONICSENSOR_DATA or bucket not in ULTRASONICSENSOR_TIMESTAMPS:
    ULTRASONICSENSOR_DATA[bucket] = []
    ULTRASONICSENSOR_TIMESTAMPS[bucket] = []
    log(0, "First measurent for bucket {0}".format(bucket))

  # Append value to data
  if ULTRASONICSENSOR_TIMESTAMPS[bucket] == [] or ULTRASONICSENSOR_TIMESTAMPS[bucket][len(ULTRASONICSENSOR_TIMESTAMPS[bucket]) - 1] != long(timestamp):
    ULTRASONICSENSOR_DATA[bucket].append(int(value))
    ULTRASONICSENSOR_TIMESTAMPS[bucket].append(long(timestamp))
    log(1, "Added sensor data at t={0}, b={1}, v={2}".format(timestamp, bucket, value))
  else:
    log(1, "Ignoring read value. Already received and parsed. t={0}, b={1}, v={2}".format(timestamp, bucket, value))

  # Cleanup
  if len(ULTRASONICSENSOR_TIMESTAMPS[bucket]) > MAX_HISTORY:
    ULTRASONICSENSOR_DATA[bucket] = ULTRASONICSENSOR_DATA[bucket][-HISTORY_PURGE_TO:]
    ULTRASONICSENSOR_TIMESTAMPS[bucket] = ULTRASONICSENSOR_DATA[bucket][-HISTORY_PURGE_TO:]


def capture_image():
  """ Take image using PI camera
  Saves image to given path
  """
  # timestamp = time.time()
  # CAMERA.capture('{0}/{1}/{2}.jpg'.format(IMAGE_PATH, VERSION, timestamp), quality=50)
  # log(0, 'Image: {0}/{1}/{2}.jpg'.format(IMAGE_PATH, VERSION, timestamp))

  # Cleanup
  #TODO


def calculate_robot_action():
  """ Either information has been received from camera or robot
  Act on this information. Train ML, respond given trained Model
  """
  #TODO
  #log(0, "Action Thread")


def log(level, msg):
  """ Print to sysout depending on given log level
  TODO write to file
  TODO also print line or method
  OR replace with a logging library
  
  Logs when level>=LOG_LEVEL

  0=debug
  1=info
  2=warn
  3=error
  """
  weight = "?"
  if level>=LOG_LEVEL:
    if level == 0:
      weight = "DEBUG"
    elif level == 1:
      weight = "INFO"
    elif level == 2:
      weight = "WARN"
    elif level == 3:
      weight = "ERROR"
    else:
      log(3, "Invalid log level: {0}".format(level))
    print "{0}: {1}".format(weight, msg)

# Init
init()