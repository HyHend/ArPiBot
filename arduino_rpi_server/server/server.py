from flask import Flask, render_template
from flask_socketio import SocketIO
from flask_socketio import send, emit
import json


LOG_LEVEL = 0                     # 0=debug, 1=info, 3=warn, 4=error
PRINT_LOG_WEBSOCKET = False       # True will print log lines over websocket connection
HOST = "x.x.x.x"
PORT = 5000


app = Flask(__name__)
socketio = SocketIO(app, logger=False)

@app.route('/')
def root():
    return app.send_static_file('index.html')

@socketio.on('robot')
def handle_robot_message(message):
  try:
    jsonMsg = json.loads(message)
  except:
    log(2,"Could not parse JSON: {0}".format(message))
    return

  # Parse and handle know message types
  if 'LOG' in jsonMsg and PRINT_LOG_WEBSOCKET:
    log(0, "Robot LOG ({0}): {1}".format(jsonMsg['LOG']['weight'], jsonMsg['LOG']['message']))
  if 'ECHO' in jsonMsg:
    # 'ECHO' is an API test. Do we have a connection?
    send_to_webclient({'received_over_channel':"robot",'original_msg':jsonMsg['ECHO']})
  #TODO handle more message types


@socketio.on('webcl')
def handle_webclient_message(message):
  try:
    jsonMsg = json.loads(message)
  except:
    log(2,"Could not parse JSON: {0}".format(message))
    return

  # Parse and handle know message types
  if 'ROBOT' in jsonMsg:
    # 'ROBOT' is a type of message containing 
    # information to be directly relayed to bot
    send_to_robot(jsonMsg['ROBOT'])
  if 'ECHO' in jsonMsg:
    # 'ECHO' is an API test. Do we have a connection?
    send_to_webclient({'received_over_channel':"webcl",'original_msg':jsonMsg['ECHO']})



def send_to_robot(message):
  """ Sends given message to connected robot
  Encodes message to JSON string
  Does not check for command validity
  """
  try:
    msgStr = json.dumps(message)
  except:
    log(2,"Could not convert message to JSON string: {0}".format(message))
    return

  try:
    log(0,"Msg to Robot: {0}".format(msgStr))
    emit('robot', msgStr, broadcast=True)
  except:
    log(2,"Could not communicate to robot: {0}".format(msgStr))


def send_to_webclient(message):
  """ Sends given message to connected web client
  Encodes message to JSON string
  Does not check for command validity
  """
  try:
    msgStr = json.dumps(message)
  except:
    log(2,"Could not convert message to JSON string: {0}".format(message))
    return

  try:
    log(0,"Msg to WebCl: {0}".format(msgStr))
    emit('webcl', msgStr, broadcast=True)
  except:
    log(2,"Could not communicate to web client: {0}".format(msgStr))


def log(level, msg):
  """ Print to sysout depending on given log level
  
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
    print("{0}: {1}".format(weight, msg))


if __name__ == '__main__':
    socketio.debug = True
    socketio.run(app, host=HOST, port=PORT)