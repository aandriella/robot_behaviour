'''
This is a module to reproduce a robot action combining speech,
facial expression and gesture. Every time we check if the action
 that by default is performed only with speech will include also face
 and gesture
'''
import rospy
from robot_behaviour.face_reproducer import Face
from robot_behaviour.speech_reproducer import Voice
from robot_behaviour.gesture_reproducer import Gesture
import robot_behaviour.config.voice_utterances.sentences as s
'''
IF CORRECT ACTION HAS BEEN PERFORMED
'''
def congratulate(speech, counter, face=None, gesture=None):
  '''
  It reproduces the audio, face and gesture provided as input
  Args:
    speech:
    face:
    gesture:
  Return:
     Bool: if the action has been executed successfully
  '''
  print("Congrats funct")
  b_executed = False


  if face!=None and gesture==None:
    speech.text_to_speech(s.sentences['congratulation'][counter])
    rospy.sleep(0.1)
    face.reproduce_face_expression("happy")
    b_executed = True
  elif face==None and gesture!=None:
    pass
  else:
    pass

  return b_executed

'''
IF WRONG ACTION HAS BEEN PERFORMED 
'''
def compassion(speech, counter, face=None, gesture=None):
  '''
    It reproduces the audio, face and gesture provided as input
    Args:
      speech:
      counter: in order to generate different sentences
      face:
      gesture:
    Return:
       Bool: if the action has been executed successfully
    '''
  print("Compassion funct")
  b_executed = False
  if face!=None and gesture==None:
    speech.text_to_speech(s.sentences['compassion'][counter])
    rospy.sleep(0.1)
    face.reproduce_face_expression("sad")
  elif face==None and gesture!=None:
    pass
  else:
    pass
  return b_executed


def move_token_back(who, token_from, token_to, speech, face=None, gesture=None):
  '''
    It reproduces the audio, face and gesture provided as input.
    The robot will pick the wrong token and move it back to its initial location
    Args:
      speech:
      face:
      gesture:
      who: if the robot has to move back the token or the user
      token_from: the current loc of the token
      token_to: the loc where the token has to be moved
    Return:
       Bool: if the action has been executed successfully
    '''
  print("Move token back")
  b_executed = False
  if face!=None and gesture==None:
    if who == "robot":
      speech.text_to_speech(s.sentences['robot_move_back'].pop())
    else:
      speech.text_to_speech(s.sentences['user_move_back'].pop())
    rospy.sleep(0.1)
    face.reproduce_face_expression("neutral")
  elif face==None and gesture!=None:
    pass#TODO
  else:
    pass#TODO
  return b_executed
'''
LEVELS OF ASSISTANCE FURNISHED BY THE ROBOT
'''

def no_assistance(speech, face=None, gesture=None):
  '''
  It provides no assistance to the user, basically only inform the user of moving a token
  Args:
    speech:
    face:
    gesture:
  Return:
  '''
  print("Lev_0")
  b_executed = False
  if face!=None and gesture==None:
    speech.text_to_speech(s.sentences['lev_0'].pop())
    rospy.sleep(0.1)
    face.reproduce_face_expression("neutral")
  elif face==None and gesture!=None:
    pass
  else:
    pass
  return b_executed

def encouragement(speech, counter, face=None, gesture=None):
  '''
  It eoncurages the user to move a token
    Args:
      speech:
      counter: to generate different sentences
      face:
      gesture:
    Return:
       Bool: if the action has been executed successfully
    '''
  print("Lev_1")
  b_executed = False
  if face!=None and gesture==None:
    speech.text_to_speech(s.sentences['lev_1'][counter])
    rospy.sleep(0.1)
    face.reproduce_face_expression("neutral")
  elif face==None and gesture!=None:
    pass
  else:
    pass
  return b_executed


def suggest_row(row, speech, counter, face=None, gesture=None):
  '''
  It tells the user the row of the board where the correct token is
    Args:
      row: the row of the board where the robot has to point
      speech:
      counter: to generate different sentences
      face:
      gesture:
    Return:
       Bool: if the action has been executed successfully
    '''
  print("Lev_2")
  b_executed = False
  if face!=None and gesture==None:
    speech.text_to_speech(s.sentences['lev_2'][counter]+"."+str(row))
    rospy.sleep(0.1)
    face.reproduce_face_expression("happy")
  elif face==None and gesture!=None:
    pass
  else:
    pass
  return b_executed

def suggest_cells(speech, counter, face=None, gesture=None, *tokens):
  '''
  It tells the user the cells near to the correct token (included the latter)
    Args:
      speech:
      counter: to generate different sentences
      face:
      gesture:
      tokens: the tokens on the board where the robot has to point
    Return:
       Bool: if the action has been executed successfully
    '''
  print("Lev_3")
  b_executed = False
  tokens_id = [t[:][0] for t in tokens]
  tokens_loc = [t[:][1] for t in tokens]
  tokens_id_to_str = " . ".join([str(t) for t in tokens_id])
  if face!=None and gesture==None:
    speech.text_to_speech(s.sentences['lev_3'][counter]+"."+tokens_id_to_str)
    rospy.sleep(0.1)
    face.reproduce_face_expression("happy")
  elif face==None and gesture!=None:
    pass
  else:
    pass
  return b_executed

def suggest_solution(speech, counter, face=None, gesture=None, *token):
  '''
  It tells the user the cell where the correct token is
    Args:
      speech:
      counter:
      face:
      gesture:
      token: The correct token, id and location
    Return:
       Bool: if the action has been executed successfully
    '''
  print("Lev_4")
  b_executed = False
  tokens_id = [t[:][0] for t in tokens]
  tokens_loc = [t[:][1] for t in tokens]
  tokens_id_to_str = " . ".join([str(t) for t in tokens_id])
  if face != None and gesture == None:
    speech.text_to_speech(s.sentences['lev_4'][counter] + tokens_id_to_str)
    rospy.sleep(0.1)
    face.reproduce_face_expression("happy")
  elif face == None and gesture != None:
    pass
  else:
    pass
  return b_executed

def offer_solution(speech, face=None, gesture=None, *token):
  '''
  It offers the user the correct token
    Args:
      speech:
      face:
      gesture:
      token: The correct token, id and location
    Return:
       Bool: if the action has been executed successfully
    '''
  print("Lev_5")
  b_executed = False
  tokens_id = [t[:][0] for t in tokens]
  tokens_loc = [t[:][1] for t in tokens]
  tokens_id_to_str = " . ".join([str(t) for t in tokens_id])
  if face != None and gesture == None:
    speech.text_to_speech(s.sentences['lev_5'][0])
    rospy.sleep(0.1)
    face.reproduce_face_expression("happy")
    speech.text_to_speech(s.sentences['lev_5'][1])
  elif face == None and gesture != None:
    pass
  else:
    pass
  return b_executed

def move_onbehalf(speech, face=None, gesture=None):
  '''
    It moves the token on the behalf of the user
      Args:
        speech:
        face:
        gesture:
        token: The correct token, id and location
      Return:
         Bool: if the action has been executed successfully
      '''
  print("move_onbehalf")
  b_executed = False
  if face!=None and gesture==None:
    speech.text_to_speech(s.sentences['max_attempt'].pop())
    rospy.sleep(0.1)
    face.reproduce_face_expression("happy")
    b_executed = True
  elif face==None and gesture!=None:
    pass
  else:
    pass

def pick(positive, speech, counter, face, gesture):
  '''
  say something when a token is grasped depending on positive value
  Args:
    positive:
    speech:
    counter:
    face:
    gesture:
  Returns:
  '''
  print("pick")
  b_executed = False
  if face!=None and gesture==None:
    if positive:
      speech.text_to_speech(s.sentences['pick_ok'][counter])
      rospy.sleep(0.1)
      face.reproduce_face_expression("happy")
      b_executed = True
    else:
      speech.text_to_speech(s.sentences['pick_no'][counter])
      rospy.sleep(0.1)
      face.reproduce_face_expression("confused")
      b_executed = True
  elif face==None and gesture!=None:
    pass
  else:
    pass



def timeout(speech, face=None, gesture=None):
  '''
    It tells the user that the time available for the current move ended
      Args:
        speech:
        face:
        gesture:
        token: The correct token, id and location
      Return:
         Bool: if the action has been executed successfully
      '''
  print("timeout")
  b_executed = False
  if face!=None and gesture==None:
    speech.text_to_speech(s.sentences['timeout'].pop())
    rospy.sleep(0.1)
    face.reproduce_face_expression("happy")
    b_executed = True
  elif face==None and gesture!=None:
    pass
  else:
    pass
  return b_executed

def instruction(speech, face=None, gesture=None):
  '''The agent provides the instructions of the exercise
  Args:
    speech:
    face:
    gesture:
    token: The
    correct
    token, id and location
  Return:
    Bool:
    if the action has been executed successfully
  '''
  print("instruction")
  b_executed = False
  if face!=None and gesture==None:
    speech.text_to_speech(s.sentences['instruction'].pop())
    rospy.sleep(0.1)
    face.reproduce_face_expression("happy")
    b_executed = True
  elif face==None and gesture!=None:
    pass
  else:
    pass



robot_actions = {
"instruction": instruction,
"congrats": congratulate,
"compassion": compassion,
"move_back": move_token_back,
"lev_0": no_assistance,
"lev_1": encouragement,
"lev_2": suggest_row,
"lev_3": suggest_cells,
"lev_4": suggest_solution,
"lev_5": offer_solution,
"max_attempt": move_onbehalf,
"timeout": timeout,
"pick": pick
}

speech = Voice("en_GB")
gesture = None
face = Face()
counter = 0
token_from = ""
token_to = ""
who = "robot"
row = 3
tokens = [("111",13)]
positive = False
#robot_actions["congrats"].__call__(speech, counter, face, gesture)
#robot_actions["compassion"].__call__(speech, counter, face, gesture)
# #take in input 5 params
#robot_actions["move_back"].__call__(who, token_from, token_to, speech, face, gesture)
#robot_actions["lev_0"].__call__(speech, face, gesture)
#rospy.sleep(3.0)
#robot_actions["lev_1"].__call__(speech, counter, face, gesture)
#robot_actions["lev_2"].__call__(row, speech, counter, face, gesture)
#robot_actions["lev_3"].__call__(speech, counter,face, gesture, *tokens)
#robot_actions["lev_4"].__call__(speech, counter, face, gesture, *tokens)
#robot_actions["lev_5"].__call__(speech, face, gesture, tokens)
#robot_actions["instruction"].__call__(speech, face, gesture)
#robot_actions["max_attempt"].__call__(speech, face, gesture)
#robot_actions["timeout"].__call__(speech, face, gesture)
robot_actions["pick"].__call__(positive, speech, counter, face, gesture)