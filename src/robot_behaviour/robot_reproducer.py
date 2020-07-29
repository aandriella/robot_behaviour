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

class Robot:
  def __init__(self, speech, face=None, gesture=None):
    '''
    :param speech: instance of class Speech
    :param face:  instance of class Face
    :param gesture: instance of class Gesture
    '''
    self.speech = speech
    self.face = face
    self.gesture = gesture

    self.action = {
      "instruction": self.instruction,
      "congrats": self.congratulate,
      "compassion": self.compassion,
      "move_back": self.move_token_back,
      "lev_0": self.no_assistance,
      "lev_1": self.encouragement,
      "lev_2": self.suggest_row,
      "lev_3": self.suggest_cells,
      "lev_4": self.suggest_solution,
      "lev_5": self.offer_solution,
      "max_attempt": self.move_onbehalf,
      "timeout": self.timeout,
      "pick": self.pick
    }

  def congratulate(self, counter):
    '''
    It greats the user for being performed a correct move
    Args:
      counter: get one of the sentences in the list
    Return:
       Bool: if the action has been executed successfully
    '''
    print("Congrats method")
    b_executed = False
    if self.face!=None and self.gesture==None:
      self.speech.text_to_speech(s.sentences['congratulation'][counter])
      rospy.sleep(0.1)
      self.face.reproduce_face_expression("happy")
      b_executed = True
    elif self.face == None and self.gesture != None:
      '''N.B For this kind of action we did not plan to move the robot'''
      assert "Error Congrats does not contemplate any gesture"

    else:
      '''N.B For this kind of action we did not plan to move the robot'''
      assert "Error Congrats does not contemplate any gesture"


    return b_executed

  def compassion(self, counter):
    '''
    offer compassion to the user for being performed a werong move
    Args:
      counter: in order to generate different sentences
    Return:
      Bool: if the action has been executed successfully
      '''
    print("Compassion funct")
    b_executed = False
    if self.face!=None and self.gesture==None:
      self.speech.text_to_speech(s.sentences['compassion'][counter])
      rospy.sleep(0.1)
      self.face.reproduce_face_expression("sad")
      b_executed = True
    elif self.face==None and self.gesture!=None:
      '''N.B For this kind of action we did not plan to move the robot'''
      assert "Error Compassion does not contemplate any gesture"
    else:
      '''N.B For this kind of action we did not plan to move the robot'''
      assert "Error Compassion does not contemplate any gesture"

    return b_executed


  def move_token_back(self, who, token):
    '''
      The robot will pick the wrong token and move it back to its initial location
      if who = "robot" otherwise the user is asked to move back the token
      Args:
        who: if "robot" the robot moves back the token if "user" the user has to do it
        token: the token to move with its from and to loc
      Return:
         Bool: if the action has been executed successfully
      '''
    print("Move token back")
    token_id, token_from, token_to =  token
    b_executed = False
    if self.face!=None and self.gesture==None:
      if who == "robot":
        self.speech.text_to_speech(s.sentences['robot_move_back'].pop())
      else:
        self.speech.text_to_speech(s.sentences['user_move_back'].pop())
      rospy.sleep(0.1)
      self.face.reproduce_face_expression("neutral")
      b_executed = True
    elif self.face==None and self.gesture!=None:
      if who == "robot":
        self.speech.text_to_speech(s.sentences['robot_move_back'].pop())
      else:
        self.speech.text_to_speech(s.sentences['user_move_back'].pop())
      rospy.sleep(0.1)
      # TODO: validation on the robot
      success = self.gesture.pick_and_place(token_from, token_to)
      self.gesture.initial_pos()
      b_executed = True
    else:
      #TODO validation on the robot
      if who == "robot":
        self.speech.text_to_speech(s.sentences['robot_move_back'].pop())
      else:
        self.speech.text_to_speech(s.sentences['user_move_back'].pop())
      rospy.sleep(0.1)
      self.face.reproduce_face_expression("neutral")
      success = self.gesture.pick_and_place(token_from, token_to)
      self.gesture.initial_pos()
      b_executed = True

    return b_executed
  '''
  LEVELS OF ASSISTANCE FURNISHED BY THE ROBOT
  '''
  def no_assistance(self):
    '''
    It provides no assistance to the user, basically only inform the user of moving a token
    Args:
      No arg
    Return:
      Bool: if the action has been executed successfully
    '''
    print("Lev_0")
    b_executed = False
    if self.face!=None and self.gesture==None:
      self.speech.text_to_speech(s.sentences['lev_0'].pop())
      rospy.sleep(0.1)
      self.face.reproduce_face_expression("neutral")
      b_executed = True
    elif self.face == None and self.gesture != None:
      '''N.B For this kind of action we did not plan to move the robot'''
      assert "Error Lev 0 does not contemplate any gesture"
    else:
      '''N.B For this kind of action we did not plan to move the robot'''
      assert "Error Lev 0 does not contemplate any gesture"
    return b_executed

  def encouragement(self, counter):
    '''
    It eoncurages the user to move a token
      Args:
        counter: to generate different sentences
      Return:
         Bool: if the action has been executed successfully
      '''
    print("Lev_1")
    b_executed = False
    if self.face!=None and self.gesture==None:
      self.speech.text_to_speech(s.sentences['lev_1'][counter])
      rospy.sleep(0.1)
      self.face.reproduce_face_expression("neutral")
      b_executed = True
    elif self.face==None and self.gesture!=None:
      assert "Error Lev 1 does not contemplate any gesture"
    else:
      assert "Error Lev 1 does not contemplate any gesture"
    return b_executed


  def suggest_row(self, row, counter):
    '''
    It tells the user the row of the board where the correct token is
      Args:
        row: the row of the board where the robot has to point
        counter: to generate different sentences
      Return:
         Bool: if the action has been executed successfully
      '''
    print("Lev_2")
    b_executed = False
    if self.face!=None and self.gesture==None:
      self.speech.text_to_speech(s.sentences['lev_2'][counter]+str(row))
      rospy.sleep(0.1)
      self.face.reproduce_face_expression("neutral")
      b_executed = True
    elif self.face==None and self.gesture!=None:
      self.speech.text_to_speech(s.sentences['lev_2'][counter]+str(row))
      rospy.sleep(0.1)
      #TODO: test on the robot and see if the voice can be sinc with the movement otherwise include speech and text in the gesture method
      self.gesture.suggest_row(row)
      b_executed = True
    else:
      self.speech.text_to_speech(s.sentences['lev_2'][counter] + str(row))
      rospy.sleep(0.1)
      self.face.reproduce_face_expression("neutral")
      # TODO: test on the robot and see if the voice can be sinc with the movement otherwise include speech and text in the gesture method
      self.gesture.suggest_row(row)
      b_executed = True
    return b_executed

  def suggest_cells(self, counter, token, *tokens):
    '''
    It tells the user the cells near to the correct token (included the latter)
      Args:
        counter: to generate different sentences
        token: the correct token
        tokens: the subset of tokens on the board where the robot has to point
      Return:
         Bool: if the action has been executed successfully
      '''
    print("Lev_3")
    b_executed = False
    #used by gesture
    token_sol_id, token_sol_from, _ = token
    #used by speech
    tokens_id = [t[:][0] for t in tokens]
    tokens_loc = [t[:][1] for t in tokens]
    tokens_id_to_str = "   ".join([str(t) for t in tokens_id])
    if self.face!=None and self.gesture==None:
      # TODO: test on the robot and see if the voice can be sinc with the movement otherwise include speech and text in the gesture method
      self.speech.text_to_speech(s.sentences['lev_3'][counter]+tokens_id_to_str)
      rospy.sleep(0.1)
      self.face.reproduce_face_expression("happy")
      b_executed = True
    elif self.face==None and self.gesture!=None:
      # TODO: test on the robot and see if the voice can be sinc with the movement otherwise include speech and text in the gesture method
      self.speech.text_to_speech(s.sentences['lev_3'][counter])
      rospy.sleep(0.1)
      self.gesture.suggest_subset(token_sol_from, self.reproduce_sentence, text= tokens_id_to_str, delay=0.1)
      b_executed = True
    else:
      # TODO: test on the robot and see if the voice can be sinc with the movement otherwise include speech and text in the gesture method
      self.speech.text_to_speech(s.sentences['lev_3'][counter])
      rospy.sleep(0.1)
      self.face.reproduce_face_expression("happy")
      self.gesture.suggest_subset(token_sol_from, self.reproduce_sentence, text=tokens_id_to_str, delay=0.1)
      b_executed = True
    return b_executed

  def suggest_solution(self, counter, token):
    '''
    It tells the user the cell where the correct token is
      Args:
        counter: to generate different sentences
        token: The correct token, id and location
      Return:
         Bool: if the action has been executed successfully
      '''
    print("Lev_4")
    b_executed = False
    #need for the gesture
    token_sol_id, token_sol_from, _ = token
    #need for speech
    tokens_id_to_str =  str(token_sol_id)
    if self.face != None and self.gesture == None:
      self.speech.text_to_speech(s.sentences['lev_4'][counter] + tokens_id_to_str)
      rospy.sleep(0.1)
      self.face.reproduce_face_expression("happy")
      b_executed = True
    elif self.face == None and self.gesture != None:
      # TODO: test on the robot and see if the voice can be sinc with the movement otherwise include speech and text in the gesture method
      self.speech.text_to_speech(s.sentences['lev_4'][counter])
      self.gesture.suggest_solution(token_sol_from, self.reproduce_sentence, tokens_id_to_str, delay=0.1)
      b_executed = True
    else:
      # TODO: test on the robot and see if the voice can be sinc with the movement otherwise include speech and text in the gesture method
      self.speech.text_to_speech(s.sentences['lev_4'][counter])
      rospy.sleep(0.1)
      self.face.reproduce_face_expression("happy")
      self.gesture.suggest_solution(token_sol_from, self.reproduce_sentence(tokens_id_to_str), delay=0.1)
      b_executed = True
    return b_executed

  def offer_solution(self, token):
    '''
    It offers the user the correct token
      Args:
        token: The correct token, id and location
      Return:
         Bool: if the action has been executed successfully
      '''
    print("Lev_5")
    b_executed = False
    #need for gesture
    token_sol_id, token_sol_from, _ = token
    #need for speech?
    token_id_to_str = " . ".join([str(t) for t in token_sol_id])
    if self.face != None and self.gesture == None:
      assert "This level has not been designed to be provided without the robot"
      self.speech.text_to_speech(s.sentences['lev_5'][0])
      rospy.sleep(0.1)
      self.face.reproduce_face_expression("happy")
      self.speech.text_to_speech(s.sentences['lev_5'][1])
      b_executed = True
    elif self.face == None and self.gesture != None:
      # TODO: test on the robot and see if the voice can be sinc with the movement otherwise include speech and text in the gesture method
      self.speech.text_to_speech(s.sentences['lev_5'][0])
      rospy.sleep(0.1)
      self.gesture.offer_token(token_sol_from, self.reproduce_sentence, s.sentences['lev_5'][1])
      b_executed = True
    else:
      # TODO: test on the robot and see if the voice can be sinc with the movement otherwise include speech and text in the gesture method
      self.speech.text_to_speech(s.sentences['lev_5'][0])
      rospy.sleep(0.1)
      self.face.reproduce_face_expression("happy")
      self.gesture.offer_token(token_sol_from, self.reproduce_sentence, s.sentences['lev_5'][1])
      b_executed = True
    return b_executed

  def move_onbehalf(self, token):
    '''
      It moves the token on the behalf of the user
        Args:
          token: The correct token, id and location
        Return:
           Bool: if the action has been executed successfully
        '''
    print("move_onbehalf")
    token_sol_id, token_sol_from, token_sol_to = token
    token_id_to_str = " . ".join([str(t) for t in token_sol_id])
    b_executed = False
    if self.face!=None and self.gesture==None:
      self.speech.text_to_speech(s.sentences['max_attempt'].pop())
      rospy.sleep(0.1)
      self.face.reproduce_face_expression("happy")
      b_executed = True
    elif self.face==None and self.gesture!=None:
      # TODO: test on the robot
      self.speech.text_to_speech(s.sentences['max_attempt'].pop())
      rospy.sleep(0.1)
      self.gesture.pick_and_place(token_sol_from, token_sol_to)
      b_executed = True
    else:
      # TODO: test on the robot
      self.speech.text_to_speech(s.sentences['max_attempt'].pop())
      rospy.sleep(0.1)
      self.face.reproduce_face_expression("happy")
      self.gesture.pick_and_place(token_sol_from, token_sol_to)
      b_executed = True

  def pick(self, positive, counter):
    '''
    say something when a token is grasped depending on positive value
    Args:
      positive: while is a positive or a negative feedback
        counter: to generate different sentences
    Returns:
    '''
    print("pick")
    b_executed = False
    if self.face!=None and self.gesture==None:
      if positive:
        self.speech.text_to_speech(s.sentences['pick_ok'][counter])
        rospy.sleep(0.1)
        self.face.reproduce_face_expression("happy")
        b_executed = True
      else:
        self.speech.text_to_speech(s.sentences['pick_no'][counter])
        rospy.sleep(0.1)
        self.face.reproduce_face_expression("confused")
        b_executed = True
    elif self.face==None and self.gesture!=None:
      assert "Error Pick does not contemplate any gesture"
    else:
      assert "Error Pick does not contemplate any gesture"

  def timeout(self):
    '''
      It tells the user that the time available for the current move ended
        Args:
          token: The correct token, id and location
        Return:
           Bool: if the action has been executed successfully
        '''
    print("timeout")
    b_executed = False
    if self.face!=None and self.gesture==None:
      self.speech.text_to_speech(s.sentences['timeout'].pop())
      rospy.sleep(0.1)
      self.face.reproduce_face_expression("happy")
      b_executed = True
    elif self.face==None and self.gesture!=None:
      assert "Error Timeout does not contemplate any gesture"
    else:
      assert "Error Timeout does not contemplate any gesture"
    return b_executed

  def instruction(self):
    '''The agent provides the instructions of the exercise
    Args:
    Return:
      Bool:
      if the action has been executed successfully
    '''
    print("instruction")
    b_executed = False
    if self.face!=None and self.gesture==None:
      self.speech.text_to_speech(s.sentences['instruction'].pop())
      rospy.sleep(0.1)
      self.face.reproduce_face_expression("happy")
      b_executed = True
    elif self.face==None and self.gesture!=None:
      assert "Error Instruction does not contemplate any gesture"
    else:
      assert "Error Instruction does not contemplate any gesture"

  def reproduce_sentence(self, text):
    self.speech.text_to_speech(text)

  def fake_function(self, reproduce_sentence, text):
    i=0
    while(i<1000):
      if i%10==0:
        reproduce_sentence(text)
      else:
        pass
      i += 1


def main():
  speech = Voice("en_GB")
  gesture = Gesture()
  face = None
  robot = Robot(speech, face, gesture)

  counter = 0
  token_from = ""
  token_to = ""
  who = "robot"
  row = 3
  tokens = [("111",13), ("256", 15), ("341", 18)]
  token = ("111", 2, 13)
  positive = False
  #robot.fake_function(robot.play_sentence)

  #robot.action["congrats"].__call__(counter)
  #robot.action["compassion"].__call__(counter)
  # #take in input 5 params
  #robot.action["move_back"].__call__(who, token)
  #robot.action["lev_0"].__call__()
  #rospy.sleep(3.0)
  #robot.action["lev_1"].__call__(counter)
  #robot.action["lev_2"].__call__(row, counter)
  #robot.action["lev_3"].__call__(counter, token, *tokens)
  #robot.action["lev_4"].__call__(counter, token)
  #robot.action["lev_5"].__call__(token)
  #robot.action["instruction"].__call__()
  #robot.action["max_attempt"].__call__(token)
  #robot.action["timeout"].__call__()
  #robot.action["pick"].__call__(positive, counter)

if __name__=="__main__":
  main()