'''
This is a module to reproduce a robot action combining speech,
facial expression and gesture. Every time we check if the action
 that by default is performed only with speech will include also face
 and gesture
'''
import rospy
import random
import ast
from robot_behaviour.face_reproducer import Face
from robot_behaviour.speech_reproducer import Speech
from robot_behaviour.gesture_reproducer import Gesture
import robot_behaviour.sentences as s

class Robot:
  def __init__(self, speech, sentences_file, face=None, gesture=None):
    '''
    :param speech: instance of class Speech
    :param sentences_file: the file where all the sentences are stored
    :param face:  instance of class Face
    :param gesture: instance of class Gesture
    '''
    self.speech = speech
    self.sentences = self.load_sentences(sentences_file)
    self.face = face
    self.gesture = gesture

    self.action = {
      "instruction": self.instruction,
      "congrats": self.congratulate,
      "compassion": self.compassion,
      "move_back": self.move_token_back,
      "assistance": self.assistance,
      "max_attempt": self.move_onbehalf,
      "timeout": self.timeout,
      "pick": self.pick
    }

    self.assistance_level = {
      "lev_0": self.no_assistance,
      "lev_1": self.encouragement,
      "lev_2": self.suggest_row,
      "lev_3": self.suggest_cells,
      "lev_4": self.suggest_solution,
      "lev_5": self.offer_solution
    }

  def send_to_rest(self):
    self.gesture.initial_pos()


  def load_sentences(self, file):
    file = open(file, "r")
    contents = file.read()
    self.senteces = ast.literal_eval(contents)
    file.close()
    return self.senteces

  def congratulate(self, counter, facial_expression):
    '''
    It greats the user for being performed a correct move
    Args:
      counter: get one of the sentences in the list
    Return:
       Bool: if the action has been executed successfully
    '''
    print("Congrats method")
    b_executed = False
    if counter >=len(self.sentences['congratulation'])-1: counter=random.randint(0, len(self.sentences['congratulation'])-1)
    time_to_reproduce = self.speech.text_to_speech(self.sentences['congratulation'][counter], locked = True)
    rospy.sleep(time_to_reproduce)
    self.face.reproduce_face_expression(facial_expression)
    b_executed = True
    return b_executed

  def compassion(self, counter, facial_expression):
    '''
    offer compassion to the user for being performed a werong move
    Args:
      counter: in order to generate different sentences
    Return:
      Bool: if the action has been executed successfully
      '''
    print("Compassion funct")
    b_executed = False
    if counter >=len(self.sentences['compassion'])-1: counter=random.randint(0, len(self.sentences['compassion'])-1)
    if self.face!=None and self.gesture==None:
      time_to_reproduce = self.speech.text_to_speech(self.sentences['compassion'][counter], locked = True)
      rospy.sleep(time_to_reproduce)
      self.face.reproduce_face_expression(facial_expression)
      b_executed = True
    elif self.face==None and self.gesture!=None:
      '''N.B For this kind of action we did not plan to move the robot'''
      assert "Error Compassion does not contemplate any gesture"
    else:
      '''N.B For this kind of action we did not plan to move the robot'''
      assert "Error Compassion does not contemplate any gesture"

    return b_executed


  def move_token_back(self, who, token, counter, facial_expression):
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
        if counter >= len(self.sentences['robot_move_back'])-1: counter = random.randint(0, len(self.sentences['robot_move_back'])-1)
        self.speech.text_to_speech(self.sentences['robot_move_back'][counter])
      else:
        if counter >= len(self.sentences['user_move_back'])-1: counter = random.randint(0, len(self.sentences['user_move_back'])-1)
        self.speech.text_to_speech(self.sentences['user_move_back'][counter])
      rospy.sleep(0.1)
      self.face.reproduce_face_expression(facial_expression)
      b_executed = True
    elif self.face==None and self.gesture!=None:
      if who == "robot":
        if counter >= len(self.sentences['robot_move_back'])-1: counter = random.randint(0, len(self.sentences['robot_move_back'])-1)
        self.speech.text_to_speech(self.sentences['robot_move_back'][counter])
      else:
        if counter >= len(self.sentences['user_move_back'])-1: counter = random.randint(0, len(self.sentences['user_move_back'])-1)
        self.speech.text_to_speech(self.sentences['user_move_back'][counter])
      rospy.sleep(0.1)
      # TODO: validation on the robot
      success = self.gesture.pick_and_place(token_from, token_to)
      self.gesture.initial_pos()
      b_executed = True
    else:
      #TODO validation on the robot
      if who == "robot":
        if counter >= len(self.sentences['robot_move_back'])-1: counter = random.randint(0, len(self.sentences['robot_move_back'])-1)
        self.speech.text_to_speech(self.sentences['robot_move_back'][counter])
        success = self.gesture.pick_and_place(token_to, token_from)
        self.gesture.initial_pos()
      else:
        if counter >= len(self.sentences['user_move_back'])-1: counter = random.randint(0, len(self.sentences['user_move_back'])-1)
        self.speech.text_to_speech(self.sentences['user_move_back'][counter])
      rospy.sleep(0.1)
      self.face.reproduce_face_expression(facial_expression)
      #success = self.gesture.pick_and_place(token_to, token_from)
      #self.gesture.initial_pos()
      b_executed = True

    return b_executed
  '''
  LEVELS OF ASSISTANCE FURNISHED BY THE ROBOT
  '''


  def assistance(self, lev_id, row, counter, token, facial_expression,  tokens, delay_for_speech):

    if lev_id == 0:
      if counter >= len(self.sentences['lev_0'])-1: counter = random.randint(0, len(self.sentences['lev_0'])-1)
      self.assistance_level["lev_0"].__call__(counter, facial_expression, delay_for_speech)
    elif lev_id == 1:
      if counter >= len(self.sentences['lev_1'])-1: counter = random.randint(0, len(self.sentences['lev_1'])-1)
      self.assistance_level["lev_1"].__call__(counter, facial_expression, delay_for_speech)
    elif lev_id == 2:
      if counter >= len(self.sentences['lev_2'])-1: counter = random.randint(0, len(self.sentences['lev_2'])-1)
      self.assistance_level["lev_2"].__call__(row, counter, facial_expression, delay_for_speech)
    elif lev_id == 3:
      if counter >= len(self.sentences['lev_3'])-1: counter = random.randint(0, len(self.sentences['lev_3'])-1)
      self.assistance_level["lev_3"].__call__(token, counter, facial_expression, tokens, delay_for_speech)
    elif lev_id == 4:
      if counter >= len(self.sentences['lev_4'])-1: counter = random.randint(0, len(self.sentences['lev_4'])-1)
      self.assistance_level["lev_4"].__call__(token, counter, facial_expression, delay_for_speech)
    elif lev_id == 5:
      if counter >= len(self.sentences['lev_5'])-1: counter = random.randint(0, len(self.sentences['lev_5'])-1)
      self.assistance_level["lev_5"].__call__(token, facial_expression, delay_for_speech)
    else:
      assert "The selected level is not in the list"


  def no_assistance(self, counter, facial_expression, delay_for_speech):
    '''
    It provides no assistance to the user, basically only inform the user of moving a token
    Args:
      No arg
    Return:
      Bool: if the action has been executed successfully
    '''
    print("Lev_0")
    b_executed = False
    self.speech.text_to_speech(self.sentences['lev_0'][counter])
    rospy.sleep(0.1)
    self.face.reproduce_face_expression(facial_expression)
    b_executed = True
    return b_executed

  def encouragement(self, counter, facial_expression, delay_for_speech):
    '''
    It eoncurages the user to move a token
      Args:
        counter: to generate different sentences
      Return:
         Bool: if the action has been executed successfully
      '''
    print("Lev_1")
    b_executed = False
    self.speech.text_to_speech(self.sentences['lev_1'][counter]+";;;;"+self.sentences['lev_0'][0])
    rospy.sleep(0.1)
    b_executed = True
    return b_executed


  def suggest_row(self, row, counter, facial_expression, delay_for_speech=2):
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
      self.speech.text_to_speech(self.sentences['lev_2'][counter]+str(row)+";"+self.sentences['lev_0'][0])
      rospy.sleep(0.1)
      self.face.reproduce_face_expression(facial_expression)
      b_executed = True
    elif self.face==None and self.gesture!=None:
      #TODO: test on the robot and see if the voice can be sinc with the movement otherwise include speech and text in the gesture method
      self.gesture.suggest_row(row)
      self.speech.text_to_speech(self.sentences['lev_2'][counter] + str(row) + ";" + self.sentences['lev_0'][0])
      rospy.sleep(delay_for_speech)
      b_executed = True
    else:
      self.face.reproduce_face_expression(facial_expression)
      # TODO: test on the robot and see if the voice can be sinc with the movement otherwise include speech and text in the gesture method
      self.gesture.suggest_row(row)
      rospy.sleep(delay_for_speech)
      self.speech.text_to_speech(self.sentences['lev_2'][counter]+"; ;"+str(row)+";;"+self.sentences['lev_0'][0])
      b_executed = True
    return b_executed

  def suggest_cells(self, token, counter, facial_expression, tokens, delay_for_speech):
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
    tokens_id_to_str = " " .join([":"+str(t)+";" for t in tokens_id])
    if self.face!=None and self.gesture==None:
      # TODO: test on the robot and see if the voice can be sinc with the movement otherwise include speech and text in the gesture method
      self.speech.text_to_speech(self.sentences['lev_3'][counter]+tokens_id_to_str+ ";"+self.sentences['lev_0'][0])
      rospy.sleep(0.1)
      self.face.reproduce_face_expression(facial_expression)
      b_executed = True
    elif self.face==None and self.gesture!=None:
      # TODO: test on the robot and see if the voice can be sinc with the movement otherwise include speech and text in the gesture method
      self.gesture.suggest_subset(token_sol_from)
      rospy.sleep(delay_for_speech)
      self.speech.text_to_speech(self.sentences['lev_3'][counter]+tokens_id_to_str+ ";"+self.sentences['lev_0'][0])
      b_executed = True
    else:
      # TODO: test on the robot and see if the voice can be sinc with the movement otherwise include speech and text in the gesture method
      self.face.reproduce_face_expression(facial_expression)
      self.gesture.suggest_subset(token_sol_from)
      rospy.sleep(delay_for_speech)
      self.speech.text_to_speech(self.sentences['lev_3'][counter]+tokens_id_to_str+ ";"+self.sentences['lev_0'][0])
      b_executed = True
    return b_executed

  def suggest_solution(self, token, counter, facial_expression, delay_for_speech):
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
      self.speech.text_to_speech(self.sentences['lev_4'][counter] + tokens_id_to_str+"; "+self.sentences['lev_0'][0])
      rospy.sleep(0.1)
      self.face.reproduce_face_expression(facial_expression)
      b_executed = True
    elif self.face == None and self.gesture != None:
      # TODO: test on the robot and see if the voice can be sinc with the movement otherwise include speech and text in the gesture method
      self.gesture.suggest_solution(token_sol_from)
      rospy.sleep(delay_for_speech)
      self.speech.text_to_speech(self.sentences['lev_4'][counter]+";;;"+tokens_id_to_str+";;;;"+self.sentences['lev_0'][0])
      b_executed = True
    else:
      # TODO: test on the robot and see if the voice can be sinc with the movement otherwise include speech and text in the gesture method
      self.face.reproduce_face_expression(facial_expression)
      self.gesture.suggest_solution(token_sol_from)
      rospy.sleep(delay_for_speech)
      self.speech.text_to_speech(self.sentences['lev_4'][counter]+";;;"+tokens_id_to_str+";;;"+self.sentences['lev_0'][0])
      b_executed = True
    return b_executed

  def offer_solution(self, token, facial_expression, delay_for_speech):
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
      self.speech.text_to_speech(self.sentences['lev_5'][0])
      rospy.sleep(delay_for_speech)
      self.face.reproduce_face_expression(facial_expression)
      self.speech.text_to_speech(self.sentences['lev_5'][1])
      self.speech.text_to_speech(self.sentences['lev_5'][2])
      b_executed = True
    elif self.face == None and self.gesture != None:
      # TODO: test on the robot and see if the voice can be sinc with the movement otherwise include speech and text in the gesture method
      self.speech.text_to_speech(self.sentences['lev_5'][0])
      self.gesture.offer_token(token_sol_from, self.reproduce_sentence, self.sentences['lev_5'][1], self.sentences['lev_5'][2])
      b_executed = True
    else:
      # TODO: test on the robot and see if the voice can be sinc with the movement otherwise include speech and text in the gesture method
      self.speech.text_to_speech(self.sentences['lev_5'][0])
      self.face.reproduce_face_expression(facial_expression)
      self.gesture.offer_token(token_sol_from, self.reproduce_sentence, self.sentences['lev_5'][1], self.sentences['lev_5'][2])
      self.gesture.initial_pos()
      b_executed = True
    return b_executed

  def move_onbehalf(self, token, counter, facial_expression):
    '''
      It moves the token on the behalf of the user
        Args:
          token: The correct token, id and location
        Return:
           Bool: if the action has been executed successfully
        '''
    print("move_onbehalf")
    if counter >= len(self.sentences['max_attempt'])-1: counter = random.randint(0, len(self.sentences['max_attempt'])-1)
    token_sol_id, token_sol_from, token_sol_to = token
    token_id_to_str = " . ".join([str(t) for t in token_sol_id])
    b_executed = False
    print("Counter:",counter, len(self.sentences['max_attempt'])-1)
    if self.face!=None and self.gesture==None:
      self.speech.text_to_speech(self.sentences['max_attempt'][counter])
      rospy.sleep(0.1)
      self.face.reproduce_face_expression(facial_expression)
      b_executed = True
    elif self.face==None and self.gesture!=None:
      # TODO: test on the robot
      self.speech.text_to_speech(self.sentences['max_attempt'][counter])
      rospy.sleep(0.1)
      self.gesture.pick_and_place(token_sol_from, token_sol_to)
      self.gesture.initial_pos()
      b_executed = True
    else:
      # TODO: test on the robot
      self.speech.text_to_speech(self.sentences['max_attempt'][counter])
      rospy.sleep(0.1)
      self.face.reproduce_face_expression(facial_expression)
      self.gesture.pick_and_place(token_sol_from, token_sol_to)
      self.gesture.initial_pos()
      b_executed = True

  def pick(self, positive, counter, facial_expression):
    '''
    say something when a token is grasped depending on positive value
    Args:
      positive: while is a positive or a negative feedback
        counter: to generate different sentences
    Returns:
    '''
    print("pick")
    b_executed = False
    if positive:
      if counter >= len(self.sentences['pick_ok'])-1: counter = random.randint(0, len(self.sentences['pick_ok'])-1)
      self.speech.text_to_speech(self.sentences['pick_ok'][counter])
      rospy.sleep(0.2)
      self.face.reproduce_face_expression(facial_expression)
      b_executed = True
    else:
      if counter >= len(self.sentences['pick_no'])-1: counter = random.randint(0, len(self.sentences['pick_no'])-1)
      self.speech.text_to_speech(self.sentences['pick_no'][counter])
      rospy.sleep(0.2)
      self.face.reproduce_face_expression(facial_expression)
      b_executed = True
    return b_executed

  def get_action_state(self):
    if self.gesture!= None:
      return self.gesture.get_action_state()
    else:
      return None

  def timeout(self, counter, facial_expression):
    '''
      It tells the user that the time available for the current move ended
        Args:
          token: The correct token, id and location
        Return:
           Bool: if the action has been executed successfully
        '''
    b_executed = False
    if counter >= len(self.sentences['timeout'])-1: counter = random.randint(0, len(self.sentences['timeout'])-1)
    self.speech.text_to_speech(self.sentences['timeout'][counter], locked = True)
    self.face.reproduce_face_expression(facial_expression)
    b_executed = True
    return b_executed

  def instruction(self, counter, facial_expression):
    '''The agent provides the instructions of the exercise
    Args:
    Return:
      Bool:
      if the action has been executed successfully
    '''
    print("instruction")
    b_executed = False
    if counter >= len(self.sentences['instruction'])-1: counter = random.randint(0, len(self.sentences['instruction'])-1)
    if self.face!=None and self.gesture==None:
      self.speech.text_to_speech(self.sentences['instruction'][counter], locked=True)
      rospy.sleep(0.1)
      self.face.reproduce_face_expression(facial_expression)
      b_executed = True
    elif self.face==None and self.gesture!=None:
      assert "Error Instruction does not contemplate any gesture"
    else:
      assert "Error Instruction does not contemplate any gesture"

  def cancel_action(self):
    if self.gesture != None:
      self.gesture.cancel_motion()
      #self.gesture.initial_pos()
      #self.speech.cancel_reproduction()

  def reproduce_sentence(self, text):
    self.speech.text_to_speech(text)

  def reset_speech_ended(self):
    self.speech.reproduction_has_ended = False
    return self.speech.reproduction_has_ended

  def has_speech_ended(self):
    return self.speech.reproduction_has_ended
  #
  # def fake_function(self, reproduce_sentence, text):
  #   i=0
  #   while(i<1000):
  #     if i%10==0:
  #       reproduce_sentence(text)
  #     else:
  #       pass
  #     i += 1


def main():
  speech = Speech("en_GB")
  gesture = Gesture()
  face = Face()
  robot = Robot(speech=speech, sentences_file="config/sentences/sentences_en_GB",  face=face, gesture=gesture)

  counter = 0
  token_from = ""
  token_to = ""
  who = "robot"
  row = 3
  tokens = [("111",8), ("256", 9), ("341", 10)]
  token = ("111", 8, 2)
  positive = False
  lev_id = 3
  #robot.fake_function(robot.play_sentence)

  #robot.action["congrats"].__call__(counter)
  #robot.action["compassion"].__call__(counter)
  # #take in input 5 params
  #robot.action["move_back"].__call__(who, token)
  #robot.action["lev_0"].__call__()
  #rospy.sleep(3.0)
  #robot.assistance(lev_id=5, row=2, counter=0, token=token, facial_expression=face,  tokens=tokens, delay_for_speech=1)
  #robot.move_onbehalf(token, counter, face)
  robot.move_token_back(who=robot, token=token, counter=counter, facial_expression=face)
  #robot.action["assistance"].__call__(lev_id, row, counter, token, *tokens)
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