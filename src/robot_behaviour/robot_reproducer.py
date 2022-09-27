# coding=utf-8
'''
This is a module to reproduce a robot action combining speech,
facial expression and gesture. Every time we check if the action
 that by default is performed only with speech will include also face
 and gesture
'''
import rospy
import random
import ast
import pickle
import numpy as np
import sys, getopt
import pygame
import time
import thread


class Robot:
  def __init__(self, audio_file, action_policy_filename=None, face=None, gesture=None):
    '''
    :param speech: instance of class Speech
    :param sentences_file: the file where all the sentences are stored
    :param face:  instance of class Face
    :param gesture: instance of class Gesture
    '''
    self.audio_file = audio_file
    self.face = face
    self.gesture = gesture
    self.sentences = ""

    self.lev_0_sentences = "your_turn.wav"
    self.lev_1_sentences = ["Lev_1_1_.wav", "Lev_1_2_.wav", "Lev_1_3_.wav"]
    self.lev_2_sentences = ["Lev_2_left_.wav", "Lev_2_center_.wav", "Lev_2_right_.wav"]
    self.lev_3_sentences = ["D_.wav", "O_.wav", "M_.wav", "R_.wav", "E_.wav",
                            "G_.wav", "L_.wav", "S_.wav", "U_.wav", "I_.wav"]
    self.end_game_sentence = "end_game.wav"
    self.instruction_sentence = "instruction.wav"
    self.move_back_sentence = "move_back.wav"
    self.pick_pos_sentences = ["pick_correct_1.wav", "pick_correct_2.wav", "pick_correct_3.wav"]
    self.pick_neg_sentences = ["pick_wrong_1.wav", "pick_wrong_2.wav", "pick_wrong_3.wav"]
    self.place_pos_sentences = ["place_correct_1.wav", "place_correct_2.wav", "place_correct_3.wav"]
    self.place_neg_sentences = ["place_wrong_1.wav", "place_wrong_2.wav", "place_wrong_3.wav"]
    self.sound_wrong_pick = "beep.mp3"
    self.sound_correct_pick = "pair.wav"


    self.action = {
      "instruction": self.instruction,
      "congrats": self.congratulate,
      "compassion": self.compassion,
      "move_back": self.move_token_back,
      "pick": self.pick,
      "end_game" : self.end_game,
      "assistance": self.assistance
    }

    self.assistance_level = {
      "lev_0": self.no_assistance,
      "lev_1": self.encouragement,
      "lev_2": self.suggest_row,
      "lev_3": self.suggest_solution,
      # "lev_4": self.suggest_solution,
      # "lev_5": self.offer_solution
    }


  def play_sound(self, file_path, delay_sound):
    time.sleep(0.1)
    pygame.mixer.init()
    pygame.mixer.music.load(self.audio_file+"/"+file_path)
    pygame.mixer.music.play()
    time.sleep(delay_sound)

  def load_robot_policy(self, learned_policy_filename):
    with open(learned_policy_filename, "rb") as f:
      loaded_policy = pickle.load(f)
      return loaded_policy

  def get_irl_state_action(self, state_index, epsilon=0.1):
    action = 0
    print("Select it between the following:", self.action_policy[state_index])
    two_max_elem = [(self.action_policy[state_index].tolist().index(x)) for x in sorted(self.action_policy[state_index].tolist(), reverse=True)[:2]]
    if random.random() < epsilon:
      action = two_max_elem[1]
    else:
      action = two_max_elem[0]
    return action

  def get_random_state_action(self):
    return random.randint(0, 6)


  def congratulate(self, counter, delay_sound):
    '''
    It greats the user for being performed a correct move
    Args:
      counter: get one of the sentences in the list
    Return:
       Bool: if the action has been executed successfully
    '''
    print("Congrats method")
    b_executed = False
    selected = random.randint(0,counter-1)
    sentence = self.pick_pos_sentences[selected]
    self.play_sound(sentence, delay_sound)
    b_executed = True

    return b_executed

  def compassion(self, counter, delay_sound):
    '''
    offer compassion to the user for being performed a werong move
    Args:
      counter: in order to generate different sentences
    Return:
      Bool: if the action has been executed successfully
      '''
    print("Compassion funct")
    selected = random.randint(0, counter - 1)
    sentence = self.place_neg_sentences[selected]
    self.play_sound(sentence, delay_sound)
    b_executed = True
    '''N.B For this kind of action we did not plan to move the robot'''
    return b_executed


  def move_token_back(self, delay_sound):
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
    sentence = self.move_back_sentence
    self.play_sound(sentence, delay_sound)
    b_executed = True

    return b_executed


  '''
  LEVELS OF ASSISTANCE FURNISHED BY THE ROBOT
  '''

  def assistance(self, lev_id, solution, location, counter,  delay_for_speech):

    if lev_id == 0:
      sentences = self.lev_0_sentences
      self.assistance_level["lev_0"].__call__(sentences,  delay_for_speech)
    elif lev_id == 1:
      sentences = self.lev_1_sentences
      self.assistance_level["lev_1"].__call__(sentences, counter, delay_for_speech)
    elif lev_id == 2:
      sentences = self.lev_2_sentences
      self.assistance_level["lev_2"].__call__(sentences, location, delay_for_speech)
    elif lev_id == 3:
      sentences = self.lev_3_sentences
      self.assistance_level["lev_3"].__call__(solution, delay_for_speech)
    else:
      assert "The selected level is not in the list"


  def no_assistance(self, sentence,  delay_sound):
    '''
    It provides no assistance to the user, basically only inform the user of moving a token
    Args:
      No arg
    Return:
      Bool: if the action has been executed successfully
    '''
    print("Lev_0")
    b_executed = False
    self.play_sound(sentence, delay_sound)

    b_executed = True
    return b_executed

  def encouragement(self, sentence_array, counter, delay_sound):
    '''
    It eoncurages the user to move a token
      Args:
        counter: to generate different sentences
      Return:
         Bool: if the action has been executed successfully
      '''
    print("Lev_1")
    b_executed = False
    print(counter)
    selected = random.randint(0, counter - 1)
    sentence = sentence_array[selected]
    self.play_sound(sentence, delay_sound)
    b_executed = True
    return b_executed


  def suggest_row(self, sentence_array, location, delay_sound):
    '''
    It tells the user the row of the board where the correct token is
      Args:
        row: the row of the board where the robot has to point
        location: 0 left 1 center 2 right
      Return:
         Bool: if the action has been executed successfully
      '''
    print("Lev_2")
    b_executed = False
    sentence =  sentence_array[location]
    self.play_sound(sentence, delay_sound)
    b_executed = True

    return b_executed

  # def suggest_cells(self, token, counter, facial_expression, tokens, delay_for_speech):
  #   '''
  #   It tells the user the cells near to the correct token (included the latter)
  #     Args:
  #       counter: to generate different sentences
  #       token: the correct token
  #       tokens: the subset of tokens on the board where the robot has to point
  #     Return:
  #        Bool: if the action has been executed successfully
  #     '''
  #   print("Lev_3")
  #   b_executed = False
  #   #used by gesture
  #   token_sol_id, token_sol_from, _ = token
  #   #used by speech
  #   tokens_id = [t[:][0] for t in tokens]
  #   tokens_loc = [t[:][1] for t in tokens]
  #   tokens_id_to_str = " " .join([":"+str(t)+";" for t in tokens_id])
  #   if self.face!=None and self.gesture==None:
  #     # TODO: test on the robot and see if the voice can be sinc with the movement otherwise include speech and text in the gesture method
  #     self.speech.text_to_speech(self.sentences['lev_3'][counter]+tokens_id_to_str+ ";"+self.sentences['lev_0'][0])
  #     rospy.sleep(0.1)
  #     self.face.reproduce_face_expression(facial_expression)
  #     b_executed = True
  #   elif self.face==None and self.gesture!=None:
  #     # TODO: test on the robot and see if the voice can be sinc with the movement otherwise include speech and text in the gesture method
  #     self.gesture.suggest_subset(token_sol_from)
  #     rospy.sleep(delay_for_speech)
  #     self.speech.text_to_speech(self.sentences['lev_3'][counter]+tokens_id_to_str+ ";"+self.sentences['lev_0'][0])
  #     b_executed = True
  #   else:
  #     # TODO: test on the robot and see if the voice can be sinc with the movement otherwise include speech and text in the gesture method
  #     self.face.reproduce_face_expression(facial_expression)
  #     self.gesture.suggest_subset(token_sol_from)
  #     rospy.sleep(delay_for_speech)
  #     self.speech.text_to_speech(self.sentences['lev_3'][counter]+tokens_id_to_str+ ";"+self.sentences['lev_0'][0])
  #     b_executed = True
  #   return b_executed
  #
  def suggest_solution(self, solution, delay_for_speech):
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
    token_sol_id, token_sol_from, _ = solution
    #need for speech
    tokens_id_to_str =  str(token_sol_id)
    if tokens_id_to_str == "D":
      self.play_sound(self.lev_3_sentences[0], delay_for_speech)
    elif tokens_id_to_str == "O":
      self.play_sound(self.lev_3_sentences[1], delay_for_speech)
    elif tokens_id_to_str == "M":
      self.play_sound(self.lev_3_sentences[2], delay_for_speech)
    elif tokens_id_to_str == "R":
      self.play_sound(self.lev_3_sentences[3], delay_for_speech)
    elif tokens_id_to_str == "E":
      self.play_sound(self.lev_3_sentences[4], delay_for_speech)
    elif tokens_id_to_str == "G":
      self.play_sound(self.lev_3_sentences[5], delay_for_speech)
    elif tokens_id_to_str == "L":
      self.play_sound(self.lev_3_sentences[6], delay_for_speech)
    elif tokens_id_to_str == "S":
      self.play_sound(self.lev_3_sentences[7], delay_for_speech)
    elif tokens_id_to_str == "U":
      self.play_sound(self.lev_3_sentences[8], delay_for_speech)
    elif tokens_id_to_str == "I":
      self.play_sound(self.lev_3_sentences[9], delay_for_speech)
    else:
      print("Letter not recognised")
    b_executed = True
    return b_executed
  #
  # def offer_solution(self, token, facial_expression, delay_for_speech):
  #   '''
  #   It offers the user the correct token
  #     Args:
  #       token: The correct token, id and location
  #     Return:
  #        Bool: if the action has been executed successfully
  #     '''
  #   print("Lev_5")
  #   b_executed = False
  #   #need for gesture
  #   token_sol_id, token_sol_from, _ = token
  #   #need for speech?
  #   token_id_to_str = token_sol_id
  #   if self.face != None and self.gesture == None:
  #     assert "This level has not been designed to be provided without the robot"
  #     self.speech.text_to_speech(self.sentences['lev_5'][0])
  #     rospy.sleep(delay_for_speech)
  #     self.speech.text_to_speech("..."+token_id_to_str)
  #     self.face.reproduce_face_expression(facial_expression)
  #     self.speech.text_to_speech(self.sentences['lev_5'][1])
  #     b_executed = True
  #   elif self.face == None and self.gesture != None:
  #     self.speech.text_to_speech(self.sentences['lev_5'][0])
  #     self.gesture.offer_token(token_sol_from, self.reproduce_sentence, self.sentences['lev_5'][1])
  #     b_executed = True
  #   else:
  #     self.speech.text_to_speech(self.sentences['lev_5'][0])
  #     self.face.reproduce_face_expression(facial_expression)
  #     self.gesture.offer_token(token_sol_from, self.reproduce_sentence, self.sentences['lev_5'][1])
  #     self.gesture.initial_pos()
  #     b_executed = True
  #   return b_executed
  #
  # def move_onbehalf(self, token, counter, facial_expression, eyes_coords=None):
  #   '''
  #     It moves the token on the behalf of the user
  #       Args:
  #         token: The correct token, id and location
  #       Return:
  #          Bool: if the action has been executed successfully
  #       '''
  #   print("move_onbehalf")
  #   if counter >= len(self.sentences['max_attempt'])-1: counter = random.randint(0, len(self.sentences['max_attempt'])-1)
  #   token_sol_id, token_sol_from, token_sol_to = token
  #   token_id_to_str = " . ".join([str(t) for t in token_sol_id])
  #   b_executed = False
  #   print("Counter:",counter, len(self.sentences['max_attempt'])-1)
  #   if self.face!=None and self.gesture==None:
  #     self.speech.text_to_speech(self.sentences['max_attempt'][counter])
  #     rospy.sleep(0.1)
  #     self.face.reproduce_face_expression(facial_expression)
  #     if eyes_coords != None:
  #       self.face.move_eyes(eyes_coords[0], eyes_coords[1])
  #
  #     b_executed = True
  #   elif self.face==None and self.gesture!=None:
  #     # TODO: test on the robot
  #     self.speech.text_to_speech(self.sentences['max_attempt'][counter])
  #     rospy.sleep(0.1)
  #     self.gesture.pick_and_place(token_sol_from, token_sol_to)
  #     self.gesture.initial_pos()
  #     b_executed = True
  #   else:
  #     # TODO: test on the robot
  #     self.speech.text_to_speech(self.sentences['max_attempt'][counter])
  #     rospy.sleep(0.1)
  #     self.face.reproduce_face_expression(facial_expression)
  #     if eyes_coords != None:
  #       self.face.move_eyes(eyes_coords[0], eyes_coords[1])
  #
  #     self.gesture.pick_and_place(token_sol_from, token_sol_to)
  #     rospy.sleep(0.1)
  #     self.gesture.initial_pos()
  #     b_executed = True

  def pick(self, positive, counter, delay_sound):
    '''
    say something when a token is grasped depending on positive value
    Args:
      positive: while is a positive or a negative feedback
        counter: to generate different sentences
    Returns:
    '''
    print("pick")
    b_executed = False
    if positive == True:
      selected = random.randint(0, counter - 1)
      sentence = self.pick_pos_sentences[selected]
      self.play_sound(sentence, delay_sound)
    else:
      selected = random.randint(0, counter - 1)
      sentence = self.pick_neg_sentences[selected]
      self.play_sound(sentence, delay_sound)

    b_executed = True

    return b_executed

  def get_action_state(self):
    if self.gesture!= None:
      return self.gesture.get_action_state()
    else:
      return None

  # def timeout(self, counter, facial_expression, eyes_coords=None):
  #   '''
  #     It tells the user that the time available for the current move ended
  #       Args:
  #         token: The correct token, id and location
  #       Return:
  #          Bool: if the action has been executed successfully
  #       '''
  #   b_executed = False
  #   if counter >= len(self.sentences['timeout'])-1: counter = random.randint(0, len(self.sentences['timeout'])-1)
  #   self.speech.text_to_speech(self.sentences['timeout'][counter], locked = True)
  #   self.face.reproduce_face_expression(facial_expression)
  #   if eyes_coords != None:
  #     self.face.move_eyes(eyes_coords[0], eyes_coords[1])
  #   b_executed = True
  #   return b_executed

  def instruction(self, sentence_array, delay_sound):
    '''The agent provides the instructions of the exercise
    Args:
    Return:
      Bool:
      if the action has been executed successfully
    '''
    print("instruction")
    b_executed = False
    sentence_1 = sentence_array
    self.play_sound(sentence_1, delay_sound)


    b_executed = True
    return b_executed

  def end_game(self, delay_sound):
    '''The agent provides the instructions of the exercise
    Args:
    Return:
      Bool:
      if the action has been executed successfully
    '''
    print("end_game")
    b_executed = False
    sentence = self.end_game_sentence
    self.play_sound(sentence, delay_sound)
    b_executed = True
    return b_executed

  def stop_sound(self):
    time.sleep(0.5)
    self.reproduction_has_ended = True
    pygame.mixer.music.stop()
    return self.reproduction_has_ended

  def reproduce_sentence(self, text, locked=False):
    self.text_to_speech(text, locked=locked)

  def reset_speech_ended(self):
    self.reproduction_has_ended = False
    return self.reproduction_has_ended

  def has_speech_ended(self):
    return self.reproduction_has_ended
  #
  # def fake_function(self, reproduce_sentence, text):
  #   i=0
  #   while(i<1):
  #     if i%10==0:
  #       reproduce_sentence(text)
  #     else:
  #       pass
  #     i += 1


def main():


  print("main --- robot behaviour")

  policy_filename = "/home/pal/carf_ws/src/carf/robot-patient-interaction/1/True/1/policy.pkl"
  audio = "/home/pal/carf_ws/src/robot_behaviour/src/robot_behaviour/config/audio/female/praise/cat/"
  robot = Robot(audio_file=audio, action_policy_filename=policy_filename)
  robot.assistance(lev_id=1, solution= "D", location=1, counter=3, delay_for_speech=10)
  #
  # start = time.time()
  # elapsed_time = 0
  # max_time = 10
  # try:
  #   time.sleep(1)
  #   thread.start_new_thread(robot.instruction, (sentences, 20,))
  #   # thread.start_new_thread(robot.instruction, (sentences, 10, ))
  #
  # except:
  #   print("ERROR unable to start the thread")
  # while (elapsed_time<max_time):
  #   elapsed_time = time.time()-start
  #   print(elapsed_time)
  # robot.stop_sound()




if __name__=="__main__":
  pass
  #main()
