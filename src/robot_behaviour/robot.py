import sys
import os
# path = os.path.abspath(__file__)
# dir_path = os.path.dirname(path)
# parent_dir_of_file = os.path.dirname(dir_path)
# parent_parent_dir_of_file = os.path.dirname(parent_dir_of_file)
# sys.path.append(parent_parent_dir_of_file+"/reinforcement_learning")
# sys.path.append(parent_parent_dir_of_file+"/PersonasSim")



#import from the package xml_reader
from xml_reader.xml_reader import XMLReader
from SKT import SKT
from robot_behaviour.speech_utterance import SpeechUtterance
from robot_behaviour.actions import Actions
import time
import numpy as np
import random as rn
class Robot:
  '''
  robot class to define the actions of assistance, combining speech and gesture
  '''

  def __init__(self, actions_file_name, xml_reader):
    '''
    :param assistive_actions_modalities:
    :param engaging_actions_modalities:
    :param congratulate_actions_modalities:
    :param assistive_actions_speech:
    :param engaging_actions_speech:
    :param congratulate_actions_speech:
    '''


    self.instructions_speech = xml_reader.get_actions_by_tag(actions_file_name, 'instruction')

    self.assistive_actions_speech = [xml_reader.get_actions_by_tag(actions_file_name, 'LEV_0'),
                                     xml_reader.get_actions_by_tag(actions_file_name, 'LEV_1'),
                                     xml_reader.get_actions_by_tag(actions_file_name, 'LEV_2'),
                                     xml_reader.get_actions_by_tag(actions_file_name, 'LEV_3'),
                                     xml_reader.get_actions_by_tag(actions_file_name, 'LEV_4')]

    # define the function for the action of engagement
    #self.engaging_actions_modalities

    self.congratulate_actions_speech = xml_reader.get_actions_by_tag(actions_file_name, 'congratulation')
    self.compassion_actions_speech = xml_reader.get_actions_by_tag(actions_file_name, 'compassion')
    self.move_back_actions_speech = xml_reader.get_actions_by_tag(actions_file_name,'move_back')
    self.timeout_action_speech = xml_reader.get_actions_by_tag(actions_file_name, "time_out")
    self.help_action_speech = xml_reader.get_actions_by_tag(actions_file_name, 'help')
    self.help_attempt = xml_reader.get_actions_by_tag(actions_file_name, "help_attempt")
    self.help_timeout = xml_reader.get_actions_by_tag(actions_file_name, "help_timeout")
    self.end_game = xml_reader.get_actions_by_tag(actions_file_name, "end_game")
    self.play_again = xml_reader.get_actions_by_tag(actions_file_name, "play_again")
    self.max_attempt = xml_reader.get_actions_by_tag(actions_file_name, "max_attempt")
    self.unexpected_beahviour = xml_reader.get_actions_by_tag(actions_file_name, "unexpected_behaviour")
    self.positive_grasping = xml_reader.get_actions_by_tag(actions_file_name, "positive_grasping")
    self.negative_grasping = xml_reader.get_actions_by_tag(actions_file_name, "negative_grasping")

  def get_instructions_speech(self):
    '''
    :return: the instructions speech
    '''
    return self.instructions_speech

  def get_congratulate_actions_speech(self):
    '''
    :return: congratulate instructions
    '''
    return self.congratulate_actions_speech

  def get_compassion_actions_speech(self):
    '''
    :return: compasstion instructions
    '''
    return self.compassion_actions_speech

  def get_move_back_actions_speech(self):
    '''
    :return: move back action
    '''
    return self.move_back_actions_speech

  def get_help_actions_speech(self):
    '''
    :return: help actions
    '''
    return self.help_action_speech

  def get_help_attempt_actions_speech(self):
    '''
    :return: help attempt actions
    '''
    return self.help_attempt

  def get_help_timeout_actions_speech(self):
    '''
    :return: help timeout actions
    '''
    return self.help_timeout

  def get_timeout_actions_speech(self):
    '''
    :return:
    '''
    return self.timeout_action_speech

  def get_play_again_speech(self):
    '''
    :return:
    '''
    return self.play_again

  def get_end_game_speech(self):
    '''
    :return:
    '''
    return self.end_game

  def get_max_attempt_speech(self):
    '''
    :return:
    '''
    return self.max_attempt

  def get_unexpected_beahviour(self):
    '''
    :return:
    '''
    return self.unexpected_beahviour

  def get_assistive_actions(self, level_index):
    '''
    :param level_index:
    :return:
    '''
    return self.assistive_actions_speech[level_index]


  def get_assistive_action_speech(self, level_index, attempt):
    '''
    :param level_index: its the index of the lev -> LEV 0:0, LEV 1: 1, and so on
    :param action_index: its the index of the action of that level
    :return: the string with the text to reproduce
    '''
    try:
      return self.assistive_actions_speech[level_index][attempt]
    except IndexError:
      print("The index is out of bound, we will set it to 0 for action index")



  def get_positive_grasping(self):
    try:
      return self.positive_grasping
    except IndexError:
      print("The index is out of bound, we will set it to 0 for action index")

  def get_negative_grasping(self):
    try:
      return self.negative_grasping
    except IndexError:
      print("The index is out of bound, we will set it to 0 for action index")

  def provide_lev_1(self, speech, file):
    speech.play_file(file)

  def provide_lev_2_col(self, token, skt, speech, robot, file_Left, file_Center, file_Right):
    board_cols = skt.get_board_size()[1]
    robot.look_at_user()

    # we need to get the location of the current token
    # and get the two that are closer
    solution_location = skt.get_token_location(token)
    # if the right token is is the last column of the board
    if (solution_location == (board_cols) or solution_location == ((2 * board_cols)) or solution_location == (
        3 * board_cols) or solution_location == (4 * board_cols)):
      speech.play_file(file_Right)
      print("Solution is on the right")
    elif (solution_location == 1 or solution_location == (board_cols + 1) or solution_location == (
            board_cols * 2) + 1 or solution_location == (board_cols * 3) + 1):
      speech.play_file(file_Left)
      print("Soluition in on the left")
    else:
      speech.play_file(file_Center)
      print("Solution is in the middle")

  def provide_lev_2_row(self, token, skt, speech, file_1Row, file_2Row, file_3Row):
    # we need to get the location of the current token
    # and get the two that are closer
    solution_location = skt.get_token_location(token)
    if solution_location>11 and solution_location<15:
      print("second row")
      speech.play_file(file_2Row)
    else:
      print("first row")
      speech.play_file(file_1Row)


  def move_token_back(self, token,  skt):
    #action to move back the token
    token_curr_loc = skt.get_token_location(token)
    token_dest_loc = skt.get_token_initial_location(token)
    return token_curr_loc, token_dest_loc


  def provide_instructions(self, speech, file):
    speech.play_file(file)


  def provide_congratulation(self, speech, file):
    speech.play_file(file)

  def provide_pos_feedback(self, attempt, max_attempt, speech, robot,  file):
    if attempt>max_attempt-1:
      attempt = np.random.randint(0, max_attempt)
    #robot.head_noddling_yes()
    speech.play_file(file[attempt])

  def provide_neg_feedback(self, attempt, max_attempt, speech, robot,  file):
    if attempt>max_attempt-1:
      attempt = np.random.randint(0, max_attempt)
    #robot.head_noddling_no()
    speech.play_file(file[attempt])

  def provide_token_back(self, speech, file):
    speech.play_file(file)

  def provide_game_completed(self, speech, file):
    speech.play_file(file)

  def provide_illegal_move(self, speech, file):
    speech.play_file(file)

# length=5
# progress=1
# timeout=15
# assistance_levels = 5
# max_attempt = 4
# assistance_probs = []
# complexity_probs = []
# total_tokens= 10
#
# actions = Actions()
#
#
# initial_board = {1:'0', 2:'0', 3:'0', 4:'0', 5:'0',
#         6:'0', 7:'0', 8:'0', 9:'0', 10:'0',
#         11:'43', 12:'21', 13:'16', 14:'38', 15:'55',
#         16:'0', 17:'0', 18:'0', 19:'0', 20:'0'
#         }
#
# current_board ={1:'0', 2:'0', 3:'0', 4:'0', 5:'0',
#         6:'0', 7:'0', 8:'0', 9:'0', 10:'0',
#         11:'43', 12:'21', 13:'16', 14:'38', 15:'55',
#         16:'0', 17:'0', 18:'0', 19:'0', 20:'0'
#         }
# solution_board = {1:'C', 2:'U', 3:'R', 4:'I', 5:'E',
#                 6:'0', 7:'0', 8:'0', 9:'0', 10:'0',
#                 11: 'A', 12: 'G', 13: '0', 14: 'B', 15: '0',
#                 16: '0', 17: 'D', 18: '0', 19: 'O', 20: '0'
#                 }
#
#
# current_board = initial_board.copy()
# tokens_list = ['A', 'G', 'U', 'B', 'E', 'C', 'D', 'I', 'O', 'R']
#
# objective = "ascending"
# board_size = (4,5)
#
# skt = SKT(board_size, length, progress, timeout, assistance_levels, max_attempt,
#          assistance_probs, complexity_probs, total_tokens,
#          initial_board, current_board, tokens_list, objective, solution_board)
#
# print(skt.get_current_board_status())
#
# xml = XMLReader()
#
# tiago = Robot("/home/pal/cognitive_game_ws/src/xml_reader/src/xml_reader/assistive_actions_definition.xml", xml)
# skt.print_board()
#
# speech = SpeechUtterance()
# actions = Actions()
#
# tiago.provide_instructions(speech,actions)
#tiago.provide_assistance(3, 1, '55', skt, speech, actions)

#for i in (tokens_list):
#  for k in range(max_attempt+2):
#      token_str = str(i)
      #tiago.provide_congratulation(k, speech, actions)
      #tiago.provide_compassion(k, speech, actions)
      #tiago.provide_assistance(0, k, token_str, skt, speech, actions)
      #tiago.provide_assistance(1, k, token_str, skt, speech, actions)
      #tiago.provide_assistance(2, k, token_str, skt, speech, actions)
#      tiago.provide_assistance(3, k, token_str, skt, speech, actions)
      #tiago.provide_assistance(4, k, token_str, skt, speech, actions)

