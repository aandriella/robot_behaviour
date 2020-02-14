

import time
import numpy as np


# import from the package xml_reader
from xml_reader.xml_reader import XMLReader
from SKT import SKT
from robot_behaviour.speech import Speech
from robot_behaviour.gesture import Gesture


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
    # self.engaging_actions_modalities


    self.congratulate_actions_speech = xml_reader.get_actions_by_tag(actions_file_name, 'congratulation')
    self.compassion_actions_speech = xml_reader.get_actions_by_tag(actions_file_name, 'compassion')
    self.move_back_actions_speech = xml_reader.get_actions_by_tag(actions_file_name, 'move_back')
    self.correct_token_speech = xml_reader.get_actions_by_tag(actions_file_name, 'correct_token')
    self.timeout_action_speech = xml_reader.get_actions_by_tag(actions_file_name, "time_out")
    self.help_action_speech = xml_reader.get_actions_by_tag(actions_file_name, 'help')
    self.help_attempt = xml_reader.get_actions_by_tag(actions_file_name, "help_attempt")
    self.help_timeout = xml_reader.get_actions_by_tag(actions_file_name, "help_timeout")
    self.end_game = xml_reader.get_actions_by_tag(actions_file_name, "end_game")
    self.play_again = xml_reader.get_actions_by_tag(actions_file_name, "play_again")
    self.max_attempt = xml_reader.get_actions_by_tag(actions_file_name, "max_attempt")
    self.unexpected_beahviour = xml_reader.get_actions_by_tag(actions_file_name, "unexpected_behaviour")


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

  def get_correct_token_speech(self):
    '''
    :return:
    '''
    return self.correct_token_speech
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

  def get_token_subset_solution(self, token, skt):
    board_cols = skt.get_board_size()[1]
    solution_subset = []
    # we need to get the location of the current token
    # and get the two that are closer
    solution_location = skt.get_token_location(token)
    # if the right token is is the last column of the board
    if (solution_location == (board_cols) or solution_location == ((2 * board_cols)) or solution_location == (
        3 * board_cols) or solution_location == (4 * board_cols)):
      left_left_closer_token = skt.get_current_board_status()[solution_location - 2]
      if left_left_closer_token != '0': solution_subset.append(left_left_closer_token)
      left_closer_token = skt.get_current_board_status()[solution_location - 1]
      if left_closer_token != '0': solution_subset.append(left_closer_token)
      solution_subset.append(token)

    # if theright token is in the first column of the board
    elif (solution_location == 1 or solution_location == (board_cols + 1) or solution_location == (
        board_cols * 2) + 1 or solution_location == (board_cols * 3) + 1):
      solution_subset.append(token)
      right_closer_token = skt.get_current_board_status()[solution_location + 1]
      if right_closer_token != '0': solution_subset.append(right_closer_token)
      right_right_closer_token = skt.get_current_board_status()[solution_location + 2]
      if right_right_closer_token != '0': solution_subset.append(right_right_closer_token)
    else:
      left_closer_token = skt.get_current_board_status()[solution_location +1]
      if left_closer_token != '0': solution_subset.append(left_closer_token)
      solution_subset.append(token)
      right_closer_token = skt.get_current_board_status()[solution_location - 1]
      if right_closer_token != '0': solution_subset.append(right_closer_token)
    return solution_subset

  def move_token_back(self, token, skt):
    # action to move back the token
    token_curr_loc = skt.get_token_location(token)
    token_dest_loc = skt.get_token_initial_location(token)
    return token_curr_loc, token_dest_loc

  def provide_reengagement_timeout(self, speech):
    for i in range(len(self.get_timeout_actions_speech())):
      # check if we need to reproduce a gesture
      if self.get_timeout_actions_speech()[i][1] == 1:
        speech.text_to_speech(self.get_timeout_actions_speech()[i][0])
        # reproduce the gesture
      else:
        speech.text_to_speech(self.get_timeout_actions_speech()[i][0])

  def provide_instructions(self, speech, actions):

    speech.text_to_speech(self.get_instructions_speech()[0][0])
      # reproduce the gesture
    # if at the first one of the instructions are with gesture then reproduce it
    if self.get_instructions_speech()[0][1] == 1:
      actions.initial_pos()

  def provide_assistance(self, level_index, attempt, token, skt, speech, actions):
    token_id, token_loc = token
    if level_index == 0:
      if attempt > len(self.get_assistive_actions(level_index)) - 1:
        attempt = np.random.randint(0, len(self.get_assistive_actions(level_index)))
      speech.text_to_speech(self.get_assistive_action_speech(level_index, attempt)[0])

    elif level_index == 1:
      if attempt > len(self.get_assistive_actions(level_index)) - 1:
        attempt = np.random.randint(0, len(self.get_assistive_actions(level_index)))
      # if in the xml file name!= "no_gesture"
      if self.get_assistive_action_speech(level_index, attempt)[1] == 1:
        speech.text_to_speech(self.get_assistive_action_speech(level_index, attempt)[0])
        # hard coded string
        speech.text_to_speech(self.get_assistive_action_speech(0, 0)[0])
      else:
        if attempt > len(self.get_assistive_actions(level_index)) - 1:
          attempt = np.random.randint(0, len(self.get_assistive_actions(level_index)))
        speech.text_to_speech(self.get_assistive_action_speech(level_index, attempt)[0])
        # hard coded string
        speech.text_to_speech(self.get_assistive_action_speech(0, 0)[0])

    elif level_index == 2:
      if attempt > len(self.get_assistive_actions(level_index)) - 1:
        attempt = np.random.randint(0, len(self.get_assistive_actions(level_index)))
      # robot action
      if self.get_assistive_action_speech(level_index, attempt)[1] == 1:
        speech.text_to_speech(self.get_assistive_action_speech(level_index, attempt)[0])
        # reproduce the gesture
        print("token ", token_id, " location ", token_loc)
        subset_solution = self.get_token_subset_solution(token_id, skt)
        actions.suggest_subset(token_loc, speech, subset_solution, 5)
        # hard coded string
        speech.text_to_speech(self.get_assistive_action_speech(0, 0)[0])
        actions.initial_pos()
      else:
        if attempt > len(self.get_assistive_actions(level_index)) - 1:
          attempt = np.random.randint(0, len(self.get_assistive_actions(level_index)))
        speech.text_to_speech(self.get_assistive_action_speech(level_index, attempt)[0])
        subset_solution = self.get_token_subset_solution(token_id, skt)
        for i in range(len(subset_solution)):
          speech.text_to_speech(subset_solution[i])
        # hard coded string
        speech.text_to_speech(self.get_assistive_action_speech(0, 0)[0])

    elif level_index == 3:
      if attempt > len(self.get_assistive_actions(level_index)) - 1:
        attempt = np.random.randint(0, len(self.get_assistive_actions(level_index)))
      # robot action
      if self.get_assistive_action_speech(level_index, attempt)[1] == 1:
        speech.text_to_speech(self.get_assistive_action_speech(level_index, attempt)[0])
        # reproduce the gesture
        actions.suggest_solution(token_id, token_loc, speech, 5)
        # hard coded string
        speech.text_to_speech(self.get_assistive_action_speech(0, 0)[0])
        actions.initial_pos()
      else:
        if attempt > len(self.get_assistive_actions(level_index)) - 1:
          attempt = np.random.randint(0, len(self.get_assistive_actions(level_index)))
        speech.text_to_speech(self.get_assistive_action_speech(level_index, attempt)[0])
        speech.text_to_speech(token_id)
        # hard coded string
        speech.text_to_speech(self.get_assistive_action_speech(0, 0)[0])

    elif level_index == 4:
      print("Warning hard coded string")
      # reproduce the gesture
      token_loc = skt.get_token_location(token)
      speech.text_to_speech(self.get_assistive_action_speech(level_index, 0)[0])
      if self.get_assistive_action_speech(level_index, 0)[1] == 1:
        actions.offer_token(token_loc, speech, self.get_assistive_action_speech(level_index, 1)[0])
        # speech.text_to_speech(self.get_assistive_action_speech(0, 0)[0])
        actions.initial_pos()
      # hard coded string
      speech.text_to_speech(self.get_assistive_action_speech(0, 0)[0])
      # speech.text_to_speech(token)
      # reproduce the gesture
      # speech.text_to_speech(self.get_assistive_action_speech(level_index, 1)[0])
    else:
      assert Exception("error in the index level, please revise it")

  def provide_congratulation(self, attempt, speech, actions):
    if attempt > len(self.get_congratulate_actions_speech()) - 1:
      attempt = np.random.randint(0, len(self.get_congratulate_actions_speech()))
    if self.get_congratulate_actions_speech()[attempt][1] == 1:
      # perform robot action
      actions.head_noddling_yes()
      speech.text_to_speech(self.get_congratulate_actions_speech()[attempt][0])
    else:
      speech.text_to_speech(self.get_congratulate_actions_speech()[attempt][0])

  def provide_compassion(self, attempt, speech, actions):
    if attempt > len(self.get_compassion_actions_speech()) - 1:
      attempt = np.random.randint(0, len(self.get_compassion_actions_speech()))

    if self.get_compassion_actions_speech()[attempt][1] == 1:
      # perform robot action
      actions.head_noddling_no()
      speech.text_to_speech(self.get_compassion_actions_speech()[attempt][0])
    else:
      speech.text_to_speech(self.get_compassion_actions_speech()[attempt][0])

  def provide_help(self, attempt, speech):
    if attempt > len(self.get_help_actions_speech()) - 1:
      attempt = np.random.randint(0, len(self.get_help_actions_speech()))

    if self.get_help_actions_speech()[attempt][1] == 1:
      speech.text_to_speech(self.get_help_actions_speech()[attempt][0])
      # perform robot action (need to implement this movement)
    else:
      speech.text_to_speech(self.get_help_actions_speech()[attempt][0])

  def provide_help_attempt(self, attempt, speech):
    if attempt > len(self.get_help_attempt_actions_speech()) - 1:
      attempt = np.random.randint(0, len(self.get_help_attempt_actions_speech()))

    if self.get_help_attempt_actions_speech()[attempt][1] == 1:
      speech.text_to_speech(self.get_help_attempt_actions_speech()[attempt][0])
      # perform robot action
    else:
      speech.text_to_speech(self.get_help_attempt_actions_speech()[attempt][0])

  def provide_help_timeout(self, attempt, speech):
    if attempt > len(self.get_help_timeout_actions_speech()) - 1:
      attempt = np.random.randint(0, len(self.get_help_timeout_actions_speech()))

    if self.get_help_timeout_actions_speech()[attempt][1] == 1:
      speech.text_to_speech(self.get_help_timeout_actions_speech()[attempt][0])
      # perform robot action
    else:
      speech.text_to_speech(self.get_help_timeout_actions_speech()[attempt][0])

  def provide_token_back(self, attempt, speech, actions, _from, _to):
    if attempt >= len(self.get_move_back_actions_speech()) - 1:
      attempt = np.random.randint(0, len(self.get_move_back_actions_speech()))

    if self.get_move_back_actions_speech()[attempt][1] == 1:
      # perform robot action
      speech.text_to_speech(self.get_move_back_actions_speech()[attempt][0])
      actions.pick_and_place(_from, _to)
    else:
      speech.text_to_speech(self.get_move_back_actions_speech()[attempt][0])

  def provide_correct_token(self, attempt, speech, actions, _from, _to):
    if attempt >= len(self.get_correct_token_speech()) - 1:
      attempt = np.random.randint(0, len(self.get_correct_token_speech()))

    if self.get_correct_token_speech()[attempt][1] == 1:
      # perform robot action
      speech.text_to_speech(self.get_correct_token_speech()[attempt][0])
      actions.pick_and_place(_from, _to)
    else:
      speech.text_to_speech(self.get_correct_token_speech()[attempt][0])

  def provide_game_completed(self, speech, actions):
    for i in range(len(self.get_end_game_speech())):
      # check if we need to reproduce a gesture
      if self.get_end_game_speech()[i][1] == 1:
        speech.text_to_speech(self.get_end_game_speech()[i][0])
        # reproduce the gesture
        actions.wave()
      else:
        speech.text_to_speech(self.get_end_game_speech()[i][0])

  def provide_play_again(self, speech):
    speech.text_to_speech(self.get_play_again_speech()[0][0])

  def provide_max_attempt(self, speech, actions, _from, _to):
    speech.text_to_speech(self.get_max_attempt_speech()[0][0])
    if self.get_max_attempt_speech()[0][1] == 1:
      actions.pick_and_place(_from, _to)

  def provide_unexpected_behaviour(self, speech, actions):
    for i in range(len(self.get_unexpected_beahviour())):
      if self.get_unexpected_beahviour()[i][1] == 1:
        speech.text_to_speech(self.get_unexpected_beahviour()[i][0])
        # reproduce action
        actions.head_noddling_no()
      else:
        speech.text_to_speech(self.get_unexpected_beahviour()[i][0])

  def provide_any_instruction(self, speech, sentence):
    speech.text_to_speech(sentence)

  def provide_illegal_move(self, speech, token, location):
    speech.text_to_speech("Move token " + token + ". in location " + location)
    time.sleep(2)

# length=5
# progress=1
# timeout=15
# assistance_levels = 5
# max_attempt = 4
# assistance_probs = []
# complexity_probs = []
# total_tokens= 10
#
# actions = Gesture()
#
#
# initial_board = {1:'0', 2:'0', 3:'0', 4:'0', 5:'0',
#         6:'0', 7:'0', 8:'0', 9:'0', 10:'0',
#         11:'A', 12:'G', 13:'U', 14:'B', 15:'E',
#         16:'C', 17:'D', 18:'I', 19:'O', 20:'R'
#         }
#
# current_board ={1:'0', 2:'0', 3:'0', 4:'0', 5:'0',
#         6:'0', 7:'0', 8:'0', 9:'0', 10:'0',
#         11:'A', 12:'G', 13:'U', 14:'B', 15:'E',
#         16:'C', 17:'D', 18:'I', 19:'O', 20:'R'
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
# skt = SKT(board_size, 5, progress, timeout, assistance_probs, 0, 0,
#             max_attempt,
#             assistance_probs, complexity_probs, total_tokens,
#             initial_board, current_board, tokens_list, objective, solution_board)
#
#
# print(skt.get_current_board_status())
#
# xml = XMLReader()
#
# tiago = Robot("/home/pal/cognitive_game_ws/src/robot_behaviour/src/robot_behaviour/config/assistive_actions_definition.xml", xml)
# skt.print_board()
# speech = Speech("en_GB")
# actions = Gesture()
#
# #tiago.provide_instructions(speech, actions)
# #tiago.provide_assistance(3, 1, '55', skt, speech, actions)
#
# for i in (tokens_list):
#  for k in range(max_attempt+2):
#   token = skt.get_expected_token()
#   print(token, " ")
  # tiago.provide_congratulation(k, speech, actions)
  # tiago.provide_compassion(k, speech, actions)
  # tiago.provide_assistance(0, k, token, skt, speech, actions)
  #tiago.provide_assistance(1, k, token, skt, speech, actions)
  #tiago.provide_assistance(2, k, token, skt, speech, actions)
  #tiago.provide_assistance(3, k, token, skt, speech, actions)
  # tiago.provide_assistance(4, k, token, skt, speech, actions)
  #skt.update_board(token[0], token[1])


