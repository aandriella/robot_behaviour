#!/usr/bin/python
# import sys
# print (sys.path)
import rospy
import time
from std_msgs.msg import String



class SpeechUtterance():
    def __init__(self):
        self.pub = rospy.Publisher('speech', String, queue_size=10)
    def reproduce_speech(self, sentence):
        rospy.init_node('big_hero', anonymous=True)
        rate = rospy.Rate(1) # 10hz
        while not rospy.is_shutdown():
            connections = self.pub.get_num_connections()
            if connections>0:
                rospy.loginfo(sentence)
                self.pub.publish(sentence)
                waiting_time = round(len(sentence))/12
                #print("sentence len", len(sentence), "waiting time ", str(waiting_time))
                #if the waiting time is smaller than 1 is reproducing a number
                if waiting_time<1:
                  waiting_time = 2

                time.sleep(waiting_time)
                break
            #rospy.sleep(pause)

# if __name__ == '__main__':
#     length = 5
#     progress = 1
#     timeout = 15
#     assistance_levels = 5
#     max_attempt = 4
#     assistance_probs = []
#     complexity_probs = []
#     total_tokens = 10
#
#     initial_board = {1: '0', 2: '0', 3: '0', 4: '0', 5: '0',
#                      6: '43', 7: '21', 8: '16', 9: '38', 10: '55'}
#
#     current_board = {1: '0', 2: '0', 3: '0', 4: '0', 5: '0',
#                      6: '43', 7: '21', 8: '16', 9: '38', 10: '55'}
#
#     board_size = (4, 5)
#
#     tokens_list = [43, 21, 16, 38, 55]
#
#     objective = "ascending"
#     skt = SKT.SKT(board_size, length, progress, timeout, assistance_levels, max_attempt,
#               assistance_probs, complexity_probs, total_tokens,
#               initial_board, current_board, tokens_list, objective)
#
#     print(skt.get_current_board_status())
#
#     tiago = Tiago.Tiago("", "", "", "", "", "")
#     skt.print_board()
#
#     for i in (tokens_list):
#         token_str = str(i)
#         solution_subset = tiago.get_assistance_lev2(1, token_str, skt)
#         solution = tiago.get_assistance_lev3(1, token_str, skt)
#         offer_token = tiago.get_assistance_lev4(1, token_str, skt)
#         move_back = tiago.move_token_back(token_str, skt)
#         print("i ", token_str, "subset ", solution_subset)
#         print("i ", token_str, "solution", solution)
#         print("i ", token_str, "offer token", offer_token)
#         print("i ", token_str, "move back ", move_back)
#
#     xml_reader = XMLReader.XMLReader()
#     file_name = '/home/aandriella/Documents/XMLReader/assistive_actions_definition.xml'
#     level = "lev2"
#     attempt = 1
#     to_say, gesture = xml_reader.get_action_assistance(file_name, level, (attempt))
#     print("to say: ", to_say, "is gesture ", str(gesture))
#
#
#
#     try:
#         speech_utterances(to_say)
#     except rospy.ROSInterruptException:
#         pass