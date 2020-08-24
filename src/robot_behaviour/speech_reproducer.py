#import ROS headers
import rospy
import actionlib
import time
from actionlib_msgs.msg import GoalStatus
#import PAL Robotics custom headers
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from sound_play.libsoundplay import SoundClient
from actionlib_msgs.msg import GoalStatus

class Speech():
  def __init__(self, language):
    rospy.init_node('big_hero', anonymous=True)
    self.client = actionlib.SimpleActionClient('/tts_to_soundplay', TtsAction)
    self.sound_client = SoundClient()
    self.language = language
    self.feedback = 0
    self.text = ""

  def event(self, event):
    #print("event {}".format(event))
    return {
      1: 'TTS_EVENT_INITIALIZATION',
      2: 'TTS_EVENT_SHUTDOWN',
      4: 'TTS_EVENT_SYNCHRONIZATION',
      8: 'TTS_EVENT_FINISHED_PLAYING_UTTERANCE',
      16: 'TTS_EVENT_MARK',
      32: 'TTS_EVENT_STARTED_PLAYING_WORD',
      64: 'TTS_EVENT_FINISHED_PLAYING_PHRASE',
      128: 'TTS_EVENT_FINISHED_PLAYING_SENTENCE',
      256: 'PLAYING'
    }[event]

  def feedbackCb(self, feedback):
    # print("event type: " + self.event(feedback.event_type))
    # print("timestamp: " + str(feedback.timestamp))
    # print("current word: " + feedback.text_said)
    # print("next word: " + feedback.next_word)
    # print("-")
    self.feedback = self.event(feedback.event_type)
    return self.feedback

  def text_to_speech(self, text, locked=False, threshold=10):
    #rospy.loginfo("Waiting for Server")
    self.client.wait_for_server()
    #rospy.loginfo("Reached Server")
    goal = TtsGoal()
    goal.rawtext.text = text
    length = len(text)
    goal.rawtext.lang_id = self.language
    self.client.send_goal(goal, feedback_cb=self.feedbackCb)
    goal_state = self.client.get_state()
    if locked:
      self.client.wait_for_result(timeout=rospy.Duration(5))

    response = self.client.get_result()
    if goal_state != GoalStatus.SUCCEEDED:
      self.client.stop_tracking_goal()


    return length/threshold

  def cancel_reproduction(self):
    rospy.loginfo("canceling...")
    self.sound_client.stopAll()
    rospy.loginfo("goal has been canceled")

  def get_status(self):
    if self.feedback == "TTS_EVENT_STARTED_PLAYING_WORD":
      return True
#
if __name__ == "__main__":
    speech = Speech("en_GB")
    text = "This is the sentence you have to reproduce"
    start = time.time()
    time_to_reproduce = speech.text_to_speech("111", True, 1)
    rospy.sleep(time_to_reproduce)
    elapsed_time = 0
    print("len:", time_to_reproduce)
    while(elapsed_time<time_to_reproduce):
      elapsed_time = time.time()-start
      print("wait")
    speech.cancel_reproduction()

    #success_against = speech.text_to_speech("test again agaist you", True)

    # while(elapsed_time<4):
    #   elapsed_time = time.time()-start
    #speech.cancel_reproduction()
    #while(speech.feedback!="TTS_EVENT_FINISHED_PLAYING_SENTENCE"):
    #  print("Do something else")
    #print("We're out")
    #while (speech.feedback)
