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
    #change it to tts if you're using from the tiago
    #TODO change the name of the action to tts
    self.client = actionlib.SimpleActionClient('/tts', TtsAction)
    self.sound_client = SoundClient()
    self.language = language
    self.reproduction_has_ended = False
  def event(self, event):
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
      r = rospy.Rate(20)
      r.sleep()
      #rospy.loginfo("Feedback ")

  def activeCb(self):
      rospy.loginfo("Processing the goal")

  def doneCb(self, state, result):
      rospy.loginfo("Action server is done.")
      self.reproduction_has_ended = True
      return self.reproduction_has_ended
      

  def text_to_speech(self, text, locked=False):
    #rospy.loginfo("Waiting for Server")
    self.client.wait_for_server()
    #rospy.loginfo("Reached Server")
    goal = TtsGoal()
    goal.rawtext.text = text
    goal.rawtext.lang_id = self.language
    self.client.send_goal(goal, active_cb=self.activeCb, feedback_cb=self.feedbackCb, done_cb=self.doneCb)
    if locked:
      self.client.wait_for_result()

  def cancel_reproduction(self):
    rospy.loginfo("canceling...")
    self.text_to_speech("", False)
    rospy.loginfo("goal has been canceled")

if __name__ == "__main__":
    speech = Speech("en_GB")
    text = "Hola"
    speech.text_to_speech(text, True)
    print(speech.reproduction_has_ended)

