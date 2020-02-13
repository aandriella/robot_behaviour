#import ROS headers
import rospy
import actionlib

#import PAL Robotics custom headers
from pal_interaction_msgs.msg import TtsAction, TtsGoal


class Speech():
  def __init__(self, language):
    rospy.init_node('big_hero', anonymous=True)
    self.client = actionlib.SimpleActionClient('tts', TtsAction)
    self.language = language
  def event(self, event):
    print("event {}".format(event))
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
    print("event type: " + self.event(feedback.event_type))
    print("timestamp: " + str(feedback.timestamp))
    print("current word: " + feedback.text_said)
    print("next word: " + feedback.next_word)
    print("-")

  def text_to_speech(self, text):
    rospy.loginfo("Waiting for Server")
    self.client.wait_for_server()
    rospy.loginfo("Reached Server")
    goal = TtsGoal()
    goal.rawtext.text = text
    goal.rawtext.lang_id = self.language
    self.client.send_goal(goal)
    self.client.wait_for_result()
    res = self.client.get_result()
    print("text: " + res.text)
#    print("warning/error msgs: " + res.msg)


if __name__ == "__main__":
  speech = Speech("en_GB")
  speech.text_to_speech("Hi I'm socrates")
  


