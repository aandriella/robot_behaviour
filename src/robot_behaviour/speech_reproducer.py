# import ROS headers
import rospy
import actionlib
# import PAL Robotics custom headers
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from sound_play.libsoundplay import SoundClient


class Voice:

    def __init__(self, language):
        rospy.init_node('robot_behaviour', anonymous=True)
        # change it to tts if you're using from the tiago
        self.client = actionlib.SimpleActionClient('/tts', TtsAction)
        self.sound_client = SoundClient()
        self.language = language

    def text_to_speech(self, text):
        self.client.wait_for_server()
        goal = TtsGoal()
        goal.rawtext.text = text
        goal.rawtext.lang_id = self.language
        self.client.send_goal(goal)
        # we wait until the action won't finish
        self.client.wait_for_result()

    def cancel_reproduction(self):
        rospy.loginfo("canceling...")
        self.text_to_speech("")
        rospy.loginfo("goal has been canceled")


if __name__ == "__main__":
    speech = Voice("it_IT")
    text = "Ciao"
    speech.text_to_speech(text)
    print(speech.reproduction_has_ended)
