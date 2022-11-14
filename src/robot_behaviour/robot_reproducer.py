# coding=utf-8
from eyes_reproducer import Eyes
from gesture_reproducer import Gesture
from speech_reproducer import Voice


class Robot:
    """
  This is a module to reproduce a robot action combining speech,
  eyes expression and gesture.
  """

    def __init__(self, voice=None, gesture=None, eyes=None):
        """
    :param voice:  instance of class Voice
    :param gesture: instance of class Gesture
    :param speech: instance of class Eyes
    """

        self.voice = voice
        self.gesture = gesture
        self.eyes = eyes

    def reproduce_behaviour(self, sentence, gesture_name,
                            eyes_expression_name):
        """
    it reproduces a complex robot behaviour by
    combining multi-modal interactions such as voice,
    gesture and eyes expression

    sentence: the text the robot has to reproduce
    gesture_name: the name of the prerecorded motion
    the robot has to play using play_motion
    eyes_expression_name: the name of the eyes expression
    """

        self.voice.text_to_speech(sentence)
        self.gesture.reproduce_motion(gesture_name)
        self.eyes.reproduce_expression(eyes_expression_name)


def main():
    voice = Voice("it_IT")
    gesture = Gesture()
    eyes = Eyes()
    robot = Robot(voice=voice, gesture=gesture, eyes=eyes)
    sentence = "Ciao sono Antonio"
    gesture_name = "nodding.yaml"
    eyes_expression_name = "sad"
    robot.reproduce_behaviour(sentence, gesture_name, eyes_expression_name)


if __name__ == "__main__":
    main()
