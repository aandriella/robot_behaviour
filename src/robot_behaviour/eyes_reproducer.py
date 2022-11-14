import rospy
import std_msgs.msg
from hri_msgs.msg import Expression


class Eyes:

    def __init__(self):
        rospy.init_node('robot_behaviour', anonymous=True)
        self.eyes_pub = rospy.Publisher('/eyes/expression', Expression, queue_size=10)

    # define publisher for changing robot facial expression
    def reproduce_expression(self, expression_name):
        expression_msg = Expression()
        expression_msg.header = std_msgs.msg.Header()
        expression_msg.expression = expression_name
        rospy.loginfo(expression_msg)
        self.eyes_pub.publish(expression_msg)

    def reset_expression(self):
        expression_msg = Expression()
        expression_msg.header = std_msgs.msg.Header()
        expression_msg.expression = "neutral"
        rospy.loginfo(expression_msg)
        self.eyes_pub.publish(expression_msg)


def main():
    eyes = Eyes()
    eyes.reproduce_expression("happy")


if __name__ == "__main__":
    main()
