import rospy
from std_msgs.msg import String





class Face:

  def __init__(self):
    rospy.init_node('big_hero', anonymous=True)
    self.face_pub = rospy.Publisher('/facial_expression', String, queue_size=10)




  #define publisher for changing robot facial expression
  def reproduce_face_expression(self, string):
    rospy.loginfo(string)
    self.face_pub.publish(string)

  def move_eyes(self, x , y):
    self.face_pub.publish("look_"+str(x)+"_"+str(y))



def main():
  face = Face()
  face.reproduce_face_expression("happy")
  rospy.sleep(2.0)
  face.reproduce_face_expression("sad")
  rospy.sleep(2.0)
  face.reproduce_face_expression("confused")
  rospy.sleep(2.0)
  face.move_eyes(0, -50)

if __name__=="__main__":
  main()
