import rospy 
import tf
from geometry_msgs.msg import Twist

#Gets the circle position
def getCenterPos():
    return 0

def getDistanceFromCenter():
    return 0

def getCenterOrientation():
    return 0

def sendDistances():
    return 0

distanceP = 0.1
angleP = 0.1

targetDistance = 10

constVx = 1

#pub = rospy.Publisher('chatter', Twist, queue_size=10)

rospy.init_node("CircleTest")
listener = tf.TransformListener()

while not rospy.is_shutdown():
    #print("hello")
    try:
        (trans,rot) = listener.lookupTransform('odom', 'base_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    print(trans[0])

    #distanceError = targetDistance - getDistanceFromCenter()
    #angleError = -getCenterOrientation

    #vX = constVx
    #vY = distanceError * distanceP
    #w = angleP * angleError

