import rospy
from std_msgs.msg import String
import cv2
import numpy as np

def nothing():
        pass

   
def setter():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)        
    # Create a black image, a window
    #img = np.zeros((300,512,3), np.uint8)
    cv2.namedWindow('image')

    # create trackbars for color change
    cv2.createTrackbar('R','image',0,255,nothing)
    cv2.createTrackbar('G','image',0,255,nothing)
    cv2.createTrackbar('B','image',0,255,nothing)

    # create switch for ON/OFF functionality
    #switch = '0 : OFF \n1 : ON'
    #cv2.createTrackbar(switch, 'image',0,1,nothing)

    while(1):
        #cv2.imshow('image',img)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break

        # get current positions of four trackbars
        #global r, g, b
        r = cv2.getTrackbarPos('R','image')
        g = cv2.getTrackbarPos('G','image')
        b = cv2.getTrackbarPos('B','image')
        #s = cv2.getTrackbarPos(switch,'image')

        #if s == 0:
        #    img[:] = 0
        #else:
        #    img[:] = [b,g,r]
        
        hi_str = "red is " + str(r) + ", blue is " + str(b) + ", green is " + str(g)
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hi_str)
        pub.publish(hi_str)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        setter()
    except rospy.ROSInterruptException:
        pass