import rospy
from std_msgs.msg import String
import cv2
import numpy as np

def nothing():
        pass

   
def setter():
    pub = rospy.Publisher('gains', String, queue_size=10)
    rospy.init_node('setter', anonymous=True)        
    # Create a black image, a window
    #img = np.zeros((300,512,3), np.uint8)
    cv2.namedWindow('image')

    # create trackbars for color change
    cv2.createTrackbar('x_gain*1E2','image',0,1000,nothing)
    cv2.createTrackbar('xmin*1E1','image',0,220,nothing)
    cv2.createTrackbar('xmax*1E1','image',0,300,nothing)
    cv2.createTrackbar('st_gain*1E1','image',0,3000,nothing)

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
        x_gain = cv2.getTrackbarPos('x_gain*1E2','image')/100.0
        xmin = cv2.getTrackbarPos('xmin*1E1','image')/10.0
        xmax = cv2.getTrackbarPos('xmax*1E1','image')/10.0
        st_gain = cv2.getTrackbarPos('st_gain*1E1','image')/10.0
        #s = cv2.getTrackbarPos(switch,'image')

        #if s == 0:
        #    img[:] = 0
        #else:
        #    img[:] = [b,g,r]
        
        gain_str = str(x_gain) + " " + str(xmin) + " " + str(xmax) + " " + str(st_gain)
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(gain_str)
        pub.publish(gain_str)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        setter()
    except rospy.ROSInterruptException:
        pass