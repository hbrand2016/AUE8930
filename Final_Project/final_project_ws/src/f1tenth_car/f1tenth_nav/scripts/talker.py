#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import cv2
import numpy as np
import threading

def nothing(self):
        pass

class setting:
    def __init__(self):
        self.ask = 0
    
    def say_hi(self):
        print "hi"
    
    def setter(self):
        # Create a black image, a window
        img = np.zeros((300,512,3), np.uint8)
        cv2.namedWindow('image')

        # create trackbars for color change
        cv2.createTrackbar('R','image',0,255,nothing)
        cv2.createTrackbar('G','image',0,255,nothing)
        cv2.createTrackbar('B','image',0,255,nothing)

        # create switch for ON/OFF functionality
        switch = '0 : OFF \n1 : ON'
        cv2.createTrackbar(switch, 'image',0,1,nothing)

        while(1):
            cv2.imshow('image',img)
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break

            # get current positions of four trackbars
            #global r, g, b
            r = cv2.getTrackbarPos('R','image')
            g = cv2.getTrackbarPos('G','image')
            b = cv2.getTrackbarPos('B','image')
            s = cv2.getTrackbarPos(switch,'image')
            
            pub = rospy.Publisher('chatter', String, queue_size=10)
            rospy.init_node('talker', anonymous=True)
            if s == 0:
                img[:] = 0
            else:
                img[:] = [b,g,r]

        cv2.destroyAllWindows()

class talking:
    def __init__(self):
        self.ask = 0

    def talker(self):
        pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        

        # create trackbars for color change
        global r, g, b

        #rate = rospy.Rate(1) # 10hz
        while not rospy.is_shutdown():
            
            hi_str = "red is " + str(r) + ", blue is " + str(b) + ", green is " + str(g)
            hello_str = "hello world %s" % rospy.get_time()
            #rospy.loginfo(hi_str)
            pub.publish(hi_str)
            #rate.sleep()

if __name__ == '__main__':
    r = 150
    g = 150
    b = 150
    
    S = setting()
    T = talking()
    
    S_t = threading.Thread(target=S.say_hi(), args=())
    T_t = threading.Thread(target=T.talker(), args=())
    S_t.start()
    T_t.start()

#    try:
#        talker()
#    except rospy.ROSInterruptException:
#        pass
