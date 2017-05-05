#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import cv2
import numpy as np
import multiprocessing

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
            r.value = cv2.getTrackbarPos('R','image')
            g.value = cv2.getTrackbarPos('G','image')
            b.value = cv2.getTrackbarPos('B','image')
            s = cv2.getTrackbarPos(switch,'image')

            if s == 0:
                img[:] = 0
            else:
                img[:] = [b.value,g.value,r.value]

        cv2.destroyAllWindows()

class talking:
    def __init__(self):
        self.ask = 0

    def talker(self):
        pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        

        # create trackbars for color change
        # global r, g, b

        rate = rospy.Rate(1) # 10hz
        while not rospy.is_shutdown():
            
            hi_str = "red is " + str(r.value) + ", blue is " + str(b.value) + ", green is " + str(g.value)
            hello_str = "hello world %s" % rospy.get_time()
            rospy.loginfo(hi_str)
            pub.publish(hi_str)
            rate.sleep()

if __name__ == '__main__':
    r = multiprocessing.Value('i',150)
    g = multiprocessing.Value('i',150)
    b = multiprocessing.Value('i',150)
    
    S = setting()
    T = talking()
    
    S_t = multiprocessing.Process(target=S.say_hi(), args=())
    T_t = multiprocessing.Process(target=T.talker(), args=())
    S_t.start()
    T_t.start()

    S_t.join()
    T_t.join()

#    try:
#        talker()
#    except rospy.ROSInterruptException:
#        pass
