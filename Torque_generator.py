#!/usr/bin/env python
import rospy
import copy
from std_msgs.msg import Float64
#from std_msgs.msg import Float64MultiArray
import numpy as np

TEMPO = 0

def callback(data):
    global TEMPO
    TEMPO = data.data
    
def Generatore_Coppia():
    t = copy.deepcopy(TEMPO)
    coppia_tr = np.zeros(7)
    coppia_tr[0] = 0 #3*np.sin(t*3.14)
    coppia_tr[1] = 3*np.sin(t*3.14)
    coppia_tr[2] = 0#1*np.sin(t*3.14)
    coppia_tr[3] = 0#1*np.sin(t*3.14)
    coppia_tr[4] = 0 #1*np.sin(t*3.14)
    coppia_tr[5] = 0#1*np.sin(t*3.14)
    coppia_tr[6] = 0#1*np.sin(t*3.14)
    rate = rospy.Rate(1000) # 1000hz
    #Coppia_to_send = coppia_tr[0]
    #Coppia_to_send.data = coppia_tr[0]
    Coppia0_pub.publish(coppia_tr[0])
    Coppia1_pub.publish(coppia_tr[1])
    Coppia2_pub.publish(coppia_tr[2])
    Coppia3_pub.publish(coppia_tr[3])
    Coppia4_pub.publish(coppia_tr[4])
    Coppia5_pub.publish(coppia_tr[5])
    Coppia6_pub.publish(coppia_tr[6])
    #print(Coppia_to_send.data
    print("t", t)
    rate.sleep()


if __name__ == '__main__':
    rospy.init_node('Torque_generator', anonymous=True)
    
    time_sub = rospy.Subscriber('/Tempo', Float64, callback,
    queue_size=10, tcp_nodelay=True)
    
    #pub = rospy.Publisher('/Coppia', Float64MultiArray, queue_size=10)
    Coppia0_pub = rospy.Publisher('/Coppia0', Float64, queue_size=10)
    Coppia1_pub = rospy.Publisher('/Coppia1', Float64, queue_size=10)
    Coppia2_pub = rospy.Publisher('/Coppia2', Float64, queue_size=10)
    Coppia3_pub = rospy.Publisher('/Coppia3', Float64, queue_size=10)
    Coppia4_pub = rospy.Publisher('/Coppia4', Float64, queue_size=10)
    Coppia5_pub = rospy.Publisher('/Coppia5', Float64, queue_size=10)
    Coppia6_pub = rospy.Publisher('/Coppia6', Float64, queue_size=10)
    while not rospy.is_shutdown():
        try:
            Generatore_Coppia()
        except rospy.ROSInterruptException:
            pass
    rospy.spin() 