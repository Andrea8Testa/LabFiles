#!/usr/bin/python2.7


##### Just Checking Whether Tensorflow works with ROS or not! #####

import tensorflow as tf
import time
from sensor_msgs.msg import Image
from std_msgs.msg import String
import rospy


class RosTensorFlow():
    def __init__(self):

#        self._sub = rospy.Subscriber('image', Image, self.callback, queue_size=1)
        self._pub = rospy.Publisher('result', String, queue_size=1)
        
        
       

    def main(self):
        
        rate = rospy.Rate(10)
        
        matrix_1 = tf.Variable([[1,2,3],[4,5,6],[7,8,9]], name="mat1")
        matrix_2 = tf.Variable([[1,2,3],[4,5,6],[7,8,9]], name="mat2")
        
        scalar = tf.constant(5)
        self.number = tf.Variable(1, name="counter")
        
        add_msg = tf.constant("\nResult of matrix addition\n")
        mul_msg = tf.constant("\nResult of matrix multipication\n")
        scalar_mul_msg = tf.constant("\nResult of scalar multipication\n")
        self.number_mul_msg = tf.constant("\nResult of Number multipication\n")
        
        mat_add = tf.add(matrix_1, matrix_2)
        mat_mul = tf.matmul(matrix_1, matrix_2)
        mat_scalar_mul = tf.multiply(scalar, mat_mul)
        self.mat_number_mul = tf.multiply(self.number, mat_mul)
        
        init_op = tf.initialize_all_variables()
        sess = tf.Session()
        tf.device("/gpu:0")
        sess.run(init_op)
        
        i=0
        while not rospy.is_shutdown():
            i+=1
            
            jstring = "i is %i" %i
            self._pub.publish(jstring)
            
            print('\nFor i =', i)
            
            print(sess.run(add_msg))
            print(sess.run(mat_add))
        
            print(sess.run(mul_msg))
            print(sess.run(mat_mul))
            
            print(sess.run(scalar_mul_msg))
            print(sess.run(mat_scalar_mul))
            
            update = tf.assign(self.number, tf.constant(i))
            sess.run(update)
            print(sess.run(self.number_mul_msg))
            print(sess.run(self.mat_number_mul))
            
            rate.sleep()
#            time.sleep(0.1)
        
        sess.close()
   
#        rospy.spin()

if __name__ == '__main__':
    try:
        
        rospy.init_node('rostensorflow')
        tensor = RosTensorFlow()
        tensor.main()
        
    except rospy.ROSInitException:
        pass


       
