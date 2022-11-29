from psmIK import *
from ambf_client import Client
import time
import numpy
import sys
import rospy
from geometry_msgs.msg import TransformStamped

class psm():
    def __init__(self):
        rospy.init_node('ambf-psm_cp', anonymous=True)
        
        c = Client('psm_ik_test')
        c.connect()
        time.sleep(2.0)
        self.rate = rospy.Rate(10) # 10hz
        #print(c.get_obj_names())
        self.b = c.get_obj_handle('psm/baselink')
        
        #self.measured_cp_sub = rospy.Subscriber("/PSM1/measured_cp", TransformStamped, self.cb_measured_cp)
        #self.servo_cp_pub = rospy.Publisher("PSM1/servo_cp", TransformStamped, queue_size=10)
        #pub = rospy.Publisher('ambf_psm_cp', TransformStamped, queue_size=10)
        
        self.pub = rospy.Publisher('ambf-psm_measured_cp', TransformStamped, queue_size=10)
        self.sub = rospy.Subscriber('ambf-psm_servo_pc', TransformStamped, self.cb_servo_cp)
        
    def cb_servo_cp(self, msg):
        print(msg)
        
    def psm_pub(self):
        while not rospy.is_shutdown():
            transform = TransformStamped()
            transform.header.frame_id = 'baselink-yawlink'
            transform.header.stamp = rospy.Time()
        
            rot_mat = compute_FK([self.b.get_joint_pos('baselink-yawlink'), self.b.get_joint_pos('yawlink-pitchbacklink'), self.b.get_joint_pos('pitchendlink-maininsertionlink'), self.b.get_joint_pos('maininsertionlink-toolrolllink'), self.b.get_joint_pos('toolrolllink-toolpitchlink'), self.b.get_joint_pos('toolpitchlink-toolgripper1link')])
        
            transform.transform.translation.x = rot_mat[0,3]
            transform.transform.translation.y = rot_mat[1,3]
            transform.transform.translation.z = rot_mat[2,3]
        
            self.pub.publish(transform)
            self.rate.sleep()
    
    
    
    
if __name__ == '__main__':
    psm = psm()
    print("psm")
    rospy.sleep(1)
    print("start")
    psm.psm_pub()
        
    
