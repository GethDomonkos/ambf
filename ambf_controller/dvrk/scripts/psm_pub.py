import sys
import rospy
import time
#from visualization_msgs.msg import Marker
from ambf_client import Client
from psmIK import *
from geometry_msgs.msg import TransformStamped

def psm_publisher():
    rospy.init_node('ambf_psm_cp', anonymous=True)
    pub = rospy.Publisher('ambf_psm_cp', TransformStamped, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    
    c = Client('psm_ik_test')
    c.connect()
    time.sleep(2.0)
    #print(c.get_obj_names())
    b = c.get_obj_handle('psm/baselink')
    
    while not rospy.is_shutdown():
        transform = TransformStamped()
        transform.header.frame_id = 'baselink-yawlink'
        transform.header.stamp = rospy.Time()
        
        rot_mat = compute_FK([b.get_joint_pos('baselink-yawlink'), b.get_joint_pos('yawlink-pitchbacklink'), b.get_joint_pos('pitchendlink-maininsertionlink'), b.get_joint_pos('maininsertionlink-toolrolllink'), b.get_joint_pos('toolrolllink-toolpitchlink'), b.get_joint_pos('toolpitchlink-toolgripper1link')])
        
        transform.transform.translation.x = rot_mat[0,3]
        transform.transform.translation.y = rot_mat[1,3]
        transform.transform.translation.z = rot_mat[2,3]
        
        pub.publish(transform)
        rate.sleep()
        
if __name__ == '__main__':
    
    
    try:
        psm_publisher()
    except rospy.ROSInterruptException:
        pass   
