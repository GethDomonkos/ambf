from psmIK import *
from ambf_client import Client
import time
import numpy
import sys
import rospy
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_matrix #Return homogeneous rotation matrix from quaternion.
from tf.transformations import quaternion_from_matrix  #Return quaternion from rotation matrix.
from tf.transformations import quaternion_multiply
from threading import Timer
#from twisted.internet import task, reactor

class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        
        self.interval   = interval
        self.function   = function
        self._timer     = None
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False

class psm():
    def __init__(self):
        rospy.init_node('ambf-psm_cp', anonymous=True)
        
        c = Client('psm_ik_test')
        c.connect()
        time.sleep(2.0)
        self.rate = rospy.Rate(10) # 10hz
        #print(c.get_obj_names())
        self.b = c.get_obj_handle('psm/baselink')
        self.target_ik = c.get_obj_handle('psm/target_ik')
        self.target_fk = c.get_obj_handle('psm/target_fk')
        time.sleep(1.0)
        
        self.rt = RepeatedTimer(0.01, self.psm_pub2)
        """"
        T_7_0 = np.eye(4,4) #np.zeros((4, 4))        
        T_7_0[0] = [-0.0, 1.0, 0.0, -0.0]
        T_7_0[1] = [1.0, 0.0, -0.0, -0.0]
        T_7_0[2] = [-0.0, -0.0, -1.0, -0.0]
        T_7_0[3] = [0.0, 0.0, 0.0, 1.0]
        """
        
        
        #self.measured_cp_sub = rospy.Subscriber("/PSM1/measured_cp", TransformStamped, self.cb_measured_cp)
        #self.servo_cp_pub = rospy.Publisher("PSM1/servo_cp", TransformStamped, queue_size=10)
        #pub = rospy.Publisher('ambf_psm_cp', TransformStamped, queue_size=10)
        
        self.pub = rospy.Publisher('/ambf-psm_measured_cp', TransformStamped, queue_size=10)
        self.sub = rospy.Subscriber('/ambf-psm_servo_cp', TransformStamped, self.cb_servo_cp)
        time.sleep(1.0)
        
        print('all running')
        
        #rospy.spin()
        
    def cb_servo_cp(self, msg):
        #print(msg)
        self.rt.stop()
        num_joints = 7
        joint_lims = np.zeros((num_joints, 2))
        joint_lims[0] = [np.deg2rad(-91.96), np.deg2rad(91.96)]
        joint_lims[1] = [np.deg2rad(-60), np.deg2rad(60)]
        joint_lims[2] = [0.0, 0.24]
        joint_lims[3] = [np.deg2rad(-175), np.deg2rad(175)]
        joint_lims[4] = [np.deg2rad(-90), np.deg2rad(90)]
        joint_lims[5] = [np.deg2rad(-85), np.deg2rad(85)]
        joint_lims[6] = [0.0, 0.0]

        T_7_0 = np.eye(4,4) #np.zeros((4, 4))        
        #T_7_0[0] = [-0.0, 1.0, 0.0, -0.0]
        #T_7_0[1] = [1.0, 0.0, -0.0, -0.0]
        #T_7_0[2] = [-0.0, -0.0, -1.0, -0.0]
        #T_7_0[3] = [0.0, 0.0, 0.0, 1.0]
        
        #T_7_0[2,3] = T_7_0[2,3] - (i/50)
        T_7_0 = quaternion_matrix([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])
        T_7_0[0,3] = msg.transform.translation.x
        T_7_0[1,3] = msg.transform.translation.y
        T_7_0[2,3] = msg.transform.translation.z
        
        print(T_7_0)

        if self.target_ik is not None:
            P_0_w = Vector(self.b.get_pos().x, self.b.get_pos().y, self.b.get_pos().z)
            R_0_w = Rotation.RPY(self.b.get_rpy()[0], self.b.get_rpy()[1], self.b.get_rpy()[2])
            T_0_w = Frame(R_0_w, P_0_w)
            T_7_w = T_0_w * convert_mat_to_frame(T_7_0)
            self.target_ik.set_pos(T_7_w.p[0], T_7_w.p[1], T_7_w.p[2])
            self.target_ik.set_rpy(T_7_w.M.GetRPY()[0], T_7_w.M.GetRPY()[1], T_7_w.M.GetRPY()[2])

        computed_q = compute_IK(convert_mat_to_frame(T_7_0))
        computed_q = enforce_limits(computed_q, joint_lims)
        
        if self.target_fk is not None:
            P_0_w = Vector(self.b.get_pos().x, self.b.get_pos().y, self.b.get_pos().z)
            R_0_w = Rotation.RPY(self.b.get_rpy()[0], self.b.get_rpy()[1], self.b.get_rpy()[2])
            T_0_w = Frame(R_0_w, P_0_w)
            computed_q.append(0)
            T_7_0_fk = compute_FK(computed_q)
            T_7_w = T_0_w * convert_mat_to_frame(T_7_0_fk)
            self.target_fk.set_pos(T_7_w.p[0], T_7_w.p[1], T_7_w.p[2])
            self.target_fk.set_rpy(T_7_w.M.GetRPY()[0], T_7_w.M.GetRPY()[1], T_7_w.M.GetRPY()[2])

        self.b.set_joint_pos('baselink-yawlink', computed_q[0])
        self.b.set_joint_pos('yawlink-pitchbacklink', computed_q[1])
        self.b.set_joint_pos('pitchendlink-maininsertionlink', computed_q[2])
        self.b.set_joint_pos('maininsertionlink-toolrolllink', computed_q[3])
        self.b.set_joint_pos('toolrolllink-toolpitchlink', computed_q[4])
        self.b.set_joint_pos('toolpitchlink-toolgripper1link', computed_q[5])
        self.b.set_joint_pos('toolpitchlink-toolgripper2link', -computed_q[5])
        
        self.rt.start()
        
        
#-------------------FK publisher-----------------------------------------------------------------------------------------------------        
    def psm_pub(self):
        while not rospy.is_shutdown():
            transform = TransformStamped()
            transform.header.frame_id = 'baselink-yawlink'
            transform.header.stamp = rospy.Time()
        
            rot_mat = compute_FK([self.b.get_joint_pos('baselink-yawlink'), self.b.get_joint_pos('yawlink-pitchbacklink'), self.b.get_joint_pos('pitchendlink-maininsertionlink'), self.b.get_joint_pos('maininsertionlink-toolrolllink'), self.b.get_joint_pos('toolrolllink-toolpitchlink'), self.b.get_joint_pos('toolpitchlink-toolgripper1link')])
        
            transform.transform.translation.x = rot_mat[0,3]
            transform.transform.translation.y = rot_mat[1,3]
            transform.transform.translation.z = rot_mat[2,3]
        
            quaternion_of_R = quaternion_from_matrix(rot_mat)
            #print(quaternion_matrix(quaternion_of_R))
            
            transform.transform.rotation.x = quaternion_of_R[0]
            transform.transform.rotation.y = quaternion_of_R[1]
            transform.transform.rotation.z = quaternion_of_R[2]
            transform.transform.rotation.w = quaternion_of_R[3]

            self.pub.publish(transform)
            self.rate.sleep()
            
    
    def psm_pub2(self):
        transform = TransformStamped()
        transform.header.frame_id = 'baselink-yawlink'
        transform.header.stamp = rospy.Time()
        
        rot_mat = compute_FK([self.b.get_joint_pos('baselink-yawlink'), self.b.get_joint_pos('yawlink-pitchbacklink'), self.b.get_joint_pos('pitchendlink-maininsertionlink'), self.b.get_joint_pos('maininsertionlink-toolrolllink'), self.b.get_joint_pos('toolrolllink-toolpitchlink'), self.b.get_joint_pos('toolpitchlink-toolgripper1link')])
       
        transform.transform.translation.x = rot_mat[0,3]
        transform.transform.translation.y = rot_mat[1,3]
        transform.transform.translation.z = rot_mat[2,3]
        
        quaternion_of_R = quaternion_from_matrix(rot_mat)
        #print(quaternion_matrix(quaternion_of_R))
            
        transform.transform.rotation.x = quaternion_of_R[0]
        transform.transform.rotation.y = quaternion_of_R[1]
        transform.transform.rotation.z = quaternion_of_R[2]
        transform.transform.rotation.w = quaternion_of_R[3]

        self.pub.publish(transform)
     #   self.rate.sleep()
        if rospy.is_shutdown():
            self.rt.stop()
    
    
    
    
if __name__ == '__main__':
    psm = psm()
    print("psm")
    rospy.sleep(1)
    print("start")
    #psm.psm_pub()
    #psm.psm_pub2()
    #rt = RepeatedTimer(0.01, psm.psm_pub2)
    
    #timeout = 0.1
    
    #l = task.LoopingCall(psm.psm_pub2)
    #l.start(timeout) # call every timeout seconds

    #reactor.run()
    
    while not rospy.is_shutdown():
        time.sleep(1)
    
    #rt.stop()
    
    #rospy.spin()
        
    #l.stop()
    #reactor.stop()
