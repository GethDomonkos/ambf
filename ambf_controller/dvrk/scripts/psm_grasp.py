import numpy as np
import rospy
import math
import time
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker

class PSM:
    def __init__(self):
        rospy.init_node('psm_grasp', anonymous = True)
        time.sleep(1.0)

        self.measured_cp = rospy.Subscriber("/ambf-psm_measured_cp", TransformStamped, self.cb_measured_cp)
        #self.measured_js_sub = rospy.Subscriber("/PSM1/jaw/measured_js", JointState, self.cb_measured_js)
        #self.dummy_marker_sub = rospy.Subscriber("/d_mark/dummy_target_marker", Marker, self.cb_dummy_marker)
        #self.dummy_cylinder_sub = rospy.Subscriber("/d_mark/dummy_cylinder", Marker, self.cb_dummy_cylinder)

        self.servo_cp_pub = rospy.Publisher("/ambf-psm_servo_cp", TransformStamped, queue_size=10)
        #self.servo_jp_pub = rospy.Publisher("PSM1/jaw/servo_jp", JointState, queue_size=10)
        
        time.sleep(1.0)
        """
        msg = self.measured_cp
        msg.transform.translation.x = 0.0
        msg.transform.translation.y = 0.0
        msg.transform.translation.z = -0.12
        msg.transform.rotation.x = 0.0
        msg.transform.rotation.y = -0.7
        msg.transform.rotation.z = 0.7
        msg.transform.rotation.w = 0.0
        self.servo_cp_pub.publish(msg)
        """

        #rospy.spin()

    def cb_measured_cp(self, msg):
        self.measured_cp = msg
        #print(self.measured_cp)
        
    def psm_home(self):
        msg = self.measured_cp
        msg.transform.translation.x = 0.0
        msg.transform.translation.y = 0.0
        msg.transform.translation.z = -0.12
        msg.transform.rotation.x = 0.0
        msg.transform.rotation.y = -0.7
        msg.transform.rotation.z = 0.7
        msg.transform.rotation.w = 0.0
        self.servo_cp_pub.publish(msg)

    def cb_measured_js(self, msg):
        self.measured_js = msg
        #print(self.measured_js)

    def cb_dummy_marker(self, msg):
        self.dmarker = msg

    def cb_dummy_cylinder(self, msg):
        self.cmarker = msg

    def move_tcp_to(self, target, v, dt):
        #print("pose before")
        #print("target:")
        #print(" x:", target[0])
        #print(" y:", target[1])
        #print(" z:", target[2])
        #print(self.measured_cp.transform)
        ratehz = 1 / dt
        rate = rospy.Rate(ratehz)
        rate.sleep()
        
        msg = self.measured_cp

        maxd = max(abs(self.measured_cp.transform.translation.x-target[0]), abs(self.measured_cp.transform.translation.y-target[1]), abs(self.measured_cp.transform.translation.z-target[2]))
    
        N = int(round((maxd / v) * ratehz))
        print("move to", target[0], target[1], target[2], "N=",N)

        xtr = np.linspace(self.measured_cp.transform.translation.x, target[0], N)
        ytr = np.linspace(self.measured_cp.transform.translation.y, target[1], N)
        ztr = np.linspace(self.measured_cp.transform.translation.z, target[2], N)

        for i in range(N):
            if rospy.is_shutdown():
                break

            msg.transform.translation.x = xtr[i]
            msg.transform.translation.y = ytr[i]
            msg.transform.translation.z = ztr[i]
            #msg.transform.rotation.x = 0.0
            #msg.transform.rotation.y = -0.7
            #msg.transform.rotation.z = 0.7
            #msg.transform.rotation.w = 0.0

            self.servo_cp_pub.publish(msg)
            rate.sleep()
            #rate.sleep()
            errorx = abs(xtr[i]-self.measured_cp.transform.translation.x)
            errory = abs(ytr[i]-self.measured_cp.transform.translation.y)
            errorz = abs(ztr[i]-self.measured_cp.transform.translation.z)
            error =   errorx + errory + errorz
            print(error)
            
            while(error > 0.1):
                print('here',error)
                rate.sleep()
                errorx = abs(xtr[i]-self.measured_cp.transform.translation.x)
                errory = abs(ytr[i]-self.measured_cp.transform.translation.y)
                errorz = abs(ztr[i]-self.measured_cp.transform.translation.z)
                error =   errorx + errory + errorz
                
                if rospy.is_shutdown():
                    break
                if i >= N:
                   break
                          

        #print("pose after")
        #print(self.measured_cp.transform)
        #print("error:")
        #print(" x:", target[0]-self.measured_cp.transform.translation.x)
        #print(" y:", target[1]-self.measured_cp.transform.translation.y)
        #print(" z:", target[2]-self.measured_cp.transform.translation.z)


    def move_jaw_to(self, target, omega, dt):
        #print("jaw before")
        #print(self.measured_js)
        msg = self.measured_js

        maxd = abs(self.measured_js.position[0]-target)

        ratehz = 1 / dt
        rate = rospy.Rate(ratehz)
        N = int(round((maxd / omega) * ratehz))
        print("jaw to", target, "N=",N)

        tr = np.linspace(self.measured_js.position[0], target, N)

        for i in range(N):
            if rospy.is_shutdown():
                break

            #msg.position[0] = tr[i]
            #msg.position = (msg.position[0] + tr[i],) + msg.position[1:]
            position = list(msg.position)
            position[0] = tr[i]
            msg.position = tuple(position)
            self.servo_jp_pub.publish(msg)
            rate.sleep()

        #print("jaw after")
        #print(self.measured_js)

    def grab_marker(self):
        #print(self.marker)
        print("grabbing marker at", self.dmarker.pose.position.x, self.dmarker.pose.position.y, self.dmarker.pose.position.z)
        self.move_tcp_to([self.dmarker.pose.position.x, self.dmarker.pose.position.y, self.dmarker.pose.position.z+0.02], vp, dtp)
        self.move_jaw_to(0.8, omegap, dtp)
        self.move_tcp_to([self.dmarker.pose.position.x, self.dmarker.pose.position.y, self.dmarker.pose.position.z+0.01], vp, dtp)
        self.move_jaw_to(0.2, omegap, dtp)

    def nav_around(self):
        print("navigating around marker, position:", self.cmarker.pose.position.x, self.cmarker.pose.position.y, self.cmarker.pose.position.z, " size:", self.cmarker.scale.x, self.cmarker.scale.y, self.cmarker.scale.z)
        self.move_tcp_to([(self.cmarker.pose.position.x+(self.cmarker.scale.x/2)), self.cmarker.pose.position.y, (self.cmarker.pose.position.z+self.cmarker.scale.z)], vp, dtp)

        maxd = abs(self.measured_js.position[0]-(self.cmarker.scale.x*math.pi))
        msg = self.measured_cp

        ratehz = 1 / dtp
        rate = rospy.Rate(ratehz)
        N = int(round((maxd / vp) * ratehz))
        print("N=",N)

        tr = np.linspace(0, 2*math.pi, N)

        for i in range(N):
            if rospy.is_shutdown():
                break

            #msg.position[0] = tr[i]
            #msg.position = (msg.position[0] + tr[i],) + msg.position[1:]
            #position = list(msg.position)
            #position[0] = tr[i]
            #msg.position = tuple(position)
            #self.servo_jp_pub.publish(msg)

            #x = (math.cos(tr[i])*(self.cmarker.scale.x/2))+self.cmarker.pose.position.x
            #y = (math.sin(tr[i])*(self.cmarker.scale.x/2))+self.cmarker.pose.position.y
            #z = self.cmarker.pose.position.z+self.cmarker.scale.z

            msg.transform.translation.x = (math.cos(tr[i])*(self.cmarker.scale.x/2))+self.cmarker.pose.position.x
            msg.transform.translation.y = (math.sin(tr[i])*(self.cmarker.scale.x/2))+self.cmarker.pose.position.y
            msg.transform.translation.z = self.cmarker.pose.position.z+self.cmarker.scale.z

            self.servo_cp_pub.publish(msg)

            #self.move_tcp_to([x, y, z], vp, dtp)

            rate.sleep()





if __name__ == '__main__':
    psm = PSM()
    print("psm")
    rospy.sleep(1)
    print("start")

    vp = 0.01  #0.05, 0.05, -0.15
    omegap = 0.1
    dtp = 0.5

    if rospy.has_param('/psm_grasp/dt'):
        dtp = rospy.get_param('/psm_grasp/dt')
    if rospy.has_param('/psm_grasp/omega'):
        omegap = rospy.get_param('/psm_grasp/omega')
    if rospy.has_param('/psm_grasp/v'):
        vp = rospy.get_param('/psm_grasp/v')

    print("speed=",vp,"omega=",omegap,"resolution=",dtp)
    
    rospy.sleep(1)
    psm.psm_home()
    rospy.sleep(1)
    psm.move_tcp_to([0.0, 0.0, -0.12], vp, dtp)          #target, v, 
    rospy.sleep(1)
    psm.move_tcp_to([0.1, 0.1, -0.20], vp, dtp)
    #psm.move_tcp_to([-0.1, 0.1, -0.06], 0.05, 0.01)
    #psm.move_tcp_to([-0.1, 0, -0.06], 0.01, 0.01)

    #psm.move_jaw_to(0.0, omegap, dtp)                     #target, omega, dt
    #psm.move_jaw_to(0.8, 0.1, 0.01)

    rospy.sleep(1)
    #psm.nav_around()
    rospy.sleep(1)
    #psm.grab_marker()


