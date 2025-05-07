#!/usr/bin/env python

import rospy
import rosnode
import roslaunch
from tf_conversions import transformations
from math import pi
import tf
import csv
from geometry_msgs.msg import PoseStamped,Twist,PoseWithCovarianceStamped
import threading
from goal_loop import MultiGoals
import numpy as np
import os
import rospkg
from mapBlur import mapBlur
import psutil
from multiprocessing import Process

ymax=1
ymin=-1.75
x_start=1.65
stop_x=np.array([[-9.5,-9.0],[-8.67,-8.17],[-7.5,-7],[-6.37,-5.87],[-5.24,-4.74],[-4.1,-3.61],[-3.6,-3.1]])+0.35
light_x=np.array([[-9.5,-9.0],[-8.67,-8.17],[-7.5,-7],[-6.37,-5.87],[-5.24,-4.74],[-4.1,-3.61],[-3.6,-3.1]])-1.4
stop_idx=4
light_idx=2

def loop_run():
        try:
            # ROS Init
            #rospy.init_node('multi_goals', anonymous=True)
            retry = 1
            goalList = []
            goalListX = []
            goalListY = []
            goalListZ = []
            goalListW = []
            map_frame = rospy.get_param('~map_frame', 'map')

            with open('nav_point.csv', 'r') as f:
                reader = csv.reader(f)

                for cols in reader:
                    goalList.append([float(value) for value in cols])

                goalList = np.array(goalList)
                print("read suc!!")
                goalListX = goalList[:, 0]
                goalListY = goalList[:, 1]
                goalListZ = goalList[:, 2]
                goalListW = goalList[:, 3]

            if len(goalListX) == len(goalListY) & len(goalListY) >= 1:
                # Constract MultiGoals Obj
                rospy.loginfo("Multi Goals Executing...")
                mg = MultiGoals(goalListX, goalListY,goalListW,goalListZ ,retry, map_frame)
                rospy.spin()
            else:
                rospy.loginfo("Lengths of goal lists are not the same")
        except KeyboardInterrupt:
            print("shutting down")


class Run_launch():
    def __init__(self,launch_dir):  
        self.uuid=roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.tracking_launch = roslaunch.parent.ROSLaunchParent(self.uuid, launch_dir)

    def start(self):
        self.tracking_launch.start()

    def shutdown(self):
        self.tracking_launch.shutdown()

class Get_Point:
    def __init__(self,csv_write):
        self.tf_listener = tf.TransformListener()
        self.csv_write=csv_write
        
        try:
            self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(1.0))
            
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            return
        
    def get_pos(self): 
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        # rospy_Time(0)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf Error")
            return None

        # data = PoseStamped()  
        # data.pose.position.x = trans[0]
        # data.pose.position.y = trans[1]
        # data.pose.orientation.z = rot[2]
        # data.pose.orientation.w = rot[3]
        # pub.publish(data)
        #print(1)
        # self.csv_write.writerow([trans[0],trans[1],rot[2],rot[3]])  
        return trans[0],trans[1],rot[2],rot[3]

class Judgement():
    def __init__(self):
        self.laser_controller=rospy.Subscriber(
                "laser_control", Twist , self.laser_control_info, queue_size=10)
        self.nav_controller=rospy.Subscriber(
                "nav_control", Twist , self.nav_control_info, queue_size=10)
        self.pub = rospy.Publisher(
            "/car/cmd_vel", Twist, queue_size=10)
        self.control_info=Twist()
        self.state="laser"

    def laser_control_info(self,data):
        if self.state=="laser":
            self.control_info.linear.x=data.linear.x
            self.control_info.angular.z=data.angular.z
            self.pub.publish(self.control_info)
        elif self.state=="stop":
            self.control_info.linear.x=1000
            self.control_info.angular.z=80
            self.pub.publish(self.control_info)
        elif self.state=='circle':
            self.control_info.linear.x=1536
            self.control_info.angular.z=117.6
            self.pub.publish(self.control_info)
        elif self.state=="laser1":
            self.control_info.linear.x=1538
            #rospy.loginfo(data.linear.x)
            self.control_info.angular.z=data.angular.z
            #rospy.loginfo(data.angular.z)
            self.pub.publish(self.control_info)

    def nav_control_info(self,data):
#        if self.state=="nav":
#            self.control_info.linear.x = data.linear.x
#            #rospy.loginfo("nav_linear_x:",data.linear.x)
#            self.control_info.angular.z = data.angular.z
#            self.pub.publish(self.control_info)
#        elif self.state=="stop":
#            self.control_info.linear.x=1000
#            rospy.loginfo("stop:",data.linear.x)
#            self.control_info.angular.z=80
#            self.pub.publish(self.control_info)
        if self.state=="stop":
            self.control_info.linear.x = 1000
            #rospy.loginfo("nav_linear_x:",data.linear.x)
            self.control_info.angular.z = 105
            self.pub.publish(self.control_info)
        elif self.state=="nav":
            self.control_info.linear.x=data.linear.x
            rospy.loginfo("stop:",data.linear.x)
            self.control_info.angular.z=data.angular.z
            self.pub.publish(self.control_info)

    def cv_control_info(self,data):
        pass

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node('main')
    rviz=Run_launch(["/racecar/src/racecar/launch/rviz.launch"])
    gmapping=Run_launch(["/racecar/src/racecar/launch/Run_gmapping.launch"]) 
    amcl=Run_launch(["/racecar/src/racecar/launch/amcl_nav_teb.launch"])
    map_save=Run_launch(["/racecar/src/racecar/launch/map_save.launch"])
    map_adv=Run_launch(["/racecar/src/racecar/launch/map_adv.launch"])
    # amcl.start()
    # rospy.sleep(10)
    # subprocess_pid={}
    # process_list = psutil.process_iter(attrs=['pid', 'name'])
    #goal_loop=Process(target=loop_run)
    
    # for process in process_list:
    #     if "move_base" in process.info['name']:
    #         subprocess_pid["move_base"]=process.info['pid']
    #     if "car_controller_new" in process.info['name']:
    #         subprocess_pid["car_controller_new"]=process.info['pid']
    #     if "amcl" in process.info['name']:
    #         subprocess_pid["amcl"]=process.info['pid']
    
    # move_base = psutil.Process(subprocess_pid["move_base"]) 
    # amcl = psutil.Process(subprocess_pid["amcl"]) 
    # car_control=psutil.Process(subprocess_pid["car_controller_new"]) 
    # move_base.suspend() 
    # amcl.suspend()
    # car_control.suspend()
    # initial_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
    # pose_msg = PoseWithCovarianceStamped()
    # amcl.start() 
                    

    # rospy.sleep(3)
    #                 #x,y,z,w=nav_point.get_pos()
    # pose_msg.header.frame_id = "map"
    # pose_msg.pose.pose.position.x = 2
    # pose_msg.pose.pose.position.y = 0
    # pose_msg.pose.covariance[0] = 0.25
    # pose_msg.pose.covariance[6 * 1 + 1] = 0.25
    # pose_msg.pose.covariance[6 * 5 + 5] = 0.06853892326654787
    # pose_msg.pose.pose.orientation.z = 1
    # pose_msg.pose.pose.orientation.w = 1
    # initial_pose_pub.publish(pose_msg)
    # rospy.sleep(60)
    gmapping.start()
    rviz.start()
     
    initial_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
    
    pub = rospy.Publisher('/car/cmd_vel', Twist, queue_size=5)
    twist = Twist()   
    
    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.frame_id = "map"
    pose_msg.pose.covariance[0] = 0.25
    pose_msg.pose.covariance[6 * 1 + 1] = 0.25
    pose_msg.pose.covariance[6 * 5 + 5] = 0.06853892326654787
    mapblur=mapBlur()
    csv_dir = "nav_point.csv" 
    judger=Judgement()
    t1=threading.Thread(target = judger.spin)
    t1.start()
    goal_loop=threading.Thread(target = loop_run)
    with open(csv_dir,'w',encoding='utf8',newline='') as f :
        csv_write = csv.writer(f, delimiter=',', quoting=csv.QUOTE_MINIMAL)
        csv_write.writerow([-6.5,0,0,1])
        #csv_write.writerow([-4.5,0,0,1])
        csv_write.writerow([-4.5,0,0,1])
        csv_write.writerow([-1.5,0,0,1])        
        csv_write.writerow([0.9,0,0,1])
        nav_point = Get_Point(csv_write)
        r= rospy.Rate(60) 
        last_x=0
        last_y=0
        flg=0 
        is_origin=0
        cirlce_flag=1
        point_flag=1
        nav_flag=1
        while not rospy.is_shutdown():

            if judger.state=="laser" or judger.state=="nav" or judger.state=="laser1" or judger.state=='circle':
                x,y,z,w=nav_point.get_pos()
                if cirlce_flag and x>=x_start-0.1 and y<=ymax and y>=ymin:
                    judger.state='circle'
                    cirlce_flag=0
                # elif x<x_start+0.5 and y<=3.75 and y>=3.4 and cirlce_flag==0 and judger.state=='circle':
                elif x<x_start+0.6 and y>=2.45 and cirlce_flag==0 and judger.state=='circle':
                    judger.state='laser'
                if x<=light_x[light_idx,1] and x>=light_x[light_idx,0] and y<=1 and y>=-1.85 and flg and judger.state=='laser':  #x<=-0.2 and x>=-4.9 and y<=1 and y>=-1.75
                    judger.state="stop"
                    #f.close()
                    #amcl.start() 
                    #rosnode.kill_nodes(["laser_go"]) 
                    amcl.start() 
                    flg=0
                    #f.close()
                    x,y,z,w=nav_point.get_pos()
                    #rospy.loginfo("aaaaaaaaaaaaaa")
                    #rospy.loginfo(x)
                    pose_msg.pose.pose.position.x = x
                    pose_msg.pose.pose.position.y = y
                    pose_msg.pose.pose.orientation.z = z
                    pose_msg.pose.pose.orientation.w = w
                    #judger.state="stop"
                    #mapblur.mapBlur()
                    #map_adv.start()
                    rospy.sleep(0.7)#0.5
                    #rospy.loginfo("bbbbbbbbbbbb")
                    #rospy.loginfo(x)
                    initial_pose_pub.publish(pose_msg)
                    #rospy.loginfo("cccccccccccc")
                    #rospy.loginfo(x)
                    #judger.state="nav"
                    rospy.sleep(0.5) #0.4
                    judger.state="laser1"
                    #rosnode.kill_nodes(["laser_go"])
                    #rospy.loginfo("gogogo")
                    #rospy.loginfo("dddddddddddd")
                    #rospy.loginfo(x)                    
                    #goal_loop.start()
                    #rospy.sleep(0.5)
                    
                    #judger.state="nav"
                    #loop_run()
                    #goal_loop.start()
                    #map_save.start()
                    #rosnode.kill_nodes(["slam_gmapping"])
                    #rospy.loginfo("eeeeeeeeeeee")                    
                    #rospy.loginfo(x)
                    #rosnode.kill_nodes(["laser_go"])
                    #rospy.sleep(1)
                    #rosnode.kill_nodes(["slam_gmapping"])
                #elif x<=light_x[light_idx,1] and x>=light_x[light_idx,0] and y<=1 and y>=-1.85 and judger.state=='nav' and nav_flag:
#                elif x<=-7.5 and x>=-9.5 and y<=1 and y>=-1.85 and judger.state=='nav' and nav_flag:#and y<=1 and y>=-1.85
#                    judger.state='stop'
#                    #rospy.init_node('stop')
#                    nav_flag=0
#                    rospy.loginfo("222")
#                    twist.linear.x = 1300
#                    rospy.sleep(3)
#                    judger.state='nav'
                elif x<=-7.25 and x>=-7.5 and y<=1.5 and y>=-1.75  and judger.state=="laser1":#x<=1.5 and x>=-0.2                    
                    rosnode.kill_nodes(["laser_go"])   
                elif x<=2.5 and x>=2.25 and y<=1.5 and y>=-1.75  and judger.state=="nav":#x<=1.5 and x>=-0.2                    
                    rosnode.kill_nodes(["slam_gmapping"])     
                elif x<=-1.25 and x>=-2.2 and y<=1 and y>=-1.75  and judger.state=="laser1":#x<=1.5 and x>=-0.2                    
                    #rosnode.kill_nodes(["laser_go"])
                    judger.state="nav"
                    #rospy.loginfo(x)
#                    rospy.loginfo(z)
#                    rospy.loginfo(x)
#                    rospy.loginfo(y)
#                    rospy.loginfo(z)
#                    rospy.loginfo(y)
#                    rospy.loginfo(z)                    
                    rospy.loginfo("333")  
                    #rosnode.kill_nodes(["slam_gmapping"])                  
                    # rospy.loginfo("nav_x",x)
                    #rosnode.kill_nodes(["slam_gmapping"])
                    #rosnode.kill_nodes(["laser_go"])
                    #pass
#                elif x<=-3 and x>=-4.5 and y<=1 and y>=-1.75  and judger.state=="nav":
#                    judger.state="stop"
#                    #rospy.loginfo("666")
                elif (((x-last_x)**2+(y-last_y)**2)**(0.5)>=1 and (judger.state!='nav')) :
                    if x>=-8.5 and y<=1 and y>=-1.75  and judger.state=="laser1" and point_flag==1:
                        rospy.loginfo("jjjjjjjjjjjjjjjj")                    
                        rospy.loginfo(x)
                        #if x<=1.0 and x>=-1.0:
                        #  x=x-7.0
                        csv_write.writerow([-4.5,0.01,0,1])
                        #rospy.loginfo("fffffffffff")                    
                        #rospy.loginfo(x)
                        f.close()
                        #rospy.loginfo("gggggggggggg") 
                        #rospy.loginfo(x)
                        goal_loop.start()
                        point_flag=0
                        #rospy.loginfo("hhhhhhhhhhhhhh")    
                        #rospy.loginfo(x)
                        #rospy.loginfo("444") 
                    elif point_flag==1:
                        csv_write.writerow([x,y,z,w])
                        flg=1
                        last_x=x
                        last_y=y
                        #rospy.loginfo("iiiiiiiiiiiiiiii") 
                        #rospy.loginfo(x)
                        #rospy.loginfo("555")
#                if  judger.state=="laser1":
#                   rospy.loginfo(x)
#                   rospy.loginfo("----------------")   
                # elif x<=-7 and x>=-11 and y<=1.5 and y>=-1.75 and is_resume==0:
                #     csv_write.writerow([x,y,z,w])
                #     is_resume=1
                #     map_save.start()
                    #print("amcl start")
                #     move_base.resume() 
                #     amcl.resume(#
                #     car_control.resume()
                    #is_resume=1
                 
                
                
            r.sleep()
        amcl.shutdown()
        map_save.shutdown()
   
