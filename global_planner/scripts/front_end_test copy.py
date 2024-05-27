#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty 
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
import numpy as np
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import PoseStamped

# Publisher of server events to trigger change of states for trajectory server 
# dbg_start_pub = rospy.Publisher('/drone0_ego_planner_node/debug/plan_start', Pose, queue_size=5)
# dbg_goal_pub = rospy.Publisher('/drone0_ego_planner_node/debug/plan_goal', Pose, queue_size=5)

class testPlanner():

    def __init__(self):
        self.rate = rospy.Rate(1) 
        self.dbg_start_pub = rospy.Publisher('/global_planner/debug/plan_start', Pose, queue_size=5)
        self.dbg_goal_pub = rospy.Publisher('/global_planner/debug/plan_goal', Pose, queue_size=5)
        self.grid_map_pub = rospy.Publisher('/grid_map/cloud', PointCloud2, queue_size=1)
        self.grid_pose_pub = rospy.Publisher('/grid_map/pose', PoseStamped, queue_size=1)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1, latch=False)

        self.plan_on_demand_pub = rospy.Publisher('/global_planner/plan_on_demand', Empty, queue_size=5)

        self.init_pose()
        self.init_end_pose()
        self.init_start_pose()
        self.init_pointcloud()

        self.pc_timer = rospy.Timer(rospy.Duration(1./30), self.publish_pointcloud)
        # self.plan_timer = rospy.Timer(rospy.Duration(1./1), self.publish_plan_request)

    def create_pose(self,x, y, z):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
        return pose

    def publish_pointcloud(self,event):
        cloud_points = [[1.0, 1.0, 0.0],[1.0, 2.0, 0.0]]
        #header
        ros_time = rospy.Time.now()

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        pc_msg = pcl2.create_cloud_xyz32(header, cloud_points)

        self.publish_posestamped(ros_time)

        self.grid_map_pub.publish(pc_msg)
        # self.grid_pose_pub.publish(self.pose)

        
    def publish_plan_request(self,event):
        empty_msg = Empty()
        self.dbg_start_pub.publish(self.start)
        self.dbg_goal_pub.publish(self.goal) 
        self.plan_on_demand_pub.publish(empty_msg)

    def init_pointcloud(self):
            #publish pointcloud
        x = np.linspace(-20, 20, 200)
        y = np.linspace(-20, 20, 200)
        xv, yv = np.meshgrid(x, y)
        points = np.vstack([xv.T.ravel(), yv.T.ravel()]).T
        self.a = np.ones((40000,3))
        self.a[:,0] = points[:,0]
        self.a[:,2] = points[:,1]

    def init_start_pose(self):
        self.start = self.create_pose(0.0, 0.0, 1.0)
        self.rate.sleep()
        self.dbg_start_pub.publish(self.start)

    def init_end_pose(self):
        self.goal = self.create_pose(6.5, 6.5, 3.0)
        self.rate.sleep()
        self.dbg_goal_pub.publish(self.goal) 

    def init_pose(self):
        self.pose = self.create_pose(0.0, 0.0, 1.0)
        self.rate.sleep()

    def create_odom(self, x, y, z, ros_time):
        pose = Odometry()

        pose.header.stamp = ros_time

        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.position.z = z

        pose.pose.pose.orientation.x = 0
        pose.pose.pose.orientation.y = 0
        pose.pose.pose.orientation.z = 0
        pose.pose.pose.orientation.w = 1
        return pose
    
    def create_posestamped(self,x, y, z, ros_time):
        pose = PoseStamped()

        pose.header.stamp = ros_time

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        return pose
    
    def publish_posestamped(self, ros_time):
        pose = self.create_posestamped(0.0, 0.0, 1.0, ros_time)
        self.grid_pose_pub.publish(pose)
        # odom = self.create_odom(0.0, 0.0, 1.0, ros_time)
        # self.odom_pub.publish(odom)


if __name__=="__main__":
    rospy.init_node('mission_startup', anonymous=True)
    print(f"Sending waypoints to UAVs")

    testplanner = testPlanner()
    rospy.spin()
