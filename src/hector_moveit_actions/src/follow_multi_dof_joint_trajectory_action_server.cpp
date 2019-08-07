#include <ros/ros.h>
#include <tf/tf.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <hector_moveit_actions/FollowMultiDofJointTrajectoryAction.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

#include <hector_uav_msgs/PoseAction.h>
#include <hector_uav_msgs/EnableMotors.h>

#include <cmath>
#define _USE_MATH_DEFINES

#define MAX_SPEED 1.5
#define EPSILON 1e-4
class FollowMultiDofJointTrajectoryAction{
    private:

        typedef actionlib::SimpleActionServer<hector_moveit_actions::FollowMultiDofJointTrajectoryAction> TrajectoryActionServer;
        typedef actionlib::SimpleActionClient<hector_uav_msgs::PoseAction> OrientationClient;
        typedef hector_moveit_actions::FollowMultiDofJointTrajectoryResult Result;
        typedef hector_moveit_actions::FollowMultiDofJointTrajectoryFeedback Feedback;
        ros::NodeHandle nh_;
        ros::Publisher vel_pub;
        ros::Subscriber pose_sub;
        TrajectoryActionServer server_;
        OrientationClient orientation_client_;
        std::string action_name;

        geometry_msgs::Twist empty,cmd;
        trajectory_msgs::MultiDOFJointTrajectory trajectory;
        geometry_msgs::Pose last_pose;
       
        Feedback feedback_;
        Result result_;
        bool success, executing;
    public:
        FollowMultiDofJointTrajectoryAction(std::string name) : action_name(name), 
            server_(nh_,name,boost::bind(&FollowMultiDofJointTrajectoryAction::executeCB,this,_1),false),
            orientation_client_("/action/pose",true){
                orientation_client_.waitForServer();
                empty.linear.x = 0;empty.linear.y = 0; empty.linear.z = 0;
                empty.angular.x = 0;empty.angular.y = 0;empty.angular.z = 0;
                vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",10);
                pose_sub = nh_.subscribe<nav_msgs::Odometry>("/ground_truth/state",10,&FollowMultiDofJointTrajectoryAction::poseCallback,this);
                ros::ServiceClient enable_motors = nh_.serviceClient<hector_uav_msgs::EnableMotors>("/enable_motors");
                hector_uav_msgs::EnableMotors srv;
                srv.request.enable = true;
                if(enable_motors.call(srv)){
                    if(srv.response.success)
                        ROS_INFO("Motors are enabled");
                }
                success = true;
                executing = false;
                server_.start();
            
            }
        void executeCB(const hector_moveit_actions::FollowMultiDofJointTrajectoryGoalConstPtr &goal){
            executing = true;
            trajectory = goal->trajectory;
            ROS_INFO_STREAM("Executing trajectory!");
            for(int i=0; i<trajectory.points.size()-1; i++){
                if(server_.isPreemptRequested() || !ros::ok()){
                    ROS_INFO("Preempt requested");
                    this->success = false;
                    executing = false;
                    break;
                }
                trajectory_msgs::MultiDOFJointTrajectoryPoint traj_point = trajectory.points[i+1];
                geometry_msgs::Twist vel_msg;
                geometry_msgs::Transform curr_t = traj_point.transforms[0];
                last_pose.position.x=curr_t.translation.x;
                last_pose.position.y=curr_t.translation.y;
                last_pose.position.z=curr_t.translation.z;
                last_pose.orientation=curr_t.rotation;
                feedback_.current_pose = last_pose;
                ros::spinOnce();
                ros::Duration(0.05).sleep();
                double goalx = curr_t.translation.x;
                double goaly = curr_t.translation.y;
                double goalz = curr_t.translation.z;
                double diffx = goalx - last_pose.position.x;
                double diffy = goaly - last_pose.position.y;
                double diffz = goalz - last_pose.position.z;

                double step_angle = atan2(diffy,diffx);
                tf::Quaternion q;
                quaternionMsgToTF(last_pose.orientation,q);
                tf::Matrix3x3 m(q);
                double tmp,heading;
                m.getRPY(tmp,tmp,heading);

                ROS_INFO("Diffz: %lf",diffz);
                ROS_INFO("Step angle: %lf, heading: %lf",step_angle,heading);
                ros::Rate r(4);
                if(((fabs(diffx)>0.01 || fabs(diffy)>0.01) && fabs(step_angle-heading) > 0.3)){
                    hector_uav_msgs::PoseGoal goal;
                    ROS_INFO("Adjust orientation");
                    geometry_msgs::Pose p;
                    p.position.x = goalx;
                    p.position.y = goaly;
                    p.position.z = goalz;
                    tf::Quaternion q;
                    if(fabs(diffx)>0.01 || fabs(diffy)>0.01)
                        q = tf::createQuaternionFromYaw(step_angle);
                    else
                        q = tf::createQuaternionFromYaw(heading);    
                    p.orientation.x = q.x();
                    p.orientation.y = q.y();
                    p.orientation.z = q.z();
                    p.orientation.w = q.w();
                    server_.publishFeedback(feedback_);
                    goal.target_pose.pose = p;
                    goal.target_pose.header.frame_id="world";
                    
                    orientation_client_.waitForServer();
                    orientation_client_.sendGoal(goal,actionlib::SimpleActionClient<hector_uav_msgs::PoseAction>::SimpleDoneCallback(),
                            actionlib::SimpleActionClient<hector_uav_msgs::PoseAction>::SimpleActiveCallback(),
                            boost::bind(&FollowMultiDofJointTrajectoryAction::actionCallback,this,_1,p));
                    orientation_client_.waitForResult();
                    continue;
                }
                while(fabs(diffz)>0.08){
                    vel_msg.linear.x = 0;
                    vel_msg.linear.z = diffz * 3;
                    vel_pub.publish(vel_msg);
                    ros::spinOnce();
                    r.sleep();
                    diffz = goalz - last_pose.position.z;
                    server_.publishFeedback(feedback_);
                }
                

                
                
                double latched_distance = sqrt(pow(diffx,2) + pow(diffy,2));
                double distance = latched_distance;
                vel_msg.linear.y = 0; vel_msg.linear.x= MAX_SPEED,vel_msg.linear.z = 0;
                if(i>trajectory.points.size()-3) // Slow down at final waypoints.
                        vel_msg.linear.x /= 3;
                ROS_INFO("Computed distance: %lf",distance);
                while(distance > 0.4*MAX_SPEED){
                    
                    vel_pub.publish(vel_msg);
                    ros::spinOnce();
                    r.sleep();
                    distance = sqrt(pow(goalx - last_pose.position.x,2) + pow(goaly - last_pose.position.y,2));
                    ROS_INFO("Distance to goal: %lf",distance);
                    server_.publishFeedback(feedback_);
                    if(distance < latched_distance) latched_distance = distance;
                    if(distance > latched_distance){
                        hector_uav_msgs::PoseGoal goal;
                        geometry_msgs::Pose p;
                        p.position.x = goalx;
                        p.position.y = goaly;
                        p.position.z = goalz;
                        tf::Quaternion q = tf::createQuaternionFromYaw(step_angle);
                        p.orientation.x = q.x();
                        p.orientation.y = q.y();
                        p.orientation.z = q.z();
                        p.orientation.w = q.w();
                        server_.publishFeedback(feedback_);
                        goal.target_pose.pose = p;
                        goal.target_pose.header.frame_id="world";
                        
                        orientation_client_.waitForServer();
                        orientation_client_.sendGoal(goal,actionlib::SimpleActionClient<hector_uav_msgs::PoseAction>::SimpleDoneCallback(),
                                actionlib::SimpleActionClient<hector_uav_msgs::PoseAction>::SimpleActiveCallback(),
                                boost::bind(&FollowMultiDofJointTrajectoryAction::actionCallback,this,_1,p));
                        orientation_client_.waitForResult();
                        break;
                    }
                }
                
            }
            executing = false;
            if(!this->success){
                result_.error_code = Result::PATH_TOLERANCE_VIOLATED;
                server_.setPreempted(result_);
                return ;
            }
            ROS_INFO_STREAM("Executed trajectory!");
            result_.error_code = Result::SUCCESSFUL;
            server_.setSucceeded(result_);
           
        }
        void idle(){
            while(ros::ok()){
                if(!executing)
                    vel_pub.publish(empty);
                ros::spinOnce();
                ros::Duration(0.25).sleep();
            }
        }
        void actionCallback(const hector_uav_msgs::PoseFeedbackConstPtr& feedback,geometry_msgs::Pose& p){
            double euler_distance = pow(p.position.x - feedback->current_pose.pose.position.x,2) + pow(p.position.y - feedback->current_pose.pose.position.y,2)
                                    + pow(p.position.z - feedback->current_pose.pose.position.z,2);
            euler_distance = sqrt(euler_distance);
            if(euler_distance < 0.15)
                orientation_client_.cancelGoal();
        }
        void poseCallback(const nav_msgs::Odometry::ConstPtr & msg)
        {
            last_pose = msg->pose.pose;
        }

};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "follow_multi_dof_joint_trajectory_action_server");
  FollowMultiDofJointTrajectoryAction controller("/follow_multi_dof_joint_trajectory");
  controller.idle();
  //ros::spin();
  return 0;
}