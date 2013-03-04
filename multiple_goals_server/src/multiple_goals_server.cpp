/*
 * Copyright (c) 2012, Stefano Rosa, Luca Carlone
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <boost/thread.hpp>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
std::vector<geometry_msgs::PoseWithCovarianceStamped> goals;
int goalIndex = 0;
boost::mutex m_mutex;

void goalReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
void clearGoals();


void spinThread(){
  ros::spin();
}

void goalReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
        boost::mutex::scoped_lock l(m_mutex);

	goals.push_back(*msg);	
	for(int i=0;i< goals.size();i++)
		ROS_INFO("Received goal %d: %f %f",i, goals[i].pose.pose.position.x,goals[i].pose.pose.position.y);
}

void clearGoals()
{
 	ROS_WARN("Clear goals");
 	goals.clear();
	
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals started.");

  ros::NodeHandle n;

  ros::Subscriber global_loc_sub_ = n.subscribe("addgoal", 2, &goalReceived);
  
  boost::thread spin_thread = boost::thread(boost::bind(&spinThread));

  MoveBaseClient ac("move_base");

  //give some time for connections to register
  sleep(2.0);

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";

  while(1)
  {
	  ROS_INFO("Inside main loop");
	  if(goals.size()>0)
	  {
		geometry_msgs::PoseWithCovarianceStamped nextGoal;
		// use mutex!	
		  ROS_INFO("Sending goal #%d", goalIndex);
		  {
		    boost::mutex::scoped_lock l(m_mutex);
		    nextGoal = goals[0];
		    goals.erase(goals.begin());
		  } 
		  goal.target_pose.header.stamp = ros::Time::now();
		  goal.target_pose.pose.position.x = nextGoal.pose.pose.position.x;
		  goal.target_pose.pose.position.y = nextGoal.pose.pose.position.y;
		  goal.target_pose.pose.orientation = nextGoal.pose.pose.orientation;

		  ac.sendGoal(goal);
		  ac.waitForResult(ros::Duration(100.0));
		  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		    ROS_INFO("Hooray, goal #%d reached",goalIndex);
		  else
		    ROS_INFO("The base failed to reach the goal for some reason");
	  }
	  sleep(2.0);
  }
  return 0;
}
