/*
Created on Thurs June 14 10:20 2019

@author: Cong Liu

 Software License Agreement (BSD License)

 Copyright (c) 2018, Dobot Co., Ltd.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the copyright holders nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
// author: Cong Liu

#include <magician_background/magician_teleop_background.h>

namespace magician_background {

TeleopBackground::TeleopBackground(moveit::planning_interface::MoveGroupInterface *group, std::string action_name,
                                   planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor):
                                   group_(group), planning_scene_monitor_(planning_scene_monitor),
                                   local_nh_("~"), action_client_(action_name, true)
{
    goal_.trajectory.joint_names=group_->getActiveJoints();
    goal_.trajectory.header.stamp.sec=0;
    goal_.trajectory.header.stamp.nsec=0;

    joint_teleop_server_=local_nh_.advertiseService("joint_teleop", &TeleopBackground::jointTeleop_cb, this);
//    cart_teleop_server_=local_nh_.advertiseService("cart_teleop", &TeleopBackground::cartTeleop_cb, this);
//    home_teleop_server_=local_nh_.advertiseService("home_teleop", &TeleopBackground::homeTeleop_cb, this);
//    teleop_stop_server_=local_nh_.advertiseService("stop_teleop", &TeleopBackground::teleopStop_cb, this);

    // parameter for teleop
    joint_speed_limit_=1.57;
    joint_speed_default_=0.78;
    cart_duration_default_=0.04;
    resolution_angle_=0.02;
    resolution_linear_=0.005;

    // active joints
    size_t active_joints[4]={0, 1, 3, 5};
    active_joints_=std::vector<size_t>(active_joints, active_joints+4);
}

TeleopBackground::~TeleopBackground()
{

}

bool TeleopBackground::jointTeleop_cb(magician_msgs::SetInt16::Request &req, magician_msgs::SetInt16::Response &resp)
{
    if(req.data==0 || abs(req.data)>goal_.trajectory.joint_names.size())
    {
        resp.success=false;
        resp.message="wrong joint teleop data";
        return true;
    }

    size_t active_joint_num=abs(req.data)-1;
    size_t joint_num=active_joints_[active_joint_num];
    std::vector<double> position_current=group_->getCurrentJointValues();
    std::vector<double> position_goal=position_current;
    double joint_current_position=position_current[joint_num];
    std::string direction=goal_.trajectory.joint_names[active_joint_num];

    for(size_t i=0; i<position_current.size(); i++)
    {
        std::cout<<"joint"<<i<<": "<<position_current[i]<<std::endl;
    }
}

}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"magician_background", ros::init_options::AnonymousName);

    ros::CallbackQueue move_group_cb_queue;

    std::string move_group_name="magician_arm";
    std::string move_group_desc="robot_description";
    ros::NodeHandle move_group_nh;
    move_group_nh.setCallbackQueue(&move_group_cb_queue);

    moveit::planning_interface::MoveGroupInterface::Options move_group_options(move_group_name, move_group_desc, move_group_nh);

    moveit::planning_interface::MoveGroupInterface move_group(move_group_options);

    ros::AsyncSpinner move_group_spinner(1, &move_group_cb_queue);
    move_group_spinner.start();

    move_group.startStateMonitor();

    move_group.getCurrentJointValues();

    ros::AsyncSpinner common_spinner(1);
    common_spinner.start();

    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startStateMonitor();
    planning_scene_monitor->startWorldGeometryMonitor();

    magician_background::TeleopBackground teleop_background(&move_group, "magician_arm_controller/follow_joint_trajectory", planning_scene_monitor);

    ros::waitForShutdown();

    return 0;

}
