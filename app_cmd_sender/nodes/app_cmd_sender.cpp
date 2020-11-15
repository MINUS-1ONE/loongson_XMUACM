/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include "autoware_msgs/VehicleCmd.h"
#include "autoware_msgs/Gear.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <string>
#include <cstdio>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <thread>
#include "httplib.h"
#include "json.hpp"
#include <opencv2/opencv.hpp>

using namespace httplib;
using json = nlohmann::json;

struct point
{
    float x,y;
    float operator*(const point& pt)
    {
        return pt.x*this->x+pt.y*this->y;
    }
    point operator*(const float& t)
    {
        return {t*this->x,t*this->y};
    }
};

bool is_planned = false;
json plannedline = json::array();
json currentpose;
std::vector<point> centerpt;
std::string pwd = "/home/nuc/autoware.ai/src/autoware/utilities/app_cmd_sender";
geometry_msgs::PoseWithCovarianceStamped init_pose;

void centerlineCallback(const visualization_msgs::MarkerArray& ary)
{
    centerpt.clear();
    for(auto pt: ary.markers[0].points)
    {
        centerpt.push_back({pt.x,pt.y});
    }
    ROS_WARN("IM centerline, SIZE=%d",  ary.markers[0].points.size());
    return;
}

void planlineCallback(const visualization_msgs::MarkerArray& ary)
{
    if(!is_planned)
    {
        std::cout << ary.markers[0].points.size();
        ROS_WARN("IM planline, SIZE=%d",  ary.markers[0].points.size());
        plannedline.clear();
        for(auto pt: ary.markers[0].points)
        {
            plannedline.push_back(std::vector<float>{pt.x,pt.y});
        }
        
        is_planned = true;
    }
    return;
}

bool ishere(const point& pt1,const point& pt2,const point& pt)
{
    point v1{pt2.x - pt1.x, pt2.y - pt1.y}, v2{pt2.y - pt1.y, pt1.x - pt2.x};
    point r = {pt.x - pt1.x, pt.y - pt1.y};

    float droad = 10;

    return std::abs(v1*r)<std::abs(v1*v1) && std::abs(v2*r) < std::sqrt(v2*v2)*droad/2;
}

void currentposeCallback(const geometry_msgs::PoseStamped& odm)
{
    if(centerpt.empty()) return;

    point pt{odm.pose.position.x,odm.pose.position.y};

    currentpose = std::vector<float>{pt.x,pt.y};

    size_t i = 0;
    for(; i != centerpt.size() - 1; ++i)
    {
        if(ishere(centerpt[i],centerpt[i+1],pt))
            break;
    }
    ///////////////////INIT POSE//////////////////////////////
    
    init_pose.header.stamp = ros::Time::now();

    point dir = {centerpt[i+1].x-centerpt[i].x,centerpt[i+1].y-centerpt[i].y};
    float dirnorm = std::sqrt(dir*dir);
    float distance = 2;
    init_pose.pose.pose.position.x = centerpt[i].x-dir.x*distance/dirnorm;
    init_pose.pose.pose.position.y = centerpt[i].y-dir.y*distance/dirnorm;

    float theta = std::atan2(dir.y,dir.x);
    init_pose.pose.pose.orientation.z = std::sin(theta/2);
    init_pose.pose.pose.orientation.w = std::cos(theta/2);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "app_cmd_sender");
    ros::NodeHandle nh;

    Server svr;

    std::cout << "vehicle sender" << std::endl;
    ros::Subscriber sub1 = nh.subscribe("/vector_map_center_lines_rviz", 1, centerlineCallback);
    ros::Subscriber sub2 = nh.subscribe("/global_waypoints_rviz", 1, planlineCallback);
    ros::Subscriber sub3 = nh.subscribe("/ndt_pose", 1, currentposeCallback);
    //ros::Subscriber sub2 = nh.subscribe("/current_velocity", 1, callbackFromCurrentVelocity);
    ros::Publisher pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    ros::Publisher pub_init = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/start", 1);
    ros::Publisher pub_vehicle_cmd = nh.advertise<std_msgs::String>("/app_vehicle_cmd",1);
    ///////////////////INIT POSE//////////////////////////////
    init_pose.header.frame_id = "world";
    init_pose.pose.covariance[6 * 0 + 0] = 0.5 * 0.5;
    init_pose.pose.covariance[6 * 1 + 1] = 0.5 * 0.5;
    init_pose.pose.covariance[6 * 5 + 5] = M_PI / 12.0 * M_PI / 12.0;

    ///////////////////GOAL POSE//////////////////////////////
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "world";
    goal_pose.header.stamp = ros::Time::now();
    goal_pose.pose.position.x = -20.7433452606;
    goal_pose.pose.position.y = -2.35068130493;
    goal_pose.pose.orientation.z = 0.569915383005;
    goal_pose.pose.orientation.w = 0.821703387004;

    svr.Post("/plan", [&](const Request &req, Response &res)
    {
        std::cout << "im here" << std::endl;
        std::cout << req.get_param_value("init").c_str() << std::endl;
        pub_init.publish(init_pose);
        pub_goal.publish(goal_pose);
        is_planned = false;

        res.set_content(plannedline.dump(), "text/plain");
    });
    svr.Post("/drive", [&](const Request &req, Response &res)
    {
        std::cout << "drive now" << std::endl;
        if(req.body == "drive")
        {
            std_msgs::String msg;
            msg.data = "drive";
            pub_vehicle_cmd.publish(msg);
        }
        else if(req.body == "stop")
        {
            std_msgs::String msg;
            msg.data = "stop";
            pub_vehicle_cmd.publish(msg);
        }
    });
    svr.Get("/currentpose", [&](const Request &req, Response &res)
    {
        res.set_content(currentpose.dump(), "text/plain");
    });
    svr.Post("/multipart", [&](const Request &req, Response &res)
    {
        const MultipartFormData &file = req.get_file_value("uploadFace");

        file.content.data();
        file.content.length();

        cv::Mat rawData(1,file.content.length(), CV_8UC1, (void*)file.content.data());
        cv::Mat face;
        cv::imdecode(rawData,CV_LOAD_IMAGE_COLOR,&face);
        
        cv::imshow("X",face);
        cv::waitKey(0);
    });
    bool ret = svr.set_mount_point("/RegisteredFace",(pwd+"/RegisteredFace").c_str());
    if(!ret)
    {
        ROS_WARN((pwd+"/RegisteredFace"+"not ready").c_str());
    }
    svr.Get("/hello",[](const Request & /*req*/, Response &res) {
        const char *fmt = "Hello loongix";
        char buf[BUFSIZ];
        snprintf(buf, sizeof(buf), fmt, res.status);
        res.set_content(buf, "text/html");
    });

    std::thread tsvr([&]()
    {
        svr.listen("::", 2333);
    });
    std::thread tros([&]()
    {
        ros::spin();
    });

    tsvr.join();
    tros.join();

    return 0;
}
