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
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <cstdio>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>

#include "control.h"
#include "pid.h"

Control* g_pctrl = nullptr;
PID* g_pid;
bool g_canimove = false;
float current_linear_velocity_;

struct CommandData
{
    double linear_x;
    double angular_z;
    int modeValue;
    int gearValue;
    int lampValue;
    int accellValue;
    int brakeValue;
    int steerValue;
    double linear_velocity;
    double steering_angle;

    void reset();
};

// enum class ZMPGear
// {
//   Drive = 1,
//   Reverse = 2,
//   Low = 3,
//   Neutral = 4,
//   Park = 5,
// };

enum class GEAR
{
    PARK = 1,
    REVERSE = 2,
    NEUTRAL = 3,
    DRIVE = 4
};

void CommandData::reset()
{
    linear_x = 0;
    angular_z = 0;
    modeValue = 0;
    gearValue = 0;
    lampValue = 0;
    accellValue = 0;
    brakeValue = 0;
    steerValue = 0;
    linear_velocity = -1;
    steering_angle = 0;
}

static CommandData command_data;

static void vehicleCmdCallback(const autoware_msgs::VehicleCmd &msg)
{
    if(!g_canimove)return;
    //command_data.linear_x = msg.twist_cmd.twist.linear.x;
    //command_data.angular_z = msg.twist_cmd.twist.angular.z;
    //command_data.modeValue = msg.mode;
    g_pctrl->brake(msg.brake_cmd.brake);
    //command_data.steerValue = msg.steer_cmd.steer;
    //g_pctrl->steer(msg.steer_cmd.steer);
    
    double steer = static_cast<double>(msg.ctrl_cmd.steering_angle)*985.;
    if(steer>418)steer = 418;
    if(steer<-418)steer = -418;
    
    //ROS_WARN("HERE %d",int(float(steer)));
    g_pctrl->steer(static_cast<int>(steer));
    if (msg.gear_cmd.gear == 1/*autoware_msgs::Gear::DRIVE*/)
    {
        command_data.gearValue = static_cast<int>(GEAR::DRIVE);
        g_pctrl->gear(command_data.gearValue);
    }
    else if (msg.gear_cmd.gear == autoware_msgs::Gear::REVERSE)
    {
        command_data.gearValue = static_cast<int>(GEAR::REVERSE);
        g_pctrl->gear(command_data.gearValue);
    }
    // else if (msg.gear_cmd.gear == autoware_msgs::Gear::LOW)
    // {
    //   command_data.gearValue = static_cast<int>(ZMPGear::Low);
    // }
    else if (msg.gear_cmd.gear == autoware_msgs::Gear::NEUTRAL)
    {
        command_data.gearValue = static_cast<int>(GEAR::NEUTRAL);
        g_pctrl->gear(command_data.gearValue);
    }
    else if (msg.gear_cmd.gear == autoware_msgs::Gear::PARK)
    {
        command_data.gearValue = static_cast<int>(GEAR::PARK);
        g_pctrl->gear(command_data.gearValue);
    }
    command_data.gearValue = static_cast<int>(GEAR::DRIVE);
    g_pctrl->gear(command_data.gearValue);
    // if (msg.lamp_cmd.l == 0 && msg.lamp_cmd.r == 0)
    // {
    //   command_data.lampValue = 0;
    // }
    // else if (msg.lamp_cmd.l == 1 && msg.lamp_cmd.r == 0)
    // {
    //   command_data.lampValue = 1;
    // }
    // else if (msg.lamp_cmd.l == 0 && msg.lamp_cmd.r == 1)
    // {
    //   command_data.lampValue = 2;
    // }
    // else if (msg.lamp_cmd.l == 1 && msg.lamp_cmd.r == 1)
    // {
    //   command_data.lampValue = 3;
    // }
    command_data.accellValue = msg.accel_cmd.accel;
    //g_pctrl->throttle(msg.accel_cmd.accel);
    //command_data.brakeValue = msg.brake_cmd.brake;

    //command_data.linear_velocity = msg.ctrl_cmd.linear_velocity;
    //command_data.steering_angle = msg.ctrl_cmd.steering_angle;
    
    g_pid->change(msg.accel_cmd.accel);
    
    double v = static_cast<double>(msg.ctrl_cmd.linear_velocity);
    static float throttle = 0;
    //throttle += g_pid->position(v-current_linear_velocity_);
    throttle = std::min(std::max(g_pid->position(v-current_linear_velocity_),0.f),40.f);

    g_pctrl->throttle(static_cast<int>(throttle));
    ROS_WARN("TURN = %d, V = %lf",static_cast<int>(steer), v);
    /*
    I CHANGED HERE TO CONTROL THROTTLE MANUALLY BY RUNTIME MANAGER VIA INTERFACE
    */
    //g_pctrl->throttle(msg.accel_cmd.accel);
}

void callbackFromCurrentVelocity(const geometry_msgs::TwistStampedConstPtr& msg)
{
  current_linear_velocity_ = static_cast<float>(msg->twist.linear.x);
}

void callbackAppCmd(const std_msgs::String::ConstPtr& msg)
{
    ROS_WARN("I RECEIVE CMD!");

    if(msg->data == "drive")
    {
        //static Control ctrl;
        //g_pctrl = &ctrl;
        if(g_pctrl)delete g_pctrl;
        g_pctrl = new Control;

        g_canimove = true;
    }else
    {
        delete g_pctrl;
        g_pctrl = nullptr;
        g_canimove = false;
    }
}

static void *sendCommand(void *arg)
{
    int *client_sockp = static_cast<int *>(arg);
    int client_sock = *client_sockp;
    delete client_sockp;

    std::ostringstream oss;
    oss << command_data.linear_x << ",";
    oss << command_data.angular_z << ",";
    oss << command_data.modeValue << ",";
    oss << command_data.gearValue << ",";
    oss << command_data.accellValue << ",";
    oss << command_data.brakeValue << ",";
    oss << command_data.steerValue << ",";
    oss << command_data.linear_velocity << ",";
    oss << command_data.steering_angle << ",";
    oss << command_data.lampValue;

    std::string cmd(oss.str());
    std::cout << "im here!!!" << std::endl;
    ssize_t n = write(client_sock, cmd.c_str(), cmd.size());
    if (n < 0)
    {
        std::perror("write");
        return nullptr;
    }

    if (close(client_sock) == -1)
    {
        std::perror("close");
        return nullptr;
    }

    std::cout << "cmd: " << cmd << ", size: " << cmd.size() << std::endl;
    return nullptr;
}

static void *receiverCaller(void *unused)
{
    constexpr int listen_port = 10001;

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1)
    {
        std::perror("socket");
        return nullptr;
    }

    sockaddr_in addr;
    sockaddr_in client;
    socklen_t len = sizeof(client);

    std::memset(&addr, 0, sizeof(sockaddr_in));
    addr.sin_family = PF_INET;
    addr.sin_port = htons(listen_port);
    addr.sin_addr.s_addr = INADDR_ANY;

    int ret = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
    if (ret == -1)
    {
        std::perror("bind");
        goto error;
    }

    ret = listen(sock, 20);
    if (ret == -1)
    {
        std::perror("listen");
        goto error;
    }

    while (true)
    {
        // get connect to android
        std::cout << "Waiting access..." << std::endl;

        int *client_sock = new int();
        *client_sock = accept(sock, reinterpret_cast<sockaddr *>(&client), &len);
        if (*client_sock == -1)
        {
            std::perror("accept");
            break;
        }

        std::cout << "get connect." << std::endl;

        pthread_t th;
        if (pthread_create(&th, nullptr, sendCommand, static_cast<void *>(client_sock)) != 0)
        {
            std::perror("pthread_create");
            break;
        }

        if (pthread_detach(th) != 0)
        {
            std::perror("pthread_detach");
            break;
        }
    }

error:
    close(sock);
    return nullptr;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehicle_sender");
    ros::NodeHandle nh;
    ROS_INFO("here : INIT");

    PID pid(3);
    g_pid = &pid;
    
    std::cout << "vehicle sender" << std::endl;
    ros::Subscriber sub1 = nh.subscribe("/vehicle_cmd", 1, vehicleCmdCallback);

    ros::Subscriber sub2 = nh.subscribe("/current_velocity", 1, callbackFromCurrentVelocity);

    ros::Subscriber sub3 = nh.subscribe("/app_vehicle_cmd", 1, callbackAppCmd);

    command_data.reset();

    // pthread_t th;
    // if (pthread_create(&th, nullptr, receiverCaller, nullptr) != 0)
    // {
    //     std::perror("pthread_create");
    //     std::exit(1);
    // }

    // if (pthread_detach(th) != 0)
    // {
    //     std::perror("pthread_detach");
    //     std::exit(1);
    // }

    ros::spin();
    return 0;
}
