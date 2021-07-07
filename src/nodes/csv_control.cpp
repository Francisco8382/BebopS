/*
 * Copyright 2021 Francisco Javier Torres Maldonado
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <thread>
#include <chrono>

#include <fstream>
#include <iostream>
#include <string>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include "bebop_simulator/Matrix3x3.h"
#include "bebop_simulator/Quaternion.h"
#include "bebop_simulator/parameters_ros.h" 

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <math.h>

#include <sys/stat.h>
#include <sys/types.h>

#include "bebop_simulator/PosController.h"
#include "bebop_simulator/AttitudeController.h"
#include "bebop_simulator/Control.h"

namespace mav_msgs {
  inline double secsFromHeaderMsg(const std_msgs::Header& msg) {
    return (double) (msg.stamp.sec) + (double) (msg.stamp.nsec)/1.0e9;
  }
}

namespace bebop_simulator {

    class CSV_Control{
        public:
            CSV_Control(std::string Topic, std::string Path, double TiempoMargen);
            ~CSV_Control();
            
        private:
            double _Tiempo;
            double _TiempoMargen;
            double _TiempoInicio;

            ros::NodeHandle nh;
            ros::Subscriber control_sub_;
            ros::Subscriber Begin_;
            ros::Subscriber End_;
            
            bool Trigger = false;
            std::string _Path;
            std::string _Topic;
            std::ofstream _csvFile;

            void BeginCallback(const std_msgs::Empty::ConstPtr& empty_msg);
            void StopCallback(const std_msgs::Empty::ConstPtr& empty_msg);
            void ControlCallback(const bebop_simulator::Control& control_msg);
    };

    CSV_Control::CSV_Control(std::string Topic, std::string Path, double TiempoMargen)
    : _Path(Path),
      _Topic(Topic),
      _TiempoMargen(TiempoMargen),
      _TiempoInicio(0.0) {
        _csvFile.open(_Path);
        _csvFile << "Tiempo,u_x,u_y,u_z,u_T,u_phi,u_theta,u_psi,omega_1,omega_2,omega_3,omega_4\n";
        control_sub_ = nh.subscribe(_Topic, 
          1, &CSV_Control::ControlCallback, this, ros::TransportHints().tcpNoDelay(true));
        Begin_ = nh.subscribe<std_msgs::Empty>("/csv/begin", 1, &CSV_Control::BeginCallback, this);
        End_ = nh.subscribe<std_msgs::Empty>("/csv/end", 1, &CSV_Control::StopCallback, this);
        
    }
    
    CSV_Control::~CSV_Control() {}

    void CSV_Control::BeginCallback(const std_msgs::Empty::ConstPtr& empty_msg) {
      if (!Trigger){
        ROS_INFO("Leyendo mensajes de topico: %s",_Topic.c_str());
      }
      Trigger = true;
    }

    void CSV_Control::StopCallback(const std_msgs::Empty::ConstPtr& empty_msg) {
      Trigger = false;
      _csvFile.close();
      ROS_INFO("Escritura de CSV finalizada para el topico: %s",_Topic.c_str());
      ros::shutdown();
    }

    void CSV_Control::ControlCallback(const bebop_simulator::Control& control_msg) {
      if (Trigger) {
        ros::param::get("/TiempoInicio", _TiempoInicio);
        _Tiempo = mav_msgs::secsFromHeaderMsg(control_msg.header) - _TiempoInicio;
        _csvFile  << _Tiempo << "," 
                  << control_msg.posCont.u_x << ","
                  << control_msg.posCont.u_y << ","
                  << control_msg.posCont.u_z << ","
                  << control_msg.posCont.u_T << ","
                  << control_msg.attCont.u_phi << ","
                  << control_msg.attCont.u_theta << ","
                  << control_msg.attCont.u_psi << ","
                  << control_msg.rotorVel.angular_velocities[0] << ","
                  << control_msg.rotorVel.angular_velocities[1] << ","
                  << control_msg.rotorVel.angular_velocities[2] << ","
                  << control_msg.rotorVel.angular_velocities[3] << "\n";
      }
    }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "csv_control");
  ros::NodeHandle nh;

  std::string Topic, Path, DateAndTime, Name, FullPath, File;
  double TiempoMargen;

  ros::param::get("~Topico", Topic);
  ros::param::get("~Ruta", Path);
  ros::param::get("/Subcarpeta", DateAndTime);
  ros::param::get("~TiempoMargen", TiempoMargen);
  ros::param::get("~Nombre", Name);

  DateAndTime.pop_back();
  FullPath = Path + DateAndTime;
  mkdir(FullPath.c_str(), 0777);

  File = FullPath + "/" + Name;

  bebop_simulator::CSV_Control csv_control_(Topic, File, TiempoMargen);

  ros::spin();
  
  return 0;

}

