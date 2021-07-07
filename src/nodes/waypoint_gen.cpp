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

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include "bebop_simulator/Matrix3x3.h"
#include "bebop_simulator/Quaternion.h"
#include "bebop_simulator/Vector3.h"
#include "bebop_simulator_msgs/default_topics.h"

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <math.h>
#include "bebop_simulator/ReferenceAngles.h"
#include "bebop_simulator/PosController.h"
#include "bebop_simulator/AttitudeController.h"
#include "bebop_simulator/Control.h"

static const float DEG_2_RAD = M_PI / 180.0;

namespace mav_msgs {
  inline Eigen::Vector4d vector4FromQuaternionMsg(const geometry_msgs::Quaternion& msg) {
    return Eigen::Vector4d(msg.x, msg.y, msg.z, msg.w);
  }
  inline double secsFromHeaderMsg(const std_msgs::Header& msg) {
    return (double) (msg.stamp.sec) + (double) (msg.stamp.nsec)/1.0e9;
  }
}

namespace bebop_simulator {

    class Waypoint{
        public:
            Waypoint(double InitialTime, double PathTime, double DeltaTime, double MarginTime, Eigen::Vector3d InitialPosition, Eigen::Vector3d PathDistance, double Gamma, bool verbose);
            ~Waypoint();
            
        private:
            double _InitialTime;
            double _PathTime;
            double _DeltaTime;
            double _MarginTime;
            double _Gamma;
            bool _verbose;
            Eigen::Vector3d _InitialPosition;
            Eigen::Vector3d _PathDistance;
            nav_msgs::Odometry odometry_;
            nav_msgs::Odometry odometry_gamma_;
            Eigen::Vector3d _Position;
            Eigen::Vector3d _Velocity;
            std_msgs::Empty empty_;
            double _Yaw;
            double _V_Yaw;
            double _Last_Yaw;
            nav_msgs::Odometry odometry_drone_;
            nav_msgs::Odometry odometry_gt_;
            
            ros::NodeHandle _n1;
            ros::Timer _timer1;
            ros::NodeHandle _n2;
            ros::Timer _timer2;
            ros::NodeHandle _n3;
            ros::Timer _timer3;
            ros::NodeHandle nh;

            ros::Subscriber odometry_sub_;
            ros::Subscriber odometry_gt_sub_;
            ros::Subscriber ref_angles_sub;
            ros::Subscriber position_controller_sub;
            ros::Subscriber attitude_controller_sub;
            ros::Subscriber rotor_velocities_sub;
            ros::Publisher trajectory_pub_;
            ros::Publisher trajectory_ref_pub_;
            ros::Publisher trajectory_gamma_pub_;
            ros::Publisher Begin_;
            ros::Publisher End_;
            ros::Publisher odometry_pub_;
            ros::Publisher odometry_gt_pub_;
            ros::Publisher Control_pub;

            bebop_simulator::Control control_msg;
            bebop_simulator::PosController _pos_controller_msg;
            bebop_simulator::AttitudeController _attitude_controller_msg;
            mav_msgs::Actuators _rotor_velocities_msg;

            trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
            bool Trigger = false;
            ros::Time TiempoInicio;
            double _roll, _pitch;

            void CallbackStart(const ros::TimerEvent& event);
            void CallbackPath(const ros::TimerEvent& event);
            void CallbackStop(const ros::TimerEvent& event);
            void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
            void OdometryGTCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
            void RefAnglesCallback(const bebop_simulator::ReferenceAngles& ref_angles_msg);
            void PosCallback(const bebop_simulator::PosController& pos_controller_msg);
            void AttCallback(const bebop_simulator::AttitudeController& att_controller_msg);
            void RotorVelCallback(const mav_msgs::Actuators& rotor_velocities_msg);
            Eigen::Vector3d Quat2RPY(Eigen::Vector4d Quaternion);
            Eigen::Vector4d RPY2Quat(Eigen::Vector3d RPY);
    };

    Waypoint::Waypoint(double InitialTime, double PathTime, double DeltaTime, double MarginTime, Eigen::Vector3d InitialPosition, Eigen::Vector3d PathDistance, double Gamma, bool verbose)
    : _InitialTime(InitialTime),
      _PathTime(PathTime),
      _DeltaTime(DeltaTime),
      _MarginTime(MarginTime),
      _Gamma(Gamma),
      _verbose(verbose),
      _InitialPosition(InitialPosition),
      _PathDistance(PathDistance),
      _Position(Eigen::Vector3d(0.0,0.0,0.0)),
      _Velocity(Eigen::Vector3d(0.0,0.0,0.0)),
      _Yaw(0.0),
      _V_Yaw(0.0),
      _Last_Yaw(0.0),
      _roll(0.0),
      _pitch(0.0) {
        odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 
          1, &Waypoint::OdometryCallback, this, ros::TransportHints().tcpNoDelay(true));
        odometry_gt_sub_ = nh.subscribe(bebop_simulator_msgs::default_topics::ODOMETRY_GT, 
          1, &Waypoint::OdometryGTCallback, this);
        trajectory_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
        trajectory_ref_pub_ = nh.advertise<nav_msgs::Odometry>("PositionReference", 1);
        trajectory_gamma_pub_ = nh.advertise<nav_msgs::Odometry>("PositionWithGamma", 1);
        odometry_pub_ = nh.advertise<nav_msgs::Odometry>("Odometry", 1);
        odometry_gt_pub_ = nh.advertise<nav_msgs::Odometry>("OdometryGT", 1);
        Begin_ = nh.advertise<std_msgs::Empty>("/csv/begin",1);
        End_ = nh.advertise<std_msgs::Empty>("/csv/end",1);
        Control_pub = nh.advertise<bebop_simulator::Control>("/Control", 1);
        ref_angles_sub = nh.subscribe("/ref_angles", 1, &Waypoint::RefAnglesCallback, this);
        position_controller_sub = nh.subscribe("/pos_controller", 1, &Waypoint::PosCallback, this);
        attitude_controller_sub = nh.subscribe("/attitude_controller", 1, &Waypoint::AttCallback, this);
        rotor_velocities_sub = nh.subscribe("/bebop/command/motors", 1, &Waypoint::RotorVelCallback, this);

        std_srvs::Empty srv;
        bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
        unsigned int i = 0;

        while (i <= 10 && !unpaused) {
          ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
          std::this_thread::sleep_for(std::chrono::seconds(1));
          unpaused = ros::service::call("/gazebo/unpause_physics", srv);
          ++i;
        }

        if (!unpaused) {
          ROS_FATAL("Could not wake up Gazebo.");
        }
        else {
          ROS_INFO("Unpaused the Gazebo simulation.");
        }

        double t_espera = 6.0;
        ros::Duration(t_espera).sleep();

        ROS_INFO("Comienza a publicar waypoint.");
        trajectory_msg.header.stamp = ros::Time::now();
        double Yaw = 0.0;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(_InitialPosition, Yaw, &trajectory_msg);
        if (_verbose) {
          ROS_INFO("Publicando waypoint en namespace %s: [%f, %f, %f].",       
            nh.getNamespace().c_str(),
            _InitialPosition.x(),
            _InitialPosition.y(),
            _InitialPosition.z());
        }
        trajectory_pub_.publish(trajectory_msg);

        ros::param::set("/theta_ref", 0);
        ros::param::set("/phi_ref", 0);

        _timer1 = _n1.createTimer(ros::Duration(_DeltaTime), &Waypoint::CallbackPath, this, false, true);
        _timer2 = _n2.createTimer(ros::Duration(_InitialTime), &Waypoint::CallbackStart, this, true, true);
        double TotalTime = _InitialTime + _PathTime + _MarginTime;
        _timer3 = _n3.createTimer(ros::Duration(TotalTime), &Waypoint::CallbackStop, this, false, true);
    }
    
    Waypoint::~Waypoint() {}

    void Waypoint::CallbackStart(const ros::TimerEvent& event) {
      Trigger = true;
      TiempoInicio = ros::Time::now();
      double TiempoSec = (double) (TiempoInicio.sec) + (double) (TiempoInicio.nsec)/1.0e9; 
      ros::param::set("/TiempoInicio", TiempoSec);
      Begin_.publish(empty_);
    }

    void Waypoint::CallbackStop(const ros::TimerEvent& event) {
      for (int i = 0; i < 50; i++){
        End_.publish(empty_);
        std::this_thread::sleep_for(std::chrono::microseconds(200));
      }
      ROS_INFO("W3 finaliza");
      ros::shutdown();
    }

    void Waypoint::CallbackPath(const ros::TimerEvent& event) {
      if (Trigger) {
        Begin_.publish(empty_);
        ros::Time TiempoActual = ros::Time::now();
        //ros::Time Tiempo = TiempoActual - TiempoInicio;
        double t = TiempoActual.toSec() - TiempoInicio.toSec() + _DeltaTime;
        if (t > _PathTime) {
          Trigger = false;
          return;
        }

        trajectory_msg.header.stamp = TiempoActual;
        double arg = 2*M_PI*t/_PathTime;
        double x = _PathDistance.x()*sin(arg) + _InitialPosition.x();
        double y = _PathDistance.y()*sin(2*arg) + _InitialPosition.y();
        double z = _PathDistance.z()*sin(arg) + _InitialPosition.z();

        double dot_x = _PathDistance.x()*2*M_PI/_PathTime*cos(arg);
        double dot_y = _PathDistance.y()*4*M_PI/_PathTime*cos(2*arg);
        double dot_z = _PathDistance.z()*2*M_PI/_PathTime*cos(arg);

        double ddot_x = -_PathDistance.x()*pow(2*M_PI/_PathTime,2)*sin(arg);
        double ddot_y = -_PathDistance.y()*pow(4*M_PI/_PathTime,2)*sin(2*arg);
        double ddot_z = -_PathDistance.z()*pow(2*M_PI/_PathTime,2)*sin(arg);
        
        double dddot_x = -_PathDistance.x()*pow(2*M_PI/_PathTime,3)*cos(arg);
        double dddot_y = -_PathDistance.y()*pow(4*M_PI/_PathTime,3)*cos(2*arg);
        double dddot_z = -_PathDistance.z()*pow(2*M_PI/_PathTime,3)*cos(arg);

        double yaw = atan2(dot_y,dot_x) + 1.5;
        double dot_yaw = ((dot_x*ddot_y - dot_y*ddot_x)/pow(dot_x,2))/(1+pow(dot_y/dot_x,2));

        while (abs(yaw - _Last_Yaw) > M_PI) {
          if ((yaw - _Last_Yaw) > 0){
            yaw -= 2*M_PI;
          }
          else {
            yaw += 2*M_PI;
          }
        }

        _Last_Yaw = yaw;

        double e_x = x - _Position.x();
        double e_y = y - _Position.y();
        double e_z = z - _Position.z();
        double e_yaw = yaw - _Yaw;

        double v_x = -_Gamma*e_x + dot_x;
        double v_y = -_Gamma*e_y + dot_y;
        double v_z = -_Gamma*e_z + dot_z;
        double v_yaw = -_Gamma*e_yaw + dot_yaw;

        double pos_x = x + _DeltaTime * v_x;
        double pos_y = y + _DeltaTime * v_y;
        double pos_z = z + _DeltaTime * v_z;
        double angle_yaw = yaw + _DeltaTime * v_yaw;

        // Posición a enviar
        Eigen::Vector3d desired_position(pos_x, pos_y, pos_z);
        double desired_yaw = angle_yaw;
        Eigen::Vector3d velocities(dot_x,dot_y,dot_z);
        Eigen::Vector3d accelerations(ddot_x,ddot_y,ddot_z);

        mav_msgs::EigenTrajectoryPoint point;
        point.position_W = desired_position;
        point.setFromYaw(desired_yaw);
        point.velocity_W = velocities;
        point.setFromYawRate(dot_yaw);
        point.acceleration_W = accelerations;
        
        mav_msgs::msgMultiDofJointTrajectoryFromEigen(point, &trajectory_msg);
        

        trajectory_pub_.publish(trajectory_msg);

        // Posición de referencia
        odometry_.header.stamp = TiempoActual;
        odometry_.pose.pose.position.x = x;
        odometry_.pose.pose.position.y = y;
        odometry_.pose.pose.position.z = z;

        Eigen::Vector4d Quaternion_ref = RPY2Quat(Eigen::Vector3d(_roll,_pitch,yaw));

        odometry_.pose.pose.orientation.x = Quaternion_ref.x();
        odometry_.pose.pose.orientation.y = Quaternion_ref.y();
        odometry_.pose.pose.orientation.z = Quaternion_ref.z();
        odometry_.pose.pose.orientation.w = Quaternion_ref.w();

        trajectory_ref_pub_.publish(odometry_);

        if (_verbose) {
          ROS_INFO("Publicando waypoint en namespace %s: [%f, %f, %f, %f].",       
                 nh.getNamespace().c_str(),
                 desired_position.x(),
                 desired_position.y(),
                 desired_position.z(),
                 desired_yaw);

          ROS_INFO("Publicando referencia en namespace %s: [%f, %f, %f, %f].",       
                 nh.getNamespace().c_str(),
                 odometry_.pose.pose.position.x,
                 odometry_.pose.pose.position.y,
                 odometry_.pose.pose.position.z,
                 yaw);
        }      

        // Posición de referencia con gamma
        odometry_gamma_.header.stamp = TiempoActual;
        odometry_gamma_.pose.pose.position.x = pos_x;
        odometry_gamma_.pose.pose.position.y = pos_y;
        odometry_gamma_.pose.pose.position.z = pos_z;
        
        Eigen::Vector4d Quaternion_gamma = RPY2Quat(Eigen::Vector3d(_roll,_pitch,angle_yaw));

        odometry_gamma_.pose.pose.orientation.x = Quaternion_gamma.x();
        odometry_gamma_.pose.pose.orientation.y = Quaternion_gamma.y();
        odometry_gamma_.pose.pose.orientation.z = Quaternion_gamma.z();
        odometry_gamma_.pose.pose.orientation.w = Quaternion_gamma.w();

        trajectory_gamma_pub_.publish(odometry_gamma_);

        odometry_drone_.header.stamp = TiempoActual;
        odometry_gt_.header.stamp = TiempoActual;
        odometry_pub_.publish(odometry_drone_);
        odometry_gt_pub_.publish(odometry_gt_);

        control_msg.header.stamp = TiempoActual;
        control_msg.posCont = _pos_controller_msg;
        control_msg.attCont = _attitude_controller_msg;
        control_msg.rotorVel = _rotor_velocities_msg;
        Control_pub.publish(control_msg);

      } 
      else {
        ros::Time TiempoActual = ros::Time::now();
        trajectory_msg.header.stamp = TiempoActual;
        double x = _InitialPosition.x();
        double y = _InitialPosition.y();
        double z = _InitialPosition.z();
        double yaw = atan2(2*_PathDistance.y(),_PathDistance.x()) + 1.5;
        double dot_yaw = 0.0;

        double dot_x = 0.0;
        double dot_y = 0.0;
        double dot_z = 0.0;

        double e_x = x - _Position.x();
        double e_y = y - _Position.y();
        double e_z = z - _Position.z();
        double e_yaw = yaw - _Yaw;

        double v_x = -_Gamma*e_x + dot_x;
        double v_y = -_Gamma*e_y + dot_y;
        double v_z = -_Gamma*e_z + dot_z;
        double v_yaw = -_Gamma*e_yaw + dot_yaw;

        double pos_x = x + _DeltaTime * v_x;
        double pos_y = y + _DeltaTime * v_y;
        double pos_z = z + _DeltaTime * v_z;
        double angle_yaw = yaw + _DeltaTime * v_yaw;

        // Posición a enviar
        Eigen::Vector3d desired_position(pos_x, pos_y, pos_z);
        double desired_yaw = angle_yaw;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

        trajectory_pub_.publish(trajectory_msg);

        // Posición de referencia
        odometry_.header.stamp = TiempoActual;
        odometry_.pose.pose.position.x = x;
        odometry_.pose.pose.position.y = y;
        odometry_.pose.pose.position.z = z;
        
        Eigen::Vector4d Quaternion_ref = RPY2Quat(Eigen::Vector3d(_roll,_pitch,yaw));

        odometry_.pose.pose.orientation.x = Quaternion_ref.x();
        odometry_.pose.pose.orientation.y = Quaternion_ref.y();
        odometry_.pose.pose.orientation.z = Quaternion_ref.z();
        odometry_.pose.pose.orientation.w = Quaternion_ref.w();

        trajectory_ref_pub_.publish(odometry_);

        // Posición de referencia con gamma
        odometry_gamma_.header.stamp = TiempoActual;
        odometry_gamma_.pose.pose.position.x = pos_x;
        odometry_gamma_.pose.pose.position.y = pos_y;
        odometry_gamma_.pose.pose.position.z = pos_z;
        
        Eigen::Vector4d Quaternion_gamma = RPY2Quat(Eigen::Vector3d(_roll,_pitch,angle_yaw));

        odometry_gamma_.pose.pose.orientation.x = Quaternion_gamma.x();
        odometry_gamma_.pose.pose.orientation.y = Quaternion_gamma.y();
        odometry_gamma_.pose.pose.orientation.z = Quaternion_gamma.z();
        odometry_gamma_.pose.pose.orientation.w = Quaternion_gamma.w();

        trajectory_gamma_pub_.publish(odometry_gamma_);

        odometry_drone_.header.stamp = TiempoActual;
        odometry_gt_.header.stamp = TiempoActual;
        odometry_pub_.publish(odometry_drone_);
        odometry_gt_pub_.publish(odometry_gt_);

        control_msg.header.stamp = TiempoActual;
        control_msg.posCont = _pos_controller_msg;
        control_msg.attCont = _attitude_controller_msg;
        control_msg.rotorVel = _rotor_velocities_msg;
        Control_pub.publish(control_msg);

      }
    }

    void Waypoint::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
      odometry_drone_.header = odometry_msg->header;
      odometry_drone_.child_frame_id = odometry_msg->child_frame_id;
      odometry_drone_.pose = odometry_msg->pose;
      odometry_drone_.twist = odometry_msg->twist;
      _Position = mav_msgs::vector3FromPointMsg(odometry_msg->pose.pose.position);
      _Velocity = mav_msgs::vector3FromMsg(odometry_msg->twist.twist.linear);
      Eigen::Vector4d _Quaternion = mav_msgs::vector4FromQuaternionMsg(odometry_msg->pose.pose.orientation);
      Eigen::Vector3d _AngularVelocity = mav_msgs::vector3FromMsg(odometry_msg->twist.twist.angular);
      Eigen::Vector3d _Orientation = Quat2RPY(_Quaternion);
      _Yaw = _Orientation.z();
      _V_Yaw = _AngularVelocity.z();
    }

    void Waypoint::OdometryGTCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
      odometry_gt_.header = odometry_msg->header;
      odometry_gt_.child_frame_id = odometry_msg->child_frame_id;
      odometry_gt_.pose = odometry_msg->pose;
      odometry_gt_.twist = odometry_msg->twist;
    }

    void Waypoint::RefAnglesCallback(const bebop_simulator::ReferenceAngles& ref_angles_msg) {
      _roll = ref_angles_msg.phi;
      _pitch = ref_angles_msg.theta;
    }

    Eigen::Vector3d Waypoint::Quat2RPY(Eigen::Vector4d Quaternion) {
      double x, y, z, w, roll, pitch, yaw;
      x = Quaternion.x();
      y = Quaternion.y();
      z = Quaternion.z();
      w = Quaternion.w();
      tf::Quaternion q(x, y, z, w);
      tf::Matrix3x3 m(q);
      m.getRPY(roll, pitch, yaw);
      return Eigen::Vector3d(roll, pitch, yaw);
    }

    Eigen::Vector4d Waypoint::RPY2Quat(Eigen::Vector3d RPY) {
      tfScalar roll, pitch, yaw;
      tf::Quaternion q;
      roll = RPY.x();
      pitch = RPY.y();
      yaw = RPY.z();
      tf::Matrix3x3 m;
      m.setEulerYPR(yaw, pitch, roll);
      m.getRotation(q);
      return Eigen::Vector4d(q.x(), q.y(), q.z(), q.w());
    }

    void Waypoint::PosCallback(const bebop_simulator::PosController& pos_controller_msg) {
      _pos_controller_msg = pos_controller_msg;
    }

    void Waypoint::AttCallback(const bebop_simulator::AttitudeController& att_controller_msg) {
      _attitude_controller_msg = att_controller_msg;
    }

    void Waypoint::RotorVelCallback(const mav_msgs::Actuators& rotor_velocities_msg) {
      _rotor_velocities_msg = rotor_velocities_msg;
    }

}

int main(int argc, char** argv){
  ros::init(argc, argv, "w3");
  ros::NodeHandle nh;

  double TIEMPO_POS_INICIAL, TIEMPO_TRAYECTORIA, DIF_TIEMPO, TIEMPO_MARGEN, X_I, Y_I, Z_I, DIST_X, DIST_Y, DIST_Z, GAMMA;
  bool VERBOSE;

  ros::param::get("~TiempoPosicionInicial", TIEMPO_POS_INICIAL);
  ros::param::get("~TiempoTrayectoria", TIEMPO_TRAYECTORIA);
  ros::param::get("~Dif_Tiempo", DIF_TIEMPO);
  ros::param::get("~TiempoMargen", TIEMPO_MARGEN);
  ros::param::get("~X_Inicial", X_I);
  ros::param::get("~Y_Inicial", Y_I);
  ros::param::get("~Z_Inicial", Z_I);
  ros::param::get("~X_Distancia", DIST_X);
  ros::param::get("~Y_Distancia", DIST_Y);
  ros::param::get("~Z_Distancia", DIST_Z);
  ros::param::get("~Gamma", GAMMA);
  ros::param::get("~verbose", VERBOSE);

  const Eigen::Vector3d POSICION_INICIAL(X_I,Y_I,Z_I);
  const Eigen::Vector3d DISTANCIA_TRAYECTORIA(DIST_X,DIST_Y,DIST_Z);

  bebop_simulator::Waypoint waypoint(TIEMPO_POS_INICIAL, TIEMPO_TRAYECTORIA, DIF_TIEMPO, TIEMPO_MARGEN, POSICION_INICIAL, DISTANCIA_TRAYECTORIA, GAMMA, VERBOSE);

  ros::spin();
  
  return 0;

}

