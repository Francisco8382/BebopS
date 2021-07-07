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
#include <ctime>

#include <fstream>
#include <iostream>
#include <stdlib.h>

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

static const float DEG_2_RAD = M_PI / 180.0;

namespace bebop_simulator {

    class Launcher{
        public:
            Launcher();
            ~Launcher();
            
        private:
            double beta_xy_i, beta_z_i, beta_phi_theta_i, beta_psi_i;
            double mu_xy_i, mu_z_i, mu_phi_theta_i, mu_psi_i;
            double beta_xy_f, beta_z_f, beta_phi_theta_f, beta_psi_f;
            double mu_xy_f, mu_z_f, mu_phi_theta_f, mu_psi_f;
            double beta_xy_d, beta_z_d, beta_phi_theta_d, beta_psi_d;
            double mu_xy_d, mu_z_d, mu_phi_theta_d, mu_psi_d;
            double beta_xy, beta_z, beta_phi_theta, beta_psi;
            double mu_xy, mu_z, mu_phi_theta, mu_psi;
            std::string pos_controller_node, csv_odometry, plot, waypoint;

            ros::NodeHandle nh1;
            ros::Timer _timer1;
            bool trigger = true;

            void InitializeParams();
            void TimerCallback(const ros::TimerEvent& event);
            void RunNodes();
            std::string getTimeStr();
    };

    Launcher::Launcher() {
      InitializeParams();
      _timer1 = nh1.createTimer(ros::Duration(1.0), &Launcher::TimerCallback, this, false, true);
    }
    
    Launcher::~Launcher() {}

    void Launcher::InitializeParams() {
      double U_x, U_y, U_z;
      double mass, in_xx, in_xy, in_xz, in_yy, in_yz, in_zz, bf, bm, l;
      double Qp_aa, Qp_bb, Qp_cc, Qp_dd, Qp_ee, Qp_ff;
      double dev_x, dev_y, dev_z, dev_vx, dev_vy, dev_vz;
      double Tsf, H;
      bool use_sim_time, csvFilesStoring, waypoint_filter, EKFActive;
      std::string user_account;
      double csvFilesStoringTime;
      double Gamma, TiempoMargen;
      double TiempoPosicionInicial, TiempoTrayectoria, Dif_Tiempo;
      std::string Ruta, Subcarpeta;
      double X_Inicial, Y_Inicial, Z_Inicial;
      double X_Distancia, Y_Distancia, Z_Distancia;
      bool verbose;
      ros::param::get("~beta_xy/beta_xy_i", beta_xy_i);
      ros::param::get("~beta_z/beta_z_i", beta_z_i);
      ros::param::get("~beta_phi_theta/beta_phi_theta_i", beta_phi_theta_i);
      ros::param::get("~beta_psi/beta_psi_i", beta_psi_i);
      ros::param::get("~mu_xy/mu_xy_i", mu_xy_i);
      ros::param::get("~mu_z/mu_z_i", mu_z_i);
      ros::param::get("~mu_phi_theta/mu_phi_theta_i", mu_phi_theta_i);
      ros::param::get("~mu_psi/mu_psi_i", mu_psi_i);
      ros::param::get("~beta_xy/beta_xy_f", beta_xy_f);
      ros::param::get("~beta_z/beta_z_f", beta_z_f);
      ros::param::get("~beta_phi_theta/beta_phi_theta_f", beta_phi_theta_f);
      ros::param::get("~beta_psi/beta_psi_f", beta_psi_f);
      ros::param::get("~mu_xy/mu_xy_f", mu_xy_f);
      ros::param::get("~mu_z/mu_z_f", mu_z_f);
      ros::param::get("~mu_phi_theta/mu_phi_theta_f", mu_phi_theta_f);
      ros::param::get("~mu_psi/mu_psi_f", mu_psi_f);
      ros::param::get("~beta_xy/beta_xy_d", beta_xy_d);
      ros::param::get("~beta_z/beta_z_d", beta_z_d);
      ros::param::get("~beta_phi_theta/beta_phi_theta_d", beta_phi_theta_d);
      ros::param::get("~beta_psi/beta_psi_d", beta_psi_d);
      ros::param::get("~mu_xy/mu_xy_d", mu_xy_d);
      ros::param::get("~mu_z/mu_z_d", mu_z_d);
      ros::param::get("~mu_phi_theta/mu_phi_theta_d", mu_phi_theta_d);
      ros::param::get("~mu_psi/mu_psi_d", mu_psi_d);
      ros::param::get("~U_xyz/U_x", U_x);
      ros::param::get("~U_xyz/U_y", U_y);
      ros::param::get("~U_xyz/U_z", U_z);
      ros::param::get("~mass", mass);
      ros::param::get("~inertia/xx", in_xx);
      ros::param::get("~inertia/xy", in_xy);
      ros::param::get("~inertia/xz", in_xz);
      ros::param::get("~inertia/yy", in_yy);
      ros::param::get("~inertia/yz", in_yz);
      ros::param::get("~inertia/zz", in_zz);
      ros::param::get("~bf", bf);
      ros::param::get("~bm", bm);
      ros::param::get("~l", l);
      ros::param::get("~Qp/aa", Qp_aa);
      ros::param::get("~Qp/bb", Qp_bb);
      ros::param::get("~Qp/cc", Qp_cc);
      ros::param::get("~Qp/dd", Qp_dd);
      ros::param::get("~Qp/ee", Qp_ee);
      ros::param::get("~Qp/ff", Qp_ff);
      ros::param::get("~dev_x", dev_x);
      ros::param::get("~dev_y", dev_y);
      ros::param::get("~dev_z", dev_z);
      ros::param::get("~dev_vx", dev_vx);
      ros::param::get("~dev_vy", dev_vy);
      ros::param::get("~dev_vz", dev_vz);
      ros::param::get("~Tsf", Tsf);
      ros::param::get("~H", H);
      ros::param::get("~use_sim_time", use_sim_time);
      ros::param::get("~csvFilesStoring", csvFilesStoring);
      ros::param::get("~csvFilesStoringTime", csvFilesStoringTime);
      ros::param::get("~user_account", user_account);
      ros::param::get("~waypoint_filter", waypoint_filter);
      ros::param::get("~EKFActive", EKFActive);
      ros::param::get("~Gamma", Gamma);
      ros::param::get("~TiempoMargen", TiempoMargen);
      ros::param::get("~Ruta", Ruta);
      ros::param::get("~TiempoPosicionInicial", TiempoPosicionInicial);
      ros::param::get("~TiempoTrayectoria", TiempoTrayectoria);
      ros::param::get("~Dif_Tiempo", Dif_Tiempo);
      ros::param::get("~X_Inicial", X_Inicial);
      ros::param::get("~Y_Inicial", Y_Inicial);
      ros::param::get("~Z_Inicial", Z_Inicial);
      ros::param::get("~X_Distancia", X_Distancia);
      ros::param::get("~Y_Distancia", Y_Distancia);
      ros::param::get("~Z_Distancia", Z_Distancia);
      ros::param::get("~verbose", verbose);

      std::string s_U_x, s_U_y, s_U_z, s_mass, s_in_xx, s_in_xy, s_in_xz, s_in_yy, s_in_yz, s_in_zz,
      s_bf, s_bm, s_l, s_Qp_aa, s_Qp_bb, s_Qp_cc, s_Qp_dd, s_Qp_ee, s_Qp_ff,
      s_dev_x, s_dev_y, s_dev_z, s_dev_vx, s_dev_vy, s_dev_vz, s_use_sim_time, 
      s_csvFilesStoring, s_waypoint_filter, s_EKFActive, s_csvFilesStoringTime,
      s_Gamma, s_TiempoMargen, s_TiempoPosicionInicial, s_TiempoTrayectoria, 
      s_Dif_Tiempo, s_X_Inicial, s_Y_Inicial, s_Z_Inicial, s_X_Distancia,
      s_Y_Distancia, s_Z_Distancia, s_verbose, s_Tsf, s_H;
      std::ostringstream stream_bf;
      stream_bf << bf;

      s_U_x = std::to_string(U_x);
      s_U_y = std::to_string(U_y);
      s_U_z = std::to_string(U_z);
      s_mass = std::to_string(mass);
      s_in_xx = std::to_string(in_xx);
      s_in_xy = std::to_string(in_xy);
      s_in_xz = std::to_string(in_xz);
      s_in_yy = std::to_string(in_yy);
      s_in_yz = std::to_string(in_yz);
      s_in_zz = std::to_string(in_zz);
      s_bf = stream_bf.str();
      s_bm = std::to_string(bm);
      s_l = std::to_string(l);
      s_Qp_aa = std::to_string(Qp_aa);
      s_Qp_bb = std::to_string(Qp_bb);
      s_Qp_cc = std::to_string(Qp_cc);
      s_Qp_dd = std::to_string(Qp_dd);
      s_Qp_ee = std::to_string(Qp_ee);
      s_Qp_ff = std::to_string(Qp_ff);
      s_dev_x = std::to_string(dev_x);
      s_dev_y = std::to_string(dev_y);
      s_dev_z = std::to_string(dev_z);
      s_dev_vx = std::to_string(dev_vx);
      s_dev_vy = std::to_string(dev_vy);
      s_dev_vz = std::to_string(dev_vz);
      s_Tsf = std::to_string(Tsf);
      s_H = std::to_string(H);
      if (use_sim_time){
        s_use_sim_time = "true";
      }
      else {
        s_use_sim_time = "false";
      }
      if (csvFilesStoring){
        s_csvFilesStoring = "true";
      }
      else {
        s_csvFilesStoring = "false";
      }
      if (waypoint_filter){
        s_waypoint_filter = "true";
      }
      else {
        s_waypoint_filter = "false";
      }
      if (EKFActive){
        s_EKFActive = "true";
      }
      else {
        s_EKFActive = "false";
      }
      s_csvFilesStoringTime = std::to_string(csvFilesStoringTime);
      s_Gamma = std::to_string(Gamma);
      s_TiempoMargen = std::to_string(TiempoMargen);
      s_TiempoPosicionInicial = std::to_string(TiempoPosicionInicial);
      s_TiempoTrayectoria = std::to_string(TiempoTrayectoria);
      s_Dif_Tiempo = std::to_string(Dif_Tiempo);
      s_X_Inicial = std::to_string(X_Inicial);
      s_Y_Inicial = std::to_string(Y_Inicial);
      s_Z_Inicial = std::to_string(Z_Inicial);
      s_X_Distancia = std::to_string(X_Distancia);
      s_Y_Distancia = std::to_string(Y_Distancia);
      s_Z_Distancia = std::to_string(Z_Distancia);
      if (verbose){
        s_verbose = "true";
      }
      else {
        s_verbose = "false";
      }

      pos_controller_node = "rosrun bebop_simulator pos_controller_node _U_xyz/U_x:=" 
      + s_U_x + " _U_xyz/U_y:=" + s_U_y + " _U_xyz/U_z:=" + s_U_z
      + " _mass:=" + s_mass + " _inertia/xx:=" + s_in_xx + " _inertia/xy:=" + s_in_xy
      + " _inertia/xz:=" + s_in_xz + " _inertia/yy:=" + s_in_yy
      + " _inertia/yz:=" + s_in_yz + " _inertia/zz:=" + s_in_zz
      + " _bf:=" + s_bf + " _bm:=" + s_bm + " _l:=" + s_l
      + " _Qp/aa:=" + s_Qp_aa + " _Qp/bb:=" + s_Qp_bb
      + " _Qp/cc:=" + s_Qp_cc + " _Qp/dd:=" + s_Qp_dd
      + " _Qp/ee:=" + s_Qp_ee + " _Qp/ff:=" + s_Qp_ff
      + " _dev_x:=" + s_dev_x + " _dev_y:=" + s_dev_y
      + " _dev_z:=" + s_dev_z + " _dev_vx:=" + s_dev_vx
      + " _dev_vy:=" + s_dev_vy + " _dev_vz:=" + s_dev_vz
      + " _Tsf:=" + s_Tsf + " _H:=" + s_H
      + " _use_sim_time:=" + s_use_sim_time
      + " _csvFilesStoring:=" + s_csvFilesStoring
      + " _csvFilesStoringTime:=" + s_csvFilesStoringTime
      + " _user_account:=" + user_account
      + " _waypoint_filter:=" + s_waypoint_filter
      + " _EKFActive:=" + s_EKFActive
      + " /command/motor_speed:=/bebop/command/motors"
      + " /odometry:=/bebop/odometry"
      + " /odometry_gt:=/bebop/odometry_gt"
      + " /filteredOutput:=/bebop/filteredOutput"
      + " /referenceAngles:=/bebop/referenceAngles"
      + " /stateErrors:=/bebop/stateErrors"
      + " /smoothedTrajectory:=/bebop/smoothedTrajectory"
      + " /command/trajectory:=/bebop/command/trajectory"
      + " /uTerrComponents:=/bebop/uTerrComponents"
      + " /zVelocityComponents:=/bebop/zVelocityComponents"
      + " /positionAndVelocityErrors:=/bebop/positionAndVelocityErrors"
      + " /angularAndAngularVelocityErrors:=/bebop/angularAndAngularVelocityErrors";

      csv_odometry = "rosrun bebop_simulator csv_odometry _Ruta:="
      + Ruta + " _TiempoMargen:=" + s_TiempoMargen;

      plot = "rosrun bebop_simulator plot.py _Ruta:=" + Ruta
      + " _TiempoMargen:=" + s_TiempoMargen 
      + " _Gamma:=" + s_Gamma + " _U_xyz/U_x:=" + s_U_x 
      + " _U_xyz/U_y:=" + s_U_y + " _U_xyz/U_z:=" + s_U_z;

      waypoint = "rosrun bebop_simulator waypoint_gen _TiempoPosicionInicial:="
      + s_TiempoPosicionInicial + " _TiempoTrayectoria:=" + s_TiempoTrayectoria
      + " _Dif_Tiempo:=" + s_Dif_Tiempo + " _TiempoMargen:=" + s_TiempoMargen
      + " _X_Inicial:=" + s_X_Inicial + " _Y_Inicial:=" + s_Y_Inicial
      + " _Z_Inicial:=" + s_Z_Inicial + " _X_Distancia:=" + s_X_Distancia
      + " _Y_Distancia:=" + s_Y_Distancia + " _Z_Distancia:=" + s_Z_Distancia
      + " _Gamma:=" + s_Gamma + " _verbose:=" + s_verbose
      + " /command/trajectory:=/bebop/command/trajectory"
      + " /Odometry:=/bebop/Odometry"
      + " /OdometryGT:=/bebop/OdometryGT"
      + " /PositionReference:=/bebop/PositionReference"
      + " /PositionWithGamma:=/bebop/PositionWithGamma"
      + " /odometry:=/bebop/odometry"
      + " /odometry_gt:=/bebop/odometry_gt";

      beta_xy = beta_xy_i;
      beta_z = beta_z_i;
      beta_phi_theta = beta_phi_theta_i;
      beta_psi = beta_psi_i;
      mu_xy = mu_xy_i;
      mu_z = mu_z_i;
      mu_phi_theta = mu_phi_theta_i;
      mu_psi = mu_psi_i;

    }

    void Launcher::TimerCallback(const ros::TimerEvent& event) {
      RunNodes();
      beta_xy += beta_xy_d;
      if (beta_xy < beta_xy_f && beta_xy_d > 0){
        return;
      }
      beta_xy = beta_xy_i;
      beta_z += beta_z_d;
      if (beta_z < beta_z_f && beta_z_d > 0){
        return;
      }
      beta_z = beta_z_i;
      beta_phi_theta += beta_phi_theta_d;
      if (beta_phi_theta < beta_phi_theta_f && beta_phi_theta_d > 0){
        return;
      }
      beta_phi_theta = beta_phi_theta_i;
      beta_psi += beta_psi_d;
      if (beta_psi < beta_psi_f && beta_psi_d > 0){
        return;
      }
      beta_psi = beta_psi_i;
      mu_xy += mu_xy_d;
      if (mu_xy < mu_xy_f && mu_xy_d > 0){
        return;
      }
      mu_xy = mu_xy_i;
      mu_z += mu_z_d;
      if (mu_z < mu_z_f && mu_z_d > 0){
        return;
      }
      mu_z = mu_z_i;
      mu_phi_theta += mu_phi_theta_d;
      if (mu_phi_theta < mu_phi_theta_f && mu_phi_theta_d > 0){
        return;
      }
      mu_phi_theta = mu_phi_theta_i;
      mu_psi += mu_psi_d;
      if (mu_psi < mu_psi_f && mu_psi_d > 0){
        return;
      }
      else {
        ros::shutdown();
      }

    }

    void Launcher::RunNodes() {
      std::string s_beta_xy, s_beta_z, s_beta_phi_theta, s_beta_psi,
      s_mu_xy, s_mu_z, s_mu_phi_theta, s_mu_psi, s_controller_params;
      std::string run_pos_controller_node, run_csv1, run_csv2, run_csv3, run_csv4,
      run_plot, run_waypoint;
      s_beta_xy = std::to_string(beta_xy);
      s_beta_z = std::to_string(beta_z);
      s_beta_phi_theta = std::to_string(beta_phi_theta);
      s_beta_psi = std::to_string(beta_psi);
      s_mu_xy = std::to_string(mu_xy);
      s_mu_z = std::to_string(mu_z);
      s_mu_phi_theta = std::to_string(mu_phi_theta);
      s_mu_psi = std::to_string(mu_psi);
      
      s_controller_params = " _beta_xy/beta_x:=" + s_beta_xy
      + " _beta_xy/beta_y:=" + s_beta_xy + " _beta_z/beta_z:=" + s_beta_z
      + " _beta_phi/beta_phi:=" + s_beta_phi_theta + " _beta_theta/beta_theta:="
      + s_beta_phi_theta + " _beta_psi/beta_psi:=" + s_beta_psi
      + " _mu_xy/mu_x:=" + s_mu_xy + " _mu_xy/mu_y:=" + s_mu_xy
      + " _mu_z/mu_z:=" + s_mu_z + " _mu_phi/mu_phi:=" + s_mu_phi_theta
      + " _mu_theta/mu_theta:=" + s_mu_phi_theta + " _mu_psi/mu_psi:=" + s_mu_psi;

      std::string Subcarpeta = getTimeStr();
      ros::param::set("/Subcarpeta", Subcarpeta.c_str());
      run_pos_controller_node = pos_controller_node + s_controller_params 
      + " __name:=pos_controller_node";
      run_csv1 = csv_odometry + " _Topico:=/bebop/Odometry"
      + " _Nombre:=odometry.csv __name:=csv_odometry_1";
      run_csv2 = csv_odometry + " _Topico:=/bebop/OdometryGT"
      + " _Nombre:=odometry_gt.csv __name:=csv_odometry_2";
      run_csv3 = csv_odometry + " _Topico:=/bebop/PositionReference"
      + " _Nombre:=reference.csv __name:=csv_odometry_3";
      run_csv4 = csv_odometry + " _Topico:=/bebop/PositionWithGamma"
      + " _Nombre:=referenceGamma.csv __name:=csv_odometry_4";
      run_plot = plot + s_controller_params + " __name:=plot";
      run_waypoint = waypoint + " __name:=waypoint_gen";
      
      ROS_INFO(run_waypoint.c_str());
      ROS_INFO(run_pos_controller_node.c_str());
      ROS_INFO(run_csv1.c_str());
      ROS_INFO(run_csv2.c_str());
      ROS_INFO(run_csv3.c_str());
      ROS_INFO(run_csv4.c_str());
      ROS_INFO(run_plot.c_str());
      

      std::thread th_pos_controller_node(system,run_pos_controller_node.c_str());
      std::thread th_csv1(system,run_csv1.c_str());
      std::thread th_csv2(system,run_csv2.c_str());
      std::thread th_csv3(system,run_csv3.c_str());
      std::thread th_csv4(system,run_csv4.c_str());
      std::thread th_plot(system,run_plot.c_str());
      std::thread th_waypoint(system,run_waypoint.c_str());

      th_pos_controller_node.join();
      th_csv1.join();
      th_csv2.join();
      th_csv3.join();
      th_csv4.join();   
      th_plot.join();  
      th_waypoint.join(); 

    }

    std::string Launcher::getTimeStr() {
      std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
      std::string s(30, '\0');
      std::strftime(&s[0], s.size(), "%Y-%m-%d %H:%M:%S ", std::localtime(&now));
      return s;
    }

}

int main(int argc, char** argv){
  ros::init(argc, argv, "launcher");
  ros::NodeHandle nh;
  bebop_simulator::Launcher launcher_;
  ros::spin();
  return 0;
}

