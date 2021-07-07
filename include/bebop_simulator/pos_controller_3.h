/*
 * Copyright 2018 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2018 Pasquale Oppido, University of Sannio in Benevento, Italy
 * Copyright 2018 Luigi Iannelli, University of Sannio in Benevento, Italy
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

#ifndef BEBOP_CONTROL_POSITION_CONTROLLER_H
#define BEBOP_CONTROL_POSITION_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include <string>

#include <ros/time.h>

#include "extendedKalmanFilter.h"
#include "waypoint_filter.h"
#include "waypointfilter_parameters.h"
#include "filter_parameters.h"
#include "stabilizer_types.h"
#include "parameters.h"
#include "common.h"

#include <gazebo_msgs/GetWorldProperties.h>

#include "bebop_simulator/ReferenceAngles.h"
#include "bebop_simulator/PosController.h"
#include "bebop_simulator/AttitudeController.h"

using namespace std;

namespace bebop_simulator {

// Default values for the Parrot Bebop controller. For more information about the control architecture, please take a look
// at the publications page into the Wiki section.
static const double K1DefaultXYController = 26.3943;
static const double K1DefaultAltitudeController = 27.2277;

static const double K1DefaultRollPitchController = 592.891;
static const double K1DefaultYawRateController = 74.0862;

static const double K2DefaultXYController = 0.9634;
static const double K2DefaultAltitudeController = 0.9645;

static const double K2DefaultRollPitchController = 11.7119;
static const double K2DefaultYawRateController = 2.1245;

static const double K3DefaultXYController = 1.0;
static const double K3DefaultAltitudeController = 1.0;

static const double K3DefaultRollPitchController = 1.0;
static const double K3DefaultYawRateController = 1.0;

static const double K4DefaultXYController = 1.0;
static const double K4DefaultAltitudeController = 1.0;

static const double K4DefaultRollPitchController = 1.0;
static const double K4DefaultYawRateController = 1.0;

static const Eigen::Vector3d UqDefaultXYZ = Eigen::Vector3d(1.1810, 1.1810, 4.6697);

static const double sigmaDefault = 0.9;

class PositionControllerParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PositionControllerParameters()
      : K1_xy_(K1DefaultXYController), 
        K1_z_(K1DefaultAltitudeController), 
        K1_phi_theta_(K1DefaultRollPitchController),  
        K1_psi_(K1DefaultYawRateController),
        K2_xy_(K2DefaultXYController),
        K2_z_(K2DefaultAltitudeController),
        K2_phi_theta_(K2DefaultRollPitchController),
        K2_psi_(K2DefaultYawRateController),
        K3_xy_(K3DefaultXYController), 
        K3_z_(K3DefaultAltitudeController), 
        K3_phi_theta_(K3DefaultRollPitchController),  
        K3_psi_(K3DefaultYawRateController),
        K4_xy_(K4DefaultXYController),
        K4_z_(K4DefaultAltitudeController),
        K4_phi_theta_(K4DefaultRollPitchController),
        K4_psi_(K4DefaultYawRateController),
	    U_q_(UqDefaultXYZ),
        sigma_(sigmaDefault){
  }

  double K1_xy_;
  double K1_z_;

  double K1_phi_theta_;
  double K1_psi_;

  double K2_xy_;
  double K2_z_;

  double K2_phi_theta_;
  double K2_psi_;

  double K3_xy_;
  double K3_z_;

  double K3_phi_theta_;
  double K3_psi_;

  double K4_xy_;
  double K4_z_;

  double K4_phi_theta_;
  double K4_psi_;

  double sigma_;
  
  Eigen::Vector3d U_q_;
};
    
    class PositionController{
        public:
            PositionController();
            ~PositionController();
            void CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities);

            void SetOdometry(const EigenOdometry& odometry);
            void SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory_positionControllerNode);
            void SetControllerGains();
            void SetVehicleParameters();
            void SetWaypointFilterParameters();
            void SetFilterParameters();
            void GetOdometry(nav_msgs::Odometry* odometry_filtered);
            void GetReferenceAngles(nav_msgs::Odometry* reference_angles);
            void GetTrajectory(nav_msgs::Odometry* smoothed_trajectory);
            void SetLaunchFileParameters();
            void GetVelocityAlongZComponents(nav_msgs::Odometry* zVelocity_components);
            void GetPositionAndVelocityErrors(nav_msgs::Odometry* positionAndVelocityErrors);
            void GetAngularAndAngularVelocityErrors(nav_msgs::Odometry* angularAndAngularVelocityErrors);
            
            PositionControllerParameters controller_parameters_;
            ExtendedKalmanFilter extended_kalman_filter_bebop_;
            VehicleParameters vehicle_parameters_;
            FilterParameters filter_parameters_;
            WaypointFilterParameters waypoint_filter_parameters_;
            WaypointFilter waypoint_filter_;

            //Launch file parameters
            std::string user_;
            double dataStoringTime_;
            bool dataStoring_active_;
            bool waypointFilter_active_;
            bool EKF_active_;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        private:
            //Boolean variables to active/unactive the controller and the data storage
            bool controller_active_;

            //Wall clock time offset variable
            double wallSecsOffset_;

            //Gazebo Message for attitude and position
            gazebo_msgs::GetWorldProperties my_messagePosition_;
            ros::NodeHandle clientHandlePosition_;
            ros::ServiceClient clientPosition_;

            ros::NodeHandle clientHandleAttitude_;
            ros::ServiceClient clientAttitude_;
            gazebo_msgs::GetWorldProperties my_messageAttitude_;

            //Sting vectors used to stare data
            std::vector<string> listControlSignals_;
            std::vector<string> listControlMixerTerms_;
            std::vector<string> listPropellersAngularVelocities_;
            std::vector<string> listReferenceAngles_;
            std::vector<string> listVelocityErrors_;
            std::vector<string> listDroneAttitude_;
            std::vector<string> listTrajectoryErrors_;
            std::vector<string> listAttitudeErrors_;
            std::vector<string> listDerivativeAttitudeErrors_;
            std::vector<string> listTimeAttitudeErrors_;
            std::vector<string> listTimePositionErrors_;
            std::vector<string> listDroneAngularVelocitiesABC_;
            std::vector<string> listDroneTrajectoryReference_;
            std::vector<string> listControlMixerTermsSaturated_;
	        std::vector<string> listControlMixerTermsUnsaturated_;
	        std::vector<string> listDroneLinearVelocitiesABC_;
	        std::vector<string> listDronePosition_;
            std::vector<string> listControlMixerTermsUnSaturatedBefore_;
          
            //Controller gains
            double K1_xy_, K1_z_;
            double K1_phi_theta_, K1_psi_;

            double K2_xy_, K2_z_;
            double K2_phi_theta_, K2_psi_;

            double K3_xy_, K3_z_;
            double K3_phi_theta_, K3_psi_;

            double K4_xy_, K4_z_;
            double K4_phi_theta_, K4_psi_;
			
	        double lambda_x_, lambda_y_, lambda_z_;

            double sigma;

            //Position and linear velocity errors
            double e_x_;
            double e_y_;
            double e_z_;
            double dot_e_x_;
            double dot_e_y_; 
            double dot_e_z_;
            double dot_e_x_ant_;
            double dot_e_y_ant_;
            double dot_e_z_ant_;

            double sum_zeta_x_;
            double sum_zeta_y_;
            double sum_zeta_z_;
            double sum_eta_x_;
            double sum_eta_y_;
            double sum_eta_z_;
 
            //Attitude and angular velocity errors
            double e_phi_;
            double e_theta_;
            double e_psi_;
            double dot_e_phi_;
            double dot_e_theta_; 
            double dot_e_psi_;
            double dot_e_phi_ant_;
            double dot_e_theta_ant_;
            double dot_e_psi_ant_;

            double sum_zeta_phi_;
            double sum_zeta_theta_;
            double sum_zeta_psi_;
            double sum_eta_phi_;
            double sum_eta_theta_;
            double sum_eta_psi_;

            //Vehicle parameters
            double bf_, m_, g_;
            double l_, bm_;
            double Ix_, Iy_, Iz_;
            
            ros::NodeHandle n1_;
            ros::NodeHandle n2_;
            ros::NodeHandle n3_;
            ros::Timer timer1_;
            ros::Timer timer2_;
            ros::Timer timer3_;

            //Callback functions to compute the errors among axis and angles
            void CallbackAttitude(const ros::TimerEvent& event);
            void CallbackPosition(const ros::TimerEvent& event);
            void CallbackSaveData(const ros::TimerEvent& event);

            nav_msgs::Odometry odometry_filtered_private_;

	        state_t state_;
            control_t control_;
            mav_msgs::EigenTrajectoryPoint command_trajectory_;
            EigenOdometry odometry_;

            ros::NodeHandle nh;
            ros::Publisher ref_angles_pub;
            ros::Publisher position_controller_pub;
            ros::Publisher attitude_controller_pub;
            bebop_simulator::ReferenceAngles ref_angles;
            bebop_simulator::PosController position_controller;
            bebop_simulator::AttitudeController attitude_controller;

            void SetOdometryEstimated();
            void Quaternion2Euler(double* roll, double* pitch, double* yaw) const;
            void AttitudeController(double* u_phi, double* u_theta, double* u_psi);
            void AngularVelocityErrors(double* dot_e_phi_, double* dot_e_theta_, double* dot_e_psi_);
            void AttitudeErrors(double* e_phi_, double* e_theta_, double* e_psi_);
            void PosController(double* u_T, double* phi_r, double* theta_r, double* u_x, double* u_y, double* u_z, double* u_Terr);
            void PositionErrors(double* e_x, double* e_y, double* e_z);
            void VelocityErrors(double* dot_e_x, double* dot_e_y, double* dot_e_z);

    };

}
#endif // BEBOP_CONTROL_POSITION_CONTROLLER_H
