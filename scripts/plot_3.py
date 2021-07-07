#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
import os
import csv
import time
import pandas as pd
import matplotlib
matplotlib.use('Agg')
#import matplotlib.pyplot as plt
#import sys

class Plots:
    def __init__(self, path, param):
        self.path = path
        self.param = param
        rospy.Subscriber("/csv/end", Empty, self.Plot)
        self.restart_pub = rospy.Publisher('/restart', Empty, queue_size=1)
        rospy.spin() 

    def Plot(self, data):
        odometry = pd.read_csv(os.path.join(self.path,'odometry.csv'))
        odometry.rename(columns={'X': r'$x$', 'Y': r'$y$', 'Z': r'$z$', 'Roll': r'$\phi$', 'Pitch': r'$\theta$', 'Yaw': r'$\psi$'}, inplace = True)
        odometry_gt = pd.read_csv(os.path.join(self.path,'odometry_gt.csv'))
        odometry_gt.rename(columns={'X': r'$x$', 'Y': r'$y$', 'Z': r'$z$', 'Roll': r'$\phi$', 'Pitch': r'$\theta$', 'Yaw': r'$\psi$'}, inplace = True)
        reference = pd.read_csv(os.path.join(self.path,'reference.csv'))
        reference.rename(columns={'X': r'$x$', 'Y': r'$y$', 'Z': r'$z$', 'Roll': r'$\phi$', 'Pitch': r'$\theta$', 'Yaw': r'$\psi$'}, inplace = True)
        referenceGamma = pd.read_csv(os.path.join(self.path,'referenceGamma.csv'))
        referenceGamma.rename(columns={'X': r'$x$', 'Y': r'$y$', 'Z': r'$z$', 'Roll': r'$\phi$', 'Pitch': r'$\theta$', 'Yaw': r'$\psi$'}, inplace = True)
        odometry_df = odometry.loc[:, ['Tiempo',r'$x$',r'$y$',r'$z$',r'$\psi$']]
        odometry_gt_df = odometry_gt.loc[:, ['Tiempo',r'$x$',r'$y$',r'$z$',r'$\psi$']]
        reference_df = reference.loc[:, ['Tiempo',r'$x$',r'$y$',r'$z$',r'$\psi$']]
        referenceGamma_df = referenceGamma.loc[:, ['Tiempo',r'$x$',r'$y$',r'$z$',r'$\psi$']]

        odometry_df.plot(x=0,grid=True,title='Odometry').get_figure().savefig(os.path.join(self.path,'odometry.png'))
        odometry_gt_df.plot(x=0,grid=True,title='Odometry_GT').get_figure().savefig(os.path.join(self.path,'odometry_gt.png'))
        reference_df.plot(x=0,grid=True,title='Reference').get_figure().savefig(os.path.join(self.path,'reference.png'))
        referenceGamma_df.plot(x=0,grid=True,title='Reference_Gamma').get_figure().savefig(os.path.join(self.path,'referenceGamma.png'))
        
        errors = {}
        #cambios= {}

        #for ax in [r'x',r'y',r'z',r'\phi',r'\theta',r'\psi']:
        for ax in [r'x',r'y',r'z',r'\psi']:
            ax_orig = r'$' + ax + r'$'
            ax_odom = r'$' + ax + r'_{odom}$'
            ax_gt = r'$' + ax + r'_{gt}$'
            ax_ref = r'$' + ax + r'_{ref}$'
            ax_gamma = r'$' + ax + r'_{gamma}$'
            ax_err = r'$e_{' + ax + r'}$'
            odometry_ = odometry_df.loc[:,['Tiempo',ax_orig]]
            odometry_.rename(columns={ax_orig: ax_odom},inplace = True)
            odometry_gt_ = odometry_gt_df.loc[:,['Tiempo',ax_orig]]
            odometry_gt_.rename(columns={ax_orig: ax_gt},inplace = True)
            reference_ = reference_df.loc[:,['Tiempo',ax_orig]]
            reference_.rename(columns={ax_orig: ax_ref},inplace = True)
            referenceGamma_ = referenceGamma_df.loc[:,['Tiempo',ax_orig]]
            referenceGamma_.rename(columns={ax_orig: ax_gamma},inplace = True)
            df = pd.merge(odometry_, odometry_gt_, on='Tiempo', how='inner')
            df = pd.merge(df, reference_, on='Tiempo', how='inner')
            df = pd.merge(df, referenceGamma_, on='Tiempo', how='inner')
            df[ax_err] = df[ax_ref] - df[ax_odom]
            errors[ax] = df[ax_err]**2
            name = ax + '.png'
            fn = os.path.join(self.path,name)
            df.plot(x=0,grid=True,title=ax_orig).get_figure().savefig(fn)
        
        errors[r'xyz'] = errors[r'x'] + errors[r'y'] + errors[r'z']
        #errors[r'angles'] = errors[r'\phi'] + errors[r'\theta']

        with open(os.path.join(self.path,'resultados.csv'), 'w') as archivo:
            writer = csv.writer(archivo)
            writer.writerow(["MSE", errors[r'xyz'].mean()])
            writer.writerow(["MSE_x", errors[r'x'].mean()])
            writer.writerow(["MSE_y", errors[r'y'].mean()])
            writer.writerow(["MSE_z", errors[r'z'].mean()])
            writer.writerow(["MSE_psi", errors[r'\psi'].mean()])
            writer.writerow(["Gamma", self.param['Gamma']])
            writer.writerow(["K1_XY", self.param['K1_XY']])
            writer.writerow(["K1_Z", self.param['K1_Z']])
            writer.writerow(["K1_Phi_Theta", self.param['K1_Phi_Theta']])
            writer.writerow(["K1_Psi", self.param['K1_Psi']])
            writer.writerow(["K2_XY", self.param['K2_XY']])
            writer.writerow(["K2_Z", self.param['K2_Z']])
            writer.writerow(["K2_Phi_Theta", self.param['K2_Phi_Theta']])
            writer.writerow(["K2_Psi", self.param['K2_Psi']])
            writer.writerow(["K3_XY", self.param['K3_XY']])
            writer.writerow(["K3_Z", self.param['K3_Z']])
            writer.writerow(["K3_Phi_Theta", self.param['K3_Phi_Theta']])
            writer.writerow(["K3_Psi", self.param['K3_Psi']])
            writer.writerow(["K4_XY", self.param['K4_XY']])
            writer.writerow(["K4_Z", self.param['K4_Z']])
            writer.writerow(["K4_Phi_Theta", self.param['K4_Phi_Theta']])
            writer.writerow(["K4_Psi", self.param['K4_Psi']])
            writer.writerow(["Sigma", self.param['sigma']])
            writer.writerow(["U_X", self.param['U_X']])
            writer.writerow(["U_Y", self.param['U_Y']])
            writer.writerow(["U_Z", self.param['U_Z']])
        self.restart_pub.publish(Empty())
        rospy.signal_shutdown("")

if __name__ == '__main__':
    rospy.init_node('plot')
    path = rospy.get_param('~Ruta')
    sub = rospy.get_param('/Subcarpeta')
    sub = sub[:-1]
    Gamma = rospy.get_param('~Gamma')
    K1_XY = rospy.get_param('~K1_xy/K1_xy')
    K1_Z = rospy.get_param('~K1_z/K1_z')
    K1_Phi_Theta = rospy.get_param('~K1_phi_theta/K1_phi_theta')
    K1_Psi = rospy.get_param('~K1_psi/K1_psi')
    K2_XY = rospy.get_param('~K2_xy/K2_xy')
    K2_Z = rospy.get_param('~K2_z/K2_z')
    K2_Phi_Theta = rospy.get_param('~K2_phi_theta/K2_phi_theta')
    K2_Psi = rospy.get_param('~K2_psi/K2_psi')
    K3_XY = rospy.get_param('~K3_xy/K3_xy')
    K3_Z = rospy.get_param('~K3_z/K3_z')
    K3_Phi_Theta = rospy.get_param('~K3_phi_theta/K3_phi_theta')
    K3_Psi = rospy.get_param('~K3_psi/K3_psi')
    K4_XY = rospy.get_param('~K4_xy/K4_xy')
    K4_Z = rospy.get_param('~K4_z/K4_z')
    K4_Phi_Theta = rospy.get_param('~K4_phi_theta/K4_phi_theta')
    K4_Psi = rospy.get_param('~K4_psi/K4_psi')
    sigma = rospy.get_param('~sigma/sigma')
    U_X = rospy.get_param('~U_xyz/U_x')
    U_Y = rospy.get_param('~U_xyz/U_y')
    U_Z = rospy.get_param('~U_xyz/U_z')
    param = {'Gamma': Gamma, 'K1_XY': K1_XY, 'K1_Z': K1_Z, \
                'K1_Phi_Theta': K1_Phi_Theta, 'K1_Psi': K1_Psi, \
                'K2_XY': K2_XY, 'K2_Z': K2_Z, \
                'K2_Phi_Theta': K2_Phi_Theta, 'K2_Psi': K2_Psi, \
                'K3_XY': K3_XY, 'K3_Z': K3_Z, \
                'K3_Phi_Theta': K3_Phi_Theta, 'K3_Psi': K3_Psi, \
                'K4_XY': K4_XY, 'K4_Z': K4_Z, \
                'K4_Phi_Theta': K4_Phi_Theta, 'K4_Psi': K4_Psi, \
                'sigma': sigma,'U_X': U_X, 'U_Y': U_Y, 'U_Z': U_Z}
    plt = Plots(os.path.join(path,sub),param)
    
    