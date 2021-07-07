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
        '''
        odometry_df = odometry.loc[:, ['Tiempo',r'$x$',r'$y$',r'$z$',r'$\phi$',r'$\theta$',r'$\psi$']]
        odometry_gt_df = odometry_gt.loc[:, ['Tiempo',r'$x$',r'$y$',r'$z$',r'$\phi$',r'$\theta$',r'$\psi$']]
        reference_df = reference.loc[:, ['Tiempo',r'$x$',r'$y$',r'$z$',r'$\phi$',r'$\theta$',r'$\psi$']]
        referenceGamma_df = referenceGamma.loc[:, ['Tiempo',r'$x$',r'$y$',r'$z$',r'$\phi$',r'$\theta$',r'$\psi$']]
        '''
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
            writer.writerow(["Kp_XY", self.param['Kp_XY']])
            writer.writerow(["Kp_Z", self.param['Kp_Z']])
            writer.writerow(["Kp_Phi_Theta", self.param['Kp_Phi_Theta']])
            writer.writerow(["Kp_Psi", self.param['Kp_Psi']])
            writer.writerow(["Td_XY", self.param['Td_XY']])
            writer.writerow(["Td_Z", self.param['Td_Z']])
            writer.writerow(["Td_Phi_Theta", self.param['Td_Phi_Theta']])
            writer.writerow(["Td_Psi", self.param['Td_Psi']])
            writer.writerow(["Ti_XY", self.param['Ti_XY']])
            writer.writerow(["Ti_Z", self.param['Ti_Z']])
            writer.writerow(["Ti_Phi_Theta", self.param['Ti_Phi_Theta']])
            writer.writerow(["Ti_Psi", self.param['Ti_Psi']])
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
    Kp_XY = rospy.get_param('~Kp_xy/Kp_xy')
    Kp_Z = rospy.get_param('~Kp_z/Kp_z')
    Kp_Phi_Theta = rospy.get_param('~Kp_phi_theta/Kp_phi_theta')
    Kp_Psi = rospy.get_param('~Kp_psi/Kp_psi')
    Td_XY = rospy.get_param('~Td_xy/Td_xy')
    Td_Z = rospy.get_param('~Td_z/Td_z')
    Td_Phi_Theta = rospy.get_param('~Td_phi_theta/Td_phi_theta')
    Td_Psi = rospy.get_param('~Td_psi/Td_psi')
    Ti_XY = rospy.get_param('~Ti_xy/Ti_xy')
    Ti_Z = rospy.get_param('~Ti_z/Ti_z')
    Ti_Phi_Theta = rospy.get_param('~Ti_phi_theta/Ti_phi_theta')
    Ti_Psi = rospy.get_param('~Ti_psi/Ti_psi')
    U_X = rospy.get_param('~U_xyz/U_x')
    U_Y = rospy.get_param('~U_xyz/U_y')
    U_Z = rospy.get_param('~U_xyz/U_z')
    param = {'Gamma': Gamma, 'Kp_XY': Kp_XY, 'Kp_Z': Kp_Z, \
                'Kp_Phi_Theta': Kp_Phi_Theta, 'Kp_Psi': Kp_Psi, \
                'Td_XY': Td_XY, 'Td_Z': Td_Z, \
                'Td_Phi_Theta': Td_Phi_Theta, 'Td_Psi': Td_Psi, \
                'Ti_XY': Ti_XY, 'Ti_Z': Ti_Z, \
                'Ti_Phi_Theta': Ti_Phi_Theta, 'Ti_Psi': Ti_Psi, \
                'U_X': U_X, 'U_Y': U_Y, 'U_Z': U_Z}
    plt = Plots(os.path.join(path,sub),param)
    
    