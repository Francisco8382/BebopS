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
            '''
            contador = 0
            signo = None
            for i in range(len(df[ax_gt]) - 1):
                diff = df[ax_gt][i+1] - df[ax_gt][i]
                if signo != None:
                    signoAnt = signo
                    if diff != 0:
                        signo = diff > 0
                    if signo != signoAnt:
                        contador += 1
                elif diff != 0:
                    signo = diff > 0
            cambios[ax] = contador
            '''
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
            '''
            writer.writerow(["MSE_angles", errors[r'angles'].mean()])
            writer.writerow(["MSE_phi", errors[r'\phi'].mean()])
            writer.writerow(["MSE_theta", errors[r'\theta'].mean()])
            '''
            writer.writerow(["MSE_psi", errors[r'\psi'].mean()])
            '''
            writer.writerow(["CSx", cambios[r'x']])
            writer.writerow(["CSy", cambios[r'y']])
            writer.writerow(["CSz", cambios[r'z']])
            writer.writerow(["CSpsi", cambios[r'\psi']])
            '''
            writer.writerow(["Gamma", self.param['Gamma']])
            writer.writerow(["Beta_X", self.param['Beta_X']])
            writer.writerow(["Beta_Y", self.param['Beta_Y']])
            writer.writerow(["Beta_Z", self.param['Beta_Z']])
            writer.writerow(["Beta_Phi", self.param['Beta_Phi']])
            writer.writerow(["Beta_Theta", self.param['Beta_Theta']])
            writer.writerow(["Beta_Psi", self.param['Beta_Psi']])
            writer.writerow(["Mu_X", self.param['Mu_X']])
            writer.writerow(["Mu_Y", self.param['Mu_Y']])
            writer.writerow(["Mu_Z", self.param['Mu_Z']])
            writer.writerow(["Mu_Phi", self.param['Mu_Phi']])
            writer.writerow(["Mu_Theta", self.param['Mu_Theta']])
            writer.writerow(["Mu_Psi", self.param['Mu_Psi']])
            writer.writerow(["U_X", self.param['U_X']])
            writer.writerow(["U_Y", self.param['U_Y']])
            writer.writerow(["U_Z", self.param['U_Z']])
        
        #print("Finalizo script para generar graficas.")
        #plt.figure().clear()
        #plt.close()
        self.restart_pub.publish(Empty())
        rospy.signal_shutdown("")

if __name__ == '__main__':
    rospy.init_node('plot')
    path = rospy.get_param('~Ruta')
    sub = rospy.get_param('/Subcarpeta')
    sub = sub[:-1]
    Gamma = rospy.get_param('~Gamma')
    Beta_X = rospy.get_param('~beta_xy/beta_x')
    Beta_Y = rospy.get_param('~beta_xy/beta_y')
    Beta_Z = rospy.get_param('~beta_z/beta_z')
    Beta_Phi = rospy.get_param('~beta_phi/beta_phi')
    Beta_Theta = rospy.get_param('~beta_theta/beta_theta')
    Beta_Psi = rospy.get_param('~beta_psi/beta_psi')
    Mu_X = rospy.get_param('~mu_xy/mu_x')
    Mu_Y = rospy.get_param('~mu_xy/mu_y')
    Mu_Z = rospy.get_param('~mu_z/mu_z')
    Mu_Phi = rospy.get_param('~mu_phi/mu_phi')
    Mu_Theta = rospy.get_param('~mu_theta/mu_theta')
    Mu_Psi = rospy.get_param('~mu_psi/mu_psi')
    U_X = rospy.get_param('~U_xyz/U_x')
    U_Y = rospy.get_param('~U_xyz/U_y')
    U_Z = rospy.get_param('~U_xyz/U_z')
    param = {'Gamma': Gamma, 'Beta_X': Beta_X, 'Beta_Y': Beta_Y, 'Beta_Z': Beta_Z, \
                'Beta_Phi': Beta_Phi, 'Beta_Theta': Beta_Theta, 'Beta_Psi': Beta_Psi, \
                'Mu_X': Mu_X, 'Mu_Y': Mu_Y, 'Mu_Z': Mu_Z, \
                'Mu_Phi': Mu_Phi, 'Mu_Theta': Mu_Theta, 'Mu_Psi': Mu_Psi, \
                'U_X': U_X, 'U_Y': U_Y, 'U_Z': U_Z}
    plt = Plots(os.path.join(path,sub),param)
    
    