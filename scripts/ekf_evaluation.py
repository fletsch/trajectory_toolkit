#! /usr/bin/env python
# Imports
import os, sys, inspect
import numpy as np

from trajectory_toolkit.TimedData import TimedData
from trajectory_toolkit.Plotter import Plotter
from trajectory_toolkit import Quaternion
from trajectory_toolkit import Utils
from trajectory_toolkit import RosDataAcquisition
from trajectory_toolkit.VIEvaluator import VIEvaluator

plotRon = True
plotAtt = True
plotPos = True
plotVel = True
plotRor = True
plotYpr = True
plotExt = True

td_filter = TimedData()
td_vicon = TimedData()
td_comp = TimedData(16)

outputFolder = '/home/frederic/Documents/BachelorThesis/ekf_evaluation'
inputPath = '/home/frederic/datasets/vicon_offline/vicon_normal2_2019-07-05-03-14-43.bag'

rovioEvaluator = VIEvaluator()
rovioEvaluator.bag = '/home/frederic/datasets/vicon_offline/vicon_normal2_2019-07-05-03-14-43.bag'
#rovioEvaluator.bag = '/home/frederic/datasets/vicon_offline/vicon_short_2019-06-23-15-10-39.bag'
rovioEvaluator.odomTopic = '/msf_core/odometry'
rovioEvaluator.gtFile = '/home/frederic/datasets/vicon_offline/vicon_normal2_2019-07-05-03-14-43.bag'
#rovioEvaluator.gtFile = '/home/frederic/datasets/vicon_offline/vicon_short_2019-06-23-15-10-39.bag'
rovioEvaluator.gtTopic = '/vrpn_client_1562248578120672780/estimated_transform'
#rovioEvaluator.extraTransformAtt = np.array([1, 0, 0, 0])   #[w x y z]
#rovioEvaluator.extraTransformPos = np.array([0, 0, 0])
rovioEvaluator.startcut = 0
rovioEvaluator.endcut = 0
rovioEvaluator.doCov = False
rovioEvaluator.doNFeatures = 25
rovioEvaluator.doExtrinsics = False
rovioEvaluator.doBiases = False
rovioEvaluator.alignMode = 1
rovioEvaluator.plotLeutiDistances = []

rovioEvaluator.initTimedData(td_filter)
rovioEvaluator.initTimedDataGT(td_vicon)
rovioEvaluator.acquireData()
rovioEvaluator.acquireDataGT()
rovioEvaluator.getAllDerivatives()
rovioEvaluator.alignTime()
rovioEvaluator.alignBodyFrame()  
rovioEvaluator.alignInertialFrame()  
rovioEvaluator.getYpr()
rovioEvaluator.evaluateSigmaBounds()



#Write output to file !!! apends to file if it already exists
td_filter.writeColsToFile(outputFolder + '/vicon_normal2_2019-07-05-03-14-43', 'pose_ekf', td_filter.getColIDs('pos') + td_filter.getColIDs('ypr'), ' ')
td_vicon.writeColsToFile(outputFolder + '/vicon_normal2_2019-07-05-03-14-43', 'pose_vicon', td_vicon.getColIDs('pos') + td_vicon.getColIDs('ypr') + td_vicon.getColIDs('att'), ' ')


#load complementary filter output and estimate time offset with rotational rate from gyroscope
gyroIDs = [1,2,3]
accIDs = [4,5,6]
orientIDs = [7,8,9,10]
ronCompID = 11
orientAlignedIDs = [12,13,14, 15]
RosDataAcquisition.rosBagLoadImuWithOrientation(inputPath, '/est_states/imu', td_comp, gyroIDs, accIDs, orientIDs)
td_comp.computeNormOfColumns(gyroIDs, ronCompID)
td_comp.applyTimeOffset(td_vicon.getFirstTime()-td_comp.getFirstTime())
to_comp = td_comp.getTimeOffset(ronCompID, td_vicon, td_vicon.getColIDs('ron'))
print('Time offset complementary filter to vicon is: ' + str(to_comp))
#apply rotation part of body alignmet to complementary filter orientation (transformation same as ekf)
# rotation = np.array([0.9999251528730725, -0.0022342581584454853, 0.01197584675309537, 0.0011295294552196493])
# rotated = Quaternion.q_mult(np.kron(np.ones([td_comp.length(),1]),rotation), td_comp.col(orientIDs))
# for i in np.arange(0,3):
    # td_comp.setCol(rotated[:,i], orientAlignedIDs[i])
td_comp.writeColsToFile(outputFolder + '/vicon_normal2_2019-07-05-03-14-43', 'attitude_comp', orientIDs, ' ')


if plotPos: # Position plotting
    plotterPos = Plotter(-1, [3,1],'Position',['','','time[s]'],['x[m]','y[m]','z[m]'],10000)
    if rovioEvaluator.doCov:
        plotterPos.addDataToSubplotMultiple(td_filter, 'posSm', [1,2,3], ['r--','r--','r--'], ['','',''])
        plotterPos.addDataToSubplotMultiple(td_filter, 'posSp', [1,2,3], ['r--','r--','r--'], ['','',''])
    plotterPos.addDataToSubplotMultiple(td_filter, 'pos', [1,2,3], ['r','r','r'], ['','',''])
    plotterPos.addDataToSubplotMultiple(td_vicon, 'pos', [1,2,3], ['b','b','b'], ['','',''])

if plotVel: # Velocity plotting
    plotterVel = Plotter(-1, [3,1],'Robocentric Velocity',['','','time[s]'],['$v_x$[m/s]','$v_y$[m/s]','$v_z$[m/s]'],10000)
    plotterVel.addDataToSubplotMultiple(td_filter, 'vel', [1,2,3], ['r','r','r'], ['','',''])
    plotterVel.addDataToSubplotMultiple(td_vicon, 'vel', [1,2,3], ['b','b','b'], ['','',''])
    
if plotAtt: # Attitude plotting
    plotterAtt = Plotter(-1, [4,1],'Attitude Quaternion',['','','','time[s]'],['w[1]','x[1]','y[1]','z[1]'],10000)
    plotterAtt.addDataToSubplotMultiple(td_filter, 'att', [1,2,3,4], ['r','r','r','r'], ['','','',''])
    plotterAtt.addDataToSubplotMultiple(td_vicon, 'att', [1,2,3,4], ['b','b','b','b'], ['','','',''])

if plotYpr: # Yaw-pitch-roll plotting
    plotterYpr = Plotter(-1, [3,1],'Yaw-Pitch-Roll Decomposition',['','','time[s]'],['roll[rad]','pitch[rad]','yaw[rad]'],10000)
    if rovioEvaluator.doCov:
        plotterYpr.addDataToSubplotMultiple(td_filter, 'yprSm', [1,2,3], ['r--','r--','r--'], ['','',''])
        plotterYpr.addDataToSubplotMultiple(td_filter, 'yprSp', [1,2,3], ['r--','r--','r--'], ['','',''])
    plotterYpr.addDataToSubplotMultiple(td_filter, 'ypr', [1,2,3], ['r','r','r'], ['','',''])
    plotterYpr.addDataToSubplotMultiple(td_vicon, 'ypr', [1,2,3], ['b','b','b'], ['','',''])
    
if plotRor: # Rotational rate plotting
    plotterRor = Plotter(-1, [3,1],'Rotational Rate',['','','time[s]'],['$\omega_x$[rad/s]','$\omega_y$[rad/s]','$\omega_z$[rad/s]'],10000)
    plotterRor.addDataToSubplotMultiple(td_filter, 'ror', [1,2,3], ['r','r','r'], ['','',''])
    plotterRor.addDataToSubplotMultiple(td_vicon, 'ror', [1,2,3], ['b','b','b'], ['','',''])

if plotRon: # Plotting rotational rate norm
    plotterRon = Plotter(-1, [1,1],'Norm of Rotational Rate',['time [s]'],['Rotational Rate Norm [rad/s]'],10000)
    plotterRon.addDataToSubplot(td_filter, 'ron', 1, 'r', 'rovio rotational rate norm')
    plotterRon.addDataToSubplot(td_vicon, 'ron', 1, 'b', 'vicon rotational rate norm')

if plotExt and rovioEvaluator.doExtrinsics: # Extrinsics Plotting
    plotterExt = Plotter(-1, [3,1],'Extrinsics Translational Part',['','','time[s]'],['x[m]','y[m]','z[m]'],10000)
    if rovioEvaluator.doCov:
        plotterExt.addDataToSubplotMultiple(td_filter, 'extPosSm', [1,2,3], ['r--','r--','r--'], ['','',''])
        plotterExt.addDataToSubplotMultiple(td_filter, 'extPosSp', [1,2,3], ['r--','r--','r--'], ['','',''])
    plotterExt.addDataToSubplotMultiple(td_filter, 'extPos', [1,2,3], ['r','r','r'], ['','',''])


raw_input("Press Enter to continue...")
           

