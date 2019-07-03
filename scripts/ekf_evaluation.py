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

td_rovio = TimedData()
td_vicon = TimedData()

rovioEvaluator = VIEvaluator()
rovioEvaluator.bag = '/home/frederic/datasets/vicon_offline/vicon_2019-06-27-15-35-54.bag'
rovioEvaluator.odomTopic = '/msf_core/odometry'
rovioEvaluator.gtFile = '/home/frederic/datasets/vicon_offline/vicon_2019-06-27-15-35-54.bag'
rovioEvaluator.gtTopic = '/vrpn_client_1560523366042626656/estimated_transform'
#rovioEvaluator.extraTransformAtt = np.array([1, 0, 0, 0])   #[w x y z]
#rovioEvaluator.extraTransformPos = np.array([0, 0, 0])
rovioEvaluator.startcut = 0
rovioEvaluator.endcut = 0
rovioEvaluator.doCov = True
rovioEvaluator.doNFeatures = 25
rovioEvaluator.doExtrinsics = False
rovioEvaluator.doBiases = False
rovioEvaluator.alignMode = 1
rovioEvaluator.plotLeutiDistances = []

rovioEvaluator.initTimedData(td_rovio)
rovioEvaluator.initTimedDataGT(td_vicon)
rovioEvaluator.acquireData()
rovioEvaluator.acquireDataGT()
rovioEvaluator.getAllDerivatives()
rovioEvaluator.alignTime()
rovioEvaluator.alignBodyFrame()
rovioEvaluator.alignInertialFrame()
rovioEvaluator.getYpr()
rovioEvaluator.evaluateSigmaBounds()
   
if plotPos: # Position plotting
    plotterPos = Plotter(-1, [3,1],'Position',['','','time[s]'],['x[m]','y[m]','z[m]'],10000)
    if rovioEvaluator.doCov:
        plotterPos.addDataToSubplotMultiple(td_rovio, 'posSm', [1,2,3], ['r--','r--','r--'], ['','',''])
        plotterPos.addDataToSubplotMultiple(td_rovio, 'posSp', [1,2,3], ['r--','r--','r--'], ['','',''])
    plotterPos.addDataToSubplotMultiple(td_rovio, 'pos', [1,2,3], ['r','r','r'], ['','',''])
    plotterPos.addDataToSubplotMultiple(td_vicon, 'pos', [1,2,3], ['b','b','b'], ['','',''])

if plotVel: # Velocity plotting
    plotterVel = Plotter(-1, [3,1],'Robocentric Velocity',['','','time[s]'],['$v_x$[m/s]','$v_y$[m/s]','$v_z$[m/s]'],10000)
    plotterVel.addDataToSubplotMultiple(td_rovio, 'vel', [1,2,3], ['r','r','r'], ['','',''])
    plotterVel.addDataToSubplotMultiple(td_vicon, 'vel', [1,2,3], ['b','b','b'], ['','',''])
    
if plotAtt: # Attitude plotting
    plotterAtt = Plotter(-1, [4,1],'Attitude Quaternion',['','','','time[s]'],['w[1]','x[1]','y[1]','z[1]'],10000)
    plotterAtt.addDataToSubplotMultiple(td_rovio, 'att', [1,2,3,4], ['r','r','r','r'], ['','','',''])
    plotterAtt.addDataToSubplotMultiple(td_vicon, 'att', [1,2,3,4], ['b','b','b','b'], ['','','',''])

if plotYpr: # Yaw-pitch-roll plotting
    plotterYpr = Plotter(-1, [3,1],'Yaw-Pitch-Roll Decomposition',['','','time[s]'],['roll[rad]','pitch[rad]','yaw[rad]'],10000)
    if rovioEvaluator.doCov:
        plotterYpr.addDataToSubplotMultiple(td_rovio, 'yprSm', [1,2,3], ['r--','r--','r--'], ['','',''])
        plotterYpr.addDataToSubplotMultiple(td_rovio, 'yprSp', [1,2,3], ['r--','r--','r--'], ['','',''])
    plotterYpr.addDataToSubplotMultiple(td_rovio, 'ypr', [1,2,3], ['r','r','r'], ['','',''])
    plotterYpr.addDataToSubplotMultiple(td_vicon, 'ypr', [1,2,3], ['b','b','b'], ['','',''])
    
if plotRor: # Rotational rate plotting
    plotterRor = Plotter(-1, [3,1],'Rotational Rate',['','','time[s]'],['$\omega_x$[rad/s]','$\omega_y$[rad/s]','$\omega_z$[rad/s]'],10000)
    plotterRor.addDataToSubplotMultiple(td_rovio, 'ror', [1,2,3], ['r','r','r'], ['','',''])
    plotterRor.addDataToSubplotMultiple(td_vicon, 'ror', [1,2,3], ['b','b','b'], ['','',''])

if plotRon: # Plotting rotational rate norm
    plotterRon = Plotter(-1, [1,1],'Norm of Rotational Rate',['time [s]'],['Rotational Rate Norm [rad/s]'],10000)
    plotterRon.addDataToSubplot(td_rovio, 'ron', 1, 'r', 'rovio rotational rate norm')
    plotterRon.addDataToSubplot(td_vicon, 'ron', 1, 'b', 'vicon rotational rate norm')

if plotExt and rovioEvaluator.doExtrinsics: # Extrinsics Plotting
    plotterExt = Plotter(-1, [3,1],'Extrinsics Translational Part',['','','time[s]'],['x[m]','y[m]','z[m]'],10000)
    if rovioEvaluator.doCov:
        plotterExt.addDataToSubplotMultiple(td_rovio, 'extPosSm', [1,2,3], ['r--','r--','r--'], ['','',''])
        plotterExt.addDataToSubplotMultiple(td_rovio, 'extPosSp', [1,2,3], ['r--','r--','r--'], ['','',''])
    plotterExt.addDataToSubplotMultiple(td_rovio, 'extPos', [1,2,3], ['r','r','r'], ['','',''])


raw_input("Press Enter to continue...")
           

