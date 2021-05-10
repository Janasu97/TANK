import vrep
import sys
import time
import numpy as np
from tank import *
import skfuzzy as fuzz
from skfuzzy import control as ctrl

vrep.simxFinish(-1) # closes all opened connections, in case any prevoius wasnt finished
clientID=vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # start a connection

if clientID!=-1:
    print ("Connected to remote API server")
else:
    print("Not connected to remote API server")
    sys.exit("Could not connect")

def create_distance_control(str_name):
    distance = ctrl.Antecedent(np.arange(0, 7, 1), 'distance_'+str(str_name))
    distance['superlow'] = fuzz.trimf(distance.universe, [0, 0, 1])
    distance['low'] = fuzz.trimf(distance.universe, [0.5, 2.5, 3.5])
    distance['medium'] = fuzz.trimf(distance.universe, [2, 3, 4])
    distance['high'] = fuzz.trimf(distance.universe, [3, 6, 6])
    return distance

def create_velocity_thresholds():
    vel = ctrl.Consequent(np.arange(0, 7, 1), 'velocity')
    vel['superlow'] = fuzz.trimf(vel.universe, [0, 0, 1])
    vel['low'] = fuzz.trimf(vel.universe, [0.5, 2, 4])
    vel['medium'] = fuzz.trimf(vel.universe, [2, 5, 6])
    vel['high'] = fuzz.trimf(vel.universe, [5, 6, 6])
    return vel


dist_NE = create_distance_control('NE')
dist_NW = create_distance_control('NW')
vel_thresh = create_velocity_thresholds()

rule0 = ctrl.Rule(dist_NE['superlow'] | dist_NW['superlow'], vel_thresh['superlow'])
rule1 = ctrl.Rule(dist_NE['low'] | dist_NW['low'], vel_thresh['low'])
rule2 = ctrl.Rule(dist_NE['medium'] | dist_NW['medium'], vel_thresh['medium'])
rule3 = ctrl.Rule(dist_NE['high'] | dist_NW['high'], vel_thresh['high'])
velocity_ctrl = ctrl.ControlSystem([rule0, rule1, rule2, rule3])

print("PRESS ANY BUTTON TO CONTINUE")
x = input()
print("SCRIPT RUNNING")

stop_dist_thresh = 1

#create instance of Tank
tank=Tank(clientID)

proximity_sensors = ["EN", "ES", "NE", "NW", "SE", "SW", "WN", "WS"]
proximity_sensors_handles = [0] * 8

# get handle to proximity sensors
for i in range(len(proximity_sensors)):
    err_code, proximity_sensors_handles[i] = vrep.simxGetObjectHandle(clientID, "Proximity_sensor_" + proximity_sensors[i], vrep.simx_opmode_blocking)

# read and print values from proximity sensors
# first reading should be done with simx_opmode_streaming, further with simx_opmode_buffer parameter
for sensor_name, sensor_handle in zip(proximity_sensors, proximity_sensors_handles):
    err_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, sensor_handle, vrep.simx_opmode_streaming)

tank.forward(5)
#continue reading and printing values from proximity sensors
t = time.time()
while (time.time()-t)<60: # read values for 60 seconds
    err_code,detectionState,NE_detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID, 83, vrep.simx_opmode_buffer)
    err_code, detectionState, NW_detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, 82, vrep.simx_opmode_buffer)

    velociting = ctrl.ControlSystemSimulation(velocity_ctrl)
    velociting.input['distance_NE'] = np.linalg.norm(NE_detectedPoint)
    velociting.input['distance_NW'] = np.linalg.norm(NW_detectedPoint)
    velociting.compute()

    if np.linalg.norm(NW_detectedPoint) < stop_dist_thresh or np.linalg.norm(NE_detectedPoint) < stop_dist_thresh:
        print("STOPPING TANK")
        tank.stop()
    else:
        tank.forward(velociting.output['velocity'])

    print("V, dNE, dNW", velociting.output['velocity'], np.linalg.norm(NE_detectedPoint), np.linalg.norm(NW_detectedPoint))

