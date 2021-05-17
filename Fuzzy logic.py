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
    distance['superlow'] = fuzz.trimf(distance.universe, [0, 0, 0.5])
    distance['low'] = fuzz.trimf(distance.universe, [0.5, 1, 1.5])
    distance['medium'] = fuzz.trimf(distance.universe, [1.5, 3, 4])
    distance['high'] = fuzz.trimf(distance.universe, [4, 10, 20])
    return distance

def create_velocity_thresholds():
    vel = ctrl.Consequent(np.arange(0, 7, 1), 'velocity')
    vel['superlow'] = fuzz.trimf(vel.universe, [0, 0, 0.5])
    vel['neg_low'] = fuzz.trimf(vel.universe, [-1, -0.5, 0])
    vel['neg_medium'] = fuzz.trimf(vel.universe, [-3, -2, -1])
    vel['low'] = fuzz.trimf(vel.universe, [0.5, 1, 1.5])
    vel['medium'] = fuzz.trimf(vel.universe, [1.5, 3, 4])
    vel['high'] = fuzz.trimf(vel.universe, [4, 5, 6])
    return vel


dist_NE = create_distance_control('NE')
dist_NW = create_distance_control('NW')
dist_EN = create_distance_control('EN')
dist_WN = create_distance_control('WN')
dist_ES = create_distance_control('ES')
dist_WS = create_distance_control('WS')
dist_SE = create_distance_control('SE')
dist_SW = create_distance_control('SW')
vel_thresh = create_velocity_thresholds()



rule0 = ctrl.Rule(dist_NE['superlow'] | dist_EN['superlow'] | dist_NW['superlow'], vel_thresh['superlow'])
rule1 = ctrl.Rule(dist_NE['low'] | dist_EN['low'], vel_thresh['low'])
rule2 = ctrl.Rule(dist_NE['medium'] | dist_EN['medium'], vel_thresh['medium'])
rule3 = ctrl.Rule(dist_NE['high'] | dist_EN['high'], vel_thresh['high'])
left_velocity_ctrl = ctrl.ControlSystem([rule0, rule1, rule2, rule3])


rule0 = ctrl.Rule(dist_NE['superlow'] | dist_EN['superlow'], vel_thresh['superlow'])
rule1 = ctrl.Rule(dist_NE['low'] | dist_EN['low'], vel_thresh['low'])
rule2 = ctrl.Rule(dist_NE['medium'] | dist_EN['medium'], vel_thresh['medium'])
rule3 = ctrl.Rule(dist_NE['high'] | dist_EN['high'], vel_thresh['high'])
rule4 = ctrl.Rule(dist_NE['low'] & dist_EN['medium'] & dist_ES['medium'], vel_thresh['neg_low'])
rule5 = ctrl.Rule(dist_NE['medium'] & dist_EN['medium'] & dist_ES['medium'], vel_thresh['neg_medium'])

right_velocity_ctrl = ctrl.ControlSystem([rule0, rule1, rule2, rule3, rule4, rule5])

print("PRESS ANY BUTTON TO CONTINUE")
x = input()
print("SCRIPT RUNNING")

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

tank.forward()
#continue reading and printing values from proximity sensors
t = time.time()
while (time.time()-t)<360: # read values for 60 seconds
    err_code, detectionState, NE_detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, 83, vrep.simx_opmode_buffer)
    err_code, detectionState, NW_detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, 82, vrep.simx_opmode_buffer)
    err_code, detectionState, EN_detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, 81, vrep.simx_opmode_buffer)
    err_code, detectionState, ES_detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, 80, vrep.simx_opmode_buffer)
    err_code, detectionState, WN_detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, 79, vrep.simx_opmode_buffer)
    err_code, detectionState, WS_detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, 78, vrep.simx_opmode_buffer)
    err_code, detectionState, SE_detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, 77, vrep.simx_opmode_buffer)
    err_code, detectionState, SW_detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, 76, vrep.simx_opmode_buffer)


    velociting_left = ctrl.ControlSystemSimulation(left_velocity_ctrl)
    velociting_left.input['distance_NE'] = np.linalg.norm(NE_detectedPoint)
    velociting_left.input['distance_NW'] = np.linalg.norm(NW_detectedPoint)
    velociting_left.input['distance_EN'] = np.linalg.norm(EN_detectedPoint)
    # velociting_left.input['distance_ES'] = np.linalg.norm(ES_detectedPoint)
    # velociting_left.input['distance_WN'] = np.linalg.norm(NE_detectedPoint)
    # velociting_left.input['distance_WS'] = np.linalg.norm(NW_detectedPoint)
    # velociting_left.input['distance_SE'] = np.linalg.norm(EN_detectedPoint)
    # velociting_left.input['distance_SW'] = np.linalg.norm(ES_detectedPoint)
    velociting_left.compute()

    velociting_right = ctrl.ControlSystemSimulation(right_velocity_ctrl)
    velociting_right.input['distance_NE'] = np.linalg.norm(NE_detectedPoint)
    # velociting_right.input['distance_NW'] = np.linalg.norm(NW_detectedPoint)
    velociting_right.input['distance_EN'] = np.linalg.norm(EN_detectedPoint)
    velociting_right.input['distance_ES'] = np.linalg.norm(ES_detectedPoint)
    velociting_right.compute()

    tank.leftvelocity = velociting_left.output['velocity']
    tank.rightvelocity = velociting_right.output['velocity']
    tank.setVelocity()

    print(velociting_left.output['velocity'], velociting_right.output['velocity'])
    # print(np.linalg.norm(NE_detectedPoint), np.linalg.norm(EN_detectedPoint))


