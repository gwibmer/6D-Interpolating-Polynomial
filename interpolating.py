#Gerardo Wibmer U2890-7597
import sys
import numpy as np
sys.path.append('PythonAPI')
import math
import time
try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')
print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')
else:
    print ('Failed connecting to remote API server')
    sys.exit('Could not connect to Vrep')
# get the handles of arm joints
err_code, armjoint1_handle = sim.simxGetObjectHandle(clientID,"UR5_joint1", 
sim.simx_opmode_blocking)
err_code, armjoint2_handle = sim.simxGetObjectHandle(clientID,"UR5_joint2", 
sim.simx_opmode_blocking)
err_code, armjoint3_handle = sim.simxGetObjectHandle(clientID,"UR5_joint3", 
sim.simx_opmode_blocking)
err_code, armjoint4_handle = sim.simxGetObjectHandle(clientID,"UR5_joint4", 
sim.simx_opmode_blocking)
err_code, armjoint5_handle = sim.simxGetObjectHandle(clientID,"UR5_joint5", 
sim.simx_opmode_blocking)
err_code, armjoint6_handle = sim.simxGetObjectHandle(clientID,"UR5_joint6", 
sim.simx_opmode_blocking)
# get the handles of hand joints
err_code, endeffector_handle = sim.simxGetObjectHandle(clientID,"suctionPad", 
sim.simx_opmode_blocking)
# set the arm to position control
sim.simxSetObjectIntParameter(clientID, armjoint1_handle, 2000, 1, 
sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint1_handle, 2001, 1, 
sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint2_handle, 2000, 1, 
sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint2_handle, 2001, 1, 
sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint3_handle, 2000, 1, 
sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint3_handle, 2001, 1, 
sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint4_handle, 2000, 1, 
sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint4_handle, 2001, 1, 
sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint5_handle, 2000, 1, 
sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint5_handle, 2001, 1, 
sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint6_handle, 2000, 1, 
sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint6_handle, 2001, 1, 
sim.simx_opmode_oneshot)
# get the collision handles
collision_handle_list = []
for i in range(40):
    err_code, collision_handle = sim.simxGetCollisionHandle(clientID, "Collision" +
str(i), sim.simx_opmode_blocking)
    sim.simxReadCollision(clientID, collision_handle, sim.simx_opmode_streaming)
    collision_handle_list.append(collision_handle)
# You do not need to modify the code above
# function to control the movement of the arm, the input are the angles of joint1, 

def move_arm(armpose):
    armpose_convert = []
    for i in range(6):
        armpose_convert.append(round(armpose[i]/180 * math.pi,3))
    sim.simxPauseCommunication(clientID,True)
    sim.simxSetJointTargetPosition(clientID, armjoint1_handle, armpose_convert[0], 
sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, armjoint2_handle, armpose_convert[1], 
sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, armjoint3_handle, armpose_convert[2], 
sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, armjoint4_handle, armpose_convert[3], 
sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, armjoint5_handle, armpose_convert[4], 
sim.simx_opmode_oneshot) 
    sim.simxSetJointTargetPosition(clientID, armjoint6_handle, armpose_convert[5], 
sim.simx_opmode_oneshot)    
    sim.simxPauseCommunication(clientID,False)
    time.sleep(3)
# function to check collision
def check_collision():
    collision_reading = np.zeros(40)
    is_collision = 0
    for i in range(40):
        collision_reading[i] = sim.simxReadCollision(clientID, 
collision_handle_list[i], sim.simx_opmode_buffer)[1]
        if collision_reading[i] == 1:
            is_collision = 1
    if is_collision == 1:
        print('Collision detected!')
        return 1
    else:
        return 0





import numpy as np





#purpose: six dimensional Interpolating Polynomials to move the UR5
#inputs:start and the goal pose
#assumption:start its a set of 6 angles
#post condition: The desire path will be returned
def interpolating(start,goal):
    old_pose =start
    #going through all the times in the array
    for j in range(len(time_arr)):
        pos =[]
        #going through all the angles in the start array
        for i in range(len(start)):
            #declaring the matrices with the times
            M = np.array([[1,t_0,(t_0)**2,(t_0)**3],
                        [0,1,2*t_0,3*t_0**2],
                        [1,t1,t1**2,t1**3],
                        [0,1,2*t1,3*(t1)**2]])

            #declaring the vector with velocity and start and goal pos
            b =np.array([[start[i]],[v0], [goal[i]],[v1]])

            #calulating the coefficients
            a = np.linalg.solve(M,b)

            #solving the equation
            q= a[0] + a[1]*(time_arr[j]) + a[2]*(time_arr[j])**2 +  a[3]*(time_arr[j])**3
            #appending the pose to pos
            pos.append(q[0])
        #checking for collsion
        if check_collision() !=0:
            # if there is a collsion the arm will move to the old pose
            move_arm(old_pose)
            print(old_pose)
            
        
        else:
            #append the nearest out of all the poses genrated
            path.append(pos)
            move_arm(pos)
            print(pos)
            #my new start will be the nearest
            old_pose = pos
    return path

#declaring variables
start = [20,20,25,25,29,20]
goal = [80,84,85,80,80,82]
pos = []
path = []
time_arr = [0,0.2,0.4,0.6,0.8,1,1.2,1.4,1.6,1.8,2,2.2,2.4,2.6,2.8,3]
v0 =0
v1=0
t_0=0
t1=3
#calling my function
free_path = interpolating(start,goal)

# printing my path
#print(free_path)





# no need to modify the code below
# close the communication between collision handles
for i in range(40):
    sim.simxReadCollision(clientID, collision_handle_list[i], 
sim.simx_opmode_discontinue)
print ('Program ended')