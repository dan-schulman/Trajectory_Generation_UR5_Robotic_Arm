from robolink import *    # API to communicate with RoboDK
from robodk import *      # robodk robotics toolbox
# Any interaction with RoboDK must be done through RDK:
RDK = Robolink()
# Select a robot (a popup is displayed if more than one robot is available)
robot = RDK.ItemUserPick('Select a robot', ITEM_TYPE_ROBOT)
if not robot.Valid():
    raise Exception('No robot selected or available')
RUN_ON_ROBOT = False
# Important: by default, the run mode is RUNMODE_SIMULATE
# If the program is generated offline manually the runmode will be RUNMODE_MAKE_ROBOTPROG,
# Therefore, we should not run the program on the robot
if RDK.RunMode() != RUNMODE_SIMULATE:
    RUN_ON_ROBOT = False

if RUN_ON_ROBOT:
    # Connect to the robot using default IP
    success = robot.Connect() # Try to connect once
    status, status_msg = robot.ConnectedState()
    if status != ROBOTCOM_READY:
        # Stop if the connection did not succeed
        print(status_msg)
        raise Exception("Failed to connect: " + status_msg)
    # This will set to run the API programs on the robot and the simulator (online programming)
    RDK.setRunMode(RUNMODE_RUN_ROBOT)

# Get the current joint position of the robot
joints_ref = robot.Joints()
# get the current position of the TCP with respect to the reference frame:
target_ref = robot.Pose()
pos_ref = target_ref.Pos()
robot.setPoseFrame(robot.PoseFrame())
robot.setPoseTool(robot.PoseTool())
robot.setZoneData(-1)


# Define Custom Functions
def sleep(time):
    robot.RunCodeCustom('sleep('+str(time)+')',INSTRUCTION_CALL_PROGRAM)

def setGripperForce(force):
    robot.RunCodeCustom('rq_set_force('+str(force)+')',INSTRUCTION_CALL_PROGRAM)

def setGripperSpeed(speed):
    robot.RunCodeCustom('rq_set_speed('+str(speed)+')',INSTRUCTION_CALL_PROGRAM)

def setGripperPos(pos):
    robot.RunCodeCustom('rq_move('+str(pos)+')',INSTRUCTION_CALL_PROGRAM)

def setGripperPos_andWait(pos):
    robot.RunCodeCustom('rq_move_and_wait('+str(pos)+')',INSTRUCTION_CALL_PROGRAM)

def clamp():
    robot.RunCodeCustom('set_standard_digital_out(0,True)',INSTRUCTION_CALL_PROGRAM)
    robot.RunCodeCustom('set_standard_digital_out(2,False)',INSTRUCTION_CALL_PROGRAM)
    sleep(.5)
    robot.RunCodeCustom('set_standard_digital_out(0,False)',INSTRUCTION_CALL_PROGRAM)

def unclamp():
    robot.RunCodeCustom('set_standard_digital_out(0,False)',INSTRUCTION_CALL_PROGRAM)
    robot.RunCodeCustom('set_standard_digital_out(2,True)',INSTRUCTION_CALL_PROGRAM)
    sleep(.5)
    robot.RunCodeCustom('set_standard_digital_out(2,False)',INSTRUCTION_CALL_PROGRAM)


# Performs a screwing action
def screw(rotations,n):
    robot.setSpeed(-1, speed_joints=999, accel_joints=999)
    joints = robot.Joints().tolist()
    joints[5] = n
    robot.MoveJ(joints)
    for i in range(rotations):
        setGripperPos(195)
        sleep(0.1)
        joints[5] = -n
        robot.MoveJ(joints)

        setGripperPos_andWait(255)
        joints[5] = n
        
        robot.MoveJ(joints)
        
    setGripperPos_andWait(195)


####################################################
####################################################
####################################################

home = [0, -90, 0, -90, 0, 0]
intermediate = [0, -100, 100, -90, -90, 0]


above_clamp = [-26.938514, -115.773416, 107.129619, -81.356203, -90.000000, 63.061486]



head = [32.561687, -75.229599, 112.795828, -127.566230, -90.000000, 122.561687]

barrel = [4.617545, -90.960361, 132.112022, -131.151661, -90.000000, 94.617545]

battery = [13.954572, -88.151571, 127.361292, -129.209721, -90.000000, 103.954572]
battery_place = [15.735524, -85.527850, 119.847558, -34.319708, -164.264476, 180.000000]
battery_above = [13.954572, -90.666906, 125.343618, -124.676712, -90.000000, 103.954572]
battery_above_clamp = [15.874455, -91.325601, 111.514849, -20.189248, -164.125545, 180.000000]

tail_cap = [24.876613, -80.290201, 121.223392, -130.933191, -90.000000, 114.876613]
tail_cap_above = [24.876613, -84.914505, 117.548685, -122.634180, -90.000000, 114.876613]


clamp_x_pos = -313.5

# Initialize the Gripper with maximum force and speed, and open it
robot.setSpeed(1200, speed_joints=100, accel_linear=300, accel_joints=300)
robot.RunCodeCustom('rq_activate()',INSTRUCTION_CALL_PROGRAM)
setGripperForce(255)
setGripperSpeed(255)
setGripperPos(1)

unclamp()			# Make sure the chuck is initially open
robot.MoveJ(home)	# Start at the "Home" position

# Go to the intermediate pose to ensure the desired the robot configuration
robot.MoveJ(intermediate)


# Move to the head of the flashlight and grip it
robot.MoveJ(head)
setGripperPos_andWait(255)
# Linear Move above head
robot.MoveL([32.546667, -79.218939, 109.148726, -119.929788, -90, 122.546667])

# Bring the head above the clamp, then bring it down
robot.MoveJ(above_clamp)
robot.MoveL(xyzrpw_2_pose([clamp_x_pos, 38, 130, -180,0,0])) # Adjustable Z

# Release the head from the gripper, clamp the chuck, and move back
setGripperPos_andWait(0)
sleep(0.5)
clamp()
robot.MoveJ(above_clamp)


# Now bring the barrel
robot.MoveJ(barrel)
setGripperPos_andWait(255)
robot.MoveL([4.593521, -99.667946, 122.498235, -112.830290, -90, 94.593521])
robot.MoveJ(above_clamp)
robot.setSpeed(75) #reduce speed for barrel
# z = 185 mm for "high" barrel position, ~155 for "low"
robot.MoveL(xyzrpw_2_pose([clamp_x_pos, 37.3, 160, -180,0,0])) # Adjustable Z
#robot.RunCodeCustom('Screw in the barrel', INSTRUCTION_COMMENT)
setGripperPos(255)
screw(10, 90)
robot.setSpeed(-1, speed_joints=100, accel_joints=300)
robot.RunCodeCustom('tighten_torque(3, 0, 1.57, 2, 3, 5, 255, 255, 50)', INSTRUCTION_INSERT_CODE)
setGripperPos(195)
robot.MoveJ(above_clamp)
robot.setSpeed(200)

# For completing the screwing action during testing
#sleep(3)


# Now we go for the Battery
robot.MoveJ(battery_above)
robot.MoveJ(battery)
setGripperSpeed(150)
setGripperPos_andWait(255)
robot.MoveJ([15.257632, -88.902803, 125.365718, -126.462915, -90, 105.257632])
robot.MoveJ(battery_above_clamp)
robot.setSpeed(50) #reduce speed for battery
robot.MoveL(battery_place) #place battery
setGripperSpeed(255)
setGripperPos_andWait(0)
robot.setSpeed(200)
robot.MoveJ(battery_above_clamp)


# Finally, the Tail Cap
robot.setSpeed(-1, speed_joints=100, accel_joints=300)
robot.MoveJ(tail_cap_above)
robot.MoveL(tail_cap)
setGripperPos_andWait(255)
robot.MoveJ([24.677712, -84.930770, 117.566801, -122.636031, -90, 114.677712])
robot.MoveJ(above_clamp)
robot.MoveL(xyzrpw_2_pose([clamp_x_pos-0.6, 36.9, 209, -180,0,0])) # Adjustable Z
screw(1,270)
setGripperPos(195)
screw(4,90)
robot.setSpeed(-1, speed_joints=100, accel_joints=300)
robot.RunCodeCustom('tighten_torque(2, 0, 1.57, 2, 3, 5, 255, 255, 50)', INSTRUCTION_INSERT_CODE)
robot.MoveL(xyzrpw_2_pose([clamp_x_pos-0.6, 36.9, 197, -180,0,0])) # Adjustable Z
setGripperPos_andWait(255)
unclamp()
robot.MoveL(xyzrpw_2_pose([clamp_x_pos-0.6, 36.9, 250, -180,0,0]))
head_back_above = [32.561687, -85.331922, 96.631190, -101.299268, -90.000000, 122.561687]
head_back = [32.561687, -80.269040, 107.930666, -117.661627, -90.000000, 122.561687] #adjust z position
robot.setSpeed(1200)
robot.MoveJ(head_back_above)
robot.setSpeed(200)
robot.MoveL(head_back)
setGripperPos_andWait(0)
robot.MoveJ(home)

print('done')
