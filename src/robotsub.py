
#!usr/bin/env python3.8
import os
import rospy
from std_msgs.msg import Int16
from scservo_sdk import *
from sensor_msgs.msg import JointState

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

                  # Uses SCServo SDK library


class sub:
    def __init__(self):
        # Control table address
        self.ADDR_SCS_TORQUE_ENABLE     = 40
        self.ADDR_STS_GOAL_ACC          = 41
        self.ADDR_STS_GOAL_POSITION     = 42
        self.ADDR_STS_GOAL_SPEED        = 46
        self.ADDR_STS_PRESENT_POSITION  = 56


        # Default setting
        self.SCS1_ID                     = 1                 # SCServo#1 ID : 1
        self.SCS2_ID                     = 2                 # SCServo#1 ID : 2
        self.SCS3_ID                     = 3                 # SCServo#1 ID : 3
        self.SCS4_ID                     = 4                 # SCServo#1 ID : 4
        self.SCS5_ID                     = 5                 # SCServo#1 ID : 5
        self.SCS6_ID                     = 6                 # SCServo#1 ID : 6
        
        self.BAUDRATE                    = 1000000           # SCServo default self.BAUDRATE : 1000000
        self.DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

        self.SCS_MINIMUM_POSITION_VALUE  = 0              # SCServo will rotate between this value
        self.SCS_MAXIMUM_POSITION_VALUE  = 4095              # and this value (note that the SCServo would not move when the position value is out of movable range. Check e-manual about the range of the SCServo you use.)
        self.SCS_MOVING_STATUS_THRESHOLD = 20                # SCServo moving status threshold
        self.SCS_MOVING_SPEED            = 800               # SCServo moving speed
        self.SCS_MOVING_ACC              = 20                 # SCServo moving acc
        self.protocol_end                = 0                 # SCServo bit end(STS/SMS=0, SCS=1)

      
        rospy.init_node('servosubnode')
        self.sub=rospy.Subscriber('/joint_states',JointState,self.callback)
        rospy.spin()
    def callback(self,data):
        self.servorun(data)
    def servorun(self,data):
        a=int(2047-(round((data.position[0]*180/3.1428)*11.375)))
        b=int(2047-(round((data.position[1]*180/3.1428)*11.375)))
        c=int(2047-(round((data.position[2]*180/3.1428)*11.375)))
        d=int(2047-(round((data.position[3]*180/3.1428)*11.375)))
        e=int(2047-(round((data.position[4]*180/3.1428)*11.375)))
        f=int(2047-(round((data.position[5]*180/3.1428)*11.375)))
        
        scs_goal_position = [a,b,c,d,e,f]         # Goal position]         # Goal position


        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        portHandler = PortHandler(self.DEVICENAME)

        # Initialize PacketHandler instance
        # Get methods and members of Protocol
        packetHandler = PacketHandler(self.protocol_end)

        # Initialize GroupSyncWrite instance
        groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, self.ADDR_STS_GOAL_POSITION, 2)

        # Open port
        if portHandler.openPort():
            #print("Succeeded to open the port")
            print('')
        else:
            #print("Failed to open the port")
            #print("Press any key to terminate...")
            getch()
            quit()


        # Set port self.BAUDRATE
        if portHandler.setBaudRate(self.BAUDRATE):
            #print("Succeeded to change the self.BAUDRATE")
            print('')
        else:
            #print("Failed to change the self.BAUDRATE")
            #print("Press any key to terminate...")
            getch()
            quit()

        # SCServo#1 acc
        scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, self.SCS1_ID, self.ADDR_STS_GOAL_ACC, self.SCS_MOVING_ACC)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))

        # SCServo#2 acc
        scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, self.SCS2_ID, self.ADDR_STS_GOAL_ACC, self.SCS_MOVING_ACC)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))

        # SCServo#3 acc
        scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, self.SCS3_ID, self.ADDR_STS_GOAL_ACC, self.SCS_MOVING_ACC)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))

        # SCServo#4 acc
        scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, self.SCS4_ID, self.ADDR_STS_GOAL_ACC, self.SCS_MOVING_ACC)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))

        # SCServo#5 acc
        scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, self.SCS5_ID, self.ADDR_STS_GOAL_ACC, self.SCS_MOVING_ACC)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))

        # SCServo#6 acc
        scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, self.SCS6_ID, self.ADDR_STS_GOAL_ACC, self.SCS_MOVING_ACC)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))

        

        # SCServo#1 speed
        scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, self.SCS1_ID, self.ADDR_STS_GOAL_SPEED, self.SCS_MOVING_SPEED)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))
        
        # SCServo#2 speed
        scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, self.SCS2_ID, self.ADDR_STS_GOAL_SPEED, self.SCS_MOVING_SPEED)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))

        # SCServo#3 speed
        scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, self.SCS3_ID, self.ADDR_STS_GOAL_SPEED, self.SCS_MOVING_SPEED)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))
        
        # SCServo#4 speed
        scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, self.SCS4_ID, self.ADDR_STS_GOAL_SPEED, self.SCS_MOVING_SPEED)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))

        # SCServo#5 speed
        scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, self.SCS5_ID, self.ADDR_STS_GOAL_SPEED, self.SCS_MOVING_SPEED)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))
        
        # SCServo#6 speed
        scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, self.SCS6_ID, self.ADDR_STS_GOAL_SPEED, self.SCS_MOVING_SPEED)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))
        

        

    
        param_goal_position1 = [SCS_LOBYTE(scs_goal_position[0]), SCS_HIBYTE(scs_goal_position[0])]
        param_goal_position2 = [SCS_LOBYTE(scs_goal_position[1]), SCS_HIBYTE(scs_goal_position[1])]
        param_goal_position3 = [SCS_LOBYTE(scs_goal_position[2]), SCS_HIBYTE(scs_goal_position[2])]
        param_goal_position4 = [SCS_LOBYTE(scs_goal_position[3]), SCS_HIBYTE(scs_goal_position[3])]
        param_goal_position5 = [SCS_LOBYTE(scs_goal_position[4]), SCS_HIBYTE(scs_goal_position[4])]
        param_goal_position6 = [SCS_LOBYTE(scs_goal_position[5]), SCS_HIBYTE(scs_goal_position[5])]
        param_goal_position=[param_goal_position1,param_goal_position2,param_goal_position3,param_goal_position4,param_goal_position5,param_goal_position6]
        
        # Add SCServo#1 goal position value to the Syncwrite parameter storage
        scs_addparam_result = groupSyncWrite.addParam(self.SCS1_ID, param_goal_position[0])
        #if scs_addparam_result != True:
            #print("[ID:%03d] groupSyncWrite addparam failed" % self.SCS1_ID)
            #quit()

        # Add SCServo#2 goal position value to the Syncwrite parameter storage
        scs_addparam_result = groupSyncWrite.addParam(self.SCS2_ID, param_goal_position[1])
        #if scs_addparam_result != True:
            #print("[ID:%03d] groupSyncWrite addparam failed" % self.SCS2_ID)
            #quit()
        
        # Add SCServo#3 goal position value to the Syncwrite parameter storage
        scs_addparam_result = groupSyncWrite.addParam(self.SCS3_ID, param_goal_position[2])
        #if scs_addparam_result != True:
            #print("[ID:%03d] groupSyncWrite addparam failed" % self.SCS3_ID)
            #quit()

        # Add SCServo#4 goal position value to the Syncwrite parameter storage
        scs_addparam_result = groupSyncWrite.addParam(self.SCS4_ID, param_goal_position[3])
        #if scs_addparam_result != True:
            #print("[ID:%03d] groupSyncWrite addparam failed" % self.SCS4_ID)
            #quit()
        
        # Add SCServo#5 goal position value to the Syncwrite parameter storage
        scs_addparam_result = groupSyncWrite.addParam(self.SCS5_ID, param_goal_position[4])
        #if scs_addparam_result != True:
            #print("[ID:%03d] groupSyncWrite addparam failed" % self.SCS5_ID)
            #quit()

        # Add SCServo#6 goal position value to the Syncwrite parameter storage
        scs_addparam_result = groupSyncWrite.addParam(self.SCS6_ID, param_goal_position[5])
        #if scs_addparam_result != True:
            #print("[ID:%03d] groupSyncWrite addparam failed" % self.SCS6_ID)
            #quit()
        

        # Syncwrite goal position
        scs_comm_result1 = groupSyncWrite.txPacket()
        #if scs_comm_result != COMM_SUCCESS:
            #print("%s" % packetHandler.getTxRxResult(scs_comm_result1))
            
        scs_comm_result2 = groupSyncWrite.txPacket()
        #if scs_comm_result != COMM_SUCCESS:
            #print("%s" % packetHandler.getTxRxResult(scs_comm_result2))
        scs_comm_result3 = groupSyncWrite.txPacket()
        #if scs_comm_result != COMM_SUCCESS:
            #print("%s" % packetHandler.getTxRxResult(scs_comm_result3))
            
        scs_comm_result4 = groupSyncWrite.txPacket()
        #if scs_comm_result != COMM_SUCCESS:
            #print("%s" % packetHandler.getTxRxResult(scs_comm_result4))
        scs_comm_result5 = groupSyncWrite.txPacket()
        #if scs_comm_result != COMM_SUCCESS:
            #print("%s" % packetHandler.getTxRxResult(scs_comm_result5))
            
        scs_comm_result6 = groupSyncWrite.txPacket()
        #if scs_comm_result != COMM_SUCCESS:
            #print("%s" % packetHandler.getTxRxResult(scs_comm_result6))

        # Clear syncwrite parameter storage
        #groupSyncWrite.clearParam()

        
        # Read SCServo#1 present position
        scs1_present_position_speed, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, self.SCS1_ID, self.ADDR_STS_PRESENT_POSITION)
        #if scs_comm_result != COMM_SUCCESS:
            #print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        #elif scs_error != 0:
            #print("%s" % packetHandler.getRxPacketError(scs_error))

        # Read SCServo#2 present position
        scs2_present_position_speed, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, self.SCS2_ID, self.ADDR_STS_PRESENT_POSITION)
        #if scs_comm_result != COMM_SUCCESS:
            #print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        #elif scs_error != 0:
            #print("%s" % packetHandler.getRxPacketError(scs_error))
        
        # Read SCServo#3 present position
        scs3_present_position_speed, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, self.SCS3_ID, self.ADDR_STS_PRESENT_POSITION)
        #if scs_comm_result != COMM_SUCCESS:
            #print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        #elif scs_error != 0:
            #print("%s" % packetHandler.getRxPacketError(scs_error))

        # Read SCServo#4 present position
        scs4_present_position_speed, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, self.SCS4_ID, self.ADDR_STS_PRESENT_POSITION)
        #if scs_comm_result != COMM_SUCCESS:
            #print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        #elif scs_error != 0:
            #print("%s" % packetHandler.getRxPacketError(scs_error))
        
        # Read SCServo#5 present position
        scs5_present_position_speed, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, self.SCS5_ID, self.ADDR_STS_PRESENT_POSITION)
        #if scs_comm_result != COMM_SUCCESS:
            #print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        #elif scs_error != 0:
            #print("%s" % packetHandler.getRxPacketError(scs_error))

        # Read SCServo#6 present position
        scs6_present_position_speed, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, self.SCS6_ID, self.ADDR_STS_PRESENT_POSITION)
        #if scs_comm_result != COMM_SUCCESS:
            #print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        #elif scs_error != 0:
            #print("%s" % packetHandler.getRxPacketError(scs_error))

        

        scs1_present_position = SCS_LOWORD(scs1_present_position_speed)
        scs1_present_speed = SCS_HIWORD(scs1_present_position_speed)

        scs2_present_position = SCS_LOWORD(scs2_present_position_speed)
        scs2_present_speed = SCS_HIWORD(scs2_present_position_speed)

        scs3_present_position = SCS_LOWORD(scs3_present_position_speed)
        scs3_present_speed = SCS_HIWORD(scs3_present_position_speed)

        scs4_present_position = SCS_LOWORD(scs4_present_position_speed)
        scs4_present_speed = SCS_HIWORD(scs4_present_position_speed)

        scs5_present_position = SCS_LOWORD(scs5_present_position_speed)
        scs5_present_speed = SCS_HIWORD(scs5_present_position_speed)

        scs6_present_position = SCS_LOWORD(scs6_present_position_speed)
        scs6_present_speed = SCS_HIWORD(scs6_present_position_speed)
        print("[ID:%03d] GoalPos:%03d PresPos:%03d PresSpd:%03d\t[ID:%03d] GoalPos:%03d PresPos:%03d PresSpd:%03d" 
                % (self.SCS1_ID, scs_goal_position[0], scs1_present_position, SCS_TOHOST(scs1_present_speed, 15), 
                    self.SCS2_ID, scs_goal_position[1], scs2_present_position, SCS_TOHOST(scs2_present_speed, 15)))
        

        

        # Close port
        #portHandler.closePort()

def main():
    test=sub()
if __name__=='__main__':
    main()