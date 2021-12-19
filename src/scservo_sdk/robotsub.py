#!usr/bin/env python3.8
import os
import rospy
from std_msgs.msg import Int16
from scservo_sdk import *
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
        self.SCS1_ID                     = 2                 # SCServo#1 ID : 1
        #self.SCS2_ID                     = 2                 # SCServo#1 ID : 2
        self.BAUDRATE                    = 1000000           # SCServo default self.BAUDRATE : 1000000
        self.DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

        self.SCS_MINIMUM_POSITION_VALUE  = 0              # SCServo will rotate between this value
        self.SCS_MAXIMUM_POSITION_VALUE  = 4095              # and this value (note that the SCServo would not move when the position value is out of movable range. Check e-manual about the range of the SCServo you use.)
        self.SCS_MOVING_STATUS_THRESHOLD = 20                # SCServo moving status threshold
        self.SCS_MOVING_SPEED            = 0                 # SCServo moving speed
        self.SCS_MOVING_ACC              = 0                 # SCServo moving acc
        self.protocol_end                = 0                 # SCServo bit end(STS/SMS=0, SCS=1)

      
        rospy.init_node('servosubnode')
        self.sub=rospy.Subscriber('servopub',Int16,self.callback)
        rospy.spin()
    def callback(self,data):
        self.servorun(data)
    def servorun(self,data):
        
        scs_goal_position = [data.data]         # Goal position


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
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()


        # Set port self.BAUDRATE
        if portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the self.BAUDRATE")
        else:
            print("Failed to change the self.BAUDRATE")
            print("Press any key to terminate...")
            getch()
            quit()

        # SCServo#1 acc
        scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, self.SCS1_ID, self.ADDR_STS_GOAL_ACC, self.SCS_MOVING_ACC)
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

        

    

        # Allocate goal position value into byte array
        param_goal_position = [SCS_LOBYTE(scs_goal_position[0]), SCS_HIBYTE(scs_goal_position[0])]

        # Add SCServo#1 goal position value to the Syncwrite parameter storage
        scs_addparam_result = groupSyncWrite.addParam(self.SCS1_ID, param_goal_position)
        if scs_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % self.SCS1_ID)
            quit()

        

        # Syncwrite goal position
        scs_comm_result = groupSyncWrite.txPacket()
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()

        
        # Read SCServo#1 present position
        scs1_present_position_speed, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, self.SCS1_ID, self.ADDR_STS_PRESENT_POSITION)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))

        

        scs1_present_position = SCS_LOWORD(scs1_present_position_speed)
        scs1_present_speed = SCS_HIWORD(scs1_present_position_speed)
        
        print("[ID:%03d] GoalPos:%03d PresPos:%03d PresSpd:%03d\t[ID:%03d] GoalPos:%03d PresPos:%03d PresSpd:%03d" 
            % (self.SCS1_ID, scs_goal_position[0], scs1_present_position, SCS_TOHOST(scs1_present_speed, 15)))
            

        # SCServo#1 torque
        scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, self.SCS1_ID, self.self.ADDR_SCS_TORQUE_ENABLE, 0)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))

        # Close port
        #portHandler.closePort()

def main():
    test=sub()
if __name__=='__main__':
    main()