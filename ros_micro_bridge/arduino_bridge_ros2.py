# Connection between ROS and Serial
# Author_1: Matthijs Roebroek
# Date: 4 December 2019
#
# Author_2: V. Garofano
# Date : Feb2022
#
# Author_3: C. Cromjongh
# Last Update : dec 19 2022
#
# Author_4: B. Boogmans
# Last Update : 10 january 2024 - Dedicate only ros2 elements, simplify code structure to single file (with only 1 ros2 version supported) and integrate IMU streaming

# Description: The script is the bridge between the Control boards on the model ships and control decision-making
# entities (e.g. a controller or a human) It uses ROS to connect with the controller, via a Raspberry Pi or a NUC,
# and a serial communication protocol to send control input to the Main Arduino on board of the ship and receive
# diagnostics.

import rclpy
from rclpy.node import Node

import time

from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import Imu, JointState

import serial
import math
import numpy as np

import ros_micro_bridge.global_state as globs
from ros_micro_bridge.global_state import GlobalState, Diagnostics
from ros_micro_bridge.HeadingUtils import HeadingStateEst
from ros_micro_bridge.serial_utils import MsgReceiver, reset_arduino, open_serial, auto_detect_serial

ARDUINO_RUNTIME_TIMEOUT = 1000 * 1E6  # 1000 ms / 1 second
ARDUINO_INIT_TIMEOUT = 6000 * 1E6  # 6000 ms / 6 seconds

ACTUATION_PRIO_MSG_TIMEOUT = 3000 *1E6 # period after last actuation_prio message when normal actuation messages are accepted

DIAGNOSTICS_LOOP_FREQ_TRACKER_TIMEOUT = 2000 * 1E6
DIAGNOSTICS_REPORT_TIMEOUT = 2 * 1E9  # nanoseconds


def euler_to_quaternion(roll, pitch, yaw):
	"""
	Converts euler angles to quaternions
	:param: roll, pitch, yaw = euler angles in radians
	:return: quaternion
	"""
	qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
	qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
	qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	return [qx, qy, qz, qw]


def get_value_from_jointstate(message:JointState,name_item:str,paramtype:int=0):
	""" looks through the jointstate messsage for the specified item 
	
	:param: message = JointState message
	:param: name_item = name of the item to look for
	:param: paramtype = 0 for position, 1 for velocity, 2 for effort
	
	"""
	for i in range(0, len(message.name) ):
		if message.name[i] == name_item:
			# The specified item has been found in the list. 
			# Return the value according to the specified paramtype
			if paramtype == 0:
				print(f"Position of {name_item} found in jointstate message with value {message.position[i]} at position {i}")
				return message.position[i]
			elif paramtype == 1:
				return message.velocity[i]
			elif paramtype == 2:
				return message.effort[i]
			else:
				# In case the specified paramtype is not valid
				return np.nan
	
	# In case the specified item is not in the list
	print(f"Item {name_item} not found in jointstate message")
	return np.nan

class Ros2ArduinoBridge(Node):
    def __init__(self, *args, **kwargs):
        print("Ros_ArduinoBridge __init__ entering")
        self.pub_telemetry = None
        self.pub_heading = None
        self.last_telem_message = 0
        self.last_actuation_reference_prio_message = 0
        super().__init__('ros_arduino_bridge', *args, **kwargs)
        self.actuation = [0.0,0.0,0.0,0.0,0.0]

        # If namespace is empty, throw warning. Common ones are e.g. RAS_TN_LB, RAS_TN_YE, RAS_GS, RAS_DELFIA_01 
        if self.get_namespace() == "":
            self.get_logger().warn("Namespace is empty. This is not expected behavior.")
            self.namespace_prefix = ""
        else:
            self.namespace_prefix = self.get_namespace()


        # Define parameter enable_microprocessor_serial_streaming
        self.declare_parameter('enable_stream_microprocessor_serial', True)
        self.declare_parameter('enable_stream_imu', True)
        self.declare_parameter('enable_stream_heading', True)


        # Not setting the port here, we want to open the port manually later
        self._ser = serial.Serial(None, 115200, timeout=1)

        self.msg_receiver_serial = MsgReceiver(self._ser)
        self.heading_state_estimator = HeadingStateEst(4, 1*math.pi)

        # Automatically try to detect the serial port and open it
        self._ser.port = auto_detect_serial(self)
        open_serial(self._ser)

        # LogInfo the port we are using
        self.get_logger().info(f"Using serial port: {self._ser.name}")

        self.diagnostics = Diagnostics()

        self.get_logger().info("Start setting up subscribers and publishers")
        self.sub_reference = self.create_subscription(JointState, f'reference/actuation', self.callback_control, 10)
        self.sub_reference_prio = self.create_subscription(JointState, f'reference/actuation_prio', self.callback_control_prio, 10)

        if self.get_parameter('enable_stream_microprocessor_serial').value:
            self.pub_telemetry = self.create_publisher(Float32MultiArray, f'telemetry/micro_serial_stream', 10)

        if self.get_parameter('enable_stream_heading').value:
            self.pub_heading = self.create_publisher(Float32, f'state/yaw', 10)

        if self.get_parameter('enable_stream_imu').value:
            self.pub_imu = self.create_publisher(Imu,'telemetry/imu',10)

        self.get_logger().info("Done setting up subscribers and publishers")
        self.get_logger().info("Running main loop now")

        """
        End of initialisation, start of program
        """

        # track last message timestamp for timeout detection
        self.last_telem_message = time.time_ns()

        self.timer = self.create_timer(1/2000, self.loop_body)

    def exit_loop(self):
        self.timer.destroy()
        self.destroy_node()
        rclpy.shutdown()
        exit()

    def loop_body(self):
        """
        Main functionality of the program. Runs in a (callback/timer)loop.
        """
        self.diagnostics.evaluate()  # Check period of several parts of the program.
        serial_msg = self.msg_receiver_serial.run()  # Collect serial data from Arduino.

        # If there is a message (None otherwise)...
        if serial_msg:
            # handle telemetry message (parse text, publish on ROS)
            self.process_serial_message(serial_msg)

        # Resetting Arduino after message timeout
        elif globs.global_state == GlobalState.ready and time.time_ns() - self.last_telem_message > ARDUINO_RUNTIME_TIMEOUT:
            print(f"[WARN] Connection with Arduino seems lost "
                  f"(no message in {ARDUINO_RUNTIME_TIMEOUT / 1E6} ms), resetting.")
            self.last_telem_message = time.time_ns()
            reset_arduino(self._ser)

        # Resetting Arduino after first message connect timeout
        elif globs.global_state == GlobalState.waiting and time.time_ns() - self.last_telem_message > ARDUINO_INIT_TIMEOUT:
            print(f"[WARN] Did not get first message from Arduino in {ARDUINO_INIT_TIMEOUT / 1E6} ms, resetting.")
            self.last_telem_message = time.time_ns()
            reset_arduino(self._ser)

        # Program shutdown
        if globs.global_state == GlobalState.shuttingDown:
            print("[INFO] Shutdown Arduino and Python")
            self.exit_loop()
            return False

        return True

    def callback_control(self, control_input: JointState):
        """
        Control message callback (regular traffic)
        Actuation references passed through this callback have secondary priority under callback_control_prio 

        :param control_input: Control inputs array
        :return:
        """
        # Process the message if enough time has passed since last priority actuation message
        if time.time_ns() - self.last_actuation_reference_prio_message > ACTUATION_PRIO_MSG_TIMEOUT:
            self.process_actuator_reference(control_input)
        
    def callback_control_prio(self, control_input: JointState):
        """
        Control message callback (priority traffic)
        Actuation references passed through this callback have priority over the default actuation topic

        :param control_input: Control inputs array
        :return:
        """
        # Set the timestamp for priority channel timeout
        self.last_actuation_reference_prio_message = time.time_ns()

        # Process the message
        self.process_actuator_reference(control_input)

    def process_actuator_reference(self, control_input: JointState):
        """
        Process actuation referencemessage

        :param control_input: Control inputs in JointState format
        :return:
        """
        # Don't send control data yet when we don't have contact yet.
        if globs.global_state != GlobalState.ready:
            return
        # The control input coming from ROS : s{ps_angle};{stb_angle};{ps_rpm};{stb_rpm};{bt_rpm}
        # s{reference_rps_P};{reference_rps_SB};{reference_rps_bow};{inputAngle_P};{inputAngle_SB}\n
        
        '''
        for i in range(len(control_input.data)):
            if not math.isnan(control_input.data[i]):
                self.actuation[i] = control_input.data[i]
        '''
        sb_aft_propeller = get_value_from_jointstate(control_input,'SB_aft_thruster_propeller',paramtype:=1)# rpm
        ps_aft_propeller = get_value_from_jointstate(control_input,'PS_aft_thruster_propeller',paramtype:=1)# rpm
        bow_thruster = get_value_from_jointstate(control_input,'BOW_thruster_propeller',paramtype:=1) # normalized

        sb_aft_angle = get_value_from_jointstate(control_input,'SB_aft_thruster_joint',paramtype:=0) # radians
        ps_aft_angle = get_value_from_jointstate(control_input,'PS_aft_thruster_joint',paramtype:=0) # radians

        # Print angles
        print(f"Angles unpacked from rosmsg: {sb_aft_angle}, {ps_aft_angle}")

        if not math.isnan(sb_aft_propeller):
            self.actuation[0] = sb_aft_propeller
        if not math.isnan(ps_aft_propeller):
            self.actuation[1] = ps_aft_propeller
        if not math.isnan(bow_thruster):
            self.actuation[2] = bow_thruster
        if not math.isnan(sb_aft_angle):
            self.actuation[3] = sb_aft_angle
        if not math.isnan(ps_aft_angle):
            self.actuation[4] = ps_aft_angle

        # Print actuation
        print(f"Actuation unpacked from rosmsg: {self.actuation}")

        control_str = f"r{self.actuation[0]:.0f};{self.actuation[1]:.0f};{self.actuation[2]:.2f};{math.degrees(self.actuation[3]):.0f};{math.degrees(self.actuation[4]):.0f}\n"
        print(f"Control string: {control_str}")

        self._ser.write(control_str.encode('utf-8'))
        self.diagnostics.track_num_actuation += 1

    def callback_telemetry(self, message: str):
        """
        Get data (numbers) from string message and send it over ROS

        :param message: Message string received from serial.
        :return:
        """

        parsed_nums = []
        try:
            nums = message.split(";")
            parsed_nums = [float(x) for x in nums]
        except:
            print(f"callback_telemetry failed to process serial_message '{message}'")

        # Publish telemetry data
        if self.get_parameter('enable_stream_microprocessor_serial').value:
            self.publish_serial_telemetry(parsed_nums)

        # Publish IMU data
        if self.get_parameter('enable_stream_imu').value:
            self.publish_imu(parsed_nums)

        # Publish heading data
        if self.get_parameter('enable_stream_heading').value:
            self.publish_heading(parsed_nums)

    def publish_serial_telemetry(self, parsed_nums):
        """
        Send the data received from the serial port to ROS
        """

        msg_telemetry = Float32MultiArray()
        msg_telemetry.data = parsed_nums
        self.pub_telemetry.publish(msg_telemetry)
    
    def publish_imu(self, parsed_nums):
        """
        Publish IMU data to ROS

        :param imu_data: List of IMU data
        :return:
        """
        try:
            # Create and send ros imu
            msg_imu = Imu()
            msg_imu.header.stamp = self.get_clock().now().to_msg()
            msg_imu.header.frame_id = self.namespace_prefix+'imu0'
            msg_imu.orientation_covariance = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
            msg_imu.angular_velocity_covariance = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
            msg_imu.linear_acceleration_covariance = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
            # Big note about the following line: it is assumed that the roll and pitch are zero. In future work the roll and pitch should be estimated from the accelerometer data
            heading = self.heading_state_estimator.process_magnetometer_data(parsed_nums[16], parsed_nums[17])
            quats = euler_to_quaternion(0.0,0.0,heading)
            msg_imu.orientation.x = quats[0]
            msg_imu.orientation.y = quats[1]
            msg_imu.orientation.z = quats[2]
            msg_imu.orientation.w = quats[3]

            # Insert linear acceleration data from index 10-12 of telemetry [m/s^2]
            msg_imu.linear_acceleration.x = parsed_nums[10]
            msg_imu.linear_acceleration.y = parsed_nums[11]
            msg_imu.linear_acceleration.z = parsed_nums[12]

            # Insert angular velocity data from index 13-15 of telemetry [rad/s]
            msg_imu.angular_velocity.x = parsed_nums[13] 
            msg_imu.angular_velocity.y = parsed_nums[14]
            msg_imu.angular_velocity.z = parsed_nums[15]

            self.pub_imu.publish(msg_imu)

        except Exception as e:
            print(f"[WARN] callback_telemetry failed to interpret and/or send heading angle or IMU msg from sensordata '{[parsed_nums[16], parsed_nums[17]],e}'")
    
    def publish_heading(self, parsed_nums):
        """
        Publish heading data to ROS
        """

        try:   
            # estimate heading and publish
            msg_heading = Float32()
            msg_heading.data = self.heading_state_estimator.process_magnetometer_data(parsed_nums[16], parsed_nums[17])
            # index 16 and 17 of telemetry refer to x and y potential field measured by the BNO055 magnetometer
            self.pub_heading.publish(msg_heading)
        except Exception as e:
            self.get_logger().warn(f"callback_telemetry failed to interpret and/or send heading angle '{[parsed_nums[16], parsed_nums[17]],e}'")

    def process_serial_message(self, message: str):
        """
        Evaluate serial message for meaning (e.g. telemetry, warning, or info) and resolve afterwards
        :param message: Message string received from serial.
        :return:
        """
        # Telemetry message
        if message[0] == 't':
            if globs.global_state == GlobalState.waiting:
                # Display if we successfully received a message after (re)set.
                globs.global_state = GlobalState.ready
                print("[INFO] Received first telemetry message, good to go.")

            self.last_telem_message = time.time_ns()
            self.diagnostics.track_num_telemetry += 1
            self.callback_telemetry(message[2:])

        # Arduino system message
        elif message[0] == '[':
            print(f"[Arduino] {message}")

        # Error: Unexpected message
        else:
            print(f"Unidentified serial message '{message}'")

def main(args=None):
    # We cannot use the rospy logger before the node is initialized.
    print("Initializing ROS node...")
    rclpy.init(args=args)

    bridge = Ros2ArduinoBridge()

    print("Done initializing ROS node")

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        print('\nNode stopped cleanly')
    except BaseException:
        import sys
        print('Exception in node:', file=sys.stderr)
        raise
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
