#!/usr/bin/env python

# Node that reads, unpacks and publishes IMU data
# Dynamixel protocol 2.0 is used
# Prerequisites: Dynamixel SDK [https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/noetic-devel]

import rospy
from sensor_msgs.msg import Imu
from dynamixel_sdk import *

# COMMUNICATION PARAMS
PORT = '/dev/ttyUSB1'   # You can use Dynamixel Wizard 2.0
BAUDRATE = 4000000      # to find these parameters
DXL_ID  = 250           #

PROTOCOL_VERSION = 2.0  # IMU-specific params
ADDRESS = 800           # Do not try to tweak
LENGTH  = 12            #

READ_RATE  = 800        # [Hz]. IMU supports up to 
                        # 800 Hz read frequency

# ROS PARAMS
NODE_NAME  = 'imu_node' # Name of this node
FRAME_ID   = 'map'      # Name of frame_id of IMU (See tf2 for more details)
TOPIC_NAME = 'IMU'      # Name of topic to publish into
   
    
# Creates Dynamixel SDK Entities to
# communicate with IMU via Dynamixel Protocol 2.0
def set_up(port, baudrate):
  portHandler = PortHandler(port)
  packetHandler = PacketHandler(PROTOCOL_VERSION)

  try:
      portHandler.openPort()
  except:
      raise RuntimeError("Failed to open the port")

  try:
      portHandler.setBaudRate(BAUDRATE)
  except:
      raise RuntimeError("Failed to change the baudrate")

  return portHandler, packetHandler


# Unpacks two bytes and interpretes
# them as signed int16
def unpack_int16(low, high):
  b = unpack_uint16(low, high)
  sgnmask = (b & (0x1 << 15))
  sgn = sgnmask >> 15
  val  = b & ((0x1 << 15) - 1) 
  sgn = 1 - (2 * sgn)

  if sgn == -1:
    return -1 * ((~val) & ((0x1 << 15) - 1))
  if sgn == 1:
    return val


# Unpacks two bytes and interpretes
# them as unsigned int16
def unpack_uint16(low, high):
  return ((high << 8) + low) & ((0x1 << 16) - 1)


# Unpacks 12-byte package recieved from IMU
def unpack(data):
  t = unpack_uint16(*data[0:2])

  sec  = int(t  * 10**-3)
  nsec = int(t * 10**6) & ((0x1 << 32) - 1)

  x = unpack_int16(*data[2:4]) 
  y = unpack_int16(*data[4:6]) 
  z = unpack_int16(*data[6:8])
  w = unpack_int16(*data[8:10])
  a = unpack_uint16(*data[10:12])

  return (x, y, z, w), (sec, nsec), a


# Forms an sensor_msgs.Imu message
# Params:
#   quat - (x, y, z, w)  tuple-packed quaternion
#   time - (secs, nsecs) tuple-packed time
#   seq  - sequential number of message (see std_msgs.Header)
def imu_msg(quat, time, seq):
  x, y, z, w = quat
  sec, nsec  = time 

  msg = Imu()

  msg.header.seq = seq
  msg.header.stamp.secs  = sec
  msg.header.stamp.nsecs = nsec
  msg.header.frame_id = FRAME_ID

  msg.orientation.x = x
  msg.orientation.y = y
  msg.orientation.z = z
  msg.orientation.w = w

  return msg


def main():
  pub = rospy.Publisher(TOPIC_NAME, Imu, queue_size=10)
  rospy.init_node(NODE_NAME)

  print(f"Opening port {PORT} with baudrate {BAUDRATE}...", end=" ")
  portHandler, packetHandler = set_up(PORT, BAUDRATE)
  print("Done")

  print(f"IMU data is being published to '/{TOPIC_NAME}' topic")
  print(f"Read rate is {READ_RATE} Hz")
  rate = rospy.Rate(READ_RATE)
  seq = 0

  while not rospy.is_shutdown():

    # Read 12 bytes
    data, _, __ = packetHandler.readTxRx(portHandler, DXL_ID, ADDRESS, LENGTH)
    
    # Unpack package
    quat, time, _ = unpack(data)
    
    # Form message
    msg = imu_msg(quat, time, seq)

    # Publish
    pub.publish(msg)
    
    # Increment counter
    seq = seq + 1
    
    # Sleep
    rate.sleep()

if __name__ == '__main__':
    main()