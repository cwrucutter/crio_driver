#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Matthew Klein
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the names of the authors nor the names of their
# affiliated organizations may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
from snowmower_msgs.msg import EncMsg

import serial,struct

class crio_driver(object):
  def __init__(self):
    # ROS init node
    rospy.init_node('crio_driver')
    # Listen for a /std_msgs/UInt8 on the topic /plow/angle
    rospy.Subscriber('/plow/angle', UInt8, self.plowAngleSubCB)
    # Listen for a /geometry_msgs/Twist on the topic /cmd_vel
    rospy.Subscriber('/cmd_vel', Twist, self.vwSubCB)
    # Publish the encoder counts using a snowmower_msgs/EncMsg on the topic enc
    self.encPub = rospy.Publisher('enc',EncMsg,queue_size=1)
    self.encMsg = EncMsg()

    # Set some parameters for the serial port (port and baudrate)
    crioPort = rospy.get_param('~port','/dev/ttyUSB2')
    crioBaudrate = rospy.get_param('~baud',57600)
    # and initialize the serial port (we shoudl add close() somwher in here)
    self.crioSerial = serial.Serial(port=crioPort,baudrate=crioBaudrate,timeout=1.0)

    # Initialize parameters
    self.velocity = 0
    self.omega = 0
    self.plowAngle = 0

  def vwSubCB(self, msg):
    # Convert m/s to mm/s
    self.velocity = msg.linear.x*1000.0
    self.omega = msg.angular.z*1000.0
    # Limit the velocity command to 2 m/s
    if self.velocity > 2000:
      self.velocity = 2000
      rospy.logwarn("Velocity command outside limit (%s m/s). Set to -2 m/s."%msg.linear.x)
    if self.velocity < -2000:
      self.velocity = -2000
      rospy.logwarn("Velocity command outside limit (%s m/s). Set to -2 m/s."%msg.linear.x)
    # Calculate the checksum
    checksum = (sum(bytearray(struct.pack('<BBBBhhB',170,160,5,1,int(round(self.velocity)),int(round(self.omega)),self.plowAngle))) % 256) & 127
    # Put together the entire serialized packet
    serialMsg = struct.pack('<BBBBhhBBB',170,160,5,1,int(round(self.velocity)),int(round(self.omega)),self.plowAngle,checksum,13)
    # and send the message!
    self.crioSerial.write(serialMsg)

    # display sent data to screen
    rospy.loginfo("Sent: (%s,%s,%s)",int(round(self.velocity)),int(round(self.omega)),self.plowAngle)


  # This function subscribes to the /plow/angle topic and saves the UInt8 as a global variable. This is sent to the cRIO over serial in the vwSubCB function.
  def plowAngleSubCB(self, msg):
    self.plowAngle = msg.data


  # This function contains an infinite loop which read from the serial port connected to the cRIO and publishes the status and encoder readings
  def run(self):
    sync0 = '\x00'; sync1 = '\x00';# sync2 = '\x00';
    while not rospy.is_shutdown():
      # READ UNTIL SYNC
      data = self.crioSerial.read(1)
      sync0 = sync1;
      sync1 = data;
      sync = sync0+sync1;
      match = '\xA0\xAA'
      if sync != match:
        continue
      # else:
        # rospy.loginfo("Beginning new message")

      msgLen = struct.unpack('<B',self.crioSerial.read(1))[0]
      msgType = struct.unpack('<B',self.crioSerial.read(1))[0]
      if msgType != 0:
        rospy.logwarn("Packet Failed: Message type not correct")
        continue

      data = self.crioSerial.read(msgLen)
      if (len(data) != msgLen):
        rospy.logwarn("Packet Failed: Message length unexpected")
        continue
      data1 = struct.unpack("<BBii",data)

      checksum = struct.unpack("<B",self.crioSerial.read(1))[0]
      checksum1 = ((sum((sum(struct.unpack("<BB",sync0+sync1)),msgLen,msgType,sum(struct.unpack("<BBBBBBBBBB",data)))) % 256) & 127)

      if checksum != checksum1:
        rospy.logwarn("Packet Failed: Checksum did not match")
        continue

      stop = struct.unpack("<B",self.crioSerial.read(1))[0]
      if stop != 13:
        rospy.logwarn("Packet Failed: Incorrect stop bit")
        continue
      
      # TODO: publish crio status
      # Populate the encoder message (right and left) with data from the crio
      self.encMsg.right = data1[2]
      self.encMsg.left = data1[3]
      # publish the data
      self.encPub.publish(self.encMsg)
      

      # display recieved info to screen
      rospy.loginfo("Recieved: (%s,%s,%s,%s)",data1[0],data1[1],data1[2],data1[3])

if __name__ == "__main__":
  cd = crio_driver()
  cd.run()
