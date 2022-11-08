#! /usr/bin/env python

'''
struct.pack(format, v1, v2, ...)¶
v1, v2값을 포함하고 포맷 문자열 format에 따라 패킹 된 바이트열 객체를 반환합니다
인자는 포맷이 요구하는 값과 정확히 일치해야 합니다.
'''
from struct import pack
import rospy
from erp_driver.msg import erpStatusMsg, erpCmdMsg
'''
erpStatusMsg

int8 control_mode
bool e_stop
uint8 gear
uint8 speed
int32 steer
uint8 brake
int32 encoder
uint8 alive

erpCmdMsg

bool e_stop
uint8 gear
uint8 speed
int32 steer
uint8 brake
'''
import serial
import numpy as np

from ByteHandler import ErpMsg2Packet, Packet2ErpMsg
'''
def Packet2ErpMsg(_byte: bytes) -> erpStatusMsg:
    formated_packet = struct.unpack('<BBBBBBhhBiBBB', _byte)
    msg = erpStatusMsg()
    msg.control_mode = formated_packet[3]
    msg.e_stop = bool(formated_packet[4])
    msg.gear = formated_packet[5]
    msg.speed = formated_packet[6]
    msg.steer = -formated_packet[7]
    msg.brake = formated_packet[8]
    msg.encoder = np.int32(formated_packet[9])
    msg.alive = formated_packet[10]

    return msg


def ErpMsg2Packet(_msg: erpCmdMsg, _alive: np.uint8) -> list:
    header = "STX".encode()
    tail="\r\n".encode()

    data = struct.pack(
        ">BBBHhBB", 1,
        _msg.e_stop,
        _msg.gear,
        _msg.speed,
        _msg.steer,
        _msg.brake,
        _alive
    )
    packet = header + data + tail

    return packet
'''

'''
SEND BYTES
┌─────┬─────┬─────┬─────┬─────┬──────┬───────┬───────┬────┬─────┬──────┬──────┐
│  S  │  T  │  X  │ A,M │  E  │ Gear │ Speed │ Steer │ Br │ Al  │ ETX0 │ ETX1 │
├─────┼─────┼─────┼─────┼─────┼──────┼───────┼───────┼────┼─────┼──────┼──────┤
│0x53 │0x54 │0x58 │0 , 1│0 , 1│0,1,2 │0 ~ 200│ ±2000 │1~33│0~255│ 0x0D │ 0x0A │
└─────┴─────┴─────┴─────┴─────┴──────┴───────┴───────┴────┴─────┴──────┴──────┘

RECV BYTES
┌─────┬─────┬─────┬─────┬─────┬──────┬───────┬───────┬────┬───────┬─────┬──────┬──────┐
│  S  │  T  │  X  │ A,M │  E  │ Gear │ Speed │ Steer │ Br │  ENC  │ Al  │ ETX0 │ ETX1 │
├─────┼─────┼─────┼─────┼─────┼──────┼───────┼───────┼────┼───────┼─────┼──────┼──────┤
│0x53 │0x54 │0x58 │0 , 1│0 , 1│0,1,2 │0 ~ 200│ ±2000 │1~33│ ±2^31 │0~255│ 0x0D │ 0x0A │
└─────┴─────┴─────┴─────┴─────┴──────┴───────┴───────┴────┴───────┴─────┴──────┴──────┘
'''

START_BITS = "535458"

class ERPHandler:
    def __init__(self) -> None:
        rospy.init_node("erp_base")
        _port = rospy.get_param("/erp_base/port")
        _baudrate = rospy.get_param("/erp_base/baudrate")
        rospy.loginfo("erp_base::Uart Port : %s", _port)
        rospy.loginfo("erp_base::Baudrate  : %s", _baudrate)

        self.seri = serial.Serial(port=_port, baudrate=_baudrate)
        rospy.loginfo("Serial %s Connected", _port)
        self.alive = 0
        self.packet = erpCmdMsg()
        self.packet.e_stop = False #0x00
        self.packet.gear = 0
        self.packet.speed = 0 # 10 => 1 KPH 50 => 5 KPH
        self.packet.steer = 0
        self.packet.brake = 1


        self.erpMotionMsg_pub = rospy.Publisher(
            "/erp42_status",
            erpStatusMsg,
            queue_size=3
        ) # def


        self.erpCmdMsg_sub = rospy.Subscriber(
            "/erp42_ctrl_cmd",
            erpCmdMsg,
            self.sendPacket
        ) # def



    def recvPacket(self) -> None:
        packet = self.seri.read(18)

        if not packet.hex().find(START_BITS) == 0:  #16
            end, data = packet.hex().split(START_BITS)
            packet = bytes.fromhex(START_BITS + data + end)
            self.erpMotionMsg_pub.publish(
                Packet2ErpMsg(packet)
            )
            rospy.loginfo(Packet2ErpMsg(packet))
            # print(packet)

        else:

            self.erpMotionMsg_pub.publish(
                 Packet2ErpMsg(packet))
            rospy.loginfo(Packet2ErpMsg(packet))
            # # print(packet)

    def sendPacket(self, _data: erpCmdMsg) -> None:
        self.packet = _data
        # print('111111111111111111111111111111111')
        #print(self.packet)


    def serialSend(self) -> None:
        packet = ErpMsg2Packet(self.packet, self.alive)
        self.seri.write(packet)
        self.alive += 1
        if self.alive == 256:
            self.alive = 0

    # def spdcallback(data):
    # # print(data)
    #     ERPHandler(10)
    # # ErpSerialHandler.packet.speed = 30

if __name__ == "__main__":
    ehandler = ERPHandler()
    rate = rospy.Rate(100)

    while rospy.is_shutdown() is False:
        # print(1)

        ehandler.recvPacket()
        ehandler.serialSend()
        # print("===========================")
        # print("e_stop: ",ehandler.packet.e_stop)
        # print("geer: ",ehandler.packet.gear)
        # print("speed: ",ehandler.packet.speed // 10)
        # print("steer: ",ehandler.packet.steer // 71)
        # print("brake: ",ehandler.packet.brake)
        rate.sleep()
