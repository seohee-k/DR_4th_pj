#!/usr/bin/env python3

# ========================== Gripper Class ==========================
from pymodbus.client.sync import ModbusTcpClient as ModbusClient

class RG():
    def __init__(self, gripper, ip, port):
        self.client = ModbusClient(
            ip,
            port=port,
            stopbits=1,
            bytesize=8,
            parity='E',
            baudrate=115200,
            timeout=1)
        if gripper not in ['rg2', 'rg6']:
            print("Please specify either rg2 or rg6.")
            return
        # 그리퍼 넓이
        self.gripper = gripper
        if self.gripper == 'rg2':
            self.max_width = 500
            self.max_force = 400
        elif self.gripper == 'rg6':
            self.max_width = 1600
            self.max_force = 1200
        self.open_connection()

    def open_connection(self):
        self.client.connect()

    def close_connection(self):
        self.client.close()

    def get_fingertip_offset(self):
        result = self.client.read_holding_registers(address=258, count=1, unit=65)
        offset_mm = result.registers[0] / 10.0
        return offset_mm

    def get_width(self):
        result = self.client.read_holding_registers(address=267, count=1, unit=65)
        width_mm = result.registers[0] / 10.0
        return width_mm

    def get_status(self):
        result = self.client.read_holding_registers(address=268, count=1, unit=65)
        status = format(result.registers[0], '016b')
        status_list = [0] * 7
        if int(status[-1]):
            print("A motion is ongoing so new commands are not accepted.")
            status_list[0] = 1
        if int(status[-2]):
            print("An internal- or external grip is detected.")
            status_list[1] = 1
        if int(status[-3]):
            print("Safety switch 1 is pushed.")
            status_list[2] = 1
        if int(status[-4]):
            print("Safety circuit 1 is activated so it will not move.")
            status_list[3] = 1
        if int(status[-5]):
            print("Safety switch 2 is pushed.")
            status_list[4] = 1
        if int(status[-6]):
            print("Safety circuit 2 is activated so it will not move.")
            status_list[5] = 1
        if int(status[-7]):
            print("Any of the safety switch is pushed.")
            status_list[6] = 1
        return status_list

    def get_width_with_offset(self):
        result = self.client.read_holding_registers(address=275, count=1, unit=65)
        width_mm = result.registers[0] / 10.0
        return width_mm

    def set_control_mode(self, command):
        result = self.client.write_register(address=2, value=command, unit=65)

    def set_target_force(self, force_val):
        result = self.client.write_register(address=0, value=force_val, unit=65)

    def set_target_width(self, width_val):
        result = self.client.write_register(address=1, value=width_val, unit=65)

    def close_gripper(self, force_val=400):
        params = [force_val, 0, 16]
        mwait(2)
        print("Start closing gripper.")
        result = self.client.write_registers(address=0, values=params, unit=65)

    def open_gripper(self, force_val=400):
        params = [force_val, self.max_width, 16]
        print("Start opening gripper.")
        result = self.client.write_registers(address=0, values=params, unit=65)

    # 추가한 함수
    def open_gripper2(self, force_val=400):
        params = [force_val, self.max_width//2, 16]
        print("Start opening gripper.")
        result = self.client.write_registers(address=0, values=params, unit=65)

    def move_gripper(self, width_val, force_val=400):
        params = [force_val, width_val, 16]
        print("Start moving gripper.")
        result = self.client.write_registers(address=0, values=params, unit=65)

# ========================== Robot Control Class ==========================

import sys
import os
import time
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
from rclpy.node import Node
import DR_init

from od_msg.srv import SrvDepthPosition
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory

# Constants
package_path = get_package_share_directory("pick_and_place_voice")
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
# 기본 속도값 변경
VELOCITY, ACC = 40, 40
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DEPTH_OFFSET = -5.0
MIN_DEPTH = 2.0

# Initialize Doosan
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
rclpy.init()
dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, trans
    from DSR_ROBOT2 import posx, DR_BASE, DR_MV_MOD_REL, check_force_condition, DR_TOOL, DR_AXIS_Z, task_compliance_ctrl
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

# Gripper Setup
gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

# RobotController Node
class RobotController(Node):
    def __init__(self):
        super().__init__("pick_and_place")
        self.init_robot()
        self.get_logger().info("로봇 위치 초기화 완료")
        time.sleep(1)
        self.open_hole()
        self.get_logger().info("주유구 열기 완료")
        time.sleep(1)
        self.pump_moving()
        self.get_logger().info("주유를 완료하였습니다.")
        time.sleep(1)
        self.get_logger().info("넛지를 위해 기다리는중")
        self.wait_for_nudge_then_comeback()
        time.sleep(2)
        self.card_moving()
        self.get_logger().info("계산이 완료 되었습니다.")
        time.sleep(2)
        self.pick_obstacle()
        self.get_logger().info("안전 운전 하세요.")
        mwait()

        self.get_position_client = self.create_client(SrvDepthPosition, "/get_3d_position")
        while not self.get_position_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_depth_position service...")
        self.get_position_request = SrvDepthPosition.Request()

        self.get_keyword_client = self.create_client(Trigger, "/get_keyword")
        while not self.get_keyword_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_keyword service...")
        self.get_keyword_request = Trigger.Request()

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
        gripper2cam = np.load(gripper2cam_path)
        coord = np.append(np.array(camera_coords), 1)
        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)
        base2cam = base2gripper @ gripper2cam
        td_coord = np.dot(base2cam, coord)
        return td_coord[:3]

    def robot_control(self):
        target_list = []
        self.get_logger().info("call get_keyword service")
        self.get_logger().info("say 'Hello Rokey' and speak what you want to pick up")
        get_keyword_future = self.get_keyword_client.call_async(self.get_keyword_request)
        rclpy.spin_until_future_complete(self, get_keyword_future)
        if get_keyword_future.result().success:
            get_keyword_result = get_keyword_future.result()
            target_list = get_keyword_result.message.split()
            for target in target_list:
                target_pos = self.get_target_pos(target)
                if target_pos is None:
                    self.get_logger().warn("No target position")
                else:
                    self.get_logger().info(f"target position: {target_pos}")
                    self.init_robot()
        else:
            self.get_logger().warn(f"{get_keyword_result.message}")
            return

    def get_target_pos(self, target):
        self.get_position_request.target = target
        self.get_logger().info("call depth position service with object_detection node")
        get_position_future = self.get_position_client.call_async(self.get_position_request)
        rclpy.spin_until_future_complete(self, get_position_future)

        if get_position_future.result():
            result = get_position_future.result().depth_position.tolist()
            self.get_logger().info(f"Received depth position: {result}")
            if sum(result) == 0:
                print("No target position")
                return None

            gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")
            robot_posx = get_current_posx()[0]
            td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)

            if td_coord[2] and sum(td_coord) != 0:
                td_coord[2] += DEPTH_OFFSET
                td_coord[2] = max(td_coord[2], MIN_DEPTH)

            target_pos = list(td_coord[:3]) + robot_posx[3:]
        return target_pos


# ===========================================================================================
# 초기 위치 함수
    def init_robot(self):
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

# 0. 주유구 열기 모션
    def open_hole(self):
        # 차량 상단 위치로 이동
        JZero = [-19.16, 9.85, 77.59, -0.53, 92.05, -3.62]
        gripper.close_gripper()
        #JZero = [416.91, -142.13, 202.04, 114.67, -179.27, 130.28]  # movel용
        movej(JZero,vel=VELOCITY, acc=ACC)
        mwait()
        # 경광등 누르고 올라오기
        movel(posx(0.0, 0.0, -56, 0, 0, 0), vel=100, acc=100, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        movel(posx(0.0, 0.0, 49, 0, 0, 0), vel=40, acc=40, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)


# 1. 주유소에서 주유기 뽑고 주유구 까지 가는 무빙
    def pump_moving(self):
        # 사용자 정의(주유기가 보이는 위치)로 이동
        JOne = [24.07, 40.60, 28.48, -1.66, 111.86, -161.23]
        movej(JOne, vel=VELOCITY, acc=ACC)
        mwait()

        # 주유기 detect했을경우 가는 좌표(테스트용)
        JTwo = [592.74, 136.66, 183.29, 82.59, -178.16, -102.06]
        movel(JTwo, vel=VELOCITY, acc=ACC)
        mwait()

        # 주유기 잡기위한 off_set
        movel(posx(0.0, 60.0, 0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        gripper.open_gripper()
        movel(posx(0.0, 0.0, -82.0 , 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        gripper.close_gripper()
        
        # 주유기 잡고 주유구 쪽으로 이동
        movel(posx(0.0, 0.0, 130.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        movel(posx(0.0, -120.0, 0.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        JThree = [7.73, 37.66, 31.52, -1.85, 111.19, -177.63]
        movej(JThree, vel=VELOCITY, acc=ACC)
        mwait()
        #JThree = [447.17, 30.70, 59.65, 120.58, -172.73, -57.36] # movel용
        JFour = [8.49, 11.99, 98.01,-7.29, 67.40,-166.85]
        movej(JFour, vel=VELOCITY, acc=ACC)
        # 주유기 주유구에 넣기
        movel(posx(0.0, -30.0, 5.0, 0, 0, 0),vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        mwait()


# 2. 주유기 주유구에서 빼고 가져다 놓기
     # 넛지 함수로 트리거
    def wait_for_nudge_then_comeback(self, path_list=None, vel=100, acc=100, threshold=20.0):
            self.get_logger().info("Z축 방향으로 최소 20.0N 이상 누르면 다음 동작을 수행합니다.")
            try:
                while rclpy.ok():
                    if not check_force_condition(axis=DR_AXIS_Z, min=threshold, ref=DR_TOOL):
                        self.get_logger().info("Nudge 감지됨! 후속 동작 수행 중")
                        break
                    time.sleep(0.1)
                movel(posx(0.0, 40.0, 0.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
                # Back to place
                movel(posx(0.0, 0.0, 130.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
                movel(posx(135.0, 0.0, 0.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
                JTwo = [592.74, 196.66, 183.29, 82.59, -178.16, -102.06]
                movel(JTwo, vel=VELOCITY, acc=ACC)
                mwait()
                movel(posx(0.0, 0.0, -80.0 , 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
                gripper.open_gripper()
                mwait()
            except Exception as e:
                print(f"예외 발생: {e}")


# 3. 주유구에서 카드 보이는 위치로 이동
    def card_moving(self):
        movel(posx(0.0, 0.0, 130.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        gripper.close_gripper()
        mwait()

        # 사용자 지정(카드가 보이는 좌표)
        JOne = [-47.17, 45.54, 39.14, 40.73, 109.0, -26.49]
        movej(JOne, vel=15, acc=15)
        mwait() 
        
        # 카드를 yolo detect 했을 경우 가는 좌표(movel로 이동시 급발진)
        JTwo = [560.20, -70.72, 103.25, 65.47, 138.07, 77.18]
        movel(JTwo, vel=VELOCITY, acc=ACC)
        mwait()
        # movej-포기
        # JTwo = []
        # movej(JTwo, vel=VELOCITY, acc=ACC)
        
        # 신용카드와 충돌로인한 경유점
        movel(posx(-30.0, 0.0, 50.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        mwait()

        # 카드를 잡기위한 off_set
        # JThree = [551.48, -58.32, 101.93, 174.24, -180.00, -93.69]  # movel로 이동시 
        JThree = [-6.59, 26.58, 64.77, 0.02, 88.66, 85.44]
        movej(JThree, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()
        movel(posx(0.0, 0.0, -60, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        gripper.close_gripper()
        mwait(3)
        movel(posx(0.0, 0.0, 40.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        # 그리퍼 90도 돌리기
        JFOur = [-6.64, 27.55, 69.19, 0.02, 83.26, 85.38-90]
        movej(JFOur, vel=VELOCITY, acc=ACC)
        mwait()
        gripper.close_gripper()
        
        # 카드 잡고 주유기 쪽으로 이동
        JFive = [548.07, 35.66, 84.82, 86.29, -175.39, 86.74]
        movel(JFive, vel=VELOCITY, acc=ACC)
        mwait()

        # 주유기 앞에서 카드 밀어넣기 off_set
        movel(posx(0.0, 30.0, 0.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        time.sleep(3)
        # 다시 빼기
        movel(posx(0.0, -40.0, 0.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        mwait()
        # 위로올리기 
        movej(JThree, vel=VELOCITY, acc=ACC)
        mwait()
        #내려가기
        movel(posx(0.0, 0.0, -60, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        gripper.open_gripper2()
        time.sleep(2)
        movel(posx(0.0, 0.0, 130.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        mwait()


    # 4. 차단기 치우기
    def pick_obstacle(self):
        JFive=[-14.37, -9.98, 103.61, -7.73, 77.79, -4.27]
        movej(JFive, vel=VELOCITY, acc=ACC)
        mwait()
        Jsix=[-21.26, 1.85, 102.93, 0.00, 75.22, -21.39]
        movej(Jsix, vel=VELOCITY, acc=ACC)
        mwait()

        gripper.close_gripper()
        time.sleep(2)
        movel(posx(0.0, 0.0, 150.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        mwait()

       
def main(args=None):
    node = RobotController()
    while rclpy.ok():
        node.robot_control()
    rclpy.shutdown()
    node.destroy_node()

if __name__ == "__main__":
    main()


