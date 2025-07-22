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
        mwait()
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

import sys, os, time
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
import DR_init
from std_msgs.msg import String, Int32, Bool
# from od_msg.srv import SrvDepthPosition
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory

# Initialize Robot and Gripper
package_path = get_package_share_directory("pick_and_place_voice")
CALIBRATION_NAME = "T_gripper2camera.npy"
CALIBRATION_PATH = os.path.join(package_path, "resource", CALIBRATION_NAME)

ROBOT_ID, ROBOT_MODEL = "dsr01", "m0609"
VELOCITY, ACC = 40, 40
GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT = "rg2", "192.168.1.1", 502

DR_init.__dsr__id, DR_init.__dsr__model = ROBOT_ID, ROBOT_MODEL
rclpy.init()
dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

from DSR_ROBOT2 import movej, movel, mwait, posx, DR_BASE, DR_MV_MOD_REL, check_force_condition, DR_TOOL, DR_AXIS_Z, get_current_posx
gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

class AutoFuelControlNode(Node):
    def __init__(self):
        super().__init__("auto_fuel_control")
        self.fuel_type = None
        self.detected_object = None
        self.stt_success = False
        self.detected_bbox = None

        self.yolo_sub = self.create_subscription(String, 'yolo_result', self.yolo_callback, 10)
        self.fuel_sub = self.create_subscription(Int32, '/oil', self.fuel_callback, 10)
        self.card_sub = self.create_subscription(Bool, '/target/CreditCard', self.card_callback, 10)
        self.status_pub = self.create_publisher(String, '/robot/status', 10)

        self.get_logger().info("자동 주유 로봇 제어 노드 시작됨.")
        self.init_robot()
        self.get_logger().info("로봇 위치 초기화 완료")

    def publish_status(self, message):
        msg = String(); msg.data = message
        self.status_pub.publish(msg)
        self.get_logger().info(f"[STATUS] {message}")

    def init_robot(self):
        movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

# ================= YOLO 콜백 =================
    def _get_depth(self, x, y):
        frame = self._wait_for_valid_data(self.img_node.get_depth_frame, "depth frame")
        try:
            return float(frame[y, x])
        except IndexError:
            self.get_logger().warn(f"Coordinates ({x},{y}) out of range.")
            return None

    def _wait_for_valid_data(self, getter, description):
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            rclpy.spin_once(self.img_node)
            self.get_logger().info(f"Retry getting {description}.")
            data = getter()
        return data

    def _pixel_to_camera_coords(self, x, y, z):
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        ppx = self.intrinsics['ppx']
        ppy = self.intrinsics['ppy']
        return (
            (x - ppx) * z / fx,
            (y - ppy) * z / fy,
            z
        )



    def yolo_callback(self, msg):
        self.get_logger().info("[YOLO] STT 명령이 success.")
        # if self.stt_success: 
        #     return
        
        try:
            parts = msg.data.split()
            if len(parts) != 5:
                self.get_logger().warn(f"YOLO 메시지 형식 오류: {msg.data}") 
                return
            
            class_name, conf = parts[0].lower(), float(parts[1])
            cx, cy, cz = map(int, parts[2:])

            # if class_name == "pump": class_name = "creditcard"
            self.get_logger().info(f"[YOLO] {class_name} 감지됨 (conf={conf:.2f})")

            if conf < 0.5: 
                return
            
            self.detected_bbox = (cx, cy, cz)

            
            if class_name == "pump": 
                self.pump_moving()
                mwait()
                self.wait_for_nudge_then_comeback()

            elif class_name == "creditcard": 
                self.card_moving()
                mwait()
                self.move_stopsign()

        except Exception as e:
            self.get_logger().error(f"YOLO 콜백 예외: {e}")

    # =============== 연료 종류 콜백 ===============

    def fuel_callback(self, msg):
        self.fuel_type = msg.data
        self.get_logger().info(f"[음성] 연료 인식됨: {'경유' if msg.data == 0 else '휘발유'}")
        self.execute_fuel_task()


    def execute_fuel_task(self):
        if self.fuel_type == 0:
            self.get_logger().info("[TASK] 경유 주유 동작 실행")
        elif self.fuel_type == 1:
            self.get_logger().info("[TASK] 휘발유 주유 동작 실행")
        self.open_hole()
        mwait()
        self.pump_moving()
        mwait()
        self.wait_for_nudge_then_comeback()
        mwait()

    def card_callback(self, msg):
        if msg.data:
            self.get_logger().info("[음성] 카드 결제 감지됨 → 카드 삽입 동작 실행")
            self.card_moving()
            mwait()
            self.move_stopsign()
            mwait()



# 0. 주유구 열기 모션
    def open_hole(self):
        # 차량 상단 위치로 이동
        movej([-19.16, 9.85, 77.59, -0.53, 92.05, -3.62], vel=VELOCITY, acc=ACC)
        gripper.close_gripper()
        mwait()
        # 경광등 누르고 올라오기
        movel(posx(0, 0, -56, 0, 0, 0), vel=100, acc=100, ref=DR_BASE, mod=DR_MV_MOD_REL)
        movel(posx(0, 0, 49, 0, 0, 0), vel=40, acc=40, ref=DR_BASE, mod=DR_MV_MOD_REL)
        self.get_logger().info(f"주유구 오픈")

    # 1. 주유소에서 주유기 뽑고 주유구 까지 가는 무빙
    def pump_moving(self):
        # 사용자 정의(주유기가 보이는 위치)로 이동
        JOne = [24.07, 40.60, 28.48, -1.66, 111.86, -161.23]
        movej(JOne, vel=VELOCITY, acc=ACC)
        mwait()
        # X축 방향 -1cm 이동 (기준: 로봇 base 좌표계)
        movel(posx(20.0, -10.0, 50.0, 0, 0, 0), vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)


        # yolo bounding box
        if self.detected_bbox:
            robot_base = get_current_posx()[0]
            obj_pos = self.tf.obj_pose_in_base(robot_base, self.detected_bbox)
            movel(obj_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
            time.time(3)

        else:
            self.get_logger().warn("[경고] YOLO 좌표가 없어 기본 위치로 이동합니다.")
        
            # 주유기 yolo detect 했을경우 가는 좌표
            JTwo = [592.74, 136.66, 183.29, 82.59, -178.16, -102.06]
            movel(JTwo, vel=VELOCITY, acc=ACC)
            mwait()

            # 주유기 잡기위한 offset
            movel(posx(0.0, 60.0, 0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
            gripper.open_gripper()
            movel(posx(0.0, 0.0, -82.0 , 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
            gripper.close_gripper()
                
            # 주유기 잡고 주유구 쪽으로 이동
            movel(posx(0.0, 0.0, 130.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
            movel(posx(0.0, -120.0, 0.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
            JThree = [7.73, 37.66, 31.52, -1.85, 111.19, -177.63]
            movej(JThree, vel=VELOCITY, acc=ACC)

            JFour = [8.49, 11.99, 98.01,-7.29, 67.40,-166.85]
            movej(JFour, vel=VELOCITY, acc=ACC)

            # 주유기 주유구에 넣기
            movel(posx(0.0, -30.0, 5.0, 0, 0, 0),vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)


# 2. 주유기 주유구에서 빼고 가져다 놓기
     # 넛지 함수
    def wait_for_nudge_then_comeback(self, path_list=None, vel=100, acc=100, threshold=20.0):
            self.get_logger().info("Z축 방향으로 최소 20move.0N 이상 누르면 다음 동작을 수행합니다.")
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

        if self.detected_bbox:
            robot_base = get_current_posx()[0]
            obj_pos = self.tf.obj_pose_in_base(robot_base, self.detected_bbox)
            movel(obj_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
            time.time(3)

        else:
            self.get_logger().warn("[YOLO] 감지된 카드 좌표 없음 → 기본 위치로 이동")

        # 카드 pick 좌표(movel로 이동시 급발진)
            # JTwo = [560.20, -70.72, 103.25, 65.47, 138.07, 77.18]
            JTwo = [560.20, -70.72, 103.25, 65.47, 138.07, 77.18]
            movel(JTwo, vel=VELOCITY, acc=ACC)
            mwait()

            # 신용카드와 충돌로인한 경유점
            movel(posx(-30.0, 0.0, 50.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
            mwait()

            # 카드를 잡기위한 off_set
            JThree = [-6.59, 26.58, 64.77, 0.02, 88.66, 85.44]
            movej(JThree, vel=VELOCITY, acc=ACC)

            gripper.open_gripper()
            mwait()

            movel(posx(0.0, 0.0, -60, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
            gripper.close_gripper()
            mwait()

            movel(posx(0.0, 0.0, 40.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
            
            # 그리퍼 90도 돌리기        
            JFour = [-6.64, 27.55, 69.19, 0.02, 83.26, 85.38-90]
            movej(JFour, vel=VELOCITY, acc=ACC)
            mwait()

            gripper.close_gripper()
            
            # 카드 잡고 주유기 쪽으로 이동
            JFive = [548.07, 35.66, 84.82, 86.29, -175.39, 86.74]
            movel(JFive, vel=VELOCITY, acc=ACC)
            mwait()

            # 주유기 앞에서 카드 밀어넣기 off_set
            movel(posx(0.0, 30.0, 0.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
            time.sleep(3)
            self.publish_status("카드 결제가 완료되었습니다.")
            
            # 다시 빼기
            movel(posx(0.0, -40.0, 0.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
            mwait()
            
            # 위로올리기 
            movej(JThree, vel=VELOCITY, acc=ACC)
            mwait()
            
            #내려가기
            movel(posx(0.0, 0.0, -60, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
            gripper.open_gripper2()
            mwait()
            movel(posx(0.0, 0.0, 130.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)

# pick and place stopsign
    def move_stopsign(self):
        JFive=[-14.37, -9.98, 103.61, -7.73, 77.79, -4.27]
        movej(JFive, vel=VELOCITY, acc=ACC)

        Jsix=[-21.26, 1.85, 102.93, 0.00, 75.22, -21.39]
        movej(Jsix, vel=VELOCITY, acc=ACC)
        gripper.close_gripper()
        self.publish_status("차단기가 올라가는중...")
        time.sleep(2)

        movel(posx(0.0, 0.0, 150.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        mwait()
        self.publish_status("좋은 하루되세요.")


def main():
    node = AutoFuelControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
