import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool
import threading
import tkinter as tk
from PIL import Image, ImageTk
import cv2
from ultralytics import YOLO

class TriggerNode(Node):
    def __init__(self):
        super().__init__('trigger_node')
        self.status_message = "대기 중..."
        self.diesel_message = "대기 중..."
        self.gasoline_message = "대기 중..."
        self.pump_message = "대기 중..."
        self.card_message = "대기 중..."
        self.obstacle_message = "대기 중..."
        # self.payment_total = 0
        # self.total_payment = 0

        # === sub =============================
        self.create_subscription(String, '/robot/status', self.status_callback, 10)
        self.create_subscription(Int32, '/oil', self.diesel_callback, 10)
        self.create_subscription(Int32, '/oil', self.gasoline_callback, 10)
        self.create_subscription(Bool, '/target/Pump', self.pump_callback, 10)
        self.create_subscription(Bool, '/target/Card', self.card_callback, 10)
        self.create_subscription(Bool, '/target/Obstacle', self.obstacle_callback, 10)
        # ==== pub ==========================

    def status_callback(self, msg):
        self.get_logger().info(f"[로봇 상태] {msg.data}")
        self.status_message = msg.data

    def diesel_callback(self, msg):
        if msg.data == 0:
            self.diesel_message = "🚚 경유 감지됨: 경유 주유 시작 중..."

    def gasoline_callback(self, msg):
        if msg.data == 1:
            self.gasoline_message = "⛽ 휘발유 감지됨: 휘발유 주유 시작 중..."

    def pump_callback(self, msg):
        if msg.data:
            self.pump_message = "🛠️ 주유기 감지됨: 주유기 접근 중..."

    def card_callback(self, msg):
        if msg.data:
            self.card_message = "💳 카드 감지됨: 결제 시도 중..."

    def obstacle_callback(self, msg):
        if msg.data:
            self.obstacle_message = "⚠️ 장애물 감지됨: 경로 재계획 중..."

    def payment_callback(self, msg):
        amount = msg.data  # 수신된 한 번의 결제 금액
        self.add_payment(amount)



def start_gui(trigger_node: TriggerNode):
    root = tk.Tk()
    root.title("⛽ ROS2 주유소 GUI")
    root.geometry("1280x720")  # 화면 크기 확장 (영상 표시 위해)

    main_frame = tk.Frame(root, width=1280, height=720)
    main_frame.pack(fill="both", expand=True)

    # 배경 이미지
    bg_image = Image.open("/home/rokey/Downloads/gas_station.png").resize((1280, 720))
    bg_photo = ImageTk.PhotoImage(bg_image)

    canvas = tk.Canvas(main_frame, width=1280, height=720, highlightthickness=0)
    canvas.place(x=0, y=0)
    canvas.create_image(0, 0, image=bg_photo, anchor="nw")

    buttons = {}

    left_sections = [
        ("로봇 상태", trigger_node.status_message, 20),
        ("경유 상태", trigger_node.diesel_message, 80),
        ("휘발유 상태", trigger_node.gasoline_message, 140),
    ]

    for title, initial_text, y in left_sections:
        btn = tk.Button(main_frame, text=f"{title}: {initial_text}",
                        font=("Arial", 12, "bold"), bg="#444444", fg="white",
                        activebackground="#666666", relief="raised", bd=3, anchor="w")
        btn.place(x=20, y=y, width=360, height=50)
        buttons[title] = btn

    right_sections = [
        ("주유기 상태", trigger_node.pump_message, 20),
        ("카드 상태", trigger_node.card_message, 80),
        ("장애물 상태", trigger_node.obstacle_message, 140),
    ]
    for title, initial_text, y in right_sections:
        btn = tk.Button(main_frame, text=f"{title}: {initial_text}",
                        font=("Arial", 12, "bold"), bg="#444444", fg="white",
                        activebackground="#666666", relief="raised", bd=3, anchor="w")
        btn.place(x=420, y=y, width=360, height=50)
        buttons[title] = btn

    # YOLO 모델 & 카메라 초기화
    model = YOLO("/home/rokey/Downloads/gasStation_1.pt")
    cap = cv2.VideoCapture(6)  # USB 카메라 인덱스 -> color image

    video_label = tk.Label(main_frame)
    video_label.place(x=820, y=20, width=440, height=330)  # 오른쪽 하단 영역에 영상 표시

    def update_gui():
        buttons["로봇 상태"].config(text=f"로봇 상태: {trigger_node.status_message}")
        buttons["경유 상태"].config(text=f"경유 상태: {trigger_node.diesel_message}")
        buttons["휘발유 상태"].config(text=f"휘발유 상태: {trigger_node.gasoline_message}")
        buttons["주유기 상태"].config(text=f"주유기 상태: {trigger_node.pump_message}")
        buttons["카드 상태"].config(text=f"카드 상태: {trigger_node.card_message}")
        buttons["장애물 상태"].config(text=f"장애물 상태: {trigger_node.obstacle_message}")

        ret, frame = cap.read()
        if ret:
            results = model.predict(source=frame, conf=0.4, verbose=False)[0]
            for box in results.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                cls_name = model.names[cls_id]
                label = f'{cls_name} ({cls_id}) {conf:.2f}'
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(rgb_frame).resize((440, 330))
            imgtk = ImageTk.PhotoImage(image=img)
            video_label.imgtk = imgtk
            video_label.config(image=imgtk)

        root.after(100, update_gui)

    update_gui()
    root.mainloop()
    cap.release()

def main():
    rclpy.init()
    trigger_node = TriggerNode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(trigger_node,), daemon=True)
    ros_thread.start()

    start_gui(trigger_node)

    trigger_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
