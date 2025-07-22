import os
import rclpy
import pyaudio
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain.chat_models import ChatOpenAI
from langchain.prompts import PromptTemplate
from langchain.chains import LLMChain
from std_srvs.srv import Trigger
from std_msgs.msg import Int32, Bool
from pathlib import Path
from voice_processing.MicController import MicController, MicConfig
from voice_processing.wakeup_word import WakeupWord
from voice_processing.stt import STT


class ExtractFuelInfo:
    def __init__(self, api_key):
        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.2, openai_api_key=api_key
        )

        self.prompt_template = PromptTemplate(
            input_variables=["user_input"],
            template="""
당신은 음성 명령에서 연료 종류와 결제 수단을 추출해야 합니다.

<목표>
- 문장에서 연료 종류(휘발유, 경유)와 결제 수단(카드, 현금 등)을 최대한 정확히 추출하세요.

<출력 형식>
- 다음과 같이 반드시 반환: [연료 / 결제]
- 연료가 없으면 앞부분은 비우고, 결제 수단이 없으면 '/' 뒤는 비워두세요.
- 각각 공백 없이 하나만 기입합니다.

<예시>
- 입력: "휘발유 원해요" → 휘발유 /
- 입력: "경유 카드로요" → 경유 / 카드
- 입력: "현금으로 주유해주세요" →  / 현금

<사용자 입력>
"{user_input}"
"""
        )

        self.lang_chain = LLMChain(llm=self.llm, prompt=self.prompt_template)

    def extract(self, text):
        response = self.lang_chain.invoke({"user_input": text})
        try:
            fuel, payment = response["text"].strip().split("/")
            fuel = fuel.strip()
            payment = payment.strip()
            return fuel, payment
        except Exception as e:
            print(f"[ERROR] 응답 파싱 실패: {e}")
            return "", ""


class AutoFuelNode(Node):
    def __init__(self):
        super().__init__("auto_fuel_node")

        # 환경 설정
        # pkg_path = get_package_share_directory("pick_and_place_voice")
        # load_dotenv(dotenv_path=os.path.join(pkg_path, "resource/.env"))
        # api_key = os.getenv("OPENAI_API_KEY")

        dotenv_path = Path(__file__).parent.parent / "resource" / ".env"
        load_dotenv(dotenv_path=dotenv_path)

        api_key = os.getenv("OPENAI_API_KEY")
        print("[DEBUG] OPENAI_API_KEY =", os.getenv("OPENAI_API_KEY"))

        # 모듈 초기화
        self.stt = STT(openai_api_key=api_key)
        self.extractor = ExtractFuelInfo(api_key)

        # 오디오 설정
        mic_config = MicConfig(
            chunk=12000, rate=48000, channels=1,
            record_seconds=5, fmt=pyaudio.paInt16,
            device_index=10, buffer_size=24000,
        )
        self.mic_controller = MicController(config=mic_config)
        self.wakeup_word = WakeupWord(mic_config.buffer_size)

        # 퍼블리셔 초기화
        self.fuel_pub = self.create_publisher(Int32, "/oil", 10)
        self.card_pub = self.create_publisher(Bool, "/target/Card", 10)
        self.card2_pub = self.create_publisher(Bool, "/target/card2", 10)
        self.obstacle_pub = self.create_publisher(Bool, "/target/Obstacle", 10)

        # 서비스 등록
        self.get_logger().info("자동 주유 음성인식 노드 시작됨.")
        self.get_keyword_srv = self.create_service(
            Trigger, "get_keyword", self.get_keyword_callback
        )

    def get_keyword_callback(self, request, response):
        try:
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)
        except OSError:
            self.get_logger().error("오디오 장치 오류")
            response.success = False
            response.message = "오디오 오류"
            return response

        self.get_logger().info("웨이크업 단어 대기 중...")
        while not self.wakeup_word.is_wakeup():
            pass

        self.get_logger().info("음성 수신 중...")
        spoken_text = self.stt.speech2text()
        fuel, payment = self.extractor.extract(spoken_text)

        self.get_logger().warn(f"[인식됨] 연료: {fuel}, 결제: {payment}")

        if fuel == "결제완료":
            self.get_logger().info("[음성] 결제 완료 감지 → 장애물 제거 퍼블리시")
            self.obstacle_pub.publish(Bool(data=True))
            response.success = True
            response.message = "결제완료 / 장애물 제거"
            return response

        # 연료 종류 퍼블리시
        if fuel == "휘발유":
            self.fuel_pub.publish(Int32(data=1))
        elif fuel == "경유":
            self.fuel_pub.publish(Int32(data=0))

        # 결제 방식 퍼블리시
        if payment == "card1":
            self.card1_pub.publish(Bool(data=True))
        elif payment == "card2":
            self.card2_pub.publish(Bool(data=True))

        response.success = True
        response.message = f"{fuel} / {payment}"
        return response


def main():
    rclpy.init()
    node = AutoFuelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
