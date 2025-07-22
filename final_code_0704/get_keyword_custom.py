import os
import rclpy
import pyaudio
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Int32, Bool, String
from dotenv import load_dotenv
# from langchain_community.chat_models import ChatOpenAI
from langchain_openai import ChatOpenAI
from langchain_core.prompts import PromptTemplate
from langchain.chains import LLMChain
from pathlib import Path
from voice_processing.MicController import MicController, MicConfig
from voice_processing.wakeup_word import WakeupWord
from voice_processing.stt import STT

# === YOLO 클래스 이름과 사용자 표현 매핑 ===
YOLO_CLASS_MAPPING = {
    "creditcard": ["카드", "신용카드", "card"],
    "pump": ["펌프", "주유기", "fuelgun", "건", "주유기기"],
    "cash": ["현금", "돈"]
}

def map_to_yolo_class(keyword: str):
    keyword = keyword.strip().lower()
    for yolo_class, alias_list in YOLO_CLASS_MAPPING.items():
        if keyword in alias_list:
            return yolo_class
    return ""


class ExtractFuelInfo:
    def __init__(self, api_key):
        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.2, openai_api_key=api_key
        )

        self.prompt_template = PromptTemplate(
            input_variables=["user_input"],
            template="""
다음 사용자 명령어를 분석해서 [연료 / 결제 / 금액] 형식으로만 결과를 출력하세요.

<제한 사항>
- 출력은 반드시 [연료 / 결제 / 금액] 형식이어야 합니다.
- '[]' 괄호와 '/' 구분자를 포함한 정확한 포맷을 지켜야 합니다.
- 연료: 휘발유 또는 경유
- 결제: 카드 또는 현금
- 금액: 숫자만 (예: "5만원" → 50000)
- 해당 값이 없으면 공백으로 두고 슬래시는 유지합니다.

<예시>
입력: 경유로 5만원 카드 결제해줘 → [경유 / 카드 / 50000]  
입력: 카드 결제할게 → [ / 카드 / ]  
입력: 휘발유로 카드 결제 → [휘발유 / 카드 / ]  

<사용자 입력>
"{user_input}"

<출력 형식>
"""
        )

        self.lang_chain = LLMChain(llm=self.llm, prompt=self.prompt_template)

    def extract(self, text):
        response = self.lang_chain.invoke({"user_input": text})
        try:
            raw = response["text"].strip().strip("[]")
            parts = raw.split("/")
            if len(parts) != 3:
                raise ValueError("응답 형식 오류")
            fuel = parts[0].strip()
            payment = parts[1].strip()
            amount = parts[2].strip()
            return fuel, payment, amount
        except Exception as e:
            print(f"[ERROR] 응답 파싱 실패: {e}")
            return "", "", ""


class AutoFuelNode(Node):
    def __init__(self):
        super().__init__("auto_fuel_node")

        dotenv_path = Path(__file__).parent.parent / "resource" / ".env"
        load_dotenv(dotenv_path=dotenv_path)
        api_key = os.getenv("OPENAI_API_KEY")

        self.stt = STT(openai_api_key=api_key)
        self.extractor = ExtractFuelInfo(api_key)

        mic_config = MicConfig(
            chunk=12000, rate=48000, channels=1,
            record_seconds=5, fmt=pyaudio.paInt16,
            device_index=10, buffer_size=24000,
        )
        self.mic_controller = MicController(mic_config)
        self.wakeup_word = WakeupWord(mic_config.buffer_size)

        self.fuel_pub = self.create_publisher(Int32, "/oil", 10)
        self.creditcard_pub = self.create_publisher(Bool, "/target/CreditCard", 10)
        self.detected_tool_pub = self.create_publisher(String, "/detected_tool", 10)
        # Add amount publisher for gui
        self.amount_pub = self.create_publisher(Int32, "/amount", 10)
        # Add finish
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

        fuel, payment, amount = self.extractor.extract(spoken_text)
        self.get_logger().info(f"[LLM 추출 결과] 연료: {fuel}, 결제: {payment}, 금액: {amount}")

        # 연료 퍼블리시
        if fuel == "경유":
            self.fuel_pub.publish(Int32(data=0))
        elif fuel == "휘발유":
            self.fuel_pub.publish(Int32(data=1))

        # 결제 수단 퍼블리시
        if payment.lower() in ["카드", "creditcard", "card"]:
            self.creditcard_pub.publish(Bool(data=True))

        # Add amount publish for gui
        if amount.isdigit():
            self.amount_pub.publish(Int32(data=int(amount)))
        # Add finish
        
        # YOLO 타겟 퍼블리시
        fuel_yolo = map_to_yolo_class(fuel)
        payment_yolo = map_to_yolo_class(payment)

        if fuel_yolo:
            self.detected_tool_pub.publish(String(data=fuel_yolo))
        if payment_yolo:
            self.detected_tool_pub.publish(String(data=payment_yolo))

        response.success = True
        response.message = f"{fuel} / {payment} / {amount}"
        return response


def main():
    rclpy.init()
    node = AutoFuelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
