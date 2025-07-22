import os
import rclpy
import pyaudio
from rclpy.node import Node
from std_msgs.msg import Bool, String, Int32
from dotenv import load_dotenv
from langchain_community.chat_models import ChatOpenAI
from langchain_core.prompts import PromptTemplate
from langchain.chains import LLMChain
from voice_processing.stt import STT
from voice_processing.MicController import MicController, MicConfig
from voice_processing.wakeup_word import WakeupWord
from pathlib import Path


class ExtractKeyword:
    def __init__(self, api_key):
        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.2, openai_api_key=api_key
        )

        self.prompt_template = PromptTemplate(
            input_variables=["user_input"],
            template="""
당신은 사용자 음성 명령에서 연료 종류, 결제 수단, 주유 금액을 추출해야 합니다.

<목표>
- 연료 종류: 휘발유, 경유 중 하나 (없으면 비워둠)
- 결제 수단: 카드 (없으면 비워둠)
- 금액: 숫자만 추출하여 "원" 단위로 반환 (예: "5만원" → 50000)

<출력 형식>
- 반드시 다음 형식으로 반환하세요: [연료 / 결제 / 금액]
- 각 항목이 없으면 공백으로 비워, '/' 구분자는 반드시 유지하세요

<예시>
- 입력: "경유로 5만원 카드 결제해줘" → [경유 / 카드 / 50000]
- 입력: "휘발유로 카드 결제" → [휘발유 / 카드 / ]
- 입력: "카드 결제할게" → [ / 카드 / ]

<사용자 입력>
"{user_input}"
"""
        )

        self.lang_chain = LLMChain(llm=self.llm, prompt=self.prompt_template)

    def extract(self, text):
        response = self.lang_chain.invoke({"user_input": text})
        try:
            raw = response["text"].strip()
            if raw.startswith("[") and raw.endswith("]"):
                raw = raw[1:-1]  # remove brackets
            parts = raw.split("/")
            if len(parts) != 3:
                raise ValueError(f"Expected 3 fields but got: {raw}")

            fuel = parts[0].strip()
            payment = parts[1].strip()
            amount_str = parts[2].strip()

            amount = int(amount_str) if amount_str.isdigit() else 0
            return fuel, payment, amount

        except Exception as e:
            print(f"[ERROR] LLM 응답 파싱 실패: {e}")
            return "", "", 0


class ToolPublisherNode(Node):
    def __init__(self):
        super().__init__("tool_publisher_node")

        # env 불러오기
        dotenv_path = Path(__file__).parent.parent / "resource" / ".env"
        load_dotenv(dotenv_path=dotenv_path)
        api_key = os.getenv("OPENAI_API_KEY")

        # STT, LLM 초기화
        self.stt = STT(openai_api_key=api_key)
        self.extractor = ExtractKeyword(api_key)

        # 오디오 마이크 설정
        mic_config = MicConfig(
            chunk=12000, rate=48000, channels=1,
            record_seconds=5, fmt=pyaudio.paInt16,
            device_index=10, buffer_size=24000,
        )
        self.mic_controller = MicController(mic_config)
        self.wakeup_word = WakeupWord(mic_config.buffer_size)

        # 퍼블리셔
        self.fuel_pub = self.create_publisher(Int32, "/oil", 10)
        self.pump_pub = self.create_publisher(Bool, "/target/Pump", 10)
        self.card_pub = self.create_publisher(Bool, "/target/CreditCard", 10)
        self.detected_tool_pub = self.create_publisher(String, "/detected_tool", 10)  # ← 추가됨

        # 시작 로그
        self.get_logger().info("도구 추출 및 퍼블리시 노드 시작됨")

        # 바로 실행
        self.run_pipeline()

    def run_pipeline(self):
        self.mic_controller.open_stream()
        self.wakeup_word.set_stream(self.mic_controller.stream)
        self.get_logger().info("웨이크업 단어 대기 중...")

        while not self.wakeup_word.is_wakeup():
            pass

        self.get_logger().info("음성 입력 수신 중...")
        
        try:
            text = self.stt.speech2text()
            self.get_logger().info(f"[STT 결과] \"{text}\"")
        except Exception as e:
            self.get_logger().error(f"STT 실패: {e}")
            return
        
        fuel, payment, amount = self.extractor.extract(text)
        self.get_logger().info(f"[LLM 추출 결과] 연료: {fuel}, 결제: {payment}, 금액: {amount}")

        # 연료 퍼블리시

        if fuel in ["휘발유", "경유"]:
            self.pump_pub.publish(Bool(data=True))
            self.detected_tool_pub.publish(String(data="pump"))
        
        if fuel == "경유":
            self.fuel_pub.publish(Int32(data=0))
        elif fuel == "휘발유":
            self.fuel_pub.publish(Int32(data=1))

        # 결제 수단 퍼블리시
        if payment in ["카드", "creditcard", "card"]:
            self.card_pub.publish(Bool(data=True))
            self.detected_tool_pub.publish(String(data="creditcard"))

        # 콘솔 로그
        self.get_logger().info(f"주유 요청: {fuel}, 결제: {payment}, 금액: {amount}")


def main():
    rclpy.init()
    node = ToolPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
