<h1 align="center">AI기반 협동 로봇 작업 어시스턴트 구현🔬 </h1>

<h2 align="center">협동로봇을 활용한 주유시스템 자동화🚗 </h2>



## 개요


비대면, 비접촉 서비스 수요가 급증하며 무인 주유소가 늘어났지만 사용자(운전자)의 번거로움이 증가. 주유 안전성과 편의성 강화를 위해 도모. 또한 자율 주행 자동차의 연장선으로 차량 내에서 편리한 주유 경험을 제공함.





## 제작 기간 & 참여 인원


-2025/06/23~2025/07/04  3명






## 사용한 기술 (기술 스택)  


<img src="https://img.shields.io/badge/python-blue?style=for-the-badge&logo=python&logoColor=white">   <img src="https://img.shields.io/badge/ROS2-black?style=for-the-badge&logo=ros&logoColor=#22314E">   <img src="https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white">   <img src="https://img.shields.io/badge/YOLO-111F68?style=for-the-badge&logo=yolo&logoColor=white">  <img src="https://img.shields.io/badge/STT-red?style=for-the-badge&logo=STT&logoColor=white">  <img src="https://img.shields.io/badge/LLM-green?style=for-the-badge&logo=LLM&logoColor=white">  <img src="https://img.shields.io/badge/DoosanAPI-blue?style=for-the-badge&logo=DoosanAPI&logoColor=white">





## High Level Architecture Diagram

<img width="1172" height="246" alt="image" src="https://github.com/user-attachments/assets/6952ab3d-2b85-4827-b818-4b2eca24732a" />



## 내가 기여한 부분


- **시나리오 설계, High Level Architecture Diagram 구상**


- **Node 통신 구현**

-Service를 Topic으로 변형 후 제작

- **STT-YOLO keyword mapping**


-STT에서 인식한 단어를 YOLO의 class name과 일치시킴



- **로봇 동작 제어 코드 구현**






   
## 🎯트러블슈팅 경험  


1. YOLO로 detect한 물체를 depth camera로 위치 계산해서 접근하는 시나리오를 작성했는데, depth 계산이 진행 됐다 안 됐다 반복함 


2. custom dataset과 coco dataset을 합치니 coco dataset에 포함되어있는 다른 객체들도 인식함




## 🔨해결방법


1-1. OBB(Oriented Bounding box)를 사용해 좌표값을 계산하려했지만 시간 부족으로 실패하고, 좋은 방법은 아니지만 대신 예외처리를 넣어서 만약 depth좌표 계산에 실패하면 하드코딩 된 좌표로 이동하게 했다


2-1. ROI를 통해 일정 영역 안에 있는 객체만 인식하게 변경하고싶었지만 시간 부족으로 custom data만 사용하는 방법을 채택했다.





## 회고 / 느낀 점

-ROI로 YOLO detect영역을 지정하지 못 한 점, STT prompt를 조금 더 고도화하여 차량 모델을 인식하고 자동으로 유종을 판단하게 하고싶었지만 못 한 점, depth 좌표 계산을 완벽하게 해내지 못 한 점, Database를 사용해 포스기를 구현하고싶었지만 구현하지 못 한 점 등 하고 싶은 것이 많았지만 구현하지 못 한 것이 많았던 프로젝트라 많이 아쉬웠다. 하지만 이러한 아이디어들을 혼자서라도 고민해보고 구현 할 방법을 찾아 작성해 놓으면 언젠가는 사용할 수 있을거라고 생각한다.
