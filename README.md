# Vehicle Smart Cruise Control
LiDAR 센서로 차량의 전방, 후방, 우측방, 좌측방의 물체를 감지하여 차량의 속도를 자동으로 조절하고 감지 방향을 경고하는 **차량 스마트 크루즈 컨트롤** 입니다.  
라즈베리 파이로 LiDAR 센서의 구동 및 영상 시뮬레이션을 구현했고 Classic AUTOSAR 기반으로 모터, LED, 진동 감지 센서를 제어하는 MPC5606B의 펌웨어를 개발했습니다.

## 📌 주요 기능
- LiDAR 센서 기준 70cm 반경의 모든 방향(0~360°)에서 물체의 위치 및 거리 감지
- 전방: 315° ~ 45° / 우측방: 45° ~ 135° / 후방: 135° ~ 225° / 좌측방: 225° ~ 315°

### 🚗 차량 속도 제어
- 전방의 물체가 점점 가까워질 경우 차량의 속도 **감속**
- 후방의 물체가 점점 가까워질 경우 차량의 속도 **가속**
- 차량의 속도에 따라서 모터 속도 제어
- 차량의 속도에 따라서 이미지 출력 간 Delay를 설정하여 차량의 속도 변화를 영상으로 표현

### 🚨 접근 방향 경고
- 접근 물체의 방향에 따라 각기 다른 LED를 점등하여 방향별 경고 표시 및 영상에 감지 방향 표시
- LED: 전방(Green), 후방(Red), 좌측방(Orange), 우측방(White)
- 영상: 전방(FRONT), 후방(REAR), 좌측방(LEFT), 우측방(RIGHT)

### 💥 충돌 감지
- 진동 감지 시 영상 정중앙에 SHOCK! 경고 표시

### 🖼️ 영상 시뮬레이션 예시
![영상 시뮬레이션](https://github.com/user-attachments/assets/cbc22867-030c-4afa-9643-d53ba5e41c45)

- 차량 주행속도: 80[km/h]
- 감지 방향: LEFT(우측방)
- 차량에 충돌이 감지되어 SHOCK! 경고 표시

## 🚀 실행 방법

### MPC5606B
1. mobilgene으로 BSW, ASW 생성
2. Static_Code/App_Code 디렉토리 내 .c 파일 복사
3. Build 진행 후 .elf 파일 확인
4. CodeWarrior로 MPC5606B에 펌웨어 업로드

### Raspberry Pi
1. rplidar_sdk/app/ultra_simple 디렉토리 내에서 make 명령어 실행
2. rplidar_sdk/output/Linux/Release 디렉토리 이동
3. 명령어 실행: ./ultra_simple --channel --serial /dev/ttyUSB0 115200

## 🛠 하드웨어 구성

### MPC5606B
- LED: Green, Red, Orange, White
- 서보모터: SG90-HV
- 진동감지 센서: SW-420

### Raspberry Pi 5
- LiDAR 센서: 슬램텍 RPLIDAR A1M8

## 💻 소프트웨어 구성

### MPC5606B
- 사용언어: C
- Classic AUTOSAR 기반 소프트웨어 플랫폼 mobilgene 사용

### Raspberry Pi 5
- 사용언어: C, C++
- 영상 시뮬레이션: OpenCV 라이브러리
- LiDAR 센서 SDK 사용

## 📝 시스템 개발 설계서
![시스템 구조](https://github.com/user-attachments/assets/257cde3b-0fce-4ef4-a362-8eda7a5f5b3c)

- CAN 통신 DATA
  - Speed(Unsigned int 8bit): 30~150 
  - Direction(Unsigned int 8bit): 0(물체감지X), 1(전방), 2(우측방), 3(후방), 4(좌측방)
  - Shock(1bit): 0(진동감지X), 1(진동감지O)

### MPC5606B: ASW 구조
![ASW+CANDB](https://github.com/user-attachments/assets/9b075e4d-1873-4eb7-84a6-64ebf4e5e6c2)

- ECU1_Msg_RP(CAN ID: 0x06)
  - Sig1(Dir): 라즈베리파이가 보낸 감지방향 값 그대로 송신(Feedback)
  - Sig2(Speed): 라즈베리파이가 보낸 차량속도 값 그대로 송신(Feedback)
  - Sig3(Shock): 진동감지 센서 값 송신

- ECU2_Msg_Dir(CNA ID: 0x07)
  - Sig1(Dir): 라즈베리파이가 보낸 감지방향 값 수신
  - Sig2(Speed): 라즈베리파이가 보낸 속도 값 수신

### Rasperry Pi
![RaspberryPi](https://github.com/user-attachments/assets/4ad1eddd-aa5b-4328-a9e7-992f6319691a)

- Main Thread
  - 차량 속도 값에 따라서 영상 20장 Sequence의 Delay 조절
  - 물체 감지 방향 표시
  - 진동 감지 경고창 표시
- LiDAR + CAN Send Thread
  - LiDAR 센서가 1frame 당 획득하는 8192개의 감지된 물체의 각도 및 거리값 처리
  - 물체 감지 방향, 차량 속도 값을 CAN ID 7번으로 송신
- CAN Receive Thread
  - MPC5606B에서 CAN ID 6번으로 보낸 진동감지 값 수신

## 📌 프로젝트 구조
```
Vehicle-Smart-Cruise-Control/
│── rplidar_sdk/            # Raspberry Pi 코드
│── Static_Code/App_Code    # mobilgene 기반 MPC5606B 코드
└── README.md               # 프로젝트 설명 파일
```

## 팀 구성원

| 이름 | GitHub |
|------|--------|
| 옥창희 | [@okchangheeok](https://github.com/okchangheeok) |
| 김우성 | [@woosmile](https://github.com/woosmile) |

