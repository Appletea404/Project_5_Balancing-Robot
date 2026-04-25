
![title](images/title.png)


# 🤖 Project 5 Balancing Robot

## **1. Project Summary (프로젝트 요약)**
Basys3(Artix-7 FPGA)와 Encoder가 있는 Gear DC 모터(JGB37-520)를 활용하여  밸런싱 로봇 제작


## 2. Key Features (주요 기능)

### 
- 조이스틱(Joystick)을 통해 차체를 조종가능
- PWM 신호를 통해 자동차의 속도를 변경 가능하고 이를 조이스틱 감도로 제어가능

### 🤖 Auto Mode (자율주행)

- 센서(Ultrasonic) 데이터를 기반으로 장애물 회피
- 데이터를 이중으로 비교하여 회전 중에도 재판단
- 코너에 진입했는데 전면과의 거리가 너무 가까우면 넓은 방향으로 후진

## 🛠 3.  Tech Stack (기술 스택)


### 3.1 Language (사용언어)

![C](https://img.shields.io/badge/c-%2300599C.svg?style=for-the-badge&logo=c&logoColor=white)

### 3.2 Development Environment (개발 환경)
| IDE | Configuration |
| :---: | :---: |
| ![STM32CubeIDE](images/stm32cubeide.png) | ![STM32CubeMX](images/stm32cubemx.png) |
| **STM32CubeIDE** | **STM32CubeMX** |

### 3.3 Collaboration Tools (협업 도구)

![Github](https://img.shields.io/badge/GitHub-100000?style=for-the-badge&logo=github&logoColor=white)![Discord](https://img.shields.io/badge/Discord-7289DA?style=for-the-badge&logo=discord&logoColor=white)![Notion](https://img.shields.io/badge/Notion-000000?style=for-the-badge&logo=notion&logoColor=white)


## 📂 4.  Project Structure (프로젝트 구조)

### 4.1 Project Tree (프로젝트 트리)

```
Project3_AutoMobility/
├── RC_CAR_R02/                 # [Receiver] RC카 본체 제어부 (STM32F411RETx)
│   ├── Core/
│   │   ├── Src/                # 핵심 주행 및 제어 소스 코드 (.c)
│   │   │   ├── main.c          # 주변장치 초기화 및 메인 제어 루프
│   │   │   ├── car.c           # L298N 모터 드라이버 구동 로직
│   │   │   ├── statemachine.c  # Manual/Auto 모드 전환 상태 머신
│   │   │   ├── ultrasonic.c    # 초음파 센서 거리 측정 및 장애물 판단
│   │   │   ├── direction.c     # 차량 조향 알고리즘 구현
│   │   │   ├── speed.c         # PWM 기반 모터 속도 제어
│   │   │   └── stm32f4xx_it.c  # 타이머/센서 인터럽트 서비스 루틴
│   │   └── Inc/                # 함수 선언 및 하드웨어 설정 헤더 (.h)
│   └── RC_CAR_R02.ioc          # STM32CubeMX 하드웨어 구성 파일
│
├── Remote/                     # [Transmitter] 조이스틱 컨트롤러 (STM32F411CEUx)
│   ├── Core/
│   │   ├── Src/                # 조종기 구동 및 통신 소스 코드 (.c)
│   │   │   ├── main.c          # 컨트롤러 메인 로직
│   │   │   ├── bt_master.c     # 블루투스 마스터 통신 (데이터 송신)
│   │   │   ├── adc.c           # 조이스틱 아날로그 신호 수집
│   │   │   └── dma.c           # 센서 데이터 고속 처리를 위한 DMA 설정
│   │   └── Inc/                # 컨트롤러 헤더 파일 (.h)
│   └── Remote.ioc              # Remote 전용 CubeMX 설정 파일
│
├── images/                     # README 및 기술 문서용 이미지 리소스 (회로도, 다이어그램 등)
└── README.md                   # 프로젝트 전체 가이드 문서
```


### 4.2 Hardware BlockDiagram (하드웨어 블록다이어그램)

![BlockDiagram](images/Project3_Automobility_BlockDiagram.png)

### 4.3 State Machine (상태 머신)

![alt text](images/Project3_Automobility_Statemachine.png)

## 🏁 5. Final Product & Demonstration (완성품 및 시연)

### 5.1 Final Product (완성품)
<br>

| **전체 샷 (Full Setup)** | **조종기 측면 (Side)** | **조종기 후면 (Back)** |
| :---: | :---: | :---: |
| ![Full](images/5.jpg) | ![Wiring](images/6.jpg)  | ![MCU](images/4.jpg) |

<br>

| **차량 전면(Front)** | **차량 측면 (Side)** | **차량 하단 (Bottom)** |
| :---: | :---: | :---: |
| ![Front](images/7.jpg) | ![Side](images/2.jpg) | ![Bottom](images/3.jpg) |

<br>

### 5.2  Demonstration (시연 영상)

<a href="https://youtube.com/playlist?list=PL6xfXHA4BYR_6b3oBZIrsHkHDf97zLFjA&si=EPpmTzlfJeoMHaCG" target="_blank">
  <img src="images/youtube.jpg" alt="Watch Demo Video" width="300" />
</a>

*이미지를 클릭하면 시연 영상(유튜브)로 이동합니다.*


## 6. Troubleshooting (문제 해결 기록)

### 6.1 초음파 센서 데이터가 튐 (Outlier)


🔍  **Issue (문제 상황)**

- 자율 주행 모드 주행 중, 전방에 장애물이 없음에도 불구하고 차량이 회피할려고 회전함

❓ **Analysis (원인 분석)**

- STM32 디버깅 툴을 통해 3개의 초음파 센서의 데이터를 검사한 결과 **200cm**가 넘는 값이 일시적으로 감지됨을 인지

- 이러한 급격한 데이터 변화가 자율 주행 로직의 판단 임계치를 순간적으로 넘기면서 시스템 오작동을 유발함

❗ **Action (해결 방법)**

- 지나치게 먼거리라 판단하면 최대거리 **100cm**으로 고정시킴

✅ **Result (결과)**

- 센서 데이터의 값이 오버해서 차량이 오작동하는 일이 없어짐

---

### 6.2 장애물 회피 시 방향 결정 알고리즘의 불안정성 (Left-Right Misjudgment) 


🔍  **Issue (문제 상황)**

- 자율 주행 모드 주행 중, 우회전해야하는 상황에서 좌회전을 하는 등 오판을 함

❓ **Analysis (원인 분석)**

- 정면 거리 측정 후 좌우 공간을 순차적으로 판단하는 우선순위 기반 로직의 특성상, 공간이 급격히 좁아지는 **코너 구석(Corner Nook)** 진입 시 측면 데이터를 충분히 반영하지 못하는 오판 현상이 발생함.

❗ **Action (해결 방법)**

- 왼쪽 중앙 오른쪽 모든 센서의 거리중에서 가장 짧은 거리를 선별하여 그 쪽을 우선하여 회피하도록 로직을 수정

✅ **Result (결과)**

- 좌우판단을 더 이상 오판하지 않음

---

### 6.3  넓은 코너에 진입하면 갇힘 (Get trapped in Wide Corner) 


🔍  **Issue (문제 상황)**

- 좌우가 넓은 코너에 코너 안쪽으로 비스듬하게 진입시 회전 판단을 미리 못하여 벽에 부딛힘

❓ **Analysis (원인 분석)**

- 코너의 폭이 넓기 때문에 측면 센서가 인식하기에 거리가 너무 멀어서 정작 정면 센서쪽이 한계거리에 도달해도 회피판단을 못함

❗ **Action (해결 방법)**

- **Crash_Distance** 변수를 추가하여 정면센서가 이 거리에 도달하면 강제 후진 로직을 최우선으로 올림
- 후진이후 강제로 회전하면서 좌우 센서값을 강제로 갱신시킴
이후 회피판단 실행

✅ **Result (결과)**

- 넓은 코너에서 더 이상 갇히는 일이 없어짐



