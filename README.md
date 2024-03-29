# Embedded_01_03

## 1. 개요
본 프로젝트에서는 사용자에게 중요물품을 보관할 수 있는 시스템을 제공하기 위해 여러 센서 및 엑추에이터들을 활용해 잠금 장치, 도난 방지 및 스마트폰을 통한 실시간 모니터링 기술들이 겸비된 "철통 보안 스마트 금고"를 개발하였다.

![KakaoTalk_20231221_171212364](https://github.com/k1sihyeon/Embedded_01_03/assets/119331034/11890a05-3731-4fa9-ba12-cbbc47db05fd)

## 2. 아이디어 소개
금고 잠금
- 버튼으로 서보모터를 제어해 잠금 기능 구현
- 금고 잠금 시, 부저를 통한 소리 출력

금고 잠금 해제
- NFC로 서보모터를 제어해 1차 잠금 해제
- 휴대폰은 블루투스 모듈을 통해 연결되어 있으며 비밀번호는 FND LED에 표시
- 휴대폰으로 비밀번호를 입력해 서보모터를 제어해 2차 잠금 해제
- 금고 잠금 및 잠금 해제 시, 부저를 통한 소리 출력
  
도난 방지
- 초음파 센서를 통해 내부 움직임 감지
- 자이로 센서를 통해 내부 기울기 변화 감지
  
실시간 내부 온도 측정
- 온도 센서의 값을 FND LED에 실시간으로 출력해 금고 내부 온도 표시
  
## 3. 프로젝트 하드웨어 및 GPIO 핀 구성 요소
![image](https://github.com/k1sihyeon/Embedded_01_03/assets/119672962/4f9a2de2-1e2f-4d7a-a472-0afdf51703fe)



![image](https://github.com/k1sihyeon/Embedded_01_03/assets/119672962/938de50b-48fc-4a78-ba89-e605e0918f73)


#### GPIO 핀 구성
###### 금고 잠금 장치
- NFC
  - NFC 모듈의 데이터 핀: GPIO2
  - NFC 모듈의 클락 핀: GPIO3
- 스위치
  - 스위치의 입력 핀: GPIO21
- FND
  - FND 세그먼트: GPIO17,27,22,10,9,11,12,7,8,2,5,24,23
- 서브모터
  - 서브모터 제어 핀: GPIO19
- 부저
  - 부저 제어 핀: GPIO26
- 블루투스 모듈
  - 블루투스 모듈 TX 핀: GPIO27(UART 2번)
  - 블루투스 모듈 RX 핀: GPIO28(UART 2번)
- 라즈베리파이 UART 통신
  - UART TX 핀: GPIO3(UART 3번)
  - UART RX 핀: GPIO5(UART 3번)
###### 금고 내부 탐지 장치
- 초음파 센서
  - 초음파 센서 TRIG 핀: GPIO15
  - 초음파 센서 ECHO 핀: GPIO18
- 자이로 센서
  - 자이로 센서 SDA 핀: GPIO10
  - 자이로 센서 SCL 핀: GPIO11
- 온/습도 센서
  - 온/습도 센서 SCL 핀: GPIO3
  - 온/습도 센서 SDA 핀: GPIO2
- FND
  - FND 세그먼트: GPIO4,17,27,22,5,6,13,19,26,21,20,16
- 라즈베리파이 UART 통신
  - UART TX 핀: GPIO0
  - UART RX 핀: GPIO1
## 4. 전체 시스템 구조
![image](https://github.com/k1sihyeon/Embedded_01_03/assets/96001080/9a3c5d2d-b6c2-45b8-919c-5aa87928ae3c)


 - 전체 회로도
   ![Sketch1_bbb](https://github.com/k1sihyeon/Embedded_01_03/assets/96001080/069f2145-eceb-4e19-a085-9808383d9c76)



## 5. 시스템 흐름도
   - 잠금 장치
     ![image](https://github.com/k1sihyeon/Embedded_01_03/assets/96001080/c9e395c9-0530-45cb-a742-4b6da07e79b9)

   - 내부 탐지 장치
     ![image](https://github.com/k1sihyeon/Embedded_01_03/assets/96001080/73ccad50-e7b8-4895-9233-9a698fe84f76)
     ![image](https://github.com/k1sihyeon/Embedded_01_03/assets/96001080/0c0befef-bfca-4529-8261-1277dede888c)



## 6. 제한 조건 구현 내용
![image](https://github.com/k1sihyeon/Embedded_01_03/assets/119331034/30fe70ca-3c16-4929-8510-6fd0213348bc)



## 7. 가산점 요소
![image](https://github.com/k1sihyeon/Embedded_01_03/assets/96001080/1e44d846-de65-4696-bf23-6f901f9e0129)
- 스마트폰 - 라즈베리 파이 통신
  1. 블루투스 모듈을 사용
  2. 라즈베리 파이에서 스마트폰으로 도난(감지) 정보를 송신
  3. 스마트폰에서 라즈베리 파이로 입력한 비밀번호를 송신
- 라즈베리 파이 - 라즈베리 파이 통신
  1. UART 통신 사용
  2. 내부 탐지 장치 라즈베리 파이에서 외부 잠금 장치 라즈베리 파이로 내부에서 감지된 정보를 송신
  3. 외부 잠금 장치 라즈베리 파이에서 내부 탐지 장치 라즈베리 파이로 잠금 및 해제 정보를 송신

## 8. 개발 시 문제점 및 해결 방안
 - 문제점
   1. 라즈베리파이 – 라즈베리파이 통신 간 잘못된 값 송/수신
   2. 다수의 GPIO핀 연결로 인한 PWM 연결의 어려움
   3. 다수의 GPIO핀 연결로 인한 UART 연결의 어려움

 - 해결 방안
   1. 디버그를 통해 송/수신된 값을 지속적으로 검사
   2. softPWM 사용
   3. 다른 UART핀 활성화 (UART2 활성화)

## 9. 유저 사용 설명서
- 초기 설정
  1. Serial Bluetooth Terminal 앱 실행
  2. Bluetooth 모듈 연결
     ![Screenshot_20231220_210624_Serial Bluetooth Terminal](https://github.com/k1sihyeon/Embedded_01_03/assets/96001080/829287b2-7507-47e2-9884-98717b291d1b)

     
- 금고 잠금
  1. 금고 문 닫기
  2. 금고 오른쪽 버튼 클릭
  3. 부저 알림과 FND의 clsd 확인
  4. 휴대폰 블루투스 터미널 알림 확인
     
- 금고 잠금 해제
  1. nfc 키를 금고 오른쪽 nfc 리더에 태그
  2. 휴대폰 블루투스 터미널에서 비밀번호 입력
  3. FND의 open 확인
  4. 금고 열림
     
- 도난 방지 알림
   - 금고가 닫힌 상태에서 금고 내부의 움직임이 감지되면 휴대폰 블루투스 터미널에 알림
   - 금고가 닫힌 상태에서 금고 자체의 움직임이 감지되면 휴대폰 블루투스 터미널에 알림
     
- 내부 온도 확인
   - 금고 왼쪽의 FND에서 내부 온도를 확인 할 수 있음
      
## 10. 데모 영상
[![Video Label](http://img.youtube.com/vi/1MKW3DmWLwo/0.jpg)](https://youtu.be/1MKW3DmWLwo)
 
 - 데모 영상 순서
   1. 버튼으로 서보모터 잠금
   2. Fnd, 부저, 스마트폰으로 잠금 상태 출력
   3. 내부/외부 움직임 감지 정보 스마트폰으로 송신 (내부탐지 장치 -> 잠금 장치 -> 스마트폰)
   4. NFC 태그 및 비밀번호 확인 후 서보모터 잠금 해제
   5. 열린 상태에서는 내부 탐지를 수행하지 않음

