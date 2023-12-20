# Embedded_01_03

## 1. 아이디어 소개
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

## 2. 전체 시스템 구조
![image](https://github.com/k1sihyeon/Embedded_01_03/assets/96001080/a2464e93-52c5-4e32-92e2-6eae61d84213)

## 3. 제한 조건 구현 내용
![image](https://github.com/k1sihyeon/Embedded_01_03/assets/96001080/1e35ae6c-1d63-47ee-9964-1215c30318ab)

## 4. 가산점 요소
![image](https://github.com/k1sihyeon/Embedded_01_03/assets/96001080/1e44d846-de65-4696-bf23-6f901f9e0129)

## 5. 개발 시 문제점 및 해결 방안
 - 문제점
   1. 라즈베리파이 – 라즈베리파이 통신 간 잘못된 값 송/수신
   2. 다수의 GPIO핀 연결로 인한 PWM 연결의 어려움
   3. 다수의 GPIO핀 연결로 인한 UART 연결의 어려움

 - 해결 방안
   1. 디버그를 통해 송/수신된 값을 지속적으로 검사
   2. softPWM 사용
   3. 다른 UART핀 활성화 (UART2 활성화)
      
## 6. 데모 영상
[![Video Label](http://img.youtube.com/vi/1MKW3DmWLwo/0.jpg)](https://youtu.be/1MKW3DmWLwo)


