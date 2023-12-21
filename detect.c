#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>
#include <string.h>

#define GPIO_COUNT 12
#define DIGIT_COUNT 4
#define SEG_COUNT 8
#define NUMS 12 // 폰트 수
#define SLAVE_ADDR_01 0x48
#define PRINT_TIME  1000 //출력 시간
#define BAUD_RATE   115200
#define BUF_SIZE  10

static const char* I2C_DEV = "/dev/i2c-1"; // I2C
static const char* UART2_DEV = "/dev/ttyAMA1";
unsigned char buf[BUF_SIZE];
int fd_serial;

#define CS_GPIO 8 // CS
#define SPI_CH 0
#define SPI_SPEED 1000000 // 1MHz
#define SPI_MODE 3

// 레지스터
#define BW_RATE 0x2C
#define POWER_CTL 0x2D // Power Control Register
#define DATA_FORMAT 0x31
#define DATAX0 0x32     // X-Axis Data 0
#define DATAX1 0x33     // X-Axis Data 1
#define DATAY0 0x34     // Y-Axis Data 0
#define DATAY1 0x35     // Y-Axis Data 1
#define DATAZ0 0x36     // Z-Axis Data 0
#define DATAZ1 0x37     // Z-Axis Data 1

const int gpio[GPIO_COUNT] = { 4, 17, 27, 22, 5, 6, 13, 19, 26, 21, 20, 16 };

int segments[SEG_COUNT]; 
int digits[DIGIT_COUNT];
unsigned char dat;
unsigned char temp;

const int  sevenseq[NUMS][SEG_COUNT] = {
    {1, 1, 1, 1, 1, 1, 0, 0}, //0
    {0, 1, 1, 0, 0, 0, 0, 0}, //1
    {1, 1, 0, 1, 1, 0, 1, 0}, //2
    {1, 1, 1, 1, 0, 0, 1, 0}, //3
    {0, 1, 1, 0, 0, 1, 1, 0}, //4
    {1, 0, 1, 1, 0, 1, 1, 0}, //5
    {1, 0, 1, 1, 1, 1, 1, 0}, //6
    {1, 1, 1, 0, 0, 1, 0, 0}, //7
    {1, 1, 1, 1, 1, 1, 1, 0}, //8
    {1, 1, 1, 1, 0, 1, 1, 0}, //9
    {1, 0, 0, 1, 1, 1, 0, 0}, //C   //10
    {0, 0, 0, 0, 0, 0, 0, 0}, //공백 //11
    };

const int  second_sevenseq[NUMS][SEG_COUNT] = {
    {1, 1, 1, 1, 1, 1, 0, 1}, //0
    {0, 1, 1, 0, 0, 0, 0, 1}, //1
    {1, 1, 0, 1, 1, 0, 1, 1}, //2
    {1, 1, 1, 1, 0, 0, 1, 1}, //3
    {0, 1, 1, 0, 0, 1, 1, 1}, //4
    {1, 0, 1, 1, 0, 1, 1, 1}, //5
    {1, 0, 1, 1, 1, 1, 1, 1}, //6
    {1, 1, 1, 0, 0, 1, 0, 1}, //7
    {1, 1, 1, 1, 1, 1, 1, 1}, //8
    {1, 1, 1, 1, 0, 1, 1, 1}, //9
    {1, 0, 0, 1, 1, 1, 0, 1}, //C   //10
    {0, 0, 0, 0, 0, 0, 0, 1}, //공백 //11
    };

void ClearPinMap() {
    printf("\nexit\n");
    for (int b = 0; b < SEG_COUNT; ++b) digitalWrite(segments[b], LOW);
    for (int b = 0; b < DIGIT_COUNT; ++b) digitalWrite(digits[b], LOW);
    exit(0);
}

void FndSelect(int position) {
    for (int i = 0; i < DIGIT_COUNT; i++) {
        if (i == position) digitalWrite(digits[i], LOW); //LOW가 on
        else digitalWrite(digits[i], HIGH); //HIGH가 off
    }
}

void FndPositionPrint(int str, int position) {
    for(int j = 0; j < SEG_COUNT; j++) {
    	if(position == 1) digitalWrite(segments[j], second_sevenseq[str][j]);
    	else digitalWrite(segments[j], sevenseq[str][j]);
    }
    FndSelect(position);
}

void FndInit() {
    int gpioDigits[DIGIT_COUNT] = {11, 8, 7, 5};
    int segmentDigits[SEG_COUNT] = {10, 6, 3, 1, 0, 9, 4, 2};

    for (int i = 0; i < DIGIT_COUNT; i++) digits[i] = gpio[gpioDigits[i]];

    for (int i = 0; i < SEG_COUNT; i++) segments[i] = gpio[segmentDigits[i]];

    if(wiringPiSetupGpio()==-1) {
    	printf("Wiringpi Error!");
        exit(-1);
    }

    for (int i = 0; i < SEG_COUNT; i++) pinMode(segments[i], OUTPUT);

    for(int i = 0; i < DIGIT_COUNT; i++) {
        pinMode(digits[i], OUTPUT);
        digitalWrite(digits[i], HIGH); //LOW가 on
    }
}

// ADXL345
void readRegister_ADXL345(char registerAddress, int numBytes, char* values) {
    values[0] = 0x80 | registerAddress;
    if (numBytes > 1)
        values[0] |= 0x40;
    digitalWrite(CS_GPIO, LOW);
    wiringPiSPIDataRW(SPI_CH, values, numBytes + 1);
    digitalWrite(CS_GPIO, HIGH);
}

void writeRegister_ADXL345(char address, char value) {
    unsigned char buff[2];
    buff[0] = address;
    buff[1] = value;
    digitalWrite(CS_GPIO, LOW);
    wiringPiSPIDataRW(SPI_CH, buff, 2);
    digitalWrite(CS_GPIO, HIGH);
}

// 센서1: 온도 센서
void* sensor_thread(void* arg) {
	int i2c_fd; //i2c
	int curVal = 0; //
	if(wiringPiSetupGpio() < 0 ) {
		printf("wiringPiSetup() is failed\n");
		return NULL;
    }	
	
    
    if ((i2c_fd = wiringPiI2CSetupInterface (I2C_DEV, SLAVE_ADDR_01))< 0 ) {
        printf("wiringPi2CSetup Failed: \n");
        return NULL;
    }

    FndInit();
    signal(SIGINT, ClearPinMap);  //SIGINI : ctrl + c시 발생 -> 종료시 모두 공백 출력

     while (1) {
        wiringPiI2CWrite(i2c_fd, 0x40 | 1);
        curVal = wiringPiI2CRead(i2c_fd); // (0~255)
        double temperature = curVal * 0.1;
        int digitValues[DIGIT_COUNT];
        digitValues[0] = curVal / 100;
        curVal %= 100;
        
        digitValues[1] = curVal / 10;
        curVal %= 10;
        
        digitValues[2] = curVal % 10;
        curVal /= 10;
        
        digitValues[3] = 10;
        
        unsigned int time = millis();
        
        while(1) {
            for(int i = 0; i < DIGIT_COUNT; i++) {
                FndPositionPrint(digitValues[i], i);
                delay(1);
            }
            //n초 동안 출력
            if(millis() - time > PRINT_TIME) { 
                time = millis();
                break;
            }
        }
        printf("Temperature: %.2f°C\n", temperature);
    }
    
    return NULL;
}

unsigned char serialRead(const int fd) {
    unsigned char x;
    if(read(fd, &x, 1) != 1)
        return -1;
   
    return x;
}

void serialWrite(const int fd, const char *s){
    write(fd,s,strlen(s));
}

// 센서2: 초음파 센서

float distance;
    
void* cho_thread(void* arg) {
    int trig = 15;
    int echo = 18;
    int start_time, end_time;

    if (wiringPiSetupGpio() == -1) {
        fprintf(stderr, "wiringPiSetupGpio() failed\n");
        return NULL;
    }

    int i = 0;
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);

    while (1) {
        i = 0;
        digitalWrite(trig, LOW);
        delay(1000);
        digitalWrite(trig, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig, LOW);

        while (digitalRead(echo) == 0);
        start_time = micros();

        while (digitalRead(echo) == 1);

        end_time = micros();
        distance = (end_time - start_time) / 29. / 2. * 10;
        printf("distance %.2f mm\n", distance);
        if(distance < 150.0) {
            if(temp == 'c') {
                fflush(stdout);
                serialWrite(fd_serial,"a");
                printf("a 전송 성공!\n");
                fflush(stdout);
            }
        }
		//delay(500);
    }

    return NULL;
}

// 센서3: 가속도 센서
void* spi_thread(void* arg) {
    unsigned char buffer[100];
    short x, y = 0, z = 0;
    short p_x=0, p_y=0, p_z=0;

    if (wiringPiSPISetupMode(SPI_CH, SPI_SPEED, SPI_MODE) == -1) {
        fprintf(stderr, "wiringPiSPISetupMode() failed\n");
        return NULL;
    }

    pinMode(CS_GPIO, OUTPUT);
    digitalWrite(CS_GPIO, HIGH);

    writeRegister_ADXL345(DATA_FORMAT, 0x01);
    writeRegister_ADXL345(BW_RATE, 0x0C);
    writeRegister_ADXL345(POWER_CTL, 0x08);

    while (1) {
        p_x = x;
        p_y = y;
        //p_z = z;
        readRegister_ADXL345(DATAX0, 6, buffer);
        x = ((short)buffer[2] << 8) | (short)buffer[1];
        y = ((short)buffer[4] << 8) | (short)buffer[3];
        z = ((short)buffer[6] << 8) | (short)buffer[5];
		delay(1000);
        printf("\n\n출력 데이터 => X : %d, Y: %d Z : %d\n", x, y, z);
        if(p_x > x+5 | p_x < x-5 | p_y > y+5 | p_y < y-5) {
            if(temp == 'c') {
                fflush(stdout);
                serialWrite(fd_serial,"a");
                printf("a 전송 성공!\n");
                fflush(stdout);
            }
        }
        //delay(500);
    }

    return NULL;
}

// 4: UART통신
void* uart_thread(void* arg) {

    if(wiringPiSetup() < 0)
        return NULL;

    if((fd_serial = serialOpen(UART2_DEV, BAUD_RATE)) < 0) {
        printf("Unable to open serial device (BT).\n");
        return NULL;
    }

    while(1) {
        while(serialDataAvail(fd_serial)) {
            dat = serialRead(fd_serial);
            printf("%c", dat);
            if(dat == 'c') {
                temp = 'c';
                printf("temp에 c를 넣었습니다!\n");
            }
            else if(dat == 'o') {
                temp = 'o';
                printf("temp에 o를 넣었습니다!\n");
            }
            delay(1000);
            //serialFlush(fd_serial);
        }
        fflush(stdout);
        delay(10);
    }

}
int main() {
    // wiringPi 초기화
    if (wiringPiSetupGpio() < 0) {
        fprintf(stderr, "wiringPiSetupGpio() failed\n");
        return 1;
    }

    // 각 센서 스레드 생성
    pthread_t sensor_tid, cho_tid, spi_tid, uart_tid;

    if (pthread_create(&sensor_tid, NULL, sensor_thread, NULL) != 0) {
        perror("Failed to create sensor thread");
        exit(EXIT_FAILURE);
    }

    if (pthread_create(&cho_tid, NULL, cho_thread, NULL) != 0) {
        perror("Failed to create cho thread");
        exit(EXIT_FAILURE);
    }

    if (pthread_create(&spi_tid, NULL, spi_thread, NULL) != 0) {
        perror("Failed to create spi thread");
        exit(EXIT_FAILURE);
    }
    
    if (pthread_create(&uart_tid, NULL, uart_thread, NULL) != 0) {
        perror("Failed to create spi thread");
        exit(EXIT_FAILURE);
    }

    // 스레드 종료 대기 및 반환값 무시
    pthread_join(sensor_tid, NULL);
    pthread_join(cho_tid, NULL);
    pthread_join(spi_tid, NULL);
    pthread_join(uart_tid, NULL);

    return 0;
}

