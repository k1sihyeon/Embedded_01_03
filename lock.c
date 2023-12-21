#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <signal.h>
#include <softPwm.h>
#include <pthread.h>
#include <sys/wait.h>
#include <spawn.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>


extern char **environ;

//// servo ////
#define SERVO 19
#define BUTTON 21
//// servo ////


//// uart - bluetooth ////
#define BAUD_RATE   115200
#define BUZZER      26
#define BUF_SIZE    5

static const char* UART2_DEV = "/dev/ttyAMA1";
unsigned char buf[BUF_SIZE];

int fd_serial_bt;
//// uart - bluetooth ////


/// uart - rasp ///
#define R_BAUD_RATE   115200
#define R_BUF_SIZE  100
static const char* UART3_DEV = "/dev/ttyAMA2";
unsigned char r_buf[R_BUF_SIZE];

int fd_serial_rasp;
/// uart - rasp ///


//// fnd ////
#define GPIO_COUNT  12  //GPIO 포트 개수
#define DIGIT_COUNT 4   //FND 4개
#define SEG_COUNT   8   //7seg + dot
#define FONT_COUNT  20  //미리 정의된 폰트 개수
#define PRINT_TIME  2000 //출력 시간

//FND에 순서대로(1~12) 연결된 GPIO 포트들
const int gpio[GPIO_COUNT] = { 17, 27, 22, 10, 9, 11, 12, 7, 8, 25, 24, 23 };

//GPIO ports for the digit 0~3 pins(BCM)
int digits[DIGIT_COUNT];

//GPIO ports for the 7seg pins(BCM)
int segments[SEG_COUNT];

const int fndFont[FONT_COUNT][SEG_COUNT] = {
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
    //
    {1, 0, 0, 1, 1, 1, 0, 0}, //C   //10
    {0, 1, 1, 1, 1, 0, 1, 0}, //D   //11
    {1, 0, 0, 1, 1, 1, 1, 0}, //E   //12
    {0, 0, 0, 1, 1, 1, 0, 0}, //L   //13
    {0, 0, 1, 0, 1, 0, 1, 0}, //N   //14
    {1, 1, 1, 1, 1, 1, 0, 0}, //O   //15
    {1, 1, 0, 0, 1, 1, 1, 0}, //P   //16
    {1, 0, 1, 1, 0, 1, 1, 0}, //S   //17
    {0, 0, 0, 0, 1, 0, 1, 0}, //R   //18
    {0, 0, 0, 0, 0, 0, 0, 0}, //공백 //19
};

int str_clsd[DIGIT_COUNT] = { 10, 13, 17, 11,};
//// fnd ////


//// servo ////
void ServoInit() {
    pinMode(SERVO, OUTPUT);
    softPwmCreate(SERVO, 0, 200);
    pinMode(BUTTON, INPUT);
    pinMode(BUZZER, OUTPUT);
}

void ServoLock() {
    printf("servo lock\n");
    softPwmWrite(SERVO, 15);
    delay(100);
}

void ServoUnlock() {
    printf("servo unlock\n");
    softPwmWrite(SERVO, 24);
    //digitalWrite(BUZZER, HIGH);
    delay(100);
}
//// servo ////

char* serialReadRasp(const int fd) {
    // unsigned char buf[10];
    char c;
    size_t idx = 0;
    
    while (idx < sizeof(r_buf) - 1) {
        if (serialDataAvail(fd) > 0) {
            c = serialGetchar(fd);
            printf("serialReadRaspChar: %c\n", c);
            if (c == '\0') {
                break;  // 문자열의 끝을 나타내는 NULL 문자를 받으면 종료
            } else {
                r_buf[idx++] = c;
            }
        }
    }

    serialFlush(fd);

    r_buf[idx] = '\0';  // 문자열의 끝에 NULL 문자 추가

    printf("serialReadRaspString: %s\n", r_buf);
    fflush(stdout);

    return r_buf;

}

unsigned char serialRead(const int fd) {
    unsigned char x;
    if(read(fd, &x, 1) != 1) {
        return -1;
    }

    printf("serialRead: %c\n", x);
        
    return x;
}


//// bluetooth ////
char* serialReadString(const int fd) {
    //unsigned char buf[10];
    char c;
    size_t idx = 0;
    
    while (idx < sizeof(buf) - 1) {
        if (serialDataAvail(fd) > 0) {
            c = serialGetchar(fd);
            printf("serialRead: %c\n", c);
            if (c == '\0') {
                break;  // 문자열의 끝을 나타내는 NULL 문자를 받으면 종료
            } else {
                buf[idx++] = c;
            }
        }
    }

    serialFlush(fd);

    buf[idx] = '\0';  // 문자열의 끝에 NULL 문자 추가

    printf("serialReadstring: %s\n", buf);
    fflush(stdout);

    return buf;
}

void serialWrite(const int fd, const unsigned char c) {
    write(fd, &c, 1);
}

void serialWriteBytes(const int fd, const char *s) {
    write(fd, s, strlen(s));
}
//// bluetooth ////


//// fnd ////
void ClearPinMap() {
    printf("\ngood-bye\n");

    for (int b = 0; b < SEG_COUNT; b++) {
        digitalWrite(segments[b], LOW);
    }

    for (int b = 0; b < DIGIT_COUNT; b++) {
        digitalWrite(digits[b], LOW);
    }

    pthread_exit(NULL);
    exit(0);
}

void FndSelect(int position) {
    
    for (int i = 0; i < DIGIT_COUNT; i++) {
        if (i == position) {
            digitalWrite(digits[i], LOW); //LOW가 on
        }
        else {
            digitalWrite(digits[i], HIGH); //HIGH가 off
        }
    }
}

void FndPositionPrint(int str, int position) {
   
    for(int j = 0; j < SEG_COUNT; j++) {
        digitalWrite(segments[j], fndFont[str][j]);
    }

    FndSelect(position);
}

void FndFlush() {
    for (int b = 0; b < SEG_COUNT; b++) {
        digitalWrite(segments[b], LOW);
    }

    for (int b = 0; b < DIGIT_COUNT; b++) {
        digitalWrite(digits[b], LOW);
    }
}

void FndPrint(int *strArr) {

    unsigned int time = millis();

    while(1) {
        for(int i = 0; i < DIGIT_COUNT; i++) {
            FndPositionPrint(strArr[i], i);
            delay(1);
        }

        //n초 동안 출력
        if(millis() - time > PRINT_TIME) { 
            time = millis();
            break;
        }
    }

    FndFlush();
}

void FndInit() {
    int gpioDigits[DIGIT_COUNT] = {11, 8, 7, 5};
    int segmentDigits[SEG_COUNT] = {10, 6, 3, 1, 0, 9, 4, 2};

    for (int i = 0; i < DIGIT_COUNT; i++) {
        digits[i] = gpio[gpioDigits[i]];
    }

    for (int i = 0; i < SEG_COUNT; i++) {
        segments[i] = gpio[segmentDigits[i]];
    }

    if(wiringPiSetupGpio()==-1){
    	printf("Wiringpi Error!");
        exit(-1);
    }

    for (int i = 0; i < SEG_COUNT; i++)
        pinMode(segments[i], OUTPUT);

    for(int i = 0; i < DIGIT_COUNT; i++) {
        pinMode(digits[i], OUTPUT);
        digitalWrite(digits[i], HIGH); //LOW가 on
    }

}
//// fnd ////

struct msg_buffer {
    long msg_type;
    int msg_int;
};


void *ThreadBuzzer(void *arg) {

    printf("[Thread] buzzer thread start!\n");

    while (1) {
        key_t key;
        int msgid;

        key = ftok("progfile", 65);
        msgid = msgget(key, 0666 | IPC_CREAT);

        struct msg_buffer message;
        msgrcv(msgid, &message, sizeof(message), 1, 0);

        printf("수신된 메시지: %d\n", message.msg_int);

        msgctl(msgid, IPC_RMID, NULL);

        if (message.msg_int == 1) {
            printf("buzzer on\n");
            digitalWrite(BUZZER, HIGH);
            delay(1000);
            digitalWrite(BUZZER, LOW);
        }
        else {
            printf("buzzer off\n");
            digitalWrite(BUZZER, LOW);
        }
    }
}

void *ThreadButton(void *arg) {


    signal(SIGINT, ClearPinMap);

    printf("[Thread] button thread start!\n");

    while (1) {
        if (digitalRead(BUTTON) == HIGH) {
            printf("[Thread] button pressed!\n");
            ServoLock();
            serialWriteBytes(fd_serial_bt, "Locked!!\n");
            // status = 0;
            FndPrint(str_clsd);
            serialWrite(fd_serial_rasp, 'c');

            //message queue
            key_t key;
            int msgid;

            key = ftok("progfile", 65);
            msgid = msgget(key, 0666 | IPC_CREAT);

            struct msg_buffer message;
            message.msg_type = 1;
            message.msg_int = 1;

            // 메시지 큐에 메시지 전송
            msgsnd(msgid, &message, sizeof(message), 0);
            printf("메시지 전송 완료\n");
        }
    }
}
// mq - button, buzzer

// uart - rasp //
void *ThreadRasp(void *arg) {
    printf("[Thread] raspberry pi uart start!\n");

    char *buf;
    char buf_c;

    while (1) {
        // rasp 읽기
        //buf = serialReadRasp(fd_serial_rasp);

        while(serialDataAvail(fd_serial_rasp)) {
            buf_c = serialRead(fd_serial_rasp);
            printf("[Thread] c: %c\n", buf_c);
            if (buf_c == 'a') {
            serialWriteBytes(fd_serial_bt, "Detected!!\n");
             }
            fflush(stdout);
        }

        delay(500);
    }
}
// uart - rasp //

int main(void) {

    pthread_t thread;
    pthread_t raspThread;
    //int fd_serial_bt;

    char *buf;
    int str_open[DIGIT_COUNT] = { 15, 16, 12, 14,};
    int str_clsd[DIGIT_COUNT] = { 10, 13, 17, 11,};
    int str_err[DIGIT_COUNT] = { 12, 18, 18, 19,};
    char *password = "1234";

    FndInit();
    ServoInit();
    signal(SIGINT, ClearPinMap);

    ServoUnlock();
    FndPrint(str_open);
    ServoLock();
    FndPrint(str_clsd);  
    
    if(wiringPiSetup() < 0)
        return 1;
    
    if(wiringPiSetupGpio() < 0)
        return 1;
    
    if((fd_serial_bt = serialOpen(UART2_DEV, BAUD_RATE)) < 0) {
        printf("Unable to open serial device (BT).\n");
        return 1;
    }

    if((fd_serial_rasp = serialOpen(UART3_DEV, R_BAUD_RATE)) < 0) {
        printf("Unable to open serial device (RASP).\n");
        return 1;
    }

    //button thread 생성
    if(pthread_create(&thread, NULL, ThreadButton, NULL) < 0) {
        printf("thread create error!\n");
        exit(0);
    }

    //rasp uart thread 생성
    if(pthread_create(&raspThread, NULL, ThreadRasp, NULL) < 0) {
        printf("thread create error\n");
        exit(0);
    }

    if(pthread_create(&thread, NULL, ThreadBuzzer, NULL) < 0) {
        printf("thread create error!\n");
        exit(0);
    }

    while(1) {
        int status;
        pid_t pid, r_pid;
        char *argv[] = {"nfc-poll", NULL};

        printf("nfc read start!\n");
        posix_spawn(&pid, "/bin/nfc-poll", NULL, NULL, argv, environ);
        if ((r_pid = waitpid(pid, &status, 0)) < 0) {
            perror("waitpid() error\n");
        }

        printf("bluetooth read start!\n");
        buf = serialReadString(fd_serial_bt);
        printf("main: %s\n", buf);
        fflush(stdout);

        //읽은 값 모두 출력
        for(int i = 0; i < BUF_SIZE; i++) {
            if(buf[i] == '\0') {
                break;
            }
            else if((buf[i] != '\0')) {
                printf("buf[%d]: %c\n", i, buf[i]);
                FndPositionPrint(buf[i] - '0', i);
                delay(500);
                fflush(stdout);
            }
            else {
                break;
            }
        }

        FndFlush();

        if(strcmp(buf, password) == 0) {
            printf("password correct!\n");
            serialWriteBytes(fd_serial_bt, "\npassword correct!\n");
            serialWrite(fd_serial_rasp, 'o');
            ServoUnlock();
            serialWriteBytes(fd_serial_bt, "Unlocked!!\n");
            FndPrint(str_open);
        }
        else {
            printf("password incorrect!\n");
            serialWriteBytes(fd_serial_bt, "\npassword incorrect!\n");
            FndPrint(str_err);
        }

        delay(10);
    }
    
}