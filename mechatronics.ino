           #include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <NewPing.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

// 센서 및 액추에이터 설정
#define TRIGGER_PIN 19
#define ECHO_PIN 23
#define MAX_DISTANCE 200
#define DHTPIN 18
#define DHTTYPE DHT11
#define SERVOPIN 26
#define VIBRATION_PIN 27
#define LIGHT_SENSOR_PIN 36
#define RED_PIN 25
#define GREEN_PIN 33
#define BLUE_PIN 32
#define MQ135_PIN 34
#define RX 16
#define TX 17
#define BUSY_PIN 14
#define CRASH_SENSOR_PIN 4

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
DHT dht(DHTPIN, DHTTYPE);
Servo myservo;
Adafruit_MPU6050 mpu;
SoftwareSerial mySerial(RX, TX);  // RX, TX
DFRobotDFPlayerMini MP3Player;

// Wi-Fi 설정
const char* ssid = "Gwak";
const char* password = "rhkrxowls";

// 타이머 설정
volatile int interruptCounter;
int totalInterruptCounter;
int danger_time;
int danger_danger_time;
int gasgas;
hw_timer_t* timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  Serial.begin(115200);
  mySerial.begin(9600);
  MP3Player.begin(mySerial, 0, 1);
  MP3Player.volume(30);

  // 타이머 초기화
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 10000, true);
  timerAlarmEnable(timer);

  Wire.begin();
  dht.begin();
  myservo.attach(SERVOPIN);
  pinMode(VIBRATION_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(CRASH_SENSOR_PIN, INPUT_PULLUP);

  // MPU6050 초기화
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    //while (1);
  }

  // Wi-Fi 연결
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    MP3Player.playFolder(2, 004);
    delay(850);
    while (!digitalRead(BUSY_PIN)) {
      Serial.println("재생중... ");
    }
    Serial.println(digitalRead(BUSY_PIN));
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  MP3Player.playFolder(2, 005);
  delay(850);
  while (!digitalRead(BUSY_PIN)) {
    Serial.println("재생중... ");
  }
  MP3Player.playFolder(2, 003);
  delay(850);
  while (!digitalRead(BUSY_PIN)) {
    Serial.println("재생중... ");
  }
}

unsigned long now{};
void loop() {
  now = millis();
  if (interruptCounter > 0) {
    portENTER_CRITICAL(&timerMux);
    interruptCounter==0;
    portEXIT_CRITICAL(&timerMux);

    totalInterruptCounter++;
    //Serial.print("An interrupt has occurred. Total number: ");
    //Serial.println(totalInterruptCounter);

    /***************************센서값 읽기***************************/

    // 초음파 센서로 거리 측정
    int distance = sonar.ping_cm();

    // 서보 모터 제어
    for (int pos = 0; pos <= 180; pos += 10) {
      myservo.write(pos);
    }
    for (int pos = 180; pos >= 0; pos -= 10) {
      myservo.write(pos);
    }

    // 가속도 및 자이로 센서 읽기
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // 온습도 센서 읽기
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    // 조도 센서 읽기
    int lightLevel = analogRead(LIGHT_SENSOR_PIN);

    // 가스 센서 읽기
    int gasLevel = analogRead(MQ135_PIN);

    /***************************센서값 으로 제어 하기*****************************/

    // ******장애물 감지 시 RGB LED 제어
    if (a.acceleration.y < -8 && lightLevel < 1000) {
      digitalWrite(VIBRATION_PIN, HIGH);
      setColor(0, 0, 255);
      MP3Player.playFolder(2, 002);
      delay(850);
      while (!digitalRead(BUSY_PIN)) {
        Serial.println("재생중... ");
      }
    } else if (distance < 50 && distance > 0) {
      digitalWrite(VIBRATION_PIN, HIGH);
      setColor(255, 0, 0);  // 장애물 감지 시 빨간색 LED 켜기
      danger_time++;
      if (danger_time > 100) {
        MP3Player.playFolder(2, 001);
        delay(850);
        while (!digitalRead(BUSY_PIN)) {
          Serial.println("재생중... ");
        }
        danger_time = 0;
        danger_danger_time++;
      }
      //위험한 상황 파악
      if (danger_danger_time > 5) {
        while (danger_danger_time < 11 && danger_danger_time > 5) {
          MP3Player.playFolder(2, 006);
          setColor(255, 0, 255);
          delay(850);
          while (!digitalRead(BUSY_PIN)) {
            Serial.println("재생중... ");
          }
          danger_danger_time++;
          //매우 위험한 상황 파악
          if (danger_danger_time > 10 ) {
            MP3Player.playFolder(2, 9);
            while (!digitalRead(BUSY_PIN)) {
              Serial.println("재생중... ");
            }
            MP3Player.playFolder(2, 10);
            help();
            while (!digitalRead(BUSY_PIN)) {
              Serial.println("구난신호... ");
            }
          }

          //구난신호 보내기
          if (!digitalRead(CRASH_SENSOR_PIN)) {
            MP3Player.playFolder(2, 007);
            delay(850);
            while (!digitalRead(BUSY_PIN)) {
              Serial.println("재생중... ");
            }
            MP3Player.playFolder(2, 11);
            delay(850);
            while (!digitalRead(BUSY_PIN)) {
              Serial.println("구난신호... ");
            }
            MP3Player.playFolder(2, 10);
            help();
            while (!digitalRead(BUSY_PIN)) {
              Serial.println("구난신호... ");
            }
            danger_danger_time = 0;
          }
        }
      }
    } else {
      digitalWrite(VIBRATION_PIN, LOW);
      setColor(0, 255, 0);  // 장애물 없을 때 초록색 LED 켜기
    }


    // 장애물 감지 시 진동 모터 제어
    if (distance < 50) {
      digitalWrite(VIBRATION_PIN, HIGH);
    } else {
      digitalWrite(VIBRATION_PIN, LOW);
    }



    //******충격 감지 시 위험 메세지 출력
    if (g.gyro.x > 4 || g.gyro.y > 4 || g.gyro.z > 4) {
      danger_danger_time = 6;
      while (danger_danger_time < 11 && danger_danger_time > 5) {
        digitalWrite(VIBRATION_PIN, HIGH);
        setColor(255, 0, 255);
        MP3Player.playFolder(2, 006);
        delay(850);
        while (!digitalRead(BUSY_PIN)) {
          Serial.println("재생중... ");
        }
        danger_danger_time++;

        //매우 위험한 상황 파악
        if (danger_danger_time > 10) {
          MP3Player.playFolder(2, 9);
          delay(850);
          while (!digitalRead(BUSY_PIN)) {
            Serial.println("재생중... ");
          }
          MP3Player.playFolder(2, 10);
          delay(850);
          help();
          while (!digitalRead(BUSY_PIN)) {
            Serial.println("구난신호... ");
        }
        }

        // 구난신호 보내기
        if (!digitalRead(CRASH_SENSOR_PIN)) {
          MP3Player.playFolder(2, 007);
          delay(850);
          while (!digitalRead(BUSY_PIN)) {
            Serial.println("재생중... ");
          }
          MP3Player.playFolder(2, 11);
          delay(850);
          while (!digitalRead(BUSY_PIN)) {
            Serial.println("구난신호... ");
          }
          MP3Player.playFolder(2, 10);
          delay(850);
          help();
          while (!digitalRead(BUSY_PIN)) {
            Serial.println("구난신호... ");
          }
          danger_danger_time = 0;
        }
      }
    }


    //*****가스 감지 안내
    if (gasLevel > 1000) {
      gasgas++;
      if (gasgas > 1000) {
        setColor(255, 255, 0);
        MP3Player.playFolder(2, 8);
        delay(850);
        while (!digitalRead(BUSY_PIN)) {
          Serial.println("재생중... ");
        }
        gasgas = 0;
      }
    }

    // 센서 데이터 출력
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.println(a.acceleration.z);

    Serial.print("Rotation X: ");
    Serial.print(g.gyro.x);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    Serial.print(", Z: ");
    Serial.println(g.gyro.z);

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");

    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");

    Serial.print("Light Level: ");
    Serial.println(lightLevel);

    Serial.print("Gas Level: ");
    Serial.println(gasLevel);
    // Wi-Fi를 통해 데이터 전송 (생략)
    // ...
  }
  while(millis() < now + 10);
}

void setColor(int red, int green, int blue) {
  analogWrite(RED_PIN, red);
  analogWrite(GREEN_PIN, green);
  analogWrite(BLUE_PIN, blue);
}

    //구난신호
  void
  help() {
  setColor(255, 255, 255);  //S
  delay(250);
  setColor(0, 0, 0);
  delay(250);
  setColor(255, 255, 255);
  delay(250);
  setColor(0, 0, 0);
  delay(250);
  setColor(255, 255, 255);
  delay(250);
  setColor(0, 0, 0);
  delay(250);
  setColor(255, 255, 255);  //O
  delay(750);
  setColor(0, 0, 0);
  delay(750);
  setColor(255, 255, 255);
  delay(750);
  setColor(0, 0, 0);
  delay(750);
  setColor(255, 255, 255);
  delay(750);
  setColor(0, 0, 0);
  delay(750);
  setColor(255, 255, 255);  //S
  delay(250);
  setColor(0, 0, 0);
  delay(250);
  setColor(255, 255, 255);
  delay(250);
  setColor(0, 0, 0);
  delay(250);
  setColor(255, 255, 255);
  delay(250);
  setColor(0, 0, 0);
  delay(250);
}
