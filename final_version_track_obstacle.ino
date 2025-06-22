#include <Servo.h>

// === PIN ASSIGNMENTS ===
#define LFSensor_0 A0
#define LFSensor_1 A1
#define LFSensor_2 A2
#define LFSensor_3 A3
#define LFSensor_4 A4

#define speedPinR 3
#define RightMotorDirPin1 12
#define RightMotorDirPin2 11
#define speedPinL 6
#define LeftMotorDirPin1 7
#define LeftMotorDirPin2 8

#define trigPin 10
#define echoPin 2

#define FAST_SPEED 100
#define SLOW_SPEED 65

Servo head; // Déclaration du servo moteur

// === MOTOR CONTROL FUNCTIONS ===
void go_Advance() {
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);
}

void go_Back() {
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, HIGH);
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, HIGH);
}

void go_Left() {
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, HIGH);
}

void go_Right() {
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, HIGH);
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);
}

void stop_Stop() {
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, LOW);
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, LOW);
}

void set_Motorspeed(int speed_L, int speed_R) {
  analogWrite(speedPinL, speed_L);
  analogWrite(speedPinR, speed_R);
}

// === INITIALIZATION ===
void init_GPIO() {
  pinMode(RightMotorDirPin1, OUTPUT);
  pinMode(RightMotorDirPin2, OUTPUT);
  pinMode(speedPinR, OUTPUT);
  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT);
  pinMode(speedPinL, OUTPUT);

  pinMode(LFSensor_0, INPUT);
  pinMode(LFSensor_1, INPUT);
  pinMode(LFSensor_2, INPUT);
  pinMode(LFSensor_3, INPUT);
  pinMode(LFSensor_4, INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  stop_Stop();
}

// === READ LINE SENSORS ===
String read_sensor_values() {
  int sensorvalue = 32;
  int sensor[5];
  sensor[0] = !digitalRead(LFSensor_0);
  sensor[1] = !digitalRead(LFSensor_1);
  sensor[2] = !digitalRead(LFSensor_2);
  sensor[3] = !digitalRead(LFSensor_3);
  sensor[4] = !digitalRead(LFSensor_4);
  sensorvalue += sensor[0]*16 + sensor[1]*8 + sensor[2]*4 + sensor[3]*2 + sensor[4];
  String senstr = String(sensorvalue, BIN);
  senstr = senstr.substring(1, 6); // Keep only 5 bits
  return senstr;
}

// === READ ULTRASONIC SENSOR ===
long readUltrasonicDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2; // distance in cm
  return distance;
}

// === LINE TRACKING LOGIC ===
void auto_tracking() {
  String sensorval = read_sensor_values();
  Serial.println("Line: " + sensorval);

  if (sensorval == "10000"  || sensorval == "01000" || sensorval == "11000") {
    go_Left();
    set_Motorspeed(FAST_SPEED, FAST_SPEED);
  }
  else if (sensorval == "00001" || sensorval == "00010" || sensorval == "00011") {
    go_Right();
    set_Motorspeed(FAST_SPEED, FAST_SPEED);
  }
  else if (sensorval == "00000") {
    stop_Stop();
    set_Motorspeed(0, 0);
  }
  else if (sensorval == "11111" || sensorval == "00100") {
    go_Advance();
    set_Motorspeed(FAST_SPEED, FAST_SPEED); // move straight
  }
  else if (sensorval == "01100" || sensorval == "10100" || sensorval == "11100") {
    go_Advance();
    set_Motorspeed(SLOW_SPEED, FAST_SPEED); // slight left
  }
  else if (sensorval == "00101" || sensorval == "00110" || sensorval == "00111") {
    go_Advance();
    set_Motorspeed(FAST_SPEED, SLOW_SPEED); // slight right
  }
}

// === OBSTACLE AVOIDING LOGIC ===
void auto_avoidance() {
  Serial.println("Obstacle detected!");

  stop_Stop();
  delay(1000);

  // Turn right to avoid
  go_Left();
  set_Motorspeed(FAST_SPEED, FAST_SPEED);
  delay(1000);

  // Move forward to bypass the obstacle
  go_Advance();
  set_Motorspeed(FAST_SPEED, FAST_SPEED);
  delay(1000);

  // Turn right to avoid
  go_Right();
  set_Motorspeed(FAST_SPEED, FAST_SPEED);
  delay(1000);

  // Move forward to bypass the obstacle
  go_Advance();
  set_Motorspeed(FAST_SPEED, FAST_SPEED);
  delay(1000);

  // Turn right to avoid
  go_Right();
  set_Motorspeed(FAST_SPEED, FAST_SPEED);
  delay(1000);

  // Move forward to bypass the obstacle
  go_Advance();
  set_Motorspeed(FAST_SPEED, FAST_SPEED);
  delay(1000);

  // Turn right to avoid
  go_Left();
  set_Motorspeed(FAST_SPEED, FAST_SPEED);
  delay(1000);

  stop_Stop();
  delay(100);

  // Try to find the line again
  while (true) {
    String sensorval = read_sensor_values();
    Serial.println("Searching line: " + sensorval);       

    if(
      sensorval == "00001" || sensorval == "00010"
    || sensorval == "00011" || sensorval == "00100" 
    || sensorval == "00101" || sensorval == "00110" 
    || sensorval == "00111" || sensorval == "01000" 
    || sensorval == "01001" || sensorval == "01010" 
    || sensorval == "01011" || sensorval == "01100" 
    || sensorval == "01101" || sensorval == "01111" 
    || sensorval == "10001" || sensorval == "10010" 
    || sensorval == "10011" || sensorval == "10100" 
    || sensorval == "10101" || sensorval == "10110" 
    || sensorval == "10111" || sensorval == "11000" 
    || sensorval == "11001" || sensorval == "11010" 
    || sensorval == "11011" || sensorval == "11100" 
    || sensorval == "11101" || sensorval == "11111"
    )
    {
      break;
    }
  }
}

// === SETUP AND LOOP ===
void setup() {
  Serial.begin(9600);
  init_GPIO();
  head.attach(9); // ATTACHER ton servo sur le pin 5 (change si nécessaire)
  head.write(90); // Mettre immédiatement le servo à 90°
}

unsigned long lastUltrasonicCheck = 0;
const unsigned long ultrasonicInterval = 500; // Mesure toutes les 100ms

void loop() {
  unsigned long currentMillis = millis();

  // Vérifier la distance toutes les 100ms
  if (currentMillis - lastUltrasonicCheck >= ultrasonicInterval) {
    lastUltrasonicCheck = currentMillis;
    long distance = readUltrasonicDistance();

    // --- FILTRE ---
    if (distance > 0 && distance < 30) { 
      auto_avoidance();
    }
    // sinon, on ne fait rien (distance fausse ignorée)
  }
  auto_tracking();
}

