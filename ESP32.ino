#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "First Floor 2.4G";       
const char* password = "03074777793";        

WebServer server(80);

// Motor Pins
#define ENA 2
#define IN1 25
#define IN2 26
#define IN3 27
#define IN4 14
#define ENB 21

// Ultrasonic Pins
#define TRIG 32
#define ECHO 33

// Motor functions
void forward() {
  digitalWrite(ENA,HIGH); digitalWrite(ENB,HIGH);
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
}

void left() {
  digitalWrite(ENA,HIGH); digitalWrite(ENB,HIGH);
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW); digitalWrite(IN4,LOW);
}

void right() {
  digitalWrite(ENA,HIGH); digitalWrite(ENB,HIGH);
  digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
}

void stopMotors() {
  digitalWrite(ENA,LOW); digitalWrite(ENB,LOW);
  digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW); digitalWrite(IN4,LOW);
}

// Ultrasonic distance function
long readUltrasonicCM() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, 30000); // timeout 30ms
  long distance = duration / 58; // cm
  return distance;
}

// HTTP handlers
void handleForward() { forward(); server.send(200,"text/plain","FORWARD"); }
void handleLeft() { left(); server.send(200,"text/plain","LEFT"); }
void handleRight() { right(); server.send(200,"text/plain","RIGHT"); }
void handleStop() { stopMotors(); server.send(200,"text/plain","STOP"); }
void handleDistance() {
  long dist = readUltrasonicCM();
  server.send(200,"text/plain",String(dist));
}

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(ENA,OUTPUT); pinMode(ENB,OUTPUT);
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);

  // Ultrasonic pins
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);

  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid,password);
  while(WiFi.status() != WL_CONNECTED){ delay(500); Serial.print("."); }
  Serial.println("\nWi-Fi connected!");
  Serial.print("ESP32 IP Address: "); Serial.println(WiFi.localIP());

  // Routes
  server.on("/forward", handleForward);
  server.on("/left", handleLeft);
  server.on("/right", handleRight);
  server.on("/stop", handleStop);
  server.on("/distance", handleDistance);

  server.begin();
  Serial.println("Server started");
}

void loop() {
  server.handleClient();
}
