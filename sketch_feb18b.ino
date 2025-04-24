#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>

#include <AccelStepper.h>

// WiFi and MQTT Setup
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Stepper Motor Pins (ULN2003 Driver)
#define IN1 26
#define IN2 25
#define IN3 33
#define IN4 32

// Encoder Pin
#define ENCODER_A 2  // Change if using a different pin

//Relay Pin (Cart Forward Reverse Control)
#define FORWARD_PIN 18
bool forwardRelayState = false;
#define REVERSE_PIN 19
bool reverseRelayState = false;

//Linear actuator pin
#define IN5 22
#define IN6 23

bool isextended = false;
bool isretracted = false;

//Stop relay
#define STOP_PIN 4
bool stopRelayState = false;



// Stepper Motor Setup
AccelStepper stepper(AccelStepper::HALF4WIRE, IN1, IN3, IN2, IN4);

// Encoder Variables
volatile unsigned long pulseCount = 0;
unsigned long prevTime = 0;
int PPR = 400;  // Set this according to your encoder specs

// Speed Calculation Constants
const float GEAR_RATIO = 300.0 / 185.0;
//const float WHEEL_DIAMETER = 0.27;
const float WHEEL_CIRCUMFERENCE = 0.00027;
float cartSpeed = 0;

void countPulses() {
  pulseCount++;  // Interrupt Service Routine (ISR)
}

void receiveCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received [");
  Serial.print(topic);
  Serial.print("]: ");

  char payloadChar[length + 1];
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    payloadChar[i] = (char)payload[i];
  }
  payloadChar[length] = '\0';  // Null terminate string
  Serial.println();

  if (strcmp(topic, "MOTOR-CONTROL") == 0) {
    if (payloadChar[0] == '1') {
      stepper.moveTo(stepper.currentPosition() + 512);
      while (stepper.distanceToGo() != 0) stepper.run();
    }
    if (payloadChar[0] == '2') {
      stepper.moveTo(stepper.currentPosition() - 512);
      while (stepper.distanceToGo() != 0) stepper.run();
    }
  }

  if (strcmp(topic, "FORWARD") == 0) {
    if ((payloadChar[0] == '1') && (forwardRelayState == false)) {
      digitalWrite(FORWARD_PIN, HIGH);
      forwardRelayState = true;
    } else if (payloadChar[0] == '0') {
      digitalWrite(FORWARD_PIN, LOW);
      forwardRelayState = false;
      reverseRelayState = false;
    }
  }

  if (strcmp(topic, "REVERSE") == 0) {
    if ((payloadChar[0] == '1') && (reverseRelayState == false)) {
      digitalWrite(REVERSE_PIN, HIGH);
      reverseRelayState = true;
    } else if (payloadChar[0] == '0') {
      digitalWrite(REVERSE_PIN, LOW);
      forwardRelayState = false;
      reverseRelayState = false;
    }
  }

  if (strcmp(topic, "STOP") == 0) {
    if (payloadChar[0] == '1') {
      digitalWrite(STOP_PIN, HIGH);
      stopRelayState = true;
      retract();
      delay(5000);
      // isretracted = true;
      // isextended = false;
    } else if (payloadChar[0] == '0' ) {
      extend();
      delay(5000);
      digitalWrite(STOP_PIN, LOW);
      stopRelayState = false;
      // isretracted = false;
      // isextended = true;
    }
  }

}

void setupWifi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin("V2027", "2fec9f17ae7f");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void setupMqtt() {
  mqttClient.setServer("broker.mqtt.cool", 1883);
  mqttClient.setCallback(receiveCallback);
}

void connectToBroker() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT...");
    if (mqttClient.connect("ESP32-4654565646")) {
      Serial.println("MQTT Connected!");
      mqttClient.subscribe("MOTOR-CONTROL");
      mqttClient.subscribe("FORWARD");
      mqttClient.subscribe("REVERSE");
      mqttClient.subscribe("STOP");
    } else {
      Serial.print("Failed: ");
      Serial.println(mqttClient.state());
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);

  pinMode(FORWARD_PIN, OUTPUT);
  pinMode(REVERSE_PIN, OUTPUT);
  digitalWrite(FORWARD_PIN, LOW);
  digitalWrite(REVERSE_PIN, LOW);

  pinMode(STOP_PIN, OUTPUT);
  digitalWrite(STOP_PIN, LOW);

  pinMode(ENCODER_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), countPulses, RISING);

  stepper.setMaxSpeed(600);
  stepper.setAcceleration(300);

  setupWifi();
  setupMqtt();
}

void loop() {
  if (!mqttClient.connected()) connectToBroker();
  mqttClient.loop();

  sendspeed();

  delay(1000);
}


void sendspeed() {
  if (millis() - prevTime >= 1000) {
    noInterrupts();
    float stepperRPM = (pulseCount * 60.0) / PPR;
    pulseCount = 0;
    prevTime = millis();
    interrupts();

    float wheelRPM = stepperRPM / GEAR_RATIO;
    //float wheelRPS = wheelRPM / 60.0;
    float speed_mps = wheelRPM * WHEEL_CIRCUMFERENCE * 60;
    cartSpeed = speed_mps;  // Convert to km/h

    Serial.print("Cart Speed: ");
    //Serial.print(cartSpeed);
    Serial.print(stepperRPM);
    Serial.println(" km/h");

    char speedMessage[10];
    dtostrf(cartSpeed, 6, 2, speedMessage);
    mqttClient.publish("CART-SPEED", speedMessage);
  }
}

void extend() {
    digitalWrite(IN5, HIGH);
    digitalWrite(IN6, LOW);
}

void retract() {
    digitalWrite(IN5, LOW);
    digitalWrite(IN6, HIGH);
}



