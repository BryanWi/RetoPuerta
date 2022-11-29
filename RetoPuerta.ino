//Sensor
#include <Wire.h>
#include <VL53L0X.h>
double sensorData=0;
VL53L0X sensor;

//Variables PID y sensor
//https://www.teachmemicro.com/arduino-pid-control-tutorial/
double kp = 0.5;
double ki = 2;
double kd = 0;

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input;
double output;
double setPoint = 50;
double cumError, rateError;
unsigned long PIDTimer;
//variables control
bool encendido = true;

//Pines para control del driver de motores Stepper
#define dirPin D8
#define stepPin D7
#define stpRev 200
bool motorOn = false;
unsigned long motorTimer=0;


//Librerias para WIFI y comunicación por Socket.IO
#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

#include <ArduinoJson.h>

#include <WebSocketsClient.h>
#include <SocketIOclient.h>

#include <Hash.h>

ESP8266WiFiMulti WiFiMulti;
SocketIOclient socketIO;

#define USE_SERIAL Serial

void socketIOEvent(socketIOmessageType_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case sIOtype_DISCONNECT:
      USE_SERIAL.printf("[IOc] Disconnected!\n");
      break;
    case sIOtype_CONNECT:
      USE_SERIAL.printf("[IOc] Connected to url: %s\n", payload);

      // join default namespace (no auto join in Socket.IO V3)
      socketIO.send(sIOtype_CONNECT, "/");
      break;
    case sIOtype_EVENT:
      USE_SERIAL.printf("[IOc] get event: %s\n", payload[0]);
      break;
    case sIOtype_ACK:
      USE_SERIAL.printf("[IOc] get ack: %u\n", length);
      hexdump(payload, length);
      break;
    case sIOtype_ERROR:
      USE_SERIAL.printf("[IOc] get error: %u\n", length);
      hexdump(payload, length);
      break;
    case sIOtype_BINARY_EVENT:
      USE_SERIAL.printf("[IOc] get binary: %u\n", length);
      hexdump(payload, length);
      break;
    case sIOtype_BINARY_ACK:
      USE_SERIAL.printf("[IOc] get binary ack: %u\n", length);
      hexdump(payload, length);
      break;
  }
}

void setup() {
  // USE_SERIAL.begin(921600);
  USE_SERIAL.begin(115200);

  //Serial.setDebugOutput(true);
  USE_SERIAL.setDebugOutput(true);

  USE_SERIAL.println();
  USE_SERIAL.println();
  USE_SERIAL.println();

  for (uint8_t t = 4; t > 0; t--) {
    USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
    USE_SERIAL.flush();
    delay(1000);
  }

  // disable AP
  if (WiFi.getMode() & WIFI_AP) {
    WiFi.softAPdisconnect(true);
  }

  WiFiMulti.addAP("WebSocketTest", "0987654321");

  //WiFi.disconnect();
  while (WiFiMulti.run() != WL_CONNECTED) {
    delay(5000);
    //WiFiMulti.addAP("WebSocketTest", "0987654321");
  }

  String ip = WiFi.localIP().toString();
  USE_SERIAL.printf("[SETUP] WiFi Connected %s\n", ip.c_str());

  // server address, port and URL
  socketIO.begin("192.168.137.1", 5000, "/socket.io/?EIO=4");

  // event handler
  socketIO.onEvent(socketIOEvent);

  //Sensor
  Wire.begin(D1, D2);

  //sensor.setMeasurementTimingBudget(20000);
  sensor.setTimeout(500);
  sensor.init();
  /*if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {Serial.println("Failed to detect and initialize sensor!");delay(1000);}
  }*/

  //Definir pines de salida y entrada
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
}

unsigned long messageTimestamp = 0;
void loop() {
  socketIO.loop();
  uint64_t now = millis();
  
  //Checar por señal de paro/cambios de variables
  //Leer Sensor
  if ((now - PIDTimer) > 2000){
  PIDTimer = now;
  sensorData = leerSensor();
  input = sensorData;
  //Calcular acción con PID
  output = computePID(sensorData);
  Serial.print(output);
  /*
  Serial.print("input: ");
  Serial.print(input);
  Serial.print(" output: ");
  Serial.println(output);
  */
  //Enviar dato del sensor
  //enviarDato();
  }



  if (now - messageTimestamp > 2000) {
    messageTimestamp = now;

    // creat JSON message for Socket.IO (event)
    DynamicJsonDocument doc(1024);
    JsonArray array = doc.to<JsonArray>();

    // add evnet name
    // Hint: socket.on('event_name', ....
    array.add("message");

    // add payload (parameters) for the event
    JsonObject param1 = array.createNestedObject();
    param1["sData"] = (uint32_t)sensorData;
    param1["output"] = (int32_t)output;
    param1["SP"] = (int32_t)setPoint;
    // JSON to String (serializion)
    String output;
    serializeJson(doc, output);

    // Send event
    socketIO.sendEVENT(output);

    // Print JSON for debugging
    //USE_SERIAL.println(output);
  }

  //si (encencido) Mover motor N pasos en la dirección indicada por PID(N=1 paso de momento)
  uint64_t nowMicros = micros();
  encendido = true;
  if (encendido) {

    uint64_t motorDelay = max(500.0,1000/(output/1000000));
    /*Serial.println();
    
    Serial.print("input: ");
    Serial.print(input);
    Serial.print("output: ");
    Serial.print(output);
    
    Serial.print("timeSinceStep: ");
    Serial.println(nowMicros - motorTimer);
    Serial.print(" motorDelay: ");
    Serial.println(motorDelay);*/
    if ((nowMicros - motorTimer) >= 2500){
      motorTimer = nowMicros;

      if (output >0){
        digitalWrite(dirPin,HIGH);
      }else{
        digitalWrite(dirPin,LOW);
      }

      if (!motorOn){
        digitalWrite(stepPin,HIGH);
        motorOn = true;
      } else{
        digitalWrite(stepPin,LOW);
        motorOn = false;
      }
    }

  }
}

//--------------Funciones Control--------------//
// Obtiene la lectura del sensor
double leerSensor() {
  return sensor.readRangeSingleMillimeters();
}
// Control PID
double computePID(double inp) {
  currentTime = millis();                              //get current time
  elapsedTime = (double)(currentTime - previousTime);  //compute time elapsed from previous computation
  inp = min(120.0,inp);
  error = setPoint - inp;                         // determine error
  cumError += error * elapsedTime;                // compute integral
  rateError = (error - lastError) / elapsedTime;  // compute derivative

  double out = kp * error + ki * cumError + kd * rateError;  //PID output

  lastError = error;           //remember current error
  previousTime = currentTime;  //remember current time

  return out;  //have function return the PID output
}

//--------------Funciones Stepper--------------//
//Steps permite dar n cantidad de pasos a cierta velocidad y el la dirección indicada
void steps(int n, int speed, bool dir) {
  digitalWrite(dirPin, dir);
  for (int i = 0; i < n; i++) {
    step(speed);
  }
}
//Da un solo paso con el tiempo de espera que le pasen
void step(int speed) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(speed);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(speed);
}