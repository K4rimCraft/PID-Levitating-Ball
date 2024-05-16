#include <PID_v1.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>

unsigned long startMillis, startMillis2;
unsigned long currentMillis;
const unsigned int period = 10;
const unsigned int period2 = 50;

const int trigPin = D2;
const int echoPin = D3;
const int fanPin = D4;
double Setpoint, Input, Output;
double duration, distance, pp;
double P = 1;
double I = 2;
double D = 0.1;

PID myPID(&Input, &Output, &Setpoint, P, I, D, REVERSE);

const char* ssid = "GalaxyS10";                     // CHANGE IT
const char* password = "gnis2682";                  // CHANGE IT
WebSocketsServer webSocket = WebSocketsServer(81);  // Port 81


void setup() {
  // Set PWM frequency outside of the human hearing range.
  analogWriteFreq(25000);

  // Init.
  Setpoint = 15;
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(fanPin, OUTPUT);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(50);
  Serial.begin(9600);


  // Connect to Wi-Fi.
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Initialize WebSocket server.
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.print("ESP8266 Web Server's IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  currentMillis = millis();     // Get current time.
  webSocket.loop();             // Websocket Running.
  Input = getDistance();        // Get distance from HC-SR04.
  myPID.Compute();              // Compute the output for fan.
  notifyClients();              // Send data to application.
  analogWrite(fanPin, Output);  // Set the Speed of the fan.
}



double getDistance() {
  // Get Distance of ball every 10ms.
  if (currentMillis - startMillis >= period) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    distance = (duration * .0343) / 2;
    startMillis = millis();
  }
  // Limit the distance.
  if (distance > 29) {
    distance = 3;
  }
  return distance;
}


void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num); // If app disconnects.
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num); // If app connects.
        Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
      }
      break;
    case WStype_TEXT:
      {
        // Recieve and parse data from application.
        String echoMessage = String((char*)payload);
        Setpoint = map(getValue(echoMessage, ',', 0).toInt(), 0, 100, 26, 3);
        P = getValue(echoMessage, ',', 1).toDouble();
        I = getValue(echoMessage, ',', 2).toDouble();
        D = getValue(echoMessage, ',', 3).toDouble();
        myPID.SetTunings(P, I, D);
      }
      break;
  }
}

void notifyClients() {
  // Send data back to the application every 50ms.
  if (currentMillis - startMillis2 >= period2) {
    // Put data into JSON format.
    char buffer[70];
    sprintf(buffer, "{\"Output\":\"%.2f\",\"Input\":\"%.2f\",\"Setpoint\":\"%.2f\",\"P\":\"%.2f\",\"I\":\"%.2f\",\"D\":\"%.2f\"}", Output, Input, Setpoint, P, I, D);
    webSocket.broadcastTXT(buffer);
    startMillis2 = millis();
  }
}

String getValue(String data, char separator, int index) {
  // Split a string literal.
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}