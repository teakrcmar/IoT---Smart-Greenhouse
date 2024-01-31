#include <DHT.h>
#include <DHT_U.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Servo.h>

#define WIFI_SSID "bijeli11"
#define WIFI_PASSWORD "ferit.2023"
#define MQTT_SERVER "192.168.23.82"
#define MQTT_PORT 1883
#define CLIENT_ID "client_id"

WiFiClient espClient;
PubSubClient client(espClient);

#define TEMP_TOPIC "data/temperature"
#define HUMIDITY_TOPIC "data/humidity"
#define LIGHT_TOPIC "data/light"
#define SET_TEMP_TOPIC "setting/temperature"
#define SET_HUMIDITY_TOPIC "setting/humidity"
#define SET_LIGHT_TOPIC "setting/light"
#define VENTILATOR_TOPIC "status/ventilator"
#define WINDOW_TOPIC "status/window"
#define BLINDS_TOPIC "status/blinds"
#define FURNACE_TOPIC "status/furnace"

// Define the pin for the DHT22 sensor
#define DHT_PIN 14
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);

// Define pins for servo motors, one for blinds and one for window
#define SERVO_BLINDS_PIN 12
#define SERVO_WINDOW_PIN 13
Servo servoBlinds;
Servo servoWindow;

// Define pin for DC motor (ventilation) and LED for furnace and alarm
#define DC_MOTOR_PIN 4
#define LED_PIN 16
#define LED_ALARM 5

// Define pin for the photoresistor
#define PHOTOSENSOR_PIN A0
#define LIGHT_THRESHOLD_DAY 800
#define LIGHT_THRESHOLD_NIGHT 200

String ventilatorControlValue;
String windowControlValue;
String blindsControlValue;
String furnaceControlValue;
String temperatureControlValue;
String lightControlValue;
String humidityControlValue;

// Define temperature and humidity thresholds
#define TEMP_THRESHOLD_HIGH 24.0
#define TEMP_THRESHOLD_LOW 18.0
#define HUMIDITY_THRESHOLD_HIGH_NIGHT 75
#define HUMIDITY_THRESHOLD_LOW_NIGHT 65
#define HUMIDITY_THRESHOLD_HIGH_DAY 90
#define HUMIDITY_THRESHOLD_LOW_DAY 80

unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 60000; // Publish every 60 seconds

float targetTemperature = 0.0;
float targetHumidity = 0.0;
int targetLight = 0.0;
float lastPublishedTemperature = 0.0;
float lastPublishedHumidity = 0.0;
int lastPublishedLight = 0;

void setup() {
  Serial.begin(115200);

  // Attach servo motors to their respective pins
  servoBlinds.attach(SERVO_BLINDS_PIN);
  servoWindow.attach(SERVO_WINDOW_PIN);
  servoBlinds.write(0);
  servoWindow.write(0);

  // Set DC motor and LED pins as OUTPUT
  pinMode(DC_MOTOR_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_ALARM, OUTPUT);

  // Set the photosensor pin as INPUT
  pinMode(PHOTOSENSOR_PIN, INPUT);

  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");

  // Connect to MQTT broker
  client.setServer(MQTT_SERVER, MQTT_PORT);
  while (!client.connected()) {
    if (client.connect(CLIENT_ID)) {
      Serial.println("Connected to MQTT broker");
      onConnectionEstablished();
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying...");
      delay(500);
    }
  }
  client.setCallback(callback);
}

void onConnectionEstablished() {
  Serial.println("MQTT Connection Established!");
  dht.begin();

  // Get temperature, humidity, and light readings
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  float light = readLightIntensity();

  // Publish initial data to MQTT topics
  client.publish(TEMP_TOPIC, String(temperature).c_str());
  client.publish(HUMIDITY_TOPIC, String(humidity).c_str());
  client.publish(LIGHT_TOPIC, String(light).c_str());

  // Subscribe to relevant topics
  client.subscribe(SET_TEMP_TOPIC);
  client.subscribe(SET_HUMIDITY_TOPIC);
  client.subscribe(SET_LIGHT_TOPIC);
  client.subscribe(VENTILATOR_TOPIC);
  client.subscribe(WINDOW_TOPIC);
  client.subscribe(BLINDS_TOPIC);
  client.subscribe(FURNACE_TOPIC);
}

void callback(char* topic, byte* payload, unsigned int length) {
  String topicStr = String(topic);
  String payloadStr = "";

  // Iterate through each byte in the payload array
  for (unsigned int i = 0; i < length; i++) {
    payloadStr += (char)payload[i];
  }

  // Now payloadStr contains the payload as a String
  Serial.println("Received message:");
  Serial.println("Topic: " + topicStr);
  Serial.println("Payload: " + payloadStr);

  if (topicStr.equals(SET_TEMP_TOPIC)) {
     temperatureControlValue = payloadStr;
     float temperatureValue = temperatureControlValue.toFloat();
     setTemperatureManually(temperatureValue);
  } else if (topicStr.equals(SET_HUMIDITY_TOPIC)) {
     humidityControlValue = payloadStr;
     float humidityValue = humidityControlValue.toFloat();
     setHumidityManually(humidityValue);
  } else if (topicStr.equals(SET_LIGHT_TOPIC)) {
     lightControlValue = payloadStr;
     float lightValue = lightControlValue.toInt();
     setLightManually(lightValue);
  } else if (topicStr.equals(VENTILATOR_TOPIC)) {
    ventilatorControlValue = payloadStr;
    controlVentilatorManually(ventilatorControlValue);
  } else if (topicStr.equals(WINDOW_TOPIC)) {
    windowControlValue = payloadStr;
    int windowPosition = windowControlValue.toInt();
    controlWindowManually(windowPosition);
  } else if (topicStr.equals(BLINDS_TOPIC)) {
    blindsControlValue = payloadStr;
    int blindsPosition = blindsControlValue.toInt();
    controlBlindsManually(blindsPosition);
  } else if (topicStr.equals(FURNACE_TOPIC)) {
    furnaceControlValue = payloadStr;
    controlFurnaceManually(furnaceControlValue);
  } else {
    Serial.println("Unknown topic received");
  }
}

unsigned long lastReadTime = 0;
const unsigned long sensorReadInterval = 5000;

void loop() {
  client.loop();
  unsigned long currentMillis = millis();

  // Read sensor data every 5 seconds (adjust the delay as needed)
  if (currentMillis - lastReadTime >= sensorReadInterval) {
    // Save the last time a sensor reading occurred
    lastReadTime = currentMillis;

    // Read sensor data
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    int light = readLightIntensity();

    // Check for a significant change in temperature
    if (abs(temperature - lastPublishedTemperature) >= 1.0) {
      lastPublishedTemperature = temperature;
      client.publish(TEMP_TOPIC, String(temperature).c_str());
      Serial.println("Temperature changed");
    }

    // Check for a significant change in humidity
    if (abs(humidity - lastPublishedHumidity) >= 10.0) {
      lastPublishedHumidity = humidity;
      client.publish(HUMIDITY_TOPIC, String(humidity).c_str());
      Serial.println("Humidity changed");
    }

    // Check for a significant change in light
    if (abs(light - lastPublishedLight) >= 10.0) {
      lastPublishedLight = light;
      client.publish(LIGHT_TOPIC, String(light).c_str());
      Serial.println("Light changed");
    }
  }

  // Publish sensor data to MQTT topics every 60 seconds (adjust the interval as needed)
  if (currentMillis - lastPublishTime >= publishInterval) {
    // Save the last time a publish occurred
    lastPublishTime = currentMillis;

    // Publish sensor data to MQTT topics
    client.publish(TEMP_TOPIC, String(lastPublishedTemperature).c_str());
    client.publish(HUMIDITY_TOPIC, String(lastPublishedHumidity).c_str());
    client.publish(LIGHT_TOPIC, String(lastPublishedLight).c_str());
    Serial.println("Published sensor data");
  }
}


int readLightIntensity() {
  int lightValue = analogRead(PHOTOSENSOR_PIN);
  int light = map(lightValue, 0, 1023, 0, 100);
  return light;
}

void setTemperatureManually(float temperatureControlValue) {
  targetTemperature = temperatureControlValue;
  Serial.print("Temperature set to: ");
  Serial.println(targetTemperature);
  if(targetTemperature < dht.readTemperature()){
     digitalWrite(LED_ALARM, HIGH);
     Serial.print("Actual emperature in the greenhouse is higher than desired.");
   }else {
     digitalWrite(LED_ALARM, LOW);
     Serial.print("Actual emperature in the greenhouse is lower than desired.");
 }
}

void setHumidityManually(float humidityControlValue) {
  targetHumidity = humidityControlValue;
  Serial.print("Humidity set to: ");
  Serial.println(targetHumidity);
  if(targetHumidity < dht.readHumidity()){
     digitalWrite(LED_ALARM, HIGH);
     Serial.print("Actual humidity in the greenhouse is higher than desired.");
  }
  else {
    digitalWrite(LED_ALARM, HIGH);
    Serial.print("Actual humidity in the greenhouse is lower than desired.");
  }
}

 void setLightManually(int lightControlValue) {
  targetLight = lightControlValue;
  Serial.print("Light set to: ");
  Serial.println(targetLight);
  if(targetLight < readLightIntensity()){
    digitalWrite(LED_ALARM, HIGH);
    Serial.print("Actual light in the greenhouse is higher than desired.");
  }
  else {
    digitalWrite(LED_ALARM, LOW);
    Serial.print("Actual light in the greenhouse is lower than desired.");
  }
 }

void controlVentilatorManually(String ventilatorControlValue) {
  if (ventilatorControlValue == "ON") {
    digitalWrite(DC_MOTOR_PIN, HIGH);
    Serial.println("Ventilator turned ON.");
  } else if (ventilatorControlValue == "OFF") {
    digitalWrite(DC_MOTOR_PIN, LOW);
    Serial.println("Ventilator turned OFF.");
  } else {
    Serial.println("Invalid ventilator control value received.");
  }
}

void controlFurnaceManually(String furnaceControlValue) {
  if (furnaceControlValue == "ON") {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("on");
  } else if (furnaceControlValue == "OFF") {
    digitalWrite(LED_PIN, LOW);
    Serial.println("off");
  }else {
    Serial.println("Invalid furnace control value received.");
    }
}

void controlBlindsManually(int blindsControlValue) {
  int mappedValue = map(blindsControlValue, 0, 100, 0, 180);
  servoBlinds.write(mappedValue);
  Serial.print("Blinds set to: ");
  Serial.println(mappedValue);
}

void controlWindowManually(int windowControlValue) {
  int mappedValue = map(windowControlValue, 0, 100, 0, 180);
  servoWindow.write(mappedValue);
  Serial.print("Window set to: ");
  Serial.println(mappedValue);
}