//UNIVERSIDAD POLITÉCNICA SALESIANA - SEDE GUAYAQUIL
//INGENIERÍA EN ELECTRÓNICA
//PROTOTIPO DE TELEMEDICINA - IOT
//AUTORES: MIGUEL ORTIZ - CRISTIAN MACIAS

// EKG - UDIBOTS

//LIBRERIAS
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <NTPClient.h>

//DEFINIR VARIABLES
#define WIFISSID "CLARO_MQUINDE" // Enter WifiSSID here
#define PASSWORD "0931017271" // Enter password here
#define TOKEN "BBFF-LvNOJ4z5bmM2UESky89WDoqT5OmLCr" // Ubidots' TOKEN

#define MQTT_CLIENT_NAME "mymqttclient" // MQTT client Name
// * Define Constants
#define VARIABLE_LABEL "ekg_olimex" // ubidots variable label
#define DEVICE_LABEL "ekg_olimex" // ubidots device label

const int analogInPin = 39; // Output Olimex is connected to GPIO35

char mqttBroker[]  = "industrial.api.ubidots.com";
char payload[10000];
char topic[150];

// Space to store values to send
char str_sensor[10];
char str_millis[20];
double epochseconds = 0;
double epochmilliseconds = 0;
double current_millis = 0;
double current_millis_at_sensordata = 0;
double timestampp = 0;
int j = 0;

WiFiClient ubidots;
PubSubClient client(ubidots);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

/****************************************
   Auxiliar Functions
 ****************************************/
void callback(char* topic, byte* payload, unsigned int length) {
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  Serial.write(payload, length);
  Serial.println(topic);
}
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    // Attemp to connect
    if (client.connect(MQTT_CLIENT_NAME, TOKEN, "")) {
      Serial.println("Connected");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      // Wait 2 seconds before retrying
      delay(2000);
    }
  }
}

/****************************************
   Main Functions
 ****************************************/
void setup() {
  Serial.begin(115200);
  analogReadResolution(10);
  WiFi.begin(WIFISSID, PASSWORD);
  Serial.println();
  Serial.print("Waiting for WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("");
  Serial.println("WiFi Connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  timeClient.begin();
  client.setServer(mqttBroker, 1883);
  client.setCallback(callback);
  timeClient.update();
  epochseconds = timeClient.getEpochTime();
  epochmilliseconds = epochseconds * 1000;
  Serial.print("epochmilliseconds=");
  Serial.println(epochmilliseconds);
  current_millis = millis();
  Serial.print("current_millis=");
  Serial.println(current_millis);
}

void loop() {
  if (!client.connected()) {
    reconnect();
    j = 0;
  }

  j = j + 1;
  // Serial.print("j=");
  // Serial.println(j);
  sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
  sprintf(payload, "%s", ""); // Cleans the payload
  sprintf(payload, "{\"%s\": [", VARIABLE_LABEL); // Adds the variable label
  float sensor = analogRead(analogInPin);
  dtostrf(float(sensor), 4, 2, str_sensor);
  Serial.print("Value: ");
  Serial.println(sensor);
  current_millis_at_sensordata = millis();
  timestampp = epochmilliseconds + (current_millis_at_sensordata - current_millis);
  dtostrf(timestampp, 10, 0, str_millis);
  sprintf(payload, "%s{\"value\":%s, \"timestamp\": %s}]}", payload, str_sensor, str_millis);
  // Serial.println("Publishing data to Ubidots Cloud");
  // Serial.println(payload);
  client.publish(topic, payload);
  delay(10);
}
