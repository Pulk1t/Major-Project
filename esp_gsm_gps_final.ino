#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <TinyGPS++.h>

#define WLAN_SSID          "admin"
#define WLAN_PASS        "1234567890"
#define AIO_UPDATE_RATE_SEC 5
#define AIO_SERVER        "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME "manav101"
#define AIO_KEY                "aio_UAGq34QCKg17jLmCTiQmnwE0IHGl"

#define rxPin 4
#define txPin 2
HardwareSerial sim800(1);
#define RXD2 16
#define TXD2 17
HardwareSerial neogps(2);
TinyGPSPlus gps;
float latitude , longitude;

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Subscribe lamp1 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/switch");
Adafruit_MQTT_Publish LAT = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/lat");
Adafruit_MQTT_Publish LNG = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/lng");
Adafruit_MQTT_Publish CRASH = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/crash");
Adafruit_MQTT_Publish FIRE = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/fire");
Adafruit_MQTT_Publish COUSTOM = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/cus");
Adafruit_MQTT_Publish gas = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/gas");
Adafruit_MQTT_Publish com = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/com");
Adafruit_MQTT_Publish SPEED = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/speed");


void MQTT_connect() {
  int8_t ret;
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);
    retries--;
    if (retries == 0) {
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}
int mq = 23;
void setup() {
  Serial.begin(115200);
  Serial.println("esp32 serial initialize");
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());
  mqtt.subscribe(&lamp1);

  sim800.begin(9600, SERIAL_8N1, rxPin, txPin);
  Serial.println("SIM800L serial initialize");

  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("neogps serial initialize");
  pinMode(mq, INPUT);

}

int piezo, ldr;
int a = 0, b = 0;
void loop()
{
  MQTT_connect();
  Adafruit_MQTT_Subscribe *subscription;
  while (subscription = mqtt.readSubscription(1000)) {
    if (subscription == &lamp1) {

      char *value = (char *)lamp1.lastread;
      Serial.print(F("Received: "));
      Serial.println(value);

      String message = String(value);
      message.trim();
      if (message == "ON") {
        Location();
      }
    }
  }
  while (neogps.available() > 0)
    if (gps.encode(neogps.read()))
    {
      if (gps.location.isValid())
      {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
      }
    }
  analogReadResolution(10);
  piezo = analogRead(34);
  ldr = analogRead(35);

  if (gps.speed.kmph() > 40)
  {
    Speed();
    Location();
    SendSMS_1();
  }
  if (digitalRead(mq) == LOW)
  {
    gass();
    Location();
    SendSMS();
    delay(3000);
  }

  if (ldr < 150)
  {
    firee();
    Location();
    SendSMS();
    delay(3000);
  }

  if (piezo > 1000)
  {
    Crash();
    Location();
    SendSMS();
    delay(3000);
  }
}

void firee()
{
  if (! FIRE.publish("FIRE ALERT")) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
  delay(50);
}

void gass()
{
  if (! gas.publish("SMOKE ALERT.....")) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
}

void Crash()
{
  if (! CRASH.publish("ACCIDENT ALERT.....")) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
  delay(50);
}

void Speed()
{
  if (! SPEED.publish("OVER SPEED ALERT.....")) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
  delay(50);
}

void Location()
{
  if (! LAT.publish(latitude, 6)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
  delay(10);
  if (! com.publish(",")) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
  delay(10);
  if (! LNG.publish(longitude, 6)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
  delay(50);
}

void SendSMS()
{
  sim800.print("AT+CMGF=1\r");                   //Set the module to SMS mode
  delay(100);
  sim800.print("AT+CMGS=\"+919131708395\"\r");  //Your phone number don't forget to include your country code, example +212123456789"
  delay(500);
  sim800.print("http://maps.google.com/maps?q=loc:");
  sim800.print(gps.location.lat(), 6);
  sim800.print(",");
  sim800.print(gps.location.lng(), 6);
  sim800.print("ACCIDENT ALERT...");
  delay(1000);
  sim800.write(0x1A);
  Serial.println("Text Sent.");
  delay(500);

}

void SendSMS_1()
{
  sim800.print("AT+CMGF=1\r");                   //Set the module to SMS mode
  delay(100);
  sim800.print("AT+CMGS=\"+919131708395\"\r");  //Your phone number don't forget to include your country code, example +212123456789"
  delay(500);
  sim800.print("http://maps.google.com/maps?q=loc:");
  sim800.print(gps.location.lat(), 6);
  sim800.print(",");
  sim800.print(gps.location.lng(), 6);
  sim800.print("OVER SPEED ALERT...");
  delay(1000);
  sim800.write(0x1A);
  Serial.println("Text Sent.");
  delay(500);

}
