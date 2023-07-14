/*
  Rui Santos
  Complete project details at our blog.
    - ESP32: https://RandomNerdTutorials.com/esp32-firebase-realtime-database/
    - ESP8266: https://RandomNerdTutorials.com/esp8266-nodemcu-firebase-realtime-database/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
  Based in the RTDB Basic Example by Firebase-ESP-Client library by mobizt
  https://github.com/mobizt/Firebase-ESP-Client/blob/main/examples/RTDB/Basic/Basic.ino
*/
#include <Arduino.h>
#if defined(ESP32)
#include <WiFi.h>
#define USE_SPIFFS true
#define ESP_DRD_USE_EEPROM true
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif

#include <Firebase_ESP_Client.h>

// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <Wire.h>
#include "soc/soc.h"
#include "soc/rtc.h"
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <DNSServer.h>
#include <ESP_WiFiManager.h>
#include <WiFiManager.h>
#include <ESP_DoubleResetDetector.h>
// Insert your network credentials
// #define WIFI_SSID "ACE"
// #define WIFI_PASSWORD "diaaxxxx4"
DoubleResetDetector *drd;
const int PIN_LED = 2;
bool initialConfig = false;
// Insert Firebase project API Key
#define API_KEY "AIzaSyCOnv0vzxE_5I9BzxPWZMou5Prm8PqFUlQ"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://homeautomation-9d333-default-rtdb.firebaseio.com/"

#define DRD_TIMEOUT 10
#define DRD_ADDRESS 0
// Temperature Sensor
#define DHTPIN 23               // Digital pin connected to the DHT sensor
#define TEMP_UPPER_THRESHOLD 30 // upper temperature threshold
#define TEMP_LOWER_THRESHOLD 25 // lower temperature threshold
#define RELAY_FAN_PIN 26
// const int output4 = 26;
// const int output1 = 27;

// Gas Sensor
const int Sensor = 35;
const int LED = 14;

const int out12 = 12;
const int out14 = 14;
// const int out19 = 19;
const int out18 = 18;
const int out5 = 5;
const int out17 = 17;
const int out16 = 16;
const int out4 = 4;
const int out0 = 0;
const int out2 = 2;   
const int out15 = 15; // Indoor light
const int out25 = 25;
int checktemp = 0;
int checklight = 0;
int checkgas = 0;
// const int out26 = 26;

// PIR SENSOR
int ledPin = 19;    // choose the pin for the LED
int inputPin = 13;  // choose the input pin (for PIR sensor)
int pirState = LOW; // we start, assuming no motion detected
int val = 0;        // variable for reading the pin status

// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment whatever type you're using!
// #define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

// Define Firebase objects
FirebaseData stream;
FirebaseAuth auth;
FirebaseConfig config;
// LiquidCrystal_I2C lcd;
FirebaseData fbdo;
// Variables to save database paths
String listenerPath = "board1/outputs/digital/";

String tempPath = "/temperature";
String humPath = "/humidity";
String fahPath = "/fahrenheit";
String gasPath = "/gas";
String timePath = "/timestamp";
String readings = "/readings";

FirebaseJson json;
// Variable to save current epoch time
int timestamp;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

// Timer variables (send new readings every three minutes)
unsigned long sendDataPrevMillis = 0;
unsigned long timerDelay = 10000;

LiquidCrystal_I2C lcd(0x27, 16, 2);

// Initialize WiFi
void initWiFi()
{
  WiFiManager wm;

  // reset settings - wipe stored credentials for testing
  // these are stored by the esp library
  wm.resetSettings();

  // Automatically connect using saved credentials,
  // if connection fails, it starts an access point with the specified name ( "AutoConnectAP"),
  // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
  // then goes into a blocking loop awaiting configuration and will return success result

  bool res;
  // res = wm.autoConnect(); // auto generated AP name from chipid
  res = wm.autoConnect("AutoConnectAP"); // anonymous ap
  // res = wm.autoConnect("AutoConnectAP","password"); // password protected ap

  if (!res)
  {
    Serial.println("Failed to connect");
    // ESP.restart();
  }
  else
  {
    // if you get here you have connected to the WiFi
    Serial.println("connected...yeey :)");
  }
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
  Serial.println();
}

// Callback function that runs on database changes
void streamCallback(FirebaseStream data)
{
  Serial.printf("stream path, %s\nevent path, %s\ndata type, %s\nevent type, %s\n\n",
                data.streamPath().c_str(),
                data.dataPath().c_str(),
                data.dataType().c_str(),
                data.eventType().c_str());
  printResult(data); // see addons/RTDBHelper.h
  Serial.println();

  // Get the path that triggered the function
  String streamPath = String(data.dataPath());

  // if the data returned is an integer, there was a change on the GPIO state on the following path /{gpio_number}
  if (data.dataTypeEnum() == fb_esp_rtdb_data_type_integer)
  {
    String gpio = streamPath.substring(1);

    int state = data.intData();
    Serial.print("GPIO: ");
    Serial.println(gpio);
    Serial.print("STATE: ");
    Serial.println(state);
    digitalWrite(gpio.toInt(), state);
    if (gpio.toInt() == 50 && state == 1)
    {
      checktemp = 1;
    }
    else if (gpio.toInt() == 50 && state == 0)
    {
      checktemp = 0;
    }
    if (gpio.toInt() == 51 && state == 1)
    {
      checklight = 1;
    }
    else if (gpio.toInt() == 51 && state == 0)
    {
      checklight = 0;
    }
    if (gpio.toInt() == 52 && state == 1)
    {
      checkgas = 1;
    }
    else if (gpio.toInt() == 52 && state == 0)
    {
      checkgas = 0;
    }
  }

  /* When it first runs, it is triggered on the root (/) path and returns a JSON with all keys
  and values of that path. So, we can get all values from the database and updated the GPIO states*/
  if (data.dataTypeEnum() == fb_esp_rtdb_data_type_json)
  {
    FirebaseJson json = data.to<FirebaseJson>();

    // To iterate all values in Json object
    size_t count = json.iteratorBegin();
    Serial.println("\n---------");
    for (size_t i = 0; i < count; i++)
    {
      FirebaseJson::IteratorValue value = json.valueAt(i);
      int gpio = value.key.toInt();
      int state = value.value.toInt();
      Serial.print("STATE: ");
      Serial.println(state);
      Serial.print("GPIO:");
      Serial.println(gpio);
      digitalWrite(gpio, state);
      Serial.printf("Name: %s, Value: %s, Type: %s\n", value.key.c_str(), value.value.c_str(), value.type == FirebaseJson::JSON_OBJECT ? "object" : "array");

      if (gpio == 50 && state == 1)
      {
        checktemp = 1;
      }
      else if (gpio == 50 && state == 0)
      {
        checktemp = 0;
      }
      if (gpio == 51 && state == 1)
      {
        checklight = 1;
      }
      else if (gpio == 51 && state == 0)
      {
        checklight = 0;
      }
      if (gpio == 52 && state == 1)
      {
        checkgas = 1;
      }
      else if (gpio == 52 && state == 0)
      {
        checkgas = 0;
      }
    }
    Serial.println();
    json.iteratorEnd(); // required for free the used memory in iteration (node data collection)
  }

  // This is the size of stream payload received (current and max value)
  // Max payload size is the payload size under the stream path since the stream connected
  // and read once and will not update until stream reconnection takes place.
  // This max value will be zero as no payload received in case of ESP8266 which
  // BearSSL reserved Rx buffer size is less than the actual stream payload.
  Serial.printf("Received stream payload size: %d (Max. %d)\n\n", data.payloadLength(), data.maxPayloadLength());
}

void streamTimeoutCallback(bool timeout)
{
  if (timeout)
    Serial.println("stream timeout, resuming...\n");
  if (!stream.httpConnected())
    Serial.printf("error code: %d, reason: %s\n\n", stream.httpCode(), stream.errorReason().c_str());
}

// Function that gets current epoch time
unsigned long getTime()
{
  timeClient.update();
  unsigned long now = timeClient.getEpochTime();
  return now;
}

void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  WiFi.mode(WIFI_STA);
  Serial.begin(115200);
  initWiFi();
  unsigned long startedAt = millis();
  digitalWrite(PIN_LED, LOW);
  Serial.print(F("After waiting "));
  int connRes = WiFi.waitForConnectResult();
  float waited = (millis() - startedAt);
  Serial.print(waited / 1000);
  Serial.print(F(" secs , Connection result is "));
  Serial.println(connRes);
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println(F("Failed to connect"));
  }
  else
  {
    Serial.print(F("Local IP: "));
    Serial.println(WiFi.localIP());
  }
  // lcd.backlight();
  // lcd.begin();
  // dht_sensor.begin();
  // Initialize Outputs

  // pinMode(Green, OUTPUT);
  // pinMode(Red, OUTPUT);
  // pinMode(Buzzer, OUTPUT);
  pinMode(Sensor, INPUT);
  pinMode(LED, OUTPUT);

  dht.begin();
  // Assign the api key (required)
  config.api_key = API_KEY;

  // Assign the user sign in credentials
  auth.user.email = "mina@mina.com";
  auth.user.password = "minamina";

  // Assign the RTDB URL (required)
  config.database_url = DATABASE_URL;

  Firebase.reconnectWiFi(true);

  // Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h

  // Assign the maximum retry of token generation
  config.max_token_generation_retry = 5;

  // Initialize the library with the Firebase authen and config
  Firebase.begin(&config, &auth);

  // Streaming (whenever data changes on a path)
  // Begin stream on a database path --> board1/outputs/digital
  if (!Firebase.RTDB.beginStream(&stream, listenerPath.c_str()))
    Serial.printf("stream begin error, %s\n\n", stream.errorReason().c_str());

  // Assign a calback function to run when it detects changes on the database
  Firebase.RTDB.setStreamCallback(&stream, streamCallback, streamTimeoutCallback);

  delay(2000);

  int sensor = analogRead(Sensor);
  Serial.println(sensor);
  sensor = map(sensor, 0, 1024, 0, 100);

  pinMode(ledPin, OUTPUT);  // declare LED as output
  pinMode(inputPin, INPUT); // declare sensor as input
  pinMode(out12, OUTPUT);
  pinMode(out14, OUTPUT);
  // pinMode(out17, OUTPUT);
  // pinMode(out19, OUTPUT);
  pinMode(out15, OUTPUT);
  pinMode(out18, OUTPUT);
  pinMode(out5, OUTPUT);
  pinMode(out4, OUTPUT);
  pinMode(out2, OUTPUT);
  pinMode(out25, OUTPUT);
  pinMode(RELAY_FAN_PIN, OUTPUT);
  // pinMode(out26, OUTPUT);
  // pinMode(out16, OUTPUT);

  pinMode(out0, OUTPUT);
}

void loop()
{

  digitalWrite(out0, LOW);

  int sensor_Aout = analogRead(Sensor);
  Serial.print("Gas Sensor: ");
  Serial.print(sensor_Aout); /*Read value printed*/
  Serial.print("\t");
  if (checkgas == 1)
  {
    if (sensor_Aout > 700)
    { /*if condition with threshold 1800*/
      Serial.println("Gas");
      digitalWrite(LED, HIGH);
      digitalWrite(out5, HIGH);  /*LED set HIGH if Gas detected */
      digitalWrite(out12, HIGH); /*LED set HIGH if Gas detected */
    }
    else
    {
      Serial.println("No Gas");
      digitalWrite(LED, LOW);
      digitalWrite(out5, LOW);  /*LED set HIGH if Gas detected */
      digitalWrite(out12, LOW); /*LED set LOW if NO Gas detected */
    }
  }
  delay(1000); /*DELAY of 1 sec*/

  //////////////////////////////////////////////////////////////////////////////

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f))
  {
    Serial.println(F("Failed to read from DHT sensor!"));
    // return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(f);
  Serial.print(F("°F  Heat index: "));
  Serial.print(hic);
  Serial.print(F("°C "));
  Serial.print(hif);
  Serial.println(F("°F"));

  ///////////////////////////////////////////////////////////////

  float temperature = dht.readTemperature();
  ; // read temperature in Celsius
  Serial.println(checktemp);
  if (isnan(temperature))
  {
    Serial.println("Failed to read from DHT sensor!");
  }
  else if (checktemp == 1)
  {
    if (temperature > TEMP_UPPER_THRESHOLD)
    {
      Serial.println("Turn the fan on");
      digitalWrite(RELAY_FAN_PIN, HIGH); // turn on AC
      digitalWrite(out4, HIGH);
    }
    else if (temperature > TEMP_LOWER_THRESHOLD)
    {
      Serial.println("Turn the fan off");
      digitalWrite(out4, HIGH); // turn on Fan
    }
  }

  // wait a 2 seconds between readings
  delay(2000);

  val = digitalRead(inputPin); // read input value
  if (checklight == 1)
  {
    if (val == HIGH)
    {
      // check if the input is HIGH
      digitalWrite(ledPin, HIGH); // turn LED ON
      if (pirState == LOW)
      {
        // we have just turned on
        Serial.println("Motion detected!");
        // We only want to print on the output change, not state
        pirState = HIGH;
      }
    }
    else
    {
      digitalWrite(ledPin, LOW); // turn LED OFF
      if (pirState == HIGH)
      {
        // we have just turned of
        Serial.println("Motion ended!");
        // We only want to print on the output change, not state
        pirState = LOW;
      }
    }
  }

  // Send new readings to database
  if (Firebase.ready() && (millis() - sendDataPrevMillis > timerDelay || sendDataPrevMillis == 0))
  {
    sendDataPrevMillis = millis();

    // Get current timestamp
    timestamp = getTime();
    Serial.print("time: ");
    Serial.println(timestamp);

    json.set(tempPath.c_str(), String(t));
    json.set(humPath.c_str(), String(h));
    json.set(fahPath.c_str(), String(f));
    json.set(gasPath.c_str(), String(sensor_Aout));
    // json.set(humPath.c_str(), String(hif));
    // json.set(presPath.c_str(), String(bme.readPressure()/100.0F));
    json.set(timePath, String(timestamp));
    Serial.printf("Set json... %s\n", Firebase.RTDB.setJSON(&fbdo, readings.c_str(), &json) ? "ok" : stream.errorReason().c_str());
  }

  if (Firebase.isTokenExpired())
  {
    Serial.println("Token Exprired");
  }
}
