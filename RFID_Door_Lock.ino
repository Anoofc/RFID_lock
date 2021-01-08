/***************************************************
     IOT Based HOME AUTOMATION AND SECURITY SYSTEM
                  #TEAM PI BOTS
               FOLLOW AND SUPPORT
                    @pi_bots

         CONTACT FOR TECHNICAL ASSISTANCE
                 Instagram.com
           -------------------------
            >>  @at_mega_328p
            
 ****************************************************/

#define BLYNK_PRINT Serial

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Servo.h>
#include <DHT.h>
#include <U8g2lib.h>


#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);


#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
BlynkTimer timer;

char auth[] = "P41b3Qgm-YaWZGNnsDzBOb-kFsCvNPw5";

WidgetLED warn(V2);
WidgetLED accessgrant(V1);

#define RELAYPIN D5
#define WARNLEDPIN D6

char tag[] = "2B006DA922CD";
char tag2[] = "2B006D8606C6";// Replace with your own Tag ID
char input[12];        // A variable to store the Tag ID being presented
int count = 0;        // A counter variable to navigate through the input[] character array
boolean flag = 0;     // A variable to store the Tag match status

int state = 0;
int pir = 16;
int valpir = 0;
int servopin = 0;

Servo servo;

/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "Pi Home"
#define WLAN_PASS       "Pihome@7560"

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "eliote"
#define AIO_KEY         "aio_HdSi39PC7W45av0H0CQGPdhHzylA"


/************ Global State (you don't need to change this!) ******************/

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

// Setup a feed called 'sensor' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish photocell = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/motion");

/*************************** Sketch Code ************************************/
void dhtsend()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature(); // or dht.readTemperature(true) for Fahrenheit

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V5, h);
  Blynk.virtualWrite(V6, t);
  
}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}

void MQTT_connect();

void warnc() {
  Serial.println("Access Denied"); // Incorrect Tag Message
  Blynk.notify("Wrong Access...!!");

  digitalWrite(WARNLEDPIN, HIGH);
  warn.on();
  delay(100);
  digitalWrite(WARNLEDPIN, LOW);
  warn.off();
  delay(100);
  digitalWrite(WARNLEDPIN, HIGH);
  warn.on();
  delay(100);
  digitalWrite(WARNLEDPIN, LOW);
  warn.off();
  delay(100);
  digitalWrite(WARNLEDPIN, HIGH);
  warn.on();
  delay(100);
  digitalWrite(WARNLEDPIN, LOW);
  warn.off();
  delay(100);
  digitalWrite(WARNLEDPIN, HIGH);
  warn.on();
  delay(100);
  digitalWrite(WARNLEDPIN, LOW);
  warn.off();
}

void notifysms(){
  if (state == 0) {
  valpir = digitalRead(pir);
  if (valpir == LOW) {
    photocell.publish("1");
    Blynk.notify("Unusual Activity Detected....");
    delay(1000);
  }
  else {
    photocell.publish("0");
    delay(1000);
  }
}

  }
void setup() {

  u8g2.begin();
  pinMode(14, OUTPUT);
  pinMode(12, OUTPUT);
  Blynk.begin(auth, WLAN_SSID, WLAN_PASS);
  dht.begin();
  timer.setInterval(1000L, dhtsend);
  servo.attach(servopin);
  servo.write(0);

  Serial.begin(9600);   // Initialise Serial Communication with the Serial Monitor
  pinMode(RELAYPIN, OUTPUT); // RELAY OUTPUT
  pinMode(WARNLEDPIN, OUTPUT); //WRONG TAG INDICATOR
  pinMode(pir, INPUT);

  Serial.println(F("Pi HOME SECURITY v.1.5"));

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());
}

uint32_t x = 0;

void loop() {

  displayt();
  Blynk.run();
  notifysms();
  timer.run();
  MQTT_connect();

  if (Serial.available()) // Check if there is incoming data in the RFID Reader Serial Buffer.
  {
    count = 0; // Reset the counter to zero
    /* Keep reading Byte by Byte from the Buffer till the RFID Reader Buffer is empty
       or till 12 Bytes (the ID size of our Tag) is read */
    while (Serial.available() && count < 12)
    {
      input[count] = Serial.read(); // Read 1 Byte of data and store it in the input[] variable
      count++; // increment counter
      delay(5);
    }
    /* When the counter reaches 12 (the size of the ID) we stop and compare each value
        of the input[] to the corresponding stored value */
    if (count == 12) //
    {
      count = 0; // reset counter varibale to 0
      flag = 1;
      /* Iterate through each value and compare till either the 12 values are
         all matching or till the first mistmatch occurs */
      while (count < 12 && flag != 0)
      {
        if (input[count] == tag[count])
          flag = 1; // everytime the values match, we set the flag variable to 1
        else
          flag = 0;
        /* if the ID values don't match, set flag variable to 0 and
           stop comparing by exiting the while loop */
        count++; // increment i
      }
    }
    if (flag == 1) // If flag variable is 1, then it means the tags match
    {
      if (state == 0) {
        state = 1;
      }
      else if (state == 1) {

        state = 0;
      }
      Serial.println("Access Allowed!");
      if (state == 1) {
        accessunlock();
      }
      else if (state == 0) {
        accesslock();

      }
    }
    else {
      warnc();
      }
  }
}

//WidgetLED accessled(V1);

void accesslock() {
  digitalWrite(RELAYPIN, LOW);
  servo.write(0);
  accessgrant.off();
}

void accessunlock() {
  digitalWrite(RELAYPIN, HIGH);
  servo.write(90);
  accessgrant.on();
}

void displayt(){
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  u8g2.clearBuffer();                       // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr);       // choose a suitable font
  u8g2.drawStr(0,10,"Temperature: " );      // write something to the internal memory
  u8g2.setCursor(90,10); u8g2.print(t,2);
  u8g2.drawStr(0,30,"Humidity: ");
  u8g2.setCursor(90,30); u8g2.print(h,2);
  u8g2.sendBuffer();                        // transfer internal memory to the display
  
}
