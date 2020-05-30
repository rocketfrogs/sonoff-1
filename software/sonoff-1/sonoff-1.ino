/////////////////////////////////////////////////////////////////////////
//////////////////////// Configuration //////////////////////////////////
/////////////////////////////////////////////////////////////////////////
#define DEVICE_NAME "sonoff-1"

#define REPORT_STATUS_PERIOD_MS 500  // set report refresh delay

// Uncomment to enable the features
#define ENABLE_OTA
#define ENABLE_SERIAL

/////////////////////////////////////////////////////////////////////////
//////////////////////// functions declarations /////////////////////////
/////////////////////////////////////////////////////////////////////////


/************************* WiFi Access Point *********************************/

#include <ESP8266WiFi.h>        // Include the Wi-Fi library
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#ifdef ENABLE_OTA
  #include <ArduinoOTA.h>
#endif
#include "credentials.h"        // Include Credentials (you need to create that file in the same folder if you cloned it from git)

/*
Content of "credentials.h" that matters for this section

// WIFI Credentials

#define WIFI_SSID        "[REPLACE BY YOUR WIFI SSID (2G)]"     // The SSID (name) of the Wi-Fi network you want to connect to
#define WIFI_PASSWORD    "[REPLACE BY YOUR WIFI PASSWORD]"      // The password of the Wi-Fi 
*/

const char* ssid     = WIFI_SSID;         // The SSID (name) of the Wi-Fi network you want to connect to
const char* password = WIFI_PASSWORD;     // The password of the Wi-Fi 

/************************* MQTT Setup *********************************/

#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "credentials.h"

/*
// MQTT Credentials

Content of "credentials.h" that matters for this section

#define AIO_SERVER      "[REPLACE BY YOUR MQTT SERVER IP ADDRESS OR ITS FQDN]"
#define AIO_SERVERPORT  [REPLACE BY THE PORT NUMBER USED FOR THE MQTT SERVICE ON YOUR MQTT SERVEUR (DEFAULT IS 1883)]       // use 8883 for SSL"
#define AIO_USERNAME    ""  // USE THIS IF YOU HAVE USERNAME AND PASSWORD ENABLED ON YOUR MQTT SERVER
#define AIO_KEY         ""  // USE THIS IF YOU HAVE USERNAME AND PASSWORD ENABLED ON YOUR MQTT SERVER
*/

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

// Setup a feeds for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish stat_power = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/" DEVICE_NAME "/stat/power"); // to change for each pistons
Adafruit_MQTT_Publish stat_button = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/" DEVICE_NAME "/stat/button"); // to change for each pistons

// Setup a feeds for subscribing to changes.
Adafruit_MQTT_Subscribe cmnd_power = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/" DEVICE_NAME "/cmnd/power"); // to change for each pistons
Adafruit_MQTT_Subscribe cmnd_led = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/" DEVICE_NAME "/cmnd/led"); // to change for each pistons


// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();


///////////////////// initialize variables  ///////////////////////////

// sonoff pins
#define RELAY_PIN 12
#define BUTTON_PIN 0
#define LED_PIN 13
#define BONUS_PIN 14


  int power_stat = 0;
  int power_cmnd = 0;
  int button_cmnd = 0;
  int button_stat = 0;
  int led_cmnd = 0;
  int led_stat = 0;


///////////////////////////////////////////////////////
//////  ///////////////////////////////////////////////
//////  ///////////////////////////////////////////////
//////  ///////////                         ///////////
//////  ///////////                         ///////////
//\        ////////          SETUP          ///////////
///\      /////////                         ///////////
////\    //////////                         ///////////
/////\  ///////////////////////////////////////////////
///////////////////////////////////////////////////////
void setup() {

///////////////////////////////////////////////////////
///////////////////// Start Serial ////////////////////
///////////////////////////////////////////////////////

#ifdef ENABLE_SERIAL
  Serial.begin(115200); // Start the Serial communication to send messages to the computer
  delay(10);
  Serial.println('\n');
#endif

///////////////////////////////////////////////////////
//////////////////// Start Wifi ///////////////////////
///////////////////////////////////////////////////////
  
  WiFi.begin(ssid, password);             // Connecting to the network
#ifdef ENABLE_SERIAL
  Serial.print("Connecting to ");
  Serial.print(ssid); Serial.println(" ...");
#endif

  int i = 0;
  while (WiFi.status() != WL_CONNECTED) { // Wait for the Wi-Fi to connect
    delay(1000);
#ifdef ENABLE_SERIAL
    Serial.print(++i); Serial.print(' ');
#endif
  }

#ifdef ENABLE_OTA
////////////////// Initialize OTA /////////////////////
  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(DEVICE_NAME);

#ifdef ENABLE_SERIAL
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
#endif /* ENABLE_SERIAL */
  ArduinoOTA.begin();
#endif /* ENABLE_OTA */

#ifdef ENABLE_SERIAL
  Serial.println('\n');
  Serial.println("Connection established!");  
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());         // Send the IP address of the ESP8266 to the computer
#endif

///////////////////////////////////////////////////////
////////////// Suscribing to MQTT topics //////////////
///////////////////////////////////////////////////////

  // Setup MQTT subscription feeds.
  mqtt.subscribe(&cmnd_power);
  mqtt.subscribe(&cmnd_led);

  ///////////////////////////////////////////////////////
  ////////////////// Initialize PINs ////////////////////
  ///////////////////////////////////////////////////////

  // INPUTS
  // (Info => https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/)
  pinMode(BONUS_PIN, INPUT);                                                            // there is a 10K ohm pull-DOWN connected to that pin 
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), button_pressed, RISING);    // there is a NAN gate between the switch and the PIN so the logic is inverted (Rising = Linit Switch triggered)
  /* 
  if (digitalRead(BUTTON_PIN) == HIGH) {
    // If the button is already pressed at start, we call the  triggered_button() interrupt fonction now as it will dot see a FALLING edge and therefore will not be able to initialize itself
    button_pressed();
  }
  */

  // OUTPUTS
  pinMode(RELAY_PIN, OUTPUT); 
  pinMode(LED_PIN, OUTPUT); 




}
///////////////////////////////////////////////////////
///  //////////////////////////////////////////////////
///  //////////////////////////////////////////////////
///  /// ///////////                        ///////////
///  ///   /////////                        ///////////
///          ///////      END OF SETUP      ///////////
////////   /////////                        ///////////
//////// ///////////                        ///////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////



///////////////////////////////////////////////////////
//////  ///////////////////////////////////////////////
//////  ///////////////////////////////////////////////
//////  ///////////                         ///////////
//////  ///////////                         ///////////
//\        ////////           LOOP          ///////////
///\      /////////                         ///////////
////\    //////////                         ///////////
/////\  ///////////////////////////////////////////////
///////////////////////////////////////////////////////

void loop() {
#ifdef ENABLE_OTA
  ArduinoOTA.handle();
#endif

  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();


        Adafruit_MQTT_Subscribe *subscription;
      if ((subscription = mqtt.readSubscription(0))) {
        float value = atof((char *)subscription->lastread);

        /// We check position target from MQTT
        if (subscription == &cmnd_power) {
          if (value >= 0 && value <= 1) {
            power_cmnd = value;
          } else {
#ifdef ENABLE_SERIAL
            Serial.println("Received INVALID New power command");
#endif
          }
        }

        /// We check PWM speed target from MQTT
        else if (subscription == &cmnd_led) {
          if (value >= 0 && value <= 1) {
            led_cmnd = value; // we calculate the pwm speed from the % value
          } else {
#ifdef ENABLE_SERIAL
            Serial.println("Received INVALID LED command");
#endif
          }
        }
      }


if (power_cmnd == 1) {

      digitalWrite(RELAY_PIN, HIGH);
      digitalWrite(LED_PIN, LOW); // inverted so LOW <=> ON
      //Serial.println("Relay to ON");
      power_stat = 1;
  
} else {

      digitalWrite(RELAY_PIN, LOW);
      digitalWrite(LED_PIN, HIGH); // inverted so HIGH <=> OFF
      //Serial.println("Relay to OFF");
      power_stat = 0;
  
}


  report_status();
}

///////////////////////////////////////////////////////
///  //////////////////////////////////////////////////
///  //////////////////////////////////////////////////
///  /// ///////////                        ///////////
///  ///   /////////                        ///////////
///          ///////       END OF LOOP      ///////////
////////   /////////                        ///////////
//////// ///////////                        ///////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////



///////////////////////////////////////////////////////
//////  ///////////////////////////////////////////////
//////  ///////////////////////////////////////////////
//////  ///////////                         ///////////
//////  ///////////                         ///////////
//\        ////////        FUNCTIONS        ///////////
///\      /////////                         ///////////
////\    //////////                         ///////////
/////\  ///////////////////////////////////////////////
///////////////////////////////////////////////////////


///////////////////////////////////////////////////////
//////////////// MQTT_connect Function ////////////////
///////////////////////////////////////////////////////

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

#ifdef ENABLE_SERIAL
  Serial.print("Connecting to MQTT... ");
#endif

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
#ifdef ENABLE_SERIAL
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
#endif
       mqtt.disconnect();
       delay(250);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
#ifdef ENABLE_SERIAL
  Serial.println("MQTT Connected!");
#endif
}



///////////////////////////////////////////////////////
//// TRIGGERED BUTTON - INTERRUPTION FUNCTION ///
///////////////////////////////////////////////////////

ICACHE_RAM_ATTR void button_pressed() {

detachInterrupt(digitalPinToInterrupt(BUTTON_PIN)); // we detach the interrupt to prevent false button push detections " like rebound"

Serial.print("button pressed >>> ");

if(power_stat == 0) {
  Serial.println("power_stat == 0 => power_cmnd = 1");
  power_cmnd = 1;
} else {
  Serial.println("power_stat == 1 => power_cmnd = 0");
  power_cmnd = 0;
}



}

///////////////////////////////////////////////////////
/////////////  REPORT THE SYSTEM STATUS  //////////////
///////////// ON THE CONSOLE AND ON MQTT //////////////
///////////////////////////////////////////////////////
void report_status()
{
  static unsigned long previous_millis = 0;
  if (millis() - previous_millis < REPORT_STATUS_PERIOD_MS) {
    // too early to report the status, abort now
    return;
  }

#ifdef ENABLE_SERIAL
  // Serial.print("hello");
  if (power_stat == 0){
  Serial.println("Relay is OFF");
  }
  if (power_stat == 1){
  Serial.println("Relay is ON");
  }
#endif

  /// we repport status and publish to mqtt
stat_power.publish(power_stat);
// stat_button.publish(button_stat);


attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), button_pressed, RISING); // we reattach the interrupt reunabling the button pressing feature

  previous_millis = millis();
}

///////////////////////////////////////////////////////
///  //////////////////////////////////////////////////
///  //////////////////////////////////////////////////
///  /// ///////////                        ///////////
///  ///   /////////                        ///////////
///          ///////    END OF FUNCTIONS    ///////////
////////   /////////                        ///////////
//////// ///////////                        ///////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
