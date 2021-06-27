/*
    Smart Water Guard Controller
    made by Real Drouin date 2021/05/22
    Ver 1.11

******** NodeMCU PIN ASSIGMENT ********
  D0: OnBoard Led.
  D1: extLeakSensor.    <---> rxc6 pin 9 "D3"  HIGH when receive paired water leak device
  D2: Pir.              <---> rxc6 pin 5 "VT"  HIGH when receive all paired device
  D3: Buzzer.
  D4: PushButton.
  D5: Water Flow PWM.
  D6: ValveControl.

  A0: Battery Monitor.
****************************************

********* YF-B5 water flow sensor Specifications ********
  • Mini. Wokring Voltage: DC 4.5V
  • Max. Working Current: 15mA (DC 5V)
  • Working Voltage: DC 5V~15V
  • Flow Rate Range: 1~30L/min
  • Frequency: F=6.6*Q(Q=L/MIN)
  • Load Capacity: ≤10mA (DC 5V)
  • Operating Temperature: ≤80℃
  • Liquid Temperature: ≤120℃
  • Operating Humidity: 35%～90%RH
  • Water Pressure: ≤1.75MPa
  • Storage Temperature: -25～+ 80℃
  • Storage Humidity: 25%～95%RH
**********************************************************

  note:

    Power Init , WaterValve Closed in Manual Mode with Notification.

    Press PushButton 2sec to Open WaterValve in (Normal Mode).

    Press PushButton 2sec to Close WaterValve in (Manual Mode).

    /////////////////
    // Normal Mode //
    ////////////////////////////////////////////////////////////////////////
    // After countdown Timer 24h , valve close automaticly.
    // If motion detected , valve open automaticly.
    //
    ///////////////////////////////////////////////////////////////////////

    ///////////////
    // Away Mode //
    ///////////////////////////////////////////////////////////////////////
    // If NO Motion detected after 30min , (Away Mode Actived),
    // Led Blink 2 time (Fast) to indicate (Away Mode Actived).
    //
    // If water flow detected, valve close after 10sec.
    //////////////////////////////////////////////////////////////////////

    /////////////////
    // Bypass Mode //
    /////////////////////////////////////////////////////////////////
    // If you press for 10sec on pushbutton, (Bypass Mode actived).
    // After 24h valve return to (Normal Mode).
    // For canceling (Bypass Mode), press for 10sec on pushbutton.
    /////////////////////////////////////////////////////////////////

    /////////////////
    // Battery Low //
    //////////////////////////////////////////////////////////////////////////////
    // If low battery detected, send Notification.
    //////////////////////////////////////////////////////////////////////////////

    //////////////////////
    // Check FlowSensor //
    //////////////////////////////////////////////////////////////////////////
    // If NO Water Flow detected after 48h, Notification. (Check FlowSensor).
    //////////////////////////////////////////////////////////////////////////

       ****************************************

******** RXC6 RF receiver 433 Mhz - 4 Channel **********

  Super Hétérodyne récepteur avec décodeur, type d'apprentissage.
  décodage PT2262, PT2260, PT2240, EV1527 et d'autres d'apprentissage
  code to puce ou compatible puces, peut apprendre et magasin 30 pcs de code
  différent télécommande intelligemment distinguer résistance aux chocs entre
  code fixe et l'apprentissage code, comme PT2262 et EV1527.

  Quick Specs

  Frequency: 433.92Mhz
  Operating Voltage: 3 to 5.5V DC
  Quiescunt Current: 4ma
  Receiver sensitivity: -105DB
  Size 30 * 14 * 7mm
  External antenna: 32cm single core wire, wound into a spiral

  PIN ASSIGMENT

  1 = ANT - External Antenna - Spiral 32cm
  2 = GND - Ground (no pin installed)
  3 = External Led +.6v (no pin installed)
  4 = External Switch to GND for programming mode (Pairing) (no pin installed)
  5 = VT: Will Flash when signal receive
  6 = Data D0
  7 = Data D1
  8 = Data D2
  9 = Data D3
  10 = Vcc
  11 = GND

  Jumper Setting

  T1 & T2 Open - Momentory Default
  T2 Closed, T1 Open - Latching Mode
  T2 Open, T1 Closed - Toggle Mode

 **** Learn button ****

  Press and hold the key for 2 seconds,
  learn indicator light at this time that has entered the state of learning key release,
  press any remote control - a key indicator flashes four times indicates success in leaning,
  and exit learning mode.

  To Reset, Press and hold leaning key indicator lit, hold about 8 seconds after the light is off,
  all the code has been succesfully cleared.

  When PIR actived , Pin D0,D1,D2 = High.

*/

const String ver = "Ver 1.11";

#include <ESP8266WiFi.h>
#include <EEPROM.h>

#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266WebServer.h>

#include <ESP8266mDNS.h>

#include <BlynkSimpleEsp8266.h>
String AuthToken = "";
String BlynkServer = "";
int BlynkPort;
volatile bool Connected2Blynk = false;

///////// Valve Variable /////////
volatile bool WaterOn = false;// State of Valve
volatile bool WaterOnNotification = true;
volatile bool WaterFlow = false;

volatile bool Bypass = false;
volatile bool BypassNotification = true;

volatile bool Leak = false; // flow sensor detect
volatile bool extLeak = false; // leak wireless sensor
volatile bool AutoMode = false;
volatile bool Setup = false;

volatile bool alarmArmed = true;
volatile bool alarmArmedNotification = true;

int AwayIn = 0;
int FlowDelay = 0;

///////// Motion Detection //////////
volatile bool Away = false;
volatile bool Motion = false;

///// 433 Mhz TxModule, Leak, ExtTrig, Batt Low /////
const int Buzzer = 0;          //  D3, Buzzer.
const int Led = 16;            //  D0, OnBoard Led.
const int ExtTrigger = 4;      //  D2, Pir.
const int extLeakSensor = 5;   //  D1, extLeakSensor.

///// Valve Logic /////
const int ValveControl = 12;  //  D6

///////// Push Button /////////
const int Button = 2; // D4 Init and Bypass Mode Button (push 10sec for Bypass 24h).
byte PushButtonCount = 0;
volatile bool PushButton;

///////// Water Flow Minuterie //////////
const byte PWM = 14;  // D5
float sensorCal = 6.6; //The hall-effect flow sensor outputs approximately 6.6 pulses per second per litre/minute of flow.
volatile int flow_frequency;
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
unsigned long oldTime;

//////// Battery Monitor ///////
// adc vcc out = vin(12v) * Rbutton(10k) / (Rbutton(33k) + Rtop(10k).
// Ratio = vin(12v) / adc vcc out(2.79)
int BAT = A0;
float battRef = 0.0;
float RatioFactor = 0.0; // resistor ratio factor
float Tvoltage = 0.0;
float Vvalue = 0.0, Rvalue = 0.0;
bool lowBattery = false;

/////// CheckWaterFlowSensor Alert ///////
volatile bool WaterFlowSensorFail = false;

// System Uptime
byte hh = 0; byte mi = 0; byte ss = 0; // hours, minutes, seconds
unsigned int dddd = 0; // days (for if you want a calendar)
unsigned long lastTick = 0; // time that the clock last "ticked"
char timestring[25]; // for output

//////////////// Millis ////////////////
const unsigned long check = 30000; // check wifi connected every 30sec
unsigned long previouscheck = 0;

const unsigned long BypassTimer = 86400000;
unsigned long previousBypassTimer = 0;

const unsigned long AutoCloseValve = 86400000;
unsigned long previousAutoCloseValve = 0;

const unsigned long CheckFlowSensor = 172800000;
unsigned long previousCheckFlowSensor = 0;

const unsigned long NotificationMillis = 10000;
unsigned long previousNotificationMillis = 0;

unsigned long Refresh = 5000;
unsigned long previousRefresh = 0;

unsigned long DelayAway = 600000;

unsigned long FlowallSec;
unsigned long FlowTimer = 1800000;
unsigned long previousFlowTimer = 0;

String webSite, javaScript, XML, header, footer, ssid, password;

///////// Button CSS /////////
const String button = ".button {background-color: white;border: 2px solid #555555;color: black;padding: 16px 32px;text-align: center;text-decoration: none;display: inline-block;font-size: 16px;margin: 4px 2px;-webkit-transition-duration: 0.4s;transition-duration: 0.4s;cursor: pointer;}.button:hover {background-color: #555555;color: white;}.disabled {opacity: 0.6;pointer-events: none;}";
/////////////////////////////

byte percentQ = 0; // wifi signal strength %

ESP8266WebServer server(80);
WiFiClient client;
String readString;

// OTA UPDATE
ESP8266HTTPUpdateServer httpUpdater; //ota

///////////
// Blynk //
///////////////////////////////////////////
BlynkTimer timer;
WidgetLED led0(V0); // PIR Indicator
WidgetLED led1(V1); // AWAY Indicator

bool isFirstConnect = true;
// This function will run every time Blynk connection is established
BLYNK_CONNECTED() {
  if (isFirstConnect) {
    // Request Blynk server to re-send latest values for all pins
    Blynk.syncAll();

    // You can also update individual virtual pins like this:
    //Blynk.syncVirtual(V0, V1, V4);

    isFirstConnect = false;
  }
}

BLYNK_WRITE(V16) //Away Set
{
  int ReadDelayAway = param[0].asInt();
  DelayAway = ReadDelayAway * 60000;
}

BLYNK_WRITE(V17) //FlowDelay Set
{
  int ReadFlowDelay = param[0].asInt();
  FlowDelay = ReadFlowDelay * 60000;
}

BLYNK_WRITE(V5) {
  bool control = param.asInt();

  if (control == true) {
    AutoMode = true;
    Open();
  } else if (control == false) {
    AutoMode = false;
    Close();
  }
}

BLYNK_WRITE(V6) {
  bool bypass = param.asInt();

  if (bypass == true) {
    Bypass = true;
    Blynk.notify("Bypass Actived!");
    unsigned long Millis = millis();
    previousAutoCloseValve = Millis;
    previousBypassTimer = Millis;
  }
  else if (bypass == false) {
    Bypass = false;
    Blynk.notify("Bypass Deactived!");
  }
  BypassNotification = true;
}

BLYNK_WRITE(V7) {
  bool control = param.asInt();

  if (control == true) {
    alarmArmed = true;
    Blynk.notify("Alarm Armed!");
  }
  else if (control == false) {
    alarmArmed = false;
    Blynk.notify("Alarm Disarmed!");
  }
  alarmArmedNotification = true;
}

WidgetTerminal terminal(V10);

/////////////////////////////////////////////
// display on Blynk terminal on mobile app //
//////////////////////////////////////////////////////////////////////////////
BLYNK_WRITE(V10) {
  if (String("?") == param.asStr()) {
    String IP = (WiFi.localIP().toString());

    terminal.clear();
    terminal.println();
    terminal.print(F("Smart Water Guard "));
    terminal.println(ver);
    terminal.println(F("======================================"));
    terminal.print(F("Connected to: "));
    terminal.println(WiFi.SSID());
    terminal.print(F("IP: "));
    terminal.println(IP);
    terminal.print(F("Admin Password: "));
    terminal.println(password);
    terminal.print(F("Batt: "));
    terminal.print(Tvoltage);
    terminal.println(" v");
    terminal.print(F("Signal Strength: "));
    terminal.print(percentQ);
    terminal.println(F("%"));
    terminal.println(F("======================================"));

    terminal.print(F("FlowRate: "));
    terminal.print(flowRate);
    terminal.println(F(" L/min"));
    terminal.print(F("Current Water Quantity: "));
    terminal.print(flowMilliLitres);
    terminal.println(F(" mL/sec"));
    terminal.print(F("Total Water Quantity: "));
    terminal.print(totalMilliLitres);
    terminal.println(F(" mL"));

    terminal.println(F("======================================"));
    terminal.print(F("Delay Before Away Set: "));
    terminal.print(DelayAway / 60000);
    terminal.println(F(" min"));
    terminal.print(F("Water Flow Delay Set: "));
    terminal.print(FlowDelay / 60000);
    terminal.println(F(" min"));
    terminal.println(F("type 'reboot' to reboot controller."));
    terminal.println(F("======================================"));
    terminal.flush();
  }

  if (String("reboot") == param.asStr()) {
    terminal.println(F("Reboot in process!"));
    terminal.flush();
    delay(3000);
    WiFi.disconnect();
    delay(3000);
    ESP.restart();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Rebooted!"));

  pinMode(Buzzer, OUTPUT);
  digitalWrite(Buzzer, HIGH);

  pinMode(ValveControl, OUTPUT);
  digitalWrite(ValveControl, LOW);

  pinMode(Led, OUTPUT); // Led
  pinMode(Button, INPUT_PULLUP);

  pinMode(PWM, INPUT_PULLUP);

  ////// ExtLeakSensor 433Mhz channel 4 //////
  pinMode(extLeakSensor, INPUT_PULLUP); // extLeakSensor Key4, ex: water leak sensor
  pinMode(ExtTrigger, INPUT_PULLUP); // ex: pir

  //////////////////////////////////////
  // START EEPROM
  EEPROM.begin(512);
  delay(200);

  //////////////////////////////////////
  // Init EEPROM
  byte Init = EEPROM.read(451);

  if (Init != 111) {
    for (int i = 0; i < 512; ++i)
    {
      EEPROM.write(i, 0);
    }
    delay(100);
    EEPROM.commit();
    delay(200);

    EEPROM.write(451, 111);
    delay(100);
    EEPROM.commit();
    delay(200);

    // Erase SSID and PASSWORD
    for (int i = 34; i < 100; ++i)
    {
      EEPROM.write(i, 0);
    }
    delay(100);
    EEPROM.commit();
    delay(200);
    Serial.println(F("Init eeprom and set default!"));
  }

  Serial.println(F("Read Config!"));
  //////////////////////////////////////
  // READ EEPROM SSID & PASSWORD
  String esid;
  for (int i = 34; i < 67; ++i)
  {
    esid += char(EEPROM.read(i));
  }
  delay(200);
  ssid = esid;

  String epass = "";
  for (int i = 67; i < 100; ++i)
  {
    epass += char(EEPROM.read(i));
  }
  delay(200);
  password = epass;

  //////////////////////////////////////
  // READ EEPROM FlowDelay
  byte ReadFlowDelay = EEPROM.read(106);
  FlowDelay = ReadFlowDelay * 60000;

  //////////////////////////////////////
  // READ EEPROM DelayAway
  byte ReadDelayAway = EEPROM.read(107);
  DelayAway = ReadDelayAway * 60000;

  //////////////////////////////////////
  // Battery RatioFactor
  String Ratio = "";
  for (int i = 200; i < 206; ++i)
  {
    Ratio += char(EEPROM.read(i));
  }

  RatioFactor = Ratio.toFloat();

  //////////////////////////////////////
  // FlowSensor Calibration
  String ReadFlowCal = "";
  for (int i = 300; i < 306; ++i)
  {
    ReadFlowCal += char(EEPROM.read(i));
  }
  sensorCal = ReadFlowCal.toFloat();

  if (sensorCal < 1.00) {
    sensorCal = 6.6;
  }

  //////////////////////////////////////
  // Blynk Ip Local Server
  int IndexBlynkServer = EEPROM.read(306) + 307;
  String ReadBlynkServer = "";
  for (int i = 307; i < IndexBlynkServer; ++i)
  {
    ReadBlynkServer += char(EEPROM.read(i));
  }
  BlynkServer = ReadBlynkServer;

  /////////////////////////////////////
  // Blynk Port Local Server
  int IndexBlynkPort = EEPROM.read(390) + 391;
  String ReadBlynkPort = "";
  for (int i = 391; i < IndexBlynkPort; ++i)
  {
    ReadBlynkPort += char(EEPROM.read(i));
  }
  BlynkPort = ReadBlynkPort.toInt();

  //////////////////////////////////////
  //BLYNK AUTH TOKEN
  int IndexToken = EEPROM.read(400) + 401;
  String ReadToken = "";
  for (int i = 401; i < IndexToken; ++i)
  {
    ReadToken += char(EEPROM.read(i));
  }
  AuthToken = ReadToken;

  EEPROM.end();

  delay(1000);

  ///////////////////
  // SSID PASSWORD //
  ///////////////////////////////////////////////////////////////
  server.on("/WiFi", []() {
    if (WiFi.status() == WL_CONNECTED) {
      if (!server.authenticate("admin", password.c_str()))
        return server.requestAuthentication();
    }

    String WifiSsid = server.arg("ssid");
    String WifiPassword = server.arg("pass");

    if (WifiPassword.length() > 9 and WifiPassword.length() < 33) {
      // START EEPROM
      EEPROM.begin(512);
      delay(200);

      for (int i = 34; i < 100; ++i)
      {
        EEPROM.write(i, 0);
      }
      delay(100);
      EEPROM.commit();
      delay(200);

      for (int i = 0; i < WifiSsid.length(); ++i)
      {
        EEPROM.write(34 + i, WifiSsid[i]);
      }

      for (int i = 0; i < WifiPassword.length(); ++i)
      {
        EEPROM.write(67 + i, WifiPassword[i]);
      }
      delay(100);
      EEPROM.commit();
      delay(200);
      EEPROM.end();

      handleREBOOT();
    }
    else if (WifiPassword.length() <= 9 or WifiPassword.length() > 32) {
      server.send(200, "text/html", "<header><h1>Error!, Please enter valid PASS! min10 max32 character <a href=/wifisetting >Back!</a></h1></header>");
    }
    else {
      server.send(200, "text/html", "<header><h1>Error!, Please enter PASS! <a href=/wifisetting >Back!</a></h1></header>");
    }
  });

  ////////
  // API //
  ///////////////////////////////////////////////////////////////
  server.on("/api", []() {

    String JsonResponse = "{\n\"waterguard\": ";

    JsonResponse += "{\n\"waterflow\":\"" + String (flowRate) + " L/min\", \n";

    JsonResponse += "\"current_water_quantity\":\"" + String (flowMilliLitres) + " mL/sec\", \n";

    JsonResponse += "\"total_water_quantity\":\"" + String (totalMilliLitres) + " mL\", \n";

    if (Leak == false) {
      JsonResponse += "\"leak\":\"no_leak_detected\", \n";
    } else {
      JsonResponse += "\"leak\":\"water_leak_detected\", \n";
    }

    if (extLeak == false) {
      JsonResponse += "\"extleak\":\"no_leak_detected\", \n";
    } else {
      JsonResponse += "\"extleak\":\"water_leak_detected\", \n";
    }

    if (alarmArmed == true) {
      JsonResponse += "\"alarm\":\"armed\", \n";
    } else {
      JsonResponse += "\"alarm\":\"disarmed\", \n";
    }

    if (Motion == true) {
      JsonResponse += "\"pir\":\"motion detected\", \n";
    } else {
      JsonResponse += "\"pir\":\"no motion\", \n";
    }

    if (WaterOn == true) {
      JsonResponse += "\"valve\":\"open\", \n";
    }
    else {
      JsonResponse += "\"valve\":\"close\", \n";
    }

    if (WaterOn == true && WaterFlow == true && Bypass == false) {
      String buf = getWaterFlow();
      JsonResponse += "\"status\":\"" + String (buf) + "\", \n";
    }
    else if (WaterFlow == true) {
      JsonResponse += "\"status\":\"water_flow_detected\", \n";
    }
    else {
      JsonResponse += "\"status\":\"no_water_flow\", \n";
    }

    if (WaterOn == true && Bypass == false && Leak == false && extLeak == false && Away == false) {
      String buf = getDelayAway();
      JsonResponse += "\"away_in\":\"" + String (buf) + "\", \n";
    } else if (WaterOn == false) {
      JsonResponse += "\"away_in\":\"----\", \n";
    }
    else if (Away == true) {
      JsonResponse += "\"away_in\":\"on\", \n";
    }

    String buf = getAutoCloseValve();
    JsonResponse += "\"auto_close\":\"" + String (buf) + "\", \n";

    buf = getCheckFlowSensor();
    JsonResponse += "\"check_flow_sensor\":\"" + String (buf) + "\", \n";

    if (Bypass == true) {
      buf = getBypassTimer();
      JsonResponse += "\"bypass_24hrs\":\"" + String (buf) + "\", \n";
    }
    else {
      JsonResponse += "\"bypass_24hrs\":\"off\", \n";
    }

    JsonResponse += "\"Vbatt\":\"" + String (Tvoltage) + " Volts\", \n";

    JsonResponse += "\"wifi_signal\":\"" + String (percentQ) + " %\"";

    JsonResponse += "\n}\n\n}";

    server.send(200, "application/json",  JsonResponse);
  });

  ///////////////
  // Blynk Key //
  ////////////////////////////////////////////////////////////
  server.on("/Blynk", []() {
    if (WiFi.status() == WL_CONNECTED) {
      if (!server.authenticate("admin", password.c_str()))
        return server.requestAuthentication();
    }

    String ReadKey = server.arg("key");
    String ReadBlynkIp = server.arg("server");
    String ReadBlynkPort = server.arg("port");

    if (ReadKey.length() < 33) {
      EEPROM.begin(512);
      delay(200);

      EEPROM.write(400, ReadKey.length());
      for (int i = 0; i < ReadKey.length(); ++i)
      {
        EEPROM.write(401 + i, ReadKey[i]);
      }
      delay(100);
      EEPROM.commit();
      delay(200);

      AuthToken = ReadKey;
      EEPROM.end();

      Blynk.config(AuthToken.c_str());
      Blynk.connect();
    }
    handleBLYNK();
  });

  ///////////////
  // Blynk Key //
  ////////////////////////////////////////////////////////////
  server.on("/BlynkServer", []() {
    if (WiFi.status() == WL_CONNECTED) {
      if (!server.authenticate("admin", password.c_str()))
        return server.requestAuthentication();
    }

    String ReadBlynkServer = server.arg("server");
    String ReadBlynkPort = server.arg("port");

    EEPROM.begin(512);
    delay(200);

    EEPROM.write(306, ReadBlynkServer.length());
    for (int i = 0; i < ReadBlynkServer.length(); ++i)
    {
      EEPROM.write(307 + i, ReadBlynkServer[i]);
    }

    EEPROM.write(390, ReadBlynkPort.length());
    for (int i = 0; i < ReadBlynkPort.length(); ++i)
    {
      EEPROM.write(391 + i, ReadBlynkPort[i]);
    }

    delay(100);
    EEPROM.commit();
    delay(200);
    EEPROM.end();

    BlynkServer = ReadBlynkServer;
    BlynkPort = ReadBlynkPort.toInt();

    if (BlynkServer.length() > 5) {
      Blynk.config(AuthToken.c_str(), BlynkServer.c_str(), BlynkPort);
    }
    else {
      Blynk.config(AuthToken.c_str(), "blynk-cloud.com", 8442);
    }

    Blynk.connect();

    handleBLYNK();
  });

  ////////////////
  // Open Valve //
  ///////////////////////////////////////////////////////////////
  server.on("/open", []() {
    if (WiFi.status() == WL_CONNECTED) {
      if (!server.authenticate("admin", password.c_str()))
        return server.requestAuthentication();
    }
    AutoMode = true;
    Open();

    handleOnConnect();
  });

  /////////////////
  // Close Valve //
  ///////////////////////////////////////////////////////////////
  server.on("/close", []() {
    if (WiFi.status() == WL_CONNECTED) {
      if (!server.authenticate("admin", password.c_str()))
        return server.requestAuthentication();
    }
    AutoMode = false;
    Close();

    handleOnConnect();
  });

  ////////////
  // Bypass //
  ///////////////////////////////////////////////////////////////
  server.on("/bypass", []() {
    if (WiFi.status() == WL_CONNECTED) {
      if (!server.authenticate("admin", password.c_str()))
        return server.requestAuthentication();
    }
    String ReadBypass = server.arg("Bypass");

    if (ReadBypass == "false") {
      Bypass = false;
    } else if (ReadBypass == "true") {
      AutoMode = true;
      Open();
      Away = false;
      Bypass = true;
    }
    unsigned long Millis = millis();
    previousAutoCloseValve = Millis;
    previousBypassTimer = Millis;

    handleOnConnect();

    Blynk.notify("Bypass Mode Actived!");
  });

  ///////////
  // Alarm //
  ///////////////////////////////////////////////////////////////
  server.on("/alarm", []() {
    if (WiFi.status() == WL_CONNECTED) {
      if (!server.authenticate("admin", password.c_str()))
        return server.requestAuthentication();
    }
    String ReadAlarm = server.arg("Arm");

    if (ReadAlarm == "false") {
      alarmArmed = false;
      Blynk.notify("Alarm Disarmed!");
    } else if (ReadAlarm == "true") {
      alarmArmed = true;
      Blynk.notify("Alarm Armed!");
    }
    alarmArmedNotification = true;
    handleWATERVALVE();
  });

  ///////////////////
  // Timer Setting //
  ///////////////////////////////////////////////////////////////
  server.on("/TimerSetting", []() {
    if (WiFi.status() == WL_CONNECTED) {
      if (!server.authenticate("admin", password.c_str()))
        return server.requestAuthentication();
    }

    String ReadDelayAway = server.arg("DelayAway");
    String ReadFlowDelay = server.arg("FlowDelay");
    String ReadBattVoltage = server.arg("BattVoltage");
    String ReadFlowCal = server.arg("FlowCal");

    EEPROM.begin(512);
    delay(200);

    if (ReadDelayAway.length() > 0) {
      int value1 = (ReadDelayAway.toInt());
      EEPROM.write(107, value1);
      DelayAway = value1 * 60000;
    }

    if (ReadFlowDelay.length() > 0) {
      int value2 = (ReadFlowDelay.toInt());
      EEPROM.write(106, value2);
      delay(100);
      FlowDelay = value2 * 60000;
    }

    if (ReadBattVoltage.length() > 0) {

      ////////////////////// Battery Voltage ///////////////////
      for (unsigned int i = 0; i < 10; i++) {
        Vvalue = Vvalue + analogRead(BAT);         //Read analog Voltage
        delay(5);                                  //ADC stable
      }
      // RatioFactor = battRef / Vvalue * 1024.0 / 5
      battRef = ReadBattVoltage.toFloat();
      Vvalue = (float)Vvalue / 10.0;               //Find average of 10 values
      Rvalue = (float)(Vvalue / 1024.0) * 5;       //Convert Voltage in 5v factor
      RatioFactor = battRef / Vvalue * 1024.0 / 5; //Find RatioFactor

      String ratio = String(RatioFactor);

      for (int i = 0; i < ratio.length(); ++i)
      {
        EEPROM.write(200 + i, ratio[i]);
      }
    }
    ////////////////////// FlowSensor Calibration ///////////////////
    if (ReadFlowCal.length() > 0) {

      for (int i = 0; i < ReadFlowCal.length(); ++i)
      {
        EEPROM.write(300 + i, ReadFlowCal[i]);
      }
      sensorCal = ReadFlowCal.toFloat();
    }

    EEPROM.commit();// save on eeprom
    delay(200);
    EEPROM.end();
    handleWATERVALVE();
  });

  //////////////////////
  // WiFi Connection //
  ////////////////////////////////////////////////////////////

  if ((ssid != "") and (password != "")) {
    WiFi.disconnect();
    Serial.println(F("Connecting... to Network!"));
    WiFi.mode(WIFI_STA);
    delay(100);
    WiFi.hostname("SmartWaterGuard");
    delay(100);
    WiFi.begin(ssid.c_str(), password.c_str());
    Setup = false;
  }
  else {
    WiFi.disconnect();
    delay(200);
    WiFi.mode(WIFI_AP);
    delay(100);
    WiFi.softAP("smartwaterguard", "");
    Serial.println(F("Setup Mode Actived!"));
    Setup = true;
  }
  delay(5000);

  if (MDNS.begin("smartwaterguard", WiFi.localIP())) {
    Serial.println("MDNS Started");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("MDNS started FAILED");
  }

  if (Setup == false) {
    httpUpdater.setup(&server, "/firmware", "admin", password.c_str());

    server.onNotFound(handleNotFound);
    server.on("/", handleOnConnect);
    server.on("/wifisetting", handleWIFISETTING);
    server.on("/valve", handleWATERVALVE);
    server.on("/blynk", handleBLYNK);
    server.on("/reboot", handleREBOOT);

    ///////////
    // Blynk //
    ///////////////////////////////////////////////////
    if (BlynkServer.length() > 5) {
      Blynk.config(AuthToken.c_str(), BlynkServer.c_str(), BlynkPort);
    }
    else {
      Blynk.config(AuthToken.c_str(), "blynk-cloud.com", 8442);
    }

    Blynk.connect();

    timer.setInterval(1000l, BlynkBroadcast);
    ///////////////////////////////////////////////////

  }
  else {
    server.onNotFound(handleNotFound);
    server.on("/", handleWIFISETTING);
  }

  server.begin();
  MDNS.addService("http", "tcp", 80);
  delay(5000);

  Close();

  flow_frequency = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  oldTime = 0;
}

void loop() {
  MDNS.update();

  if (Connected2Blynk) {
    Blynk.run();
    timer.run();
  }

  server.handleClient();
  unsigned long currentMillis = millis();

  if ((currentMillis - previouscheck >= check)) {

    percentQ = 0;

    if (WiFi.RSSI() <= -100) {
      percentQ = 0;
    } else if (WiFi.RSSI() >= -50) {
      percentQ = 100;
    } else {
      percentQ = 2 * (WiFi.RSSI() + 100);
    }

    /////////////////////////////////////Battery Voltage//////////////////////////////////
    if (WaterFlow == false) {
      for (unsigned int i = 0; i < 10; i++) {
        Vvalue = Vvalue + analogRead(BAT);     //Read analog Voltage
        delay(5);                              //ADC stable
      }
      Vvalue = (float)Vvalue / 10.0;        //Find average of 10 values
      Rvalue = (float)(Vvalue / 1024.0) * 5; //Convert Voltage in 5v factor
      Tvoltage = Rvalue * RatioFactor;      //Find original voltage by multiplying with factor

      if (Tvoltage < 6.0 && lowBattery == false) {
        lowBattery = true;
        Blynk.notify("SmartWaterGuard, Battery Low!");
        delay(100);
        Blynk.email("SmartWaterGuard", "Alert - Battery Low!");
        delay(100);
      } else {
        lowBattery = false;
      }
      Blynk.virtualWrite(V3, Tvoltage);
    }
    ///////////////////////////////////////////////////////////////////////////////////////

    if (Setup == false and WiFi.status() != WL_CONNECTED) {
      Serial.println(F("Reconnecting to WiFi..."));
      WiFi.disconnect();
      delay(100);
      WiFi.mode(WIFI_STA);
      delay(100);
      WiFi.hostname("SmartWaterGuard");
      delay(100);
      WiFi.begin(ssid.c_str(), password.c_str());
    }
    else if (WiFi.status() == WL_CONNECTED) {
      String IP = (WiFi.localIP().toString());
      Serial.print(F("Connected! to "));
      Serial.print (WiFi.SSID());
      Serial.print (F(", Ip: "));
      Serial.println (IP);

      Connected2Blynk = Blynk.connected();
      if (!Connected2Blynk) {
        Serial.println("Not connected! to Blynk server");
        Blynk.connect(3333);  // timeout set to 10 seconds and then continue without Blynk
      }
      else {
        Serial.println("Connected! to Blynk server");
      }

    }
    else if (Setup == true) {
      Serial.println(F("Setup Mode Actived, Please Connect to SSID: SmartWaterGuard, IP: 192.168.4.1"));
    }
    previouscheck = millis();
  }

  // System Uptime
  if ((micros() - lastTick) >= 1000000UL) {
    lastTick += 1000000UL;
    ss++;
    if (ss >= 60) {
      ss -= 60;
      mi++;
    }
    if (mi >= 60) {
      mi -= 60;
      hh++;
    }
    if (hh >= 24) {
      hh -= 24;
      dddd++;
    }
  }

  ///////////////////////////////////////////////////////////////
  //
  // FlowTimer Away or Normal.
  //
  ///////////////////////////////////////////////////////////////
  if (Away == true || WaterOn == false) {
    FlowTimer = 10000;
  }
  else {
    FlowTimer = FlowDelay;
  }

  ///////////////////////////////////////////////////////////////
  //
  // ExtLeakSensor key4 Receiver 433mhz.
  //
  ///////////////////////////////////////////////////////////////
  if (extLeak == false && digitalRead(extLeakSensor) == HIGH) {
    extLeak = true;
    Blynk.notify("Sensor Leak Detected!");
    delay(100);
    Blynk.email("SmartWaterGuard", "Sensor Leak Detected!");
    delay(100);

    Close();
  }

  ///////////////////////////////////////////////////////////////
  //
  // Push Button (init controller and Bypass mode).
  //
  ///////////////////////////////////////////////////////////////
  PushButtonCount = 0;
  PushButton = digitalRead(Button);

  if (PushButton == LOW && PushButtonCount == 0) {
    digitalWrite(Buzzer, LOW);
    delay(50);
    digitalWrite(Buzzer, HIGH);

    while (PushButton == LOW) {
      PushButton = digitalRead(Button);
      delay(1000);
      PushButtonCount++;

      if (PushButtonCount >= 10) {
        digitalWrite(Buzzer, LOW);
        delay(50);
        digitalWrite(Buzzer, HIGH);
        Serial.println(F("Controller reboot in Setup Mode, SSID = SmartWaterGuard."));
        break;
      }
    }
    unsigned long Millis = millis();

    previousAutoCloseValve = Millis;
    previousBypassTimer = Millis;
  }

  ///////////////////////////////////////////////////////////////
  //
  // Erase Ssid & Password to default
  //
  ///////////////////////////////////////////////////////////////

  if (PushButtonCount < 10 and PushButtonCount > 1)
  {
    Leak = false;
    extLeak = false;
    Away = false;
    AutoMode = true;

    if (WaterOn == false) {
      Open();
    } else {
      Close();
      AutoMode = false;
    }
  }

  else if (PushButtonCount >= 10) {

    Blynk.notify("Alert - Init Setup Mode!");
    delay(100);
    Blynk.email("SmartWaterGuard", "Alert - Init Setup Mode!");

    // START EEPROM
    EEPROM.begin(512);
    delay(200);

    // Erase SSID and PASSWORD
    for (int i = 34; i < 100; ++i)
    {
      EEPROM.write(i, 0);
    }
    delay(100);
    EEPROM.commit();
    delay(200);
    EEPROM.end();
    delay(200);
    WiFi.disconnect();
    delay(3000);
    ESP.restart();
  }

  ///////////////////////////////////////////////////////////////
  //
  // PIR External rf 433Mhz Trigger to HIGH.
  //
  ///////////////////////////////////////////////////////////////
  if (digitalRead(ExtTrigger) == HIGH) {
    Motion = true;
    Away = false;

    if (WaterOn == false and AutoMode == true) {
      Open();
    }

    if (alarmArmed == true) {
      Blynk.notify("Alarm - Motion Detected - Alert!");
      delay(100);
      Blynk.email("SmartWaterGuard", "Alarm - Motion Detected - Alert!");
    }

    unsigned long Millis = millis();
    previousAutoCloseValve = Millis;
    previousNotificationMillis = Millis;
  }
  else {
    Motion = false;
  }

  ///////////////////////////////////////////////////////////////
  //
  // Disable Bypass after 24H.
  //
  ///////////////////////////////////////////////////////////////
  if (Bypass == true && millis() - previousBypassTimer >= BypassTimer)
  {
    Bypass = false;
    unsigned long Millis = millis();
    previousCheckFlowSensor = Millis;
    previousAutoCloseValve = Millis;
    previousBypassTimer = Millis;
  }

  ///////////////////////////////////////////////////////////////
  //
  // Notification WaterFlowSensor after 48h if no Flow Detect.
  //
  ///////////////////////////////////////////////////////////////
  if (millis() - previousCheckFlowSensor >= CheckFlowSensor) {
    WaterFlowSensorFail = true;
    Blynk.notify("Alert - Check Water Flow Sensor!");
    delay(100);
    Blynk.email("SmartWaterGuard", "Alert - Check Water Flow Sensor!");
    previousCheckFlowSensor = millis();
  }

  ///////////////////////////////////////////////////////////////
  //
  // Auto CloseValve after 24h if NO Motion Detection.
  //
  ///////////////////////////////////////////////////////////////
  if (Bypass == false && WaterOn == true) {
    if (millis() - previousAutoCloseValve >= AutoCloseValve) {
      Blynk.notify("Alert - AutoClose Water Valve!");
      delay(100);
      Blynk.email("SmartWaterGuard", "Alert - AutoClose Water Valve!");
      delay(100);
      Close();
    }
  }

  ///////////////////////////////////////////////////////////////
  //
  // If no Motion detected after 30min, set Away to true.
  //
  ///////////////////////////////////////////////////////////////
  if (millis() - previousAutoCloseValve >= DelayAway)
  {
    if (Bypass == false) {
      Away = true;
    }
    else {
      Away = false;
    }
  }

  ///////////////////////////////////////////////////////////////
  //
  // Buzzer and Led Notification for 10sec when motion detected.
  //
  ///////////////////////////////////////////////////////////////
  if (millis() - previousNotificationMillis >= NotificationMillis)
  {
    digitalWrite(Buzzer, HIGH);
  }
  else {
    if (Bypass == true || WaterFlowSensorFail == true || Leak == true || extLeak == true)
    {
      digitalWrite(Buzzer, LOW);
    }
  }

  ///////////////////////////////////////////////////////////////
  //
  // Led Indicator
  //
  ///////////////////////////////////////////////////////////////
  if (millis() - previousRefresh >= Refresh) {

    if (Setup == true) {
      Refresh = 250;
      digitalWrite(Led, LOW); // Turn On Led
      delay(50);
      digitalWrite(Led, HIGH); // Turn off LED
      delay(50);
    }
    else {
      Refresh = 5000;
    }

    if (Setup == false && WaterFlow == false && WaterOn == true) {
      Refresh = 5000;
      if (Away == true) {
        digitalWrite(Led, LOW); // Turn On Led
        delay(50);
        digitalWrite(Led, HIGH); // Turn off LED
        delay(150);
      }
      digitalWrite(Led, LOW); // Turn On Led
      delay(50);
      digitalWrite(Led, HIGH); // Turn off LED
    }

    unsigned long Millis = millis();
    previousRefresh = Millis;
  }

  ///////////////////////////////////////////////////////////////
  //
  // Flow Processing PWM (AllEffect Flow Sensor).
  //
  ///////////////////////////////////////////////////////////////
  if (millis() - oldTime > 1000)
  {
    unsigned long Millis = millis();
    detachInterrupt(digitalPinToInterrupt(PWM));

    // Because this loop may not complete in exactly 1 second intervals we calculate
    // the number of milliseconds that have passed since the last execution and use
    // that to scale the output. We also apply the sensorFrequency to scale the output
    // based on the number of pulses per second per units of measure (litres/minute in
    // this case) coming from the sensor.
    flowRate = ((1000.0 / (millis() - oldTime)) * flow_frequency) / sensorCal;

    // Note the time this processing pass was executed. Note that because we've
    // disabled interrupts the millis() function won't actually be incrementing right
    // at this point, but it will still return the value it was set to just before
    // interrupts went away.
    oldTime = millis();

    // Divide the flow rate in litres/minute by 60 to determine how many litres have
    // passed through the sensor in this 1 second interval, then multiply by 1000 to
    // convert to millilitres.
    flowMilliLitres = (flowRate / 60) * 1000;

    // Add the millilitres passed in this second to the cumulative total
    totalMilliLitres += flowMilliLitres;

    Blynk.virtualWrite(V2, flowRate);
    Blynk.virtualWrite(V4, flowMilliLitres);
    Blynk.virtualWrite(V8, totalMilliLitres);

    if (flowRate > 0) {
      digitalWrite(Led, LOW); // Turn on LED
      WaterFlow = true;

      if (Bypass == false and WaterOn == true) {

        FlowallSec = (FlowTimer - (Millis - previousFlowTimer)) / 1000;

        if (FlowallSec <= 0) {
          Leak = true;
          Close();
          Blynk.notify("Water Flow Leak Detected!");
          delay(100);
          Blynk.email("SmartWaterGuard", "Water Flow Leak Detected!");
          delay(100);
        }
      }
      WaterFlowSensorFail = false;

      previousCheckFlowSensor = Millis;
      previousAutoCloseValve = Millis;
      Serial.println(F("Water Flow Detected!"));
    }
    else {
      digitalWrite(Led, HIGH); // Turn off LED
      WaterFlow = false;
      previousFlowTimer = Millis;
    }
    flow_frequency = 0;
    //oldTime = Millis;
    attachInterrupt(digitalPinToInterrupt(PWM), flow, RISING);
  }
}
///////////////////////// END LOOP ////////////////////////////

///////////////////////////////////////////////////////////////
//
// Flow Processing PWM (AllEffect Flow Sensor).
//
///////////////////////////////////////////////////////////////

ICACHE_RAM_ATTR void flow()
{
  flow_frequency++;
}

///////////////////////////////////////////////////////////////
//
// Open Water Valve.
//
///////////////////////////////////////////////////////////////
void Open()
{
  Serial.println(F("detachInterrupt flow sensor!"));
  detachInterrupt(digitalPinToInterrupt(PWM));

  if (AutoMode == true && Leak == false && extLeak == false && WaterOn == false) {

    WaterOn = true;
    digitalWrite(ValveControl, HIGH);

    Blynk.notify("Water Valve Open!");
    delay(100);
    Blynk.email("SmartWaterGuard", "Water Valve Open!");
    WaterOnNotification = true;
    Serial.println(F("Opening Valve!"));
  } else {
    Serial.println(F("Valve Already Opened!"));
  }
  unsigned long Millis = millis();
  previousAutoCloseValve = Millis;
  previousNotificationMillis = Millis;
}

///////////////////////////////////////////////////////////////
//
// Close Water Valve.
//
///////////////////////////////////////////////////////////////
void Close()
{
  Serial.println(F("detachInterrupt flow sensor!"));
  detachInterrupt(digitalPinToInterrupt(PWM));

  if (WaterOn == true) {
    digitalWrite(ValveControl, LOW);

    WaterOn = false;
    Bypass = false;
    Away = false;

    Blynk.notify("Water Valve Close!");
    delay(100);
    Blynk.email("SmartWaterGuard", "Water Valve Close!");
    WaterOnNotification = true;
    Serial.println(F("Closing Valve!"));
  } else {
    Serial.println(F("Valve Already Closed!"));
  }

  unsigned long Millis = millis();
  previousAutoCloseValve = Millis;
  previousNotificationMillis = Millis;
}

///////////////////
// HANDLE REBOOT //
////////////////////////////////////////////////////////////
void handleREBOOT() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!server.authenticate("admin", password.c_str()))
      return server.requestAuthentication();
  }

  String spinner = (F("<html>"));
  spinner += (F("<head><center><meta http-equiv=\"refresh\" content=\"30;URL='http://smartwaterguard.local/'\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"><style>"));

  spinner += (F(".loader {border: 16px solid #f3f3f3;border-radius: 50%;border-top: 16px solid #3498db;"));
  spinner += (F("width: 120px;height: 120px;-webkit-animation: spin 2s linear infinite;animation: spin 2s linear infinite;}"));

  spinner += (F("@-webkit-keyframes spin {0% { -webkit-transform: rotate(0deg); }100% { -webkit-transform: rotate(360deg); }}"));

  spinner += (F("@keyframes spin {0% { transform: rotate(0deg); }100% { transform: rotate(360deg); }}"));

  spinner += (F("</style></head>"));

  spinner += (F("<body>"));
  spinner += (F("<p><h2>Rebooting Please Wait...</h2></p>"));
  spinner += (F("<div class=\"loader\"></div>"));
  spinner += (F("</body></center></html>"));

  server.send(200, "text/html",  spinner);

  delay(1000);
  WiFi.disconnect();
  delay(3000);
  ESP.restart();
}

////////////////////////////////////////////////////////////
//
// Water Valve Setting
//
////////////////////////////////////////////////////////////
void handleWATERVALVE() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!server.authenticate("admin", password.c_str()))
      return server.requestAuthentication();
  }

  buildHeader();
  webSite = header;

  webSite += (F("<div class='topnav' id='myTopnav'>"));
  webSite += (F("<a href='/'>Exit</a>"));
  webSite += (F("<a href='/valve' class='active'>Water Valve</a>"));
  webSite += (F("<a href='/wifisetting'>Wifi</a>"));
  webSite += (F("<a href='/blynk'>Blynk</a>"));
  webSite += (F("<a href='/reboot' onclick=\"return confirm('Are you sure ? ');\">Reboot</a>"));
  webSite += (F("<a href='/firmware'>Firmware Update</a>"));
  webSite += (F("<a href='javascript:void(0);' class='icon' onclick='myFunction()'>"));
  webSite += (F("<i class='fa fa-bars'></i></a>"));
  webSite += (F("</div>"));

  webSite += (F("<br><b><h1 style='font-family:verdana; color:blue; font-size:300%; text-align:center;'>Smart Water Guard</font></h1></b><hr><hr>"));

  webSite += (F("<h1 style='font-family:verdana; color:blue;'><u>Manual Control</u></h1>\n"));

  webSite += (F("<p><a href =/open class=button>Open Valve</a>"));
  webSite += (F("<a href =/close class=button>Close Valve</a></p>"));

  if (Bypass == false) {
    webSite += (F("<p><a href =/bypass?Bypass=true class=button>Active Bypass for 24h</a></p>"));
  } else {
    webSite += (F("<p><a href =/bypass?Bypass=false class=button>Deactive Bypass.</a></p>"));
  }
  webSite += (F("<hr>"));
  if (alarmArmed == false) {
    webSite += (F("<p style='background-color:green;'><a href =/alarm?Arm=true class=button>Alarm Disarm!</a></p>"));
  } else {
    webSite += (F("<p style='background-color:red;'><a href =/alarm?Arm=false class=button>Alarm Armed!</a></p>"));
  }

  webSite += (F("<hr><br><p><form method='get' action='TimerSetting'><label>Delay Before Away (min)</label><br><input type=number min='5' max='60' name='DelayAway' maxlength=2 value="));
  webSite += String(DelayAway / 60000);
  webSite += (F("><input type='submit'></form></p>\n"));

  webSite += (F("<p><form method='get' action='TimerSetting'><label>Water Flow Delay Before Close Valve (min)</label><br><input type=number min='15' max='60' name='FlowDelay' maxlength=2 value="));
  webSite += String(FlowDelay / 60000);
  webSite += (F("><input type='submit'></form></p>\n"));

  webSite += (F("<br><hr><hr><br><p><b>Instruction:</b></p>"));
  webSite += (F("<p>Power Init , WaterValve Closed in Manually Mode with Notification.</p>"));
  webSite += (F("<p>Press PushButton 2sec to Open WaterValve in (Automatic Mode).</p>"));
  webSite += (F("<p>Press PushButton 2sec to Close WaterValve in (Manual Mode).</p>"));
  webSite += (F("<p>Press PushButton 10sec to enter in Setup Mode, SSID = SmartWaterGuard.</p>"));

  webSite += (F("<br><hr><p><b>Normal Mode</b></p>"));
  webSite += (F("<p>After countdown Timer 24h , valve close automaticly.</p>"));
  webSite += (F("<p>If motion detected , valve open automaticly.</p>"));

  webSite += (F("<br><hr><b><p>Away Mode</p></b>"));

  webSite += (F("<p>If <b>NO</b> motion detected after "));
  webSite +=  String(DelayAway / 60000);
  webSite += (F("min, (Away Mode Actived).</p>"));
  webSite += (F("<p>Led Blink 2 times (Fast) to indicate (Away Mode is Actived).</p>"));
  webSite += (F("<p>If water flow detected, valve close after 10sec.</p>"));

  webSite += (F("<br><hr><b><p>Bypass Mode</p></b>"));
  webSite += (F("<p>The controller open the water valve.</p>"));
  webSite += (F("<p>After 24h the valve controller return to (Normal Mode).</p>"));

  webSite += (F("<br><hr><b><p>Check FlowSensor</p></b>"));
  webSite += (F("<p>If NO Water Flow detected after 48h, Notification. (Check FlowSensor).</p>"));

  webSite += (F("<br><hr><b><p>Learning Sensor '433Mhz PiR, Water leak sensor'.</p></b>"));

  webSite += (F("<p>Press and hold the key for 2 seconds,</p>"));
  webSite += (F("<p>learn indicator light at this time that has entered the state of learning key release,</p>"));
  webSite += (F("<p>active pir or water leak sensor, indicator flashes four times indicates success in leaning,</p>"));
  webSite += (F("<p>and exit learning mode.</p>"));

  webSite += (F("<br><p>To Reset, Press and hold leaning key indicator lit, hold about 8 seconds after the light is off,</p>"));
  webSite += (F("<p>all the code has been succesfully cleared.</p>"));
  webSite += (F("<br><hr>"));

  webSite += (F("<b><p>Battery Reading Voltage Value.</p></b>"));
  webSite += (F("<p>Note: Battery model GB4.0-6 (6V 4.0Ah)</p>"));
  webSite += (F("<p>Read the Battery Voltage with a Multimeter.</p>"));
  webSite += (F("<p>After, Save value on input box.</p>"));
  webSite += (F("<br><p><form method='get' action='TimerSetting'><label>Battery Voltage Calibration</label><br><input type=number min='0' step='.01' name='BattVoltage' maxlength=2 ><input type='submit'></form></p>\n"));
  webSite += (F("<br><hr>"));

  webSite += (F("<b><p>Ball Valve CR-03, 5Volts</p></b>"));
  webSite += (F("<p>Red Wire = 5vcc.</p>"));
  webSite += (F("<p>Yellow Wire = 5vcc=Open and Not Connected=Close.</p>"));
  webSite += (F("<p>Blue Wire = Gnd.</p>"));
  webSite += (F("<br><hr>"));

  webSite += (F("<b><p>FlowSensor Calibration</p></b>"));
  webSite += (F("<p>ex: Pulse frequency (Hz) = 7.5Q, Q is flow rate in Litres/minute.</p>"));
  webSite += (F("<p>Frequency set to: <b>"));
  webSite +=  String(sensorCal);
  webSite += (F("</b></p><br><p><form method='get' action='TimerSetting'><label>Enter Sensor Frequency (hz)</label><br><input type=number min='0' step='.1' name='FlowCal' maxlength=2 ><input type='submit'></form></p>\n"));
  webSite += (F("<br><hr>"));

  buildFooter();
  webSite += footer;

  server.send(200, "text/html",  webSite);
}

//////////////////
// WIFI SETTING //
////////////////////////////////////////////////////////////
void handleWIFISETTING() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!server.authenticate("admin", password.c_str()))
      return server.requestAuthentication();
  }

  buildHeader();
  webSite = header;

  webSite += (F("<div class='topnav' id='myTopnav'>"));
  webSite += (F("<a href='/'>Exit</a>"));
  webSite += (F("<a href='/valve'>Water Valve</a>"));
  webSite += (F("<a href='/wifisetting' class='active'>Wifi</a>"));
  webSite += (F("<a href='/blynk'>Blynk</a>"));
  webSite += (F("<a href='/reboot' onclick=\"return confirm('Are you sure ? ');\">Reboot</a>"));
  webSite += (F("<a href='/firmware'>Firmware Update</a>"));
  webSite += (F("<a href='javascript:void(0);' class='icon' onclick='myFunction()'>"));
  webSite += (F("<i class='fa fa-bars'></i></a>"));
  webSite += (F("</div>"));

  webSite += (F("<br><b><h1 style='font-family:verdana; color:blue; font-size:300%; text-align:center;'>Smart Water Guard</font></h1></b><hr><hr>"));


  webSite += (F("<h1 style='font-family:verdana; color:blue;'><u>Wireless Network</u></h1>\n"));

  if (Setup == false) {
    if (WiFi.status() == WL_CONNECTED) {
      String IP = (WiFi.localIP().toString());
      webSite += (F("<p>Network Connected! to <mark>"));
      webSite += WiFi.SSID();
      webSite += (F("</mark></p>"));
      webSite += (F("<p>Ip: "));
      webSite += IP;
      webSite += (F("</p><hr><hr>"));
    }
    else {
      webSite += (F("<p><font color=red>Network Not Connected!</font></p>"));
    }
  }
  else {
    webSite += (F("<hr><p><font color=blue><u>Wifi Scan</u></font></p>"));

    // WiFi.scanNetworks will return the number of networks found
    int n = WiFi.scanNetworks(false, true);

    // sort by RSSI
    int indices[n];
    for (int i = 0; i < n; i++) {
      indices[i] = i;
    }
    for (int i = 0; i < n; i++) {
      for (int j = i + 1; j < n; j++) {
        if (WiFi.RSSI(indices[j]) > WiFi.RSSI(indices[i])) {
          std::swap(indices[i], indices[j]);
        }
      }
    }

    String st = "";
    if (n > 5) n = 5;
    for (int i = 0; i < n; ++i)
    {
      // Print SSID and RSSI for each network found
      st += "<small><li>";
      st += WiFi.RSSI(indices[i]);
      st += " dBm, ";
      st += WiFi.SSID(indices[i]);
      st += "</small></li>";
    }

    webSite += (F("<p>"));
    webSite += st;
    webSite += (F("</p>"));

    //// WiFi SSID & PASSWORD
    webSite += (F("<hr><hr><h1 style='font-family:verdana; color:blue;'><u>Wifi Ssid & Pass</u></h1>\n"));
    webSite += (F("<form method='get' action='WiFi'><label>SSID: </label><input name='ssid' type='text' maxlength=32><br><br><label>PASS: </label><input name='pass' type='password' maxlength=32><br><br><input type='submit'></form>"));

  }

  webSite += (F("<br><p><b>Reset:</b> Push on Button for 10sec to active Setup Mode,</p>"));
  webSite += (F("<p>Controller reboot in Setup Mode, SSID SmartWaterGuard.</p>"));
  webSite += (F("<p>Ip: 192.168.4.1</p>"));
  webSite += (F("<hr>"));

  buildFooter();
  webSite += footer;

  server.send(200, "text/html",  webSite);
}

//////////////////
// HANDLE BLYNK //
////////////////////////////////////////////////////////////
void handleBLYNK() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!server.authenticate("admin", password.c_str()))
      return server.requestAuthentication();
  }

  buildHeader();
  webSite = header;

  webSite += (F("<div class='topnav' id='myTopnav'>"));
  webSite += (F("<a href='/'>Exit</a>"));
  webSite += (F("<a href='/valve'>Water Valve</a>"));
  webSite += (F("<a href='/wifisetting'>Wifi</a>"));
  webSite += (F("<a href='/blynk' class='active'>Blynk</a>"));
  webSite += (F("<a href='/reboot' onclick=\"return confirm('Are you sure ? ');\">Reboot</a>"));
  webSite += (F("<a href='/firmware'>Firmware Update</a>"));
  webSite += (F("<a href='javascript:void(0);' class='icon' onclick='myFunction()'>"));
  webSite += (F("<i class='fa fa-bars'></i></a>"));
  webSite += (F("</div>"));

  webSite += (F("<br><b><h1 style='font-family:verdana; color:blue; font-size:300%; text-align:center;'>Smart Water Guard</font></h1></b><hr><hr>"));

  // BLYNK CONFIG

  webSite += (F("<p><h1 style='font-family:verdana; color:blue; font-size:200%;'>Please Download Blynk APP.</h1></p>"));
  webSite += (F("<p><form method='get' action='Blynk'><label>AuthToken: </label><input type='password' name='key' maxlength='32' value="));
  webSite += String(AuthToken);
  webSite += (F("><input type='submit'></form></p>\n"));

  webSite += (F("<hr><p><h1 style='font-family:verdana; color:blue; font-size:200%;'>Blynk Server</h1></p>"));
  webSite += (F("<p><form method='get' action='BlynkServer'><label>Server: </label><input type='text' name='server' maxlength='80' value="));

  if (BlynkServer.length() > 5) {
    webSite += String(BlynkServer);
  }
  else {
    webSite += (F("blynk-cloud.com"));
  }

  webSite += (F("><label> Port: </label><input type='number' name='port' maxlength='4' value="));

  if (BlynkServer.length() > 5) {
    webSite += String(BlynkPort);
  }
  else {
    webSite += (F("8442"));
  }

  webSite += (F("><p><input type='submit'></form></p>\n"));

  webSite += (F("<p>Note: Enter your Blynk server address or leave empty to use default Blynk server.</p>"));
  webSite += (F("<br><p>Led:V0(Pir), V1(Away).</p>"));
  webSite += (F("<p>Gauge:V2(L/min), V3(Battery). </p>"));
  webSite += (F("<p>Button:V5(Valve Control), V6(Bypass), V7(Alarm).</p>"));
  webSite += (F("<p>Labeled Value:V4(Current Water Quantity), V8(Total Water Quantity), V11(Water Flow), V12(Away in), V13(Auto Close), V14(Bypass), V15(Sensor Check Byte).</p>"));
  webSite += (F("<p>Numeric Input:V16(Away min), V17(Flow Close min).</p>"));
  webSite += (F("<p>V10:(Terminal), Notification and Email.</p>"));
  webSite += (F("<hr><hr>"));

  buildFooter();
  webSite += footer;

  server.send(200, "text/html",  webSite);
}

void handleOnConnect() {
  server.send(200, "text/html", SendHTML());
}

/////////////////////
// HANDLE NOTFOUND //
////////////////////////////////////////////////////////////
void handleNotFound() {
  server.sendHeader("Location", "/", true);  //Redirect to our html web page
  server.send(302, "text/plane", "");
}

String SendHTML() {

  String ptr = (F("<!DOCTYPE html>"));
  ptr += (F("<html lang='en'>"));

  ptr += (F("<head>"));
  //  <!-- Required meta tags -->
  ptr += (F("<title>Smart Water Guard</title>"));
  ptr += (F("<meta charset='utf-8'>"));

  ptr += (F("<meta name='viewport' content='width=device-width, initial-scale=1.0' shrink-to-fit=no'>"));
  ptr += (F("<meta name='description' content='Smart Water Guard'>"));
  ptr += (F("<meta name='author' content='Made by Real Drouin'>"));

  ptr += (F("<link href='https://fonts.googleapis.com/css?family=Open+Sans:300,400,600' rel='stylesheet'>"));
  ptr += (F("<style>"));
  ptr += (F("html { font-family: 'Open Sans', sans-serif; display: block; margin: 0px auto; text-align: center;color: #444444;}"));
  ptr += (button);

  ptr += (F("</style>"));

  // AJAX script
  ptr += (F("<script>\n"));
  ptr += (F("setInterval(loadDoc,1000);\n")); // Update WebPage Every 1sec
  ptr += (F("function loadDoc() {\n"));
  ptr += (F("var xhttp = new XMLHttpRequest();\n"));
  ptr += (F("xhttp.onreadystatechange = function() {\n"));
  ptr += (F("if (this.readyState == 4 && this.status == 200) {\n"));
  ptr += (F("document.body.innerHTML =this.responseText}\n"));
  ptr += (F("};\n"));
  ptr += (F("xhttp.open(\"GET\", \"/\", true);\n"));
  ptr += (F("xhttp.send();\n"));
  ptr += (F("}\n"));
  ptr += (F("</script>\n"));
  ///////////////////////////////////////

  ptr += (F("</head>"));
  ptr += (F("<body>"));
  ptr += (F("<br><b><h1 style='font-family:verdana;color:blue; font-size:300%; text-align: center;'>Smart Water Guard</font></h1></b>"));

  if (Connected2Blynk) {
    ptr += (F("<p><mark>Blynk Server Connected!</mark></p>"));
  }
  else {
    ptr += (F("<p><mark>Blynk Server Disconnected!</mark></p>"));
  }

  ptr += (F("<hr><hr><h1>"));

  ///////////////////////////////////////////
  if (Away == true) {
    ptr += (F("<p><font color='red'>** Away Actived! **</font></p>"));
  }
  ///////////////////////////////////////////
  if (Leak == true)
  {
    ptr += (F("<p><font color='red'>Alert! from Water Flow Sensor!</font></p>"));
  }
  if (extLeak == true) {
    ptr += (F("<p><font color='red'>Alert! from Wireless Water Leak Sensor!</font></p>"));
  }
  ///////////////////////////////////////////
  if (WaterFlowSensorFail == true) {
    ptr += (F("<p><font color='red'>** Check FlowSensor! **</font></p>"));
  }

  ptr += (F("</h1><h2>"));

  ////////////////////////////////////////////
  if (WaterOn == true) {
    ptr += (F("<p><font color='blue'>Valve: </font> Open</p>"));
  }
  else {
    ptr += (F("<p><font color='blue'>Valve: </font> Close</p>"));
  }

  ////////// WaterFlow Timer ////////

  if (WaterOn == true && WaterFlow == true && Bypass == false) {
    String buf = getWaterFlow();
    ptr += (F("<hr><p><font color='red'>Close Water Valve in: </font>"));
    ptr += (buf);
    ptr += (F("</p>"));

  }
  else if (WaterFlow == true) {
    ptr += (F("<hr><p><font color='red'>Water Flow Detected!</font>"));
  }
  else {
    ptr += (F("<hr><p><font color='blue'>NO Water Flow!</font>"));
  }

  ptr += (F("</h2><hr><hr>"));

  ///////////// Away ////////////
  if (WaterOn == true && Bypass == false && Leak == false && extLeak == false && Away == false) {
    String buf = getDelayAway();
    ptr += (F("<h4><p><font color='blue'>Away in:</font> "));
    ptr += (buf);
    ptr += (F("</p></h4>"));
  }

  ///////////// AutoCloseValve ////////////
  if (WaterOn == true && Bypass == false && Leak == false && extLeak == false) {
    String buf = getAutoCloseValve();
    ptr += (F("<h4><p><font color='blue'>AutoClose in:</font> "));
    ptr += (buf);
    ptr += (F("</p></h4>"));
  }

  ////////////////// Bypass /////////////
  if (WaterOn == true && Bypass == true && Leak == false && extLeak == false) {
    String buf = getBypassTimer();
    ptr += (F("<h4><p><font color='red'>Smart Water Guard Bypass for: </font>"));
    ptr += (buf);
    ptr += (F("</p></h4>"));
  }

  ///////////// Check Flow Sensor /////////////
  if (extLeak == false && Leak == false && WaterFlowSensorFail == false) {
    String buf = getCheckFlowSensor();
    ptr += (F("<h4><p><font color='blue'>Check! Flow Sensor in:</font> "));
    ptr += (buf);
    ptr += (F("</p></h4><hr><hr>"));
  }
  //////////////////////////////////////////////////

  ptr += (F("<br></body>"));

  if (Setup == true) {
    ptr += (F("<h3><font color='black'><mark>Setup Mode Actived!</mark></font></h3>"));
  }
  else {
    if (AutoMode == false) {
      ptr += (F("<h3><font color='black'><mark>Mode Manual Actived!</mark></font></h3>"));
    }
    else {
      if (Motion == true) {
        ptr += (F("<h3><font color='black'><mark>Motion Detect!</mark></font></h3>"));
      } else {
        ptr += (F("<h3><font color='blue'>PiR</font></h3>"));
      }
    }
  }

  ptr += (F("<br><p><a href ='/valve' class='button'>Admin</a></p>"));
  ptr += (F("<hr><hr>"));
  //////////////////////////////////////////////////////////////////////
  ptr += (F("<p><font color = 'blue'>Flow rate: </font>"));
  ptr += (flowRate);
  ptr += (F("<font color = 'blue'> L/min</font></p>"));
  ptr += (F("<p><font color = 'blue'>Current Water Quantity: </font>"));
  ptr += (flowMilliLitres);
  ptr += (F("<font color = 'blue'> mL/sec</font></p>"));
  ptr += (F("<p><font color = 'blue'>Total Water Quantity: </font>"));
  ptr += (totalMilliLitres);
  ptr += (F("<font color = 'blue'> mL</font></p>"));
  //////////////////////////////////////////////////////////////////////
  ptr += (F("<br><p><font color = 'blue'>Battery Voltage: </font>"));
  ptr += (Tvoltage);
  ptr += (F("<font color = 'blue'> volts</font></p>"));

  if (Tvoltage < 6.0) {
    ptr += (F("<p><font color = 'Red'>Low Battery!</font></p>"));
  }
  else {
    ptr += (F("<p><font color = 'green'>Battery OK!</font></p>"));
  }

  ptr += (F("<hr><br><p><font color = 'blue'><i>Signal Strength: </i></font> "));
  ptr += String(percentQ);
  ptr += (F(" %</p>"));
  // System Uptime
  sprintf(timestring, "%d days %02d:%02d:%02d", dddd , hh, mi, ss);
  ptr += (F("<p>System Uptime: "));
  ptr += String(timestring);
  ptr += (F("</p>"));

  //////////////////////////////////////

  ptr += (F("<p><small>Smart Water Guard "));
  ptr += (ver);
  ptr += (F("</p><p><small>Made by Real Drouin ve2droid@gmail.com</small></p>"));
  ptr += (F("</html>\n"));
  return ptr;
}

//////////////////
// BUILD HEADER //
////////////////////////////////////////////////////////////
void buildHeader() {

  header = "";
  header += (F("<!doctype html>\n"));
  header += (F("<html lang='en'>"));

  header += (F("<head>"));
  //  <!-- Required meta tags -->
  header += (F("<title>Smart Water Guard</title>"));
  header += (F("<meta charset='utf-8'>"));
  header += (F("<meta name='viewport' content='width=device-width, initial-scale=1.0' shrink-to-fit=no'>"));
  header += (F("<meta name='description' content='Smart Water Guard'>"));
  header += (F("<meta name='author' content='Made by Real Drouin'>"));
  header += (F("<link rel='stylesheet' href='https://cdnjs.cloudflare.com/ajax/libs/font-awesome/4.7.0/css/font-awesome.min.css'>"));

  header += (F("<style>body {margin: 0;text-align: center;font-family: Arial, Helvetica, sans-serif;}"));
  header += (F(".topnav {overflow: hidden;background-color: #333;}"));
  header += (F(".topnav a {float: left;display: block;color: #f2f2f2;text-align: center;padding: 14px 16px;text-decoration: none;font-size: 17px;}"));
  header += (F(".topnav a:hover {background-color: #0066CC;color: black;}"));
  header += (F(".topnav a.active {background-color: blue;color: white;}"));
  header += (F(".topnav .icon {display: none;}"));

  header += (F("@media screen and (max-width: 600px) {.topnav a:not(:first-child) {display: none;}.topnav a.icon {float: right;display: block;}}"));
  header += (F("@media screen and (max-width: 600px) {.topnav.responsive {position: relative;}.topnav.responsive .icon {position: absolute;right: 0;top: 0;}"));
  header += (F(".topnav.responsive a {float: none;display: block;text-align: left;}}"));

  header += String(button);
  header += (F("</style>\n"));

  header += (F("</head>\n"));
  header += (F("<body>\n"));
}

//////////////////
// BUILD FOOTER //
////////////////////////////////////////////////////////////
void buildFooter() {

  footer = (F("<br>"));
  footer += (F("<address> Contact: <a href='mailto:ve2droid@gmail.com'>Real Drouin</a>"));
  footer += (F("</address>"));
  footer += (F("<p><small>Made by Real Drouin, Smart Water Guard "));
  footer += String(ver);
  footer += (F("</small>"));
  footer += (F("</footer>"));

  footer += (F("<script>function myFunction() {var x = document.getElementById('myTopnav');if (x.className === 'topnav') {x.className += ' responsive';} else {x.className = 'topnav';}}</script>"));

  footer += (F("</body>\n"));
  footer += (F("</html>\n"));
}

void BlynkBroadcast() {

  if (Away == true) {
    led1.on();
  } else {
    led1.off();
  }

  if (Motion == true) {
    led0.on();
  } else {
    led0.off();
  }
  /////////////////////////////
  if (alarmArmedNotification == true) {
    Blynk.virtualWrite(V7, alarmArmed);
    if (alarmArmed == true) {
      Blynk.setProperty(V7, "onLabel", "Alarm Armed!");
      Blynk.setProperty(V7, "offLabel", "...");
    }
    else if (alarmArmed == false) {
      Blynk.setProperty(V7, "onLabel", "...");
      Blynk.setProperty(V7, "offLabel", "Alarm Disarmed!");
    }
    alarmArmedNotification = false;
  }
  ////////////////////////////
  if (BypassNotification == true) {
    Blynk.virtualWrite(V6, Bypass);
    if (Bypass == true) {
      Blynk.setProperty(V6, "onLabel", "Bypass Actived! for 24h");
      Blynk.setProperty(V6, "offLabel", "...");
    }
    else if (Bypass == false) {
      Blynk.setProperty(V6, "onLabel", "...");
      Blynk.setProperty(V6, "offLabel", "Bypass Deactived!");
    }
    BypassNotification = false;
  }
  ////////////////////////////
  if (WaterOnNotification == true) {
    Blynk.virtualWrite(V5, WaterOn);
    if (WaterOn == true) {
      Blynk.setProperty(V5, "onLabel", "Water Valve Open!");
      Blynk.setProperty(V5, "offLabel", "...");
    }
    else if (WaterOn == false) {
      Blynk.setProperty(V5, "onLabel", "...");
      Blynk.setProperty(V5, "offLabel", "Water Valve Close!");
    }
    WaterOnNotification = false;
  }

  ////////// WaterFlow Timer ////////
  if (WaterOn == true && WaterFlow == true && Bypass == false) {
    String buf = getWaterFlow();
    Blynk.virtualWrite(V11, buf);
  }
  else if (WaterFlow == true) {
    Blynk.virtualWrite(V11, "Water Flow Detected!");
  }
  else {
    Blynk.virtualWrite(V11, "NO Water Flow Detected!");
  }

  ///////////// Away ////////////
  if (WaterOn == true && Bypass == false && Leak == false && extLeak == false && Away == false) {
    String buf = getDelayAway();
    Blynk.virtualWrite(V12, buf);
  }
  else if (WaterOn == false) {
    Blynk.virtualWrite(V12, "----");
  }
  else if (Away == true) {
    Blynk.virtualWrite(V12, "Away!");
  }

  ///////////// AutoCloseValve ////////////
  if (WaterOn == true && Bypass == false && Leak == false && extLeak == false) {
    String buf = getAutoCloseValve();
    Blynk.virtualWrite(V13, buf);
  }
  else if (WaterOn == false) {
    Blynk.virtualWrite(V13, "----");
  }

  ////////////////// Bypass /////////////
  if (WaterOn == true && Bypass == true && Leak == false && extLeak == false) {
    String buf = getBypassTimer();
    Blynk.virtualWrite(V14, buf);
  }
  else {
    Blynk.virtualWrite(V14, "Bypass OFF!");
  }

  ///////////// Check Flow Sensor /////////////
  if (extLeak == false && Leak == false && WaterFlowSensorFail == false) {
    String buf = getCheckFlowSensor();
    Blynk.virtualWrite(V15, buf);
  }
  else if (WaterFlowSensorFail == true) {
    Blynk.virtualWrite(V15, "Check Flow Sensor!");
  }
}

String getCheckFlowSensor() {
  unsigned long allSec = (CheckFlowSensor - (millis() - previousCheckFlowSensor)) / 1000;

  int secsRemaining = allSec % 3600;
  int Hrs = allSec / 3600;
  int Min = secsRemaining / 60;
  int Sec = secsRemaining % 60;

  char buf[21];
  sprintf(buf, "%02dh:%02dm:%02ds", Hrs, Min, Sec);
  return (buf);
}

String getBypassTimer() {
  unsigned long allSec = (BypassTimer - (millis() - previousBypassTimer)) / 1000;
  int secsRemaining = allSec % 3600;
  int Hrs = allSec / 3600;
  int Min = secsRemaining / 60;
  int Sec = secsRemaining % 60;

  char buf[21];
  sprintf(buf, "Bypass ON! for: %02dh:%02dm:%02ds", Hrs, Min, Sec);
  return (buf);
}

String getAutoCloseValve() {
  unsigned long allSec = (AutoCloseValve - (millis() - previousAutoCloseValve)) / 1000;

  int secsRemaining = allSec % 3600;
  int Hrs = allSec / 3600;
  int Min = secsRemaining / 60;
  int Sec = secsRemaining % 60;

  char buf[21];
  sprintf(buf, "%02dh:%02dm:%02ds", Hrs, Min, Sec);
  return (buf);
}

String getDelayAway() {
  unsigned long allSec = (DelayAway - (millis() - previousAutoCloseValve)) / 1000;

  int secsRemaining = allSec % 3600;
  int Min = secsRemaining / 60;
  int Sec = secsRemaining % 60;

  char buf[21];
  sprintf(buf, "%02dm:%02ds", Min, Sec);
  return (buf);
}

String getWaterFlow() {
  int secsRemaining = FlowallSec % 3600;
  int Min = secsRemaining / 60;
  int Sec = secsRemaining % 60;

  char buf[21];
  sprintf(buf, "Close Water Valve in: %02dm:%02ds", Min, Sec);
  return (buf);
}

