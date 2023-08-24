#include <WiFi.h>
#include <HTTPClient.h>
#include <TimeLib.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <math.h>
#include <WiFiClientSecure.h>
#include <Ticker.h>

//SGP4
#include <Sgp4.h>

#define PIN_LED 2
//#define xDEBUG 
#define DEBUG
//#define DEBUG_UPDATE_SAT
//#define DEBUG_IN_PASS
#define DEBUG_END_PASS
//
#define HALL_SENSORS  //홀센서 사용안할경우 주석처리


#include <AccelStepper.h>
#include <MultiStepper.h>





// Azimuth를 조절 해주는 모터 핀
#define AZ_DIR_PIN 33
#define AZ_STEP_PIN 32
// Elevation를 조절 해주는 모터 핀
#define EL_DIR_PIN 18
#define EL_STEP_PIN 5

#define ENABLE_PIN 4  //모터 드라이브에 접속할 핀

#define AZ_ZERO_PIN 19  //Azimuth 홀 센서
#define EL_ZERO_PIN 23  //Elevation 홀 센서



enum Calib { LOST,
             AZ_ZERO1,
             AZ_ZERO2,
             AZ_ZERO3,
             EL_ZERO1,
             EL_ZERO2,
             EL_ZERO3,
             EL_ZERO4,
             HORIZONTAL,
             NORTH,
             CALIB_OK,
             GET_TLE };
#ifdef HALL_SENSORS
int calStatus = LOST;  //홀 센서를 장착할 경우
#else
int calStatus = CALIB_OK;  //홀 센서를 장착하지 않을 경우
#endif
long calTime;



// Define some steppers and the pins the will use
AccelStepper AZstepper(AccelStepper::DRIVER, AZ_STEP_PIN, AZ_DIR_PIN);
AccelStepper ELstepper(AccelStepper::DRIVER, EL_STEP_PIN, EL_DIR_PIN);

// Up to 10 steppers can be handled as a group by MultiStepper
MultiStepper steppers;
long positions[2];  // 스텝 2개를 위한 스텝 위치 배열
int satAZsteps;
int satELsteps;
int turns = 0;
int satAZsteps2;
int satELsteps2;
int AZspeed, ELspeed;
long oneTurn = 2048; 
boolean dirCW = true;
boolean posCW = true;
boolean tracking = false;

//SGP4
#define NB_SAT 10
int nbSat = 1;                                                           //Number of satellites to track
char TLE[500];                                                           //length of satellite TLE
char satnames[NB_SAT][30] = {"NAVSTAR 52 (USA 168)"};  //Names of satellites. (found here : https://www.celestrak.com/satcat/search.php)
String startURL = "/NORAD/elements/gp.php?CATNR=";
int satID[NB_SAT] = {27704};  //ID of Celestrak TLEs for satellites
String TLEURL = "&FORMAT=TLE";

char TLE1[NB_SAT][100];
char TLE2[NB_SAT][100];

float myLat = 51.169392, myLong = 71.449074, myAlt = 347;  

int delayNext = 0;  //used to delay acquisition (in s) 
int timeSpeed = 5;  //used to speed up the time during tracking  
boolean firstTimeSpeed = true;
boolean noTLE = true;  //boot with no TLE and try to get them either via Wifi (celestrack) 
long TLEtimeOut;
int prevCalStatus;
int TLEday, nowDay;
WiFiClient client;  //used to get TLE

Sgp4 sat;

//wifi
String ssid = "neededyas";     //WiFi ID
String password = "neededyas";  //WiFi SSID


//time
boolean noTime = true;  //boot with no time and try to get it via wifi (NTP) or from smartphone (BLE)

int TimeZone = 6;
const int dst = 0;

unsigned long long LastStep;
long LastBLEnotification;  //bluetooth communication and real time map of acquisition in Pro version
long lastWake;

int i;
int k;
int SAT;
int nextSat;
int AZstart;
long passEnd;
int satVIS;
char satname[] = { 3 };
int passStatus = 0;
char server[] = "104.168.149.178";  //Celesctrak의 웹사이트 주소
int years;
int months;
int days;
int hours;
int minutes;
double seconds;
int today;
long nextpassEpoch;
long upcomingPasses[4];

unsigned long timeNow = 0;
double jTimeNow;  //julian date with ms precision
long lastDebug;
long nbLoops = 0;
float loopDuration;


//multitasking
TaskHandle_t Task1;


void core0Task1(void* parameter)  //this task will run for ever on core 0
{
  for (;;) {
    delay(2);
    if (tracking) {
      //      AZstepper.runToNewPosition(satAZsteps2);
      //      ELstepper.runToNewPosition(satELsteps2);
      positions[0] = satELsteps2;
      positions[1] = satAZsteps2;
      steppers.moveTo(positions);
      steppers.runSpeedToPosition();  // Blocks until all are in position
    } else delay(200);
  }
}

int nextSatPass(long _nextpassEpoch[NB_SAT]) {  // Replace with number of satellites
  for (i = 0; i < nbSat; ++i) {
    if (_nextpassEpoch[0] - timeNow >= _nextpassEpoch[i] - timeNow) {
      _nextpassEpoch[0] = _nextpassEpoch[i];
      nextSat = i;
    }
  }
  return nextSat;
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);  //disable
  pinMode(AZ_ZERO_PIN, INPUT_PULLUP);
  pinMode(EL_ZERO_PIN, INPUT_PULLUP);
  Serial.begin(115200);




  //connect to WiFi
  WiFi.begin(ssid.c_str(), password.c_str());
  long start = millis();
  while ((WiFi.status() != WL_CONNECTED) && (millis() - start < 15000)) {
    delay(500);
    Serial.print(".");
  }
  //init and get the time
  Serial.println("trying to get time 1");
  configTime(TimeZone * 3600, dst * 0, "pool.ntp.org");
  printLocalTime();

  //init and get the time
  Serial.println("trying to get time 2");  //call it twice to have a well synchronized time on soft reset... Why ? bex=caus eit works...
  delay(2000);
  configTime(TimeZone * 3600, dst * 0, "pool.ntp.org");
  printLocalTime();

  //SGP4
  sat.site(myLat, myLong, myAlt);  //set location latitude[°], longitude[°] and altitude[m]

  if (!noTime)  //don't try to fetch TLE if time is not set
  {

    time_t now;
    struct tm* timeinfo;
    time(&now);
    timeinfo = localtime(&now);
    TLEday = timeinfo->tm_mday;
    nowDay = TLEday;
    timeNow = unix_time_get_time();
    // Get TLEs //
    for (SAT = 0; SAT < nbSat; SAT++) {
      getTLE(SAT);
    }
    updateSatellites();
    // Print obtained TLE in serial. //
#ifdef DEBUG_TLE
    for (SAT = 0; SAT < nbSat; SAT++)


    {
      Serial.println("TLE set #:" + String(SAT));
      for (i = 0; i < 70; i++) {
        Serial.print(TLE1[SAT][i]);
      }
      Serial.println();
      for (i = 0; i < 70; i++) {
        Serial.print(TLE2[SAT][i]);
      }
      Serial.println();
    }
    Serial.println("Next satellite: " + String(nextSat));
#endif
  }



  // Setup stepper movements //
  digitalWrite(ENABLE_PIN, HIGH);  //disable
  digitalRead(ENABLE_PIN);
  ELstepper.setMaxSpeed(700);
  ELstepper.setCurrentPosition(0);  // Elevation stepper starts at 0 degrees above horizon
  ELstepper.setAcceleration(40);
  AZstepper.setMaxSpeed(700);
  AZstepper.setCurrentPosition(0);  // Azimuth stepper starts at 0 (north)
  AZstepper.setAcceleration(40);

  //  ELstepper.setPinsInverted(true, false, false); //DIR pin inverted on EL motor --> uncomment to inverse EL motor
  //AZstepper.setPinsInverted(true,false,false ); //DIR pin inverted on AZ motor --> uncomment to inverse AZ motor

  // Then give them to MultiStepper to manage
  steppers.addStepper(ELstepper);
  steppers.addStepper(AZstepper);

  //refresh timing
  LastStep = esp_timer_get_time();
  lastDebug = millis();
  lastWake = millis();


  //multitask
  xTaskCreatePinnedToCore(
    core0Task1, /* Task function. */
    "Task_1",   /* name of task. */
    5000,       /* Stack size of task */
    NULL,       /* parameter of the task */
    1,          /* priority of the task */
    &Task1,     /* Task handle to keep track of created task */
    0);
}

void printLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("d to obtain time");
    noTime = true;
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  noTime = false;
}

int64_t unix_time_get_time() {
  struct timeval tv;
  static long delta = 0;
  gettimeofday(&tv, NULL);
  //return (tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL));


  firstTimeSpeed = true;
  delta = 0;
  jTimeNow = (double(((tv.tv_sec + delayNext) * 1000LL + (tv.tv_usec / 1000LL))) / 86400.0 / 1000.) + 2440587.5;
  return (tv.tv_sec + delayNext);
}
//-----------------------------------------------------------------------------------------------------------------

void loop() {

  timeNow = unix_time_get_time();  //update time
  switch (calStatus) {
    case LOST:
      digitalWrite(ENABLE_PIN, LOW);         //enable
      if (digitalRead(AZ_ZERO_PIN) == HIGH)  //run Az motor CW until hall effect sensor reached
      {
        AZstepper.setSpeed(150);
        AZstepper.runSpeed();
        //Serial.print(digitalRead(AZ_ZERO_PIN));
      } else {
        Serial.println(digitalRead(AZ_ZERO_PIN));
        AZstepper.stop();  //magnet reached
        Serial.println("azimuth sensor reached");
        AZstepper.setCurrentPosition(0);
        calTime = millis();
        AZstepper.moveTo(-oneTurn * 10 / 360);  //45° CCW로 이동하여 홀 센서의 잠금을 해제합니다
        calStatus = AZ_ZERO1; 
        //        calStatus = AZ_ZERO3;
      }
      break;

    case AZ_ZERO1:
      AZstepper.run();
      if ((millis() - calTime) > 5000)  //홀 효과 센서에 도달할 때까지 Az 모터를 작동합니다
      {
        AZstepper.setCurrentPosition(0);
        calStatus = AZ_ZERO2;
      }
      break;

    case AZ_ZERO2:
    Serial.println("AZZERO2 status");
      if (digitalRead(AZ_ZERO_PIN) == HIGH)  //홀 효과 센서에 도달할 때까지 Az 모터를 작동합니다
      {
        AZstepper.setSpeed(150);
        AZstepper.runSpeed();
      } else {
        AZstepper.stop();
        Serial.println("azimuth sensor reached");
        AZstepper.setCurrentPosition(0);
        calTime = millis();
        //        AZstepper.moveTo(oneTurn * 10 / 360);  //자석의 중심에 맞춘다.
        calStatus = AZ_ZERO3;
      }
      break;

    case AZ_ZERO3:
    
      AZstepper.run();
      if ((millis() - calTime) > 4000)  //wait 6s
      {
        AZstepper.setCurrentPosition(0);
        Serial.println("The AZ motor has been calibrated.");
        calStatus = EL_ZERO1;
      }
      break;

    case EL_ZERO1:
      if (digitalRead(EL_ZERO_PIN) == HIGH)  //홀 효과 센서에 도달할 때까지 Az 모터 CW를 작동합니다
      {
        ELstepper.setSpeed(100);
        ELstepper.runSpeed();
        //Serial.print(digitalRead(EL_ZERO_PIN));
      } else {
       // Serial.println(digitalRead(EL_ZERO_PIN));
        ELstepper.stop();  //magnet reached
        Serial.println("elevation sensor reached");
        ELstepper.setCurrentPosition(0);
        calTime = millis();
        ELstepper.moveTo(-oneTurn * 10 / 360);  //45° CCW로 이동하여 홀 센서의 잠금을 해제합니다
        calStatus = EL_ZERO2;
        //        calStatus = EL_ZERO4;
      }
      break;

    case EL_ZERO2:
      ELstepper.run();
      if ((millis() - calTime) > 5000)  //6초후 자석앞에 선다.
      {
        ELstepper.setCurrentPosition(0);
        calStatus = EL_ZERO3;
      }
      break;

    case EL_ZERO3:
      if (digitalRead(EL_ZERO_PIN) == HIGH)  //run Az motor until hall effect sensor reached
      {
        ELstepper.setSpeed(100);
        ELstepper.runSpeed();
      } else {
        ELstepper.stop();
        Serial.println("elevation sensor reached");
        ELstepper.setCurrentPosition(0);
        calTime = millis();
        //        ELstepper.moveTo(oneTurn / 360 * 3);  //align with the center of the magnet
        calStatus = EL_ZERO4;
      }
      break;

    case EL_ZERO4:
      //ELstepper.run();
      //if ((millis() - calTime) > 5000) //wait 8s
      {
        ELstepper.setCurrentPosition(0);
        Serial.println("EL motor calibrated.");
        calTime = millis();
        if (!noTime) {
          ELstepper.moveTo(-oneTurn / 360 * 88);  //자석의 중심에서 88° 떨어진 곳에 정렬합니다
          calStatus = HORIZONTAL;
        }
      }
      break;

    case HORIZONTAL:
      ELstepper.run();
      if ((millis() - calTime) > 17000)  //wait 12s
      {
        ELstepper.setCurrentPosition(0);  //EL stepper is now horizontal
        Serial.println("antenna horizontal");
        calTime = millis();
        if (!noTLE) {
          AZstepper.moveTo(-oneTurn / 360 * 93);  //자석의 중심에서 93° 떨어진 곳에 정렬합니다
          calStatus = NORTH;
        }
      }
      break;

    case NORTH:
      AZstepper.run();
      if ((millis() - calTime) > 18000)  //wait 12s
      {
        AZstepper.setCurrentPosition(0);  //AZ스텝 모터가 북쪽을 바라보고있다.
        Serial.println("The antenna is looking north.");
        calStatus = CALIB_OK;
      }
      break;

    case GET_TLE: 
      if ((millis() - TLEtimeOut) > 4000)  //wait 2s to ingest the TLE
      {
        calStatus = prevCalStatus;  //go back to previous state
      }
      break;

    case CALIB_OK:
      sat.findsat(jTimeNow);  //will update following sat. variables. Pass a double to allow ms accuracy
      //  satLat  //Latidude satellite (degrees)
      //  satLon //longitude satellite (degrees)
      //  satAlt  //Altitude satellite (degrees)
      //  satAz  //Azimuth satellite (degrees)
      //  satEl //elevation satellite (degrees)
      //  satDist  //Distance to satellite (km)
      //  satJd  //time (julian day)
      satAZsteps = round(sat.satAz * oneTurn / 360);  //Convert degrees to stepper steps
      satELsteps = round(sat.satEl * oneTurn / 360);
#ifdef xDEBUG
      invjday(sat.satJd, TimeZone, true, years, months, days, hours, minutes, seconds);
      Serial.println("\nLocal time: " + String(days) + '/' + String(months) + '/' + String(years) + ' ' + String(hours) + ':' + String(minutes) + ':' + String(seconds));
      Serial.println("azimuth = " + String(sat.satAz) + " elevation = " + String(sat.satEl) + " distance = " + String(sat.satDist));
      Serial.println("latitude = " + String(sat.satLat) + " longitude = " + String(sat.satLon) + " altitude = " + String(sat.satAlt));
#endif


      while (true) {
        if (nextpassEpoch - timeNow < 60 && nextpassEpoch + 5 - timeNow > 0)  //one minute before pass (and less than 5s after start)
        {
          tracking = false;
          prepass();
          break;
        }
        if ((sat.satVis != -2) && (passStatus == 1))  //satellite visible and tracking
        {
          tracking = true;
          inPass();
          passEnd = timeNow;
          nbLoops++;
          break;
        } else tracking = false;
        if (timeNow - passEnd < 59)  //post pass during 1 minute
        {
          tracking = false;
          postpass();
          break;
        }
        if (timeNow - passEnd < 60)  //end pass during 1 s
        {
          tracking = false;
          endpass();
          break;
        }
        if (sat.satVis == -2)  //stand by 1 min after pass and sat not visible
        {
          tracking = false;
          standby();
          break;
        }
      }

      // Update TLE & Unix time everyday.//
      time_t now;
      struct tm* timeinfo;
      time(&now);
      timeinfo = localtime(&now);
      nowDay = timeinfo->tm_mday;


      if (passStatus == 0 && nowDay != TLEday) {
        if ((WiFi.status() == WL_CONNECTED) && (timeinfo->tm_hour == 3)) {
          ESP.restart();  //@3 am next day, if no pass then ESP will retart to refresh time and TLEs
        }
      }
      break;
  }
}


void standby() {
  digitalWrite(ENABLE_PIN, HIGH);  //disable

  if ((millis() - lastDebug) > 30000) {
    lastDebug = millis();
#ifdef DEBUG
    long int AZCurrent = AZstepper.currentPosition();
    long int ELCurrent = ELstepper.currentPosition();
    Serial.println("Standby : next satellite is : " + String(satnames[nextSat]) + " in: " + String(nextpassEpoch - timeNow));
#endif
  }
}

void prepass() {
  // Pass is less than 60 seconds away, move antenna to start location and wait.
  passStatus = 1;
  if ((AZstart < 360) && (AZstart > 180)) AZstart = AZstart - 360;  //Goes to start counter-clockwise if closer.
  if (AZstart < 0) posCW = false;
  else posCW = true;
  digitalWrite(ENABLE_PIN, LOW);  //enable
  AZstepper.runToNewPosition(AZstart * oneTurn / 360);
  ELstepper.runToNewPosition(0);
#ifdef DEBUG
  if ((millis() - lastDebug) > 10000) {
    lastDebug = millis();
    Serial.println("Pre-pass for : " + String(satnames[nextSat]) + " in : " + String(nextpassEpoch - timeNow) + " Az : " + String(AZstart));
  }
#endif
}

void inPass() {
  // Handle zero crossings
  if (AZstepper.currentPosition() < 0) AZstepper.setCurrentPosition(AZstepper.currentPosition() + oneTurn);
  if (AZstepper.currentPosition() > oneTurn) AZstepper.setCurrentPosition(AZstepper.currentPosition() - oneTurn);
  if ((AZstepper.currentPosition() > oneTurn * 3 / 4) && (satAZsteps < oneTurn / 4)) satAZsteps += oneTurn;                                                //zero cross CW
  if ((satAZsteps > oneTurn * 3 / 4) && (AZstepper.currentPosition() < oneTurn / 4)) AZstepper.setCurrentPosition(AZstepper.currentPosition() + oneTurn);  //zero cross CCW
  if (satAZsteps > AZstepper.currentPosition()) dirCW = true;
  if (satAZsteps < AZstepper.currentPosition()) dirCW = false;
  // Update stepper position
  digitalWrite(ENABLE_PIN, LOW);  //enable

  satAZsteps2 = satAZsteps;
  satELsteps2 = satELsteps;
  //  AZstepper.runToNewPosition(satAZsteps);
  //  ELstepper.runToNewPosition(satELsteps);
  //Serial.println(satAZsteps2);
  passStatus = 1;
#ifdef DEBUG_IN_PASS
  if ((millis() - lastDebug) > 5000) {
    lastDebug = millis();
    Serial.println("in pass : " + String(satnames[nextSat]));
    Serial.println("azimuth = " + String(sat.satAz) + " elevation = " + String(sat.satEl) + " distance = " + String(sat.satDist));
    Serial.println("latitude = " + String(sat.satLat) + " longitude = " + String(sat.satLon) + " altitude = " + String(sat.satAlt));
  }
#endif
}

void postpass() {
#ifdef DEBUG
  if ((millis() - lastDebug) > 5000) {
    lastDebug = millis();
    Serial.println("Post pass time left: " + String(passEnd + 60 - timeNow));
  }
#endif
  if (timeNow - passEnd < 10)  //this is needed to stop the multisteppers at the current postion
  {
    AZstepper.setCurrentPosition(AZstepper.currentPosition());
    ELstepper.setCurrentPosition(ELstepper.currentPosition());
  }
  if (timeNow - passEnd > 20) {
    digitalWrite(ENABLE_PIN, LOW);  //enable
    int i = 0;
#ifdef xDEBUG_END_PASS
    Serial.print(String(AZstepper.currentPosition()) + " " + String(posCW) + " " + String(dirCW));
#endif
    //compute way back motion to zero depending on initial position CW or CCW and further direction of motion dir CW or dir CCW
    if (posCW == false)  //position CCW
    {
      if (dirCW == true)  //motion CW
      {
        if (AZstepper.currentPosition() > oneTurn / 2) AZstepper.setCurrentPosition(AZstepper.currentPosition() - oneTurn);  //now will go to zero CW
        i = 0;
      } else i = oneTurn;  //motion CCW
    } else                 //position CW ==true
    {
      if (dirCW == true)  //motion CW
      {
        i = 0;
      } else  //motion CCW
      {
        if (AZstepper.currentPosition() > oneTurn / 2) AZstepper.setCurrentPosition(AZstepper.currentPosition() - oneTurn);  //now will go to zero CW
        i = 0;
      }
    }
#ifdef xDEBUG_END_PASS
    Serial.println(" " + String(i) + " " + String(AZstepper.currentPosition()));
#endif
    AZstepper.runToNewPosition(i);
    ELstepper.runToNewPosition(0);  //Standby at 0 degrees above horizon
  }
}

void endpass() {
#ifdef DEBUG_END_PASS
  Serial.println("end of pass");
#endif
  delayNext = 0;                   //goes back to real time
  timeNow = unix_time_get_time();  //update time
  updateSatellites();
  AZstepper.setCurrentPosition(0);
  passStatus = 0;
  tracking = false;
  delay(1000);
}

void updateSatellites() {
  if (calStatus != GET_TLE) prevCalStatus = calStatus;  //then first TLE has been sent, more will come
  calStatus = GET_TLE;
  TLEtimeOut = millis();  //reset the TLE timeout used to escape from GET_TLE state
  for (SAT = 0; SAT < nbSat; SAT++) {
    sat.init(satname, TLE1[SAT], TLE2[SAT]);
    sat.findsat(timeNow);
    upcomingPasses[SAT] = Predict(1);
#ifdef DEBUG_UPDATE_SAT
    Serial.println("Next pass for Satellite #: " + String(SAT) + " in: " + String(upcomingPasses[SAT] - timeNow));
#endif
  }
  nextSat = nextSatPass(upcomingPasses);
  sat.init(satname, TLE1[nextSat], TLE2[nextSat]);
  Predict(1);
}

void getTLE(int SAT) {
  // Make HTTP request and get TLE for satellite//
  if (client.connect(server, 80)) {
#ifdef DEBUG
    Serial.println("connected to server");
    Serial.println("Request #: " + String(SAT) + " For: " + String(satnames[SAT]));
#endif
    // Make a HTTP request: //
    client.println("GET " + startURL + String(satID[SAT]) + "&FORMAT=TLE HTTP/1.1");
    client.println("Accept: */*");
    client.println("Host: celestrak.org");
    client.println("User-Agent: ESP32");
    client.println("Connection: close");
    client.println();
  }
  int connectLoop = 0;
  while (client.connected()) {
    int j = 0;
    while (client.available()) {
      char c = client.read();
      TLE[j] = c;  //store characters to string
      j++;
      connectLoop = 0;
    }
    k = 0;
    for (j = 233; j < 256; j++) {  //TLE line 1 spans characters 232 - 256
      satnames[SAT][k] = TLE[j];
      k++;
    }
    k = 0;
    for (j = 259; j < 328; j++) {  //TLE line 1 spans characters 257 - 327
      TLE1[SAT][k] = TLE[j];
      k++;
    }
    k = 0;
    for (j = 330; j < 399; j++) {  //TLE line 2 spans characters 328 - 398
      TLE2[SAT][k] = TLE[j];
      k++;
    }
    noTLE = false;
    connectLoop++;
    delay(100);
    if (connectLoop > 100) {
      client.stop();
      break;
    }
  }
}
// Adapted from sgp4 library
// Predicts time of next pass and start azimuth for satellites
long Predict(int many) {
  passinfo overpass;                //structure to store overpass info
  sat.initpredpoint(timeNow, 0.0);  //finds the startpoint
  bool error;
  for (int i = 0; i < many; i++) {
    error = sat.nextpass(&overpass, 10);  //search for the next overpass, if there are more than 20 maximums below the horizon it returns false
    delay(0);

    if (error == 1) {                                          //no error, prints overpass information
      nextpassEpoch = (overpass.jdstart - 2440587.5) * 86400;  //2440587.5 is the julian day at 1/1/1970 0:00 UTC
      AZstart = overpass.azstart;
      invjday(overpass.jdstart, TimeZone, true, years, months, days, hours, minutes, seconds);  // Convert Julian date to print in serial.
#ifdef DEBUG

      Serial.println("Next pass for: " + String(satnames[SAT]) + " In: " + String(nextpassEpoch - timeNow));
      Serial.println("Start: az=" + String(overpass.azstart) + "° " + String(hours) + ':' + String(minutes) + ':' + String(seconds));

#endif
    } else {
#ifdef DEBUG
      Serial.println("Prediction error");
#endif
    }
    delay(0);
  }
  return nextpassEpoch;
}
