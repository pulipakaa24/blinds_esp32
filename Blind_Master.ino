#include <WiFi.h>
#include <Streaming.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <SocketIOclient.h>
#include <driver/gptimer.h>

#define ccwSpeed 6500
#define cwSpeed 3300
#define offSpeed 4900

#define ccwMax 10
#define cwMax 0

#define getMovingCW(port) ((movingCW & (1 << port)) >> port)
#define setMovingCW(port) (movingCW |= (1 << port))
#define clearMovingCW(port) (movingCW &= ~(1 << port))
#define getMovingCCW(port) ((movingCCW & (1 << port)) >> port)
#define setMovingCCW(port) (movingCCW |= (1 << port))
#define clearMovingCCW(port) (movingCCW &= ~(1 << port))
#define getCalibCW(port) ((calibCW & (1 << port)) >> port)
#define setCalibCW(port) (calibCW |= (1 << port))
#define clearCalibCW(port) (calibCW &= ~(1 << port))
#define getCalibCCW(port) ((calibCCW & (1 << port)) >> port)
#define setCalibCCW(port) (calibCCW |= (1 << port))
#define clearCalibCCW(port) (calibCCW &= ~(1 << port))
#define getCalibDone(port) ((calibDone & (1 << port)) >> port)
#define setCalibDone(port) (calibDone |= (1 << port))
#define clearCalibDone(port) (calibDone &= ~(1 << port))
#define getBlocked(port) ((blocked & (1 << port)) >> port)
#define setBlocked(port) (blocked |= (1 << port))
#define clearBlocked(port) (blocked &= ~(1 << port))
#define getPos10(port) ((movingCCW & (1 << (port + 4))) >> (port + 4))
#define setPos10(port) (movingCCW |= (1 << (port + 4)))
#define clearPos10(port) (movingCCW &= ~(1 << (port + 4)))
#define getPos0(port) ((movingCW & (1 << (port + 4))) >> (port + 4))
#define setPos0(port) (movingCW |= (1 << (port + 4)))
#define clearPos0(port) (movingCW &= ~(1 << (port + 4)))

#define prefNameCalibs "periph_info_"

const uint8_t portToServo[4] = {15, 12, 13, 26};
const uint8_t portTo0Button[4] = {2, 36, 34, 14};
const uint8_t portTo1Button[4] = {4, 39, 35, 25};

volatile uint8_t calibCCW = 0;
volatile uint8_t calibCW = 0;
uint8_t targetPos[4] = {0, 0, 0, 0};
volatile uint8_t movingCCW = 0;
volatile uint8_t movingCW = 0;
volatile uint32_t cwTimes[4] = {0,0,0,0};
volatile uint32_t ccwTimes[4] = {0,0,0,0};
volatile uint32_t totalTimes[4] = {0,0,0,0};
volatile uint8_t calibDone = 0;
uint8_t blocked = 0;

gptimer_handle_t portTimers[4] = {NULL, NULL, NULL, NULL};
gptimer_config_t timer_config = {
    .clk_src = GPTIMER_CLK_SRC_DEFAULT,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 10000, // 10KHz, 1 tick = 1ms
};

portMUX_TYPE sharedData = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR portStop(uint8_t portNum) {
  gptimer_stop(portTimers[portNum]);
  portENTER_CRITICAL_ISR(&sharedData);
  ledcWrite(portToServo[portNum], offSpeed);
  clearMovingCW(portNum);
  clearMovingCCW(portNum);
  portEXIT_CRITICAL_ISR(&sharedData);
}

void IRAM_ATTR portStopButton(uint8_t portNum) {
  portENTER_CRITICAL_ISR(&sharedData);
  ledcWrite(portToServo[portNum], offSpeed);
  clearMovingCW(portNum);
  clearMovingCCW(portNum);
  portEXIT_CRITICAL_ISR(&sharedData);
}

static IRAM_ATTR bool port0Stop(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
  BaseType_t high_task_awoken = pdFALSE;
  portStop(0);
  return high_task_awoken == pdTRUE;
}
static IRAM_ATTR bool port1Stop(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
  BaseType_t high_task_awoken = pdFALSE;
  portStop(1);
  return high_task_awoken == pdTRUE;
}
static IRAM_ATTR bool port2Stop(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
  BaseType_t high_task_awoken = pdFALSE;
  portStop(2);
  return high_task_awoken == pdTRUE;
}
static IRAM_ATTR bool port3Stop(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
  BaseType_t high_task_awoken = pdFALSE;
  portStop(3);
  return high_task_awoken == pdTRUE;
}

void IRAM_ATTR port0StopButton() {portStopButton(0);}
void IRAM_ATTR port1StopButton() {portStopButton(1);}
void IRAM_ATTR port2StopButton() {portStopButton(2);}
void IRAM_ATTR port3StopButton() {portStopButton(3);}

typedef bool (* GptimerEventCallback_t)(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data);

GptimerEventCallback_t portStops[] = {
  &port0Stop,
  &port1Stop,
  &port2Stop,
  &port3Stop
};

void (*portStopsButton[])(void) = {
  &port0StopButton, // Take the address of the function
  &port1StopButton,
  &port2StopButton,
  &port3StopButton
};

void IRAM_ATTR portSwitchCalib(uint8_t portNum) {
  portENTER_CRITICAL_ISR(&sharedData);
  if (!getCalibCW(portNum)) {
    ledcWrite(portToServo[portNum], cwSpeed);
    ESP_ERROR_CHECK(gptimer_stop(portTimers[portNum]));
    uint64_t count;
    ESP_ERROR_CHECK(gptimer_get_raw_count(portTimers[portNum], &count));
    ccwTimes[portNum] = count;
    setCalibCW(portNum);
  }
  portEXIT_CRITICAL_ISR(&sharedData);
}

void IRAM_ATTR port0SwitchCalib() {portSwitchCalib(0);}
void IRAM_ATTR port1SwitchCalib() {portSwitchCalib(1);}
void IRAM_ATTR port2SwitchCalib() {portSwitchCalib(2);}
void IRAM_ATTR port3SwitchCalib() {portSwitchCalib(3);}

void (*portSwitchCalibs[])(void) = {
  &port0SwitchCalib, // Take the address of the function
  &port1SwitchCalib,
  &port2SwitchCalib,
  &port3SwitchCalib
};

void IRAM_ATTR portEndCalib(uint8_t portNum) {
  portENTER_CRITICAL_ISR(&sharedData);
  if (!getCalibDone(portNum)) {
    ledcWrite(portToServo[portNum], offSpeed);
    ESP_ERROR_CHECK(gptimer_stop(portTimers[portNum]));
    uint64_t count;
    ESP_ERROR_CHECK(gptimer_get_raw_count(portTimers[portNum], &count));
    cwTimes[portNum] = count;
    setCalibDone(portNum);
  }
  portEXIT_CRITICAL_ISR(&sharedData);
}

void IRAM_ATTR port0EndCalib() {portEndCalib(0);}
void IRAM_ATTR port1EndCalib() {portEndCalib(1);}
void IRAM_ATTR port2EndCalib() {portEndCalib(2);}
void IRAM_ATTR port3EndCalib() {portEndCalib(3);}

void (*portEndCalibs[])(void) = {
  &port0EndCalib, // Take the address of the function
  &port1EndCalib,
  &port2EndCalib,
  &port3EndCalib
};

void IRAM_ATTR hitPos0(uint8_t portNum) {
  portENTER_CRITICAL_ISR(&sharedData);
  if (!getPos0(portNum)) {
    ledcWrite(portToServo[portNum], offSpeed);
    setPos0(portNum);
  }
  portEXIT_CRITICAL_ISR(&sharedData);
}

void IRAM_ATTR port0hit0() {hitPos0(0);}
void IRAM_ATTR port1hit0() {hitPos0(1);}
void IRAM_ATTR port2hit0() {hitPos0(2);}
void IRAM_ATTR port3hit0() {hitPos0(3);}

void (*portsHit0[])(void) = {
  &port0hit0, // Take the address of the function
  &port1hit0,
  &port2hit0,
  &port3hit0
};

void IRAM_ATTR hitPos10(uint8_t portNum) {
  portENTER_CRITICAL_ISR(&sharedData);
  if (!getPos10(portNum)) {
    ledcWrite(portToServo[portNum], offSpeed);
    setPos10(portNum);
  }
  portEXIT_CRITICAL_ISR(&sharedData);
}

void IRAM_ATTR port0hit10() {hitPos10(0);}
void IRAM_ATTR port1hit10() {hitPos10(1);}
void IRAM_ATTR port2hit10() {hitPos10(2);}
void IRAM_ATTR port3hit10() {hitPos10(3);}

void (*portsHit10[])(void) = {
  &port0hit10, // Take the address of the function
  &port1hit10,
  &port2hit10,
  &port3hit10
};

Preferences prefs;
SocketIOclient socketIO;

String ssid = "";
String password = "";
String token = "";
const String socketUrl = "/socket.io/?EIO=4";
const char* ntpServer = "pool.ntp.org";
bool ssidgiven = false;
bool passgiven = false;
bool tokengiven = false;
bool connecting = false;
bool socketIoHandshakeDone = false;
BLECharacteristic *ssidChar;
BLECharacteristic *passChar;
BLECharacteristic *ssidListChar;
BLECharacteristic *tokenChar;
BLECharacteristic *ssidRefreshChar;
BLECharacteristic *connectConfirmChar;
HTTPClient http;
bool restart = false;


const String address = "192.168.1.190";
const uint16_t port = 3000;
const String path = "/socket.io/?EIO=4&transport=websocket";
const String httpSrv = "http://192.168.1.190:3000/";

void socketIOEvent(socketIOmessageType_t type, uint8_t * payload, size_t length);
String getEncryption(wifi_auth_mode_t type) {
  switch (type) {
    case WIFI_AUTH_OPEN:
      return "OPEN";
    case WIFI_AUTH_WEP:
    case WIFI_AUTH_WPA_PSK:
    case WIFI_AUTH_WPA2_PSK:
    case WIFI_AUTH_WPA_WPA2_PSK:
    case WIFI_AUTH_WPA3_PSK:
      return "SECURED";
    default:
      // These include WPA2/WPA3-Enterprise and others requiring user/password or certs
      return "UNSUPPORTED";
  }
}

void scanAndUpdateSSIDList() {
  int n = WiFi.scanNetworks();
  String ssidList = "";

  if (n == 0) ssidList = ";";

  else for (int i = 0; i < n; ++i) {
    ssidList += WiFi.SSID(i) + "," + getEncryption(WiFi.encryptionType(i));
    if (i < n - 1) ssidList += ";";  // comma-delimited
  }

  ssidListChar->setValue(ssidList.c_str());
  Serial.println("Updated SSID list: " + ssidList);
}

class MyCallbacks;
class MyServerCallbacks;

void saveWiFiCreds(const String& ssid, const String& password) {
  prefs.begin("wifi"); // false = read/write
  prefs.putString("ssid", ssid);
  if (password != "") prefs.putString("password", password);
  prefs.end();
}

void saveToken(const String& token) {
  prefs.begin("auth", false);
  prefs.putString("token", token);
  prefs.end();
}

void saveCalibTimes(const uint8_t& portNum) {
  String prefName = prefNameCalibs + String(portNum);
  prefs.begin(prefName.c_str());
  portENTER_CRITICAL(&sharedData);
  uint32_t cwTime = cwTimes[portNum];
  uint32_t ccwTime = ccwTimes[portNum];
  portEXIT_CRITICAL(&sharedData);
  Serial.printf("CwTime: %d", cwTime);
  Serial.printf("CcwTime: %d", ccwTime);
  prefs.putULong("cwTime", cwTime);
  prefs.putULong("ccwTime", ccwTime);
  if (!prefs.getBool("calibrated")) prefs.putBool("calibrated", true);
  if (prefs.getUChar("pos") != 0) prefs.putUChar("pos", 0);
  prefs.end();
  detachInterrupt(portTo1Button[portNum]);
  portENTER_CRITICAL(&sharedData);
  clearCalibDone(portNum);
  portEXIT_CRITICAL(&sharedData);
}

void beginCalib(const uint8_t& portNum) {
  prefs.begin((prefNameCalibs + String(portNum)).c_str());
  if (prefs.getBool("calibrated")) prefs.putBool("calibrated", false);
  prefs.end();
  Serial.printf("beginCalib%d\n", portNum);
  ledcWrite(portToServo[portNum], ccwSpeed);
  ESP_ERROR_CHECK(gptimer_disable(portTimers[portNum]));
  ESP_ERROR_CHECK(gptimer_del_timer(portTimers[portNum]));
  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &portTimers[portNum]));
  gptimer_event_callbacks_t cbs = {
    .on_alarm = portStops[portNum],
  };
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(portTimers[portNum], &cbs, NULL));
  ESP_ERROR_CHECK(gptimer_enable(portTimers[portNum]));
  ESP_ERROR_CHECK(gptimer_set_raw_count(portTimers[portNum], 0));
  ESP_ERROR_CHECK(gptimer_start(portTimers[portNum]));
  attachInterrupt(portTo0Button[portNum], portSwitchCalibs[portNum], RISING);
  portENTER_CRITICAL(&sharedData);
  clearCalibCCW(portNum);
  portEXIT_CRITICAL(&sharedData);
}

void switchCalib(const uint8_t& portNum) {
  detachInterrupt(portTo0Button[portNum]);
  ledcWrite(portToServo[portNum], cwSpeed);
  ESP_ERROR_CHECK(gptimer_set_raw_count(portTimers[portNum], 0));
  ESP_ERROR_CHECK(gptimer_start(portTimers[portNum]));
  attachInterrupt(portTo1Button[portNum], portEndCalibs[portNum], RISING);
  portENTER_CRITICAL(&sharedData);
  clearCalibCW(portNum);
  portEXIT_CRITICAL(&sharedData);
}

void cancelCalib(const uint8_t& portNum) {
  detachInterrupt(portTo0Button[portNum]);
  detachInterrupt(portTo1Button[portNum]);
  ledcWrite(portToServo[portNum], offSpeed);
  portENTER_CRITICAL(&sharedData);
  clearCalibCCW(portNum);
  clearCalibCW(portNum);
  clearCalibDone(portNum);
  cwTimes[portNum] = 0;
  ccwTimes[portNum] = 0;
  portEXIT_CRITICAL(&sharedData);
  ESP_ERROR_CHECK(gptimer_stop(portTimers[portNum]));
}

bool getCalibStatus(const uint8_t& portNum) {
  prefs.begin((prefNameCalibs + String(portNum)).c_str());
  bool retVal = prefs.getBool("calibrated");
  prefs.end();
  return retVal;
}

uint32_t getCWTime(const uint8_t& portNum) {
  prefs.begin((prefNameCalibs + String(portNum)).c_str());
  uint32_t retVal = prefs.getULong("cwTime");
  prefs.end();
  return retVal;
}

uint32_t getCCWTime(const uint8_t& portNum) {
  prefs.begin((prefNameCalibs + String(portNum)).c_str());
  uint32_t retVal = prefs.getULong("ccwTime");
  prefs.end();
  return retVal;
}

void clearNamespace(const char* nameSpace) {
  prefs.begin(nameSpace);
  prefs.clear();
  prefs.end();
}

void clearStorage() {
  clearNamespace("wifi");
  clearNamespace("auth");
  for (uint8_t i = 0; i < 4; i++) {
    String prefName = prefNameCalibs + String(i);
    clearNamespace(prefName.c_str());
  }
}

uint8_t getPos(const uint8_t& portNum) {
  prefs.begin((prefNameCalibs + String(portNum)).c_str());
  uint8_t prevPos = prefs.getUChar("pos");
  prefs.end();
  return prevPos;
}

void putPos(const uint8_t& portNum, const uint8_t& pos) {
  prefs.begin((prefNameCalibs + String(portNum)).c_str());
  prefs.putUChar("pos", pos);
  prefs.end();
}

uint32_t posUpdateAndGetTurnTime(const uint8_t& portNum, const uint8_t& newPos) {
  prefs.begin((prefNameCalibs + String(portNum)).c_str());
  uint8_t prevPos = prefs.getUChar("pos");
  if (newPos != prevPos) prefs.putUChar("pos", newPos);
  prefs.end();

  uint32_t timeToTurn = 0;
  uint32_t remainingTime;
  uint64_t readMillis;
  gptimer_get_raw_count(portTimers[portNum], &readMillis);
  portENTER_CRITICAL(&sharedData);
  remainingTime = (totalTimes[portNum] > readMillis) ? totalTimes[portNum] - (uint32_t)readMillis : 0;
  portEXIT_CRITICAL(&sharedData);
  uint32_t cwTime = getCWTime(portNum);
  uint32_t ccwTime = getCCWTime(portNum);

  if (newPos < prevPos) {
    timeToTurn = ((prevPos - newPos) * cwTime) / 10;
    Serial.printf("TimeToTurn: %d\n", timeToTurn);
    portENTER_CRITICAL(&sharedData);
    if (getMovingCW(portNum)) {
      portEXIT_CRITICAL(&sharedData);
      gptimer_stop(portTimers[portNum]);
      timeToTurn += remainingTime;
    } else if (getMovingCCW(portNum)) {
      portEXIT_CRITICAL(&sharedData);
      gptimer_stop(portTimers[portNum]);
      double result = ((double)cwTime / (double) ccwTime) * (double) remainingTime;
      uint32_t intRes = result;
      if (intRes > timeToTurn) {
        result = ((double)ccwTime / (double)cwTime) * (double) timeToTurn;
        timeToTurn = remainingTime - (uint32_t)result;
      } else {
        timeToTurn -= intRes;
        portENTER_CRITICAL(&sharedData);
        clearMovingCCW(portNum);
        setMovingCW(portNum);
        portEXIT_CRITICAL(&sharedData);
      }
    } else {
      setMovingCW(portNum);
      portEXIT_CRITICAL(&sharedData);
    }
  }
  else if (newPos > prevPos) {
    timeToTurn = ((newPos - prevPos) * ccwTime) / 10;
    portENTER_CRITICAL(&sharedData);
    if (getMovingCCW(portNum)) {
      portEXIT_CRITICAL(&sharedData);
      gptimer_stop(portTimers[portNum]);
      timeToTurn += remainingTime;
    } else if (getMovingCW(portNum)) {
      portEXIT_CRITICAL(&sharedData);
      gptimer_stop(portTimers[portNum]);
      double result = ((double)ccwTime / (double)cwTime) * (double)remainingTime;
      uint32_t intRes = result;
      if (intRes > timeToTurn) {
        result = ((double)cwTime / (double)ccwTime) * (double)timeToTurn;
        timeToTurn = remainingTime - (uint32_t)result;
      } else {
        timeToTurn -= intRes;
        portENTER_CRITICAL(&sharedData);
        clearMovingCW(portNum);
        setMovingCCW(portNum);
        portEXIT_CRITICAL(&sharedData);
      }
    } else {
      setMovingCCW(portNum);
      portEXIT_CRITICAL(&sharedData);
    }
  }
  else timeToTurn = 0;

  return timeToTurn;
}

void notifyServerCalibrated(const uint8_t& port) {
  DynamicJsonDocument doc(256);
  JsonArray array = doc.to<JsonArray>();

  array.add("calib_done");
  JsonObject param1 = array.createNestedObject();
  param1["port"] = port + 1;
  String toSend;
  serializeJson(doc, toSend);
  socketIO.sendEVENT(toSend);
}

void notifyPos(const uint8_t& port, const uint8_t& pos) {
  DynamicJsonDocument doc(256);
  JsonArray array = doc.to<JsonArray>();

  array.add("pos_hit");
  JsonObject param1 = array.createNestedObject();
  param1["port"] = port + 1;
  param1["pos"] = pos;
  String toSend;
  serializeJson(doc, toSend);
  socketIO.sendEVENT(toSend);
}

void initSetup();
void connect();
void initializePeriphs();

void setup() {
  Serial.begin(9600);
  // while(!Serial);
  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH);

  connect();

  configTime(0, 0, ntpServer);

  Serial.println("Waiting for NTP time synchronization...");
  struct tm timeinfo;
  uint8_t retryCount = 0;
  // Wait for time to be set (it might take a few seconds)
  while (!getLocalTime(&timeinfo) && retryCount < 10) {
    Serial.print(".");
    delay(1000);
    retryCount++;
  }

  if (retryCount >= 10) {
    Serial.println("\nFailed to obtain time from NTP server after multiple retries!");
  } else {
    Serial.println("\nTime synchronized successfully!");
  }

  for (uint8_t i = 0; i < 4; i++) {
    ledcAttach(portToServo[i], 50, 16);
    ledcWrite(portToServo[i], offSpeed);
  }
  for (uint8_t i = 0; i < 4; i++) pinMode(portTo0Button[i], INPUT);
  for (uint8_t i = 0; i < 4; i++) pinMode(portTo1Button[i], INPUT);
  for (uint8_t i = 0; i < 4; i++) {
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &portTimers[i]));
    gptimer_event_callbacks_t cbs = {
      .on_alarm = portStops[i],
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(portTimers[i], &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(portTimers[i]));
  }

  String auth = String("Bearer ") + token;
  
  // server address, port and URL
  socketIO.begin(address, port, socketUrl, true, auth);

  // event handler
  socketIO.onEvent(socketIOEvent);

  initializePeriphs();
}

void loop() {
  socketIO.loop();
  for (uint8_t i = 0; i < 4; i++) {
    portENTER_CRITICAL(&sharedData);
    if (getCalibCCW(i)) {
      portEXIT_CRITICAL(&sharedData);
      beginCalib(i);
    } else if (getCalibCW(i)) {
      portEXIT_CRITICAL(&sharedData);
      switchCalib(i);
    } else if (getCalibDone(i)) {
      portEXIT_CRITICAL(&sharedData);
      saveCalibTimes(i);
      notifyServerCalibrated(i);
    } else if (getPos0(i)) {
      clearPos0(i);
      clearMovingCW(i);
      portEXIT_CRITICAL(&sharedData);
      gptimer_stop(portTimers[i]);
      if (!(getPos(i) == 0)) putPos(i, 0);
      notifyPos(i, 0);
      detachInterrupt(portTo1Button[i]);
    } else if (getPos10(i)) {
      clearPos10(i);
      clearMovingCCW(i);
      portEXIT_CRITICAL(&sharedData);
      gptimer_stop(portTimers[i]);
      if (!(getPos(i) == 10)) putPos(i, 10);
      notifyPos(i, 10);
      detachInterrupt(portTo0Button[i]);
    }
    else {
      portEXIT_CRITICAL(&sharedData);
      if (getCalibStatus(i)) {
        portENTER_CRITICAL(&sharedData);
        if (getBlocked(i) && !getMovingCW(i) && !getMovingCCW(i)) clearBlocked(i);
        portEXIT_CRITICAL(&sharedData);

        if (targetPos[i] != 0 && targetPos[i] != 10) {
          if (!getBlocked(i)) {
            uint32_t timeToTurn = posUpdateAndGetTurnTime(i, targetPos[i]);
            if (timeToTurn != 0) {
              portENTER_CRITICAL(&sharedData);
              totalTimes[i] = timeToTurn;
              bool movingcw = getMovingCW(i);
              bool movingccw = getMovingCCW(i);
              portEXIT_CRITICAL(&sharedData);
              if (movingcw) {
                ledcWrite(portToServo[i], cwSpeed);
                attachInterrupt(portTo1Button[i], portsHit0[i], RISING);
              }
              else if (movingccw) {
                ledcWrite(portToServo[i], ccwSpeed);
                attachInterrupt(portTo0Button[i], portsHit10[i], RISING);
              }
              gptimer_set_raw_count(portTimers[i], 0);
              gptimer_alarm_config_t alarm_config = {
                .alarm_count = timeToTurn,
                .flags = {
                  .auto_reload_on_alarm = false,
                }
              };
              ESP_ERROR_CHECK(gptimer_set_alarm_action(portTimers[i], &alarm_config));
              ESP_ERROR_CHECK(gptimer_start(portTimers[i]));
            }
          }
        } else if (!(targetPos[i] == getPos(i))){
          setBlocked(i);
          putPos(i, targetPos[i]);
          if (targetPos[i] == 0) {
            ledcWrite(portToServo[i], cwSpeed);
            gptimer_stop(portTimers[i]);
            portENTER_CRITICAL(&sharedData);
            setMovingCW(i);
            clearMovingCCW(i);
            portEXIT_CRITICAL(&sharedData);
            attachInterrupt(portTo1Button[i], portStopsButton[i], RISING);
          } 
          else {
            ledcWrite(portToServo[i], ccwSpeed);
            gptimer_stop(portTimers[i]);
            portENTER_CRITICAL(&sharedData);
            setMovingCCW(i);
            clearMovingCW(i);
            portEXIT_CRITICAL(&sharedData);
            attachInterrupt(portTo0Button[i], portStopsButton[i], RISING);
          }
        }
      }
    }
  }
}

void initializePeriphs() {
  if (http.begin(httpSrv + "position")) {
    http.addHeader("Authorization", "Bearer " + token);
    int httpCode = http.GET();
    if (httpCode > 0) {
      String payload = http.getString();
      StaticJsonDocument<256> doc;
      DeserializationError error = deserializeJson(doc, payload);

      if (error) {
        Serial.println("Error getting details");
      }

      JsonArray docArray = doc.as<JsonArray>();
      for (JsonObject periph : docArray) {
        uint8_t periphNum = periph["peripheral_number"].as<uint8_t>() - 1;
        if (periph["await_calib"].as<bool>()) {
          portENTER_CRITICAL(&sharedData);
          setCalibCCW(periphNum);
          clearCalibCW(periphNum);
          clearCalibDone(periphNum);
          portEXIT_CRITICAL(&sharedData);
        }
        targetPos[periphNum] = periph["last_pos"].as<uint8_t>();
      }
    }
  }
  http.end();
}

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    // Code to execute when a client connects
    Serial.println("Client connected");
  }

  void onDisconnect(BLEServer* pServer) {
    // Code to execute when a client disconnects
    Serial.println("Client disconnected");
    restart = true;
    // For example, restart advertising
    if (connecting) pServer->startAdvertising();
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    if (pChar == ssidChar) {
      ssid = pChar->getValue().c_str();
      Serial.println("SSID received: " + ssid);
      ssidgiven = true;
    } else if (pChar == passChar) {
      password = pChar->getValue().c_str();
      Serial.println("Password received: " + password);
      passgiven = true;
    }
    else if (pChar == tokenChar) {
      token = pChar->getValue().c_str();
      Serial.println("Token received: " + token);
      tokengiven = true;
    }
    else if (pChar == ssidRefreshChar) {
      Serial.println("Wifi Refresh");
      restart = true;
      scanAndUpdateSSIDList();
      ssidListChar->notify();
    }
  }
};

void socketIOEvent(socketIOmessageType_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case sIOtype_DISCONNECT:
            Serial.printf("[IOc] Disconnected!\n");
            break;
        case sIOtype_CONNECT:
            Serial.printf("[IOc] Connected to url: %s\n", payload);

            // join default namespace (no auto join in Socket.IO V3)
            socketIO.send(sIOtype_CONNECT, "/");
            break;
        case sIOtype_EVENT:
        {
            char * sptr = NULL;
            int id = strtol((char *)payload, &sptr, 10);
            Serial.printf("[IOc] get event: %s id: %d\n", payload, id);
            if(id) {
                payload = (uint8_t *)sptr;
            }
            DynamicJsonDocument doc(1024);
            DeserializationError error = deserializeJson(doc, payload, length);
            if(error) {
                Serial.print(F("deserializeJson() failed: "));
                Serial.printf(error.c_str());
                return;
            }

            String eventName = doc[0];
            Serial.printf("[IOc] event name: %s\n", eventName.c_str());

            if (eventName == "posUpdates") {
              JsonArray updatesList = doc[1].as<JsonArray>();
              for (JsonVariant update : updatesList) {
                if (update.is<JsonObject>()) {
                  JsonObject updateJSON = update.as<JsonObject>();
                  uint8_t periphNum = updateJSON["periphNum"].as<uint8_t>() - 1;
                  uint8_t pos = updateJSON["pos"].as<uint8_t>();
                  Serial.printf("Peripheral: %d, Position: %d\n", periphNum, pos);
                  targetPos[periphNum] = pos;
                }
              }
            } else if (eventName == "calib") {
              JsonObject toCalib = doc[1].as<JsonObject>();
              uint8_t periphNum = toCalib["periphNum"].as<uint8_t>() - 1;
              portENTER_CRITICAL(&sharedData);
              if (!(getCalibCW(periphNum) || getCalibDone(periphNum))) setCalibCCW(periphNum);
              portEXIT_CRITICAL(&sharedData);
            } else if (eventName == "cancel_calib") {
              JsonObject toCancel = doc[1].as<JsonObject>();
              cancelCalib(toCancel["periphNum"].as<uint8_t>() - 1);
            }
        }
            break;
        case sIOtype_ACK:
            Serial.printf("[IOc] get ack: %u\n", length);
            break;
        case sIOtype_ERROR:
            Serial.printf("[IOc] get error: %u\n", length);
            break;
        case sIOtype_BINARY_EVENT:
            Serial.printf("[IOc] get binary: %u\n", length);
            break;
        case sIOtype_BINARY_ACK:
            Serial.printf("[IOc] get binary ack: %u\n", length);
            break;
    }
}

void initSetup() {
  BLEDevice::init("BlindMaster Device");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks);
  BLEService *pService = pServer->createService(BLEUUID((uint16_t)0x181C)); // custom service

  BLESecurity *pSecurity = new BLESecurity();
  pSecurity->setCapability(ESP_IO_CAP_NONE);
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_ONLY); // or ESP_LE_AUTH_BOND
  pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

  ssidListChar = pService->createCharacteristic("0000", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  ssidListChar->addDescriptor(new BLE2902());
  ssidRefreshChar = pService->createCharacteristic("0004", BLECharacteristic::PROPERTY_WRITE);
  ssidChar = pService->createCharacteristic("0001", BLECharacteristic::PROPERTY_WRITE);
  passChar = pService->createCharacteristic("0002", BLECharacteristic::PROPERTY_WRITE);
  connectConfirmChar = pService->createCharacteristic("0005", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  connectConfirmChar->addDescriptor(new BLE2902());
  tokenChar = pService->createCharacteristic("0003", BLECharacteristic::PROPERTY_WRITE);

  ssidChar->setCallbacks(new MyCallbacks());
  passChar->setCallbacks(new MyCallbacks());
  tokenChar->setCallbacks(new MyCallbacks());
  ssidRefreshChar->setCallbacks(new MyCallbacks());

  scanAndUpdateSSIDList();

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID("181C");
  pAdvertising->start();
  Serial.println("BLE service started.");

  while (1) {
    while (!(ssidgiven && passgiven)) delay(100);
    if (password == "") WiFi.begin(ssid.c_str());
    else WiFi.begin(ssid.c_str(), password.c_str());

    uint32_t startAttemptTime = millis();
    const uint32_t timeout = 10000;
    Serial.print("Connecting to WiFi");

    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout) {
      Serial.print(".");
      delay(500);
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial<<endl<<"Connected at "<<WiFi.localIP()<<endl;
      connectConfirmChar->setValue("Connected");
      connectConfirmChar->notify();
      connectConfirmChar->setValue("");  // Clear value after notify
      restart = false;
      saveWiFiCreds(ssid, password);
      while (!tokengiven) {
        delay(100);
        if (restart) {
          ssidgiven = false;
          passgiven = false;
          break;
        }
      }
      if (restart) continue;
      if (http.begin(httpSrv + "verify_device")) {
        http.addHeader("Authorization", "Bearer " + token);
        int httpCode = http.GET();
        if (httpCode > 0) {
          String payload = http.getString();
          StaticJsonDocument<256> doc;
          DeserializationError error = deserializeJson(doc, payload);
          if (error) {
            Serial.print("deserializeJson() failed: ");
            Serial.println(error.f_str());
            ssidgiven = false;
            passgiven = false;
            tokengiven = false;
          }
          token = doc["token"].as<String>();
          Serial.println(token);
          saveToken(token);
          break;
        }
        http.end();
      }
      
    } else {
      Serial<<endl<<"Timed out"<<endl;
      ssidgiven = false;
      passgiven = false;
      WiFi.disconnect(true);
    }
    WiFi.disconnect(true);
    connectConfirmChar->setValue("Error");
    connectConfirmChar->notify();
    connectConfirmChar->setValue("");  // Clear value after notify
  }
  pAdvertising->stop();
}

void connect() {
  connecting = true;
  prefs.begin("wifi", true); // true = read only
  String storedSSID = prefs.getString("ssid", "");
  String storedPass = prefs.getString("password", "");
  prefs.end();

  if (storedSSID.length() == 0 && storedPass.length() == 0) {
    initSetup();
  } else if (storedSSID.length() != 0 && storedPass.length() == 0) {
    WiFi.begin(storedSSID.c_str());
    uint32_t startAttemptTime = millis();
    const uint32_t timeout = 10000;
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout) {
      Serial.print(".");
      delay(500);
    }

    if (WiFi.status() != WL_CONNECTED) {
      initSetup();
      connect();
    }
  } else {
    WiFi.begin(storedSSID.c_str(), storedPass.c_str());
    uint32_t startAttemptTime = millis();
    const uint32_t timeout = 10000;
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout) {
      Serial.print(".");
      delay(500);
    }

    Serial.println(WiFi.status());

    if (WiFi.status() != WL_CONNECTED) {
      initSetup();
      connect();
    }
  } 

  connecting = false;
  prefs.begin("auth", true);
  token = prefs.getString("token", "");
  prefs.end();
}