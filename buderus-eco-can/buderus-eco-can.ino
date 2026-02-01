#include <Arduino.h>

#include <vector>
#include <map>

#include <SPI.h>     //Library for using SPI Communication
#include <mcp2515.h> //Library for using CAN Communication

#include <GTimer.h>
#include <WiFi.h>
#include <PubSubClient.h> //mqtt
#include <GSON.h>

#include <ArduinoOTA.h>
#include <NetworkUdp.h>

#define SPI_CC_PIN 7
#define TOPIC_PREFIX "Buderus/"

const char *wifi_ssid = "wifi ssid"; 
const char *wifi_password = "wifi pass";

const char *hostname = "esp32-buderus-can";

const char *OTA_password = "pass for OTA update";
uint32_t last_ota_time = 0;

const char *mqtt_client_id = hostname;
const char *mqtt_server = "IP or DNS name";
const char *mqtt_username = "mqtt username";
const char *mqtt_password = "mqtt pass";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
GTimer<millis> tmrCheckConnections(5000, true);
GTimer<millis> tmrUpdateAndUptime(60000, true);

struct can_frame canMsg;
MCP2515 mcp2515(SPI_CC_PIN);

//************************************************************************

enum sensorType {
  _temperature,
  _flag,
  _enumeration,
  _memory,
  _time
};

enum methodsOfReading {
  flag,
  bits,
  number,
  number_div2,
  number_signed
};

struct properties {
  String topic;
  sensorType type;
  methodsOfReading method;
  uint8_t mask = 0b11111111;
};

struct monitoringModule {
  String topic;
  std::map<uint8_t, std::vector<properties>> bytes;
};

char buf[100];

std::map<String, String> collectedData;
std::map<String, String>::iterator collectedData_iterator = collectedData.begin();
std::vector<uint8_t> unsupportedModules;

const std::map<uint8_t, std::vector<properties>> heatingCircuitBytes = {
  { 0,
    { { "Automatic", sensorType::_flag, methodsOfReading::flag, 0b00000100 },
      { "Manual", sensorType::_flag, methodsOfReading::flag, 0b10000000 },
      { "Byte1", sensorType::_enumeration, methodsOfReading::bits, 0b01111011 } } },
  { 1,
    { { "Summer", sensorType::_flag, methodsOfReading::flag, 0b00000001 },
      { "Day", sensorType::_flag, methodsOfReading::flag, 0b00000010 },
      { "Byte2", sensorType::_enumeration, methodsOfReading::bits, 0b11111100 } } },
  { 2,
    { { "Fluid target temperature", sensorType::_temperature, methodsOfReading::number } } },
  { 3,
    { { "Fluid actual temperature", sensorType::_temperature, methodsOfReading::number } } },
  { 4,
    { { "Room target temperature", sensorType::_temperature, methodsOfReading::number_div2 } } },
  { 5,
    { { "Room actual temperature", sensorType::_temperature, methodsOfReading::number_div2 } } }
};

const std::map<uint8_t, std::vector<properties>> hotWaterBytes = {
  { 0,
    { { "Automatic", sensorType::_flag, methodsOfReading::flag, 0b00000001 },
      { "Disinfection", sensorType::_flag, methodsOfReading::flag, 0b00000010 },
      { "Error", sensorType::_flag, methodsOfReading::flag, 0b11110000 },
      { "Byte1", sensorType::_enumeration, methodsOfReading::bits, 0b11111100 } } },
  { 1,
    { { "Manual", sensorType::_flag, methodsOfReading::flag, 0b00000010 },
      { "Day", sensorType::_flag, methodsOfReading::flag, 0b00100000 },
      { "Warm", sensorType::_flag, methodsOfReading::flag, 0b01000000 },
      { "Byte2", sensorType::_enumeration, methodsOfReading::bits, 0b10011101 } } },
  { 2,
    { { "Water target temperature", sensorType::_temperature, methodsOfReading::number } } },
  { 3,
    { { "Water actual temperature", sensorType::_temperature, methodsOfReading::number } } }
};

const std::map<uint8_t, std::vector<properties>> boilerBytes = {
  { 0,
    { { "Preset supply temperature", sensorType::_temperature, methodsOfReading::number } } },
  { 1,
    { { "Actual supply temperature", sensorType::_temperature, methodsOfReading::number } } },
  { 2,
    { { "Burner start-up temperature", sensorType::_temperature, methodsOfReading::number } } },
  { 3,
    { { "Burner shutdown temperature", sensorType::_temperature, methodsOfReading::number } } },
  { 6,
    { { "Errors", sensorType::_enumeration, methodsOfReading::bits, 0b11111111 } } },
  // { 7,
  //   { { "Byte7", methodsOfReading::bits, 0b01111111 } } },
  { 8,
    { { "Burner status", sensorType::_flag, methodsOfReading::flag, 0b00000001 } } }
};

const std::map<uint8_t, std::vector<properties>> configurationBytes = {
  { 0,
    { { "Outside temperature", sensorType::_temperature, methodsOfReading::number_signed } } },
  { 18,
    { { "Preset supply temperature", sensorType::_temperature, methodsOfReading::number } } },
  { 19,
    { { "Actual supply temperature", sensorType::_temperature, methodsOfReading::number } } }
};

const std::map<uint8_t, monitoringModule> monitoringModules = {
  { 0x80, { "Heating circuit 1", heatingCircuitBytes } },
  { 0x81, { "Heating circuit 2", heatingCircuitBytes } },
  { 0x82, { "Radiators", heatingCircuitBytes } },
  { 0x83, { "Floor", heatingCircuitBytes } },
  { 0x84, { "Hot water", hotWaterBytes } },
  { 0x88, { "Boiler", boilerBytes } },
  { 0x89, { "Configuration", configurationBytes } },
  { 0x87, { "Error protocol", {} } },  // пропускаем, не разобрался как расшифровать
  { 0x9a, { "KNX FM446", {} } }        // пропускаем, не понял что это такое
};

String uint8_to_bin(uint8_t byte) {
  char output[9];
  uint8_t pos = 0;
  for (uint8_t mask = 0b10000000; mask != 0; mask >>= 1) {
    if (byte & mask)
      output[pos] = '1';
    else
      output[pos] = '0';
    pos++;
  }
  output[8] = '\0';
  return output;
}

String decode_value(uint8_t byte, methodsOfReading method, uint8_t mask) {
  String decoded_value;

  byte = byte & mask;

  switch (method) {
    case methodsOfReading::number:
      return String((int)byte);
      break;

    case methodsOfReading::number_div2:
      return String((float)byte / 2);
      break;

    case methodsOfReading::number_signed:
      return String((int)(int8_t)byte);
      break;

    case methodsOfReading::flag:
      return bool(byte) ? "ON" : "OFF";
      break;

    case methodsOfReading::bits:
      return uint8_to_bin(byte);
      break;
  }
  return decoded_value;
};

void create_ha_sensor(const String &topic, const sensorType &type) {
  size_t unique_id = su::hash(topic.c_str());

  String name = topic;
  name.replace("/", " - ");

  String platform;
  String device_class;
  String unit_of_measurement;

  switch (type) {
    case sensorType ::_enumeration:
      platform = "sensor";
      device_class = "enum";
      break;

    case sensorType ::_flag:
      platform = "binary_sensor";
      break;

    case sensorType ::_temperature:
      platform = "sensor";
      device_class = "temperature";
      unit_of_measurement = "\u00b0C";
      break;

    case sensorType ::_memory:
      platform = "sensor";
      device_class = "data_size";
      unit_of_measurement = "kB";
      break;

    case sensorType ::_time:
      platform = "sensor";
      device_class = "duration";
      unit_of_measurement = "min";
      break;

    default:
      return;
  }

  gson::Str json;
  json('{');
  if (!device_class.isEmpty()) {
    json["device_class"] = device_class;
  }
  if (!unit_of_measurement.isEmpty()) {
    json["unit_of_measurement"] = unit_of_measurement;
  }
  json["name"] = name;
  snprintf(buf, sizeof(buf), "bud-0x%08x ", unique_id);
  json["unique_id"] = buf;
  json["state_topic"] = TOPIC_PREFIX + topic;
  json["device"]('{');
  json["identifiers"]('[');
  json += "logamatic";
  json(']');
  json["name"] = "Logamatic";
  json('}');
  json["platform"] = "sensor";
  json('}');

  json.concat('\0');

  snprintf(buf, sizeof(buf), "homeassistant/%s/bud-0x%08x/config", platform.c_str(), unique_id);

  mqttClient.publish(buf, json.buf());
  //mqttClient.publish(buf, json.buf(), true); //retained
};

void collect_data(const String &topic, const String &value, const sensorType &type) {
  //пытаемся вставить пустую строку, чтобы понять, первая это вставка или нет
  //возвращает пару <iterator, bool>
  auto result = collectedData.insert({ topic, "" });

  if (result.second)  //первая вставка
  {
    //не будем публиковать значение, чтобы HA успел обработать новый объект, опубликуем в следующий раз
    create_ha_sensor(topic, type);
  } else if (result.first->second != value) {
    result.first->second = value;  //обновление существующей записи
    mqttClient.publish((TOPIC_PREFIX + topic).c_str(), value.c_str());
  }
};

// описание байтов в data
//  0 — id модуля
//  1 — offset
//  2-7 — данные (6 бит)
void decode_message(uint8_t (&data)[]) {
  if (auto iter_module = monitoringModules.find(data[0]); iter_module != monitoringModules.end())  // ищем описание данных
  {
    const monitoringModule &module = iter_module->second;

    for (uint8_t i = 0; i <= 5; i++)  // перебираем байты в данных и и проверяем, интересует ли нас в них что-то
    {
      if (auto iter_byte = module.bytes.find(data[1] + i); iter_byte != module.bytes.end()) {
        const std::vector<properties> &properties = iter_byte->second;
        for (const auto &element : properties)  // считываем необходимое по правилам
          collect_data(module.topic + "/" + element.topic, decode_value(data[i + 2], element.method, element.mask), element.type);
      }
    }
  } else if (!count(unsupportedModules.begin(), unsupportedModules.end(), data[0]))  // фиксируем id модулей, для которых нет описания данных
  {
    unsupportedModules.push_back(data[0]);

    int bi = 0;
    for (const auto &element : unsupportedModules) {
      bi += snprintf(buf + bi, sizeof(buf) - bi, "0x%02x ", element);
    }
    collect_data("Status/Unsupported modules", buf, sensorType::_enumeration);
  }
};

void update_and_uptime() {
  // uptime
  uint32_t mins = millis() / 60000ul;
  collect_data("Status/Uptime", String(mins), sensorType::_time);

  // memory
  collect_data("Status/Free memory", String(esp_get_free_heap_size() / 1024), sensorType::_memory);

  // постепенное (один в минуту) обновление всех показателей (на всякий случай)
  if (!collectedData.empty()) {
    if (collectedData_iterator == collectedData.end()) {
      collectedData_iterator = collectedData.begin();
    }
    mqttClient.publish((TOPIC_PREFIX + collectedData_iterator->first).c_str(), collectedData_iterator->second.c_str());
    collectedData_iterator++;
  }
}

void setup_wifi() {
  delay(10);

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.hostname(hostname);
  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void check_wifi() {
  if (WiFi.status() != WL_CONNECTED) {
    while (WiFi.status() != WL_CONNECTED) {
      Serial.println("");
      Serial.print("Reconnecting to WiFi...");

      WiFi.disconnect();
      WiFi.reconnect();

      int i = 0;
      while (WiFi.status() != WL_CONNECTED && i <= 20) {
        Serial.print(".");
        delay(500);
        i++;
      }
    }

    Serial.println("");
    Serial.println("WiFi connected");
  }
}

void setup_OTA() {

  // Port defaults to 3232
  ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname(hostname);

  // Password can be set with plain text (will be hashed internally)
  // The authentication uses PBKDF2-HMAC-SHA256 with 10,000 iterations
  ArduinoOTA.setPassword(OTA_password);

  // Or set password with pre-hashed value (SHA256 hash of "admin")
  // SHA256(admin) = 8c6976e5b5410415bde908bd4dee15dfb167a9c873fc4bb8a81f6f2ab448a918
  // ArduinoOTA.setPasswordHash("8c6976e5b5410415bde908bd4dee15dfb167a9c873fc4bb8a81f6f2ab448a918");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      if (millis() - last_ota_time > 500) {
        Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
        last_ota_time = millis();
      }
    })
    .onError([](ota_error_t error) {
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

  Serial.println("Setup OTA...");

  ArduinoOTA.begin();
}

void check_mqtt_server() {
  if (!mqttClient.connected()) {
    while (!mqttClient.connected() && WiFi.status() == WL_CONNECTED) {
      Serial.print("Attempting MQTT connection...");

      if (mqttClient.connect(mqtt_client_id, mqtt_username, mqtt_password)) {
        Serial.println("connected");
        // Subscribe
        // mqttClient.subscribe(TOPIC_PREFIX "from-python/send/");
      } else {
        Serial.print("failed, rc=");
        Serial.print(mqttClient.state());
        Serial.println(" try again in 5 seconds");
        // Wait 5 seconds before retrying
        delay(5000);
      }
    }
  }
}

void check_connections() {
  check_wifi();
  check_mqtt_server();
}

void setup() {
  Serial.begin(115200);

  setup_wifi();
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setBufferSize(512);
  // mqttClient.setCallback(callback);
  check_mqtt_server();

  setup_OTA();

  SPI.begin();  // инициализируем связь по протоколу SPI

  mcp2515.reset();
  mcp2515.setBitrate(CAN_50KBPS, MCP_8MHZ);  // устанавливаем скорость шины CAN 50 кбит/с и частоту кварцевого генератора 8 МГц
  mcp2515.setNormalMode();                   // устанавливаем CAN-шину в обычный режим
}

void loop() {

  ArduinoOTA.handle();

  mqttClient.loop();

  if (tmrCheckConnections)
    check_connections();
  if (tmrUpdateAndUptime)
    update_and_uptime();

  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {  // To receive data (Poll Read)

    // int bi = snprintf(buf, sizeof(buf), "%lx;%x;%02x %02x %02x %02x %02x %02x %02x %02x", canMsg.can_id, canMsg.can_dlc, canMsg.data[0], canMsg.data[1], canMsg.data[2], canMsg.data[3], canMsg.data[4], canMsg.data[5], canMsg.data[6], canMsg.data[7]);
    // mqttClient.publish(TOPIC_PREFIX "can/recv/", buf);

    if (canMsg.can_id == 0x400 & canMsg.can_dlc == 8)
      decode_message(canMsg.data);
  }
}

// void callback(char* topic, byte* payload, unsigned int length) {

//   unsigned l = sizeof(buf) - 1 < length ? sizeof(buf) : length;
//   memcpy(buf, payload, l);
//   Serial.println(buf);
//   int r = sscanf(buf, "%x;%lx; %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx", &canMsg.can_dlc, &canMsg.can_id, &canMsg.data[0], &canMsg.data[1], &canMsg.data[2], &canMsg.data[3], &canMsg.data[4], &canMsg.data[5], &canMsg.data[6], &canMsg.data[7]);
//   canMsg.can_dlc = 8;

//   if (r == 10){
//     mcp2515.sendMessage(&canMsg);

//     int bi = snprintf(buf, sizeof(buf), "%lx;%x;%02x %02x %02x %02x %02x %02x %02x %02x", canMsg.can_id, canMsg.can_dlc, canMsg.data[0], canMsg.data[1], canMsg.data[2], canMsg.data[3], canMsg.data[4], canMsg.data[5], canMsg.data[6], canMsg.data[7]);

//     mqttClient.publish(TOPIC_PREFIX "can/send/", buf);
//   }
// }
