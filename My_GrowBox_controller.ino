// GrowBox Controller v.1.1
// Вмдалено керування реле вентилятора БЖ
// Додана логіка керування вентилятором БЖ PWM
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <EEPROM.h>
#include <time.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <Update.h>


// Налаштування WiFi
const char* ssid = "TP-LINK_7BB2";
const char* password = "Hodapu99";

// Налаштування статичної IP-адреси
IPAddress local_IP(192, 168, 0, 165);         // Бажана IP-адреса
IPAddress gateway(192, 168, 0, 1);            // IP-адреса шлюзу (зазвичай роутера)
IPAddress subnet(255, 255, 255, 0);           // Маска підмережі
IPAddress primaryDNS(8, 8, 8, 8);             // Основний DNS (можна змінити)
IPAddress secondaryDNS(8, 8, 4, 4);           // Додатковий DNS (можна змінити)


// Піни для реле та датчиків
#define LIGHT1_PIN 33 // 
#define LIGHT2_PIN 26 // 
#define LIGHT3_PIN 27 // 
#define HEATER_PIN 14 // Обігрівач
#define HUMIDIFIER_PIN 19 // Зволожувач
// #define VENT_PIN 18 // Вентилятор провітрювання
#define FAN_PIN 32  // PWM пін для вентилятора обдуву
#define DHT_PIN 15 // 
#define BACKUP_POWER_RELAY 17 // Пін для реле резервного живлення 
#define VOLTAGE_PIN 36 // Аналоговий пін для вимірювання напруги
#define NEW_FAN_PIN 25 // Пін для вентилятора БЖ PWM

#define NEW_OUTSIDE_FAN_PIN 21   // Пін для вентилятора з вулиці (PWM)
#define NEW_OUTSIDE_FAN_TACH_PIN 34 // Пін для тахометра вентилятора з вулиці
#define NEW_VENT_FAN_PIN 22      // Пін для вентилятора провітрювання (PWM)
#define NEW_VENT_FAN_TACH_PIN 35  // Пін для тахометра вентилятора провітрювання

#define EEPROM_SIZE 512

// Налаштування DHT11
DHT dht(DHT_PIN, DHT11);

// Створення веб-сервера
AsyncWebServer server(80);

// Структура для зберігання налаштувань
struct Settings {
  // Налаштування освітлення
  int light1_on_hour = 6;
  int light1_on_minute = 0;
  int light1_off_hour = 18;
  int light1_off_minute = 0;

  int light2_on_hour = 6;
  int light2_on_minute = 0;
  int light2_off_hour = 18;
  int light2_off_minute = 0;

  int light3_on_hour = 6;
  int light3_on_minute = 0;
  int light3_off_hour = 18;
  int light3_off_minute = 0;

  // Налаштування температури
  float temp_min = 20.0;
  float temp_max = 28.0;
  float temp_critical = 32.0;

  // Налаштування вологості
  float humidity_min = 40.0;
  float humidity_max = 70.0;
  float humidity_critical = 85.0;

  // Налаштування вентиляції
  int vent_on_minutes = 15;
  int vent_off_minutes = 45;
  int vent_on_speed = 70; // Швидкість вентилятора провітрювання під час роботи (за замовчуванням 70%)
  int vent_off_speed = 0; // Швидкість вентилятора провітрювання під час паузи (за замовчуванням 0%)
  int fan_speed = 50;  // 0-100%
} settings;

// Змінні стану
bool light1_state = false;
bool light2_state = false;
bool light3_state = false;
bool heater_state = false;
bool humidifier_state = false;
bool vent_state = false;
unsigned long last_vent_switch = 0;
float current_temp = 0;
float current_humidity = 0;
bool critical_temp_state = false;
bool critical_humidity_state = false;
unsigned long last_debug_print = 0;
int outsideFanSpeed = 0;
int ventFanSpeed = 0;
volatile unsigned long outsideFanPulses = 0;
volatile unsigned long ventFanPulses = 0;
unsigned long lastTachCheck = 0;
float outsideFanRPM = 0;
float ventFanRPM = 0;
bool isBackupPower = false; // Прапорець стану резервного живлення
float currentVoltage = 0.0; // Поточна напруга
unsigned long lastPowerCheck = 0; // Час останньої перевірки напруги
const float R1 = 100000.0; // 100 кОм
const float R2 = 33000.0; // 33 кОм
const float thresholdVoltage = 10.0; // Порогова напруга (10 В)
const unsigned long powerCheckInterval = 5000; // Інтервал перевірки напруги (5 секунд)


float readVoltage() {
  int rawValue = analogRead(VOLTAGE_PIN);
  float voltage = rawValue * (3.3 / 4095.0); // Перетворення аналогового значення в напругу (3.3V - опорна напруга АЦП)
  float realVoltage = voltage / (R2 / (R1 + R2)); // Розрахунок реальної напруги з урахуванням дільника
  return realVoltage;
}

void checkPower() {
  unsigned long currentTime = millis();
  if (currentTime - lastPowerCheck >= powerCheckInterval) {
    lastPowerCheck = currentTime;
    currentVoltage = readVoltage();

    if (currentVoltage < thresholdVoltage && !isBackupPower) {
      Serial.println("Перехід на резервне живлення!");
      digitalWrite(BACKUP_POWER_RELAY, LOW); // Активуємо реле
      isBackupPower = true;
    } else if (currentVoltage >= thresholdVoltage && isBackupPower) {
      Serial.println("Повернення на основне живлення!");
      digitalWrite(BACKUP_POWER_RELAY, HIGH); // Вимикаємо реле
      isBackupPower = false;
    }
  }
}

void saveSettings() { // Функція зберігання налаштувань в EEPROM
  EEPROM.put(0, settings);
  EEPROM.commit();
  Serial.println("Налаштування збережено в EEPROM");
}

void loadSettings() { // Функція завантаження налаштувань з EEPROM
  EEPROM.get(0, settings);
}

void setupTime() {
  configTime(2 * 3600, 0, "pool.ntp.org", "time.nist.gov"); // UTC + 2 для Києва

  Serial.println("Очікування синхронізації часу...");
  time_t now = time(nullptr);
  while (now < 24 * 3600) {
    delay(500);
    now = time(nullptr);
  }
  Serial.println("Час синхронізовано");
}

String getTimeString() {
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  char buffer[26];
  sprintf(buffer, "%02d:%02d:%02d %02d/%02d/%04d",
          timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec,
          timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900);
  return String(buffer);
}

void checkLighting() {
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  int current_minutes = timeinfo->tm_hour * 60 + timeinfo->tm_min;

  // Перевірка Лінії 1
  int light1_on_minutes = settings.light1_on_hour * 60 + settings.light1_on_minute;
  int light1_off_minutes = settings.light1_off_hour * 60 + settings.light1_off_minute;
  if (current_minutes >= light1_on_minutes && current_minutes < light1_off_minutes) {
    if (!light1_state) {
      digitalWrite(LIGHT1_PIN, HIGH);
      light1_state = true;
    }
  } else {
    if (light1_state) {
      digitalWrite(LIGHT1_PIN, LOW);
      light1_state = false;
    }
  }

  // Перевірка Лінії 2
  int light2_on_minutes = settings.light2_on_hour * 60 + settings.light2_on_minute;
  int light2_off_minutes = settings.light2_off_hour * 60 + settings.light2_off_minute;
  if (current_minutes >= light2_on_minutes && current_minutes < light2_off_minutes) {
    if (!light2_state) {
      digitalWrite(LIGHT2_PIN, HIGH);
      light2_state = true;
    }
  } else {
    if (light2_state) {
      digitalWrite(LIGHT2_PIN, LOW);
      light2_state = false;
    }
  }

  // Перевірка Лінії 3
  int light3_on_minutes = settings.light3_on_hour * 60 + settings.light3_on_minute;
  int light3_off_minutes = settings.light3_off_hour * 60 + settings.light3_off_minute;
  if (current_minutes >= light3_on_minutes && current_minutes < light3_off_minutes) {
    if (!light3_state) {
      digitalWrite(LIGHT3_PIN, HIGH);
      light3_state = true;
    }
  } else {
    if (light3_state) {
      digitalWrite(LIGHT3_PIN, LOW);
      light3_state = false;
    }
  }
  updateNewFanSpeed(); // Оновлюємо швидкість НОВОГО вентилятора після зміни стану освітлення
}

void controlHeater() {   // Нові функції керування обігрівачем та зволожувачем
  // Звичайна логіка обігрівача
  if (current_temp < settings.temp_min) {
    if (!heater_state) {
      digitalWrite(HEATER_PIN, HIGH);
      heater_state = true;
    }
  } else if (current_temp > settings.temp_max) {
    if (heater_state) {
      digitalWrite(HEATER_PIN, LOW);
      heater_state = false;
    }
  }
}

void controlHumidifier() {
  // Звичайна логіка зволожувача
  if (current_humidity < settings.humidity_min) {
    if (!humidifier_state) {
      digitalWrite(HUMIDIFIER_PIN, HIGH);
      humidifier_state = true;
    }
  } else if (current_humidity > settings.humidity_max) {
    if (humidifier_state) {
      digitalWrite(HUMIDIFIER_PIN, LOW);
      humidifier_state = false;
    }
  }
}

void checkTemperature() { // Нові функції checkTemperature() та checkHumidity() (без логіки обігрівача/зволожувача)
  if (current_temp > settings.temp_max) {
    setVentFanSpeed(0);
    if (current_temp >= settings.temp_critical) {
      setOutsideFanSpeed(100);
    } else {
      int speed = map(current_temp, settings.temp_max, settings.temp_critical, 0, 100);
      setOutsideFanSpeed(constrain(speed, 0, 100));
    }
  } else {
    setOutsideFanSpeed(0);
  }
}

void checkHumidity() {
  if (current_humidity > settings.humidity_critical) {
    setOutsideFanSpeed(100);
  }
}

void checkVentilation() {
  if (current_temp > settings.temp_max || current_humidity > settings.humidity_critical) {
    return; // Не виконуємо цикл вентиляції при підвищеній температурі або вологості
  }

  unsigned long current_time = millis();
  unsigned long vent_cycle = (settings.vent_on_minutes + settings.vent_off_minutes) * 60000UL;
  unsigned long time_in_cycle = (current_time - last_vent_switch) % vent_cycle;

  if (time_in_cycle < (settings.vent_on_minutes * 60000UL)) {
    setVentFanSpeed(settings.vent_on_speed);
  } else {
    setVentFanSpeed(settings.vent_off_speed);
  }
}

void printDebugInfo() { // Функція для діагностичного виводу
  unsigned long current_time = millis();
  if (current_time - last_debug_print >= 5000) { // Виводимо кожні 5 секунд
    last_debug_print = current_time;

    Serial.println("\n--- Debug Info ---");
    Serial.print("Temperature: "); Serial.println(current_temp);
    Serial.print("Humidity: "); Serial.println(current_humidity);
    Serial.print("Critical Temp State: "); Serial.println(critical_temp_state);
    Serial.print("Critical Humidity State: "); Serial.println(critical_humidity_state);
    Serial.print("Vent State: "); Serial.println(vent_state);
    Serial.print("Temp Max: "); Serial.println(settings.temp_max);
    Serial.print("Temp Critical: "); Serial.println(settings.temp_critical);
    Serial.print("Humidity Max: "); Serial.println(settings.humidity_max);
    Serial.print("Humidity Critical: "); Serial.println(settings.humidity_critical);
    Serial.println("----------------");
  }
}

void setOutsideFanSpeed(int speed) {
  outsideFanSpeed = speed;
  int pwm_value = map(speed, 0, 100, 0, 255);
  ledcWrite(2, pwm_value); // Використовуємо канал 2 для вентилятора з вулиці
}

void setVentFanSpeed(int speed) {
  ventFanSpeed = speed;
  int pwm_value = map(speed, 0, 100, 0, 255);
  ledcWrite(3, pwm_value); // Використовуємо канал 3 для вентилятора провітрювання
}

void setFanSpeed(int speed) {
  int pwm_value = map(speed, 0, 100, 0, 255);
  ledcWrite(0, pwm_value);  // Канал 0 для PWM
}

void setNewFanSpeed(int speed) { // Функція для встановлення швидкості вентилятора БЖ PWM
  int pwm_value = map(speed, 0, 100, 0, 255);
  ledcWrite(1, pwm_value); // Використовуємо канал 1 для НОВОГО вентилятора
}

void updateNewFanSpeed() { // Функція для оновлення швидкості вентилятора БЖ PWM на основі ввімкнених ліній освітлення
  int activeLights = 0;

  if (digitalRead(LIGHT1_PIN) == HIGH) {
    activeLights++;
  }
  if (digitalRead(LIGHT2_PIN) == HIGH) {
    activeLights++;
  }
  if (digitalRead(LIGHT3_PIN) == HIGH) {
    activeLights++;
  }

  int fanSpeed;

  switch (activeLights) {
    case 0:
      fanSpeed = 0; // 0% ШІМ (вимкнено)
      break;
    case 1:
      fanSpeed = 33; // ~33% ШІМ
      break;
    case 2:
      fanSpeed = 66; // ~66% ШІМ
      break;
    case 3:
      fanSpeed = 100; // 100% ШІМ
      break;
    default: //на всякий випадок якщо буде помилка
      fanSpeed = 0;
      break;
  }

  setNewFanSpeed(fanSpeed);
}

void IRAM_ATTR outsideFanTachISR() {
  outsideFanPulses++;
}

void IRAM_ATTR ventFanTachISR() {
  ventFanPulses++;
}

void checkTachometers() {
  unsigned long currentTime = millis();
  if (currentTime - lastTachCheck >= 1000) { // Перевірка кожну секунду
    noInterrupts(); // Вимикаємо переривання під час обчислень
    outsideFanRPM = (outsideFanPulses * 60.0) / (currentTime - lastTachCheck);
    ventFanRPM = (ventFanPulses * 60.0) / (currentTime - lastTachCheck);
    outsideFanPulses = 0;
    ventFanPulses = 0;
    lastTachCheck = currentTime;
    interrupts(); // Знову вмикаємо переривання
  }
}

void setup() {
  Serial.begin(115200);

  // Ініціалізація EEPROM
  EEPROM.begin(EEPROM_SIZE);
  loadSettings();

  // Налаштування пінів
  pinMode(LIGHT1_PIN, OUTPUT);
  pinMode(LIGHT2_PIN, OUTPUT);
  pinMode(LIGHT3_PIN, OUTPUT);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(HUMIDIFIER_PIN, OUTPUT);
  pinMode(NEW_FAN_PIN, OUTPUT); // Встановлюємо пін нового вентилятора як вихід
  pinMode(NEW_OUTSIDE_FAN_PIN, OUTPUT);
  pinMode(NEW_VENT_FAN_PIN, OUTPUT);
  pinMode(NEW_OUTSIDE_FAN_TACH_PIN, INPUT_PULLUP);
  pinMode(NEW_VENT_FAN_TACH_PIN, INPUT_PULLUP);
  pinMode(BACKUP_POWER_RELAY, OUTPUT);
  pinMode(VOLTAGE_PIN, INPUT_PULLUP);



  digitalWrite(LIGHT1_PIN, LOW);
  digitalWrite(LIGHT2_PIN, LOW);
  digitalWrite(LIGHT3_PIN, LOW);
  digitalWrite(HEATER_PIN, LOW);
  digitalWrite(HUMIDIFIER_PIN, LOW);
  digitalWrite(BACKUP_POWER_RELAY, HIGH); // Початково реле вимкнено (живлення від основного джерела)


  ledcSetup(1, 5000, 8); // Канал 1 для НОВОГО вентилятора, 5кГц, 8-біт роздільна здатність
  ledcAttachPin(NEW_FAN_PIN, 1);
  setNewFanSpeed(0); // Початково вимикаємо НОВИЙ вентилятор

  ledcSetup(0, 5000, 8);  // Канал 0, для вентилятора обдуву 5кГц, 8-біт роздільна здатність
  ledcAttachPin(FAN_PIN, 0);
  setFanSpeed(settings.fan_speed);

  ledcSetup(2, 5000, 8); // Канал 2 для вентилятора з вулиці
  ledcAttachPin(NEW_OUTSIDE_FAN_PIN, 2);
  ledcSetup(3, 5000, 8); // Канал 3 для вентилятора провітрювання
  ledcAttachPin(NEW_VENT_FAN_PIN, 3);

  attachInterrupt(digitalPinToInterrupt(NEW_OUTSIDE_FAN_TACH_PIN), outsideFanTachISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(NEW_VENT_FAN_TACH_PIN), ventFanTachISR, FALLING);

  setOutsideFanSpeed(0);
  setVentFanSpeed(0);

  // Ініціалізація DHT11
  dht.begin();

  // Спроба встановити статичну IP-адресу
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Не вдалося налаштувати статичну IP-адресу");
  }

  // Підключення до WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Підключення до WiFi...");
  }
  Serial.println("WiFi підключено!");
  Serial.print("IP адреса: ");
  Serial.println(WiFi.localIP());

  // Ініціалізація mDNS із назвою "growbox"
  if (!MDNS.begin("growboxx")) {
    Serial.println("Помилка налаштування mDNS");
  } else {
    Serial.println("mDNS відповідає як growbox.local");
  }

  // Ініціалізація SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("Помилка монтування SPIFFS");
    return;
  }


  // OTA setup
  ArduinoOTA.setHostname("GrowBox");
// ArduinoOTA.setPassword("your_ota_password");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed"); // Виправлено
    else if (error == OTA_END_ERROR) Serial.println("End Failed");       // Виправлено
  });
  ArduinoOTA.begin();

  // Налаштування часу
  setupTime();

  
    // Обробник для оновлення прошивки
    AsyncElegantOTA.begin(&server);    // Start ElegantOTA


  // Налаштування маршрутів сервера
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.htm", "text/html");
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest * request) {
    StaticJsonDocument<512> doc;
    doc["time"] = getTimeString();
    doc["temperature"] = current_temp;
    doc["humidity"] = current_humidity;
    doc["light1"] = light1_state;
    doc["light2"] = light2_state;
    doc["light3"] = light3_state;
    doc["heater"] = heater_state;
    doc["humidifier"] = humidifier_state;
    doc["vent"] = vent_state;
    doc["fan_speed"] = settings.fan_speed;
    doc["is_backup_power"] = isBackupPower; // Додано


    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest * request) {
    StaticJsonDocument<512> doc;

    doc["light1_on_hour"] = settings.light1_on_hour;
    doc["light1_on_minute"] = settings.light1_on_minute;
    doc["light1_off_hour"] = settings.light1_off_hour;
    doc["light1_off_minute"] = settings.light1_off_minute;

    doc["light2_on_hour"] = settings.light2_on_hour;
    doc["light2_on_minute"] = settings.light2_on_minute;
    doc["light2_off_hour"] = settings.light2_off_hour;
    doc["light2_off_minute"] = settings.light2_off_minute;

    doc["light3_on_hour"] = settings.light3_on_hour;
    doc["light3_on_minute"] = settings.light3_on_minute;
    doc["light3_off_hour"] = settings.light3_off_hour;
    doc["light3_off_minute"] = settings.light3_off_minute;

    doc["temp_min"] = settings.temp_min;
    doc["temp_max"] = settings.temp_max;
    doc["temp_critical"] = settings.temp_critical;

    doc["humidity_min"] = settings.humidity_min;
    doc["humidity_max"] = settings.humidity_max;
    doc["humidity_critical"] = settings.humidity_critical;

    doc["vent_on_minutes"] = settings.vent_on_minutes;
    doc["vent_off_minutes"] = settings.vent_off_minutes;
    doc["vent_on_speed"] = settings.vent_on_speed; // Додано
    doc["vent_off_speed"] = settings.vent_off_speed; // Додано
    doc["fan_speed"] = settings.fan_speed;

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  server.on("/update", HTTP_POST, [](AsyncWebServerRequest * request) {}, NULL,
  [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {
    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, (const char*)data, len);

    if (error) {
      request->send(400, "text/plain", "Помилка JSON");
      return;
    }

    // Оновлення налаштувань
    if (doc.containsKey("light1_on_hour")) settings.light1_on_hour = doc["light1_on_hour"];
    if (doc.containsKey("light1_on_minute")) settings.light1_on_minute = doc["light1_on_minute"];
    if (doc.containsKey("light1_off_hour")) settings.light1_off_hour = doc["light1_off_hour"];
    if (doc.containsKey("light1_off_minute")) settings.light1_off_minute = doc["light1_off_minute"];

    if (doc.containsKey("light2_on_hour")) settings.light2_on_hour = doc["light2_on_hour"];
    if (doc.containsKey("light2_on_minute")) settings.light2_on_minute = doc["light2_on_minute"];
    if (doc.containsKey("light2_off_hour")) settings.light2_off_hour = doc["light2_off_hour"];
    if (doc.containsKey("light2_off_minute")) settings.light2_off_minute = doc["light2_off_minute"];

    if (doc.containsKey("light3_on_hour")) settings.light3_on_hour = doc["light3_on_hour"];
    if (doc.containsKey("light3_on_minute")) settings.light3_on_minute = doc["light3_on_minute"];
    if (doc.containsKey("light3_off_hour")) settings.light3_off_hour = doc["light3_off_hour"];
    if (doc.containsKey("light3_off_minute")) settings.light3_off_minute = doc["light3_off_minute"];

    if (doc.containsKey("temp_min")) settings.temp_min = doc["temp_min"];
    if (doc.containsKey("temp_max")) settings.temp_max = doc["temp_max"];
    if (doc.containsKey("temp_critical")) settings.temp_critical = doc["temp_critical"];

    if (doc.containsKey("humidity_min")) settings.humidity_min = doc["humidity_min"];
    if (doc.containsKey("humidity_max")) settings.humidity_max = doc["humidity_max"];
    if (doc.containsKey("humidity_critical")) settings.humidity_critical = doc["humidity_critical"];

    if (doc.containsKey("vent_on_minutes")) settings.vent_on_minutes = doc["vent_on_minutes"].as<int>();
    if (doc.containsKey("vent_off_minutes")) settings.vent_off_minutes = doc["vent_off_minutes"].as<int>();
    if (doc.containsKey("vent_on_speed")) settings.vent_on_speed = doc["vent_on_speed"].as<int>(); // Додано
    if (doc.containsKey("vent_off_speed")) settings.vent_off_speed = doc["vent_off_speed"].as<int>(); // Додано
    if (doc.containsKey("fan_speed")) {
      settings.fan_speed = doc["fan_speed"].as<int>();
      setFanSpeed(settings.fan_speed);
    }

    saveSettings();
    request->send(200, "text/plain", "Налаштування оновлено");
  });

  // Запуск сервера
  server.begin();
}

void checkWiFiConnection() { // Функція перевірки підключення WiFi
  static unsigned long lastWiFiCheck = 0;
  const unsigned long CHECK_WIFI_INTERVAL = 30000; // Перевірка кожні 30 секунд

  if (millis() - lastWiFiCheck >= CHECK_WIFI_INTERVAL) {
    lastWiFiCheck = millis();

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("З'єднання з WiFi втрачено. Спроба перепідключення...");
      WiFi.disconnect();
      WiFi.begin(ssid, password);

      unsigned long startAttemptTime = millis();

      while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 20000) {
        delay(500);
        Serial.print(".");
      }

      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi перепідключено успішно");
        Serial.print("IP адреса: ");
        Serial.println(WiFi.localIP());
      } else {
        Serial.println("\nНе вдалося відновити з'єднання з WiFi");
      }
    }
  }
}

void updateSensorData() { // Функція оновлення показників датчиків
  static unsigned long lastSensorUpdate = 0;
  const unsigned long SENSOR_UPDATE_INTERVAL = 2000; // Оновлення кожні 2 секунди

  if (millis() - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL) {
    lastSensorUpdate = millis();

    float newTemp = dht.readTemperature();
    float newHumidity = dht.readHumidity();

    if (!isnan(newTemp) && !isnan(newHumidity)) {
      current_temp = newTemp;
      current_humidity = newHumidity;
    }
  }
}

void loop() {
  checkWiFiConnection();
  updateSensorData();

  // Перевірка та оновлення стану всіх систем
  checkPower();
  checkLighting();
  controlHeater();      // Викликаємо функцію керування обігрівачем
  controlHumidifier();  // Викликаємо функцію керування зволожувачем
  checkTemperature();
  checkHumidity();
  checkVentilation();
  checkTachometers();

  updateNewFanSpeed(); // Додатковий виклик для синхронізації, якщо стан освітлення змінюється не тільки в checkLighting()

  printDebugInfo(); // Додаємо діагностичний вивід

  ArduinoOTA.handle(); // Обробка OTA оновлень


  // Коротка затримка для стабільності
  delay(100);
}
