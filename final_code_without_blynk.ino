#include <Servo.h>
#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <RTClib.h>  // RTC DS3231
#include <LiquidCrystal_I2C.h>
#include <UniversalTelegramBot.h>
#include <WiFiClientSecure.h>

#define USE_DEBUG_PRINT   0
#define USE_TELEGRAM_BOT  1

#define SERVO_OPEN_DURATION_MS  250
#define SERVO_OPEN_DEGREE       35
#define SERVO_CLOSE_DEGREE      0

// Ultrasonic & LCD Setup
#define TRIG_PIN          D6
#define ECHO_PIN          D7
#define MIN_HEIGHT_CM   30
#define MAX_HEIGHT_CM    3
#define MAX_PERCENTAGE    100
#define MIN_PRECENTAGE    0
#define PERCENTAGE_TRESH  20

// Telegram Bot Setup
#define BOT_TOKEN         "7942156151:AAHXZ1jBfbuHC75lD7oqDzk-z9ebTdYnzWo"
#define CHAT_ID           "1705069823"

// wifi setup
#define WIFI_SSID                 "gendhis"
#define WIFI_PWD                  "tanyaajadulu"
#define WIFI_TIMEOUT_MS           30000
#define WIFI_CONNECT_INTERVAL_MS  500

// lcd setup
#define LCD_I2C_ADDR      0x27
#define LCD_COL           16
#define LCD_ROW           2

// serial monitor setup
#define SERIAL_BAUD       115200

#define GMT_7_TIME_OFFSET       25200
#define NTP_UPDATE_INTERVAL_MS  60000

typedef enum time_id_e {
  TIME_ID_HOUR,
  TIME_ID_MIN,
  TIME_ID_SEC,
  TIME_ID_SIZE,
} time_id_t;

bool send_to_bot(String message);
wl_status_t connect_to_wifi(int timeout);
String update_time(bool ntp_status);
float calculate_fodder_height(void);
long calculate_fodder_percentage(float height);

unsigned long lastTriggerMillis = 0;
bool servoAtZero = false, is_servo_was_opened = false;

int last_wifi_status = 0, current_time[TIME_ID_SIZE];

bool is_on_feed_schedule = false, useNTP = false;
String time_source;

float fodder_height = 0;
long fodder_percentage = 0;

// Servo & Time Setup
Servo myServo;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", GMT_7_TIME_OFFSET, NTP_UPDATE_INTERVAL_MS);

RTC_DS3231 rtc;

LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COL, LCD_ROW);  // I2C LCD

// telegram bot setup
WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);

bool send_to_bot(String message) {
  yield();
  if (WiFi.status() != WL_CONNECTED) return false;
  return bot.sendMessage(CHAT_ID, message, "Markdown");
}

wl_status_t connect_to_wifi(int timeout) {
  int wifi_connect_time_ms = 0;
  WiFi.disconnect();
  delay(1000);

  WiFi.begin(WIFI_SSID, WIFI_PWD);
  Serial.print("Connecting to WiFi");
  while (wifi_connect_time_ms < timeout) {
    delay(WIFI_CONNECT_INTERVAL_MS);
    wifi_connect_time_ms += WIFI_CONNECT_INTERVAL_MS;
    Serial.print(".");
    if (WiFi.status() == WL_CONNECTED) return WL_CONNECTED;
  }
  return WiFi.status();
}

String update_time(bool ntp_status) {
  if (ntp_status) {
    if (!timeClient.isTimeSet()) {
      Serial.println("update RTC from NTP");
      adjust_rtc();
    }
    current_time[TIME_ID_HOUR] = timeClient.getHours();
    current_time[TIME_ID_MIN] = timeClient.getMinutes();
    current_time[TIME_ID_SEC] = timeClient.getSeconds();
    return "NTP";
  }

  last_wifi_status = WiFi.status();
  current_time[TIME_ID_HOUR] = rtc.now().hour();
  current_time[TIME_ID_MIN] = rtc.now().minute();
  current_time[TIME_ID_SEC] = rtc.now().second();
  return "RTC";
}

void adjust_rtc(void) {
  yield();
  if(timeClient.update()) {
    rtc.adjust(DateTime(timeClient.getEpochTime()));
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  myServo.attach(D5);

  myServo.write(SERVO_CLOSE_DEGREE);
#if defined (USE_DEBUG_PRINT) && (USE_DEBUG_PRINT)
  Serial.println("Initial servo position set to 0°");
#endif

  secured_client.setInsecure();

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting time from NTP after WiFi connect...");
  }

  if (connect_to_wifi(WIFI_TIMEOUT_MS) != WL_CONNECTED) {
    Serial.printf("\nFailed to connect to WiFi. reason: %d\r\nObtain time info from RTC.", WiFi.status());
    if (rtc.lostPower()) {
      Serial.println("RTC lost power while failed connection. halting...");
      while(1);
    }
  }
  else {
    last_wifi_status = WL_CONNECTED;
    Serial.println("\nConnected to WiFi!");
    send_to_bot(String("\u2728 Terkoneksi ke SSID: " + String(WIFI_SSID) + ". RSSI: " + String(WiFi.RSSI()) + "dBm."));
  }
  timeClient.begin();

  // Ultrasonic & LCD Setup
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  lcd.init();
  lcd.backlight();
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    useNTP = true;
    if (last_wifi_status != WL_CONNECTED) {
      send_to_bot(String("\u2728 Terkoneksi kembali ke SSID: " + String(WIFI_SSID) + ". RSSI: " + String(WiFi.RSSI()) + "dBm."));
      last_wifi_status = WL_CONNECTED;
    }
  }
  else {
    useNTP = false;
  }

  time_source = update_time(useNTP);

#if defined (USE_DEBUG_PRINT) && (USE_DEBUG_PRINT)
  Serial.printf("Current Time: %02d:%02d:%02d WIB (%s)\n", current_time[TIME_ID_HOUR], current_time[TIME_ID_MIN], current_time[TIME_ID_SEC], time_source.c_str());
#endif

  // bool is_on_feed_schedule = (current_time[TIME_ID_SEC] % 20 == 0);
  bool is_on_feed_schedule = (current_time[TIME_ID_HOUR] == 7) || (current_time[TIME_ID_HOUR] == 17) || (current_time[TIME_ID_HOUR] == 22);

  if (is_on_feed_schedule && !servoAtZero && !is_servo_was_opened) {
    Serial.printf(
      "Trigger time matched! Moving servo to 35° at %02d:%02d:%02d WIB (%s)\n", 
      current_time[TIME_ID_HOUR], current_time[TIME_ID_MIN], current_time[TIME_ID_SEC], time_source.c_str()
    );

    myServo.write(SERVO_OPEN_DEGREE);

    servoAtZero = is_servo_was_opened = true;

    lastTriggerMillis = millis();
  }

  if (servoAtZero && (millis() - lastTriggerMillis >= SERVO_OPEN_DURATION_MS)) {
    Serial.printf("Returning servo to 0° after %.3f second.\r\n", float(SERVO_OPEN_DURATION_MS) / 1000);
    myServo.write(SERVO_CLOSE_DEGREE);
    servoAtZero = false;

#if defined (USE_TELEGRAM_BOT) && (USE_TELEGRAM_BOT)
    send_to_bot(
      String("\u2705 Pemberian pakan berhasil! Waktu: ") + 
      String(current_time[TIME_ID_HOUR]) + ":" + 
      (current_time[TIME_ID_MIN] < 10 ? "0" : "") + 
      String(current_time[TIME_ID_MIN])
    );
#endif
    adjust_rtc();
  }

  is_servo_was_opened = is_on_feed_schedule ? is_servo_was_opened : false;

  yield();
  fodder_height = calculate_fodder_height();
  fodder_percentage = calculate_fodder_percentage(fodder_height);

  lcd.setCursor(0, 0);
  lcd.print("Sisa Pakan: ");
  lcd.print(fodder_percentage);
  lcd.print("%    ");
  lcd.setCursor(0, 1);
  if (fodder_percentage < PERCENTAGE_TRESH) {
    lcd.print("ISI PAKAN!!     ");
  } else {
    lcd.print("                ");
  }

#if defined (USE_DEBUG_PRINT) && (USE_DEBUG_PRINT)
  Serial.print("Sisa: ");
  Serial.print(persen);
  Serial.println("%");
#endif

  delay(float(SERVO_OPEN_DURATION_MS)/2);
}

float calculate_fodder_height(void) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  yield();
  float length = duration * 0.034 / 2.0;

#if defined (USE_DEBUG_PRINT) && (USE_DEBUG_PRINT)
  Serial.printf("length: %d\r\nduration: %d\r\n", length, duration);
#endif

  return length;
}

long calculate_fodder_percentage(float height) {
  if (height >= MIN_HEIGHT_CM) return MIN_PRECENTAGE;
  if (height <= MAX_HEIGHT_CM) return MAX_PERCENTAGE;

  long percentage = map(height, MAX_HEIGHT_CM, MIN_HEIGHT_CM, MAX_PERCENTAGE, MIN_PRECENTAGE);

#if defined (USE_DEBUG_PRINT) && (USE_DEBUG_PRINT)
  Serial.printf("persentase: %ld\r\n", percentage);
#endif

  return percentage;
}
