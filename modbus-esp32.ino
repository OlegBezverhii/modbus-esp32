/*
 * modbus-esp32 — пассивный логгер Modbus RTU на шине RS-485.
 *
 * ESP32-C3 + MAX485 (DE/RE на GND, только приём) + DS3231 + microSD + SSD1306.
 *
 * Формат лога (одна строка на кадр):
 *   12:34:56.789 [01 03 00 00 00 0A C5 CD]
 *   12:34:56.791 [01 03 14 00 00 ... ] bad
 *
 *   HH:MM:SS.mmm — локальное время RTC + миллисекунды из millis()
 *   bad          — CRC16 не сошёлся (шум на шине / обрезанный кадр)
 *   ovf          — кадр длиннее 256 байт (не Modbus, обычно неверный baud)
 *
 * Ключевые отличия от предыдущей версии:
 *   - t3.5 считается от baud, а не фиксированные 5 мс
 *   - UART дренируется без SD-операций, запись на карту — пачками
 *   - таймстампы с миллисекундами
 *   - CRC16 проверяется и помечается
 *   - нет String в горячем пути, есть контроль свободного места
 *   - разбивка лога по времени: выкл / 1 / 3 / 5 / 10 / 15 / 30 / 60 мин
 */

#define EB_FAST_TIME 120

#include <Wire.h>
#include <string.h>
#include <GyverOLED.h>
#include <EncButton.h>
#include <HardwareSerial.h>
#include <SD.h>
#include <SPI.h>
#include <GyverDS3231.h>

// ===== Пины =====
#define S1_PIN        0
#define S2_PIN        1
#define KEY_PIN       2
#define SD_CS_PIN     7
#define LCD_SDA_PIN   8
#define LCD_SCL_PIN   9
#define MODBUS_RX_PIN 20
#define MODBUS_TX_PIN 21
#define GPS_RX_PIN    3
#define GPS_TX_PIN    10

// ===== Параметры логгера =====
#define FRAME_MAX       256             // максимальный кадр Modbus RTU
#define OUT_BUF         1024            // текст, накопленный перед записью на SD
#define RX_BUF_SIZE     2048            // UART RX (на 115200 это ~180 мс)
#define KEEP_FREE_BYTES 134217728ULL    // 128 МБ: ниже — удаляем самые старые логи
#define STOP_FREE_BYTES 8388608ULL      // 8 МБ: ниже — аварийно останавливаем запись
#define FLUSH_MS        2000            // период logFile.flush()
#define TZ_OFFSET_MIN   180             // часовой пояс для GPS: UTC+3 (Москва)
#define CLOCK_POLL_MS   20              // как часто можно читать RTC (мс)

// ===== Объекты =====
GyverOLED<SSD1306_128x64, OLED_I2C> oled;
EncButton enc(S1_PIN, S2_PIN, KEY_PIN, INPUT_PULLUP);
GyverDS3231Min rtc;
File logFile;

// ===== Массивы настроек =====
const uint32_t BAUD_RATES[] = {
  300, 600, 1200, 2400, 4800, 9600,
  14400, 19200, 28800, 38400, 57600, 115200
};
const uint8_t BAUD_COUNT = sizeof(BAUD_RATES) / sizeof(BAUD_RATES[0]);

const uint32_t SERIAL_CONFIGS[] = {
  SERIAL_7N1, SERIAL_8N1, SERIAL_7N2, SERIAL_8N2
};
const char* SERIAL_CONFIG_NAMES[] = {
  "SER_7N1", "SER_8N1", "SER_7N2", "SER_8N2"
};
const uint8_t SERIAL_CFG_COUNT = 4;

// Варианты разбивки лога по времени (0 = один файл на сессию)
const uint16_t ROT_MINUTES[] = {0, 1, 3, 5, 10, 15, 30, 60};
const uint8_t  ROT_COUNT = sizeof(ROT_MINUTES) / sizeof(ROT_MINUTES[0]);

// ===== Настройки =====
uint8_t baudIdx    = 5;   // 9600
uint8_t gapExtraMs = 1;   // запас сверх t3.5, мс
uint8_t serCfgIdx  = 1;   // SERIAL_8N1
uint8_t rotIdx     = 0;   // индекс в ROT_MINUTES

// ===== Состояние =====
int8_t  menuCursor   = 0;
bool    loggingActive = false;
char    rtcCache[10]     = "00:00:00";
char    rtcDateCache[9]  = "00.00.00";

uint32_t t35us = 5000;       // межкадровый таймаут (мкс), считается в initSerial1()

// Приём кадра
uint8_t  rxBuf[FRAME_MAX];
uint16_t rxLen   = 0;
uint32_t rxLast  = 0;        // micros() последнего принятого байта
uint32_t rxStart = 0;        // micros() первого байта кадра
bool     rxOvf   = false;    // кадр переполнил FRAME_MAX

// Накопитель текста перед записью на SD
char     outBuf[OUT_BUF];
uint16_t outLen = 0;

uint32_t statPkts = 0;
uint32_t statBad  = 0;
uint32_t fileStartMs = 0;    // millis() открытия текущего лог-файла

// Кеш времени с фазой секунды
struct {
  uint8_t  h, m, s;
  uint32_t phase;      // millis() в момент обнаружения смены секунды
  uint32_t lastPoll;
  bool     valid;
} clk = {0, 0, 0, 0, 0, false};

// ===== Прототипы =====
void drawTimeBar();
void drawLine(uint8_t y);
void drawMenu(const char* items[], uint8_t N, int8_t cur);
bool navPoll(int8_t& dir, int32_t& lastCnt);
void flushEncoder();
bool initSD();
bool initRTC();
void initSerial1();
String makeFileName();
const char* getRTCTimeStr();
void clockPoll(bool uartIdle);
void clockNow(char* out, size_t n);
void drainUart();
void finishFrame();
void frameToText();
void outAppend(const char* s, uint16_t len);
void outFlushToSd();
bool ensureSpace();
bool deleteOldestLog();
bool openLogFile(String& nameOut);
void drawRecording(const String& name);
void enterMainMenu();
void enterTimeSettings();
void setTimeManually();
bool getTimeFromGPS();
void enterPortSettings();
void selectBaudRate();
void selectGapExtra();
void selectSerialConfig();
void enterRecSettings();
void selectRotation();
void enterRecording();
void enterViewLogs();
void viewFileHex(File f);
void viewFileParsed(File f);
uint16_t parsePacketAt(File& f, uint32_t pos, uint8_t* data, uint16_t maxLen);
const char* getFuncName(uint8_t fc);
const char* getExcName(uint8_t code);
void shiftDate(int& year, int& month, int& day, int delta);

// ===== Утилиты =====

// Сдвиг даты на delta дней (для перевода UTC -> локальное время)
void shiftDate(int& year, int& month, int& day, int delta) {
  static const uint8_t mdays[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
  while (delta > 0) {
    uint8_t dm = mdays[month - 1];
    if (month == 2 && (year % 4 == 0)) dm = 29;
    if (++day > dm) { day = 1; if (++month > 12) { month = 1; year++; } }
    delta--;
  }
  while (delta < 0) {
    if (--day < 1) {
      if (--month < 1) { month = 12; year--; }
      uint8_t dm = mdays[month - 1];
      if (month == 2 && (year % 4 == 0)) dm = 29;
      day = dm;
    }
    delta++;
  }
}

// CRC16 Modbus (poly 0xA001, init 0xFFFF)
uint16_t modbusCrc(const uint8_t* d, uint16_t n) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (uint8_t j = 0; j < 8; j++)
      crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
  }
  return crc;
}

bool modbusCrcOk(const uint8_t* d, uint16_t n) {
  if (n < 4) return false;
  uint16_t crc = modbusCrc(d, n - 2);
  return (uint8_t)(crc & 0xFF) == d[n - 2] && (uint8_t)(crc >> 8) == d[n - 1];
}

// Извлечь один байт из двух hex-символов
bool hexByte(const char* s, uint8_t& out) {
  uint8_t v = 0;
  for (uint8_t i = 0; i < 2; i++) {
    char c = s[i];
    if (c >= '0' && c <= '9')      v = (v << 4) | (c - '0');
    else if (c >= 'A' && c <= 'F') v = (v << 4) | (c - 'A' + 10);
    else if (c >= 'a' && c <= 'f') v = (v << 4) | (c - 'a' + 10);
    else return false;
  }
  out = v;
  return true;
}

// Сброс подвисших событий энкодера при входе в меню
void flushEncoder() {
  enc.tick();
  enc.turn();
  enc.click();
  enc.tick();
  enc.turn();
  enc.click();
}

// ===== Инициализация =====
bool initSD() {
  if (!SD.begin(SD_CS_PIN)) return false;
  if (SD.cardType() == CARD_NONE) return false;
  if (!SD.exists("/logs")) SD.mkdir("/logs");
  return true;
}

bool initRTC() {
  rtc.begin();
  if (rtc.isReset()) rtc.setBuildTime();
  clk.valid = false;
  return rtc.isOK();
}

void initSerial1() {
  uint32_t baud = BAUD_RATES[baudIdx];

  // t3.5 = 3.5 символа по 11 бит; для baud > 19200 спека фиксирует 1.75 мс
  float t35 = 3.5f * 11.0f * 1000000.0f / (float)baud;
  if (t35 < 1750.0f) t35 = 1750.0f;
  t35us = (uint32_t)t35 + (uint32_t)gapExtraMs * 1000UL;

  Serial1.end();
  Serial1.setRxBufferSize(RX_BUF_SIZE);   // до begin()!
  Serial1.begin(baud, SERIAL_CONFIGS[serCfgIdx],
                MODBUS_RX_PIN, MODBUS_TX_PIN);

  rxLen = 0;
  rxOvf = false;
}

String makeFileName() {
  Datime t = rtc.getTime();
  for (uint8_t r = 0; r < 3; r++) {
    if (t.year > 2000) break;
    delay(2);
    t = rtc.getTime();
  }

  char buf[56];
  for (uint8_t k = 0; k < 10; k++) {
    if (k == 0)
      snprintf(buf, sizeof(buf), "/logs/%04d-%02d-%02d_%02d-%02d-%02d_log.txt",
               t.year, t.month, t.day, t.hour, t.minute, t.second);
    else
      snprintf(buf, sizeof(buf), "/logs/%04d-%02d-%02d_%02d-%02d-%02d_%d_log.txt",
               t.year, t.month, t.day, t.hour, t.minute, t.second, k);
    if (!SD.exists(buf)) break;
  }
  return String(buf);
}

// ===== Часы =====

const char* getRTCTimeStr() {
  for (uint8_t retry = 0; retry < 3; retry++) {
    Datime dt = rtc.getTime();
    if (dt.year >= 2000) {
      snprintf(rtcCache,     sizeof(rtcCache),     "%02d:%02d:%02d",
               dt.hour, dt.minute, dt.second);
      snprintf(rtcDateCache, sizeof(rtcDateCache), "%02d.%02d.%02d",
               dt.day, dt.month, dt.year % 100);
      return rtcCache;
    }
    delay(1);
  }
  return rtcCache;
}

/*
 * Держим кеш времени и фазу секунды. Читаем RTC только когда шина
 * простаивает (или раз в 5 с при совсем странном потоке): I2C-транзакция
 * посреди кадра испортила бы измерение межбайтовых пауз.
 */
void clockPoll(bool uartIdle) {
  uint32_t now = millis();
  if ((uint32_t)(now - clk.lastPoll) < CLOCK_POLL_MS) return;

  bool force = !clk.valid || (uint32_t)(now - clk.phase) > 5000;
  if (!uartIdle && !force) return;

  clk.lastPoll = now;
  Datime dt = rtc.getTime();
  if (dt.year < 2000) return;

  clk.h = dt.hour;
  clk.m = dt.minute;
  if (!clk.valid || dt.second != clk.s) {
    clk.s     = dt.second;
    clk.phase = now;          // граница секунды с точностью до CLOCK_POLL_MS
    clk.valid = true;
  }
}

void clockNow(char* out, size_t n) {
  uint32_t sub = clk.valid ? (uint32_t)((millis() - clk.phase) % 1000) : 0;
  snprintf(out, n, "%02u:%02u:%02u.%03lu",
           (unsigned)clk.h, (unsigned)clk.m, (unsigned)clk.s,
           (unsigned long)sub);
}

// ===== Приём и запись =====

void outFlushToSd() {
  if (!logFile || outLen == 0) return;
  logFile.write((const uint8_t*)outBuf, outLen);
  outLen = 0;
}

void outAppend(const char* s, uint16_t len) {
  if (!logFile) return;
  if ((uint32_t)outLen + len > OUT_BUF) outFlushToSd();
  if (len > OUT_BUF) {                       // теоретически недостижимо
    logFile.write((const uint8_t*)s, len);
    return;
  }
  memcpy(outBuf + outLen, s, len);
  outLen += len;
}

// Форматирование текущего кадра в текст (только RAM, без SD)
void frameToText() {
  if (rxLen < 4) return;                     // огрызок/шум
  if (!logFile) return;

  bool ok = modbusCrcOk(rxBuf, rxLen);

  char ts[16];
  clockNow(ts, sizeof(ts));

  static char line[900];
  uint16_t p = 0;
  p += snprintf(line + p, sizeof(line) - p, "%s [", ts);

  static const char hexd[] = "0123456789ABCDEF";
  for (uint16_t i = 0; i < rxLen && p + 4 < sizeof(line); i++) {
    if (i) line[p++] = ' ';
    line[p++] = hexd[rxBuf[i] >> 4];
    line[p++] = hexd[rxBuf[i] & 0x0F];
  }
  if (p + 10 < sizeof(line)) {
    line[p++] = ']';
    if (!ok)   { memcpy(line + p, " bad", 4); p += 4; }
    if (rxOvf) { memcpy(line + p, " ovf", 4); p += 4; }
    line[p++] = '\n';
  }

  outAppend(line, p);
  statPkts++;
  if (!ok) statBad++;
}

void finishFrame() {
  frameToText();
  rxLen = 0;
  rxOvf = false;
}

/*
 * Дренаж UART: только read() + micros(), никаких SD/I2C операций.
 * Границы кадров — по паузе t3.5.
 */
void drainUart() {
  while (Serial1.available()) {
    uint8_t  b   = Serial1.read();
    uint32_t now = micros();

    if (rxLen == 0) {
      rxStart = now;
      rxLast  = now;
    } else if ((uint32_t)(now - rxLast) > t35us) {
      finishFrame();
      rxStart = now;
      rxLast  = now;
    } else {
      rxLast = now;
    }

    if (rxLen < FRAME_MAX) rxBuf[rxLen++] = b;
    else                   rxOvf = true;

    // Страховка: поток без пауз (неверный baud) не должен висеть вечно
    if (rxOvf && (uint32_t)(now - rxStart) > 100000UL) finishFrame();
  }
}

// ===== SD: место и ротация =====

bool deleteOldestLog() {
  File dir = SD.open("/logs");
  if (!dir || !dir.isDirectory()) { if (dir) dir.close(); return false; }

  String oldest;
  File f;
  while ((f = dir.openNextFile())) {
    if (!f.isDirectory()) {
      String n = f.name();
      if (oldest.length() == 0 || n < oldest) oldest = n;
    }
    f.close();
  }
  dir.close();

  if (oldest.length() == 0) return false;
  String path = oldest.startsWith("/") ? oldest : String("/logs/") + oldest;
  return SD.remove(path);
}

bool ensureSpace() {
  for (uint8_t i = 0; i < 8; i++) {
    uint64_t total = SD.totalBytes();
    uint64_t used  = SD.usedBytes();
    if (total == 0) return false;
    if (total - used >= KEEP_FREE_BYTES) return true;
    if (!deleteOldestLog()) return false;
  }
  return false;
}

// Открыть новый лог-файл; nameOut — короткое имя для экрана
bool openLogFile(String& nameOut) {
  String fname = makeFileName();
  logFile = SD.open(fname, FILE_WRITE);
  if (!logFile) return false;
  nameOut = fname.substring(6, 22);
  fileStartMs = millis();
  return true;
}

// ===== Отрисовка =====

void drawTimeBar() {
  getRTCTimeStr();
  oled.setCursor(0, 0);
  oled.print(rtcDateCache);
  oled.print(" ");
  oled.print(rtcCache);
}

void drawLine(uint8_t y) {
  oled.line(0, y, 127, y);
}

void drawMenu(const char* items[], uint8_t N, int8_t cur) {
  oled.clear();
  oled.home();
  drawTimeBar();
  drawLine(10);

  for (uint8_t i = 0; i < N; i++) {
    oled.setCursor(0, 2 + i);
    oled.print(i == cur ? "> " : "  ");
    oled.print(items[i]);
  }
  oled.update();
}

void drawRecording(const String& name) {
  oled.clear();
  oled.home();
  oled.println(F("ИДЕТ ЗАПИСЬ"));
  drawLine(10);
  oled.setCursor(0, 2); oled.print(F("Файл:"));
  oled.setCursor(0, 3); oled.print(name);
  oled.setCursor(0, 4); oled.print(F("Пакетов: ")); oled.print(statPkts);
  oled.print(F(" !")); oled.print(statBad);
  oled.setCursor(0, 5);
  oled.print(F("Смена: "));
  if (ROT_MINUTES[rotIdx] == 0) {
    oled.print(F("выкл"));
  } else {
    oled.print(ROT_MINUTES[rotIdx]);
    oled.print(F(" мин"));
  }
  oled.setCursor(0, 6); oled.print(F("Кнопка - стоп"));
  oled.update();
}

// ===== Навигация =====

bool navPoll(int8_t& dir, int32_t& lastCnt) {
  enc.tick();

  if (enc.counter != lastCnt) {
    int32_t d = enc.counter - lastCnt;
    lastCnt = enc.counter;
    dir = (d > 0) ? 1 : -1;
    return true;
  }

  if (enc.click()) {
    dir = 99;
    lastCnt = enc.counter;
    return true;
  }

  return false;
}

// ===== ЭКРАН 1: Главное меню =====

void enterMainMenu() {
  const uint8_t N = 5;
  const char* items[N] = {
    "Настройка времени",
    "Настройки порта",
    "Настройки записи",
    "Начать запись",
    "Посмотреть лог"
  };

  flushEncoder();
  int32_t lastCnt  = enc.counter;
  uint32_t lastDraw = millis();

  while (1) {
    drawMenu(items, N, menuCursor);
    lastDraw = millis();

    while (1) {
      int8_t dir = 0;
      if (navPoll(dir, lastCnt)) {
        if (dir == 1)       menuCursor = (menuCursor + 1) % N;
        else if (dir == -1) menuCursor = (menuCursor == 0) ? N - 1 : menuCursor - 1;
        else if (dir == 99) {
          switch (menuCursor) {
            case 0: enterTimeSettings(); break;
            case 1: enterPortSettings(); break;
            case 2: enterRecSettings();  break;
            case 3: enterRecording();    break;
            case 4: enterViewLogs();     break;
          }
        }
        break;
      }
      if (millis() - lastDraw >= 1000) break;
      yield();
    }
  }
}

// ===== ЭКРАН 2: Настройки времени =====

void enterTimeSettings() {
  const uint8_t N = 3;
  const char* items[N] = {
    "Задать время",
    "Получить через GPS",
    "Вернуться назад"
  };

  flushEncoder();
  int32_t lastCnt = enc.counter;
  int8_t cur = 0;

  while (1) {
    drawMenu(items, N, cur);

    while (1) {
      int8_t dir = 0;
      if (!navPoll(dir, lastCnt)) { yield(); continue; }

      if (dir == 1)       cur = (cur + 1) % N;
      else if (dir == -1) cur = (cur == 0) ? N - 1 : cur - 1;
      else if (dir == 99) {
        switch (cur) {
          case 0: setTimeManually(); break;
          case 1: getTimeFromGPS();  break;
          case 2: return;
        }
      }
      break;
    }
  }
}

// ВАЖНО: RTC хранит ЛОКАЛЬНОЕ время (как и часы на Pi), чтобы метки
// в логе и в pcap совпадали без пересчёта.
void setTimeManually() {
  Datime t = rtc.getTime();
  int yr = t.year, mo = t.month, dy = t.day;
  int hr = t.hour, mn = t.minute, sc = t.second;
  int* fields[]   = {&hr, &mn, &sc, &dy, &mo, &yr};
  const char* labels[] = {"Час:", "Мин:", "Сек:", "День:", "Мес:", "Год:"};
  uint8_t field = 0;
  const uint8_t FC = 6;

  flushEncoder();
  int32_t lastCnt = enc.counter;

  while (1) {
    oled.clear();
    oled.home();
    oled.println(F("Задать время:"));
    drawLine(10);

    for (uint8_t i = 0; i < FC; i++) {
      oled.setCursor(0, 2 + i);
      oled.print(labels[i]);
      if (*fields[i] < 10) oled.print("0");
      oled.print(*fields[i]);
    }
    oled.setCursor(0, 2 + field);
    oled.print(">");
    oled.update();

    while (1) {
      int8_t dir = 0;
      if (!navPoll(dir, lastCnt)) { yield(); continue; }

      if (dir == 1 || dir == -1) {
        *fields[field] += dir;
        if (field == 0 && *fields[field] > 23) *fields[field] = 0;
        else if (field == 0 && *fields[field] < 0) *fields[field] = 23;
        else if ((field == 1 || field == 2) && *fields[field] > 59) *fields[field] = 0;
        else if ((field == 1 || field == 2) && *fields[field] < 0) *fields[field] = 59;
        else if (field == 3 && *fields[field] > 31) *fields[field] = 1;
        else if (field == 3 && *fields[field] < 1) *fields[field] = 31;
        else if (field == 4 && *fields[field] > 12) *fields[field] = 1;
        else if (field == 4 && *fields[field] < 1) *fields[field] = 12;
        else if (field == 5 && *fields[field] > 2099) *fields[field] = 2000;
        else if (field == 5 && *fields[field] < 2000) *fields[field] = 2099;
      } else if (dir == 99) {
        field++;
        if (field >= FC) {
          rtc.setTime(yr, mo, dy, hr, mn, sc);
          clk.valid = false;
          return;
        }
      }
      break;
    }
  }
}

// ===== GPS: время (NMEA RMC отдаёт UTC -> переводим в локальное) =====

bool getTimeFromGPS() {
  Serial1.end();
  Serial1.setRxBufferSize(512);
  Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  oled.clear();
  oled.home();
  oled.println(F("GPS: ожидание..."));
  oled.update();

  uint32_t start = millis();
  char nmea[128];
  uint8_t idx = 0;

  while (millis() - start < 30000) {
    enc.tick();
    if (enc.click()) { initSerial1(); return false; }

    while (Serial1.available()) {
      char c = Serial1.read();
      if (c == '\n') {
        nmea[idx] = 0;
        if (!strncmp(nmea, "$GPRMC", 6) || !strncmp(nmea, "$GNRMC", 6)) {
          char* f[13];
          uint8_t nf = 0;
          f[nf++] = nmea;
          for (char* p = nmea; *p && nf < 13; p++)
            if (*p == ',') { *p = 0; f[nf++] = p + 1; }

          // f[1]=hhmmss.ss, f[2]=A/V, f[9]=ddmmyy
          if (nf >= 10 && f[2][0] == 'A' &&
              strlen(f[1]) >= 6 && strlen(f[9]) >= 6) {
            int h = (f[1][0]-'0')*10 + (f[1][1]-'0');
            int m = (f[1][2]-'0')*10 + (f[1][3]-'0');
            int s = (f[1][4]-'0')*10 + (f[1][5]-'0');
            int day   = (f[9][0]-'0')*10 + (f[9][1]-'0');
            int month = (f[9][2]-'0')*10 + (f[9][3]-'0');
            int year  = 2000 + (f[9][4]-'0')*10 + (f[9][5]-'0');

            int total = h * 60 + m + TZ_OFFSET_MIN;
            int shift = 0;
            while (total < 0)     { total += 1440; shift--; }
            while (total >= 1440) { total -= 1440; shift++; }
            shiftDate(year, month, day, shift);

            rtc.setTime(year, month, day, total / 60, total % 60, s);
            clk.valid = false;
            initSerial1();

            oled.clear(); oled.home();
            oled.println(F("GPS: время получено!"));
            oled.setCursor(0, 2);
            oled.print(rtc.getTime().toString());
            oled.update();
            delay(1500);
            return true;
          }
        }
        idx = 0;
      } else if (c != '\r' && idx < 127) {
        nmea[idx++] = c;
      }
    }
    yield();
  }

  initSerial1();
  oled.clear(); oled.home();
  oled.println(F("GPS: таймаут"));
  oled.update();
  delay(1500);
  return false;
}

// ===== ЭКРАН 3: Настройки порта =====

void enterPortSettings() {
  const uint8_t N = 4;
  const char* items[N] = {
    "Скорость обмена",
    "Запас t3.5",
    "Данные/Чет/Стоп",
    "Вернуться назад"
  };

  flushEncoder();
  int32_t lastCnt = enc.counter;
  int8_t cur = 0;

  while (1) {
    drawMenu(items, N, cur);

    while (1) {
      int8_t dir = 0;
      if (!navPoll(dir, lastCnt)) { yield(); continue; }

      if (dir == 1)       cur = (cur + 1) % N;
      else if (dir == -1) cur = (cur == 0) ? N - 1 : cur - 1;
      else if (dir == 99) {
        switch (cur) {
          case 0: selectBaudRate();    break;
          case 1: selectGapExtra();    break;
          case 2: selectSerialConfig(); break;
          case 3: return;
        }
        initSerial1();
      }
      break;
    }
  }
}

void selectBaudRate() {
  int8_t sel = baudIdx;
  flushEncoder();
  int32_t lastCnt = enc.counter;

  while (1) {
    oled.clear(); oled.home();
    oled.println(F("Скорость обмена"));
    drawLine(10);
    oled.setCursor(0, 3);
    oled.print(F("  < "));
    oled.print(BAUD_RATES[sel]);
    oled.print(F(" >"));
    oled.update();

    while (1) {
      int8_t dir = 0;
      if (!navPoll(dir, lastCnt)) { yield(); continue; }
      if (dir == 1 && sel < BAUD_COUNT - 1) sel++;
      if (dir == -1 && sel > 0) sel--;
      if (dir == 99) { baudIdx = sel; return; }
      break;
    }
  }
}

void selectGapExtra() {
  int8_t sel = gapExtraMs;
  flushEncoder();
  int32_t lastCnt = enc.counter;

  while (1) {
    // t3.5 для текущей скорости (без запаса)
    float t35 = 3.5f * 11.0f * 1000000.0f / (float)BAUD_RATES[baudIdx];
    if (t35 < 1750.0f) t35 = 1750.0f;

    oled.clear(); oled.home();
    oled.println(F("Запас t3.5"));
    drawLine(10);
    oled.setCursor(0, 2);
    oled.print(F("t3.5 = "));
    oled.print((uint32_t)t35);
    oled.print(F(" us"));
    oled.setCursor(0, 3);
    oled.print(F("  < +"));
    oled.print(sel);
    oled.print(F(" ms >"));
    oled.update();

    while (1) {
      int8_t dir = 0;
      if (!navPoll(dir, lastCnt)) { yield(); continue; }
      if (dir == 1 && sel < 15) sel++;
      if (dir == -1 && sel > 0) sel--;
      if (dir == 99) { gapExtraMs = sel; return; }
      break;
    }
  }
}

void selectSerialConfig() {
  int8_t sel = serCfgIdx;
  flushEncoder();
  int32_t lastCnt = enc.counter;

  while (1) {
    oled.clear(); oled.home();
    oled.println(F("Данные/Чет/Стоп"));
    drawLine(10);
    oled.setCursor(0, 3);
    oled.print(F("  < "));
    oled.print(SERIAL_CONFIG_NAMES[sel]);
    oled.print(F(" >"));
    oled.update();

    while (1) {
      int8_t dir = 0;
      if (!navPoll(dir, lastCnt)) { yield(); continue; }
      if (dir == 1 && sel < SERIAL_CFG_COUNT - 1) sel++;
      if (dir == -1 && sel > 0) sel--;
      if (dir == 99) { serCfgIdx = sel; return; }
      break;
    }
  }
}

// ===== ЭКРАН 3б: Настройки записи =====

void enterRecSettings() {
  const uint8_t N = 2;
  const char* items[N] = {
    "Разбивка файла",
    "Вернуться назад"
  };

  flushEncoder();
  int32_t lastCnt = enc.counter;
  int8_t cur = 0;

  while (1) {
    drawMenu(items, N, cur);

    while (1) {
      int8_t dir = 0;
      if (!navPoll(dir, lastCnt)) { yield(); continue; }

      if (dir == 1)       cur = (cur + 1) % N;
      else if (dir == -1) cur = (cur == 0) ? N - 1 : cur - 1;
      else if (dir == 99) {
        switch (cur) {
          case 0: selectRotation(); break;
          case 1: return;
        }
      }
      break;
    }
  }
}

void selectRotation() {
  int8_t sel = rotIdx;
  flushEncoder();
  int32_t lastCnt = enc.counter;

  while (1) {
    oled.clear(); oled.home();
    oled.println(F("Разбивка файла"));
    drawLine(10);
    oled.setCursor(0, 3);
    oled.print(F("  < "));
    if (ROT_MINUTES[sel] == 0) {
      oled.print(F("выкл"));
    } else {
      oled.print(ROT_MINUTES[sel]);
      oled.print(F(" мин"));
    }
    oled.print(F(" >"));
    oled.setCursor(0, 5);
    oled.print(F("0 = один файл"));
    oled.update();

    while (1) {
      int8_t dir = 0;
      if (!navPoll(dir, lastCnt)) { yield(); continue; }
      if (dir == 1 && sel < ROT_COUNT - 1) sel++;
      if (dir == -1 && sel > 0) sel--;
      if (dir == 99) { rotIdx = sel; return; }
      break;
    }
  }
}

// ===== ЭКРАН 4: Запись лога =====

void enterRecording() {
  if (loggingActive) return;

  if (!ensureSpace()) {
    oled.clear(); oled.home();
    oled.println(F("Нет места на SD"));
    oled.update();
    delay(2000);
    return;
  }

  String dispName;
  if (!openLogFile(dispName)) {
    oled.clear(); oled.home();
    oled.println(F("Ошибка открытия"));
    oled.update();
    delay(2000);
    return;
  }

  loggingActive = true;
  statPkts = 0;
  statBad  = 0;
  rxLen    = 0;
  rxOvf    = false;
  outLen   = 0;
  clk.valid = false;

  initSerial1();
  clockPoll(true);
  flushEncoder();

  uint32_t lastDisp  = 0;
  uint32_t lastFlush = millis();
  uint32_t lastSpace = millis();
  bool     diskFull  = false;
  bool     writeErr  = false;

  drawRecording(dispName);

  while (1) {
    enc.tick();
    if (enc.click()) break;

    drainUart();

    // Кадр оборвался — закрываем его по таймауту
    if (rxLen > 0 && (uint32_t)(micros() - rxLast) > t35us) finishFrame();

    // Дальше — только когда кадр закрыт и шина молчит
    bool idle = (rxLen == 0) && !Serial1.available();
    if (idle) {
      clockPoll(true);

      if (outLen) outFlushToSd();

      // Разбивка по времени: новый файл в паузе между кадрами
      uint16_t rot = ROT_MINUTES[rotIdx];
      if (rot && (uint32_t)(millis() - fileStartMs) >= (uint32_t)rot * 60000UL) {
        outFlushToSd();
        logFile.flush();
        logFile.close();
        if (!ensureSpace() || !openLogFile(dispName)) {
          writeErr = true;
          break;
        }
        lastFlush = millis();
        drawRecording(dispName);
      }

      if (millis() - lastFlush >= FLUSH_MS) {
        outFlushToSd();
        logFile.flush();
        lastFlush = millis();
      }

      // Раз в 10 с проверяем место: лучше остановиться самим,
      // чем получить битый файл на переполненной карте
      if (millis() - lastSpace >= 10000) {
        lastSpace = millis();
        uint64_t tot = SD.totalBytes();
        uint64_t usd = SD.usedBytes();
        if (tot && tot - usd < STOP_FREE_BYTES) { diskFull = true; break; }
      }

      if (millis() - lastDisp >= 1000) {
        lastDisp = millis();
        drawRecording(dispName);
      }
    } else {
      clockPoll(false);
    }

    yield();
  }

  // Корректная остановка
  outFlushToSd();
  logFile.flush();
  logFile.close();
  loggingActive = false;

  oled.clear(); oled.home();
  if (diskFull)      oled.println(F("SD заполнена!"));
  else if (writeErr) oled.println(F("Ошибка записи"));
  else               oled.println(F("Лог остановлен"));
  oled.setCursor(0, 2); oled.print(F("Пакетов: ")); oled.print(statPkts);
  oled.setCursor(0, 3); oled.print(F("Ошибок:  ")); oled.print(statBad);
  oled.setCursor(0, 4); oled.print(F("Файл: "));    oled.print(dispName);
  oled.update();
  delay(2000);
}

// ===== ЭКРАН 5: Просмотр логов =====

void enterViewLogs() {
  if (loggingActive) return;

  File dir = SD.open("/logs");
  if (!dir || !dir.isDirectory()) {
    oled.clear(); oled.home();
    oled.println(F("Нет папки /logs"));
    oled.update();
    delay(1500);
    if (dir) dir.close();
    return;
  }

  const uint8_t MAX_FILES = 50;
  String fileNames[MAX_FILES];
  uint8_t fileCount = 0;
  File f;
  while ((f = dir.openNextFile())) {
    if (!f.isDirectory() && fileCount < MAX_FILES) {
      String name = f.name();
      if (name.startsWith("/logs/")) name = name.substring(6);
      fileNames[fileCount++] = name;
    }
    f.close();
  }
  dir.close();

  if (fileCount == 0) {
    oled.clear(); oled.home();
    oled.println(F("Нет лог-файлов"));
    oled.update();
    delay(1500);
    return;
  }

  // Сортировка по имени = хронологический порядок
  for (uint8_t i = 1; i < fileCount; i++) {
    String key = fileNames[i];
    int8_t j = i - 1;
    while (j >= 0 && fileNames[j] > key) {
      fileNames[j + 1] = fileNames[j];
      j--;
    }
    fileNames[j + 1] = key;
  }

  flushEncoder();
  int32_t lastCnt = enc.counter;
  int16_t cursor = 0;
  const uint8_t VIS = 4;

  while (1) {
    uint8_t start = (cursor < VIS - 1) ? 0 : cursor - (VIS - 2);
    if (start + VIS > fileCount + 1)
      start = (fileCount + 1 > VIS) ? fileCount + 1 - VIS : 0;

    oled.clear(); oled.home();
    oled.println(F("Лог-файлы:"));
    drawLine(10);

    oled.setCursor(0, 2);
    oled.print(cursor == 0 ? F("> ..назад") : F("  ..назад"));
    uint8_t line = 3;
    for (uint8_t i = start; i < fileCount && line < 7; i++) {
      oled.setCursor(0, line);
      oled.print((int16_t)i == cursor - 1 ? F("> ") : F("  "));
      oled.print(fileNames[i]);
      line++;
    }
    oled.update();

    while (1) {
      int8_t dir = 0;
      if (!navPoll(dir, lastCnt)) { yield(); continue; }
      if (dir == 1)       cursor = (cursor + 1) % (fileCount + 1);
      else if (dir == -1) cursor = (cursor == 0) ? fileCount : cursor - 1;
      else if (dir == 99) {
        if (cursor == 0) return;
        String path = "/logs/" + fileNames[cursor - 1];
        File lf = SD.open(path, FILE_READ);
        if (lf) {
          const uint8_t VM = 3;
          const char* vItems[VM] = {"Простой вид", "Разбор пакетов", "Назад"};
          int8_t vCur = 0;
          flushEncoder();
          int32_t vCnt = enc.counter;
          bool done = false;
          while (!done) {
            drawMenu(vItems, VM, vCur);
            while (1) {
              int8_t vd = 0;
              if (!navPoll(vd, vCnt)) { yield(); continue; }
              if (vd == 1)       vCur = (vCur + 1) % VM;
              else if (vd == -1) vCur = (vCur == 0) ? VM - 1 : vCur - 1;
              else if (vd == 99) {
                switch (vCur) {
                  case 0: viewFileHex(lf);    done = true; break;
                  case 1: viewFileParsed(lf); done = true; break;
                  case 2: done = true; break;
                }
              }
              break;
            }
          }
          lf.close();
        } else {
          oled.clear(); oled.home();
          oled.println(F("Ошибка чтения!"));
          oled.update();
          delay(1000);
        }
      }
      break;
    }
  }
}

// ===== Простой просмотр файла (hex) =====

void viewFileHex(File f) {
  int32_t fileSize = f.size();
  int32_t viewPos  = 0;

  flushEncoder();
  int32_t lastCnt = enc.counter;

  while (1) {
    oled.clear(); oled.home();
    f.seek(viewPos);
    for (uint8_t row = 0; row < 7 && f.available(); row++) {
      oled.setCursor(0, row);
      char lineBuf[22];
      uint8_t li = 0;
      while (f.available() && li < 21) {
        char c = f.read();
        if (c == '\n' || c == '\r') { if (li == 0) continue; break; }
        lineBuf[li++] = c;
      }
      lineBuf[li] = 0;
      oled.print(lineBuf);
      if (!f.available()) break;
      if (f.peek() == '\r') f.read();
      if (f.peek() == '\n') f.read();
    }
    int32_t nextPos = f.position();
    oled.setCursor(0, 7);
    oled.print(F("Кнопка-вых <- ->"));
    oled.update();

    while (1) {
      int8_t dir = 0;
      if (!navPoll(dir, lastCnt)) { yield(); continue; }
      if (dir == 1) {
        viewPos = nextPos;
        if (viewPos >= fileSize - 64) viewPos = fileSize > 128 ? fileSize - 128 : 0;
      } else if (dir == -1) {
        viewPos -= 512;
        if (viewPos < 0) viewPos = 0;
      } else if (dir == 99) {
        return;
      }
      break;
    }
  }
}

// ===== Имена функций Modbus =====

const char* getFuncName(uint8_t fc) {
  switch (fc) {
    case 0x01: return "Read Coils";
    case 0x02: return "Read Discrete In";
    case 0x03: return "Read Holding Reg";
    case 0x04: return "Read Input Reg";
    case 0x05: return "Write Single Coil";
    case 0x06: return "Write Single Reg";
    case 0x0F: return "Write Mult Coils";
    case 0x10: return "Write Mult Regs";
    default:   return "?";
  }
}

const char* getExcName(uint8_t code) {
  switch (code) {
    case 0x01: return "Illegal Func";
    case 0x02: return "Illegal Addr";
    case 0x03: return "Illegal Value";
    case 0x04: return "Dev Failure";
    case 0x05: return "Ack";
    case 0x06: return "Dev Busy";
    case 0x08: return "Mem Parity";
    case 0x0A: return "Gateway Path";
    case 0x0B: return "Gateway Resp";
    default:   return "?";
  }
}

// Прочитать один пакет по позиции pktPos[idx]
uint16_t parsePacketAt(File& f, uint32_t pos, uint8_t* data, uint16_t maxLen) {
  f.seek(pos);
  bool in = false;
  uint16_t n = 0;
  char hp[3];
  uint8_t hi = 0;

  while (f.available()) {
    char c = f.read();
    if (!in) {
      if (c == '[') in = true;
      continue;
    }
    if (c == ']') break;
    if (c == '\r' || c == '\n') { if (n == 0) continue; break; }
    if (c == ' ') {
      if (hi == 2 && n < maxLen && hexByte(hp, data[n])) n++;
      hi = 0;
    } else if (hi < 2) {
      hp[hi++] = c;
    }
  }
  if (hi == 2 && n < maxLen && hexByte(hp, data[n])) n++;
  return n;
}

// ===== Разбор Modbus-пакетов =====

void viewFileParsed(File f) {
  const uint16_t MAXPKT = 60;
  uint32_t pktPos[MAXPKT];
  uint16_t pktCount = 0;

  // Сканируем файл, ищем строки с '['
  f.seek(0);
  while (f.available() && pktCount < MAXPKT) {
    uint32_t pos = f.position();
    if (f.peek() == '\r' || f.peek() == '\n') { f.read(); continue; }
    bool hasBracket = false;
    while (f.available()) {
      char c = f.read();
      if (c == '[') hasBracket = true;
      if (c == '\n' || c == '\r') break;
    }
    if (hasBracket) pktPos[pktCount++] = pos;
  }
  f.seek(0);

  if (pktCount == 0) {
    oled.clear(); oled.home();
    oled.println(F("Нет Modbus-пакетов"));
    oled.update();
    delay(1500);
    return;
  }

  int16_t pktIdx = 0;
  flushEncoder();
  int32_t lastCnt = enc.counter;

  uint8_t data[FRAME_MAX];
  uint8_t prev[FRAME_MAX];

  while (1) {
    uint16_t dlen  = parsePacketAt(f, pktPos[pktIdx], data, sizeof(data));
    uint16_t plen  = 0;
    if (pktIdx > 0) plen = parsePacketAt(f, pktPos[pktIdx - 1], prev, sizeof(prev));

    oled.clear(); oled.home();

    if (dlen < 4) {
      oled.println(F("Короткий пакет"));
    } else {
      uint8_t fc    = data[1];
      bool    isExc = (fc & 0x80) != 0;
      uint8_t base  = fc & 0x7F;

      // Направление: 0x01-0x04 запрос = 8 байт; 0x0F/0x10 ответ = 8 байт;
      // 0x05/0x06 запрос и ответ совпадают -> сравниваем с предыдущим кадром.
      bool isReq;
      if (isExc) {
        isReq = false;
      } else if (base == 0x01 || base == 0x02 || base == 0x03 || base == 0x04) {
        isReq = (dlen == 8);
      } else if (base == 0x0F || base == 0x10) {
        isReq = (dlen > 8);
      } else if (base == 0x05 || base == 0x06) {
        bool echo = (plen == dlen && plen > 0 &&
                     prev[0] == data[0] &&
                     memcmp(prev, data, dlen) == 0);
        isReq = !echo;
      } else {
        isReq = true;
      }

      oled.setCursor(0, 0);
      oled.print(F("Slave:0x"));
      if (data[0] < 0x10) oled.print("0");
      oled.print(data[0], HEX);
      oled.print(isExc ? F(" EXC") : (isReq ? F(" REQ") : F(" RESP")));

      oled.setCursor(0, 1);
      oled.print(F("Func: 0x"));
      if (fc < 0x10) oled.print("0");
      oled.print(fc, HEX);
      oled.print(" ");
      if (isExc && dlen >= 3) {
        oled.print(getExcName(data[2]));
      } else {
        oled.print(getFuncName(base));
      }

      uint8_t row = 2;

      if (isExc) {
        if (dlen >= 3) {
          oled.setCursor(0, row++);
          oled.print(F("Code: ")); oled.print(data[2]);
        }
      } else if (isReq) {
        if (base == 0x01 || base == 0x02 || base == 0x03 || base == 0x04) {
          if (dlen >= 4) {
            uint16_t addr = (data[2] << 8) | data[3];
            oled.setCursor(0, row++);
            oled.print(F("Addr: 0x")); oled.print(addr, HEX);
            oled.print("("); oled.print(addr); oled.print(")");
          }
          if (dlen >= 6) {
            uint16_t qty = (data[4] << 8) | data[5];
            oled.setCursor(0, row++);
            oled.print(F("Qty: ")); oled.print(qty);
          }
        } else if (base == 0x05 || base == 0x06) {
          if (dlen >= 4) {
            uint16_t reg = (data[2] << 8) | data[3];
            oled.setCursor(0, row++);
            oled.print(F("Reg: 0x")); oled.print(reg, HEX);
            oled.print("("); oled.print(reg); oled.print(")");
          }
          if (dlen >= 6) {
            uint16_t val = (data[4] << 8) | data[5];
            oled.setCursor(0, row++);
            oled.print(F("Val: 0x")); oled.print(val, HEX);
            oled.print("("); oled.print(val); oled.print(")");
          }
        } else if (base == 0x0F || base == 0x10) {
          if (dlen >= 4) {
            uint16_t addr = (data[2] << 8) | data[3];
            oled.setCursor(0, row++);
            oled.print(F("Addr: 0x")); oled.print(addr, HEX);
          }
          if (dlen >= 6) {
            uint16_t qty = (data[4] << 8) | data[5];
            oled.setCursor(0, row++);
            oled.print(F("Qty: ")); oled.print(qty);
          }
          if (dlen >= 7) {
            oled.setCursor(0, row++);
            oled.print(F("Bytes: ")); oled.print(data[6]);
          }
        }
      } else {
        if (base == 0x01 || base == 0x02 || base == 0x03 || base == 0x04) {
          uint8_t bc = data[2];
          oled.setCursor(0, row++);
          oled.print(F("Bytes: ")); oled.print(bc);

          if (bc > 0 && dlen >= 5) {
            oled.setCursor(0, row++);
            oled.print(F("Data: "));
            uint8_t show = bc < 8 ? bc : 6;
            for (uint8_t i = 0; i < show && (uint16_t)(3 + i) < dlen - 2; i++) {
              if (data[3 + i] < 0x10) oled.print("0");
              oled.print(data[3 + i], HEX);
              oled.print(" ");
            }
            if (bc > 6) oled.print("...");
          }
        } else if (base == 0x05 || base == 0x06) {
          if (dlen >= 4) {
            uint16_t reg = (data[2] << 8) | data[3];
            oled.setCursor(0, row++);
            oled.print(F("Reg: 0x")); oled.print(reg, HEX);
          }
          if (dlen >= 6) {
            uint16_t val = (data[4] << 8) | data[5];
            oled.setCursor(0, row++);
            oled.print(F("Val: 0x")); oled.print(val, HEX);
          }
        } else if (base == 0x0F || base == 0x10) {
          if (dlen >= 4) {
            uint16_t addr = (data[2] << 8) | data[3];
            oled.setCursor(0, row++);
            oled.print(F("Addr: 0x")); oled.print(addr, HEX);
          }
          if (dlen >= 6) {
            uint16_t qty = (data[4] << 8) | data[5];
            oled.setCursor(0, row++);
            oled.print(F("Done: ")); oled.print(qty);
          }
        }
      }

      if (row < 7) {
        oled.setCursor(0, row++);
        oled.print(F("CRC: "));
        oled.print(modbusCrcOk(data, dlen) ? F("OK") : F("BAD"));
      }
    }

    oled.setCursor(0, 7);
    oled.print(pktIdx + 1);
    oled.print("/");
    oled.print(pktCount);
    oled.print(F(" <- -> Кн-вых"));
    oled.update();

    while (1) {
      int8_t dir = 0;
      if (!navPoll(dir, lastCnt)) { yield(); continue; }
      if (dir == 1) {
        if (pktIdx < pktCount - 1) pktIdx++;
      } else if (dir == -1) {
        if (pktIdx > 0) pktIdx--;
      } else if (dir == 99) {
        return;
      }
      break;
    }
  }
}

// ===== SETUP / LOOP =====

void setup() {
  Serial.begin(115200);
  Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN);
  Wire.setClock(400000L);

  oled.init();
  oled.clear();
  oled.autoPrintln(true);
  oled.setScale(1);

  oled.println(F("Init..."));
  oled.update();

  oled.print(F("RTC: "));
  oled.println(initRTC() ? F("OK") : F("ERR"));
  oled.update();

  oled.print(F("SD: "));
  oled.println(initSD() ? F("OK") : F("ERR"));
  oled.update();

  initSerial1();
  clockPoll(true);

  enc.setEncType(EB_STEP2);
  enc.counter = 0;
  menuCursor = 0;
  loggingActive = false;
  delay(1000);
}

void loop() {
  enterMainMenu();
}
