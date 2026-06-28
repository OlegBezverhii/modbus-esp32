#define EB_FAST_TIME 120

#include <Wire.h>
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

// ===== Объекты =====
GyverOLED<SSD1306_128x64, OLED_I2C> oled;
EncButton enc(S1_PIN, S2_PIN, KEY_PIN, INPUT_PULLUP);
GyverDS3231Min rtc;

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

// ===== Настройки =====
uint8_t baudIdx   = 5;   // 9600
uint8_t timeoutMs = 5;   // 5ms
uint8_t serCfgIdx = 1;   // SERIAL_8N1

// ===== Состояние =====
int8_t  menuCursor    = 0;
bool    loggingActive = false;
File    logFile;
char    rtcCache[10] = "00:00:00";   // кеш последнего валидного времени RTC

// ===== Прототипы =====
void drawTimeBar();
void drawLine(uint8_t y);
bool initSD();
bool initRTC();
void initSerial1();
String makeFileName();
const char* getRTCTimeStr();

void enterMainMenu();
void enterTimeSettings();
void setTimeManually();
bool getTimeFromGPS();
void enterPortSettings();
void selectBaudRate();
void selectTimeout();
void selectSerialConfig();
void enterRecording();
void enterViewLogs();
void viewFileHex(File f);
void viewFileParsed(File f);
const char* getFuncName(uint8_t fc);

// Сброс подвисших событий энкодера при входе в меню
void flushEncoder() {
  enc.tick();
  enc.turn();
  enc.click();
  enc.tick();
  enc.turn();
  enc.click();
}

// ===== Инициализация RTC / SD / Serial1 =====
bool initSD() {
  if (!SD.begin(SD_CS_PIN)) return false;
  if (SD.cardType() == CARD_NONE) return false;
  if (!SD.exists("/logs")) SD.mkdir("/logs");
  return true;
}
bool initRTC() {
  rtc.begin();
  if (rtc.isReset()) rtc.setBuildTime();
  return rtc.isOK();
}
void initSerial1() {
  Serial1.end();
  Serial1.begin(BAUD_RATES[baudIdx], SERIAL_CONFIGS[serCfgIdx],
                MODBUS_RX_PIN, MODBUS_TX_PIN);
}
String makeFileName() {
  Datime t = rtc.getTime();
  for (uint8_t r = 0; r < 3; r++) {
    if (t.year > 2000) break;
    delay(2);
    t = rtc.getTime();
  }
  char buf[40];
  sprintf(buf, "/logs/%04d-%02d-%02d_%02d-%02d-%02d_log.txt",
          t.year, t.month, t.day, t.hour, t.minute, t.second);
  return String(buf);
}

// ===== Безопасное чтение времени RTC (с повтором при сбое I2C) =====
char rtcDateCache[9] = "00.00.00";

const char* getRTCTimeStr() {
  for (uint8_t retry = 0; retry < 3; retry++) {
    Datime dt = rtc.getTime();
    if (dt.second != 0 || dt.minute != 0 || dt.hour != 0) {
      sprintf(rtcCache,   "%02d:%02d:%02d", dt.hour, dt.minute, dt.second);
      sprintf(rtcDateCache, "%02d.%02d.%02d", dt.day, dt.month, dt.year % 100);
      return rtcCache;
    }
    delay(1);
  }
  return rtcCache;
}

// ===== Отрисовка =====
void drawTimeBar() {
  getRTCTimeStr();  // обновить кеш из RTC
  oled.setCursor(0, 0);
  oled.print(rtcDateCache);
  oled.print(" ");
  oled.print(rtcCache);
}
void drawLine(uint8_t y) {
  oled.line(0, y, 127, y);
}

// Универсальная отрисовка меню (items[N] — список строк)
void drawMenu(const char* items[], uint8_t N, int8_t cur) {
  oled.clear();
  oled.home();
  drawTimeBar();
  drawLine(10);

  for (uint8_t i = 0; i < N; i++) {
    oled.setCursor(0, 2 + i);
    if (i == cur) {
      oled.print("> ");
    } else {
      oled.print("  ");
    }
    oled.print(items[i]);
  }
  oled.update();
}

// ===== Навигация: возвращает true если было действие =====
// dir = выход: -1/0/+1 (поворот), 99 = click
// ВНИМАНИЕ: использует enc.counter для надёжности
bool navPoll(int8_t& dir, int32_t& lastCnt) {
  enc.tick();

  // поворот энкодера
  if (enc.counter != lastCnt) {
    int32_t d = enc.counter - lastCnt;
    lastCnt = enc.counter;
    dir = (d > 0) ? 1 : -1;
    return true;
  }

  // клик кнопкой
  if (enc.click()) {
    dir = 99;
    lastCnt = enc.counter;
    return true;
  }

  return false;
}

// ===== ЭКРАН 1: Главное меню =====
void enterMainMenu() {
  const uint8_t N = 4;
  const char* items[N] = {
    "Настройка времени",
    "Настройки порта",
    "Начать запись",
    "Посмотреть лог"
  };

  flushEncoder();
  int32_t lastCnt = enc.counter;
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
            case 2: enterRecording();    break;
            case 3: enterViewLogs();     break;
          }
        }
        break;
      }
      // обновление часов каждую секунду
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

// ===== Установка времени вручную =====
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
          return;
        }
      }
      break;
    }
  }
}

// ===== GPS: получение времени =====
bool getTimeFromGPS() {
  Serial1.end();
  Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  oled.clear();
  oled.home();
  oled.println(F("GPS: ожидание..."));
  oled.update();

  uint32_t start = millis();
  char nmea[128];
  uint8_t idx = 0;

  while (millis() - start < 30000) {
    // в GPS-режиме кнопка энкодера = отмена
    enc.tick();
    if (enc.click()) {
      initSerial1();
      return false;
    }

    while (Serial1.available()) {
      char c = Serial1.read();
      if (c == '\n') {
        nmea[idx] = 0;
        if (strncmp(nmea, "$GPRMC", 6) == 0 || strncmp(nmea, "$GNRMC", 6) == 0) {
          char* p = nmea;
          for (uint8_t i = 0; i < 9; i++) { p = strchr(p, ','); if (!p) break; p++; }
          if (p && strlen(p) >= 6) {
            char ts[7]; strncpy(ts, p, 6); ts[6] = 0;
            int h = (ts[0]-'0')*10 + (ts[1]-'0');
            int m = (ts[2]-'0')*10 + (ts[3]-'0');
            int s = (ts[4]-'0')*10 + (ts[5]-'0');
            p = strchr(p, ',');
            if (p && strlen(p) > 7) {
              p++;
              char ds[7]; strncpy(ds, p, 6); ds[6] = 0;
              int day   = (ds[0]-'0')*10 + (ds[1]-'0');
              int month = (ds[2]-'0')*10 + (ds[3]-'0');
              int year  = 2000 + (ds[4]-'0')*10 + (ds[5]-'0');
              rtc.setTime(year, month, day, h, m, s);
              initSerial1();
              oled.clear(); oled.home();
              oled.println(F("GPS: время получено!"));
              oled.setCursor(0, 2); oled.print(rtc.getTime().toString());
              oled.update(); delay(1500);
              return true;
            }
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
  oled.update(); delay(1500);
  return false;
}

// ===== ЭКРАН 3: Настройки порта =====
void enterPortSettings() {
  const uint8_t N = 4;
  const char* items[N] = {
    "Скорость обмена",
    "Таймаут м/у сим",
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
          case 1: selectTimeout();     break;
          case 2: selectSerialConfig(); break;
          case 3: return;
        }
        initSerial1();
      }
      break;
    }
  }
}

// ===== Выбор скорости обмена =====
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

// ===== Выбор таймаута =====
void selectTimeout() {
  int8_t sel = timeoutMs;
  flushEncoder();
  int32_t lastCnt = enc.counter;

  while (1) {
    oled.clear(); oled.home();
    oled.println(F("Таймаут м/у сим"));
    drawLine(10);
    oled.setCursor(0, 3);
    oled.print(F("  < "));
    oled.print(sel);
    oled.print(F(" ms >"));
    oled.update();

    while (1) {
      int8_t dir = 0;
      if (!navPoll(dir, lastCnt)) { yield(); continue; }
      if (dir == 1 && sel < 15) sel++;
      if (dir == -1 && sel > 0) sel--;
      if (dir == 99) { timeoutMs = sel; return; }
      break;
    }
  }
}

// ===== Выбор конфигурации Serial =====
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

// ===== ЭКРАН 4: Запись лога =====
void enterRecording() {
  if (loggingActive) return;

  String fname = makeFileName();
  logFile = SD.open(fname, FILE_WRITE);
  if (!logFile) {
    oled.clear(); oled.home();
    oled.println(F("Ошибка открытия")); oled.update();
    delay(2000); return;
  }

  loggingActive = true;
  uint32_t lastByteTime = 0;
  uint8_t  buffer[256];
  uint16_t bufPos = 0;
  uint32_t pktCount = 0;
  String dispName = fname.substring(6, 22);

  initSerial1();
  flushEncoder();

  oled.clear(); oled.home();
  oled.println(F("ИДЕТ ЗАПИСЬ"));
  drawLine(10);
  oled.setCursor(0, 2); oled.print(F("Файл:"));
  oled.setCursor(0, 3); oled.print(dispName);
  oled.setCursor(0, 5); oled.print(F("Кнопка - стоп"));
  uint32_t lastDisp = millis();

  while (1) {
    enc.tick();
    if (enc.click()) {
      loggingActive = false;
      logFile.close();
      oled.clear(); oled.home();
      oled.println(F("Лог остановлен"));
      oled.setCursor(0, 2); oled.print(F("Пакетов: ")); oled.print(pktCount);
      oled.setCursor(0, 3); oled.print(F("Файл: "));    oled.print(dispName);
      oled.update(); delay(2000);
      return;
    }

    while (Serial1.available()) {
      uint8_t b = Serial1.read();
      uint32_t now = micros();
      if (bufPos == 0 ||
          (uint32_t)(now - lastByteTime) <= ((uint32_t)timeoutMs * 1000)) {
        buffer[bufPos++] = b;
      } else {
        if (bufPos >= 4) {
          String line = getRTCTimeStr();
          line += " [";
          for (uint16_t i = 0; i < bufPos; i++) {
            if (buffer[i] < 0x10) line += "0";
            line += String(buffer[i], HEX);
            if (i < bufPos - 1) line += " ";
          }
          line += "]";
          logFile.println(line); logFile.flush(); pktCount++;
        }
        buffer[0] = b; bufPos = 1;
      }
      lastByteTime = now;
    }

    if (bufPos > 0 &&
        (uint32_t)(micros() - lastByteTime) > ((uint32_t)timeoutMs * 1000)) {
      if (bufPos >= 4) {
        String line = getRTCTimeStr();
        line += " [";
        for (uint16_t i = 0; i < bufPos; i++) {
          if (buffer[i] < 0x10) line += "0";
          line += String(buffer[i], HEX);
          if (i < bufPos - 1) line += " ";
        }
        line += "]";
        logFile.println(line); logFile.flush(); pktCount++;
      }
      bufPos = 0;
    }

    if (millis() - lastDisp > 2000) {
      lastDisp = millis();
      oled.setCursor(0, 4); oled.print(F("Пакетов: "));
      oled.print(pktCount); oled.print(F("   "));
      getRTCTimeStr();
      oled.setCursor(0, 0); oled.print(rtcDateCache);
      oled.print(" "); oled.print(rtcCache);
      oled.print(F("   "));
      oled.update();
    }
    yield();
  }
}

// ===== ЭКРАН 5: Просмотр логов =====
void enterViewLogs() {
  if (loggingActive) return;

  File dir = SD.open("/logs");
  if (!dir || !dir.isDirectory()) {
    oled.clear(); oled.home();
    oled.println(F("Нет папки /logs")); oled.update();
    delay(1500); if (dir) dir.close(); return;
  }

  const uint8_t MAX_FILES = 50;
  String fileNames[MAX_FILES];
  uint8_t fileCount = 0;
  File f;
  while (f = dir.openNextFile()) {
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
    oled.println(F("Нет лог-файлов")); oled.update();
    delay(1500); return;
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
    oled.print(cursor == 0 ? F("> ..назад")      : F("  ..назад"));
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
          // Выбор режима просмотра
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
                  case 0: viewFileHex(lf);   done = true; break;
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
          oled.println(F("Ошибка чтения!")); oled.update(); delay(1000);
        }
      }
      break;
    }
  }
}

// ===== Простой просмотр файла (hex) =====
void viewFileHex(File f) {
  int32_t fileSize = f.size();
  int32_t viewPos = 0;

  flushEncoder();
  int32_t lastCnt = enc.counter;

  while (1) {
    oled.clear(); oled.home();
    f.seek(viewPos);
    for (uint8_t row = 0; row < 7 && f.available(); row++) {
      oled.setCursor(0, row);
      char lineBuf[22]; uint8_t li = 0;
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
        viewPos -= 512; if (viewPos < 0) viewPos = 0;
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

// Извлечь один байт из hex-строки (2 символа)
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

// ===== Разбор Modbus-пакетов =====
void viewFileParsed(File f) {
  const uint16_t MAXPKT = 60;
  uint32_t pktPos[MAXPKT];   // позиции начала пакетов в файле
  uint16_t pktCount = 0;
  int32_t fileSize = f.size();

  // Сканируем файл, ищем строки с '['
  f.seek(0);
  while (f.available() && pktCount < MAXPKT) {
    uint32_t pos = f.position();
    if (f.peek() == '\r' || f.peek() == '\n') {
      f.read();
      continue;
    }
    bool hasBracket = false;
    while (f.available()) {
      char c = f.read();
      if (c == '[') hasBracket = true;
      if (c == '\n' || c == '\r') break;
    }
    if (hasBracket) {
      pktPos[pktCount++] = pos;
    }
  }
  f.seek(0);

  if (pktCount == 0) {
    oled.clear(); oled.home();
    oled.println(F("Нет Modbus-пакетов"));
    oled.update(); delay(1500); return;
  }

  int16_t pktIdx = 0;
  flushEncoder();
  int32_t lastCnt = enc.counter;

  while (1) {
    // Позиционируемся на текущий пакет
    f.seek(pktPos[pktIdx]);

    // Читаем hex-байты между [ ]
    uint8_t data[128]; uint8_t dlen = 0;
    bool inBracket = false;
    char hexPair[3]; uint8_t hi = 0;

    while (f.available() && dlen < 128) {
      char c = f.read();
      if (c == '[')  { inBracket = true; continue; }
      if (c == ']')  break;
      if (c == '\n' || c == '\r') break;
      if (!inBracket) continue;

      if (c == ' ') {
        if (hi == 2) {
          if (hexByte(hexPair, data[dlen])) dlen++;
          hi = 0;
        }
      } else {
        if (hi < 2) hexPair[hi++] = c;
      }
    }
    if (hi == 2 && hexByte(hexPair, data[dlen])) dlen++;

    // Вывод на экран
    oled.clear(); oled.home();

    if (dlen < 4) {
      oled.println(F("Короткий пакет"));
    } else {
      uint8_t fc = data[1];

      // Эвристика Request vs Response:
      // 0x01-0x04: запрос всегда 8 байт, ответ = 5 + byteCount
      // 0x05-0x06: запрос и ответ идентичны (8 байт)
      // 0x0F-0x10: ответ всегда 8 байт, запрос длиннее
      bool isReq;
      if (fc == 0x01 || fc == 0x02 || fc == 0x03 || fc == 0x04) {
        isReq = (dlen == 8);
      } else if (fc == 0x0F || fc == 0x10) {
        isReq = (dlen > 8);
      } else {
        isReq = true;  // 0x05/0x06/прочее — показываем как запрос
      }  // Write Mult — запрос длиннее

      oled.setCursor(0, 0);
      oled.print(F("Slave:0x"));
      if (data[0] < 0x10) oled.print("0");
      oled.print(data[0], HEX);
      oled.print(isReq ? F(" REQ") : F(" RESP"));

      oled.setCursor(0, 1);
      oled.print(F("Func: 0x"));
      if (fc < 0x10) oled.print("0");
      oled.print(fc, HEX);
      oled.print(" ");
      oled.print(getFuncName(fc));

      uint8_t row = 2;

      if (isReq) {
        // === ЗАПРОС ===
        if (fc == 0x01 || fc == 0x02 || fc == 0x03 || fc == 0x04) {
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
        } else if (fc == 0x05 || fc == 0x06) {
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
        } else if (fc == 0x0F || fc == 0x10) {
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
        // === ОТВЕТ ===
        if (fc == 0x01 || fc == 0x02 || fc == 0x03 || fc == 0x04) {
          uint8_t bc = data[2];
          oled.setCursor(0, row++);
          oled.print(F("Bytes: ")); oled.print(bc);

          if (bc > 0 && dlen >= 5) {
            oled.setCursor(0, row++);
            oled.print(F("Data: "));
            uint8_t show = bc < 8 ? bc : 6;  // показать до 6 первых байт
            for (uint8_t i = 0; i < show && (3 + i) < dlen - 2; i++) {
              if (data[3 + i] < 0x10) oled.print("0");
              oled.print(data[3 + i], HEX);
              oled.print(" ");
            }
            if (bc > 6) oled.print("...");
          }
        } else if (fc == 0x05 || fc == 0x06) {
          // Эхо-ответ: повторяет запрос
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
        } else if (fc == 0x0F || fc == 0x10) {
          // Ответ на запись нескольких: 8 байт
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

      if (dlen >= 2 && row < 7) {
        uint16_t crc = (data[dlen-1] << 8) | data[dlen-2];
        oled.setCursor(0, row++);
        oled.print(F("CRC: 0x"));
        oled.print(crc, HEX);
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
  if (initRTC()) oled.println(F("OK"));
  else oled.println(F("ERR"));
  oled.update();

  oled.print(F("SD: "));
  if (initSD()) oled.println(F("OK"));
  else oled.println(F("ERR"));
  oled.update();

  initSerial1();
  enc.setEncType(EB_STEP2);
  enc.counter = 0;
  menuCursor = 0;
  loggingActive = false;
  delay(1000);
}

void loop() {
  enterMainMenu();
}
