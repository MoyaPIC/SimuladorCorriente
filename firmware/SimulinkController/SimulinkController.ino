/*
  Simulink Controller - Arduino UNO / ATmega328P
  ------------------------------------------------
  Hardware:
    MCP4725  -> I2C DAC, default 0x60
    ADS1115  -> I2C ADC, default 0x48, AIN0 single-ended
    24C512   -> I2C EEPROM, default 0x50
    HC-05    -> SoftwareSerial RX=D9, TX=D10, 9600 baud
    Buzzer   -> D6
    Loop error input -> D8

  Protocol compatible with Simulink PWA:
    GET:STATUS
    SET:CURRENT:<mA>
    SET:OUTPUT:ON
    SET:OUTPUT:OFF
    PROFILE:NEW:<slot>:<count>[:<repeats>]
    PROFILE:POINT:<slot>:<index>:<time_s>:<mA>
    PROFILE:SAVE:<slot>
    PROFILE:RUN:<slot>
    PROFILE:STOP
    PROFILE:DELETE:<slot>
    PROFILE:LIST
    CAL:OUT:POINTS:<measured_at_4mA>:<measured_at_20mA>
    CAL:OUT:CODES:<dac_code_4mA>:<dac_code_20mA>
    CAL:IN:POINTS:<displayed_at_real_4mA>:<displayed_at_real_20mA>
    CAL:IN:RAW:<raw_4mA>:<raw_20mA>
    CAL:IN:CAPTURE:4
    CAL:IN:CAPTURE:20
    CAL:READ
    PING

  Notes:
    - No external libraries: only Wire + SoftwareSerial from Arduino core.
    - Profiles execute non-blocking from the external 24C512.
    - Output commands outside 4.0..20.0 mA are rejected.
*/

#include <Wire.h>
#include <SoftwareSerial.h>

// --------------------------- Pinout ---------------------------
static const uint8_t PIN_BUZZER     = 6;
static const uint8_t PIN_LOOP_ERROR = 8;
static const uint8_t PIN_BT_RX      = 9;   // Arduino RX  <- HC-05 TX
static const uint8_t PIN_BT_TX      = 10;  // Arduino TX  -> HC-05 RX

// ----------------------- User adjustments --------------------
// Change to LOW if your hardware asserts D8 LOW when the 4-20 mA loop is open.
static const uint8_t LOOP_ERROR_ACTIVE_LEVEL = HIGH;

// true = pinMode(INPUT_PULLUP), false = pinMode(INPUT).
static const bool LOOP_ERROR_USE_PULLUP = false;

// true = passive buzzer driven with tone(); false = active buzzer driven HIGH/LOW.
static const bool BUZZER_PASSIVE = false;
static const uint16_t BUZZER_FREQ_HZ = 2500;

// --------------------------- I2C ------------------------------
static const uint8_t MCP4725_ADDR = 0x60;
static const uint8_t ADS1115_ADDR = 0x48;
static const uint8_t EEPROM_ADDR  = 0x50;

// ---------------------- Serial / timing -----------------------
static const char FIRMWARE_VERSION[] = "1.0.1";
static const uint32_t USB_BAUD = 115200UL;
static const uint32_t BT_BAUD  = 9600UL;

static const uint16_t STATUS_PERIOD_MS = 500;
static const uint16_t ADC_PERIOD_MS    = 25;
static const uint16_t LOOP_DEBOUNCE_MS = 30;
static const uint32_t LOOP_ARM_TIME_MS = 5000UL;
static const uint16_t PROFILE_DAC_UPDATE_MS = 20;

// ---------------------- 4-20 mA limits ------------------------
static const float CURRENT_MIN_MA = 4.0f;
static const float CURRENT_MAX_MA = 20.0f;

// -------------------- 24C512 memory map -----------------------
// 24C512 = 65536 bytes.
static const uint16_t EEPROM_CONFIG_BASE = 0;
static const uint16_t EEPROM_PROFILE_BASE = 1024;

static const uint8_t  PROFILE_SLOT_COUNT = 16;
static const uint16_t PROFILE_SLOT_SIZE  = 3968;  // 16 * 3968 + 1024 = 64512
static const uint16_t PROFILE_HEADER_SIZE = 16;
static const uint16_t PROFILE_POINT_SIZE  = 4;

// Each point: uint16_t time in deciseconds + uint16_t current in centi-mA.
static const uint16_t PROFILE_MAX_POINTS =
  (PROFILE_SLOT_SIZE - PROFILE_HEADER_SIZE) / PROFILE_POINT_SIZE; // 988

static const uint16_t CONFIG_MAGIC  = 0xC55A;
static const uint8_t  CONFIG_VER    = 1;
static const uint16_t PROFILE_MAGIC = 0xA55A;
static const uint8_t  PROFILE_VER   = 1;

// ------------------------- Instances --------------------------
SoftwareSerial btSerial(PIN_BT_RX, PIN_BT_TX);

// ------------------------- State ------------------------------
struct CalibrationConfig {
  uint16_t dacCode4;
  uint16_t dacCode20;
  int16_t adcRaw4;
  int16_t adcRaw20;
};

struct ProfileHeader {
  uint16_t magic;
  uint8_t version;
  uint8_t flags;
  uint16_t pointCount;
  uint16_t repeats;
  uint16_t crc;
  uint16_t reserved1;
  uint16_t reserved2;
  uint16_t reserved3;
};

struct ProfilePoint {
  uint16_t timeDs;
  uint16_t currentCentiMa;
};

CalibrationConfig cal = {
  0,
  4095,
  5312,
  26560
};

bool hasMcp4725 = false;
bool hasAds1115 = false;
bool has24c512 = false;

bool outputEnabled = true;
float outputSetpointMa = 12.0f;
uint16_t currentDacCode = 0;

int16_t lastAdcRaw = 0;
float inputMeasuredMa = 0.0f;
bool inputValid = false;

bool loopOpen = false;
bool loopRawLast = false;
bool loopAlarmArmed = false;
uint32_t loopRawChangedMs = 0;
uint32_t loopClosedSinceMs = 0;

uint32_t lastStatusMs = 0;
uint32_t lastAdcMs = 0;
uint32_t lastProfileDacMs = 0;

bool uploadActive = false;
uint8_t uploadSlot = 0;
uint16_t uploadExpected = 0;
uint16_t uploadReceived = 0;
uint16_t uploadRepeats = 1;

bool profileRunning = false;
uint8_t runningSlot = 0;
ProfileHeader runningHeader;
uint16_t runningRepeat = 0;
uint16_t runningIndex = 0;
ProfilePoint runningPointA;
ProfilePoint runningPointB;
bool runningHasB = false;
uint32_t runningStartMs = 0;

uint32_t beepUntilMs = 0;

static const uint8_t LINE_BUF_SIZE = 112;
char btLine[LINE_BUF_SIZE];
uint8_t btLineLen = 0;
char usbLine[LINE_BUF_SIZE];
uint8_t usbLineLen = 0;

// ---------------------- Utility functions ---------------------
static float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static uint16_t clampU16(long v, uint16_t lo, uint16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return (uint16_t)v;
}

static uint16_t readU16LE(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t readI16LE(const uint8_t *p) {
  return (int16_t)readU16LE(p);
}

static void writeU16LE(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(v >> 8);
}

static void writeI16LE(uint8_t *p, int16_t v) {
  writeU16LE(p, (uint16_t)v);
}

static uint16_t crc16Update(uint16_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 1) crc = (crc >> 1) ^ 0xA001;
    else crc >>= 1;
  }
  return crc;
}

static uint16_t crc16Buffer(const uint8_t *data, uint16_t len, uint16_t crc = 0xFFFF) {
  while (len--) crc = crc16Update(crc, *data++);
  return crc;
}

static bool i2cProbe(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

// ------------------------- Buzzer -----------------------------
static void buzzerSet(bool on) {
  if (BUZZER_PASSIVE) {
    if (on) tone(PIN_BUZZER, BUZZER_FREQ_HZ);
    else noTone(PIN_BUZZER);
  } else {
    digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
  }
}

static void buzzerBeep(uint16_t durationMs) {
  beepUntilMs = millis() + durationMs;
}

static void buzzerTask() {
  uint32_t now = millis();

  // El buzzer de error de lazo solo actúa si el sistema ya fue armado
  // por haber detectado carga conectada durante más de 5 s.
  if (loopAlarmArmed && loopOpen) {
    buzzerSet(true);
    return;
  }

  if ((int32_t)(beepUntilMs - now) > 0) buzzerSet(true);
  else buzzerSet(false);
}

// -------------------------- MCP4725 ---------------------------
static bool mcp4725WriteCode(uint16_t code) {
  if (!hasMcp4725) return false;
  if (code > 4095) code = 4095;

  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write((uint8_t)0x40);
  Wire.write((uint8_t)(code >> 4));
  Wire.write((uint8_t)((code & 0x0F) << 4));
  bool ok = (Wire.endTransmission() == 0);
  if (ok) currentDacCode = code;
  return ok;
}

static uint16_t currentToDacCode(float currentMa) {
  currentMa = clampFloat(currentMa, CURRENT_MIN_MA, CURRENT_MAX_MA);

  float spanCode = (float)((int32_t)cal.dacCode20 - (int32_t)cal.dacCode4);
  float ratio = (currentMa - CURRENT_MIN_MA) /
                (CURRENT_MAX_MA - CURRENT_MIN_MA);
  long code = lroundf((float)cal.dacCode4 + ratio * spanCode);
  return clampU16(code, 0, 4095);
}

static bool applyCurrentSetpoint(float currentMa) {
  if (currentMa < CURRENT_MIN_MA || currentMa > CURRENT_MAX_MA) return false;

  outputSetpointMa = currentMa;
  float appliedMa = outputEnabled ? outputSetpointMa : CURRENT_MIN_MA;
  return mcp4725WriteCode(currentToDacCode(appliedMa));
}

// --------------------------- ADS1115 --------------------------
static bool ads1115WriteRegister(uint8_t reg, uint16_t value) {
  if (!hasAds1115) return false;
  Wire.beginTransmission(ADS1115_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)(value & 0xFF));
  return Wire.endTransmission() == 0;
}

static bool ads1115Configure() {
  return ads1115WriteRegister(0x01, 0xC083);
}

static bool ads1115ReadRaw(int16_t &raw) {
  if (!hasAds1115) return false;

  Wire.beginTransmission(ADS1115_ADDR);
  Wire.write((uint8_t)0x00);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((uint8_t)ADS1115_ADDR, (uint8_t)2) != 2) return false;
  uint16_t v = ((uint16_t)Wire.read() << 8);
  v |= Wire.read();
  raw = (int16_t)v;
  return true;
}

static float adcRawToCurrentMa(int16_t raw) {
  int32_t den = (int32_t)cal.adcRaw20 - (int32_t)cal.adcRaw4;
  if (den == 0) return 0.0f;

  float ratio = ((float)raw - (float)cal.adcRaw4) / (float)den;
  return CURRENT_MIN_MA +
         ratio * (CURRENT_MAX_MA - CURRENT_MIN_MA);
}

static void adcTask() {
  uint32_t now = millis();
  if ((uint32_t)(now - lastAdcMs) < ADC_PERIOD_MS) return;
  lastAdcMs = now;

  int16_t raw;
  if (ads1115ReadRaw(raw)) {
    lastAdcRaw = raw;
    inputMeasuredMa = adcRawToCurrentMa(raw);
    inputValid = true;
  } else {
    inputValid = false;
  }
}

// --------------------------- 24C512 ---------------------------
static bool eepromWaitReady(uint16_t timeoutMs = 15) {
  uint32_t start = millis();
  while ((uint32_t)(millis() - start) < timeoutMs) {
    Wire.beginTransmission(EEPROM_ADDR);
    if (Wire.endTransmission() == 0) return true;
    delay(1);
  }
  return false;
}

static bool eepromWriteBlock(uint16_t address, const uint8_t *data, uint16_t len) {
  if (!has24c512) return false;

  while (len) {
    uint8_t pageRemaining = (uint8_t)(128 - (address & 0x7F));
    uint8_t chunk = (len > 28) ? 28 : (uint8_t)len;
    if (chunk > pageRemaining) chunk = pageRemaining;

    Wire.beginTransmission(EEPROM_ADDR);
    Wire.write((uint8_t)(address >> 8));
    Wire.write((uint8_t)(address & 0xFF));
    for (uint8_t i = 0; i < chunk; i++) Wire.write(data[i]);

    if (Wire.endTransmission() != 0) return false;
    if (!eepromWaitReady()) return false;

    address += chunk;
    data += chunk;
    len -= chunk;
  }
  return true;
}

static bool eepromReadBlock(uint16_t address, uint8_t *data, uint16_t len) {
  if (!has24c512) return false;

  while (len) {
    uint8_t chunk = (len > 28) ? 28 : (uint8_t)len;

    Wire.beginTransmission(EEPROM_ADDR);
    Wire.write((uint8_t)(address >> 8));
    Wire.write((uint8_t)(address & 0xFF));
    if (Wire.endTransmission(false) != 0) return false;

    uint8_t got = Wire.requestFrom((uint8_t)EEPROM_ADDR, chunk);
    if (got != chunk) return false;

    for (uint8_t i = 0; i < chunk; i++) data[i] = Wire.read();

    address += chunk;
    data += chunk;
    len -= chunk;
  }
  return true;
}

// ---------------------- Calibration storage -------------------
static bool saveCalibration() {
  if (!has24c512) return false;

  uint8_t b[16];
  memset(b, 0, sizeof(b));
  writeU16LE(&b[0], CONFIG_MAGIC);
  b[2] = CONFIG_VER;
  b[3] = 0;
  writeU16LE(&b[4], cal.dacCode4);
  writeU16LE(&b[6], cal.dacCode20);
  writeI16LE(&b[8], cal.adcRaw4);
  writeI16LE(&b[10], cal.adcRaw20);

  uint16_t crc = crc16Buffer(b, 12);
  writeU16LE(&b[12], crc);

  return eepromWriteBlock(EEPROM_CONFIG_BASE, b, sizeof(b));
}

static bool loadCalibration() {
  if (!has24c512) return false;

  uint8_t b[16];
  if (!eepromReadBlock(EEPROM_CONFIG_BASE, b, sizeof(b))) return false;

  if (readU16LE(&b[0]) != CONFIG_MAGIC) return false;
  if (b[2] != CONFIG_VER) return false;

  uint16_t storedCrc = readU16LE(&b[12]);
  uint16_t calcCrc = crc16Buffer(b, 12);
  if (storedCrc != calcCrc) return false;

  CalibrationConfig tmp;
  tmp.dacCode4 = readU16LE(&b[4]);
  tmp.dacCode20 = readU16LE(&b[6]);
  tmp.adcRaw4 = readI16LE(&b[8]);
  tmp.adcRaw20 = readI16LE(&b[10]);

  if (tmp.dacCode4 > 4095 || tmp.dacCode20 > 4095) return false;
  if (tmp.dacCode4 == tmp.dacCode20) return false;
  if (tmp.adcRaw4 == tmp.adcRaw20) return false;

  cal = tmp;
  return true;
}

// ------------------------- Profiles ---------------------------
static uint16_t profileSlotBase(uint8_t slot) {
  return EEPROM_PROFILE_BASE +
         (uint16_t)(slot - 1) * PROFILE_SLOT_SIZE;
}

static bool profileSlotValid(uint8_t slot) {
  return slot >= 1 && slot <= PROFILE_SLOT_COUNT;
}

static bool profileWriteHeader(uint8_t slot, const ProfileHeader &h) {
  if (!profileSlotValid(slot)) return false;

  uint8_t b[PROFILE_HEADER_SIZE];
  memset(b, 0, sizeof(b));
  writeU16LE(&b[0], h.magic);
  b[2] = h.version;
  b[3] = h.flags;
  writeU16LE(&b[4], h.pointCount);
  writeU16LE(&b[6], h.repeats);
  writeU16LE(&b[8], h.crc);
  writeU16LE(&b[10], h.reserved1);
  writeU16LE(&b[12], h.reserved2);
  writeU16LE(&b[14], h.reserved3);

  return eepromWriteBlock(profileSlotBase(slot), b, sizeof(b));
}

static bool profileReadHeader(uint8_t slot, ProfileHeader &h) {
  if (!profileSlotValid(slot)) return false;

  uint8_t b[PROFILE_HEADER_SIZE];
  if (!eepromReadBlock(profileSlotBase(slot), b, sizeof(b))) return false;

  h.magic = readU16LE(&b[0]);
  h.version = b[2];
  h.flags = b[3];
  h.pointCount = readU16LE(&b[4]);
  h.repeats = readU16LE(&b[6]);
  h.crc = readU16LE(&b[8]);
  h.reserved1 = readU16LE(&b[10]);
  h.reserved2 = readU16LE(&b[12]);
  h.reserved3 = readU16LE(&b[14]);

  return true;
}

static uint16_t profilePointAddress(uint8_t slot, uint16_t zeroBasedIndex) {
  return profileSlotBase(slot) + PROFILE_HEADER_SIZE +
         zeroBasedIndex * PROFILE_POINT_SIZE;
}

static bool profileWritePoint(uint8_t slot, uint16_t zeroBasedIndex,
                              const ProfilePoint &p) {
  if (!profileSlotValid(slot) || zeroBasedIndex >= PROFILE_MAX_POINTS) return false;

  uint8_t b[PROFILE_POINT_SIZE];
  writeU16LE(&b[0], p.timeDs);
  writeU16LE(&b[2], p.currentCentiMa);
  return eepromWriteBlock(profilePointAddress(slot, zeroBasedIndex),
                          b, sizeof(b));
}

static bool profileReadPoint(uint8_t slot, uint16_t zeroBasedIndex,
                             ProfilePoint &p) {
  if (!profileSlotValid(slot) || zeroBasedIndex >= PROFILE_MAX_POINTS) return false;

  uint8_t b[PROFILE_POINT_SIZE];
  if (!eepromReadBlock(profilePointAddress(slot, zeroBasedIndex),
                       b, sizeof(b))) return false;

  p.timeDs = readU16LE(&b[0]);
  p.currentCentiMa = readU16LE(&b[2]);
  return true;
}

static uint16_t profileCalculateCrc(uint8_t slot, uint16_t pointCount) {
  uint16_t crc = 0xFFFF;
  uint8_t b[PROFILE_POINT_SIZE];

  for (uint16_t i = 0; i < pointCount; i++) {
    if (!eepromReadBlock(profilePointAddress(slot, i), b, sizeof(b))) return 0;
    crc = crc16Buffer(b, sizeof(b), crc);
  }
  return crc;
}

static bool profileValidate(uint8_t slot, ProfileHeader &h) {
  if (!profileReadHeader(slot, h)) return false;
  if (h.magic != PROFILE_MAGIC || h.version != PROFILE_VER) return false;
  if (h.pointCount < 1 || h.pointCount > PROFILE_MAX_POINTS) return false;
  if (h.repeats < 1) h.repeats = 1;

  uint16_t crc = profileCalculateCrc(slot, h.pointCount);
  return crc != 0 && crc == h.crc;
}

static void profileStop() {
  profileRunning = false;
  runningSlot = 0;
  runningIndex = 0;
  runningRepeat = 0;
  runningHasB = false;
}

static bool profileStart(uint8_t slot) {
  ProfileHeader h;
  if (!profileValidate(slot, h)) return false;

  ProfilePoint p0;
  if (!profileReadPoint(slot, 0, p0)) return false;

  runningHeader = h;
  runningSlot = slot;
  runningRepeat = 0;
  runningIndex = 0;
  runningPointA = p0;
  runningHasB = false;

  if (h.pointCount > 1) {
    if (!profileReadPoint(slot, 1, runningPointB)) return false;
    runningHasB = true;
  }

  outputEnabled = true;
  profileRunning = true;
  runningStartMs = millis();
  lastProfileDacMs = 0;

  applyCurrentSetpoint((float)p0.currentCentiMa / 100.0f);
  buzzerBeep(80);
  return true;
}

static void profileRestartRepeat() {
  runningIndex = 0;
  profileReadPoint(runningSlot, 0, runningPointA);

  runningHasB = false;
  if (runningHeader.pointCount > 1) {
    if (profileReadPoint(runningSlot, 1, runningPointB)) runningHasB = true;
  }

  runningStartMs = millis();
}

static void profileTask() {
  if (!profileRunning) return;

  uint32_t now = millis();
  uint32_t elapsedMs = now - runningStartMs;

  while (runningHasB &&
         elapsedMs >= (uint32_t)runningPointB.timeDs * 100UL) {
    runningPointA = runningPointB;
    runningIndex++;

    if ((uint16_t)(runningIndex + 1) < runningHeader.pointCount) {
      if (!profileReadPoint(runningSlot, runningIndex + 1, runningPointB)) {
        profileStop();
        return;
      }
      runningHasB = true;
    } else {
      runningHasB = false;
    }
  }

  if (!runningHasB) {
    uint32_t finalMs = (uint32_t)runningPointA.timeDs * 100UL;
    if (elapsedMs >= finalMs) {
      applyCurrentSetpoint((float)runningPointA.currentCentiMa / 100.0f);

      if ((uint16_t)(runningRepeat + 1) < runningHeader.repeats) {
        runningRepeat++;
        profileRestartRepeat();
      } else {
        profileStop();
        buzzerBeep(150);
      }
      return;
    }
  }

  if ((uint32_t)(now - lastProfileDacMs) < PROFILE_DAC_UPDATE_MS) return;
  lastProfileDacMs = now;

  float outMa = (float)runningPointA.currentCentiMa / 100.0f;

  if (runningHasB) {
    uint32_t ta = (uint32_t)runningPointA.timeDs * 100UL;
    uint32_t tb = (uint32_t)runningPointB.timeDs * 100UL;

    if (tb > ta && elapsedMs > ta) {
      float frac = (float)(elapsedMs - ta) / (float)(tb - ta);
      frac = clampFloat(frac, 0.0f, 1.0f);

      float a = (float)runningPointA.currentCentiMa / 100.0f;
      float b = (float)runningPointB.currentCentiMa / 100.0f;
      outMa = a + (b - a) * frac;
    }
  }

  applyCurrentSetpoint(clampFloat(outMa, CURRENT_MIN_MA, CURRENT_MAX_MA));
}

// ------------------------ Loop error --------------------------
static bool readLoopErrorRaw() {
  return digitalRead(PIN_LOOP_ERROR) == LOOP_ERROR_ACTIVE_LEVEL;
}

static void loopErrorTask() {
  bool raw = readLoopErrorRaw();
  uint32_t now = millis();

  // Debounce de la entrada D8.
  if (raw != loopRawLast) {
    loopRawLast = raw;
    loopRawChangedMs = now;
  }

  if (raw != loopOpen &&
      (uint32_t)(now - loopRawChangedMs) >= LOOP_DEBOUNCE_MS) {
    loopOpen = raw;

    if (!loopOpen) {
      // Carga conectada: comienza el tiempo de validación.
      loopClosedSinceMs = now;
    } else {
      // Carga desconectada: no se arma aquí. Si nunca hubo una
      // conexión válida de >5 s, el buzzer debe permanecer apagado.
      loopClosedSinceMs = 0;
    }
  }

  // Armado permanente después de 5 s continuos con carga conectada.
  // Si la placa arrancó desconectada, queda desarmada hasta que
  // posteriormente detecte una conexión estable durante >5 s.
  if (!loopAlarmArmed && !loopOpen) {
    if (loopClosedSinceMs == 0) loopClosedSinceMs = now;

    if ((uint32_t)(now - loopClosedSinceMs) >= LOOP_ARM_TIME_MS) {
      loopAlarmArmed = true;
    }
  }
}

// ----------------------- Status output ------------------------
static void printFloat3(Stream &s, float v) {
  char tmp[18];
  dtostrf(v, 0, 3, tmp);
  s.print(tmp);
}

static void sendStatus(Stream &s) {
  s.print(F("DATA:OUT="));
  printFloat3(s, outputEnabled ? outputSetpointMa : CURRENT_MIN_MA);

  s.print(F(":IN="));
  if (inputValid) printFloat3(s, inputMeasuredMa);
  else s.print(F("nan"));

  s.print(F(":LOOP="));
  s.print(loopOpen ? F("OPEN") : F("CLOSED"));

  s.print(F(":ARM="));
  s.print(loopAlarmArmed ? 1 : 0);

  s.print(F(":RUN="));
  s.print(profileRunning ? 1 : 0);

  s.print(F(":SLOT="));
  s.print(profileRunning ? runningSlot : 0);

  s.print(F(":DAC="));
  s.print(currentDacCode);

  s.println();
}

static void periodicStatusTask() {
  uint32_t now = millis();
  if ((uint32_t)(now - lastStatusMs) < STATUS_PERIOD_MS) return;
  lastStatusMs = now;

  if (!uploadActive) sendStatus(btSerial);
  sendStatus(Serial);
}

// ---------------------- Protocol helpers ----------------------
static void replyOK(Stream &s) {
  s.println(F("OK"));
}

static void replyError(Stream &s, const __FlashStringHelper *msg) {
  s.print(F("ERR:"));
  s.println(msg);
}

static void printCalibration(Stream &s) {
  s.print(F("CAL:OUT:CODES:"));
  s.print(cal.dacCode4);
  s.print(':');
  s.println(cal.dacCode20);

  s.print(F("CAL:IN:RAW:"));
  s.print(cal.adcRaw4);
  s.print(':');
  s.println(cal.adcRaw20);

  s.print(F("LIMITS:CURRENT:"));
  printFloat3(s, CURRENT_MIN_MA);
  s.print(':');
  printFloat3(s, CURRENT_MAX_MA);
  s.println();

  s.print(F("PROFILE:CAPACITY:SLOTS="));
  s.print(PROFILE_SLOT_COUNT);
  s.print(F(":POINTS="));
  s.println(PROFILE_MAX_POINTS);
}

// -------------------- Calibration commands -------------------
static bool calibrateOutputFromMeasured(float measuredAt4,
                                        float measuredAt20) {
  if (measuredAt20 <= measuredAt4 + 0.01f) return false;

  float c4 = (float)cal.dacCode4;
  float c20 = (float)cal.dacCode20;

  float codePerMa = (c20 - c4) / (measuredAt20 - measuredAt4);

  long newCode4 = lroundf(c4 + (CURRENT_MIN_MA - measuredAt4) * codePerMa);
  long newCode20 = lroundf(c4 + (CURRENT_MAX_MA - measuredAt4) * codePerMa);

  if (newCode4 < 0 || newCode4 > 4095 ||
      newCode20 < 0 || newCode20 > 4095 ||
      newCode4 == newCode20) return false;

  cal.dacCode4 = (uint16_t)newCode4;
  cal.dacCode20 = (uint16_t)newCode20;
  saveCalibration();
  applyCurrentSetpoint(outputSetpointMa);
  return true;
}

static bool calibrateInputFromDisplayed(float displayedAtReal4,
                                       float displayedAtReal20) {
  if (displayedAtReal20 <= displayedAtReal4 + 0.01f) return false;

  float rawSpan = (float)((int32_t)cal.adcRaw20 - (int32_t)cal.adcRaw4);

  float rawAtReal4 =
    (float)cal.adcRaw4 +
    ((displayedAtReal4 - CURRENT_MIN_MA) /
     (CURRENT_MAX_MA - CURRENT_MIN_MA)) * rawSpan;

  float rawAtReal20 =
    (float)cal.adcRaw4 +
    ((displayedAtReal20 - CURRENT_MIN_MA) /
     (CURRENT_MAX_MA - CURRENT_MIN_MA)) * rawSpan;

  long r4 = lroundf(rawAtReal4);
  long r20 = lroundf(rawAtReal20);

  if (r4 < -32768L || r4 > 32767L ||
      r20 < -32768L || r20 > 32767L ||
      r4 == r20) return false;

  cal.adcRaw4 = (int16_t)r4;
  cal.adcRaw20 = (int16_t)r20;
  saveCalibration();
  return true;
}

// ---------------------- Command parser ------------------------
static void handleProfileCommand(Stream &src, char *savePtr) {
  char *op = strtok_r(NULL, ":", &savePtr);
  if (!op) {
    replyError(src, F("PROFILE_CMD"));
    return;
  }

  if (!strcmp(op, "NEW")) {
    char *slotS = strtok_r(NULL, ":", &savePtr);
    char *countS = strtok_r(NULL, ":", &savePtr);
    char *repeatS = strtok_r(NULL, ":", &savePtr);

    if (!slotS || !countS) {
      replyError(src, F("PROFILE_NEW_ARGS"));
      return;
    }

    uint8_t slot = (uint8_t)atoi(slotS);
    uint16_t count = (uint16_t)atoi(countS);
    uint16_t repeats = repeatS ? (uint16_t)atoi(repeatS) : 1;

    if (!has24c512) {
      replyError(src, F("EEPROM_OFFLINE"));
      return;
    }
    if (!profileSlotValid(slot)) {
      replyError(src, F("SLOT_1_16"));
      return;
    }
    if (count < 1 || count > PROFILE_MAX_POINTS) {
      replyError(src, F("POINT_COUNT"));
      return;
    }
    if (repeats < 1) repeats = 1;

    ProfileHeader invalid;
    memset(&invalid, 0, sizeof(invalid));
    profileWriteHeader(slot, invalid);

    uploadActive = true;
    uploadSlot = slot;
    uploadExpected = count;
    uploadReceived = 0;
    uploadRepeats = repeats;

    if (&src != &btSerial) replyOK(src);
    return;
  }

  if (!strcmp(op, "POINT")) {
    char *slotS = strtok_r(NULL, ":", &savePtr);
    char *indexS = strtok_r(NULL, ":", &savePtr);
    char *timeS = strtok_r(NULL, ":", &savePtr);
    char *maS = strtok_r(NULL, ":", &savePtr);

    if (!slotS || !indexS || !timeS || !maS) {
      replyError(src, F("PROFILE_POINT_ARGS"));
      return;
    }

    uint8_t slot = (uint8_t)atoi(slotS);
    uint16_t oneBasedIndex = (uint16_t)atoi(indexS);
    float timeSec = atof(timeS);
    float ma = atof(maS);

    if (!uploadActive || slot != uploadSlot) {
      replyError(src, F("NO_UPLOAD"));
      return;
    }
    if (oneBasedIndex != (uint16_t)(uploadReceived + 1)) {
      replyError(src, F("SEQUENCE"));
      return;
    }
    if (oneBasedIndex < 1 || oneBasedIndex > uploadExpected) {
      replyError(src, F("INDEX"));
      return;
    }
    if (timeSec < 0.0f || timeSec > 6553.5f) {
      replyError(src, F("TIME_RANGE"));
      return;
    }
    if (ma < CURRENT_MIN_MA || ma > CURRENT_MAX_MA) {
      replyError(src, F("CURRENT_RANGE"));
      return;
    }

    ProfilePoint p;
    p.timeDs = (uint16_t)lroundf(timeSec * 10.0f);
    p.currentCentiMa = (uint16_t)lroundf(ma * 100.0f);

    if (!profileWritePoint(slot, oneBasedIndex - 1, p)) {
      replyError(src, F("EEPROM_WRITE"));
      return;
    }

    uploadReceived++;

    if (&src != &btSerial) replyOK(src);
    return;
  }

  if (!strcmp(op, "SAVE")) {
    char *slotS = strtok_r(NULL, ":", &savePtr);
    if (!slotS) {
      replyError(src, F("PROFILE_SAVE_ARGS"));
      return;
    }

    uint8_t slot = (uint8_t)atoi(slotS);

    if (!uploadActive || slot != uploadSlot) {
      replyError(src, F("NO_UPLOAD"));
      return;
    }
    if (uploadReceived != uploadExpected) {
      replyError(src, F("MISSING_POINTS"));
      return;
    }

    uint16_t crc = profileCalculateCrc(slot, uploadExpected);
    if (crc == 0) {
      replyError(src, F("EEPROM_READ"));
      return;
    }

    ProfileHeader h;
    memset(&h, 0, sizeof(h));
    h.magic = PROFILE_MAGIC;
    h.version = PROFILE_VER;
    h.pointCount = uploadExpected;
    h.repeats = uploadRepeats;
    h.crc = crc;

    if (!profileWriteHeader(slot, h)) {
      replyError(src, F("EEPROM_WRITE"));
      return;
    }

    uploadActive = false;
    buzzerBeep(80);
    replyOK(src);
    return;
  }

  if (!strcmp(op, "RUN")) {
    char *slotS = strtok_r(NULL, ":", &savePtr);
    if (!slotS) {
      replyError(src, F("PROFILE_RUN_ARGS"));
      return;
    }

    uint8_t slot = (uint8_t)atoi(slotS);
    if (!profileSlotValid(slot)) {
      replyError(src, F("SLOT_1_16"));
      return;
    }

    if (!profileStart(slot)) {
      replyError(src, F("PROFILE_INVALID"));
      return;
    }

    replyOK(src);
    return;
  }

  if (!strcmp(op, "STOP")) {
    profileStop();
    replyOK(src);
    return;
  }

  if (!strcmp(op, "DELETE")) {
    char *slotS = strtok_r(NULL, ":", &savePtr);
    if (!slotS) {
      replyError(src, F("PROFILE_DELETE_ARGS"));
      return;
    }

    uint8_t slot = (uint8_t)atoi(slotS);
    if (!profileSlotValid(slot)) {
      replyError(src, F("SLOT_1_16"));
      return;
    }

    ProfileHeader h;
    memset(&h, 0, sizeof(h));
    if (!profileWriteHeader(slot, h)) {
      replyError(src, F("EEPROM_WRITE"));
      return;
    }

    if (profileRunning && runningSlot == slot) profileStop();
    replyOK(src);
    return;
  }

  if (!strcmp(op, "LIST")) {
    for (uint8_t slot = 1; slot <= PROFILE_SLOT_COUNT; slot++) {
      ProfileHeader h;
      if (profileReadHeader(slot, h) &&
          h.magic == PROFILE_MAGIC &&
          h.version == PROFILE_VER &&
          h.pointCount >= 1 &&
          h.pointCount <= PROFILE_MAX_POINTS) {
        src.print(F("PROFILE:"));
        src.print(slot);
        src.print(F(":COUNT="));
        src.print(h.pointCount);
        src.print(F(":REP="));
        src.println(h.repeats);
      }
    }
    replyOK(src);
    return;
  }

  replyError(src, F("PROFILE_UNKNOWN"));
}

static void handleCalibrationCommand(Stream &src, char *savePtr) {
  char *channel = strtok_r(NULL, ":", &savePtr);
  if (!channel) {
    replyError(src, F("CAL_CHANNEL"));
    return;
  }

  if (!strcmp(channel, "READ")) {
    printCalibration(src);
    replyOK(src);
    return;
  }

  char *op = strtok_r(NULL, ":", &savePtr);
  if (!op) {
    replyError(src, F("CAL_CMD"));
    return;
  }

  if (!strcmp(op, "TYPE") || !strcmp(op, "RANGE") || !strcmp(op, "CURRENT")) {
    replyOK(src);
    return;
  }

  if (!strcmp(channel, "OUT")) {
    if (!strcmp(op, "POINTS")) {
      char *m4S = strtok_r(NULL, ":", &savePtr);
      char *m20S = strtok_r(NULL, ":", &savePtr);
      if (!m4S || !m20S) {
        replyError(src, F("CAL_OUT_ARGS"));
        return;
      }

      if (!calibrateOutputFromMeasured(atof(m4S), atof(m20S))) {
        replyError(src, F("CAL_OUT_INVALID"));
        return;
      }

      replyOK(src);
      return;
    }

    if (!strcmp(op, "CODES")) {
      char *c4S = strtok_r(NULL, ":", &savePtr);
      char *c20S = strtok_r(NULL, ":", &savePtr);
      if (!c4S || !c20S) {
        replyError(src, F("CAL_OUT_CODES_ARGS"));
        return;
      }

      long c4 = atol(c4S);
      long c20 = atol(c20S);
      if (c4 < 0 || c4 > 4095 || c20 < 0 || c20 > 4095 || c4 == c20) {
        replyError(src, F("DAC_CODE_RANGE"));
        return;
      }

      cal.dacCode4 = (uint16_t)c4;
      cal.dacCode20 = (uint16_t)c20;
      saveCalibration();
      applyCurrentSetpoint(outputSetpointMa);
      replyOK(src);
      return;
    }
  }

  if (!strcmp(channel, "IN")) {
    if (!strcmp(op, "POINTS")) {
      char *m4S = strtok_r(NULL, ":", &savePtr);
      char *m20S = strtok_r(NULL, ":", &savePtr);
      if (!m4S || !m20S) {
        replyError(src, F("CAL_IN_ARGS"));
        return;
      }

      if (!calibrateInputFromDisplayed(atof(m4S), atof(m20S))) {
        replyError(src, F("CAL_IN_INVALID"));
        return;
      }

      replyOK(src);
      return;
    }

    if (!strcmp(op, "RAW")) {
      char *r4S = strtok_r(NULL, ":", &savePtr);
      char *r20S = strtok_r(NULL, ":", &savePtr);
      if (!r4S || !r20S) {
        replyError(src, F("CAL_IN_RAW_ARGS"));
        return;
      }

      long r4 = atol(r4S);
      long r20 = atol(r20S);
      if (r4 < -32768L || r4 > 32767L ||
          r20 < -32768L || r20 > 32767L ||
          r4 == r20) {
        replyError(src, F("ADC_RAW_RANGE"));
        return;
      }

      cal.adcRaw4 = (int16_t)r4;
      cal.adcRaw20 = (int16_t)r20;
      saveCalibration();
      replyOK(src);
      return;
    }

    if (!strcmp(op, "CAPTURE")) {
      char *pointS = strtok_r(NULL, ":", &savePtr);
      if (!pointS || !inputValid) {
        replyError(src, F("CAL_CAPTURE"));
        return;
      }

      int point = atoi(pointS);
      if (point == 4) cal.adcRaw4 = lastAdcRaw;
      else if (point == 20) cal.adcRaw20 = lastAdcRaw;
      else {
        replyError(src, F("CAL_POINT_4_20"));
        return;
      }

      if (cal.adcRaw4 == cal.adcRaw20) {
        replyError(src, F("CAL_RAW_EQUAL"));
        return;
      }

      saveCalibration();
      replyOK(src);
      return;
    }
  }

  replyError(src, F("CAL_UNKNOWN"));
}

static void processCommand(Stream &src, const char *line) {
  if (!line || !line[0]) return;

  char cmd[LINE_BUF_SIZE];
  strncpy(cmd, line, sizeof(cmd) - 1);
  cmd[sizeof(cmd) - 1] = 0;

  char *savePtr = NULL;
  char *root = strtok_r(cmd, ":", &savePtr);
  if (!root) return;

  if (!strcmp(root, "PING")) {
    src.println(F("PONG"));
    return;
  }

  if (!strcmp(root, "GET")) {
    char *what = strtok_r(NULL, ":", &savePtr);
    if (what && !strcmp(what, "STATUS")) {
      sendStatus(src);
      return;
    }
    replyError(src, F("GET_UNKNOWN"));
    return;
  }

  if (!strcmp(root, "SET")) {
    char *what = strtok_r(NULL, ":", &savePtr);
    if (!what) {
      replyError(src, F("SET_CMD"));
      return;
    }

    if (!strcmp(what, "CURRENT")) {
      char *maS = strtok_r(NULL, ":", &savePtr);
      if (!maS) {
        replyError(src, F("CURRENT_ARG"));
        return;
      }

      float ma = atof(maS);
      if (ma < CURRENT_MIN_MA || ma > CURRENT_MAX_MA) {
        replyError(src, F("CURRENT_RANGE_4_20"));
        return;
      }

      profileStop();
      outputEnabled = true;

      if (!applyCurrentSetpoint(ma)) {
        replyError(src, F("DAC_WRITE"));
        return;
      }

      replyOK(src);
      return;
    }

    if (!strcmp(what, "OUTPUT")) {
      char *action = strtok_r(NULL, ":", &savePtr);
      if (!action) {
        replyError(src, F("OUTPUT_ARG"));
        return;
      }

      if (!strcmp(action, "ON")) {
        outputEnabled = true;
        applyCurrentSetpoint(outputSetpointMa);
        replyOK(src);
        return;
      }

      if (!strcmp(action, "OFF")) {
        profileStop();
        outputEnabled = false;
        applyCurrentSetpoint(CURRENT_MIN_MA);
        replyOK(src);
        return;
      }

      if (!strcmp(action, "OPEN")) {
        replyError(src, F("OPEN_UNSUPPORTED"));
        return;
      }

      replyError(src, F("OUTPUT_UNKNOWN"));
      return;
    }

    replyError(src, F("SET_UNKNOWN"));
    return;
  }

  if (!strcmp(root, "PROFILE")) {
    handleProfileCommand(src, savePtr);
    return;
  }

  if (!strcmp(root, "CAL")) {
    handleCalibrationCommand(src, savePtr);
    return;
  }

  if (!strcmp(root, "INFO")) {
    src.print(F("INFO:FW="));
    src.print(FIRMWARE_VERSION);
    src.print(F(":MCP4725="));
    src.print(hasMcp4725 ? 1 : 0);
    src.print(F(":ADS1115="));
    src.print(hasAds1115 ? 1 : 0);
    src.print(F(":EEPROM="));
    src.print(has24c512 ? 1 : 0);
    src.print(F(":BTBAUD="));
    src.println(BT_BAUD);
    printCalibration(src);
    return;
  }

  replyError(src, F("UNKNOWN_CMD"));
}

// ------------------------- RX lines ---------------------------
static void serviceInput(Stream &s, char *buf, uint8_t &len) {
  while (s.available()) {
    char c = (char)s.read();

    if (c == '' || c == '
') {
      if (len > 0) {
        buf[len] = 0;
        processCommand(s, buf);
        len = 0;
      }
      continue;
    }

    if (len < LINE_BUF_SIZE - 1) {
      buf[len++] = c;
    } else {
      len = 0;
      replyError(s, F("LINE_TOO_LONG"));
    }
  }
}

// --------------------------- Setup ----------------------------
void setup() {
  pinMode(PIN_BUZZER, OUTPUT);
  buzzerSet(false);

  pinMode(PIN_LOOP_ERROR,
          LOOP_ERROR_USE_PULLUP ? INPUT_PULLUP : INPUT);

  Serial.begin(USB_BAUD);
  btSerial.begin(BT_BAUD);

  Wire.begin();
  Wire.setClock(100000UL);

  delay(25);

  hasMcp4725 = i2cProbe(MCP4725_ADDR);
  hasAds1115 = i2cProbe(ADS1115_ADDR);
  has24c512 = i2cProbe(EEPROM_ADDR);

  if (hasAds1115) ads1115Configure();

  if (has24c512) {
    if (!loadCalibration()) saveCalibration();
  }

  loopRawLast = readLoopErrorRaw();
  loopOpen = loopRawLast;
  loopRawChangedMs = millis();
  loopAlarmArmed = false;
  loopClosedSinceMs = loopOpen ? 0 : millis();

  outputEnabled = true;
  outputSetpointMa = 12.0f;
  applyCurrentSetpoint(outputSetpointMa);

  Serial.print(F("Simulink Controller v"));
  Serial.print(FIRMWARE_VERSION);
  Serial.println(F(" READY"));
  btSerial.println(F("READY:SIMULINK"));

  printCalibration(Serial);

  if (!hasMcp4725) Serial.println(F("WARN:MCP4725_OFFLINE"));
  if (!hasAds1115) Serial.println(F("WARN:ADS1115_OFFLINE"));
  if (!has24c512) Serial.println(F("WARN:24C512_OFFLINE"));

  buzzerBeep(120);
}

// ---------------------------- Loop ----------------------------
void loop() {
  serviceInput(btSerial, btLine, btLineLen);
  serviceInput(Serial, usbLine, usbLineLen);

  adcTask();
  loopErrorTask();
  profileTask();
  buzzerTask();
  periodicStatusTask();
}
