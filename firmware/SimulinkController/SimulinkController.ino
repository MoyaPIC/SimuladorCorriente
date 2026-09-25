/*
  ============================================================================
  SIMULINK CONTROLLER
  Firmware para Arduino UNO / ATmega328P
  ============================================================================
  Objetivo:
    Controlar una placa capaz de generar y medir señales industriales 4-20 mA,
    almacenar perfiles automáticos en EEPROM externa y comunicarse con la PWA
    Simulink mediante un módulo Bluetooth HC-05.

  HARDWARE UTILIZADO
  ------------------
  1) MCP4725
     - DAC I2C de 12 bits.
     - Genera la tensión de referencia utilizada por la etapa 4-20 mA.
     - Dirección I2C por defecto: 0x60.

  2) ADS1115
     - ADC I2C de 16 bits.
     - Mide la entrada de corriente mediante la etapa de acondicionamiento.
     - Se utiliza AIN0 en modo single-ended.
     - Dirección I2C por defecto: 0x48.

  3) 24C512
     - EEPROM I2C externa de 64 KBytes.
     - Guarda calibraciones y perfiles de corriente.
     - Dirección I2C por defecto: 0x50.

  4) HC-05
     - Comunicación Bluetooth clásica SPP.
     - Se maneja mediante SoftwareSerial.
     - D9  = RX del Arduino  <- TX del HC-05.
     - D10 = TX del Arduino  -> RX del HC-05.
     - Velocidad: 9600 baud.

  5) Buzzer activo
     - Conectado a D6.
     - Solo debe sonar cuando se detecta una desconexión de carga DESPUÉS
       de que el sistema haya confirmado una carga conectada durante 5 s.

  6) Entrada de error de lazo
     - Conectada a D8.
     - HIGH significa error / lazo abierto.
     - LOW significa carga conectada / lazo cerrado.

  COMPORTAMIENTO DE LA ALARMA DE LAZO
  ------------------------------------
  - Al arrancar, la alarma está DESARMADA.
  - Si la placa arranca con la carga desconectada, el buzzer NO suena.
  - Cuando D8 permanece indicando carga conectada durante 5 segundos
    continuos, la alarma queda ARMADA.
  - Una vez armada, si D8 pasa a HIGH, el buzzer se enciende continuamente.
  - Al reconectar la carga, el buzzer se apaga.
  - El buzzer no se usa para confirmaciones, perfiles ni arranque.

  PROTOCOLO PRINCIPAL CON SIMULINK
  --------------------------------
    GET:STATUS
    SET:CURRENT:<mA>
    SET:OUTPUT:ON
    SET:OUTPUT:OFF

    PROFILE:NEW:<slot>:<cantidad>[:<repeticiones>]
    PROFILE:POINT:<slot>:<indice>:<tiempo_s>:<mA>
    PROFILE:SAVE:<slot>
    PROFILE:RUN:<slot>
    PROFILE:STOP
    PROFILE:DELETE:<slot>
    PROFILE:LIST

    CAL:OUT:POINTS:<medido_4mA>:<medido_20mA>
    CAL:OUT:CODES:<codigo_4mA>:<codigo_20mA>
    CAL:IN:POINTS:<indicado_4mA>:<indicado_20mA>
    CAL:IN:RAW:<raw_4mA>:<raw_20mA>
    CAL:IN:CAPTURE:4
    CAL:IN:CAPTURE:20
    CAL:READ

    PING
    INFO

  CARACTERÍSTICAS IMPORTANTES
  ---------------------------
  - No utiliza librerías externas para MCP4725, ADS1115 ni 24C512.
  - Solo usa Wire y SoftwareSerial del core de Arduino.
  - Todo el funcionamiento es cooperativo/no bloqueante salvo los pocos
    milisegundos necesarios para completar una escritura física en EEPROM.
  - Los perfiles se leen directamente desde la 24C512 para no consumir los
    2 KB de SRAM del ATmega328P.
  - Toda orden de corriente fuera del rango 4.0-20.0 mA es rechazada.
  - La calibración se guarda de forma persistente en la 24C512.
  - Los perfiles almacenados se verifican mediante CRC16 antes de ejecutarse.
  ============================================================================
*/

#include <Wire.h>
#include <SoftwareSerial.h>

// ============================================================================
// 1. ASIGNACIÓN DE PINES
// ============================================================================
// Los pines I2C del Arduino UNO son A4=SDA y A5=SCL y por eso no se declaran
// aquí como GPIO normales. MCP4725, ADS1115 y 24C512 comparten ese mismo bus.
static const uint8_t PIN_BUZZER     = 6;
static const uint8_t PIN_LOOP_ERROR = 8;
static const uint8_t PIN_BT_RX      = 9;   // Arduino RX  <- HC-05 TX
static const uint8_t PIN_BT_TX      = 10;  // Arduino TX  -> HC-05 RX

// ============================================================================
// 2. AJUSTES DE HARDWARE
// ============================================================================
// Estas constantes permiten adaptar el mismo firmware a pequeñas variantes
// de la placa sin modificar la lógica principal.
// En esta placa D8 pasa a HIGH cuando la electrónica detecta lazo abierto.
static const uint8_t LOOP_ERROR_ACTIVE_LEVEL = HIGH;

// La entrada D8 ya dispone de una señal lógica definida por el hardware.
// Por eso se deja sin pull-up interno.
static const bool LOOP_ERROR_USE_PULLUP = false;

// El buzzer instalado es ACTIVO: basta aplicar nivel HIGH para que suene.
// Se conserva la constante de frecuencia únicamente por compatibilidad con
// la función genérica buzzerSet(), aunque en este hardware no se utiliza tone().
static const bool BUZZER_PASSIVE = false;
static const uint16_t BUZZER_FREQ_HZ = 2500;

// ============================================================================
// 3. DIRECCIONES DEL BUS I2C
// ============================================================================
// Si en una revisión futura de la placa cambian A0/A1/A2 de algún integrado,
// solo será necesario modificar estas direcciones.
static const uint8_t MCP4725_ADDR = 0x60;
static const uint8_t ADS1115_ADDR = 0x48;
static const uint8_t EEPROM_ADDR  = 0x50;

// ============================================================================
// 4. COMUNICACIONES Y TEMPORIZACIONES
// ============================================================================
// USB_BAUD se usa solo para monitor serie/diagnóstico.
// BT_BAUD debe coincidir con la velocidad configurada en el HC-05.
static const char FIRMWARE_VERSION[] = "1.0.2";
static const uint32_t USB_BAUD = 115200UL;
static const uint32_t BT_BAUD  = 9600UL;

// Cada cuánto se envía un paquete DATA:... de estado.
static const uint16_t STATUS_PERIOD_MS = 500;
// Período de lectura del ADS1115. 25 ms equivale a 40 lecturas/s.
static const uint16_t ADC_PERIOD_MS    = 25;
// Filtrado digital de D8 para evitar falsos cambios por ruido/rebote.
static const uint16_t LOOP_DEBOUNCE_MS = 30;
// Tiempo mínimo continuo de carga conectada necesario para armar la alarma.
static const uint32_t LOOP_ARM_TIME_MS = 5000UL;
// Frecuencia máxima de actualización del DAC durante un perfil automático.
static const uint16_t PROFILE_DAC_UPDATE_MS = 20;

// ============================================================================
// 5. LÍMITES ELÉCTRICOS DE OPERACIÓN
// ============================================================================
// El firmware nunca acepta una consigna normal fuera del rango 4-20 mA.
static const float CURRENT_MIN_MA = 4.0f;
static const float CURRENT_MAX_MA = 20.0f;

// ============================================================================
// 6. MAPA DE MEMORIA DE LA EEPROM 24C512
// ============================================================================
// La 24C512 dispone de 65536 bytes.
// 0...1023     : configuración/calibración.
// 1024...64511 : 16 slots independientes de perfiles.
// El espacio restante queda reservado para ampliaciones futuras.
// 24C512 = 65536 bytes.
static const uint16_t EEPROM_CONFIG_BASE = 0;
static const uint16_t EEPROM_PROFILE_BASE = 1024;

static const uint8_t  PROFILE_SLOT_COUNT = 16;
static const uint16_t PROFILE_SLOT_SIZE  = 3968;  // 16 * 3968 + 1024 = 64512
static const uint16_t PROFILE_HEADER_SIZE = 16;
static const uint16_t PROFILE_POINT_SIZE  = 4;

// Formato compacto de cada punto:
//   timeDs          = tiempo absoluto desde el inicio, en décimas de segundo.
//   currentCentiMa  = corriente en centésimas de mA.
// Ejemplo: 12.50 mA se almacena como 1250.
static const uint16_t PROFILE_MAX_POINTS =
  (PROFILE_SLOT_SIZE - PROFILE_HEADER_SIZE) / PROFILE_POINT_SIZE; // 988

// Valores "magic" permiten distinguir datos válidos de memoria vacía/corrupta.
// CONFIG_VER y PROFILE_VER permiten migrar formatos en versiones futuras.
static const uint16_t CONFIG_MAGIC  = 0xC55A;
static const uint8_t  CONFIG_VER    = 1;
static const uint16_t PROFILE_MAGIC = 0xA55A;
static const uint8_t  PROFILE_VER   = 1;

// ============================================================================
// 7. OBJETOS DE COMUNICACIÓN
// ============================================================================
SoftwareSerial btSerial(PIN_BT_RX, PIN_BT_TX);

// ============================================================================
// 8. ESTRUCTURAS Y VARIABLES DE ESTADO
// ============================================================================
// Parámetros de calibración eléctrica.
// dacCode4/dacCode20: códigos reales del MCP4725 que producen 4 y 20 mA.
// adcRaw4/adcRaw20: cuentas RAW del ADS1115 equivalentes a 4 y 20 mA.
struct CalibrationConfig {
  uint16_t dacCode4;
  uint16_t dacCode20;
  int16_t adcRaw4;
  int16_t adcRaw20;
};

// Cabecera persistente de cada perfil.
// Se guarda al comienzo del slot y permite validar cantidad de puntos,
// repeticiones y CRC antes de iniciar una ejecución automática.
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

// Un punto de un perfil: tiempo absoluto + corriente objetivo.
struct ProfilePoint {
  uint16_t timeDs;
  uint16_t currentCentiMa;
};

// Valores iniciales de respaldo.
// Serán reemplazados por la calibración guardada si la 24C512 contiene datos
// válidos. Los valores RAW del ADC son aproximados y deben calibrarse.
CalibrationConfig cal = {
  0,
  4095,
  5312,
  26560
};

// Banderas de presencia de periféricos detectados durante setup().
bool hasMcp4725 = false;
bool hasAds1115 = false;
bool has24c512 = false;

// Estado de la salida analógica.
bool outputEnabled = true;
float outputSetpointMa = 12.0f;
uint16_t currentDacCode = 0;

// Última medida obtenida desde el ADS1115.
int16_t lastAdcRaw = 0;
float inputMeasuredMa = 0.0f;
bool inputValid = false;

// Estado de la supervisión del lazo 4-20 mA.
// loopOpen=true      -> D8 confirma carga desconectada.
// loopAlarmArmed=true -> ya se validaron al menos 5 s continuos de carga.
bool loopOpen = false;
bool loopRawLast = false;
bool loopAlarmArmed = false;
uint32_t loopRawChangedMs = 0;
uint32_t loopClosedSinceMs = 0;

// Marcas temporales utilizadas por las tareas cooperativas basadas en millis().
uint32_t lastStatusMs = 0;
uint32_t lastAdcMs = 0;
uint32_t lastProfileDacMs = 0;

// Estado temporal mientras Simulink está transfiriendo un perfil a la EEPROM.
bool uploadActive = false;
uint8_t uploadSlot = 0;
uint16_t uploadExpected = 0;
uint16_t uploadReceived = 0;
uint16_t uploadRepeats = 1;

// Estado de ejecución autónoma.
// Solo se mantienen en RAM dos puntos consecutivos para interpolar entre ellos.
bool profileRunning = false;
uint8_t runningSlot = 0;
ProfileHeader runningHeader;
uint16_t runningRepeat = 0;
uint16_t runningIndex = 0;
ProfilePoint runningPointA;
ProfilePoint runningPointB;
bool runningHasB = false;
uint32_t runningStartMs = 0;

// Buffers de recepción ASCII.
// Se usan buffers fijos para evitar String y fragmentación de SRAM.
static const uint8_t LINE_BUF_SIZE = 112;
char btLine[LINE_BUF_SIZE];
uint8_t btLineLen = 0;
char usbLine[LINE_BUF_SIZE];
uint8_t usbLineLen = 0;

// ============================================================================
// 9. FUNCIONES AUXILIARES GENERALES
// ============================================================================
// Limita un valor float al intervalo [lo, hi].
static float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Equivalente para valores enteros sin signo de 16 bits.
static uint16_t clampU16(long v, uint16_t lo, uint16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return (uint16_t)v;
}

// Conversión explícita little-endian usada en la EEPROM para que el formato
// persistente no dependa de cómo el compilador empaquete structs.
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

// CRC16 Modbus (polinomio 0xA001).
// Se usa para detectar perfiles incompletos o datos corruptos en EEPROM.
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

// Comprueba si un dispositivo responde con ACK en una dirección I2C.
static bool i2cProbe(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

// ============================================================================
// 10. BUZZER ACTIVO - ALARMA EXCLUSIVA DE LAZO ABIERTO
// ============================================================================
// IMPORTANTE:
// El buzzer NO se usa como confirmación de arranque, guardado o perfil.
// Solo se enciende cuando:
//   1) la alarma ya fue armada tras >5 s de carga conectada, y
//   2) D8 indica posteriormente que la carga se desconectó.
// Aplica físicamente el estado del buzzer.
// Para el hardware actual BUZZER_PASSIVE=false y se usa digitalWrite().
static void buzzerSet(bool on) {
  if (BUZZER_PASSIVE) {
    if (on) tone(PIN_BUZZER, BUZZER_FREQ_HZ);
    else noTone(PIN_BUZZER);
  } else {
    digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
  }
}

// Tarea de alarma.
// No usa delay(), por lo que nunca bloquea Bluetooth, ADC ni perfiles.
static void buzzerTask() {
  // Una vez armada la supervisión, un lazo abierto mantiene el buzzer activo.
  // Si todavía no se armó o la carga está conectada, permanece apagado.
  buzzerSet(loopAlarmArmed && loopOpen);
}

// ============================================================================
// 11. MCP4725 - GENERACIÓN DE LA SALIDA 4-20 mA
// ============================================================================
// Envía un código de 12 bits al registro DAC volátil del MCP4725.
// Se utiliza el modo "fast write" al DAC; no se escribe la EEPROM interna
// del MCP4725 para evitar desgaste y porque la calibración vive en la 24C512.
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

// Convierte una consigna física de corriente a código DAC.
// La conversión es lineal entre los dos puntos reales de calibración:
// cal.dacCode4  <-> 4.00 mA
// cal.dacCode20 <-> 20.00 mA
static uint16_t currentToDacCode(float currentMa) {
  currentMa = clampFloat(currentMa, CURRENT_MIN_MA, CURRENT_MAX_MA);

  float spanCode = (float)((int32_t)cal.dacCode20 - (int32_t)cal.dacCode4);
  float ratio = (currentMa - CURRENT_MIN_MA) /
                (CURRENT_MAX_MA - CURRENT_MIN_MA);
  long code = lroundf((float)cal.dacCode4 + ratio * spanCode);
  return clampU16(code, 0, 4095);
}

// Aplica una consigna al sistema.
// Rechaza valores fuera de 4-20 mA antes de tocar el DAC.
static bool applyCurrentSetpoint(float currentMa) {
  if (currentMa < CURRENT_MIN_MA || currentMa > CURRENT_MAX_MA) return false;

  outputSetpointMa = currentMa;
  float appliedMa = outputEnabled ? outputSetpointMa : CURRENT_MIN_MA;
  return mcp4725WriteCode(currentToDacCode(appliedMa));
}

// ============================================================================
// 12. ADS1115 - MEDICIÓN DE LA ENTRADA 4-20 mA
// ============================================================================
// Escritura de un registro de 16 bits del ADS1115.
// El dispositivo transmite primero el byte más significativo.
static bool ads1115WriteRegister(uint8_t reg, uint16_t value) {
  if (!hasAds1115) return false;
  Wire.beginTransmission(ADS1115_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)(value & 0xFF));
  return Wire.endTransmission() == 0;
}

// Configuración actual: AIN0 single-ended, PGA ±6.144 V, modo continuo,
// 128 SPS y comparador deshabilitado.
// 0xC083 se deja centralizado aquí para poder cambiar el modo más adelante.
static bool ads1115Configure() {
  return ads1115WriteRegister(0x01, 0xC083);
}

// Lee el registro de conversión del ADS1115.
// Devuelve el valor RAW con signo de 16 bits.
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

// Convierte cuentas RAW a corriente mediante los dos puntos calibrados.
static float adcRawToCurrentMa(int16_t raw) {
  int32_t den = (int32_t)cal.adcRaw20 - (int32_t)cal.adcRaw4;
  if (den == 0) return 0.0f;

  float ratio = ((float)raw - (float)cal.adcRaw4) / (float)den;
  return CURRENT_MIN_MA +
         ratio * (CURRENT_MAX_MA - CURRENT_MIN_MA);
}

// Tarea periódica de adquisición.
// Se ejecuta cada ADC_PERIOD_MS sin detener el resto del firmware.
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

// ============================================================================
// 13. 24C512 - ACCESO DE BAJO NIVEL A EEPROM EXTERNA
// ============================================================================
// Después de una escritura, la EEPROM necesita unos milisegundos para
// completar internamente el ciclo. Durante ese tiempo no responde con ACK.
// Esta función realiza ACK polling en lugar de usar un delay fijo largo.
static bool eepromWaitReady(uint16_t timeoutMs = 15) {
  uint32_t start = millis();
  while ((uint32_t)(millis() - start) < timeoutMs) {
    Wire.beginTransmission(EEPROM_ADDR);
    if (Wire.endTransmission() == 0) return true;
    delay(1);
  }
  return false;
}

// Escribe un bloque respetando:
// - páginas físicas de 128 bytes de la 24C512;
// - el buffer I2C limitado del Arduino UNO.
// Por eso se transfieren como máximo 28 bytes útiles por transacción.
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

// Lectura secuencial de EEPROM en fragmentos pequeños compatibles con Wire.
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

// ============================================================================
// 14. ALMACENAMIENTO PERSISTENTE DE CALIBRACIÓN
// ============================================================================
// La calibración se guarda con magic, versión y CRC.
// Si cualquiera de esos controles falla al arrancar, se usan los valores
// predeterminados y se crea una nueva estructura válida en la EEPROM.
// Serializa la calibración a un formato fijo de 16 bytes y calcula CRC16.
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

// Recupera y valida la calibración almacenada.
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

// ============================================================================
// 15. PERFILES AUTOMÁTICOS GUARDADOS EN 24C512
// ============================================================================
// Cada slot dispone de una cabecera de 16 bytes y hasta 988 puntos.
// La ejecución es autónoma: una vez guardado un perfil, el teléfono puede
// desconectarse y el Arduino continúa generando la corriente.
// Devuelve la dirección absoluta donde comienza un slot.
static uint16_t profileSlotBase(uint8_t slot) {
  return EEPROM_PROFILE_BASE +
         (uint16_t)(slot - 1) * PROFILE_SLOT_SIZE;
}

// Los slots expuestos al usuario se numeran del 1 al 16.
static bool profileSlotValid(uint8_t slot) {
  return slot >= 1 && slot <= PROFILE_SLOT_COUNT;
}

// Guarda la cabecera en formato explícito para evitar padding de structs.
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

// Lee y reconstruye la cabecera de un perfil.
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

// Calcula la dirección de un punto específico dentro de un slot.
static uint16_t profilePointAddress(uint8_t slot, uint16_t zeroBasedIndex) {
  return profileSlotBase(slot) + PROFILE_HEADER_SIZE +
         zeroBasedIndex * PROFILE_POINT_SIZE;
}

// Escribe un punto de 4 bytes: tiempo + corriente.
static bool profileWritePoint(uint8_t slot, uint16_t zeroBasedIndex,
                              const ProfilePoint &p) {
  if (!profileSlotValid(slot) || zeroBasedIndex >= PROFILE_MAX_POINTS) return false;

  uint8_t b[PROFILE_POINT_SIZE];
  writeU16LE(&b[0], p.timeDs);
  writeU16LE(&b[2], p.currentCentiMa);
  return eepromWriteBlock(profilePointAddress(slot, zeroBasedIndex),
                          b, sizeof(b));
}

// Recupera un punto de perfil desde EEPROM.
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

// Calcula el CRC sobre todos los puntos del perfil.
static uint16_t profileCalculateCrc(uint8_t slot, uint16_t pointCount) {
  uint16_t crc = 0xFFFF;
  uint8_t b[PROFILE_POINT_SIZE];

  for (uint16_t i = 0; i < pointCount; i++) {
    if (!eepromReadBlock(profilePointAddress(slot, i), b, sizeof(b))) return 0;
    crc = crc16Buffer(b, sizeof(b), crc);
  }
  return crc;
}

// Antes de ejecutar se comprueban magic, versión, cantidad y CRC.
// De esta forma un perfil incompleto nunca controla la salida.
static bool profileValidate(uint8_t slot, ProfileHeader &h) {
  if (!profileReadHeader(slot, h)) return false;
  if (h.magic != PROFILE_MAGIC || h.version != PROFILE_VER) return false;
  if (h.pointCount < 1 || h.pointCount > PROFILE_MAX_POINTS) return false;
  if (h.repeats < 1) h.repeats = 1;

  uint16_t crc = profileCalculateCrc(slot, h.pointCount);
  return crc != 0 && crc == h.crc;
}

// Detiene la máquina de estados de ejecución sin borrar el perfil almacenado.
static void profileStop() {
  profileRunning = false;
  runningSlot = 0;
  runningIndex = 0;
  runningRepeat = 0;
  runningHasB = false;
}

// Inicia un perfil validado.
// Se precargan solo P0 y P1 para mantener el uso de SRAM muy bajo.
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
  return true;
}

// Reinicia el perfil cuando todavía quedan repeticiones configuradas.
static void profileRestartRepeat() {
  runningIndex = 0;
  profileReadPoint(runningSlot, 0, runningPointA);

  runningHasB = false;
  if (runningHeader.pointCount > 1) {
    if (profileReadPoint(runningSlot, 1, runningPointB)) runningHasB = true;
  }

  runningStartMs = millis();
}

// Máquina de estados no bloqueante de ejecución.
// Interpola linealmente la corriente entre dos puntos consecutivos en función
// del tiempo transcurrido desde el comienzo de la repetición.
static void profileTask() {
  if (!profileRunning) return;

  uint32_t now = millis();
  uint32_t elapsedMs = now - runningStartMs;

  // Si el tiempo ya superó uno o más puntos, avanzamos en la EEPROM hasta
  // volver a tener como A/B el segmento temporal que corresponde al presente.
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

  // Sin punto B significa que estamos en el último punto del perfil.
  if (!runningHasB) {
    uint32_t finalMs = (uint32_t)runningPointA.timeDs * 100UL;
    if (elapsedMs >= finalMs) {
      applyCurrentSetpoint((float)runningPointA.currentCentiMa / 100.0f);

      if ((uint16_t)(runningRepeat + 1) < runningHeader.repeats) {
        runningRepeat++;
        profileRestartRepeat();
      } else {
        profileStop();
      }
      return;
    }
  }

  // Limitamos cuántas veces por segundo se actualiza el DAC.
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

// ============================================================================
// 16. SUPERVISIÓN DE CARGA / LAZO ABIERTO
// ============================================================================
// D8=HIGH significa carga desconectada.
// La lógica de armado evita alarmas molestas cuando el equipo se enciende
// deliberadamente sin ninguna carga conectada.
// Normaliza el nivel eléctrico de D8 a una variable booleana:
// true = error/lazo abierto.
static bool readLoopErrorRaw() {
  return digitalRead(PIN_LOOP_ERROR) == LOOP_ERROR_ACTIVE_LEVEL;
}

// Tarea de supervisión:
// 1) aplica debounce;
// 2) mide cuánto tiempo lleva el lazo cerrado;
// 3) arma la alarma después de 5 s continuos;
// 4) no desarma automáticamente una vez validada la carga.
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

// ============================================================================
// 17. TELEMETRÍA Y ESTADO HACIA SIMULINK
// ============================================================================
// Imprime float con tres decimales sin usar String.
static void printFloat3(Stream &s, float v) {
  char tmp[18];
  dtostrf(v, 0, 3, tmp);
  s.print(tmp);
}

// Paquete compacto que puede enviarse tanto por Bluetooth como por USB.
// Ejemplo:
// DATA:OUT=12.000:IN=11.995:LOOP=CLOSED:ARM=1:RUN=0:SLOT=0:DAC=2048
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

// Emisión automática de telemetría.
// Durante la carga masiva de un perfil se pausa la telemetría Bluetooth para
// evitar intercalar DATA:... con los comandos PROFILE:POINT.
static void periodicStatusTask() {
  uint32_t now = millis();
  if ((uint32_t)(now - lastStatusMs) < STATUS_PERIOD_MS) return;
  lastStatusMs = now;

  if (!uploadActive) sendStatus(btSerial);
  sendStatus(Serial);
}

// ============================================================================
// 18. RESPUESTAS DEL PROTOCOLO ASCII
// ============================================================================
// Confirmación estándar de comando aceptado.
static void replyOK(Stream &s) {
  s.println(F("OK"));
}

// Respuesta estándar de error: ERR:<motivo>.
static void replyError(Stream &s, const __FlashStringHelper *msg) {
  s.print(F("ERR:"));
  s.println(msg);
}

// Envía calibraciones y capacidad de perfiles para diagnóstico.
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

// ============================================================================
// 19. CALIBRACIÓN DE ENTRADA Y SALIDA
// ============================================================================
// Recalibra la salida a partir de dos mediciones reales hechas por el usuario.
// Ejemplo: se ordenan 4 y 20 mA, se miden 4.07 y 19.82 mA, y con esos datos se
// corrigen los dos códigos extremos del DAC.
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

// Corrige la escala del ADC a partir de lo que el sistema estaba indicando
// cuando externamente se aplicaron 4 mA y 20 mA reales.
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

// ============================================================================
// 20. INTÉRPRETE DE COMANDOS ASCII
// ============================================================================
// Se utiliza strtok_r() sobre buffers char para ahorrar SRAM.
// No se utiliza la clase String.
// Subparser de PROFILE:*.
// Gestiona transferencia, validación, guardado, borrado y ejecución.
static void handleProfileCommand(Stream &src, char *savePtr) {
  char *op = strtok_r(NULL, ":", &savePtr);
  if (!op) {
    replyError(src, F("PROFILE_CMD"));
    return;
  }

  // PROFILE:NEW:<slot>:<count>[:<repeats>]
  // Inicia una transferencia e invalida primero la cabecera anterior.
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

  // PROFILE:POINT:<slot>:<indice>:<tiempo_s>:<mA>
  // Los puntos deben llegar consecutivamente para detectar paquetes perdidos.
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

  // PROFILE:SAVE:<slot>
  // Solo vuelve válido el perfil si llegaron TODOS los puntos y el CRC pudo
  // calcularse correctamente.
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
    replyOK(src);
    return;
  }

  // PROFILE:RUN:<slot>
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

  // PROFILE:STOP
  if (!strcmp(op, "STOP")) {
    profileStop();
    replyOK(src);
    return;
  }

  // PROFILE:DELETE:<slot>
  // Se borra lógicamente invalidando la cabecera; no hace falta limpiar todos
  // los bytes del slot.
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

  // PROFILE:LIST
  // Enumera cabeceras plausibles. La validación CRC completa se realiza al RUN.
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

// Subparser de CAL:*.
// TYPE/RANGE/CURRENT pertenecen al escalado de ingeniería de la PWA y por eso
// el firmware simplemente los reconoce; la calibración eléctrica real se hace
// con CODES/POINTS/RAW/CAPTURE.
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

// Parser de primer nivel.
// "src" puede ser btSerial o Serial, por lo que los mismos comandos pueden
// probarse desde el monitor USB y desde Simulink.
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

// ============================================================================
// 21. RECEPCIÓN DE LÍNEAS SERIE
// ============================================================================
// Acumula caracteres hasta CR/LF y luego entrega una línea completa al parser.
// Si una línea supera LINE_BUF_SIZE se descarta para proteger la memoria.
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

// ============================================================================
// 22. INICIALIZACIÓN
// ============================================================================
void setup() {
  // El buzzer se configura primero y se fuerza apagado para evitar un pulso
  // accidental durante el arranque.
  pinMode(PIN_BUZZER, OUTPUT);
  buzzerSet(false);

  pinMode(PIN_LOOP_ERROR,
          LOOP_ERROR_USE_PULLUP ? INPUT_PULLUP : INPUT);

  // Puerto USB para diagnóstico y puerto Bluetooth HC-05.
  Serial.begin(USB_BAUD);
  btSerial.begin(BT_BAUD);

  // Todos los dispositivos periféricos comparten el mismo bus I2C.
  // Se utiliza 100 kHz priorizando robustez sobre velocidad.
  Wire.begin();
  Wire.setClock(100000UL);

  delay(25);

  // Autodetección básica de cada dispositivo por ACK.
  hasMcp4725 = i2cProbe(MCP4725_ADDR);
  hasAds1115 = i2cProbe(ADS1115_ADDR);
  has24c512 = i2cProbe(EEPROM_ADDR);

  if (hasAds1115) ads1115Configure();

  // Si existe calibración válida se recupera; de lo contrario se guardan los
  // valores iniciales para dejar una estructura consistente en EEPROM.
  if (has24c512) {
    if (!loadCalibration()) saveCalibration();
  }

  // Estado inicial de D8:
  // siempre arrancamos DESARMADOS, aunque la carga ya esté conectada.
  // Si está conectada, el temporizador de armado comienza ahora.
  loopRawLast = readLoopErrorRaw();
  loopOpen = loopRawLast;
  loopRawChangedMs = millis();
  loopAlarmArmed = false;
  loopClosedSinceMs = loopOpen ? 0 : millis();

  // Consigna segura inicial en el punto medio del rango.
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
}

// ============================================================================
// 23. BUCLE PRINCIPAL
// ============================================================================
// No hay delays de operación normal. Cada subsistema ejecuta una pequeña tarea
// y devuelve el control rápidamente para mantener respuesta Bluetooth fluida.
void loop() {
  // 1) Atiende primero comunicaciones para minimizar latencia de comandos.
  serviceInput(btSerial, btLine, btLineLen);
  serviceInput(Serial, usbLine, usbLineLen);

  // 2) Adquisición de entrada.
  adcTask();

  // 3) Supervisión de la carga y armado de la alarma.
  loopErrorTask();

  // 4) Generación autónoma si hay un perfil ejecutándose.
  profileTask();

  // 5) El buzzer refleja exclusivamente una desconexión de carga armada.
  buzzerTask();

  // 6) Telemetría periódica hacia Simulink y monitor serie.
  periodicStatusTask();
}
