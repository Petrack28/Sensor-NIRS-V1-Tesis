/*
 * ADPD1080 — Dual Sensor NIRS/PPG (ESP32) v12 — Bluetooth Low Energy (BLE)
 * =================================================================================
 * Sensor 1: Wire  (SDA=21, SCL=22)  → 0x64
 * Sensor 2: Wire1 (SDA=18, SCL=19)  → 0x64
 *
 * COMUNICACIÓN — TODO LOCAL, SIN INTERNET NI LAPTOP:
 *   1) BLE (Bluetooth Low Energy) — el ESP32 anuncia un servicio GATT tipo
 *      "Nordic UART Service" (NUS), visible como "NIRS_ESP32". La app web
 *      (Chrome en Android, vía Web Bluetooth API) se conecta directo, sin
 *      pasar por ninguna red ni nube. Dos características:
 *        RX (escritura, app → ESP32): comandos, protocolo idéntico a v6/v7
 *        TX (notify, ESP32 → app): datos en vivo, ACKs, STATUS, archivos
 *      Los mensajes largos (líneas de archivo en base64) se fragmentan
 *      automáticamente en paquetes de ~180 bytes y se reensamblan del
 *      otro lado por el delimitador '\n' — igual de transparente que el
 *      Bluetooth SPP de la v6 original, pero vía BLE (compatible iPhone
 *      Y Android, aunque la app web con Web Bluetooth solo funciona en
 *      Android/Chrome; en iPhone necesitarías una app nativa).
 *   2) USB Serial (115200) — solo debug.
 *
 *   NOTA: la página web (HTML/JS) NO la sirve el ESP32 — vive en GitHub
 *   Pages (o donde la hospedes). El teléfono la carga UNA VEZ con datos
 *   móviles o cualquier WiFi (no importa cuál, ni si es la del ESP32);
 *   después de cargada, todo el control pasa por Bluetooth local. Esto
 *   significa que actualizar la interfaz (gráficas, botones, etc.) ya
 *   NUNCA requiere reflashear el ESP32 — solo resubir el index.html.
 *
 * Protocolo de comandos (idéntico a v6/v7, ahora por BLE o USB):
 *   IR_S1=0xXXXX / Rojo_S1=0xXXXX
 *   TIA_A_S1=0xXXXX / TIA_B_S1=0xXXXX / TIA_S1=0xXXXX
 *   PULSES_A_S1=0xXXXX / PULSES_B_S1=0xXXXX / PULSES_S1=0xXXXX
 *   FSAMPLE_S1=0xXXXX / AVG_S1=0xXXXX
 *   CHOP_S1=0/1 / VBIAS_S1=0/1 / STATUS_S1
 *   (ídem _S2, y globales IR= / Rojo= / TIA= / PULSES= / FSAMPLE= / AVG= / STATUS)
 *   RECORD_START:<nombre>:<isoDateTime> / RECORD_STOP / LIST_FILES / DELETE_FILE:<n>
 *   SPACE  → responde "SPACE:usados|total" (bytes en LittleFS)
 *   DOWNLOAD_FILE:<nombre>  → envía el archivo completo por BLE en pedacitos
 *     base64 con prefijo "FILEDATA:". Ver sección siguiente.
 *
 * DESCARGA DE ARCHIVOS POR BLE:
 *   La app manda DOWNLOAD_FILE:<nombre> por la característica RX. El ESP32
 *   responde por TX:
 *     ACK: FILE_START:<nombre>:<bytesTotal>:<totalChunks>
 *     FILEDATA:<nombre>|<indice>|<totalChunks>|<base64chunk>   (uno por línea, en orden)
 *     ACK: FILE_END:<nombre>
 *   El navegador reensambla los chunks y arma la descarga con un Blob.
 *
 * Formato de grabación en el ESP32 — BINARIO COMPACTO (8 bytes/muestra):
 *   uint32 t_ms (little-endian) + uint16 IR_ADC (LE) + uint16 Rojo_ADC (LE)
 *   Conviértelo a tu CSV final (tiempo_s,IR_ADC,IR_A,Rojo_ADC,Rojo_A) con
 *   "bin_to_csv_nirs.py" después de bajar el .bin + .json desde la app web.
 *
 * LIBRERÍAS NECESARIAS (Arduino IDE → Administrador de Bibliotecas):
 *   - Ninguna — BLEDevice/BLEServer/BLEUtils/BLE2902 y LittleFS vienen
 *     incluidas en el core ESP32. Cero librerías externas que instalar.
 *
 * IMPORTANTE — Partición de memoria:
 *   En Herramientas → Partition Scheme elige un esquema CON espacio para
 *   SPIFFS/LittleFS, p.ej. "No OTA (2MB APP/2MB SPIFFS)" — ya no hace
 *   falta tanto espacio de programa como con WiFi+MQTT+TLS+servidor web,
 *   así que este esquema sobra de espacio para tus archivos grabados.
 *
 * CAMBIOS vs v11 (WiFi+MQTT):
 *   - Se retira POR COMPLETO: WiFiManager, WiFiClientSecure, PubSubClient,
 *     ESPmDNS, ESPAsyncWebServer, AsyncTCP, y el HTML embebido. Cero
 *     dependencia de red, nube, HiveMQ, ni credenciales.
 *   - Se agrega servidor BLE (Nordic UART Service) — reemplaza a MQTT
 *     para comandos/datos en vivo/descarga de archivos.
 *   - Elimina de raíz todos los problemas de red anteriores: aislamiento
 *     de clientes en redes institucionales, contenido mixto HTTPS/HTTP,
 *     IPs que cambian, portal cautivo de WiFiManager.
 *   - Libera bastante RAM al quitar WiFi+TLS+servidor web — mucho más
 *     margen que cuando coexistían con Bluetooth clásico (la causa del
 *     crash que arreglamos en v9 quitando BT).
 *   - Contraparte: se pierde el control remoto "desde cualquier lugar"
 *     (ya no hay nube) y la app web con Bluetooth solo funciona en
 *     Android/Chrome (Web Bluetooth no existe en iPhone/Safari).
 */

#include <Wire.h>
#include <ctype.h>
#include <LittleFS.h>
#include "mbedtls/base64.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define DBG(x)   Serial.print(x)
#define DBGLN(x) Serial.println(x)

// -------------------------------------------------------
// BLE — Nordic UART Service (NUS), compatible además con apps
// genéricas de terminal BLE si alguna vez quieres probar sin la app web.
// -------------------------------------------------------
#define NUS_SERVICE_UUID  "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_CHAR_RX_UUID  "6e400002-b5a3-f393-e0a9-e50e24dcca9e" // app -> ESP32 (Write)
#define NUS_CHAR_TX_UUID  "6e400003-b5a3-f393-e0a9-e50e24dcca9e" // ESP32 -> app (Notify)

BLEServer*         bleServer      = nullptr;
BLECharacteristic*  bleTxChar     = nullptr;
BLECharacteristic*  bleRxChar     = nullptr;
volatile bool       bleConnected  = false;
const char*          BLE_DEVICE_NAME = "NIRS_ESP32";

// -------------------------------------------------------
// Aviso visual de conexión BLE — no hay buzzer en el hardware, así que
// el LED rojo (LED2/Slot B) de ambos sensores parpadea 5 veces en 1
// segundo al conectarse. Se maneja como máquina de estados no bloqueante
// en loop() (con millis()) en vez de delay() dentro del callback de BLE,
// que corre en el hilo del stack BLE y no debe bloquearse.
// -------------------------------------------------------
volatile bool  g_blinkActive        = false;
uint8_t        g_blinkToggleCount   = 0;
bool           g_blinkLedOn         = false;
uint32_t       g_lastBlinkToggleMs  = 0;
const uint32_t BLINK_TOGGLE_MS      = 100; // 5 Hz → on/off cada 100 ms
const uint8_t  BLINK_TOTAL_TOGGLES  = 10;  // 10 cambios = 5 parpadeos completos en 1 s

// -------------------------------------------------------
// Pines I2C
// -------------------------------------------------------
const int I2C0_SDA = 21, I2C0_SCL = 22;
const int I2C1_SDA = 18, I2C1_SCL = 19;
const uint8_t ADPD_ADDR = 0x64;


// -------------------------------------------------------
// Registros ADPD1080
// -------------------------------------------------------
const uint8_t REG_MODE          = 0x10;
const uint8_t REG_SLOT_EN       = 0x11;
const uint8_t REG_FSAMPLE       = 0x12;
const uint8_t REG_PD_LED_SEL    = 0x14;
const uint8_t REG_NUM_AVG       = 0x15;
const uint8_t REG_ILED1_COARSE  = 0x23;
const uint8_t REG_ILED2_COARSE  = 0x24;
const uint8_t REG_LED_DISABLE   = 0x34;
const uint8_t REG_SLOTA_PULSES  = 0x31;
const uint8_t REG_SLOTB_PULSES  = 0x36;
const uint8_t REG_SLOTA_TIA     = 0x42;
const uint8_t REG_SLOTB_TIA     = 0x44;
const uint8_t REG_CLOCK_32K     = 0x4B;
const uint8_t REG_SLOTA_CH1     = 0x64;
const uint8_t REG_SLOTB_CH1     = 0x68;
const uint8_t REG_INTEG_ORDER_A = 0x17;
const uint8_t REG_INTEG_ORDER_B = 0x1D;
const uint8_t REG_MATH          = 0x58;
const uint8_t REG_SLOTA_CH1_OFF = 0x18;
const uint8_t REG_SLOTA_CH2_OFF = 0x19;
const uint8_t REG_SLOTA_CH3_OFF = 0x1A;
const uint8_t REG_SLOTA_CH4_OFF = 0x1B;
const uint8_t REG_SLOTB_CH1_OFF = 0x1E;
const uint8_t REG_SLOTB_CH2_OFF = 0x1F;
const uint8_t REG_SLOTB_CH3_OFF = 0x20;
const uint8_t REG_SLOTB_CH4_OFF = 0x21;
const uint8_t REG_PD_BIAS       = 0x54;

const uint16_t ENABLE_BOTH_SLOTS = 0x1021;
const uint16_t MODE_PROGRAM      = 0x0001;
const uint16_t MODE_NORMAL       = 0x0002;

// -------------------------------------------------------
// Estado de cada sensor
// -------------------------------------------------------
struct SensorState {
  uint16_t led1;
  uint16_t led2;
  uint16_t tia_a;
  uint16_t tia_b;
  uint16_t pulses_a;
  uint16_t pulses_b;
  uint16_t led_dis;
  uint16_t fsample;
  uint16_t avg;
  bool     chop;
  bool     vbias;
};

SensorState S1 = {
  .led1     = 0x3008,
  .led2     = 0x3008,
  .tia_a    = 0x1C38,
  .tia_b    = 0x1C38,
  .pulses_a = (8 << 8) | 0x18,
  .pulses_b = (8 << 8) | 0x18,
  .led_dis  = 0x0000,
  .fsample  = 0x0010,   // 500 Hz
  .avg      = 0x0000,
  .chop     = false,
  .vbias    = false,
};

SensorState S2 = {
  .led1     = 0x3008,
  .led2     = 0x3008,
  .tia_a    = 0x1C38,
  .tia_b    = 0x1C38,
  .pulses_a = (8 << 8) | 0x18,
  .pulses_b = (8 << 8) | 0x18,
  .led_dis  = 0x0000,
  .fsample  = 0x0010,   // 500 Hz
  .avg      = 0x0000,
  .chop     = false,
  .vbias    = false,
};

uint32_t g_loop_period_us = 2000;

// -------------------------------------------------------
// Estado de grabación CSV (LittleFS)
// -------------------------------------------------------
bool     g_recording   = false;
String   g_rec_name    = "";
File     f_s1, f_s2;
uint8_t  buf_s1_bin[4096], buf_s2_bin[4096];   // buffer binario antes de flush a flash
size_t   buf_s1_len = 0, buf_s2_len = 0;
uint32_t rec_start_ms  = 0;
double   f_ir1 = 0, f_rojo1 = 0, f_ir2 = 0, f_rojo2 = 0;

// Throttle de difusión en vivo por BLE (no limita la grabación local)
uint32_t g_last_ble_broadcast_ms = 0;
const uint32_t BLE_BROADCAST_INTERVAL_MS = 50; // 20 Hz — BLE local aguanta más que MQTT en la nube

// =========================================================
// SALIDA UNIFICADA — BLE (una sola característica TX tipo "stream",
// igual que hacía SerialBT.println en la v6 original)
// printStatus() arma cada línea con varias llamadas a btPrint()
// seguidas de btPrintln() — se acumulan en _outBuf y se envían
// como una sola línea BLE en cada btPrintln/btPrintlnHex.
// =========================================================
String _outBuf = "";
const size_t BLE_NOTIFY_CHUNK = 180; // bytes/paquete — conservador, funciona con MTU bajo

// Envía una línea completa por BLE, fragmentándola en paquetes de
// notify si es más larga que el tamaño de paquete (p.ej. chunks de
// archivo en base64). El '\n' al final marca fin de línea para que
// la app web sepa dónde cortar al reensamblar el stream de bytes.
void bleSendLine(const String &s) {
  if (!bleConnected || !bleTxChar) return;
  String withNl = s + "\n";
  size_t len = withNl.length();
  size_t offset = 0;
  while (offset < len) {
    size_t chunkLen = min(BLE_NOTIFY_CHUNK, len - offset);
    bleTxChar->setValue((uint8_t*)(withNl.c_str() + offset), chunkLen);
    bleTxChar->notify();
    offset += chunkLen;
    delay(3); // pequeño respiro entre paquetes, evita saturar el stack BLE
  }
}
void sendReply(const String &s) { bleSendLine(s); }
void btPrint(const char* s)   { _outBuf += s; }
void btPrintln(const char* s) { _outBuf += s; sendReply(_outBuf); _outBuf = ""; }
void btPrint(uint16_t v)      { _outBuf += String(v); }
void btPrintln(uint16_t v)    { _outBuf += String(v); sendReply(_outBuf); _outBuf = ""; }
void btPrint(uint32_t v)      { _outBuf += String(v); }
void btPrintln(uint32_t v)    { _outBuf += String(v); sendReply(_outBuf); _outBuf = ""; }
void btPrint(int v)           { _outBuf += String(v); }
void btPrintln(int v)         { _outBuf += String(v); sendReply(_outBuf); _outBuf = ""; }
void btPrintHex(uint16_t v)   { _outBuf += String(v, HEX); }
void btPrintlnHex(uint16_t v) { _outBuf += String(v, HEX); sendReply(_outBuf); _outBuf = ""; }

// =========================================================
// Período de muestreo real (idéntico a v6)
// =========================================================
uint32_t sensorPeriodUs(SensorState &s) {
  uint8_t pa = (s.pulses_a >> 8) & 0xFF;
  uint8_t pb = (s.pulses_b >> 8) & 0xFF;
  if (pa < 1) pa = 1;
  if (pb < 1) pb = 1;
  uint32_t fmax_us = 25UL + pa*24UL + 68UL + 25UL + pb*24UL + 20UL + 222UL;
  uint32_t fsample_us = (uint32_t)s.fsample * 125UL;
  uint32_t base_us = max(fmax_us, fsample_us);
  uint8_t avg_bits = (s.avg >> 4) & 0x7;
  uint32_t avg_f   = (avg_bits > 0) ? (1UL << avg_bits) : 1UL;
  return base_us * avg_f;
}

void recalcLoopPeriod() {
  uint32_t p1 = sensorPeriodUs(S1);
  uint32_t p2 = sensorPeriodUs(S2);
  g_loop_period_us = max(p1, p2);
  g_loop_period_us = constrain(g_loop_period_us, 500UL, 500000UL);
}

// =========================================================
// I2C HELPERS
// =========================================================
void writeReg(TwoWire &bus, uint8_t reg, uint16_t val) {
  bus.beginTransmission(ADPD_ADDR);
  bus.write(reg);
  bus.write((val >> 8) & 0xFF);
  bus.write(val & 0xFF);
  bus.endTransmission();
}

uint16_t readReg(TwoWire &bus, uint8_t reg) {
  bus.beginTransmission(ADPD_ADDR);
  bus.write(reg);
  bus.endTransmission(false);
  bus.requestFrom(ADPD_ADDR, (uint8_t)2);
  if (bus.available() == 2)
    return ((uint16_t)bus.read() << 8) | bus.read();
  return 0;
}

// =========================================================
// PERSISTENCIA DE CONFIGURACIÓN EN FLASH (LittleFS)
// Guarda S1/S2 cada vez que se aplica un cambio de config (LED,
// TIA, pulsos, fSAMPLE, avg, VBias) para que el ESP32 arranque con
// la última configuración usada — pedido por el Modo Desarrollador
// de la app web — en vez de siempre volver a los valores de fábrica.
// =========================================================
const char*   CONFIG_FILE  = "/sensor_cfg.bin";
const uint8_t CONFIG_MAGIC[4] = { 'N', 'I', 'R', '1' };

void writeSensorStateBin(File &f, SensorState &s) {
  uint16_t buf[9] = { s.led1, s.led2, s.tia_a, s.tia_b, s.pulses_a,
                       s.pulses_b, s.led_dis, s.fsample, s.avg };
  f.write((uint8_t*)buf, sizeof(buf));
  uint8_t flags = (s.chop ? 1 : 0) | (s.vbias ? 2 : 0);
  f.write(&flags, 1);
}

void readSensorStateBin(File &f, SensorState &s) {
  uint16_t buf[9];
  f.read((uint8_t*)buf, sizeof(buf));
  s.led1=buf[0]; s.led2=buf[1]; s.tia_a=buf[2]; s.tia_b=buf[3];
  s.pulses_a=buf[4]; s.pulses_b=buf[5]; s.led_dis=buf[6];
  s.fsample=buf[7]; s.avg=buf[8];
  uint8_t flags = 0;
  f.read(&flags, 1);
  s.chop  = flags & 1;
  s.vbias = flags & 2;
}

void saveSensorConfig() {
  File f = LittleFS.open(CONFIG_FILE, "w");
  if (!f) return;
  f.write(CONFIG_MAGIC, 4);
  writeSensorStateBin(f, S1);
  writeSensorStateBin(f, S2);
  f.close();
}

bool loadSensorConfig() {
  File f = LittleFS.open(CONFIG_FILE, "r");
  if (!f) return false;
  uint8_t magic[4];
  bool ok = (f.read(magic, 4) == 4);
  for (int i = 0; ok && i < 4; i++) if (magic[i] != CONFIG_MAGIC[i]) ok = false;
  if (!ok) { f.close(); return false; }
  readSensorStateBin(f, S1);
  readSensorStateBin(f, S2);
  f.close();
  return true;
}

// =========================================================
// INICIALIZACIÓN SENSOR
// =========================================================
bool initSensor(TwoWire &bus, SensorState &s, const char* name) {
  bus.beginTransmission(ADPD_ADDR);
  if (bus.endTransmission() != 0) {
    Serial.print("ERROR: "); Serial.print(name); Serial.println(" no detectado.");
    return false;
  }
  writeReg(bus, REG_CLOCK_32K,    0x2692);
  writeReg(bus, REG_MODE,         MODE_PROGRAM);
  writeReg(bus, REG_FSAMPLE,      s.fsample);
  writeReg(bus, REG_NUM_AVG,      s.avg);
  writeReg(bus, REG_PD_LED_SEL,   0x0559);
  writeReg(bus, REG_ILED1_COARSE, s.led1);
  writeReg(bus, REG_ILED2_COARSE, s.led2);
  writeReg(bus, REG_SLOTA_TIA,    s.tia_a);
  writeReg(bus, REG_SLOTB_TIA,    s.tia_b);
  writeReg(bus, REG_SLOTA_PULSES, s.pulses_a);
  writeReg(bus, REG_SLOTB_PULSES, s.pulses_b);
  // Los LEDs arrancan APAGADOS al energizar el sensor, sin importar el
  // último estado guardado en flash — solo se restaura la corriente
  // (led1/led2) para que al encenderlos manualmente usen el valor
  // correcto. Evita que el sensor gaste energía con los LEDs prendidos
  // sin que nadie los esté usando todavía.
  s.led_dis |= (1 << 8) | (1 << 9);
  writeReg(bus, REG_LED_DISABLE,  s.led_dis);
  writeReg(bus, REG_SLOT_EN,      ENABLE_BOTH_SLOTS);
  writeReg(bus, REG_MODE,         MODE_NORMAL);
  Serial.print("OK: "); Serial.print(name); Serial.println(" inicializado.");
  return true;
}

// =========================================================
// COMANDOS AFE (idénticos a v6)
// =========================================================
void applyLED(TwoWire &bus, SensorState &s, bool isIR, uint16_t val) {
  if (isIR) {
    if (val == 0) { s.led_dis |=  (1 << 8); }
    else          { s.led_dis &= ~(1 << 8); s.led1 = val; writeReg(bus, REG_ILED1_COARSE, val); }
  } else {
    if (val == 0) { s.led_dis |=  (1 << 9); }
    else          { s.led_dis &= ~(1 << 9); s.led2 = val; writeReg(bus, REG_ILED2_COARSE, val); }
  }
  writeReg(bus, REG_LED_DISABLE, s.led_dis);
  saveSensorConfig();
}

void applyTIA(TwoWire &bus, SensorState &s, bool slotA, bool slotB, uint16_t val) {
  writeReg(bus, REG_MODE, MODE_PROGRAM);
  if (slotA) { s.tia_a = val; writeReg(bus, REG_SLOTA_TIA, val); }
  if (slotB) { s.tia_b = val; writeReg(bus, REG_SLOTB_TIA, val); }
  writeReg(bus, REG_MODE, MODE_NORMAL);
  saveSensorConfig();
}

void applyPulses(TwoWire &bus, SensorState &s, bool slotA, bool slotB, uint16_t val) {
  writeReg(bus, REG_MODE, MODE_PROGRAM);
  if (slotA) { s.pulses_a = val; writeReg(bus, REG_SLOTA_PULSES, val); }
  if (slotB) { s.pulses_b = val; writeReg(bus, REG_SLOTB_PULSES, val); }
  writeReg(bus, REG_MODE, MODE_NORMAL);
  // BUG FIX (v13): applyFsample() y applyAvg() siempre recalculaban el
  // período del loop tras cambiar su registro — pero applyPulses() NO,
  // a pesar de que sensorPeriodUs() sí depende de pulses_a/pulses_b (el
  // datasheet confirma que más pulsos = más tiempo por muestra:
  // tA = SLOTA_LED_OFFSET + nA × SLOTA_PERIOD). Sin este recálculo, el
  // ESP32 seguía leyendo al ritmo viejo, desincronizado del chip real,
  // y podía releer el mismo dato varias veces seguidas tras subir los
  // pulsos — exactamente el síntoma reportado ("no cambia nada en la
  // señal"). Bug heredado del firmware original v6, nunca se había
  // notado porque nadie había probado este caso específico.
  recalcLoopPeriod();
  saveSensorConfig();
}

void applyFsample(TwoWire &bus, SensorState &s, uint16_t val) {
  if (val < 1) val = 1;
  s.fsample = val;
  writeReg(bus, REG_MODE,    MODE_PROGRAM);
  writeReg(bus, REG_FSAMPLE, val);
  writeReg(bus, REG_MODE,    MODE_NORMAL);
  recalcLoopPeriod();
  uint32_t hz = 32000UL / ((uint32_t)val * 4);
  DBG("  FSAMPLE aplicado: 0x"); Serial.print(val, HEX);
  DBG(" = "); Serial.print(hz); DBGLN(" Hz");
  saveSensorConfig();
}

void applyAvg(TwoWire &bus, SensorState &s, uint16_t val) {
  s.avg = val;
  writeReg(bus, REG_MODE,    MODE_PROGRAM);
  writeReg(bus, REG_NUM_AVG, val);
  writeReg(bus, REG_MODE,    MODE_NORMAL);
  recalcLoopPeriod();
  saveSensorConfig();
}

void applyChop(TwoWire &bus, SensorState &s, bool en) {
  // Deshabilitado — ver nota histórica en v6 (config incompleta causaba señal en 0)
  s.chop = false;
}

void applyVbias(TwoWire &bus, SensorState &s, bool en) {
  s.vbias = en;
  writeReg(bus, REG_MODE,    MODE_PROGRAM);
  writeReg(bus, REG_PD_BIAS, en ? 0x0A80 : 0x0000);
  writeReg(bus, REG_MODE,    MODE_NORMAL);
  saveSensorConfig();
}

void printStatus(TwoWire &bus, SensorState &s, const char* name) {
  uint32_t hz = 32000UL / ((uint32_t)readReg(bus, REG_FSAMPLE) * 4);
  Serial.print("--- STATUS "); Serial.print(name); Serial.println(" ---");
  Serial.print("MODE=0x");     Serial.println(readReg(bus, REG_MODE), HEX);
  Serial.print("FSAMPLE=0x");  Serial.print(readReg(bus, REG_FSAMPLE), HEX);
  Serial.print(" ("); Serial.print(hz); Serial.println(" Hz)");
  Serial.print("NUM_AVG=0x");  Serial.println(readReg(bus, REG_NUM_AVG), HEX);
  Serial.print("PULSES_A=0x"); Serial.println(readReg(bus, REG_SLOTA_PULSES), HEX);
  Serial.print("PULSES_B=0x"); Serial.println(readReg(bus, REG_SLOTB_PULSES), HEX);
  Serial.print("Loop period="); Serial.print(g_loop_period_us); Serial.println(" us");
  Serial.println("--- FIN ---");

  btPrint("--- STATUS "); btPrintln(name);
  btPrint("MODE=0x");     btPrintlnHex(readReg(bus, REG_MODE));
  btPrint("FSAMPLE=0x");  btPrintHex(readReg(bus, REG_FSAMPLE));
  btPrint(" ("); btPrint(hz); btPrintln(" Hz)");
  btPrint("NUM_AVG=0x");  btPrintlnHex(readReg(bus, REG_NUM_AVG));
  btPrint("TIA_A=0x");    btPrintlnHex(readReg(bus, REG_SLOTA_TIA));
  btPrint("TIA_B=0x");    btPrintlnHex(readReg(bus, REG_SLOTB_TIA));
  btPrint("PULSES_A=0x"); btPrintlnHex(readReg(bus, REG_SLOTA_PULSES));
  btPrint("PULSES_B=0x"); btPrintlnHex(readReg(bus, REG_SLOTB_PULSES));
  btPrint("LED1=0x");     btPrintlnHex(readReg(bus, REG_ILED1_COARSE));
  btPrint("LED2=0x");     btPrintlnHex(readReg(bus, REG_ILED2_COARSE));
  // LED_DIS y VBIAS no son registros derivables de una sola lectura I2C de
  // forma limpia (LED_DIS sí, pero VBias se maneja como estado en RAM) —
  // se reportan desde el SensorState en memoria para que la app web pueda
  // reconstruir el estado ON/OFF real de los LEDs y de VBias al conectar.
  btPrint("LED_DIS=0x");  btPrintlnHex(s.led_dis);
  btPrint("VBIAS=");      btPrintln(s.vbias ? 1 : 0);
  btPrint("Loop period="); btPrint(g_loop_period_us); btPrintln(" us");
  btPrintln("--- FIN ---");
}

// =========================================================
// CONVERSIÓN A AMPERIOS — misma fórmula que usaba la GUI Python
// (VREF=1.0, I(A) = ADC * VREF / (65535 * RF * N_pulsos))
// =========================================================
double rfOhmFromTiaReg(uint16_t tia_reg) {
  switch (tia_reg & 0x3) {
    case 0: return 200000.0;
    case 1: return 100000.0;
    case 2: return 50000.0;
    case 3: return 25000.0;
  }
  return 200000.0;
}

double factorAmperiosPorCuenta(uint16_t tia_reg, uint8_t pulses_n) {
  double rf = rfOhmFromTiaReg(tia_reg);
  if (rf <= 0 || pulses_n < 1) return 0;
  return 1.0 / (65535.0 * rf * (double)pulses_n);
}

double ledCurrentMa(uint16_t reg) {
  if (reg == 0) return 0;
  uint8_t scale  = (reg >> 13) & 0x1;
  uint8_t coarse = reg & 0xF;
  const double FINE_FACTOR = 1.004; // 0.74 + 0.022*12 (ILED_FINE_DEFAULT=0xC)
  double ma = (50.3 + 19.8 * coarse) * FINE_FACTOR * (0.1 + 0.9 * scale);
  return round(ma * 10.0) / 10.0;
}

// =========================================================
// GRABACIÓN CSV (LittleFS)
// =========================================================
void writeSensorConfigJson(File &jf, const char* label, SensorState &s, double fir, double frojo) {
  uint8_t pa = (s.pulses_a >> 8) & 0xFF;
  uint8_t pb = (s.pulses_b >> 8) & 0xFF;
  double fs_hz  = s.fsample ? (32000.0 / ((double)s.fsample * 4.0)) : 0.0;
  uint8_t avg_bits = (s.avg >> 4) & 0x7;
  uint32_t avg_f   = (avg_bits > 0) ? (1UL << avg_bits) : 1UL;

  jf.print("  \""); jf.print(label); jf.println("\": {");
  jf.print("    \"fSAMPLE_Hz\": ");     jf.print(fs_hz, 1);                 jf.println(",");
  jf.print("    \"promediado_factor\": "); jf.print(avg_f);                 jf.println(",");
  jf.print("    \"IR_RF_A_ohm\": ");    jf.print(rfOhmFromTiaReg(s.tia_a), 0); jf.println(",");
  jf.print("    \"IR_pulsos_N\": ");    jf.print(pa);                       jf.println(",");
  jf.print("    \"IR_corriente_LED_mA\": "); jf.print(ledCurrentMa(s.led1)); jf.println(",");
  jf.print("    \"IR_factor_escala_A_por_cuenta\": "); jf.print(fir, 12);   jf.println(",");
  jf.print("    \"Rojo_RF_B_ohm\": ");  jf.print(rfOhmFromTiaReg(s.tia_b), 0); jf.println(",");
  jf.print("    \"Rojo_pulsos_N\": ");  jf.print(pb);                       jf.println(",");
  jf.print("    \"Rojo_corriente_LED_mA\": "); jf.print(ledCurrentMa(s.led2)); jf.println(",");
  jf.print("    \"Rojo_factor_escala_A_por_cuenta\": "); jf.print(frojo, 12); jf.println(",");
  jf.print("    \"chop_mode\": ");      jf.print(s.chop ? "true" : "false"); jf.println(",");
  jf.print("    \"vbias\": ");          jf.print(s.vbias ? "true" : "false"); jf.println();
  jf.print("  }");
}

void writeConfigJson(const String &name, const String &isoDate) {
  File jf = LittleFS.open("/" + name + "_config.json", "w");
  if (!jf) return;
  jf.println("{");
  jf.print("  \"fecha_hora\": \""); jf.print(isoDate); jf.println("\",");
  jf.println("  \"VREF_V\": 1.0,");
  jf.println("  \"ADC_max_cuentas\": 65535,");
  jf.println("  \"formato_bin\": \"registros consecutivos de 8 bytes: uint32 t_ms (little-endian) + uint16 IR_ADC (LE) + uint16 Rojo_ADC (LE)\",");
  writeSensorConfigJson(jf, "S1", S1, f_ir1, f_rojo1);
  jf.println(",");
  writeSensorConfigJson(jf, "S2", S2, f_ir2, f_rojo2);
  jf.println();
  jf.println("}");
  jf.close();
}

String sanitizeName(const String &raw) {
  String clean = "";
  for (unsigned int i = 0; i < raw.length(); i++) {
    char c = raw.charAt(i);
    if (isalnum((int)c) || c == '_' || c == '-') clean += c;
  }
  if (clean.length() == 0) clean = "registro";
  if (clean.length() > 40) clean = clean.substring(0, 40);
  return clean;
}

bool startRecording(const String &rawName, const String &isoDate) {
  if (g_recording) return false;
  String clean = sanitizeName(rawName);
  g_rec_name = clean;

  // Formato binario compacto: cada registro = 8 bytes
  //   uint32 t_ms (LE)  |  uint16 IR_ADC (LE)  |  uint16 CH2_ADC (LE)
  // Documentado también en <nombre>_config.json → "formato_bin".
  String p1 = "/" + clean + "_S1_datos.bin";
  String p2 = "/" + clean + "_S2_datos.bin";
  f_s1 = LittleFS.open(p1, "w");
  f_s2 = LittleFS.open(p2, "w");
  if (!f_s1 || !f_s2) {
    sendReply("ERR: no se pudo crear archivo (revisa espacio en LittleFS)");
    return false;
  }

  uint8_t pa1 = (S1.pulses_a >> 8) & 0xFF, pb1 = (S1.pulses_b >> 8) & 0xFF;
  uint8_t pa2 = (S2.pulses_a >> 8) & 0xFF, pb2 = (S2.pulses_b >> 8) & 0xFF;
  f_ir1   = factorAmperiosPorCuenta(S1.tia_a, pa1);
  f_rojo1 = factorAmperiosPorCuenta(S1.tia_b, pb1);
  f_ir2   = factorAmperiosPorCuenta(S2.tia_a, pa2);
  f_rojo2 = factorAmperiosPorCuenta(S2.tia_b, pb2);

  writeConfigJson(clean, isoDate);

  buf_s1_len = 0; buf_s2_len = 0;
  rec_start_ms = millis();
  g_recording = true;
  sendReply("ACK: RECORD_START:" + clean);
  return true;
}

void stopRecording() {
  if (!g_recording) return;
  if (buf_s1_len) { f_s1.write(buf_s1_bin, buf_s1_len); buf_s1_len = 0; }
  if (buf_s2_len) { f_s2.write(buf_s2_bin, buf_s2_len); buf_s2_len = 0; }
  f_s1.close();
  f_s2.close();
  g_recording = false;
  sendReply("ACK: RECORD_STOP:" + g_rec_name);
}

// Empaqueta un registro de 8 bytes: t_ms(4) + ch1_ADC(2) + ch2_ADC(2), todo LE.
inline void packRecord(uint8_t *dst, uint32_t t_ms, uint16_t ch1, uint16_t ch2) {
  dst[0] = t_ms & 0xFF; dst[1] = (t_ms>>8)&0xFF; dst[2] = (t_ms>>16)&0xFF; dst[3] = (t_ms>>24)&0xFF;
  dst[4] = ch1 & 0xFF;  dst[5] = (ch1>>8)&0xFF;
  dst[6] = ch2 & 0xFF;  dst[7] = (ch2>>8)&0xFF;
}

void recordSample(uint16_t ir1, uint16_t red1, uint16_t ir2, uint16_t red2) {
  uint32_t t_ms = millis() - rec_start_ms;

  if (buf_s1_len + 8 > sizeof(buf_s1_bin)) { f_s1.write(buf_s1_bin, buf_s1_len); buf_s1_len = 0; }
  packRecord(&buf_s1_bin[buf_s1_len], t_ms, ir1, red1);
  buf_s1_len += 8;

  if (buf_s2_len + 8 > sizeof(buf_s2_bin)) { f_s2.write(buf_s2_bin, buf_s2_len); buf_s2_len = 0; }
  packRecord(&buf_s2_bin[buf_s2_len], t_ms, ir2, red2);
  buf_s2_len += 8;
}

// Lista archivos guardados como "FILES:nombre1|tam1;nombre2|tam2;..."
void listFiles() {
  String out = "FILES:";
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  bool first = true;
  while (file) {
    if (!file.isDirectory()) {
      String nm = String(file.name());
      if (nm.startsWith("/")) nm = nm.substring(1);
      // El archivo de configuración del sensor no es una grabación del
      // usuario — se excluye de la lista para no mezclarlo con los
      // archivos de datos (ver también el bloqueo en DELETE_FILE).
      if (nm != String(CONFIG_FILE + 1)) {
        if (!first) out += ";";
        out += nm + "|" + String(file.size());
        first = false;
      }
    }
    file = root.openNextFile();
  }
  sendReply(out);
}

// Reporta espacio usado/total de LittleFS como "SPACE:usados|total"
void reportSpace() {
  sendReply("SPACE:" + String(LittleFS.usedBytes()) + "|" + String(LittleFS.totalBytes()));
}

// =========================================================
// DESCARGA DE ARCHIVOS POR MQTT (sin depender de la red local)
// Envía el archivo en pedacitos codificados en base64 por el tópico
// nirs/filedata: "<nombre>|<indice>|<totalChunks>|<base64>"
// Antes/después envía por BLE:
//   ACK: FILE_START:<nombre>:<bytesTotal>:<totalChunks>
//   ACK: FILE_END:<nombre>
// Cada chunk se manda como una línea "FILEDATA:<nombre>|<indice>|<totalChunks>|<base64>"
// — el prefijo FILEDATA: permite distinguirlos del resto de mensajes de
// estado dentro del mismo stream BLE (antes viajaban en un tópico MQTT
// aparte; ahora todo comparte una sola característica TX).
// =========================================================
const size_t FILE_CHUNK_RAW = 768; // múltiplo de 3 → base64 exacto, sin relleno

void sendFileOverBle(const String &rawName) {
  if (g_recording) {
    sendReply("ERR: no se puede descargar mientras hay una grabación activa");
    return;
  }
  String fn = rawName;
  fn.trim();
  if (!fn.startsWith("/")) fn = "/" + fn;

  if (!LittleFS.exists(fn)) {
    sendReply("ERR: archivo no encontrado: " + rawName);
    return;
  }
  File f = LittleFS.open(fn, "r");
  if (!f) {
    sendReply("ERR: no se pudo abrir: " + rawName);
    return;
  }

  size_t total = f.size();
  size_t totalChunks = (total + FILE_CHUNK_RAW - 1) / FILE_CHUNK_RAW;
  if (totalChunks == 0) totalChunks = 1; // archivo vacío, igual mandamos FILE_END

  String nombreLimpio = fn.startsWith("/") ? fn.substring(1) : fn;
  sendReply("ACK: FILE_START:" + nombreLimpio + ":" + String(total) + ":" + String(totalChunks));

  uint8_t  rawBuf[FILE_CHUNK_RAW];
  // base64 de 768 bytes = 1024 caracteres exactos + 1 byte para '\0'
  char     b64Buf[1040];

  for (size_t idx = 0; idx < totalChunks; idx++) {
    size_t n = f.read(rawBuf, FILE_CHUNK_RAW);
    size_t outLen = 0;
    mbedtls_base64_encode((unsigned char*)b64Buf, sizeof(b64Buf), &outLen, rawBuf, n);
    b64Buf[outLen] = '\0';

    String msg = "FILEDATA:" + nombreLimpio + "|" + String(idx) + "|" + String(totalChunks) + "|" + String(b64Buf);
    bleSendLine(msg);
    delay(5); // pequeño respiro para no saturar el stack BLE
  }
  f.close();
  sendReply("ACK: FILE_END:" + nombreLimpio);
}

// =========================================================
// PARSER DE COMANDOS (idéntico a v6 + comandos nuevos de grabación)
// =========================================================
uint16_t parseHex(const String &cmd, int offset) {
  return (uint16_t)strtol(cmd.substring(offset).c_str(), NULL, 16);
}

void handleCommand(const String &cmdIn) {
  String cmd = cmdIn;

  // ── Grabación / archivos (comandos nuevos para la app web) ──────────
  if (cmd.startsWith("RECORD_START:")) {
    String rest = cmd.substring(13);           // "<nombre>:<isoDate>"
    int sep = rest.indexOf(':');
    String nombre = (sep >= 0) ? rest.substring(0, sep) : rest;
    String iso    = (sep >= 0) ? rest.substring(sep + 1) : "";
    startRecording(nombre, iso);
    return;
  }
  if (cmd == "RECORD_STOP") { stopRecording(); return; }
  if (cmd == "LIST_FILES")  { listFiles();      return; }
  if (cmd == "SPACE")       { reportSpace();     return; }
  if (cmd.startsWith("DOWNLOAD_FILE:")) {
    sendFileOverBle(cmd.substring(14));
    return;
  }
  if (cmd.startsWith("DELETE_FILE:")) {
    String fn = cmd.substring(12);
    fn.trim();
    if (!fn.startsWith("/")) fn = "/" + fn;
    // Protege el archivo de configuración del sensor incluso si alguien
    // lo pide por nombre a mano (comando manual en Modo Dev) — no solo
    // se oculta de LIST_FILES, además el borrado se rechaza aquí.
    if (fn == CONFIG_FILE) {
      sendReply("ERR: no se puede borrar el archivo de configuracion del sistema");
      return;
    }
    bool ok = LittleFS.remove(fn);
    sendReply(ok ? ("ACK: DELETE_FILE:" + fn) : ("ERR: no se pudo borrar " + fn));
    return;
  }

  // ──── SENSOR 1 ────────────────────────────────────────
  if      (cmd.startsWith("IR_S1="))       { uint16_t v=parseHex(cmd,6);  applyLED(Wire,S1,true,v);  sendReply("ACK: IR_S1=0x"+String(v,HEX)); }
  else if (cmd.startsWith("Rojo_S1="))     { uint16_t v=parseHex(cmd,8);  applyLED(Wire,S1,false,v); sendReply("ACK: Rojo_S1=0x"+String(v,HEX)); }
  else if (cmd.startsWith("TIA_A_S1="))   { applyTIA(Wire,S1,true,false,parseHex(cmd,9));  sendReply("ACK: TIA_A_S1=0x"+String(S1.tia_a,HEX)); }
  else if (cmd.startsWith("TIA_B_S1="))   { applyTIA(Wire,S1,false,true,parseHex(cmd,9));  sendReply("ACK: TIA_B_S1=0x"+String(S1.tia_b,HEX)); }
  else if (cmd.startsWith("TIA_S1="))     { uint16_t v=parseHex(cmd,7);  applyTIA(Wire,S1,true,true,v);   sendReply("ACK: TIA_S1=0x"+String(v,HEX)); }
  else if (cmd.startsWith("PULSES_A_S1=")){ applyPulses(Wire,S1,true,false,parseHex(cmd,12)); sendReply("ACK: PULSES_A_S1=0x"+String(S1.pulses_a,HEX)); }
  else if (cmd.startsWith("PULSES_B_S1=")){ applyPulses(Wire,S1,false,true,parseHex(cmd,12)); sendReply("ACK: PULSES_B_S1=0x"+String(S1.pulses_b,HEX)); }
  else if (cmd.startsWith("PULSES_S1="))  { uint16_t v=parseHex(cmd,10); applyPulses(Wire,S1,true,true,v);  sendReply("ACK: PULSES_S1=0x"+String(v,HEX)); }
  else if (cmd.startsWith("FSAMPLE_S1=")) { applyFsample(Wire,S1,parseHex(cmd,11)); sendReply("ACK: FSAMPLE_S1=0x"+String(S1.fsample,HEX)); }
  else if (cmd.startsWith("AVG_S1="))     { applyAvg(Wire,S1,parseHex(cmd,7)); sendReply("ACK: AVG_S1"); }
  else if (cmd.startsWith("CHOP_S1="))    { applyChop(Wire,S1,cmd.charAt(8)=='1');  sendReply("ACK: CHOP_S1="+String(S1.chop)); }
  else if (cmd.startsWith("VBIAS_S1="))   { applyVbias(Wire,S1,cmd.charAt(9)=='1'); sendReply("ACK: VBIAS_S1="+String(S1.vbias)); }
  else if (cmd == "STATUS_S1")            { printStatus(Wire,S1,"S1"); }

  // ──── SENSOR 2 ────────────────────────────────────────
  else if (cmd.startsWith("IR_S2="))       { uint16_t v=parseHex(cmd,6);  applyLED(Wire1,S2,true,v);  sendReply("ACK: IR_S2=0x"+String(v,HEX)); }
  else if (cmd.startsWith("Rojo_S2="))     { uint16_t v=parseHex(cmd,8);  applyLED(Wire1,S2,false,v); sendReply("ACK: Rojo_S2=0x"+String(v,HEX)); }
  else if (cmd.startsWith("TIA_A_S2="))   { applyTIA(Wire1,S2,true,false,parseHex(cmd,9));  sendReply("ACK: TIA_A_S2=0x"+String(S2.tia_a,HEX)); }
  else if (cmd.startsWith("TIA_B_S2="))   { applyTIA(Wire1,S2,false,true,parseHex(cmd,9));  sendReply("ACK: TIA_B_S2=0x"+String(S2.tia_b,HEX)); }
  else if (cmd.startsWith("TIA_S2="))     { uint16_t v=parseHex(cmd,7);  applyTIA(Wire1,S2,true,true,v);   sendReply("ACK: TIA_S2=0x"+String(v,HEX)); }
  else if (cmd.startsWith("PULSES_A_S2=")){ applyPulses(Wire1,S2,true,false,parseHex(cmd,12)); sendReply("ACK: PULSES_A_S2=0x"+String(S2.pulses_a,HEX)); }
  else if (cmd.startsWith("PULSES_B_S2=")){ applyPulses(Wire1,S2,false,true,parseHex(cmd,12)); sendReply("ACK: PULSES_B_S2=0x"+String(S2.pulses_b,HEX)); }
  else if (cmd.startsWith("PULSES_S2="))  { uint16_t v=parseHex(cmd,10); applyPulses(Wire1,S2,true,true,v);  sendReply("ACK: PULSES_S2=0x"+String(v,HEX)); }
  else if (cmd.startsWith("FSAMPLE_S2=")) { applyFsample(Wire1,S2,parseHex(cmd,11)); sendReply("ACK: FSAMPLE_S2=0x"+String(S2.fsample,HEX)); }
  else if (cmd.startsWith("AVG_S2="))     { applyAvg(Wire1,S2,parseHex(cmd,7)); sendReply("ACK: AVG_S2"); }
  else if (cmd.startsWith("CHOP_S2="))    { applyChop(Wire1,S2,cmd.charAt(8)=='1');  sendReply("ACK: CHOP_S2="+String(S2.chop)); }
  else if (cmd.startsWith("VBIAS_S2="))   { applyVbias(Wire1,S2,cmd.charAt(9)=='1'); sendReply("ACK: VBIAS_S2="+String(S2.vbias)); }
  else if (cmd == "STATUS_S2")            { printStatus(Wire1,S2,"S2"); }

  // ──── GLOBALES ────────────────────────────────────────
  else if (cmd.startsWith("IR="))     { uint16_t v=parseHex(cmd,3);  applyLED(Wire,S1,true,v);   applyLED(Wire1,S2,true,v);   sendReply("ACK: IR=0x"+String(v,HEX)); }
  else if (cmd.startsWith("Rojo="))   { uint16_t v=parseHex(cmd,5);  applyLED(Wire,S1,false,v);  applyLED(Wire1,S2,false,v);  sendReply("ACK: Rojo=0x"+String(v,HEX)); }
  else if (cmd.startsWith("TIA="))    { uint16_t v=parseHex(cmd,4);  applyTIA(Wire,S1,true,true,v); applyTIA(Wire1,S2,true,true,v); sendReply("ACK: TIA=0x"+String(v,HEX)); }
  else if (cmd.startsWith("PULSES=")) { uint16_t v=parseHex(cmd,7);  applyPulses(Wire,S1,true,true,v); applyPulses(Wire1,S2,true,true,v); sendReply("ACK: PULSES=0x"+String(v,HEX)); }
  else if (cmd.startsWith("FSAMPLE=")){ uint16_t v=parseHex(cmd,8);  applyFsample(Wire,S1,v); applyFsample(Wire1,S2,v); sendReply("ACK: FSAMPLE=0x"+String(v,HEX)); }
  else if (cmd.startsWith("AVG="))    { uint16_t v=parseHex(cmd,4);  applyAvg(Wire,S1,v);  applyAvg(Wire1,S2,v);  sendReply("ACK: AVG global"); }
  else if (cmd == "STATUS")           { printStatus(Wire,S1,"S1"); printStatus(Wire1,S2,"S2"); }
  else if (cmd.length() > 0)          { sendReply("ERR: Comando desconocido: " + cmd); }
}

// =========================================================
// BLE — servidor GATT (Nordic UART Service) + parser de línea entrante
// Reemplaza por completo a WiFi+MQTT+servidor web: ya no hace falta
// ninguna red, ni credenciales, ni certificados — todo es local.
// =========================================================
String bleRxBuffer = "";

class NirsServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* srv) override {
    bleConnected = true;
    Serial.println("BLE: app conectada");
    // Solo arma la máquina de estados; el parpadeo real ocurre en loop()
    // vía handleConnectBlink() — nada de I2C ni delay() aquí (este
    // callback corre en el hilo del stack BLE, no en el loop principal).
    g_blinkActive       = true;
    g_blinkToggleCount  = 0;
    g_blinkLedOn        = false;
    g_lastBlinkToggleMs = millis();
  }
  void onDisconnect(BLEServer* srv) override {
    bleConnected = false;
    Serial.println("BLE: app desconectada — reanudando advertising...");
    delay(200);
    srv->getAdvertising()->start();
  }
};

class NirsRxCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* chr) override {
    String raw = chr->getValue();
    for (size_t i = 0; i < raw.length(); i++) {
      char c = raw[i];
      if (c == '\n' || c == '\r') {
        if (bleRxBuffer.length() > 0) {
          bleRxBuffer.trim();
          handleCommand(bleRxBuffer);
          bleRxBuffer = "";
        }
      } else {
        bleRxBuffer += c;
        if (bleRxBuffer.length() > 256) bleRxBuffer = ""; // seguridad ante basura/overflow
      }
    }
  }
};

void setupBLE() {
  BLEDevice::init(BLE_DEVICE_NAME);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new NirsServerCallbacks());

  BLEService* service = bleServer->createService(NUS_SERVICE_UUID);

  bleTxChar = service->createCharacteristic(
      NUS_CHAR_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  bleTxChar->addDescriptor(new BLE2902());

  bleRxChar = service->createCharacteristic(
      NUS_CHAR_RX_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  bleRxChar->setCallbacks(new NirsRxCallbacks());

  service->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.print("BLE listo — nombre visible: ");
  Serial.println(BLE_DEVICE_NAME);
}

// =========================================================
// SETUP
// =========================================================
void setup() {
  Serial.begin(115200);
  delay(100);

  // ── LittleFS ──────────────────────────────────────────
  if (!LittleFS.begin(true)) {   // true = formatear si falla el montaje
    Serial.println("ERROR: no se pudo montar LittleFS.");
  } else {
    Serial.printf("LittleFS OK — usados %u / %u bytes\n",
                   (unsigned)LittleFS.usedBytes(), (unsigned)LittleFS.totalBytes());
  }

  // ── Configuración guardada — S1/S2 empiezan con sus valores de
  // fábrica (arriba); si hay una config guardada por el Modo
  // Desarrollador, la sobreescribe ANTES de initSensor() para que
  // el ESP32 arranque con la última configuración usada.
  if (loadSensorConfig()) {
    Serial.println("Config cargada desde flash (ultima configuracion guardada).");
  } else {
    Serial.println("Sin config guardada en flash — usando valores de fabrica.");
  }

  // ── BLE ────────────────────────────────────────────────
  setupBLE();

  // ── I2C sensores ───────────────────────────────────────
  Wire.begin(I2C0_SDA, I2C0_SCL);   Wire.setClock(400000);
  Wire1.begin(I2C1_SDA, I2C1_SCL);  Wire1.setClock(400000);
  delay(100);

  bool ok1 = initSensor(Wire,  S1, "Sensor1");
  bool ok2 = initSensor(Wire1, S2, "Sensor2");
  if (!ok1 && !ok2) { Serial.println("ERROR FATAL: Ningun sensor."); while(1); }

  // initSensor() no toca REG_PD_BIAS — si VBias estaba guardado activo,
  // restaurarlo aquí con la misma escritura que usa applyVbias().
  if (ok1 && S1.vbias) {
    writeReg(Wire, REG_MODE, MODE_PROGRAM);
    writeReg(Wire, REG_PD_BIAS, 0x0A80);
    writeReg(Wire, REG_MODE, MODE_NORMAL);
  }
  if (ok2 && S2.vbias) {
    writeReg(Wire1, REG_MODE, MODE_PROGRAM);
    writeReg(Wire1, REG_PD_BIAS, 0x0A80);
    writeReg(Wire1, REG_MODE, MODE_NORMAL);
  }

  recalcLoopPeriod();
  Serial.print("ADPD1080 Dual v12 (Bluetooth BLE, sin WiFi/nube) listo. Loop period=");
  Serial.print(g_loop_period_us); Serial.println(" us");
  delay(200);
}

// =========================================================
// PARPADEO DE AVISO AL CONECTAR BLE (LED rojo, ambos sensores)
// Máquina de estados no bloqueante basada en millis() — se arma en
// NirsServerCallbacks::onConnect() y se ejecuta aquí, en el loop
// principal, para no bloquear el hilo del stack BLE con delay().
// Al terminar los 5 parpadeos, restaura el LED rojo a su estado real
// (S1.led_dis / S2.led_dis), que arranca apagado por defecto.
// =========================================================
void handleConnectBlink() {
  if (!g_blinkActive) return;
  uint32_t now = millis();
  if (now - g_lastBlinkToggleMs < BLINK_TOGGLE_MS) return;
  g_lastBlinkToggleMs = now;
  g_blinkLedOn = !g_blinkLedOn;

  if (g_blinkLedOn) {
    writeReg(Wire,  REG_ILED2_COARSE, S1.led2 ? S1.led2 : 0x3008);
    writeReg(Wire1, REG_ILED2_COARSE, S2.led2 ? S2.led2 : 0x3008);
    writeReg(Wire,  REG_LED_DISABLE, S1.led_dis & ~(1 << 9));
    writeReg(Wire1, REG_LED_DISABLE, S2.led_dis & ~(1 << 9));
  } else {
    writeReg(Wire,  REG_LED_DISABLE, S1.led_dis | (1 << 9));
    writeReg(Wire1, REG_LED_DISABLE, S2.led_dis | (1 << 9));
  }

  g_blinkToggleCount++;
  if (g_blinkToggleCount >= BLINK_TOTAL_TOGGLES) {
    g_blinkActive = false;
    // Restaurar el estado real de encendido/apagado del LED rojo
    // (por defecto apagado desde initSensor(), salvo que el usuario ya
    // lo hubiera encendido manualmente antes de esta reconexión).
    writeReg(Wire,  REG_LED_DISABLE, S1.led_dis);
    writeReg(Wire1, REG_LED_DISABLE, S2.led_dis);
  }
}

// =========================================================
// LOOP
// =========================================================
void loop() {
  uint32_t t_start = micros();

  // Comandos desde USB Serial (debug)
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    handleCommand(cmd);
  }
  // (Los comandos por BLE llegan de forma asíncrona vía NirsRxCallbacks::onWrite)

  handleConnectBlink();

  if (bleConnected || g_recording) {
    uint16_t ir1  = readReg(Wire,  REG_SLOTA_CH1);
    uint16_t red1 = readReg(Wire,  REG_SLOTB_CH1);
    uint16_t ir2  = readReg(Wire1, REG_SLOTA_CH1);
    uint16_t red2 = readReg(Wire1, REG_SLOTB_CH1);

    if (bleConnected) {
      uint32_t now = millis();
      if (now - g_last_ble_broadcast_ms >= BLE_BROADCAST_INTERVAL_MS) {
        g_last_ble_broadcast_ms = now;
        String line = String(ir1) + "," + String(red1) + "," + String(ir2) + "," + String(red2);
        bleSendLine(line);
      }
    }

    if (g_recording) recordSample(ir1, red1, ir2, red2);
  }

  uint32_t elapsed = micros() - t_start;
  if (elapsed < g_loop_period_us) {
    delayMicroseconds(g_loop_period_us - elapsed);
  }
}
