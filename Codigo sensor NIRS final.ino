/*
 * ADPD1080 — Dual Sensor NIRS/PPG (ESP32) v6 — Bluetooth SPP
 * =============================================================
 * Sensor 1: Wire  (SDA=21, SCL=22)  → 0x64
 * Sensor 2: Wire1 (SDA=18, SCL=19)  → 0x64
 *
 * COMUNICACIÓN:
 *   - Datos y comandos via Bluetooth SPP (BluetoothSerial)
 *   - Nombre BT: "NIRS_ESP32"  (visible al parear desde PC/Android)
 *   - USB Serial (115200) solo para debug — NO envía datos
 *   - En Windows: parear en Configuración → BT → aparece como
 *     "NIRS_ESP32", se asigna un puerto COMxx automáticamente.
 *     Seleccionar ese COMxx en el GUI Python.
 *
 * Salida BT: "IR1,Rojo1,IR2,Rojo2\n"
 *
 * CAMBIOS vs v5:
 *   - Reemplaza Serial (datos) por SerialBT (BluetoothSerial)
 *   - Serial USB queda solo para debug/STATUS en monitor serie
 *   - BT_connected() verifica conexión antes de enviar datos
 *   - Comandos se leen de SerialBT (además de Serial para debug)
 *
 * COMANDOS (sufijo _S1 o _S2):
 *   IR_S1=0xXXXX / Rojo_S1=0xXXXX
 *   TIA_A_S1=0xXXXX / TIA_B_S1=0xXXXX / TIA_S1=0xXXXX
 *   PULSES_A_S1=0xXXXX / PULSES_B_S1=0xXXXX / PULSES_S1=0xXXXX
 *   FSAMPLE_S1=0xXXXX / AVG_S1=0xXXXX
 *   CHOP_S1=0/1 / VBIAS_S1=0/1 / STATUS_S1
 *   (ídem _S2)
 *
 * COMANDOS GLOBALES:
 *   IR= / Rojo= / TIA= / PULSES= / FSAMPLE= / AVG= / STATUS
 */

#include <Wire.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

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
// Chop Mode — registros correctos ADPD1080 (datasheet Rev C, Table 25)
const uint8_t REG_INTEG_ORDER_A = 0x17;   // bits[3:0] = patrón inversión slot A
const uint8_t REG_INTEG_ORDER_B = 0x1D;   // bits[3:0] = patrón inversión slot B
const uint8_t REG_MATH          = 0x58;   // matemática suma/resta pulsos
// Offsets ADC slot A (deben ser 0x2000 con chop activo)
const uint8_t REG_SLOTA_CH1_OFF = 0x18;
const uint8_t REG_SLOTA_CH2_OFF = 0x19;
const uint8_t REG_SLOTA_CH3_OFF = 0x1A;
const uint8_t REG_SLOTA_CH4_OFF = 0x1B;
// Offsets ADC slot B
const uint8_t REG_SLOTB_CH1_OFF = 0x1E;
const uint8_t REG_SLOTB_CH2_OFF = 0x1F;
const uint8_t REG_SLOTB_CH3_OFF = 0x20;
const uint8_t REG_SLOTB_CH4_OFF = 0x21;
// VBias — registro correcto ADPD1080
const uint8_t REG_PD_BIAS       = 0x54;   // bit7=enable, bits[9:8]=slotA ~250mV, bits[11:10]=slotB ~250mV

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
  uint16_t fsample;    // ← NUEVO: valor actual del registro FSAMPLE
  uint16_t avg;        // ← NUEVO: valor actual del registro NUM_AVG
  bool     chop;
  bool     vbias;
};

// Configuración inicial: 1000 Hz, sin promediado
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

// -------------------------------------------------------
// Período de loop — se recalcula cuando cambia FSAMPLE
// Usa el sensor más lento (mayor período) para no perder muestras
// -------------------------------------------------------
uint32_t g_loop_period_us = 2000;  // µs — default 500 Hz (conservador al inicio)

// -------------------------------------------------------
// Helpers BT — envían datos solo si hay cliente conectado
// -------------------------------------------------------
#define DBG(x)   Serial.print(x)
#define DBGLN(x) Serial.println(x)

void btPrint(const char* s)   { if (SerialBT.hasClient()) SerialBT.print(s); }
void btPrintln(const char* s) { if (SerialBT.hasClient()) SerialBT.println(s); }
void btPrint(uint16_t v)      { if (SerialBT.hasClient()) SerialBT.print(v); }
void btPrintln(uint16_t v)    { if (SerialBT.hasClient()) SerialBT.println(v); }
void btPrint(uint32_t v)      { if (SerialBT.hasClient()) SerialBT.print(v); }
void btPrintln(uint32_t v)    { if (SerialBT.hasClient()) SerialBT.println(v); }
void btPrint(int v)           { if (SerialBT.hasClient()) SerialBT.print(v); }
void btPrintln(int v)         { if (SerialBT.hasClient()) SerialBT.println(v); }
void btPrintHex(uint16_t v)   { if (SerialBT.hasClient()) SerialBT.print(v, HEX); }
void btPrintlnHex(uint16_t v) { if (SerialBT.hasClient()) SerialBT.println(v, HEX); }

// Calcula el período mínimo real del sensor en µs considerando
// pulsos configurados (fMAX real) y factor de promediado.
// fMAX = 1e6 / (25 + Pa*24 + 68 + 25 + Pb*24 + 20 + 222)  µs
// Si fSAMPLE < fMAX, el período real es fSAMPLE, si no es fMAX.
uint32_t sensorPeriodUs(SensorState &s) {
  uint8_t pa = (s.pulses_a >> 8) & 0xFF;
  uint8_t pb = (s.pulses_b >> 8) & 0xFF;
  if (pa < 1) pa = 1;
  if (pb < 1) pb = 1;
  uint32_t fmax_us = 25UL + pa*24UL + 68UL + 25UL + pb*24UL + 20UL + 222UL;
  uint32_t fsample_us = (uint32_t)s.fsample * 125UL;
  // Período base = el mayor de los dos (sensor no puede ir más rápido que fMAX)
  uint32_t base_us = max(fmax_us, fsample_us);
  // Con promediado Nx, el sensor actualiza cada N períodos base
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

// =======================================================
// I2C HELPERS
// =======================================================
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


// =======================================================
// INICIALIZACIÓN
// =======================================================
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
  writeReg(bus, REG_LED_DISABLE,  s.led_dis);
  writeReg(bus, REG_SLOT_EN,      ENABLE_BOTH_SLOTS);
  writeReg(bus, REG_MODE,         MODE_NORMAL);
  Serial.print("OK: "); Serial.print(name); Serial.println(" inicializado.");
  return true;
}

// =======================================================
// COMANDOS
// =======================================================
void applyLED(TwoWire &bus, SensorState &s, bool isIR, uint16_t val) {
  if (isIR) {
    if (val == 0) { s.led_dis |=  (1 << 8); }
    else          { s.led_dis &= ~(1 << 8); s.led1 = val; writeReg(bus, REG_ILED1_COARSE, val); }
  } else {
    if (val == 0) { s.led_dis |=  (1 << 9); }
    else          { s.led_dis &= ~(1 << 9); s.led2 = val; writeReg(bus, REG_ILED2_COARSE, val); }
  }
  writeReg(bus, REG_LED_DISABLE, s.led_dis);
}

void applyTIA(TwoWire &bus, SensorState &s, bool slotA, bool slotB, uint16_t val) {
  writeReg(bus, REG_MODE, MODE_PROGRAM);
  if (slotA) { s.tia_a = val; writeReg(bus, REG_SLOTA_TIA, val); }
  if (slotB) { s.tia_b = val; writeReg(bus, REG_SLOTB_TIA, val); }
  writeReg(bus, REG_MODE, MODE_NORMAL);
}

void applyPulses(TwoWire &bus, SensorState &s, bool slotA, bool slotB, uint16_t val) {
  writeReg(bus, REG_MODE, MODE_PROGRAM);
  if (slotA) { s.pulses_a = val; writeReg(bus, REG_SLOTA_PULSES, val); }
  if (slotB) { s.pulses_b = val; writeReg(bus, REG_SLOTB_PULSES, val); }
  writeReg(bus, REG_MODE, MODE_NORMAL);
}

// ── BUG FIX: entra en PROGRAM antes de escribir FSAMPLE ──
void applyFsample(TwoWire &bus, SensorState &s, uint16_t val) {
  if (val < 1) val = 1;
  s.fsample = val;                        // guardar en estado
  writeReg(bus, REG_MODE,    MODE_PROGRAM);
  writeReg(bus, REG_FSAMPLE, val);
  writeReg(bus, REG_MODE,    MODE_NORMAL);
  recalcLoopPeriod();                     // actualizar período del loop
  uint32_t hz = 32000UL / ((uint32_t)val * 4);
  DBG("  FSAMPLE aplicado: 0x"); Serial.print(val, HEX);
  DBG(" = "); Serial.print(hz); DBGLN(" Hz");
}

void applyAvg(TwoWire &bus, SensorState &s, uint16_t val) {
  s.avg = val;
  writeReg(bus, REG_MODE,    MODE_PROGRAM);
  writeReg(bus, REG_NUM_AVG, val);
  writeReg(bus, REG_MODE,    MODE_NORMAL);
  recalcLoopPeriod();   // actualizar período considerando el nuevo promediado
}

void applyChop(TwoWire &bus, SensorState &s, bool en) {
  // Chop Mode requiere configuración compleja de 4+ registros (0x17,0x1D,0x58).
  // Implementaciones incorrectas causan señal en 0. Función deshabilitada.
  s.chop = false;
}

void applyVbias(TwoWire &bus, SensorState &s, bool en) {
  // VBias correcto: registro 0x54
  // bit7=1 habilita, bits[9:8]=0x2 slot A ~250mV, bits[11:10]=0x2 slot B ~250mV → 0x0A80
  s.vbias = en;
  writeReg(bus, REG_MODE,    MODE_PROGRAM);
  writeReg(bus, REG_PD_BIAS, en ? 0x0A80 : 0x0000);
  writeReg(bus, REG_MODE,    MODE_NORMAL);
}

// Envía STATUS tanto por BT como por USB Serial
void printStatus(TwoWire &bus, SensorState &s, const char* name) {
  uint32_t hz = 32000UL / ((uint32_t)readReg(bus, REG_FSAMPLE) * 4);
  // USB debug
  Serial.print("--- STATUS "); Serial.print(name); Serial.println(" ---");
  Serial.print("MODE=0x");     Serial.println(readReg(bus, REG_MODE), HEX);
  Serial.print("FSAMPLE=0x");  Serial.print(readReg(bus, REG_FSAMPLE), HEX);
  Serial.print(" ("); Serial.print(hz); Serial.println(" Hz)");
  Serial.print("NUM_AVG=0x");  Serial.println(readReg(bus, REG_NUM_AVG), HEX);
  Serial.print("PULSES_A=0x"); Serial.println(readReg(bus, REG_SLOTA_PULSES), HEX);
  Serial.print("PULSES_B=0x"); Serial.println(readReg(bus, REG_SLOTB_PULSES), HEX);
  Serial.print("Loop period="); Serial.print(g_loop_period_us); Serial.println(" us");
  Serial.println("--- FIN ---");
  // BT (para el GUI Python)
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
  btPrint("Loop period="); btPrint(g_loop_period_us); btPrintln(" us");
  btPrintln("--- FIN ---");
}

// =======================================================
// PARSER DE COMANDOS
// =======================================================
uint16_t parseHex(const String &cmd, int offset) {
  return (uint16_t)strtol(cmd.substring(offset).c_str(), NULL, 16);
}

void handleCommand(const String &cmd) {

  // ──── SENSOR 1 ────────────────────────────────────────
  if      (cmd.startsWith("IR_S1="))       { uint16_t v=parseHex(cmd,6);  applyLED(Wire,S1,true,v);  if(SerialBT.hasClient()) SerialBT.print("ACK: IR_S1=0x");    if(SerialBT.hasClient()) SerialBT.println(v,HEX); }
  else if (cmd.startsWith("Rojo_S1="))     { uint16_t v=parseHex(cmd,8);  applyLED(Wire,S1,false,v); if(SerialBT.hasClient()) SerialBT.print("ACK: Rojo_S1=0x"); if(SerialBT.hasClient()) SerialBT.println(v,HEX); }
  else if (cmd.startsWith("TIA_A_S1="))   { applyTIA(Wire,S1,true,false,parseHex(cmd,9));  if(SerialBT.hasClient()) SerialBT.print("ACK: TIA_A_S1=0x");  if(SerialBT.hasClient()) SerialBT.println(S1.tia_a,HEX); }
  else if (cmd.startsWith("TIA_B_S1="))   { applyTIA(Wire,S1,false,true,parseHex(cmd,9));  if(SerialBT.hasClient()) SerialBT.print("ACK: TIA_B_S1=0x");  if(SerialBT.hasClient()) SerialBT.println(S1.tia_b,HEX); }
  else if (cmd.startsWith("TIA_S1="))     { uint16_t v=parseHex(cmd,7);  applyTIA(Wire,S1,true,true,v);   if(SerialBT.hasClient()) SerialBT.print("ACK: TIA_S1=0x");   if(SerialBT.hasClient()) SerialBT.println(v,HEX); }
  else if (cmd.startsWith("PULSES_A_S1=")){ applyPulses(Wire,S1,true,false,parseHex(cmd,12)); if(SerialBT.hasClient()) SerialBT.print("ACK: PULSES_A_S1=0x"); if(SerialBT.hasClient()) SerialBT.println(S1.pulses_a,HEX); }
  else if (cmd.startsWith("PULSES_B_S1=")){ applyPulses(Wire,S1,false,true,parseHex(cmd,12)); if(SerialBT.hasClient()) SerialBT.print("ACK: PULSES_B_S1=0x"); if(SerialBT.hasClient()) SerialBT.println(S1.pulses_b,HEX); }
  else if (cmd.startsWith("PULSES_S1="))  { uint16_t v=parseHex(cmd,10); applyPulses(Wire,S1,true,true,v);  if(SerialBT.hasClient()) SerialBT.print("ACK: PULSES_S1=0x");  if(SerialBT.hasClient()) SerialBT.println(v,HEX); }
  else if (cmd.startsWith("FSAMPLE_S1=")) { applyFsample(Wire,S1,parseHex(cmd,11)); if(SerialBT.hasClient()) SerialBT.print("ACK: FSAMPLE_S1=0x"); if(SerialBT.hasClient()) SerialBT.println(S1.fsample,HEX); }
  else if (cmd.startsWith("AVG_S1="))     { applyAvg(Wire,S1,parseHex(cmd,7)); if(SerialBT.hasClient()) SerialBT.println("ACK: AVG_S1"); }
  else if (cmd.startsWith("CHOP_S1="))    { applyChop(Wire,S1,cmd.charAt(8)=='1');  if(SerialBT.hasClient()) SerialBT.print("ACK: CHOP_S1=");  if(SerialBT.hasClient()) SerialBT.println(S1.chop); }
  else if (cmd.startsWith("VBIAS_S1="))   { applyVbias(Wire,S1,cmd.charAt(9)=='1'); if(SerialBT.hasClient()) SerialBT.print("ACK: VBIAS_S1="); if(SerialBT.hasClient()) SerialBT.println(S1.vbias); }
  else if (cmd == "STATUS_S1")            { printStatus(Wire,S1,"S1"); }

  // ──── SENSOR 2 ────────────────────────────────────────
  else if (cmd.startsWith("IR_S2="))       { uint16_t v=parseHex(cmd,6);  applyLED(Wire1,S2,true,v);  if(SerialBT.hasClient()) SerialBT.print("ACK: IR_S2=0x");    if(SerialBT.hasClient()) SerialBT.println(v,HEX); }
  else if (cmd.startsWith("Rojo_S2="))     { uint16_t v=parseHex(cmd,8);  applyLED(Wire1,S2,false,v); if(SerialBT.hasClient()) SerialBT.print("ACK: Rojo_S2=0x"); if(SerialBT.hasClient()) SerialBT.println(v,HEX); }
  else if (cmd.startsWith("TIA_A_S2="))   { applyTIA(Wire1,S2,true,false,parseHex(cmd,9));  if(SerialBT.hasClient()) SerialBT.print("ACK: TIA_A_S2=0x");  if(SerialBT.hasClient()) SerialBT.println(S2.tia_a,HEX); }
  else if (cmd.startsWith("TIA_B_S2="))   { applyTIA(Wire1,S2,false,true,parseHex(cmd,9));  if(SerialBT.hasClient()) SerialBT.print("ACK: TIA_B_S2=0x");  if(SerialBT.hasClient()) SerialBT.println(S2.tia_b,HEX); }
  else if (cmd.startsWith("TIA_S2="))     { uint16_t v=parseHex(cmd,7);  applyTIA(Wire1,S2,true,true,v);   if(SerialBT.hasClient()) SerialBT.print("ACK: TIA_S2=0x");   if(SerialBT.hasClient()) SerialBT.println(v,HEX); }
  else if (cmd.startsWith("PULSES_A_S2=")){ applyPulses(Wire1,S2,true,false,parseHex(cmd,12)); if(SerialBT.hasClient()) SerialBT.print("ACK: PULSES_A_S2=0x"); if(SerialBT.hasClient()) SerialBT.println(S2.pulses_a,HEX); }
  else if (cmd.startsWith("PULSES_B_S2=")){ applyPulses(Wire1,S2,false,true,parseHex(cmd,12)); if(SerialBT.hasClient()) SerialBT.print("ACK: PULSES_B_S2=0x"); if(SerialBT.hasClient()) SerialBT.println(S2.pulses_b,HEX); }
  else if (cmd.startsWith("PULSES_S2="))  { uint16_t v=parseHex(cmd,10); applyPulses(Wire1,S2,true,true,v);  if(SerialBT.hasClient()) SerialBT.print("ACK: PULSES_S2=0x");  if(SerialBT.hasClient()) SerialBT.println(v,HEX); }
  else if (cmd.startsWith("FSAMPLE_S2=")) { applyFsample(Wire1,S2,parseHex(cmd,11)); if(SerialBT.hasClient()) SerialBT.print("ACK: FSAMPLE_S2=0x"); if(SerialBT.hasClient()) SerialBT.println(S2.fsample,HEX); }
  else if (cmd.startsWith("AVG_S2="))     { applyAvg(Wire1,S2,parseHex(cmd,7)); if(SerialBT.hasClient()) SerialBT.println("ACK: AVG_S2"); }
  else if (cmd.startsWith("CHOP_S2="))    { applyChop(Wire1,S2,cmd.charAt(8)=='1');  if(SerialBT.hasClient()) SerialBT.print("ACK: CHOP_S2=");  if(SerialBT.hasClient()) SerialBT.println(S2.chop); }
  else if (cmd.startsWith("VBIAS_S2="))   { applyVbias(Wire1,S2,cmd.charAt(9)=='1'); if(SerialBT.hasClient()) SerialBT.print("ACK: VBIAS_S2="); if(SerialBT.hasClient()) SerialBT.println(S2.vbias); }
  else if (cmd == "STATUS_S2")            { printStatus(Wire1,S2,"S2"); }

  // ──── GLOBALES ────────────────────────────────────────
  else if (cmd.startsWith("IR="))     { uint16_t v=parseHex(cmd,3);  applyLED(Wire,S1,true,v);   applyLED(Wire1,S2,true,v);   if(SerialBT.hasClient()) SerialBT.print("ACK: IR=0x");     if(SerialBT.hasClient()) SerialBT.println(v,HEX); }
  else if (cmd.startsWith("Rojo="))   { uint16_t v=parseHex(cmd,5);  applyLED(Wire,S1,false,v);  applyLED(Wire1,S2,false,v);  if(SerialBT.hasClient()) SerialBT.print("ACK: Rojo=0x");   if(SerialBT.hasClient()) SerialBT.println(v,HEX); }
  else if (cmd.startsWith("TIA="))    { uint16_t v=parseHex(cmd,4);  applyTIA(Wire,S1,true,true,v); applyTIA(Wire1,S2,true,true,v); if(SerialBT.hasClient()) SerialBT.print("ACK: TIA=0x"); if(SerialBT.hasClient()) SerialBT.println(v,HEX); }
  else if (cmd.startsWith("PULSES=")) { uint16_t v=parseHex(cmd,7);  applyPulses(Wire,S1,true,true,v); applyPulses(Wire1,S2,true,true,v); if(SerialBT.hasClient()) SerialBT.print("ACK: PULSES=0x"); if(SerialBT.hasClient()) SerialBT.println(v,HEX); }
  else if (cmd.startsWith("FSAMPLE=")){ uint16_t v=parseHex(cmd,8);  applyFsample(Wire,S1,v); applyFsample(Wire1,S2,v); if(SerialBT.hasClient()) SerialBT.print("ACK: FSAMPLE=0x"); if(SerialBT.hasClient()) SerialBT.println(v,HEX); }
  else if (cmd.startsWith("AVG="))    { uint16_t v=parseHex(cmd,4);  applyAvg(Wire,S1,v);  applyAvg(Wire1,S2,v);  if(SerialBT.hasClient()) SerialBT.println("ACK: AVG global"); }
  else if (cmd == "STATUS")           { printStatus(Wire,S1,"S1"); printStatus(Wire1,S2,"S2"); }
  else if (cmd.length() > 0)          { if(SerialBT.hasClient()) SerialBT.print("ERR: Comando desconocido: "); if(SerialBT.hasClient()) SerialBT.println(cmd); }
}

// =======================================================
// SETUP
// =======================================================
void setup() {
  Serial.begin(115200);    // USB — solo debug
  delay(100);

  // ── Bluetooth SPP ──────────────────────────────────
  // El nombre "NIRS_ESP32" aparece al buscar dispositivos BT en Windows.
  // Después de parear, Windows asigna un COMxx — usarlo en el GUI Python.
  SerialBT.begin("SENSOR_NIRS");
  Serial.println("BT iniciado como SENSOR_NIRS — esperando conexion...");

  Wire.begin(I2C0_SDA, I2C0_SCL);   Wire.setClock(400000);
  Wire1.begin(I2C1_SDA, I2C1_SCL);  Wire1.setClock(400000);
  delay(100);

  bool ok1 = initSensor(Wire,  S1, "Sensor1");
  bool ok2 = initSensor(Wire1, S2, "Sensor2");
  if (!ok1 && !ok2) { Serial.println("ERROR FATAL: Ningun sensor."); while(1); }

  recalcLoopPeriod();
  Serial.print("ADPD1080 Dual v6-BT listo. Loop period=");
  Serial.print(g_loop_period_us); Serial.println(" us");
  delay(200);
}

// =======================================================
// LOOP — respeta el período configurado del sensor
// =======================================================
void loop() {
  uint32_t t_start = micros();

  // Procesar comandos desde BT (GUI Python)
  if (SerialBT.available() > 0) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();
    handleCommand(cmd);
  }

  // Procesar comandos desde USB Serial (debug / Monitor Serie Arduino)
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    handleCommand(cmd);
  }

  // Enviar datos solo si hay cliente BT conectado
  if (SerialBT.hasClient()) {
    uint16_t ir1  = readReg(Wire,  REG_SLOTA_CH1);
    uint16_t red1 = readReg(Wire,  REG_SLOTB_CH1);
    uint16_t ir2  = readReg(Wire1, REG_SLOTA_CH1);
    uint16_t red2 = readReg(Wire1, REG_SLOTB_CH1);

    SerialBT.print(ir1);  SerialBT.print(",");
    SerialBT.print(red1); SerialBT.print(",");
    SerialBT.print(ir2);  SerialBT.print(",");
    SerialBT.println(red2);
  }

  // Esperar el tiempo restante del período
  uint32_t elapsed = micros() - t_start;
  if (elapsed < g_loop_period_us) {
    delayMicroseconds(g_loop_period_us - elapsed);
  }
}