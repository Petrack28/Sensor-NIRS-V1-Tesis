#include <AsyncUDP.h>

/*
 * ADPD1080 — Dual Sensor NIRS/PPG (ESP32) v8 — WiFi+MQTT (nube) + Bluetooth SPP
 * =================================================================================
 * Sensor 1: Wire  (SDA=21, SCL=22)  → 0x64
 * Sensor 2: Wire1 (SDA=18, SCL=19)  → 0x64
 *
 * COMUNICACIÓN (misma arquitectura que Estimulador_CICATA_UNAM_GOOD):
 *   1) WiFi normal (STA) — el ESP32 se conecta a tu red WiFi con internet
 *      (la de casa/laboratorio) usando WiFiManager: si no hay credenciales
 *      guardadas, abre un portal cautivo "NIRS_Setup" — te conectas a esa
 *      red temporal desde el celular, eliges tu WiFi real e ingresas la
 *      contraseña. Se guarda en flash, no hay que reflashear si cambias
 *      de red.
 *   2) MQTT en la nube (HiveMQ Cloud, TLS) — control y datos en vivo desde
 *      CUALQUIER lugar con internet (el teléfono no necesita estar en la
 *      misma red que el ESP32). Tópicos:
 *        nirs/cmd     (teléfono → ESP32)  comandos, protocolo idéntico a v7
 *        nirs/data    (ESP32 → teléfono)  "IR1,Rojo1,IR2,Rojo2" (10 Hz)
 *        nirs/status  (ESP32 → teléfono)  ACKs, STATUS, ONLINE/OFFLINE (LWT)
 *   3) Servidor web LOCAL (HTTP, ESPAsyncWebServer) — sigue activo en la IP
 *      local del ESP32 (o http://nirs32.local/ vía mDNS). Sirve la misma
 *      app web y, sobre todo, la DESCARGA de archivos CSV/bin: eso requiere
 *      estar en la misma red que el ESP32 (típico: estás en el laboratorio
 *      terminando una grabación).
 *   4) Bluetooth SPP — se mantiene igual que en v6/v7, respaldo para PC.
 *   5) USB Serial (115200) — solo debug.
 *
 * Protocolo de comandos (idéntico a v6/v7, ahora también por MQTT):
 *   IR_S1=0xXXXX / Rojo_S1=0xXXXX
 *   TIA_A_S1=0xXXXX / TIA_B_S1=0xXXXX / TIA_S1=0xXXXX
 *   PULSES_A_S1=0xXXXX / PULSES_B_S1=0xXXXX / PULSES_S1=0xXXXX
 *   FSAMPLE_S1=0xXXXX / AVG_S1=0xXXXX
 *   CHOP_S1=0/1 / VBIAS_S1=0/1 / STATUS_S1
 *   (ídem _S2, y globales IR= / Rojo= / TIA= / PULSES= / FSAMPLE= / AVG= / STATUS)
 *   RECORD_START:<nombre>:<isoDateTime> / RECORD_STOP / LIST_FILES / DELETE_FILE:<n>
 *   WIFI_RESET   → borra credenciales WiFi guardadas y reinicia en modo portal
 *                  (útil si cambias de red o de router)
 *
 * Formato de grabación en el ESP32 — BINARIO COMPACTO (8 bytes/muestra):
 *   uint32 t_ms (little-endian) + uint16 IR_ADC (LE) + uint16 Rojo_ADC (LE)
 *   Conviértelo a tu CSV final (tiempo_s,IR_ADC,IR_A,Rojo_ADC,Rojo_A) con
 *   "bin_to_csv_nirs.py" después de bajar el .bin + .json desde la app web.
 *
 * DESCARGA DESDE EL TELÉFONO (requiere estar en la misma red que el ESP32):
 *   /download?f=<nombre>  → el navegador del teléfono lo baja a Descargas.
 *
 * LIBRERÍAS NECESARIAS (Arduino IDE → Administrador de Bibliotecas):
 *   - "ESPAsyncWebServer" (fork "ESP32Async/ESPAsyncWebServer" recomendado)
 *   - "AsyncTCP"          (fork "ESP32Async/AsyncTCP" recomendado)
 *   - "WiFiManager" de tzapu
 *   - "PubSubClient" de Nick O'Leary
 *   - LittleFS, WiFi, WiFiClientSecure, ESPmDNS, BluetoothSerial →
 *     incluidas en el core ESP32, no requieren instalación aparte.
 *
 * CONFIGURA ANTES DE SUBIR:
 *   Reemplaza MQTT_HOST / MQTT_USER / MQTT_PASS con los datos de TU cluster
 *   de HiveMQ Cloud (ver instrucciones aparte para crear uno gratis).
 *
 * IMPORTANTE — Partición de memoria:
 *   En Herramientas → Partition Scheme elige un esquema CON espacio para
 *   SPIFFS/LittleFS, p.ej. "Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)".
 *
 * CAMBIOS vs v7:
 *   - Se reemplaza el WiFi Access Point por WiFi STA + WiFiManager (portal
 *     cautivo solo si no hay red guardada) — el ESP32 ahora tiene internet.
 *   - Se agrega cliente MQTT (HiveMQ Cloud, TLS) para control/datos en vivo
 *     desde cualquier lugar, replicando la arquitectura de
 *     Estimulador_CICATA_UNAM_GOOD.
 *   - El WebSocket local de v7 se retira (su función la cubre MQTT); el
 *     servidor web local se mantiene SOLO para listar/descargar/borrar
 *     archivos y servir la página cuando estás en la misma red.
 *   - Se agrega mDNS (http://nirs32.local/) para no depender de recordar
 *     la IP local.
 *   - BT se mantiene 100% funcional (modo dual, según lo solicitado).
 */

#include <Wire.h>
#include <ctype.h>
#include <BluetoothSerial.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
// =========================================================
// Web app NIRS — control y datos en vivo por MQTT (HiveMQ Cloud),
// funciona desde cualquier lugar con internet. La descarga de
// archivos CSV/bin requiere estar en la misma red que el ESP32
// (usa su IP local o http://nirs32.local/).
//
// Esta misma página la sirve el ESP32 localmente (conveniente en el
// laboratorio) Y puede publicarse tal cual en GitHub Pages u otro
// hosting estático para acceso remoto — no tiene dependencias de
// servidor propias, todo el estado vive en el broker MQTT.
// =========================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1">
<title>NIRS 2.0 — Control</title>
<script src="https://cdnjs.cloudflare.com/ajax/libs/mqtt/5.3.4/mqtt.min.js"></script>
<style>
  :root{
    --bg:#12151a; --panel:#1b1f27; --panel2:#232833; --border:#323846;
    --text:#e8eaef; --muted:#8a91a3; --accent:#4da3ff; --accent2:#ff6b6b;
    --green:#3ddc84; --orange:#ffb648; --red:#ff5c5c;
  }
  *{box-sizing:border-box;}
  body{margin:0;background:var(--bg);color:var(--text);
       font-family:-apple-system,Segoe UI,Roboto,Arial,sans-serif;font-size:15px;}
  header{position:sticky;top:0;z-index:10;background:var(--panel);
         border-bottom:1px solid var(--border);padding:10px 14px;
         display:flex;align-items:center;justify-content:space-between;}
  header h1{font-size:16px;margin:0;font-weight:600;}
  .dot{width:10px;height:10px;border-radius:50%;display:inline-block;margin-right:6px;background:var(--red);}
  .dot.on{background:var(--green);}
  #connLabel{font-size:12px;color:var(--muted);display:flex;align-items:center;}
  nav{display:flex;background:var(--panel);border-bottom:1px solid var(--border);
      position:sticky;top:41px;z-index:9;overflow-x:auto;}
  nav button{flex:1;min-width:80px;background:none;border:none;color:var(--muted);
             padding:11px 6px;font-size:13px;font-weight:600;border-bottom:2px solid transparent;}
  nav button.active{color:var(--accent);border-bottom-color:var(--accent);}
  main{padding:12px;max-width:640px;margin:0 auto;}
  .tab{display:none;} .tab.active{display:block;}
  .card{background:var(--panel);border:1px solid var(--border);border-radius:12px;
        padding:12px;margin-bottom:12px;}
  .card h2{font-size:13px;text-transform:uppercase;letter-spacing:.04em;color:var(--muted);
           margin:0 0 10px;}
  .row{display:flex;gap:8px;align-items:center;margin-bottom:8px;flex-wrap:wrap;}
  .row label{flex:0 0 96px;color:var(--muted);font-size:13px;}
  input,select{background:var(--panel2);border:1px solid var(--border);color:var(--text);
        border-radius:8px;padding:8px 10px;font-size:14px;flex:1;min-width:0;}
  input[type=number]{width:90px;flex:0 0 90px;}
  button.btn{background:var(--accent);color:#08131f;border:none;border-radius:8px;
        padding:9px 14px;font-size:13px;font-weight:700;cursor:pointer;}
  button.btn.secondary{background:var(--panel2);color:var(--text);border:1px solid var(--border);}
  button.btn.danger{background:var(--red);color:#2a0000;}
  button.btn.wide{width:100%;padding:12px;font-size:15px;}
  button.btn:active{opacity:.8;}
  .btnrow{display:flex;gap:8px;flex-wrap:wrap;}
  canvas{width:100%;height:150px;background:#0c0e12;border-radius:8px;display:block;
         border:1px solid var(--border);}
  .legend{display:flex;gap:14px;font-size:12px;color:var(--muted);margin:6px 0 2px;}
  .legend span{display:inline-flex;align-items:center;gap:5px;}
  .sw{width:10px;height:10px;border-radius:2px;display:inline-block;}
  .muted{color:var(--muted);font-size:12px;}
  .statusline{font-size:13px;margin-top:6px;}
  .statusline.ok{color:var(--green);} .statusline.warn{color:var(--orange);} .statusline.err{color:var(--red);}
  #log{background:#0c0e12;border:1px solid var(--border);border-radius:8px;padding:8px;
       height:110px;overflow-y:auto;font-family:ui-monospace,Consolas,monospace;font-size:11px;
       color:var(--muted);white-space:pre-wrap;}
  .filelist{list-style:none;margin:0;padding:0;}
  .filelist li{display:flex;align-items:center;justify-content:space-between;gap:8px;
        padding:8px 0;border-bottom:1px solid var(--border);}
  .filelist li:last-child{border-bottom:none;}
  .fname{font-size:13px;word-break:break-all;}
  .fsize{font-size:11px;color:var(--muted);}
  .flinks a, .flinks button{font-size:12px;padding:6px 9px;}
  a.dl{background:var(--green);color:#062814;border-radius:6px;padding:6px 9px;
       text-decoration:none;font-weight:700;font-size:12px;}
  .bar{height:6px;border-radius:3px;background:var(--panel2);overflow:hidden;margin-top:6px;}
  .bar>div{height:100%;background:var(--accent);}
  .grid2{display:grid;grid-template-columns:1fr 1fr;gap:8px;}
  .pill{display:inline-block;background:var(--panel2);border:1px solid var(--border);
        border-radius:999px;padding:2px 9px;font-size:11px;color:var(--muted);}
  fieldset{border:1px solid var(--border);border-radius:10px;margin:0 0 10px;padding:8px 10px;}
  legend{font-size:12px;color:var(--muted);padding:0 4px;}
</style>
</head>
<body>

<header>
  <h1>NIRS 2.0</h1>
  <div id="connLabel"><span class="dot" id="connDot"></span><span id="connText">Conectando…</span></div>
</header>

<nav>
  <button class="active" data-tab="principal">Principal</button>
  <button data-tab="s1">Sensor 1</button>
  <button data-tab="s2">Sensor 2</button>
  <button data-tab="archivos">Archivos</button>
</nav>

<main>

  <!-- ======================= TAB PRINCIPAL ======================= -->
  <section class="tab active" id="tab-principal">

    <div class="card">
      <h2>Sensor 1 — IR / Rojo</h2>
      <canvas id="chart1" width="600" height="150"></canvas>
      <div class="legend">
        <span><i class="sw" style="background:#4da3ff"></i>IR</span>
        <span><i class="sw" style="background:#ff5c5c"></i>Rojo</span>
      </div>
    </div>

    <div class="card">
      <h2>Sensor 2 — IR / Rojo</h2>
      <canvas id="chart2" width="600" height="150"></canvas>
      <div class="legend">
        <span><i class="sw" style="background:#4da3ff"></i>IR</span>
        <span><i class="sw" style="background:#ff5c5c"></i>Rojo</span>
      </div>
    </div>

    <div class="card">
      <h2>Adquisición</h2>
      <div class="row">
        <label>Modo</label>
        <select id="acqMode">
          <option value="realtime">Tiempo real (sin guardar)</option>
          <option value="timed">Temporizado (guarda CSV)</option>
        </select>
      </div>
      <div id="timedFields">
        <div class="row"><label>Nombre</label><input id="acqName" type="text" placeholder="experimento1"></div>
        <div class="row"><label>Duración (s)</label><input id="acqDuration" type="number" value="600" min="1"></div>
      </div>
      <div class="btnrow">
        <button class="btn" id="btnPlay">▶ Iniciar</button>
        <button class="btn secondary" id="btnStop" disabled>■ Detener</button>
      </div>
      <div class="statusline" id="acqStatus">Listo.</div>
    </div>

    <div class="card">
      <h2>Consola</h2>
      <div id="log"></div>
    </div>

  </section>

  <!-- ======================= TAB SENSOR (plantilla) ======================= -->
  <section class="tab" id="tab-s1"></section>
  <section class="tab" id="tab-s2"></section>

  <!-- ======================= TAB ARCHIVOS ======================= -->
  <section class="tab" id="tab-archivos">
    <div class="card" id="localAddrCard" style="display:none;">
      <h2>Conexión local (para descargar)</h2>
      <div class="muted" style="margin-bottom:8px;">
        Esta página se cargó desde internet, así que para listar/descargar
        archivos necesito la dirección local del ESP32 — solo funciona si
        tu teléfono está en la MISMA red WiFi que él.
      </div>
      <div class="row">
        <label>IP o nirs32.local</label>
        <input type="text" id="localAddr" placeholder="192.168.1.45 o nirs32.local">
      </div>
      <button class="btn wide" id="btnSetLocalAddr">Conectar localmente</button>
    </div>
    <div class="card">
      <h2>Espacio en el ESP32</h2>
      <div class="muted" id="spaceLabel">-- / -- KB</div>
      <div class="bar"><div id="spaceBar" style="width:0%"></div></div>
    </div>
    <div class="card">
      <h2>Archivos CSV / config</h2>
      <div class="btnrow" style="margin-bottom:10px;">
        <button class="btn secondary" id="btnRefreshFiles">↻ Actualizar lista</button>
      </div>
      <ul class="filelist" id="fileList"><li class="muted">Sin archivos aún.</li></ul>
    </div>
  </section>

</main>

<script>
// ============================================================
// CONFIGURACIÓN / CONSTANTES (mismos mapeos que la GUI de Python)
// ============================================================
const FSAMPLE_PRESETS = {
  "1000 Hz":0x0008, "500 Hz":0x0010, "400 Hz":0x0014, "200 Hz":0x0028,
  "100 Hz":0x0050, "50 Hz":0x00A0, "25 Hz":0x0140, "10 Hz":0x0320
};
const AVG_PRESETS = {
  "Sin promediado (1x)":0x0000, "2x":0x0110, "4x":0x0220,
  "8x":0x0330, "16x":0x0440, "32x":0x0550
};
const TIA_RF_MAP   = {"200 kΩ":0x0, "100 kΩ":0x1, "50 kΩ":0x2, "25 kΩ":0x3};
const RINT_MAP     = {"400 kΩ (max SNR)":0x000, "200 kΩ":0x100, "100 kΩ":0x200};
const AFE_BASE_CONFIG = 0x1C38;

const ILED_FINE_DEFAULT = 0xC;
const FINE_FACTOR = 0.74 + 0.022*ILED_FINE_DEFAULT;
function getCoarseForCurrent(targetMa, scale){
  const sf = 0.1 + 0.9*scale;
  let bestC=0, bestE=Infinity;
  for(let c=0;c<16;c++){
    const e = Math.abs((50.3+19.8*c)*FINE_FACTOR*sf - targetMa);
    if(e<bestE){bestE=e; bestC=c;}
  }
  return bestC;
}
function convertMaToReg(ma){
  if(ma<=0) return 0;
  const MAX_SCALE0 = (50.3+19.8*15)*FINE_FACTOR*0.1;
  const scale = ma<=MAX_SCALE0 ? 0 : 1;
  return 0x1000 | (scale<<13) | getCoarseForCurrent(ma, scale);
}
function getActualCurrentMa(reg){
  const scale=(reg>>13)&0x1, coarse=reg&0xF;
  return Math.round((50.3+19.8*coarse)*FINE_FACTOR*(0.1+0.9*scale)*10)/10;
}
function buildTiaReg(rfKey, rintKey){
  return AFE_BASE_CONFIG | (RINT_MAP[rintKey]||0) | (TIA_RF_MAP[rfKey]||0);
}

// ============================================================
// MQTT (HiveMQ Cloud, vía WebSocket seguro — control y datos en vivo
// desde cualquier lugar con internet, sin importar la red del ESP32)
// ============================================================
// ⚠️ Deben coincidir EXACTAMENTE con los valores del firmware (.ino)
const MQTT_HOST     = "981aa97ca1ed427699decebef462d689.s1.eu.hivemq.cloud";
const MQTT_WSS_PORT = 8884;
const MQTT_USER     = "sensornirs";
const MQTT_PASS     = "sensornirs123";
const TOPIC_CMD    = "nirs/cmd";
const TOPIC_DATA   = "nirs/data";
const TOPIC_STATUS = "nirs/status";

let mqttClientJS = null, mqttConnected = false;
function mqttConnectJS(){
  const url = `wss://${MQTT_HOST}:${MQTT_WSS_PORT}/mqtt`;
  mqttClientJS = mqtt.connect(url, {
    username: MQTT_USER, password: MQTT_PASS,
    clientId: "nirs_web_" + Math.random().toString(16).slice(2),
    reconnectPeriod: 2000,
  });
  mqttClientJS.on('connect', () => {
    mqttConnected = true; setConn(true);
    mqttClientJS.subscribe(TOPIC_DATA);
    mqttClientJS.subscribe(TOPIC_STATUS);
    send('STATUS');
  });
  mqttClientJS.on('close',  () => { mqttConnected = false; setConn(false); });
  mqttClientJS.on('error',  (e) => { console.error('MQTT error:', e); });
  mqttClientJS.on('message', (topic, payload) => {
    const msg = payload.toString();
    if(topic === TOPIC_DATA)   onDataMessage(msg);
    else                       onStatusMessage(msg);
  });
}
function setConn(ok){
  document.getElementById('connDot').classList.toggle('on', ok);
  document.getElementById('connText').textContent = ok ? "Conectado (MQTT)" : "Desconectado — reintentando…";
}
function send(cmd){
  if(mqttClientJS && mqttConnected) mqttClientJS.publish(TOPIC_CMD, cmd);
  logLine("> " + cmd);
}
const dataLineRe = /^\d+,\d+,\d+,\d+$/;
function onDataMessage(msg){
  if(dataLineRe.test(msg)){
    const [ir1,red1,ir2,red2] = msg.split(',').map(Number);
    pushSample(1, ir1, red1);
    pushSample(2, ir2, red2);
  }
}
function onStatusMessage(msg){
  if(msg === "ONLINE" || msg === "OFFLINE"){ logLine("ESP32: " + msg); return; }
  if(msg.startsWith("FILES:")) { renderFiles(msg.substring(6)); return; }
  if(msg.startsWith("ACK: RECORD_START:")) { onRecordStarted(msg.substring(19)); }
  if(msg.startsWith("ACK: RECORD_STOP:"))  { onRecordStopped(); }
  logLine(msg);
}
function logLine(s){
  const el = document.getElementById('log');
  el.textContent += s + "\n";
  if(el.textContent.length > 12000) el.textContent = el.textContent.slice(-8000);
  el.scrollTop = el.scrollHeight;
}

// ============================================================
// GRÁFICAS (canvas nativo, ventana deslizante de 10 s)
// ============================================================
const WINDOW_S = 10.0;
const charts = {
  1: {canvas:null, ctx:null, t:[], ir:[], red:[], t0:performance.now()},
  2: {canvas:null, ctx:null, t:[], ir:[], red:[], t0:performance.now()},
};
function initCharts(){
  for(const id of [1,2]){
    const c = document.getElementById('chart'+id);
    const ctx = c.getContext('2d');
    const ratio = window.devicePixelRatio || 1;
    c.width = c.clientWidth*ratio; c.height = c.clientHeight*ratio;
    ctx.scale(ratio, ratio);
    charts[id].canvas = c; charts[id].ctx = ctx;
  }
}
function pushSample(id, ir, red){
  const ch = charts[id];
  const t = (performance.now() - ch.t0)/1000.0;
  ch.t.push(t); ch.ir.push(ir); ch.red.push(red);
  const tMin = t - WINDOW_S;
  while(ch.t.length && ch.t[0] < tMin){ ch.t.shift(); ch.ir.shift(); ch.red.shift(); }
}
function drawChart(id){
  const ch = charts[id];
  if(!ch.ctx || ch.t.length<2) return;
  const ctx = ch.ctx;
  const W = ch.canvas.clientWidth, H = ch.canvas.clientHeight;
  ctx.clearRect(0,0,W,H);
  const tMax = ch.t[ch.t.length-1], tMin = tMax - WINDOW_S;
  let mn = Math.min(...ch.ir, ...ch.red), mx = Math.max(...ch.ir, ...ch.red);
  if(mx-mn < 10){ mx += 10; mn -= 10; }
  const pad = (mx-mn)*0.08; mn -= pad; mx += pad;
  const px = t => ((t-tMin)/(tMax-tMin||1))*W;
  const py = v => H - ((v-mn)/(mx-mn||1))*H;
  // grid
  ctx.strokeStyle = "#232833"; ctx.lineWidth=1;
  for(let i=0;i<=4;i++){ const y=H*i/4; ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(W,y); ctx.stroke(); }
  function drawLine(arr, color){
    ctx.strokeStyle = color; ctx.lineWidth = 1.6; ctx.beginPath();
    for(let i=0;i<ch.t.length;i++){
      const x = px(ch.t[i]), y = py(arr[i]);
      if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    }
    ctx.stroke();
  }
  drawLine(ch.ir, "#4da3ff");
  drawLine(ch.red, "#ff5c5c");
}
function animate(){ drawChart(1); drawChart(2); requestAnimationFrame(animate); }

// ============================================================
// ADQUISICIÓN (tiempo real / temporizado con grabación CSV)
// ============================================================
let recording = false, recTimer = null, recRemaining = 0;
document.getElementById('acqMode').addEventListener('change', (e)=>{
  document.getElementById('timedFields').style.display = (e.target.value==='timed') ? 'block' : 'none';
});
document.getElementById('btnPlay').addEventListener('click', ()=>{
  const mode = document.getElementById('acqMode').value;
  if(mode === 'realtime'){
    setAcqStatus('Viendo en tiempo real (sin guardar).', 'ok');
    document.getElementById('btnPlay').disabled = true;
    document.getElementById('btnStop').disabled = false;
    return;
  }
  const name = (document.getElementById('acqName').value || 'registro').trim();
  const dur  = parseInt(document.getElementById('acqDuration').value || '600', 10);
  const iso  = new Date().toISOString();
  send('RECORD_START:' + name + ':' + iso);
  recRemaining = dur;
  document.getElementById('btnPlay').disabled = true;
});
document.getElementById('btnStop').addEventListener('click', ()=>{
  if(recording){ send('RECORD_STOP'); }
  else {
    document.getElementById('btnPlay').disabled = false;
    document.getElementById('btnStop').disabled = true;
    setAcqStatus('Listo.', '');
  }
});
function onRecordStarted(name){
  recording = true;
  document.getElementById('btnStop').disabled = false;
  setAcqStatus('Grabando "' + name + '"… ' + recRemaining + ' s restantes', 'warn');
  clearInterval(recTimer);
  recTimer = setInterval(()=>{
    recRemaining--;
    setAcqStatus('Grabando "' + name + '"… ' + Math.max(recRemaining,0) + ' s restantes', 'warn');
    if(recRemaining<=0){ clearInterval(recTimer); send('RECORD_STOP'); }
  }, 1000);
}
function onRecordStopped(){
  recording = false;
  clearInterval(recTimer);
  document.getElementById('btnPlay').disabled = false;
  document.getElementById('btnStop').disabled = true;
  setAcqStatus('Grabación guardada. Ve a la pestaña "Archivos" para descargarla.', 'ok');
  refreshFiles();
}
function setAcqStatus(txt, cls){
  const el = document.getElementById('acqStatus');
  el.textContent = txt;
  el.className = 'statusline ' + (cls||'');
}

// ============================================================
// ARCHIVOS — requieren la IP/host LOCAL del ESP32
// ============================================================
// Si la página se sirvió desde el propio ESP32 (IP local, .local, o
// localhost), usamos rutas relativas directamente. Si se cargó desde
// un hosting externo (GitHub Pages, etc.), pedimos la IP local al usuario.
function isLocalHost(h){
  return h === 'localhost' || h.endsWith('.local') ||
         /^192\.168\.\d+\.\d+$/.test(h) || /^10\.\d+\.\d+\.\d+$/.test(h) ||
         /^172\.(1[6-9]|2\d|3[0-1])\.\d+\.\d+$/.test(h);
}
let localBase = isLocalHost(location.hostname) ? '' : null;
if(localBase === null){
  const saved = localStorage_safe_get('nirs_local_addr');
  if(saved) localBase = 'http://' + saved;
}
function localStorage_safe_get(k){ try{ return localStorage.getItem(k); }catch(e){ return null; } }
function localStorage_safe_set(k,v){ try{ localStorage.setItem(k,v); }catch(e){} }

function updateLocalAddrUI(){
  const card = document.getElementById('localAddrCard');
  card.style.display = isLocalHost(location.hostname) ? 'none' : 'block';
}
document.getElementById('btnSetLocalAddr').addEventListener('click', ()=>{
  const v = document.getElementById('localAddr').value.trim();
  if(!v) return;
  localBase = 'http://' + v;
  localStorage_safe_set('nirs_local_addr', v);
  refreshFiles();
});

function refreshFiles(){
  if(localBase === null){
    document.getElementById('fileList').innerHTML =
      '<li class="muted">Ingresa la IP local del ESP32 arriba para ver tus archivos.</li>';
    return;
  }
  fetch(localBase+'/files').then(r=>r.json()).then(list=>{
    const ul = document.getElementById('fileList');
    if(!list.length){ ul.innerHTML = '<li class="muted">Sin archivos aún.</li>'; return; }
    ul.innerHTML = '';
    list.forEach(f=>{
      const li = document.createElement('li');
      li.innerHTML =
        '<div><div class="fname">'+f.name+'</div><div class="fsize">'+(f.size/1024).toFixed(1)+' KB</div></div>' +
        '<div class="flinks"><a class="dl" href="'+localBase+'/download?f='+encodeURIComponent(f.name)+'">Descargar</a> ' +
        '<button class="btn danger" onclick="deleteFile(\''+f.name+'\')">Borrar</button></div>';
      ul.appendChild(li);
    });
  }).catch(()=>{
    document.getElementById('fileList').innerHTML =
      '<li class="muted">No se pudo conectar al ESP32 en esa dirección (¿misma red WiFi?).</li>';
  });
  fetch(localBase+'/space').then(r=>r.json()).then(s=>{
    const usedKB = (s.used/1024).toFixed(0), totKB=(s.total/1024).toFixed(0);
    document.getElementById('spaceLabel').textContent = usedKB+' / '+totKB+' KB usados';
    const pct = s.total ? Math.min(100, (s.used/s.total)*100) : 0;
    document.getElementById('spaceBar').style.width = pct+'%';
    document.getElementById('spaceBar').style.background = pct>85 ? 'var(--red)' : (pct>60 ? 'var(--orange)':'var(--accent)');
  }).catch(()=>{});
}
function deleteFile(name){
  if(!confirm('¿Borrar '+name+'?')) return;
  fetch(localBase+'/delete?f='+encodeURIComponent(name)).then(()=>refreshFiles());
}
document.getElementById('btnRefreshFiles').addEventListener('click', refreshFiles);

// ============================================================
// PANEL DE SENSOR (S1 / S2) — generado dinámicamente
// ============================================================
function buildSensorTab(suffix, label){
  const wrap = document.createElement('div');
  wrap.innerHTML = `
    <div class="card">
      <h2>${label} — Corriente LED</h2>
      <div class="row"><label>IR (Slot A)</label>
        <input type="number" id="ir_ma_${suffix}" value="50" step="0.1">
        <button class="btn" onclick="applyCurrent('${suffix}','IR')">Aplicar</button>
        <button class="btn secondary" onclick="ledOff('${suffix}','IR')">OFF</button>
      </div>
      <div class="row"><label>Rojo (Slot B)</label>
        <input type="number" id="red_ma_${suffix}" value="50" step="0.1">
        <button class="btn" onclick="applyCurrent('${suffix}','Rojo')">Aplicar</button>
        <button class="btn secondary" onclick="ledOff('${suffix}','Rojo')">OFF</button>
      </div>
      <div class="muted" id="led_info_${suffix}">—</div>
    </div>

    <div class="card">
      <h2>${label} — Ganancia AFE</h2>
      <fieldset><legend>Slot A — IR</legend>
        <div class="row"><label>R_F (TIA)</label>
          <select id="rf_a_${suffix}">${opts(TIA_RF_MAP,'200 kΩ')}</select></div>
        <div class="row"><label>R_INT</label>
          <select id="rint_a_${suffix}">${opts(RINT_MAP,'400 kΩ (max SNR)')}</select></div>
        <div class="row"><label>Pulsos</label>
          <input type="number" id="pulse_a_${suffix}" value="8" min="1" max="255"></div>
        <button class="btn" onclick="applyAfeSlot('${suffix}','A')">Aplicar Slot A</button>
      </fieldset>
      <fieldset><legend>Slot B — Rojo</legend>
        <div class="row"><label>R_F (TIA)</label>
          <select id="rf_b_${suffix}">${opts(TIA_RF_MAP,'200 kΩ')}</select></div>
        <div class="row"><label>R_INT</label>
          <select id="rint_b_${suffix}">${opts(RINT_MAP,'400 kΩ (max SNR)')}</select></div>
        <div class="row"><label>Pulsos</label>
          <input type="number" id="pulse_b_${suffix}" value="8" min="1" max="255"></div>
        <button class="btn" onclick="applyAfeSlot('${suffix}','B')">Aplicar Slot B</button>
      </fieldset>
    </div>

    <div class="card">
      <h2>${label} — Frecuencia y promediado</h2>
      <div class="row"><label>Frecuencia</label>
        <select id="fsample_${suffix}">${opts(FSAMPLE_PRESETS,'100 Hz')}</select></div>
      <div class="row"><label>Promediado</label>
        <select id="avg_${suffix}">${opts(AVG_PRESETS,'Sin promediado (1x)')}</select></div>
      <button class="btn wide" onclick="applyFsAvg('${suffix}')">Aplicar frecuencia y promediado</button>
      <div class="muted" style="margin-top:6px;">Si promedias Nx, sube fSAMPLE Nx para mantener la tasa real de salida.</div>
    </div>

    <div class="card">
      <h2>${label} — VBias / Chop</h2>
      <div class="row"><label>VBias</label>
        <input type="checkbox" id="vbias_${suffix}" style="flex:0;width:20px;">
        <button class="btn secondary" onclick="applyVbias('${suffix}')">Aplicar</button>
      </div>
      <div class="muted">Chop Mode deshabilitado en firmware (config. incompleta causaba señal en 0).</div>
    </div>

    <div class="card">
      <h2>${label} — Comando manual / estado</h2>
      <div class="row">
        <input type="text" id="manual_${suffix}" placeholder="Ej: STATUS${suffix}">
        <button class="btn secondary" onclick="sendManual('${suffix}')">Enviar</button>
      </div>
      <button class="btn secondary wide" onclick="send('STATUS${suffix}')">Leer STATUS${suffix}</button>
    </div>
  `;
  return wrap;
}
function opts(map, def){
  return Object.keys(map).map(k => `<option value="${k}" ${k===def?'selected':''}>${k}</option>`).join('');
}
function applyCurrent(suffix, led){
  const id = led==='IR' ? 'ir_ma_'+suffix : 'red_ma_'+suffix;
  const ma = parseFloat(document.getElementById(id).value);
  if(isNaN(ma) || ma<0){ alert('Ingresa una corriente válida en mA.'); return; }
  const reg = convertMaToReg(ma);
  send((led==='IR'?'IR':'Rojo') + suffix + '=0x' + reg.toString(16));
  document.getElementById('led_info_'+suffix).textContent =
    led+': '+(ma<=0?'OFF':getActualCurrentMa(reg)+' mA')+' (reg 0x'+reg.toString(16)+')';
}
function ledOff(suffix, led){
  send((led==='IR'?'IR':'Rojo') + suffix + '=0x0000');
}
function applyAfeSlot(suffix, slot){
  const sl = slot.toLowerCase();
  const p = Math.max(1, Math.min(255, parseInt(document.getElementById('pulse_'+sl+'_'+suffix).value||'8',10)));
  const rfKey   = document.getElementById('rf_'+sl+'_'+suffix).value;
  const rintKey = document.getElementById('rint_'+sl+'_'+suffix).value;
  const tiaReg   = buildTiaReg(rfKey, rintKey);
  const pulseReg = (p<<8) | 0x18;
  send('TIA_'+slot+suffix+'=0x'+tiaReg.toString(16));
  send('PULSES_'+slot+suffix+'=0x'+pulseReg.toString(16));
}
function applyFsAvg(suffix){
  const fsKey  = document.getElementById('fsample_'+suffix).value;
  const avgKey = document.getElementById('avg_'+suffix).value;
  send('FSAMPLE'+suffix+'=0x'+FSAMPLE_PRESETS[fsKey].toString(16));
  send('AVG'+suffix+'=0x'+AVG_PRESETS[avgKey].toString(16));
}
function applyVbias(suffix){
  const on = document.getElementById('vbias_'+suffix).checked ? '1' : '0';
  send('VBIAS'+suffix+'='+on);
}
function sendManual(suffix){
  const el = document.getElementById('manual_'+suffix);
  const cmd = el.value.trim();
  if(cmd) send(cmd);
}

// ============================================================
// NAVEGACIÓN / INIT
// ============================================================
document.querySelectorAll('nav button').forEach(btn=>{
  btn.addEventListener('click', ()=>{
    document.querySelectorAll('nav button').forEach(b=>b.classList.remove('active'));
    document.querySelectorAll('.tab').forEach(t=>t.classList.remove('active'));
    btn.classList.add('active');
    document.getElementById('tab-'+btn.dataset.tab).classList.add('active');
  });
});

window.addEventListener('load', ()=>{
  document.getElementById('tab-s1').appendChild(buildSensorTab('_S1','Sensor 1'));
  document.getElementById('tab-s2').appendChild(buildSensorTab('_S2','Sensor 2'));
  initCharts();
  animate();
  updateLocalAddrUI();
  mqttConnectJS();
  refreshFiles();
});
</script>
</body>
</html>
)rawliteral";

BluetoothSerial SerialBT;

#define DBG(x)   Serial.print(x)
#define DBGLN(x) Serial.println(x)

// -------------------------------------------------------
// WiFi (STA, vía WiFiManager) + MQTT (HiveMQ Cloud)
// -------------------------------------------------------
WiFiManager wifiManager;
const char* WIFI_PORTAL_NAME = "NIRS_Setup"; // AP temporal solo para configurar tu WiFi real
const int   MAX_INTENTOS_WIFI = 5;

// ⚠️ REEMPLAZA estos 3 valores con los de TU cluster de HiveMQ Cloud
const char* MQTT_HOST = "981aa97ca1ed427699decebef462d689.s1.eu.hivemq.cloud";
const int   MQTT_PORT = 8883;
const char* MQTT_USER = "sensornirs";
const char* MQTT_PASS = "sensornirs123";

const char* TOPIC_CMD    = "nirs/cmd";
const char* TOPIC_DATA   = "nirs/data";
const char* TOPIC_STATUS = "nirs/status";

WiFiClientSecure secureClient;
PubSubClient mqttClient(secureClient);

const char* MDNS_HOSTNAME = "nirs32"; // → http://nirs32.local/

AsyncWebServer server(80);

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

// Throttle de difusión en vivo hacia MQTT (no limita la grabación local)
uint32_t g_last_mqtt_broadcast_ms = 0;
const uint32_t MQTT_BROADCAST_INTERVAL_MS = 100; // 10 Hz — cuida el ancho de banda del broker

// =========================================================
// SALIDA UNIFICADA — BT + MQTT (reemplaza los btPrint* de v6/v7)
// =========================================================
void sendReply(const String &s) {
  if (SerialBT.hasClient()) SerialBT.println(s);
  if (mqttClient.connected()) mqttClient.publish(TOPIC_STATUS, s.c_str());
}
// Compatibilidad con el código heredado de v6/v7 (mismas firmas, ahora
// también emiten por MQTT). Se usan sobre todo en printStatus().
void btPrint(const char* s)   { if (SerialBT.hasClient()) SerialBT.print(s); }
void btPrintln(const char* s) { sendReply(String(s)); }
void btPrint(uint16_t v)      { if (SerialBT.hasClient()) SerialBT.print(v); }
void btPrintln(uint16_t v)    { sendReply(String(v)); }
void btPrint(uint32_t v)      { if (SerialBT.hasClient()) SerialBT.print(v); }
void btPrintln(uint32_t v)    { sendReply(String(v)); }
void btPrint(int v)           { if (SerialBT.hasClient()) SerialBT.print(v); }
void btPrintln(int v)         { sendReply(String(v)); }
void btPrintHex(uint16_t v)   { if (SerialBT.hasClient()) SerialBT.print(v, HEX); }
void btPrintlnHex(uint16_t v) { sendReply(String(v, HEX)); }

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
}

void applyAvg(TwoWire &bus, SensorState &s, uint16_t val) {
  s.avg = val;
  writeReg(bus, REG_MODE,    MODE_PROGRAM);
  writeReg(bus, REG_NUM_AVG, val);
  writeReg(bus, REG_MODE,    MODE_NORMAL);
  recalcLoopPeriod();
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
      if (!first) out += ";";
      String nm = String(file.name());
      if (nm.startsWith("/")) nm = nm.substring(1);
      out += nm + "|" + String(file.size());
      first = false;
    }
    file = root.openNextFile();
  }
  sendReply(out);
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
  if (cmd == "WIFI_RESET") {
    sendReply("ACK: WIFI_RESET — borrando credenciales y reiniciando...");
    delay(300);
    wifiManager.resetSettings();
    delay(300);
    ESP.restart();
    return;
  }
  if (cmd.startsWith("DELETE_FILE:")) {
    String fn = cmd.substring(12);
    fn.trim();
    if (!fn.startsWith("/")) fn = "/" + fn;
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
// MQTT — callback de mensajes entrantes (tópico nirs/cmd)
// =========================================================
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  String msg;
  msg.reserve(length);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();
  if (String(topic) == TOPIC_CMD) handleCommand(msg);
}

// =========================================================
// SERVIDOR WEB LOCAL — rutas HTTP (archivos + página, misma red)
// =========================================================
String contentTypeFor(const String &fn) {
  if (fn.endsWith(".csv"))  return "text/csv";
  if (fn.endsWith(".json")) return "application/json";
  if (fn.endsWith(".bin"))  return "application/octet-stream";
  return "text/plain";
}

void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", INDEX_HTML);
  });

  // Listado de archivos en JSON simple: [{"name":"x.csv","size":123}, ...]
  server.on("/files", HTTP_GET, [](AsyncWebServerRequest *request) {
    String out = "[";
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    bool first = true;
    while (file) {
      if (!file.isDirectory()) {
        if (!first) out += ",";
        String nm = String(file.name());
        if (nm.startsWith("/")) nm = nm.substring(1);
        out += "{\"name\":\"" + nm + "\",\"size\":" + String(file.size()) + "}";
        first = false;
      }
      file = root.openNextFile();
    }
    out += "]";
    request->send(200, "application/json", out);
  });

  // Descarga de archivo: /download?f=nombre.csv  → el navegador lo baja
  server.on("/download", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("f")) { request->send(400, "text/plain", "Falta parametro f"); return; }
    String fn = request->getParam("f")->value();
    if (!fn.startsWith("/")) fn = "/" + fn;
    if (!LittleFS.exists(fn)) { request->send(404, "text/plain", "No existe"); return; }
    request->send(LittleFS, fn, contentTypeFor(fn), true); // true = descargar (Content-Disposition)
  });

  // Borrar archivo: /delete?f=nombre.csv
  server.on("/delete", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("f")) { request->send(400, "text/plain", "Falta parametro f"); return; }
    String fn = request->getParam("f")->value();
    if (!fn.startsWith("/")) fn = "/" + fn;
    bool ok = LittleFS.remove(fn);
    request->send(ok ? 200 : 500, "text/plain", ok ? "OK" : "ERROR");
  });

  // Espacio usado/total en LittleFS (bytes)
  server.on("/space", HTTP_GET, [](AsyncWebServerRequest *request) {
    String out = "{\"used\":" + String(LittleFS.usedBytes()) +
                 ",\"total\":" + String(LittleFS.totalBytes()) + "}";
    request->send(200, "application/json", out);
  });

  server.begin();
}

// =========================================================
// WIFI (STA vía WiFiManager) + MQTT (HiveMQ Cloud)
// =========================================================
bool conectarWiFi() {
  wifiManager.setConnectTimeout(10);        // segundos por intento con credenciales guardadas
  wifiManager.setConfigPortalTimeout(180);  // 3 min con el portal de configuración abierto

  for (int intento = 1; intento <= MAX_INTENTOS_WIFI; intento++) {
    Serial.printf("Intento de conexion WiFi #%d de %d\n", intento, MAX_INTENTOS_WIFI);
    // autoConnect: usa credenciales guardadas si existen; si no, abre el
    // portal "NIRS_Setup" para configurarlas desde el celular.
    bool exito = wifiManager.autoConnect(WIFI_PORTAL_NAME);
    if (exito && WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi conectado. IP: " + WiFi.localIP().toString());
      Serial.println("Red: " + WiFi.SSID());
      return true;
    }
    Serial.println("Fallo el intento. Reintentando...");
    delay(500);
  }
  Serial.println("No se pudo conectar al WiFi tras varios intentos.");
  return false;
}

void conectarMQTT() {
  secureClient.setInsecure(); // Para mayor seguridad en producción, usa el certificado CA de HiveMQ
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(onMqttMessage);
  mqttClient.setBufferSize(512);
  mqttClient.setKeepAlive(60);

  int intentos = 0;
  while (!mqttClient.connected() && intentos < 5) {
    Serial.println("Conectando a HiveMQ Cloud...");
    if (mqttClient.connect("ESP32_NIRS", MQTT_USER, MQTT_PASS,
                            TOPIC_STATUS, 1, true, "OFFLINE")) {
      Serial.println("Conectado a MQTT");
      mqttClient.subscribe(TOPIC_CMD);
      mqttClient.publish(TOPIC_STATUS, "ONLINE", true);
    } else {
      Serial.print("Fallo MQTT, rc="); Serial.println(mqttClient.state());
      delay(1500);
    }
    intentos++;
  }
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

  // ── WiFi (STA) + MQTT (nube) ──────────────────────────
  if (!conectarWiFi()) {
    Serial.println("Sin WiFi por ahora — se reintentará en loop(). "
                    "El servidor local y la grabación seguirán funcionando vía BT/USB.");
  } else {
    if (MDNS.begin(MDNS_HOSTNAME)) {
      Serial.print("mDNS activo: http://"); Serial.print(MDNS_HOSTNAME); Serial.println(".local/");
      MDNS.addService("http", "tcp", 80);
    }
    conectarMQTT();
  }
  setupWebServer(); // rutas HTTP locales (archivos + página) — no depende de MQTT

  // ── Bluetooth SPP (respaldo, modo dual) ───────────────
  SerialBT.begin("SENSOR_NIRS");
  Serial.println("BT iniciado como SENSOR_NIRS — esperando conexion...");

  // ── I2C sensores ───────────────────────────────────────
  Wire.begin(I2C0_SDA, I2C0_SCL);   Wire.setClock(400000);
  Wire1.begin(I2C1_SDA, I2C1_SCL);  Wire1.setClock(400000);
  delay(100);

  bool ok1 = initSensor(Wire,  S1, "Sensor1");
  bool ok2 = initSensor(Wire1, S2, "Sensor2");
  if (!ok1 && !ok2) { Serial.println("ERROR FATAL: Ningun sensor."); while(1); }

  recalcLoopPeriod();
  Serial.print("ADPD1080 Dual v8 (WiFi+MQTT+BT) listo. Loop period=");
  Serial.print(g_loop_period_us); Serial.println(" us");
  delay(200);
}

// =========================================================
// LOOP
// =========================================================
void loop() {
  uint32_t t_start = micros();

  // Comandos desde BT
  if (SerialBT.available() > 0) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();
    handleCommand(cmd);
  }

  // Comandos desde USB Serial (debug)
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    handleCommand(cmd);
  }

  // Mantener WiFi/MQTT vivos (no bloquea grabación: esa sigue en flash local)
  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t lastTry = 0;
    if (millis() - lastTry > 5000) {
      lastTry = millis();
      Serial.println("WiFi desconectado. Reintentando...");
      conectarWiFi();
    }
  } else {
    if (!mqttClient.connected()) conectarMQTT();
    mqttClient.loop();
  }

  bool haveBTClient = SerialBT.hasClient();
  bool haveMQTT     = mqttClient.connected();

  if (haveBTClient || haveMQTT || g_recording) {
    uint16_t ir1  = readReg(Wire,  REG_SLOTA_CH1);
    uint16_t red1 = readReg(Wire,  REG_SLOTB_CH1);
    uint16_t ir2  = readReg(Wire1, REG_SLOTA_CH1);
    uint16_t red2 = readReg(Wire1, REG_SLOTB_CH1);

    if (haveBTClient) {
      SerialBT.print(ir1);  SerialBT.print(",");
      SerialBT.print(red1); SerialBT.print(",");
      SerialBT.print(ir2);  SerialBT.print(",");
      SerialBT.println(red2);
    }

    if (haveMQTT) {
      uint32_t now = millis();
      if (now - g_last_mqtt_broadcast_ms >= MQTT_BROADCAST_INTERVAL_MS) {
        g_last_mqtt_broadcast_ms = now;
        String line = String(ir1) + "," + String(red1) + "," + String(ir2) + "," + String(red2);
        mqttClient.publish(TOPIC_DATA, line.c_str());
      }
    }

    if (g_recording) recordSample(ir1, red1, ir2, red2);
  }

  uint32_t elapsed = micros() - t_start;
  if (elapsed < g_loop_period_us) {
    delayMicroseconds(g_loop_period_us - elapsed);
  }
}
