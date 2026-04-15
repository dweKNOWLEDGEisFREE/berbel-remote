/**
 * ============================================================================
 * Berbel BFB 6bT - BLE Dual Role Bridge + Home Assistant
 * ============================================================================
 * ESP32-S3 N16R8
 *
 * Features:
 *  - Setup-AP beim ersten Start (kein hardcodiertes WiFi)
 *  - Webbasierter Setup-Wizard (WiFi + MQTT konfigurieren)
 *  - Settings-Seite im Webserver (nachträglich ändern)
 *  - OTA-Update über Webserver (/update)
 *  - BLE Dual Role: Peripheral (für FB) + Central (für Haube)
 *  - MQTT Auto-Discovery für Home Assistant
 *  - Alexa via HA
 * ============================================================================
 */

#include <NimBLEDevice.h>
#include <esp_mac.h>
#include <esp_coexist.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <Update.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>

// ============================================================================
// Berbel BLE UUIDs
// ============================================================================
#define BERBEL_SERVICE_UUID  "f004f000-5745-4053-8043-62657262656c"
#define BERBEL_NOTIFY_UUID   "f004f002-5745-4053-8043-62657262656c"
#define BERBEL_WRITE_UUID    "f004f001-5745-4053-8043-62657262656c"

// ============================================================================
// Button Codes
// ============================================================================
#define BTN_POWER       0x01
#define BTN_FAN_1       0x02
#define BTN_FAN_2       0x03
#define BTN_FAN_3       0x04
#define BTN_FAN_P       0x05
#define BTN_LIGHT_UP    0x06
#define BTN_MOVE_UP     0x09
#define BTN_LIGHT_DOWN  0x0A
#define BTN_TIMER       0x0C
#define BTN_MOVE_DOWN   0x0D

// ============================================================================
// MQTT Topics
// ============================================================================
#define MQTT_BASE           "berbel/hood"
#define MQTT_AVAIL          MQTT_BASE "/available"
#define MQTT_STATE          MQTT_BASE "/state"
#define MQTT_CMD_LIGHT_UP   MQTT_BASE "/light_up/set"
#define MQTT_CMD_LIGHT_DOWN MQTT_BASE "/light_down/set"
#define MQTT_CMD_FAN_PRESET MQTT_BASE "/fan/preset/set"
#define MQTT_CMD_POWER      MQTT_BASE "/power/set"
#define MQTT_CMD_NACHLAUF   MQTT_BASE "/nachlauf/set"
#define MQTT_CMD_POSITION   MQTT_BASE "/position/set"
#define MQTT_CMD_MOVE_UP    MQTT_BASE "/move_up/set"
#define MQTT_CMD_MOVE_DOWN  MQTT_BASE "/move_down/set"
#define MQTT_CMD_DEBUG      MQTT_BASE "/debug/send"

// ============================================================================
// Setup AP
// ============================================================================
#define SETUP_AP_SSID     "Berbel-Setup"
#define SETUP_AP_IP_STR   "192.168.4.1"

// ============================================================================
// LED (ESP32-S3 DevKitC-1 = GPIO 48)
// ============================================================================
#define LED_PIN      48
#define LED_BLINK_MS 500

// ============================================================================
// Config stored in NVS (Preferences)
// ============================================================================
struct Config {
  char wifi_ssid[64]    = "";
  char wifi_pass[64]    = "";
  char mqtt_host[64]    = "192.168.1.1";
  int  mqtt_port        = 1883;
  char mqtt_user[32]    = "berbel-mqtt";
  char mqtt_pass[64]    = "";
  bool hood_has_cover   = true;
};

Config cfg;
Preferences prefs;

void loadConfig() {
  prefs.begin("berbel", true);
  prefs.getString("wifi_ssid",  cfg.wifi_ssid,  sizeof(cfg.wifi_ssid));
  prefs.getString("wifi_pass",  cfg.wifi_pass,  sizeof(cfg.wifi_pass));
  prefs.getString("mqtt_host",  cfg.mqtt_host,  sizeof(cfg.mqtt_host));
  cfg.mqtt_port = prefs.getInt("mqtt_port", 1883);
  prefs.getString("mqtt_user",  cfg.mqtt_user,  sizeof(cfg.mqtt_user));
  prefs.getString("mqtt_pass",  cfg.mqtt_pass,  sizeof(cfg.mqtt_pass));
  cfg.hood_has_cover = prefs.getBool("has_cover", true);
  prefs.end();
  Serial.printf("[CFG] Loaded: ssid='%s' mqtt='%s:%d'\n",
    cfg.wifi_ssid, cfg.mqtt_host, cfg.mqtt_port);
}

void saveConfig() {
  prefs.begin("berbel", false);
  prefs.putString("wifi_ssid",  cfg.wifi_ssid);
  prefs.putString("wifi_pass",  cfg.wifi_pass);
  prefs.putString("mqtt_host",  cfg.mqtt_host);
  prefs.putInt   ("mqtt_port",  cfg.mqtt_port);
  prefs.putString("mqtt_user",  cfg.mqtt_user);
  prefs.putString("mqtt_pass",  cfg.mqtt_pass);
  prefs.putBool  ("has_cover",  cfg.hood_has_cover);
  prefs.end();
  Serial.println("[CFG] Saved to NVS");
}

void clearConfig() {
  prefs.begin("berbel", false);
  prefs.clear();
  prefs.end();
  Serial.println("[CFG] Cleared");
}

// ============================================================================
// Hood State
// ============================================================================
struct HoodState {
  bool lightUp     = false;
  bool lightDown   = false;
  uint8_t fanSpeed = 0;
  bool nachlauf    = false;
  const char* position   = "Oben";
  const char* coverState = "up";
  bool hoodBleConnected  = false;
  bool fbBleConnected    = false;
  uint8_t raw[9] = {0};
};

HoodState hood;

// ============================================================================
// Globals
// ============================================================================
// BLE Peripheral (towards FB)
NimBLEServer*         pServer     = nullptr;
NimBLECharacteristic* pNotifyChar = nullptr;
volatile bool fbConnected    = false;
bool          fbConnectedOld = false;

// BLE Central (towards hood)
NimBLEClient*               pClient         = nullptr;
NimBLERemoteCharacteristic* pHoodNotifyChar = nullptr;
NimBLERemoteCharacteristic* pHoodWriteChar  = nullptr;
volatile bool hoodConnected    = false;
bool          hoodConnectedOld = false;
bool          hoodScanStarted  = false;

// WiFi / MQTT / Web
WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);
WebServer    webServer(80);
DNSServer    dnsServer;

bool wifiStarted      = false;
bool otaReady         = false;
bool hoodStateValid   = false;
bool setupMode        = false;  // true = AP mode, false = normal mode
unsigned long lastMqttReconnect = 0;
unsigned long lastHoodReconnect = 0;

// Status update from hood
volatile bool newStatusReceived = false;
uint8_t pendingStatus[9];

// Command Queue
struct CmdEntry { uint8_t code; char name[16]; };
#define CMD_QUEUE_SIZE 16
#define CMD_DELAY_MS   300
CmdEntry      cmdQueue[CMD_QUEUE_SIZE];
int           cmdQueueHead = 0;
int           cmdQueueTail = 0;
unsigned long lastCmdSent  = 0;

// Raw Advertising Data
static uint8_t raw_adv_data[] = {
  0x02, 0x01, 0x05,
  0x12, 0x21,
  0x6c, 0x65, 0x62, 0x72, 0x65, 0x62, 0x43, 0x80,
  0x53, 0x40, 0x45, 0x57, 0x00, 0xf0, 0x00, 0xf0,
  0x01
};

// ============================================================================
// Forward declarations
// ============================================================================
void startPeripheralAdvertising();
void startHoodScan();
bool connectToHood();
void setupWebServer();
void setupWebServerAP();
void publishState();
void publishDiscovery();
void queueButton(uint8_t code, const char* name);
void mqttReconnect();
static const char* fanPresetName(uint8_t speed);

// ============================================================================
// HTML Pages (PROGMEM)
// ============================================================================

// ---------- Shared CSS ----------
static const char CSS_COMMON[] PROGMEM = R"css(
<style>
@import url('https://fonts.googleapis.com/css2?family=DM+Sans:wght@300;400;500;600&family=DM+Mono:wght@400;500&display=swap');
:root{--bg:#0f0f11;--surface:#18181c;--surface2:#222228;--border:#2a2a32;--accent:#e8c97a;--accent2:#7ac4e8;--text:#e8e8ec;--muted:#6b6b78;--on:#4ade80;--off:#ef4444;--radius:14px;}
*{box-sizing:border-box;margin:0;padding:0;}
body{font-family:'DM Sans',sans-serif;background:var(--bg);color:var(--text);min-height:100vh;padding:24px 16px 48px;}
header{max-width:520px;margin:0 auto 28px;display:flex;align-items:center;justify-content:space-between;}
.brand h1{font-size:1.4rem;font-weight:600;letter-spacing:-.02em;color:var(--accent);}
.brand span{font-size:.75rem;color:var(--muted);font-family:'DM Mono',monospace;margin-top:2px;display:block;}
.container{max-width:520px;margin:0 auto;display:flex;flex-direction:column;gap:16px;}
.card{background:var(--surface);border:1px solid var(--border);border-radius:var(--radius);padding:20px;}
.card-label{font-size:.68rem;font-family:'DM Mono',monospace;text-transform:uppercase;letter-spacing:.1em;color:var(--muted);margin-bottom:16px;}
.nav{display:flex;gap:8px;}
.nav a{padding:7px 14px;border-radius:8px;font-size:.78rem;font-weight:500;text-decoration:none;color:var(--muted);border:1px solid var(--border);transition:all .15s;}
.nav a:hover,.nav a.active{border-color:var(--accent);color:var(--accent);}
input,select{width:100%;background:var(--surface2);border:1px solid var(--border);border-radius:8px;color:var(--text);font-family:'DM Sans',sans-serif;font-size:.85rem;padding:10px 12px;margin-top:6px;outline:none;transition:border-color .15s;}
input:focus,select:focus{border-color:var(--accent2);}
label{font-size:.78rem;color:var(--muted);display:block;margin-top:12px;}
label:first-child{margin-top:0;}
.btn{display:inline-flex;align-items:center;gap:6px;border:none;border-radius:10px;font-family:'DM Sans',sans-serif;font-size:.85rem;font-weight:600;padding:11px 20px;cursor:pointer;transition:all .15s;}
.btn:hover{opacity:.85;transform:translateY(-1px);}
.btn:disabled{opacity:.4;cursor:default;transform:none;}
.btn-primary{background:var(--accent);color:#0f0f11;}
.btn-blue{background:var(--accent2);color:#0f0f11;}
.btn-danger{background:var(--off);color:#fff;}
.btn-ghost{background:var(--surface2);color:var(--text);border:1px solid var(--border);}
.btn-row{display:flex;gap:10px;margin-top:16px;flex-wrap:wrap;}
.toast{position:fixed;bottom:24px;left:50%;transform:translateX(-50%) translateY(80px);background:var(--surface2);border:1px solid var(--border);border-radius:20px;padding:10px 20px;font-size:.8rem;color:var(--text);transition:transform .3s cubic-bezier(.34,1.56,.64,1);z-index:99;white-space:nowrap;}
.toast.show{transform:translateX(-50%) translateY(0);}
.alert{padding:12px 16px;border-radius:10px;font-size:.82rem;margin-bottom:12px;}
.alert-success{background:rgba(74,222,128,.1);border:1px solid rgba(74,222,128,.3);color:#4ade80;}
.alert-error{background:rgba(239,68,68,.1);border:1px solid rgba(239,68,68,.3);color:#ef4444;}
.alert-info{background:rgba(122,196,232,.1);border:1px solid rgba(122,196,232,.3);color:var(--accent2);}
</style>
)css";

// ---------- Main Control Page ----------
static const char HTML_MAIN[] PROGMEM = R"html(<!DOCTYPE html>
<html lang="de"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Berbel</title>)html";

static const char HTML_MAIN2[] PROGMEM = R"html(
<style>
.ble-row{max-width:520px;margin:0 auto 16px;display:grid;grid-template-columns:1fr 1fr;gap:10px;}
.ble-badge{display:flex;align-items:center;gap:6px;padding:8px 12px;border-radius:10px;background:var(--surface);border:1px solid var(--border);font-size:.72rem;font-family:'DM Mono',monospace;color:var(--muted);transition:all .3s;}
.ble-badge.connected{border-color:var(--on);color:var(--on);}
.ble-dot{width:7px;height:7px;border-radius:50%;background:var(--muted);flex-shrink:0;transition:background .3s;}
.ble-badge.connected .ble-dot{background:var(--on);box-shadow:0 0 8px var(--on);animation:pulse 2s infinite;}
.ble-label{font-size:.6rem;opacity:.6;display:block;}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.4}}
.fan-grid{display:grid;grid-template-columns:repeat(5,1fr);gap:8px;}
.fan-btn{background:var(--surface2);border:1px solid var(--border);border-radius:10px;color:var(--text);font-family:'DM Sans',sans-serif;font-size:.8rem;font-weight:500;padding:12px 4px;cursor:pointer;transition:all .15s;text-align:center;}
.fan-btn:hover{border-color:var(--accent);color:var(--accent);}
.fan-btn.active{background:var(--accent);border-color:var(--accent);color:#0f0f11;font-weight:600;}
.light-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;}
.light-btn{background:var(--surface2);border:1px solid var(--border);border-radius:10px;color:var(--muted);font-family:'DM Sans',sans-serif;font-size:.85rem;font-weight:500;padding:16px;cursor:pointer;transition:all .15s;display:flex;flex-direction:column;align-items:center;gap:8px;}
.light-btn svg{opacity:.4;transition:opacity .15s;}
.light-btn:hover{border-color:var(--accent2);color:var(--text);}
.light-btn:hover svg{opacity:.8;}
.light-btn.on{border-color:var(--accent);color:var(--accent);background:rgba(232,201,122,.07);}
.light-btn.on svg{opacity:1;}
.action-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;}
.action-btn{background:var(--surface2);border:1px solid var(--border);border-radius:10px;color:var(--text);font-family:'DM Sans',sans-serif;font-size:.82rem;font-weight:500;padding:14px;cursor:pointer;transition:all .15s;display:flex;align-items:center;justify-content:center;gap:8px;}
.action-btn:hover{border-color:var(--accent2);}
.action-btn.nachlauf-on{border-color:var(--accent2);color:var(--accent2);background:rgba(122,196,232,.07);}
.action-btn.power{border-color:#3a1a1a;color:var(--off);}
.action-btn.power:hover{border-color:var(--off);background:rgba(239,68,68,.08);}
.cover-grid{display:grid;grid-template-columns:1fr auto 1fr;gap:10px;align-items:center;}
.cover-btn{background:var(--surface2);border:1px solid var(--border);border-radius:10px;color:var(--text);font-family:'DM Sans',sans-serif;font-size:.82rem;font-weight:500;padding:14px 10px;cursor:pointer;transition:all .15s;display:flex;flex-direction:column;align-items:center;gap:8px;}
.cover-btn:hover{border-color:var(--accent2);color:var(--accent2);}
.cover-btn.active{border-color:var(--accent2);color:var(--accent2);background:rgba(122,196,232,.07);}
.cover-state-badge{text-align:center;font-size:.7rem;font-family:'DM Mono',monospace;color:var(--muted);padding:6px 4px;line-height:1.4;}
.pair-grid{display:flex;flex-direction:column;gap:0;}
.pair-row{display:flex;align-items:center;justify-content:space-between;gap:16px;padding:4px 0;}
.pair-divider{height:1px;background:var(--border);margin:14px 0;}
.pair-info h3{font-size:.88rem;font-weight:500;margin-bottom:3px;}
.pair-info p{font-size:.72rem;color:var(--muted);line-height:1.5;}
.pair-btn{flex-shrink:0;border:none;border-radius:10px;font-family:'DM Sans',sans-serif;font-size:.78rem;font-weight:600;padding:10px 14px;cursor:pointer;transition:all .15s;white-space:nowrap;display:flex;align-items:center;gap:6px;}
.pair-btn:hover{opacity:.85;transform:translateY(-1px);}
.pair-btn:disabled{opacity:.4;cursor:default;transform:none;}
.hood-pair{background:var(--accent);color:#0f0f11;}
.hood-pair.searching{background:var(--surface2);color:var(--accent);border:1px solid var(--accent);}
.fb-pair{background:var(--accent2);color:#0f0f11;}
.fb-pair.waiting{background:var(--surface2);color:var(--accent2);border:1px solid var(--accent2);}
.status-bar{background:var(--surface);border:1px solid var(--border);border-radius:var(--radius);padding:14px 18px;display:flex;align-items:center;justify-content:space-between;}
.status-bar span{font-size:.72rem;font-family:'DM Mono',monospace;color:var(--muted);}
.raw-bytes{color:var(--surface2);font-size:.65rem!important;letter-spacing:.05em;}
.disc-hint{text-align:center;padding:12px;background:rgba(239,68,68,.07);border:1px solid rgba(239,68,68,.2);border-radius:10px;font-size:.8rem;color:#f87171;display:none;}
.disc-hint.show{display:block;}
</style>
</head><body>
<header>
  <div class="brand"><h1>Berbel</h1><span>Skyline &middot; berbel-remote.local</span></div>
  <nav class="nav">
    <a href="/" class="active">Steuerung</a>
    <a href="/settings">Einstellungen</a>
    <a href="/update">Update</a>
  </nav>
</header>
<div class="ble-row">
  <div class="ble-badge" id="hoodBadge"><div class="ble-dot"></div><div><span class="ble-label">HAUBE</span><span id="hoodText">Getrennt</span></div></div>
  <div class="ble-badge" id="fbBadge"><div class="ble-dot"></div><div><span class="ble-label">FERNBEDIENUNG</span><span id="fbText">Getrennt</span></div></div>
</div>
<div class="container">
  <div class="disc-hint" id="discHint">Haube nicht verbunden &mdash; Befehle werden in die Warteschlange gestellt.</div>
  <div class="card"><div class="card-label">L&uuml;fter</div>
    <div class="fan-grid">
      <button class="fan-btn" id="fan0" onclick="setFan('Aus')">Aus</button>
      <button class="fan-btn" id="fan1" onclick="setFan('Stufe 1')">Stufe 1</button>
      <button class="fan-btn" id="fan2" onclick="setFan('Stufe 2')">Stufe 2</button>
      <button class="fan-btn" id="fan3" onclick="setFan('Stufe 3')">Stufe 3</button>
      <button class="fan-btn" id="fan4" onclick="setFan('Power')">Power</button>
    </div>
  </div>
  <div class="card"><div class="card-label">Beleuchtung</div>
    <div class="light-grid">
      <button class="light-btn" id="lightUp" onclick="toggleLight('up')">
        <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8"><circle cx="12" cy="12" r="4"/><path d="M12 2v2M12 20v2M4.93 4.93l1.41 1.41M17.66 17.66l1.41 1.41M2 12h2M20 12h2M6.34 17.66l-1.41 1.41M19.07 4.93l-1.41 1.41"/></svg>
        Oberlicht
      </button>
      <button class="light-btn" id="lightDown" onclick="toggleLight('down')">
        <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8"><path d="M9 18h6M10 22h4M12 2v2"/><path d="M5 12a7 7 0 1 0 14 0 7 7 0 0 0-14 0z"/></svg>
        Unterlicht
      </button>
    </div>
  </div>
  <div class="card"><div class="card-label">Aktionen</div>
    <div class="action-grid">
      <button class="action-btn" id="nachlaufBtn" onclick="toggleNachlauf()">
        <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="10"/><polyline points="12 6 12 12 16 14"/></svg>
        Nachlauf
      </button>
      <button class="action-btn power" onclick="powerOff()">
        <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M18.36 6.64a9 9 0 1 1-12.73 0"/><line x1="12" y1="2" x2="12" y2="12"/></svg>
        Ausschalten
      </button>
    </div>
  </div>
  <div class="card"><div class="card-label">Position</div>
    <div class="cover-grid">
      <button class="cover-btn" id="coverUp" onclick="moveHood('up')">
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="17 11 12 6 7 11"/><polyline points="17 18 12 13 7 18"/></svg>
        Hochfahren
      </button>
      <div class="cover-state-badge" id="coverState">&ndash;</div>
      <button class="cover-btn" id="coverDown" onclick="moveHood('down')">
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="7 13 12 18 17 13"/><polyline points="7 6 12 11 17 6"/></svg>
        Herunterfahren
      </button>
    </div>
  </div>
  <div class="card"><div class="card-label">Verbindungen</div>
    <div class="pair-grid">
      <div class="pair-row">
        <div class="pair-info"><h3>Berbel Haube</h3><p>ESP32 sucht automatisch nach der Haube.</p></div>
        <button class="pair-btn hood-pair" id="hoodPairBtn" onclick="startHoodPair()">Haube suchen</button>
      </div>
      <div class="pair-divider"></div>
      <div class="pair-row">
        <div class="pair-info"><h3>Fernbedienung</h3><p>Klicken, dann FB einschalten oder Reset.</p></div>
        <button class="pair-btn fb-pair" id="fbPairBtn" onclick="startFbPair()">FB koppeln</button>
      </div>
    </div>
  </div>
  <div class="status-bar"><span>RAW</span><span class="raw-bytes" id="rawBytes">-- -- -- -- -- -- -- -- --</span></div>
</div>
<div class="toast" id="toast"></div>
<script>
function showToast(m){const t=document.getElementById('toast');t.textContent=m;t.classList.add('show');setTimeout(()=>t.classList.remove('show'),2200);}
function cmd(url){fetch(url).then(r=>r.json()).then(d=>showToast((d.ok?'✓ ':'✗ ')+d.msg)).catch(()=>showToast('✗ Fehler'));}
function setFan(l){cmd('/api/fan?level='+encodeURIComponent(l));}
function toggleLight(w){cmd('/api/light?which='+w);}
function toggleNachlauf(){cmd('/api/nachlauf');}
function powerOff(){cmd('/api/power');}
function moveHood(d){cmd('/api/cover?dir='+d);}
function startHoodPair(){
  const b=document.getElementById('hoodPairBtn');
  b.disabled=true;b.classList.add('searching');b.textContent='Suche...';
  fetch('/api/scan').then(r=>r.json()).then(d=>{
    showToast(d.msg);
    setTimeout(()=>{b.disabled=false;b.classList.remove('searching');b.textContent='Haube suchen';},10000);
  }).catch(()=>{b.disabled=false;b.classList.remove('searching');b.textContent='Haube suchen';});
}
function startFbPair(){
  const b=document.getElementById('fbPairBtn');
  b.disabled=true;b.classList.add('waiting');b.textContent='Warte auf FB...';
  fetch('/api/pair_fb').then(r=>r.json()).then(d=>{
    showToast(d.msg);
    setTimeout(()=>{b.disabled=false;b.classList.remove('waiting');b.textContent='FB koppeln';},15000);
  }).catch(()=>{b.disabled=false;b.classList.remove('waiting');b.textContent='FB koppeln';});
}
function updateUI(s){
  const hb=document.getElementById('hoodBadge'),ht=document.getElementById('hoodText');
  hb.className='ble-badge'+(s.hood_ble?' connected':'');ht.textContent=s.hood_ble?'Verbunden':'Getrennt';
  const fb=document.getElementById('fbBadge'),ft=document.getElementById('fbText');
  fb.className='ble-badge'+(s.fb_ble?' connected':'');ft.textContent=s.fb_ble?'Verbunden':'Getrennt';
  document.getElementById('discHint').className='disc-hint'+(s.hood_ble?'':' show');
  const fm={'Aus':'fan0','Stufe 1':'fan1','Stufe 2':'fan2','Stufe 3':'fan3','Power':'fan4'};
  Object.values(fm).forEach(id=>document.getElementById(id).classList.remove('active'));
  if(fm[s.fan])document.getElementById(fm[s.fan]).classList.add('active');
  document.getElementById('lightUp').className='light-btn'+(s.light_up?' on':'');
  document.getElementById('lightDown').className='light-btn'+(s.light_down?' on':'');
  document.getElementById('nachlaufBtn').className='action-btn'+(s.nachlauf?' nachlauf-on':'');
  const pm={'up':'Oben','down':'Unten','moving up':'▲ fährt\nhoch','moving down':'▼ fährt\nrunter'};
  document.getElementById('coverState').textContent=pm[s.cover_state]||'–';
  document.getElementById('coverUp').className='cover-btn'+(s.cover_state==='moving up'?' active':'');
  document.getElementById('coverDown').className='cover-btn'+(s.cover_state==='moving down'?' active':'');
  document.getElementById('rawBytes').textContent=s.raw||'-- -- -- -- -- -- -- -- --';
}
function poll(){fetch('/api/state').then(r=>r.json()).then(updateUI).catch(()=>{});}
poll();setInterval(poll,2000);
</script></body></html>
)html";

// ---------- Settings Page ----------
static const char HTML_SETTINGS[] PROGMEM = R"html(<!DOCTYPE html>
<html lang="de"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Berbel &middot; Einstellungen</title>)html";

static const char HTML_SETTINGS2[] PROGMEM = R"html(
</head><body>
<header>
  <div class="brand"><h1>Berbel</h1><span>Einstellungen</span></div>
  <nav class="nav">
    <a href="/">Steuerung</a>
    <a href="/settings" class="active">Einstellungen</a>
    <a href="/update">Update</a>
  </nav>
</header>
<div class="container">
  <div id="msg"></div>
  <div class="card">
    <div class="card-label">WLAN</div>
    <label>Netzwerk (SSID)<input type="text" id="wifi_ssid" placeholder="Dein WLAN-Name"></label>
    <label>Passwort<input type="password" id="wifi_pass" placeholder="WLAN-Passwort"></label>
  </div>
  <div class="card">
    <div class="card-label">MQTT</div>
    <label>Server (IP oder Hostname)<input type="text" id="mqtt_host" placeholder="192.168.178.42"></label>
    <label>Port<input type="number" id="mqtt_port" placeholder="1883" value="1883"></label>
    <label>Benutzername<input type="text" id="mqtt_user" placeholder="berbel-mqtt"></label>
    <label>Passwort<input type="password" id="mqtt_pass" placeholder="MQTT-Passwort"></label>
  </div>
  <div class="card">
    <div class="card-label">Berbel Haube</div>
    <label>H&ouml;henverstellung (motorisch)?
      <select id="has_cover">
        <option value="1">Ja &mdash; Haube hat Hochfahren/Herunterfahren</option>
        <option value="0">Nein &mdash; keine H&ouml;henverstellung</option>
      </select>
    </label>
  </div>
  <div class="card">
    <div class="card-label">Aktionen</div>
    <div class="btn-row">
      <button class="btn btn-primary" onclick="saveSettings()">&#10003; Speichern &amp; Neustart</button>
      <button class="btn btn-ghost" onclick="loadSettings()">&#8635; Laden</button>
      <button class="btn btn-danger" onclick="factoryReset()">&#9888; Werksreset</button>
    </div>
  </div>
</div>
<div class="toast" id="toast"></div>
<script>
function showToast(m){const t=document.getElementById('toast');t.textContent=m;t.classList.add('show');setTimeout(()=>t.classList.remove('show'),2200);}
function showMsg(m,type){document.getElementById('msg').innerHTML='<div class="alert alert-'+type+'">'+m+'</div>';}
function loadSettings(){
  fetch('/api/config').then(r=>r.json()).then(d=>{
    document.getElementById('wifi_ssid').value=d.wifi_ssid||'';
    document.getElementById('wifi_pass').value=d.wifi_pass||'';
    document.getElementById('mqtt_host').value=d.mqtt_host||'';
    document.getElementById('mqtt_port').value=d.mqtt_port||1883;
    document.getElementById('mqtt_user').value=d.mqtt_user||'';
    document.getElementById('mqtt_pass').value=d.mqtt_pass||'';
    document.getElementById('has_cover').value=d.has_cover?'1':'0';
  });
}
function saveSettings(){
  const data={
    wifi_ssid:document.getElementById('wifi_ssid').value,
    wifi_pass:document.getElementById('wifi_pass').value,
    mqtt_host:document.getElementById('mqtt_host').value,
    mqtt_port:parseInt(document.getElementById('mqtt_port').value)||1883,
    mqtt_user:document.getElementById('mqtt_user').value,
    mqtt_pass:document.getElementById('mqtt_pass').value,
    has_cover:document.getElementById('has_cover').value==='1'
  };
  fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)})
    .then(r=>r.json()).then(d=>{
      if(d.ok){showMsg('Gespeichert! ESP32 startet neu...','success');}
      else showMsg('Fehler: '+d.msg,'error');
    });
}
function factoryReset(){
  if(!confirm('Alle Einstellungen loeschen und Setup-AP starten?'))return;
  fetch('/api/reset',{method:'POST'}).then(r=>r.json()).then(d=>{
    showMsg('Reset! Verbinde mit "Berbel-Setup" WLAN...','info');
  });
}
loadSettings();
</script></body></html>
)html";

// ---------- OTA Update Page ----------
static const char HTML_UPDATE[] PROGMEM = R"html(<!DOCTYPE html>
<html lang="de"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Berbel &middot; Update</title>)html";

static const char HTML_UPDATE2[] PROGMEM = R"html(
<style>
.progress-wrap{background:var(--surface2);border-radius:8px;height:12px;overflow:hidden;margin:16px 0;}
.progress-bar{height:100%;background:var(--accent);width:0%;transition:width .3s;border-radius:8px;}
#dropzone{border:2px dashed var(--border);border-radius:var(--radius);padding:40px;text-align:center;cursor:pointer;transition:all .2s;color:var(--muted);}
#dropzone:hover,#dropzone.drag{border-color:var(--accent);color:var(--accent);}
#dropzone svg{margin-bottom:12px;opacity:.5;}
</style>
</head><body>
<header>
  <div class="brand"><h1>Berbel</h1><span>OTA Update</span></div>
  <nav class="nav">
    <a href="/">Steuerung</a>
    <a href="/settings">Einstellungen</a>
    <a href="/update" class="active">Update</a>
  </nav>
</header>
<div class="container">
  <div class="card">
    <div class="card-label">Firmware Update</div>
    <div id="dropzone" onclick="document.getElementById('fw').click()">
      <svg width="40" height="40" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5">
        <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/>
        <polyline points="17 8 12 3 7 8"/><line x1="12" y1="3" x2="12" y2="15"/>
      </svg>
      <p style="font-weight:500;margin-bottom:4px;">Firmware-Datei hier ablegen</p>
      <p style="font-size:.8rem;">oder klicken zum Ausw&auml;hlen (.bin)</p>
    </div>
    <input type="file" id="fw" accept=".bin" style="display:none" onchange="startUpdate(this)">
    <div class="progress-wrap" id="progWrap" style="display:none">
      <div class="progress-bar" id="progBar"></div>
    </div>
    <div id="updateMsg"></div>
  </div>
  <div class="card">
    <div class="card-label">Info</div>
    <div id="sysInfo" style="font-family:'DM Mono',monospace;font-size:.75rem;color:var(--muted);line-height:2;"></div>
  </div>
</div>
<div class="toast" id="toast"></div>
<script>
function showToast(m){const t=document.getElementById('toast');t.textContent=m;t.classList.add('show');setTimeout(()=>t.classList.remove('show'),2200);}
const dz=document.getElementById('dropzone');
dz.addEventListener('dragover',e=>{e.preventDefault();dz.classList.add('drag');});
dz.addEventListener('dragleave',()=>dz.classList.remove('drag'));
dz.addEventListener('drop',e=>{e.preventDefault();dz.classList.remove('drag');const f=e.dataTransfer.files[0];if(f)uploadFile(f);});
function startUpdate(input){if(input.files[0])uploadFile(input.files[0]);}
function uploadFile(file){
  if(!file.name.endsWith('.bin')){showToast('Nur .bin Dateien erlaubt');return;}
  const wrap=document.getElementById('progWrap'),bar=document.getElementById('progBar');
  const msg=document.getElementById('updateMsg');
  wrap.style.display='block';bar.style.width='0%';
  msg.innerHTML='<div class="alert alert-info">Upload l&auml;uft... nicht unterbrechen!</div>';
  const xhr=new XMLHttpRequest();
  xhr.upload.onprogress=e=>{if(e.lengthComputable){const p=Math.round(e.loaded/e.total*100);bar.style.width=p+'%';}};
  xhr.onload=()=>{
    if(xhr.status===200){
      bar.style.width='100%';
      msg.innerHTML='<div class="alert alert-success">Update erfolgreich! ESP32 startet neu...</div>';
    } else {
      msg.innerHTML='<div class="alert alert-error">Fehler: '+xhr.responseText+'</div>';
    }
  };
  xhr.onerror=()=>msg.innerHTML='<div class="alert alert-error">Upload fehlgeschlagen</div>';
  xhr.open('POST','/api/update');
  xhr.send(file);
}
fetch('/api/sysinfo').then(r=>r.json()).then(d=>{
  document.getElementById('sysInfo').innerHTML=
    'Chip: '+d.chip+'<br>Flash: '+d.flash+'<br>Heap frei: '+d.heap+' bytes<br>IP: '+d.ip+'<br>Uptime: '+d.uptime+'s';
});
</script></body></html>
)html";

// ---------- Setup AP Page ----------
static const char HTML_SETUP[] PROGMEM = R"html(<!DOCTYPE html>
<html lang="de"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Berbel Setup</title>)html";

static const char HTML_SETUP2[] PROGMEM = R"html(
</head><body>
<header>
  <div class="brand"><h1>Berbel Setup</h1><span>Ersteinrichtung</span></div>
</header>
<div class="container">
  <div class="alert alert-info" style="margin-bottom:0;">
    Willkommen! Bitte WLAN und MQTT konfigurieren um die Berbel-Haube mit Home Assistant zu verbinden.
  </div>
  <div id="msg"></div>
  <div class="card">
    <div class="card-label">WLAN</div>
    <label>Netzwerk (SSID)<input type="text" id="wifi_ssid" placeholder="Dein WLAN-Name"></label>
    <label>Passwort<input type="password" id="wifi_pass" placeholder="WLAN-Passwort"></label>
  </div>
  <div class="card">
    <div class="card-label">MQTT (Home Assistant)</div>
    <label>Server IP<input type="text" id="mqtt_host" placeholder="192.168.178.42"></label>
    <label>Port<input type="number" id="mqtt_port" value="1883"></label>
    <label>Benutzername<input type="text" id="mqtt_user" placeholder="berbel-mqtt"></label>
    <label>Passwort<input type="password" id="mqtt_pass" placeholder="MQTT-Passwort"></label>
  </div>
  <div class="card">
    <div class="card-label">Berbel Haube</div>
    <label>H&ouml;henverstellung (motorisch)?
      <select id="has_cover">
        <option value="1">Ja &mdash; Hochfahren/Herunterfahren vorhanden</option>
        <option value="0">Nein &mdash; keine H&ouml;henverstellung</option>
      </select>
    </label>
  </div>
  <div class="btn-row">
    <button class="btn btn-primary" onclick="save()" style="width:100%;justify-content:center;">
      &#10003;&nbsp; Speichern &amp; Verbinden
    </button>
  </div>
</div>
<div class="toast" id="toast"></div>
<script>
function showToast(m){const t=document.getElementById('toast');t.textContent=m;t.classList.add('show');setTimeout(()=>t.classList.remove('show'),2200);}
function showMsg(m,type){document.getElementById('msg').innerHTML='<div class="alert alert-'+type+'">'+m+'</div>';}
function save(){
  const d={
    wifi_ssid:document.getElementById('wifi_ssid').value.trim(),
    wifi_pass:document.getElementById('wifi_pass').value,
    mqtt_host:document.getElementById('mqtt_host').value.trim(),
    mqtt_port:parseInt(document.getElementById('mqtt_port').value)||1883,
    mqtt_user:document.getElementById('mqtt_user').value.trim(),
    mqtt_pass:document.getElementById('mqtt_pass').value,
    has_cover:document.getElementById('has_cover').value==='1'
  };
  if(!d.wifi_ssid){showMsg('Bitte WLAN-Name eingeben','error');return;}
  if(!d.mqtt_host){showMsg('Bitte MQTT Server eingeben','error');return;}
  fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)})
    .then(r=>r.json()).then(r=>{
      if(r.ok){
        showMsg('Gespeichert! ESP32 verbindet sich mit deinem WLAN...<br>Bitte verbinde dich wieder mit deinem normalen WLAN und &ouml;ffne <b>http://berbel-remote.local</b>','success');
      } else showMsg('Fehler: '+r.msg,'error');
    }).catch(()=>showMsg('Verbindungsfehler','error'));
}
</script></body></html>
)html";

// ============================================================================
// BLE Peripheral (towards FB)
// ============================================================================
void startPeripheralAdvertising() {
  NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
  pAdv->stop();
  NimBLEAdvertisementData advData;
  advData.addData(std::string(
    reinterpret_cast<const char*>(raw_adv_data), sizeof(raw_adv_data)));
  pAdv->setAdvertisementData(advData);
  NimBLEAdvertisementData scanData;
  pAdv->setScanResponseData(scanData);
  pAdv->setMinInterval(0x20);
  pAdv->setMaxInterval(0x40);
  pAdv->start();
  Serial.println("[PERIPHERAL] Advertising as hood (for original FB)");
}

class FBWriteCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar) override {
    std::string value = pChar->getValue();
    if (value.length() == 2) {
      uint8_t code = (uint8_t)value[0];
      if (code != 0x00) {
        char name[16];
        snprintf(name, sizeof(name), "FB->0x%02X", code);
        Serial.printf("[FB] Button: 0x%02X\n", code);
        queueButton(code, name);
      }
    }
  }
};

class PeripheralCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer) override {
    fbConnected = true;
    Serial.println("[PERIPHERAL] FB connected!");
  }
  void onDisconnect(NimBLEServer* pServer) override {
    fbConnected = false;
    Serial.println("[PERIPHERAL] FB disconnected");
    delay(100);
    startPeripheralAdvertising();
  }
};

// ============================================================================
// BLE Central (towards hood)
// ============================================================================
void hoodNotifyCallback(NimBLERemoteCharacteristic* pChar,
    uint8_t* pData, size_t length, bool isNotify) {
  Serial.printf("[HOOD] Status (%d): ", length);
  for (size_t i = 0; i < length; i++) Serial.printf("%02X ", pData[i]);
  Serial.println();
  if (length == 9) {
    memcpy(pendingStatus, pData, 9);
    newStatusReceived = true;
    if (fbConnected && pNotifyChar) {
      pNotifyChar->setValue(pData, length);
      pNotifyChar->notify();
    }
  }
}

class HoodClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pClient) override {
    hoodConnected = true;
    Serial.println("[CENTRAL] Connected to hood!");
  }
  void onDisconnect(NimBLEClient* pClient) override {
    hoodConnected = false;
    pHoodNotifyChar = nullptr;
    pHoodWriteChar  = nullptr;
    Serial.println("[CENTRAL] Hood disconnected");
  }
};

void startHoodScan() {
  if (hoodConnected) return;
  Serial.println("[SCAN] Scanning for Berbel hood (5s)...");
  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setActiveScan(true);
  pScan->setInterval(100);
  pScan->setWindow(99);
  pScan->start(5, false);
  hoodScanStarted = true;
}

bool connectToHood() {
  NimBLEScan* pScan = NimBLEDevice::getScan();
  NimBLEScanResults results = pScan->getResults();

  NimBLEAddress targetAddress;
  bool found = false;
  for (int i = 0; i < results.getCount(); i++) {
    NimBLEAdvertisedDevice device = results.getDevice(i);
    if (device.haveServiceUUID() &&
        device.isAdvertisingService(NimBLEUUID(BERBEL_SERVICE_UUID))) {
      targetAddress = device.getAddress();
      found = true;
      break;
    }
  }
  if (!found) return false;

  Serial.printf("[CENTRAL] Connecting to %s...\n",
    targetAddress.toString().c_str());

  pClient = NimBLEDevice::createClient();
  pClient->setClientCallbacks(new HoodClientCallbacks(), false);
  pClient->setConnectionParams(12, 12, 0, 51);

  if (!pClient->connect(targetAddress)) {
    Serial.println("[CENTRAL] Connection failed");
    NimBLEDevice::deleteClient(pClient);
    pClient = nullptr;
    return false;
  }

  NimBLERemoteService* pService = pClient->getService(BERBEL_SERVICE_UUID);
  if (!pService) {
    Serial.println("[CENTRAL] Service not found");
    pClient->disconnect();
    return false;
  }

  pHoodNotifyChar = pService->getCharacteristic(BERBEL_NOTIFY_UUID);
  if (!pHoodNotifyChar) {
    Serial.println("[CENTRAL] Notify char not found");
    pClient->disconnect();
    return false;
  }

  pHoodWriteChar = pService->getCharacteristic(BERBEL_WRITE_UUID);
  if (pHoodWriteChar && pHoodWriteChar->canNotify()) {
    pHoodWriteChar->subscribe(true, hoodNotifyCallback);
    Serial.println("[CENTRAL] Subscribed to hood status");
  }

  Serial.println("[CENTRAL] Hood ready!");
  hoodConnected = true;
  return true;
}

// ============================================================================
// Button Send / Queue
// ============================================================================
void sendButtonToHood(uint8_t code, const char* name) {
  if (!hoodConnected || !pHoodNotifyChar) return;
  Serial.printf("[BTN] %s (0x%02X)\n", name, code);
  uint8_t press[]   = {code, 0x00};
  uint8_t release[] = {0x00, 0x00};
  pHoodNotifyChar->writeValue(press, 2, false);
  delay(100);
  pHoodNotifyChar->writeValue(release, 2, false);
}

void queueButton(uint8_t code, const char* name) {
  int next = (cmdQueueHead + 1) % CMD_QUEUE_SIZE;
  if (next == cmdQueueTail) { Serial.println("[CMD] Queue full!"); return; }
  cmdQueue[cmdQueueHead].code = code;
  strncpy(cmdQueue[cmdQueueHead].name, name, sizeof(cmdQueue[cmdQueueHead].name)-1);
  cmdQueueHead = next;
}

void processCmdQueue() {
  if (cmdQueueHead == cmdQueueTail) return;
  if (!hoodConnected || !pHoodNotifyChar) return;
  if (millis() - lastCmdSent < CMD_DELAY_MS) return;
  sendButtonToHood(cmdQueue[cmdQueueTail].code, cmdQueue[cmdQueueTail].name);
  cmdQueueTail = (cmdQueueTail + 1) % CMD_QUEUE_SIZE;
  lastCmdSent = millis();
}

// ============================================================================
// MQTT
// ============================================================================
static const char* fanPresetName(uint8_t speed) {
  switch (speed) {
    case 1: return "Stufe 1"; case 2: return "Stufe 2";
    case 3: return "Stufe 3"; case 4: return "Power";
    default: return "Aus";
  }
}

void publishState() {
  if (!mqtt.connected()) return;
  if (!hoodStateValid) {
    char j[96];
    snprintf(j, sizeof(j), "{\"hood_ble\":\"%s\",\"fb_ble\":\"%s\"}",
      hood.hoodBleConnected?"ON":"OFF", hood.fbBleConnected?"ON":"OFF");
    mqtt.publish(MQTT_STATE, j, true);
    return;
  }
  char j[512];
  snprintf(j, sizeof(j),
    "{\"light_up\":\"%s\",\"light_down\":\"%s\",\"fan_preset\":\"%s\","
    "\"nachlauf\":\"%s\",\"position\":\"%s\",\"cover_state\":\"%s\","
    "\"hood_ble\":\"%s\",\"fb_ble\":\"%s\","
    "\"status_raw\":\"%02X %02X %02X %02X %02X %02X %02X %02X %02X\"}",
    hood.lightUp?"ON":"OFF", hood.lightDown?"ON":"OFF",
    fanPresetName(hood.fanSpeed), hood.nachlauf?"ON":"OFF",
    hood.position, hood.coverState,
    hood.hoodBleConnected?"ON":"OFF", hood.fbBleConnected?"ON":"OFF",
    hood.raw[0],hood.raw[1],hood.raw[2],hood.raw[3],hood.raw[4],
    hood.raw[5],hood.raw[6],hood.raw[7],hood.raw[8]);
  mqtt.publish(MQTT_STATE, j, true);
}

static const char DISC_DEV[] =
  ",\"avty_t\":\"" MQTT_AVAIL "\""
  ",\"dev\":{\"ids\":[\"berbel_hood\"],\"name\":\"Berbel Hood\","
  "\"mf\":\"Berbel\",\"mdl\":\"BFB 6bT\"}}";

void pubDisc(const char* topic, const char* f) {
  char buf[768];
  snprintf(buf, sizeof(buf), "{%s%s", f, DISC_DEV);
  mqtt.publish(topic, buf, true);
  delay(50);
}

void publishDiscovery() {
  pubDisc("homeassistant/light/berbel_hood/light_up/config",
    "\"name\":\"Oberlicht\",\"uniq_id\":\"berbel_light_up\","
    "\"stat_t\":\"" MQTT_STATE "\",\"cmd_t\":\"" MQTT_CMD_LIGHT_UP "\","
    "\"stat_val_tpl\":\"{{ value_json.light_up }}\",\"ic\":\"mdi:ceiling-light\"");
  pubDisc("homeassistant/light/berbel_hood/light_down/config",
    "\"name\":\"Unterlicht\",\"uniq_id\":\"berbel_light_down\","
    "\"stat_t\":\"" MQTT_STATE "\",\"cmd_t\":\"" MQTT_CMD_LIGHT_DOWN "\","
    "\"stat_val_tpl\":\"{{ value_json.light_down }}\",\"ic\":\"mdi:desk-lamp\"");
  pubDisc("homeassistant/select/berbel_hood/fan/config",
    "\"name\":\"L\\u00fcfter\",\"uniq_id\":\"berbel_fan\","
    "\"stat_t\":\"" MQTT_STATE "\",\"val_tpl\":\"{{ value_json.fan_preset }}\","
    "\"cmd_t\":\"" MQTT_CMD_FAN_PRESET "\","
    "\"ops\":[\"Aus\",\"Stufe 1\",\"Stufe 2\",\"Stufe 3\",\"Power\"],\"ic\":\"mdi:fan\"");
  pubDisc("homeassistant/select/berbel_hood/position/config",
    "\"name\":\"Position\",\"uniq_id\":\"berbel_position\","
    "\"stat_t\":\"" MQTT_STATE "\",\"val_tpl\":\"{{ value_json.position }}\","
    "\"cmd_t\":\"" MQTT_CMD_POSITION "\","
    "\"ops\":[\"Oben\",\"Unten\"],\"ic\":\"mdi:arrow-up-down\"");
  pubDisc("homeassistant/button/berbel_hood/power/config",
    "\"name\":\"Ausschalten\",\"uniq_id\":\"berbel_power\","
    "\"cmd_t\":\"" MQTT_CMD_POWER "\",\"ic\":\"mdi:power\"");
  pubDisc("homeassistant/switch/berbel_hood/nachlauf/config",
    "\"name\":\"Nachlauf\",\"uniq_id\":\"berbel_nachlauf\","
    "\"stat_t\":\"" MQTT_STATE "\",\"val_tpl\":\"{{ value_json.nachlauf }}\","
    "\"cmd_t\":\"" MQTT_CMD_NACHLAUF "\",\"ic\":\"mdi:timer-sand\"");
  pubDisc("homeassistant/button/berbel_hood/move_up/config",
    "\"name\":\"Hochfahren\",\"uniq_id\":\"berbel_move_up\","
    "\"cmd_t\":\"" MQTT_CMD_MOVE_UP "\",\"ic\":\"mdi:arrow-up\"");
  pubDisc("homeassistant/button/berbel_hood/move_down/config",
    "\"name\":\"Herunterfahren\",\"uniq_id\":\"berbel_move_down\","
    "\"cmd_t\":\"" MQTT_CMD_MOVE_DOWN "\",\"ic\":\"mdi:arrow-down\"");
  pubDisc("homeassistant/binary_sensor/berbel_hood/hood_ble/config",
    "\"name\":\"BLE Haube\",\"uniq_id\":\"berbel_hood_ble\","
    "\"stat_t\":\"" MQTT_STATE "\",\"val_tpl\":\"{{ value_json.hood_ble }}\","
    "\"dev_cla\":\"connectivity\",\"ent_cat\":\"diagnostic\"");
  pubDisc("homeassistant/binary_sensor/berbel_hood/fb_ble/config",
    "\"name\":\"BLE Fernbedienung\",\"uniq_id\":\"berbel_fb_ble\","
    "\"stat_t\":\"" MQTT_STATE "\",\"val_tpl\":\"{{ value_json.fb_ble }}\","
    "\"dev_cla\":\"connectivity\",\"ent_cat\":\"diagnostic\"");
  Serial.println("[MQTT] Discovery published");
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char msg[128];
  unsigned int l = length < sizeof(msg)-1 ? length : sizeof(msg)-1;
  memcpy(msg, payload, l); msg[l] = '\0';
  String t(topic);
  if      (t==MQTT_CMD_LIGHT_UP)   { if((strcmp(msg,"ON")==0)!=hood.lightUp)   queueButton(BTN_LIGHT_UP,"Light Up"); }
  else if (t==MQTT_CMD_LIGHT_DOWN) { if((strcmp(msg,"ON")==0)!=hood.lightDown) queueButton(BTN_LIGHT_DOWN,"Light Down"); }
  else if (t==MQTT_CMD_POWER)      queueButton(BTN_POWER,"Power Off");
  else if (t==MQTT_CMD_NACHLAUF)   { if((strcmp(msg,"ON")==0)!=hood.nachlauf)  queueButton(BTN_TIMER,"Timer"); }
  else if (t==MQTT_CMD_FAN_PRESET) {
    uint8_t tgt=0,btn=BTN_POWER; const char* n="Fan Off";
    if      (strcmp(msg,"Stufe 1")==0){tgt=1;btn=BTN_FAN_1;n="Fan 1";}
    else if (strcmp(msg,"Stufe 2")==0){tgt=2;btn=BTN_FAN_2;n="Fan 2";}
    else if (strcmp(msg,"Stufe 3")==0){tgt=3;btn=BTN_FAN_3;n="Fan 3";}
    else if (strcmp(msg,"Power")==0)  {tgt=4;btn=BTN_FAN_P;n="Fan Power";}
    if(tgt!=hood.fanSpeed) queueButton(btn,n);
  }
  else if (t==MQTT_CMD_POSITION)  { queueButton(strcmp(msg,"Oben")==0?BTN_MOVE_UP:BTN_MOVE_DOWN,"Position"); }
  else if (t==MQTT_CMD_MOVE_UP)   queueButton(BTN_MOVE_UP,"Move Up");
  else if (t==MQTT_CMD_MOVE_DOWN) queueButton(BTN_MOVE_DOWN,"Move Down");
  else if (t=="homeassistant/status" && strcmp(msg,"online")==0) { publishDiscovery(); publishState(); }
}

void mqttReconnect() {
  if (WiFi.status()!=WL_CONNECTED || mqtt.connected()) return;
  if (millis()-lastMqttReconnect < 5000) return;
  lastMqttReconnect = millis();
  Serial.printf("[MQTT] Connecting %s:%d...\n", cfg.mqtt_host, cfg.mqtt_port);
  if (mqtt.connect("berbel-remote", cfg.mqtt_user, cfg.mqtt_pass,
                   MQTT_AVAIL, 0, true, "offline")) {
    mqtt.publish(MQTT_AVAIL,"online",true);
    mqtt.subscribe(MQTT_CMD_LIGHT_UP); mqtt.subscribe(MQTT_CMD_LIGHT_DOWN);
    mqtt.subscribe(MQTT_CMD_POWER);    mqtt.subscribe(MQTT_CMD_NACHLAUF);
    mqtt.subscribe(MQTT_CMD_FAN_PRESET); mqtt.subscribe(MQTT_CMD_POSITION);
    mqtt.subscribe(MQTT_CMD_MOVE_UP);  mqtt.subscribe(MQTT_CMD_MOVE_DOWN);
    mqtt.subscribe("homeassistant/status");
    publishDiscovery(); publishState();
    Serial.println("[MQTT] Connected!");
  } else {
    Serial.printf("[MQTT] Failed rc=%d\n", mqtt.state());
  }
}

// ============================================================================
// JSON helpers
// ============================================================================
String buildStateJson() {
  char j[320];
  snprintf(j, sizeof(j),
    "{\"hood_ble\":%s,\"fb_ble\":%s,\"fan\":\"%s\","
    "\"light_up\":%s,\"light_down\":%s,\"nachlauf\":%s,"
    "\"position\":\"%s\",\"cover_state\":\"%s\","
    "\"raw\":\"%02X %02X %02X %02X %02X %02X %02X %02X %02X\"}",
    hood.hoodBleConnected?"true":"false",
    hood.fbBleConnected?"true":"false",
    fanPresetName(hood.fanSpeed),
    hood.lightUp?"true":"false", hood.lightDown?"true":"false",
    hood.nachlauf?"true":"false",
    hood.position, hood.coverState,
    hood.raw[0],hood.raw[1],hood.raw[2],hood.raw[3],hood.raw[4],
    hood.raw[5],hood.raw[6],hood.raw[7],hood.raw[8]);
  return String(j);
}

// Simple JSON string extractor
bool jsonStr(const String& json, const char* key, char* out, size_t len) {
  String k = "\""; k+=key; k+="\":\"";
  int s = json.indexOf(k);
  if (s<0) return false;
  s += k.length();
  int e = json.indexOf('"', s);
  if (e<0) return false;
  String v = json.substring(s, e);
  strncpy(out, v.c_str(), len-1); out[len-1]='\0';
  return true;
}

bool jsonInt(const String& json, const char* key, int& out) {
  String k = "\""; k+=key; k+="\":";
  int s = json.indexOf(k);
  if (s<0) return false;
  s += k.length();
  out = json.substring(s).toInt();
  return true;
}

bool jsonBool(const String& json, const char* key) {
  String k = "\""; k+=key; k+="\":true";
  return json.indexOf(k) >= 0;
}

// ============================================================================
// Web Server — Normal Mode
// ============================================================================
void setupWebServer() {
  // Main page
  webServer.on("/", HTTP_GET, []() {
    String page = String(FPSTR(HTML_MAIN));
    page += String(FPSTR(CSS_COMMON));
    page += String(FPSTR(HTML_MAIN2));
    webServer.send(200, "text/html", page);
  });

  // Settings page
  webServer.on("/settings", HTTP_GET, []() {
    String page = String(FPSTR(HTML_SETTINGS));
    page += String(FPSTR(CSS_COMMON));
    page += String(FPSTR(HTML_SETTINGS2));
    webServer.send(200, "text/html", page);
  });

  // OTA Update page
  webServer.on("/update", HTTP_GET, []() {
    String page = String(FPSTR(HTML_UPDATE));
    page += String(FPSTR(CSS_COMMON));
    page += String(FPSTR(HTML_UPDATE2));
    webServer.send(200, "text/html", page);
  });

  // API: state
  webServer.on("/api/state", HTTP_GET, []() {
    webServer.sendHeader("Access-Control-Allow-Origin","*");
    webServer.send(200, "application/json", buildStateJson());
  });

  // API: config GET
  webServer.on("/api/config", HTTP_GET, []() {
    char j[320];
    snprintf(j, sizeof(j),
      "{\"wifi_ssid\":\"%s\",\"wifi_pass\":\"%s\","
      "\"mqtt_host\":\"%s\",\"mqtt_port\":%d,"
      "\"mqtt_user\":\"%s\",\"mqtt_pass\":\"%s\","
      "\"has_cover\":%s}",
      cfg.wifi_ssid, "********",
      cfg.mqtt_host, cfg.mqtt_port,
      cfg.mqtt_user, "********",
      cfg.hood_has_cover?"true":"false");
    webServer.send(200, "application/json", j);
  });

  // API: config POST (save + restart)
  webServer.on("/api/config", HTTP_POST, []() {
    String body = webServer.arg("plain");
    char tmp[64]; int ival;
    if (jsonStr(body,"wifi_ssid",tmp,sizeof(tmp))) strncpy(cfg.wifi_ssid,tmp,sizeof(cfg.wifi_ssid));
    if (jsonStr(body,"wifi_pass",tmp,sizeof(tmp)) && strcmp(tmp,"********")!=0)
      strncpy(cfg.wifi_pass,tmp,sizeof(cfg.wifi_pass));
    if (jsonStr(body,"mqtt_host",tmp,sizeof(tmp))) strncpy(cfg.mqtt_host,tmp,sizeof(cfg.mqtt_host));
    if (jsonInt(body,"mqtt_port",ival)) cfg.mqtt_port = ival;
    if (jsonStr(body,"mqtt_user",tmp,sizeof(tmp))) strncpy(cfg.mqtt_user,tmp,sizeof(cfg.mqtt_user));
    if (jsonStr(body,"mqtt_pass",tmp,sizeof(tmp)) && strcmp(tmp,"********")!=0)
      strncpy(cfg.mqtt_pass,tmp,sizeof(cfg.mqtt_pass));
    cfg.hood_has_cover = jsonBool(body,"has_cover");
    saveConfig();
    webServer.send(200,"application/json","{\"ok\":true,\"msg\":\"Gespeichert\"}");
    delay(500);
    ESP.restart();
  });

  // API: factory reset
  webServer.on("/api/reset", HTTP_POST, []() {
    webServer.send(200,"application/json","{\"ok\":true,\"msg\":\"Reset\"}");
    delay(300);
    clearConfig();
    ESP.restart();
  });

  // API: system info
  webServer.on("/api/sysinfo", HTTP_GET, []() {
    char j[256];
    snprintf(j, sizeof(j),
      "{\"chip\":\"ESP32-S3\",\"flash\":\"%dMB\","
      "\"heap\":%u,\"ip\":\"%s\",\"uptime\":%lu}",
      16, esp_get_free_heap_size(),
      WiFi.localIP().toString().c_str(),
      millis()/1000);
    webServer.send(200,"application/json",j);
  });

  // API: OTA firmware upload
  webServer.on("/api/update", HTTP_POST,
    []() {
      webServer.send(Update.hasError() ? 500 : 200, "text/plain",
        Update.hasError() ? "Update fehlgeschlagen" : "OK");
      delay(500);
      ESP.restart();
    },
    []() {
      HTTPUpload& upload = webServer.upload();
      if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("[OTA] Update: %s\n", upload.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
          Serial.printf("[OTA] Update OK: %u bytes\n", upload.totalSize);
        } else {
          Update.printError(Serial);
        }
      }
    }
  );

  // API: fan
  webServer.on("/api/fan", HTTP_GET, []() {
    String level = webServer.arg("level");
    uint8_t btn=BTN_POWER,tgt=0; const char* n="Fan Off";
    if(level=="Stufe 1"){tgt=1;btn=BTN_FAN_1;n="Fan 1";}
    else if(level=="Stufe 2"){tgt=2;btn=BTN_FAN_2;n="Fan 2";}
    else if(level=="Stufe 3"){tgt=3;btn=BTN_FAN_3;n="Fan 3";}
    else if(level=="Power"){tgt=4;btn=BTN_FAN_P;n="Fan Power";}
    if(tgt==hood.fanSpeed){webServer.send(200,"application/json","{\"ok\":true,\"msg\":\"Bereits aktiv\"}");return;}
    queueButton(btn,n);
    webServer.send(200,"application/json","{\"ok\":true,\"msg\":\"L\\u00fcfter gesetzt\"}");
  });

  // API: light
  webServer.on("/api/light", HTTP_GET, []() {
    String w = webServer.arg("which");
    if(w=="up") queueButton(BTN_LIGHT_UP,"Light Up");
    else        queueButton(BTN_LIGHT_DOWN,"Light Down");
    webServer.send(200,"application/json","{\"ok\":true,\"msg\":\"Licht umgeschaltet\"}");
  });

  // API: nachlauf
  webServer.on("/api/nachlauf", HTTP_GET, []() {
    queueButton(BTN_TIMER,"Nachlauf");
    webServer.send(200,"application/json","{\"ok\":true,\"msg\":\"Nachlauf umgeschaltet\"}");
  });

  // API: power
  webServer.on("/api/power", HTTP_GET, []() {
    queueButton(BTN_POWER,"Power Off");
    webServer.send(200,"application/json","{\"ok\":true,\"msg\":\"Ausschalten\"}");
  });

  // API: cover
  webServer.on("/api/cover", HTTP_GET, []() {
    String d = webServer.arg("dir");
    queueButton(d=="up"?BTN_MOVE_UP:BTN_MOVE_DOWN,"Cover");
    webServer.send(200,"application/json","{\"ok\":true,\"msg\":\"Position gesendet\"}");
  });

  // API: scan for hood
  webServer.on("/api/scan", HTTP_GET, []() {
    if(hoodConnected){webServer.send(200,"application/json","{\"ok\":true,\"msg\":\"Haube bereits verbunden\"}");return;}
    startHoodScan();
    webServer.send(200,"application/json","{\"ok\":true,\"msg\":\"Suche gestartet...\"}");
  });

  // API: pair FB
  webServer.on("/api/pair_fb", HTTP_GET, []() {
    if(fbConnected){webServer.send(200,"application/json","{\"ok\":true,\"msg\":\"FB bereits verbunden\"}");return;}
    NimBLEDevice::deleteAllBonds();
    startPeripheralAdvertising();
    webServer.send(200,"application/json","{\"ok\":true,\"msg\":\"Bereit fuer FB\"}");
  });

  webServer.onNotFound([]() {
    webServer.send(404,"application/json","{\"ok\":false,\"msg\":\"Not found\"}");
  });

  webServer.begin();
  Serial.printf("[WEB] http://berbel-remote.local  (IP: %s)\n",
    WiFi.localIP().toString().c_str());
}

// ============================================================================
// Web Server — Setup AP Mode
// ============================================================================
void setupWebServerAP() {
  webServer.on("/", HTTP_GET, []() {
    String page = String(FPSTR(HTML_SETUP));
    page += String(FPSTR(CSS_COMMON));
    page += String(FPSTR(HTML_SETUP2));
    webServer.send(200, "text/html", page);
  });

  // Captive portal redirects
  webServer.on("/generate_204", HTTP_GET, []() { webServer.sendHeader("Location","/"); webServer.send(302); });
  webServer.on("/fwlink", HTTP_GET, []() { webServer.sendHeader("Location","/"); webServer.send(302); });

  webServer.on("/api/config", HTTP_POST, []() {
    String body = webServer.arg("plain");
    char tmp[64]; int ival;
    if (jsonStr(body,"wifi_ssid",tmp,sizeof(tmp))) strncpy(cfg.wifi_ssid,tmp,sizeof(cfg.wifi_ssid));
    if (jsonStr(body,"wifi_pass",tmp,sizeof(tmp))) strncpy(cfg.wifi_pass,tmp,sizeof(cfg.wifi_pass));
    if (jsonStr(body,"mqtt_host",tmp,sizeof(tmp))) strncpy(cfg.mqtt_host,tmp,sizeof(cfg.mqtt_host));
    if (jsonInt(body,"mqtt_port",ival)) cfg.mqtt_port = ival;
    if (jsonStr(body,"mqtt_user",tmp,sizeof(tmp))) strncpy(cfg.mqtt_user,tmp,sizeof(cfg.mqtt_user));
    if (jsonStr(body,"mqtt_pass",tmp,sizeof(tmp))) strncpy(cfg.mqtt_pass,tmp,sizeof(cfg.mqtt_pass));
    cfg.hood_has_cover = jsonBool(body,"has_cover");
    saveConfig();
    webServer.send(200,"application/json","{\"ok\":true,\"msg\":\"Gespeichert\"}");
    delay(1000);
    ESP.restart();
  });

  webServer.onNotFound([]() {
    webServer.sendHeader("Location", "/");
    webServer.send(302);
  });

  webServer.begin();
  Serial.println("[WEB-AP] Setup server started: http://192.168.4.1");
}

// ============================================================================
// BLE Init (Peripheral side)
// ============================================================================
void initBLEPeripheral() {
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new PeripheralCallbacks());

  NimBLEService* pDev = pServer->createService(NimBLEUUID((uint16_t)0x180A));
  auto* pMfr = pDev->createCharacteristic(NimBLEUUID((uint16_t)0x2A29), NIMBLE_PROPERTY::READ);
  pMfr->setValue("Berbel");
  pDev->start();

  NimBLEService* pBat = pServer->createService(NimBLEUUID((uint16_t)0x180F));
  auto* pBatLvl = pBat->createCharacteristic(
    NimBLEUUID((uint16_t)0x2A19), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  uint8_t bat=90; pBatLvl->setValue(&bat,1);
  pBat->start();

  NimBLEService* pHid = pServer->createService(NimBLEUUID((uint16_t)0x1812));
  auto* pHidInfo = pHid->createCharacteristic(NimBLEUUID((uint16_t)0x2A4A), NIMBLE_PROPERTY::READ);
  uint8_t hi[]={0x11,0x01,0x00,0x01}; pHidInfo->setValue(hi,4);
  pHid->createCharacteristic(NimBLEUUID((uint16_t)0x2A4C), NIMBLE_PROPERTY::WRITE_NR);
  auto* pProt = pHid->createCharacteristic(NimBLEUUID((uint16_t)0x2A4E),
    NIMBLE_PROPERTY::READ|NIMBLE_PROPERTY::WRITE_NR);
  uint8_t pm=0x01; pProt->setValue(&pm,1);
  auto* pRmap = pHid->createCharacteristic(NimBLEUUID((uint16_t)0x2A4B), NIMBLE_PROPERTY::READ);
  const uint8_t rm[]={0x05,0x0C,0x09,0x01,0xA1,0x01,0x85,0x01,
                      0x09,0xE0,0x15,0xE8,0x25,0x18,0x75,0x08,0x95,0x01,0x81,0x06,0xC0};
  pRmap->setValue(rm,sizeof(rm));
  auto* pRep = pHid->createCharacteristic(NimBLEUUID((uint16_t)0x2A4D),
    NIMBLE_PROPERTY::READ|NIMBLE_PROPERTY::NOTIFY);
  auto* pRRef = pRep->createDescriptor(NimBLEUUID((uint16_t)0x2908), NIMBLE_PROPERTY::READ);
  uint8_t rr[]={0x01,0x01}; pRRef->setValue(rr,2);
  pHid->start();

  NimBLEService* pBerbel = pServer->createService(BERBEL_SERVICE_UUID);
  pNotifyChar = pBerbel->createCharacteristic(
    BERBEL_NOTIFY_UUID,
    NIMBLE_PROPERTY::READ|NIMBLE_PROPERTY::NOTIFY|NIMBLE_PROPERTY::WRITE_NR);
  pNotifyChar->setCallbacks(new FBWriteCallbacks());
  uint8_t iv[]={0x00,0x00}; pNotifyChar->setValue(iv,2);
  auto* pWr = pBerbel->createCharacteristic(
    BERBEL_WRITE_UUID,
    NIMBLE_PROPERTY::READ|NIMBLE_PROPERTY::NOTIFY|NIMBLE_PROPERTY::WRITE_NR);
  pWr->setValue(iv,2);
  pBerbel->start();

  startPeripheralAdvertising();
}

// ============================================================================
// Hood status processing
// ============================================================================
void processHoodStatus(const uint8_t* data, size_t len) {
  if (len!=9) return;
  bool isSync=true;
  for (int i=0;i<9;i++) if(data[i]!=0x11){isSync=false;break;}
  if (isSync) { Serial.println("[HOOD] Sync ignored"); return; }

  hoodStateValid=true;
  memcpy(hood.raw,data,9);
  hood.lightUp   = (hood.raw[2]&0x10);
  hood.lightDown = (hood.raw[4]&0x10);
  if      (hood.raw[2]&0x09) hood.fanSpeed=4;
  else if (hood.raw[1]&0x10) hood.fanSpeed=3;
  else if (hood.raw[1]&0x01) hood.fanSpeed=2;
  else if (hood.raw[0]&0x10) hood.fanSpeed=1;
  else                       hood.fanSpeed=0;
  hood.nachlauf=(hood.raw[5]&0x90);
  if      (hood.raw[4]&0x01){hood.coverState="moving up";  hood.position="Oben";}
  else if (hood.raw[6]&0x01){hood.coverState="moving down";hood.position="Unten";}
  else if (strcmp(hood.coverState,"moving up")==0)   hood.coverState="up";
  else if (strcmp(hood.coverState,"moving down")==0) hood.coverState="down";
  publishState();
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.println("\n========================================");
  Serial.println("  BERBEL BRIDGE v2.0 - ESP32-S3 N16R8");
  Serial.println("========================================\n");

  // Load config from NVS
  loadConfig();

  // BLE init (MAC must be set before WiFi)
  Serial.println("[MAC] Setting TI OUI...");
  uint8_t ti_mac[6]={0x88,0x01,0xF9,0xAA,0xBB,0xCC};
  esp_base_mac_addr_set(ti_mac);

  NimBLEDevice::init("berbel-remote");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setSecurityAuth(true, false, true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  NimBLEDevice::setSecurityInitKey(BLE_SM_PAIR_KEY_DIST_ENC);
  NimBLEDevice::setSecurityRespKey(BLE_SM_PAIR_KEY_DIST_ENC);
  esp_coex_preference_set(ESP_COEX_PREFER_BT);

  initBLEPeripheral();
  Serial.println("[BLE] Peripheral ready");

  // Check if WiFi is configured
  if (strlen(cfg.wifi_ssid) == 0) {
    // ---- SETUP MODE: start AP ----
    setupMode = true;
    Serial.println("\n[SETUP] No WiFi configured!");
    Serial.printf("[SETUP] Starting AP: %s\n", SETUP_AP_SSID);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(SETUP_AP_SSID);
    delay(100);

    // Captive portal DNS
    dnsServer.start(53, "*", WiFi.softAPIP());

    setupWebServerAP();

    Serial.printf("[SETUP] Connect to '%s' and open http://%s\n",
      SETUP_AP_SSID, SETUP_AP_IP_STR);
    Serial.println("[SETUP] Waiting for configuration...");
  } else {
    // ---- NORMAL MODE: connect to WiFi ----
    setupMode = false;
    Serial.printf("[WiFi] Connecting to '%s'...\n", cfg.wifi_ssid);
    WiFi.mode(WIFI_STA);
    WiFi.setHostname("berbel-remote");
    WiFi.begin(cfg.wifi_ssid, cfg.wifi_pass);

    int attempts = 0;
    while (WiFi.status()!=WL_CONNECTED && attempts<20) {
      delay(500); Serial.print("."); attempts++;
    }

    if (WiFi.status()==WL_CONNECTED) {
      Serial.printf("\n[WiFi] Connected! IP: %s\n",
        WiFi.localIP().toString().c_str());

      // MQTT
      mqtt.setServer(cfg.mqtt_host, cfg.mqtt_port);
      mqtt.setBufferSize(1024);
      mqtt.setCallback(mqttCallback);

      // OTA
      ArduinoOTA.setHostname("berbel-remote");
      ArduinoOTA.onStart([](){esp_coex_preference_set(ESP_COEX_PREFER_WIFI);});
      ArduinoOTA.onEnd([](){esp_coex_preference_set(ESP_COEX_PREFER_BT);});
      ArduinoOTA.onProgress([](unsigned int p,unsigned int t){
        Serial.printf("[OTA] %u%%\r",p*100/t);});
      ArduinoOTA.begin();

      setupWebServer();
      wifiStarted = true;
      startHoodScan();

      Serial.println("\n========================================");
      Serial.println("  Ready!");
      Serial.printf ("  Web:  http://berbel-remote.local\n");
      Serial.printf ("  IP:   http://%s\n", WiFi.localIP().toString().c_str());
      Serial.println("========================================\n");
    } else {
      // WiFi failed — fallback to setup AP
      Serial.println("\n[WiFi] Failed! Starting setup AP...");
      setupMode = true;
      WiFi.mode(WIFI_AP);
      WiFi.softAP(SETUP_AP_SSID);
      dnsServer.start(53, "*", WiFi.softAPIP());
      setupWebServerAP();
    }
  }
}

// ============================================================================
// Loop
// ============================================================================
void loop() {
  static unsigned long lastHeap = 0;
  unsigned long now = millis();

  // Setup mode: only handle DNS + webserver
  if (setupMode) {
    dnsServer.processNextRequest();
    webServer.handleClient();
    // LED fast blink in setup mode
    static unsigned long lastLed = 0;
    if (now-lastLed > 200) { lastLed=now; digitalWrite(LED_PIN,!digitalRead(LED_PIN)); }
    return;
  }

  // Heap log
  if (now-lastHeap > 30000) {
    lastHeap=now;
    Serial.printf("[SYS] Heap:%u Hood:%s FB:%s WiFi:%s\n",
      esp_get_free_heap_size(),
      hoodConnected?"on":"off", fbConnected?"on":"off",
      WiFi.status()==WL_CONNECTED?"on":"off");
  }

  // WiFi + services
  if (WiFi.status()==WL_CONNECTED) {
    ArduinoOTA.handle();
    webServer.handleClient();
    if (!mqtt.connected()) mqttReconnect();
    else mqtt.loop();
  } else {
    static unsigned long lastRetry=0;
    if (now-lastRetry > 30000) {
      lastRetry=now;
      Serial.println("[WiFi] Reconnecting...");
      WiFi.reconnect();
    }
  }

  // Hood reconnect every 15s
  if (!hoodConnected && now-lastHoodReconnect > 15000) {
    lastHoodReconnect=now;
    NimBLEScan* pScan = NimBLEDevice::getScan();
    if (!pScan->isScanning()) startHoodScan();
  }
  if (!hoodConnected) {
    NimBLEScan* pScan = NimBLEDevice::getScan();
    if (!pScan->isScanning() && pScan->getResults().getCount()>0) {
      connectToHood();
      pScan->clearResults();
    }
  }

  // LED
  {
    static unsigned long lastLed=0;
    uint16_t iv = hoodConnected?(fbConnected?0:1000):300;
    if (iv==0) { digitalWrite(LED_PIN,LOW); }
    else if (now-lastLed>=iv) { lastLed=now; digitalWrite(LED_PIN,!digitalRead(LED_PIN)); }
  }

  // BLE state changes
  if (hoodConnected!=hoodConnectedOld) {
    hood.hoodBleConnected=hoodConnected;
    if(!hoodConnected) cmdQueueHead=cmdQueueTail=0;
    publishState(); hoodConnectedOld=hoodConnected;
  }
  if (fbConnected!=fbConnectedOld) {
    hood.fbBleConnected=fbConnected;
    publishState(); fbConnectedOld=fbConnected;
  }

  // Hood status
  if (newStatusReceived) {
    newStatusReceived=false;
    processHoodStatus(pendingStatus,9);
  }

  processCmdQueue();
  delay(10);
}
