/**
 * ============================================================================
 * Berbel BFB 6bT - Remote Control Emulator + Home Assistant Bridge
 * ============================================================================
 *
 * Emulates a Berbel kitchen hood remote control using an ESP32, with
 * WiFi/MQTT bridge to Home Assistant using MQTT Auto-Discovery.
 * Uses NimBLE stack for ~100KB heap savings over Arduino BLE (Bluedroid).
 *
 * BLE Protocol (reverse engineered):
 * - Button press:   [code, 0x00] as 2-byte notification on f004f002
 * - Button release: [0x00, 0x00] as 2-byte notification on f004f002
 * - Hood status:    9-byte writes from hood on f004f001
 * - Sync packet:    all 0x11 (ignored, sent on connect)
 *
 * Hood Status Bytes (9 bytes, bitmask-based):
 *   Byte[0] & 0x10  = Fan Stufe 1
 *   Byte[1] & 0x01  = Fan Stufe 2
 *   Byte[1] & 0x10  = Fan Stufe 3
 *   Byte[2] & 0x09  = Fan Power
 *   Byte[2] & 0x10  = Oberlicht (upper light)
 *   Byte[4] & 0x10  = Unterlicht (lower light)
 *   Byte[4] & 0x01  = Cover moving up (retracting)
 *   Byte[5] & 0x90  = Nachlauf (afterrun timer active)
 *   Byte[6] & 0x01  = Cover moving down (deploying)
 *
 * HA Entities (via MQTT Auto-Discovery):
 *   - Oberlicht       (light)          Toggle upper light
 *   - Unterlicht      (light)          Toggle lower light
 *   - Luefter         (select)         Fan speed: Aus, Stufe 1-3, Power
 *   - Ausschalten     (button)         Power off (starts Nachlauf)
 *   - Nachlauf        (switch)         Toggle afterrun timer
 *   - Position        (select)         Oben/Unten (only with HOOD_HAS_COVER)
 *   - Hochfahren      (button)         Move up unconditionally (only with HOOD_HAS_COVER)
 *   - Herunterfahren  (button)         Move down unconditionally (only with HOOD_HAS_COVER)
 *   - BLE Verbindung  (binary_sensor)  BLE connection status
 *   - Cover State     (sensor)         Diagnostic: up/moving up/moving down/down (only with HOOD_HAS_COVER)
 *   - Status Raw      (sensor)         Raw 9-byte hex for debugging
 *
 * Critical requirements:
 * 1. MAC must use Texas Instruments OUI (88:01:F9 or 30:AF:7E)
 * 2. Advertising must include Service Data with UUID f000f000-...-berbel
 * 3. GATT services in correct order (DevInfo, Battery, HID, Berbel)
 * 4. Legacy Pairing, LTK only (no IRK)
 * 5. BLE must have radio priority (esp_coex_preference_set)
 *
 * ============================================================================
 */

#include <NimBLEDevice.h>
#include <esp_mac.h>
#include <esp_coexist.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>

#include "config.h"

// ============================================================================
// Berbel Custom Service UUIDs
// ============================================================================
#define BERBEL_SERVICE_UUID   "f004f000-5745-4053-8043-62657262656c"
#define BERBEL_NOTIFY_UUID    "f004f002-5745-4053-8043-62657262656c"
#define BERBEL_WRITE_UUID     "f004f001-5745-4053-8043-62657262656c"

// ============================================================================
// Button Codes
// ============================================================================
#define BTN_POWER       0x01
#define BTN_FAN_1       0x02
#define BTN_FAN_2       0x03
#define BTN_FAN_3       0x04
#define BTN_FAN_P       0x05
#define BTN_LIGHT_UP    0x06
#define BTN_PLAY        0x07
#define BTN_RELOAD      0x08
#define BTN_MOVE_UP     0x09
#define BTN_LIGHT_DOWN  0x0A
#define BTN_RECORD      0x0B
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
#if HOOD_HAS_COVER
#define MQTT_CMD_POSITION   MQTT_BASE "/position/set"
#define MQTT_CMD_MOVE_UP    MQTT_BASE "/move_up/set"
#define MQTT_CMD_MOVE_DOWN  MQTT_BASE "/move_down/set"
#endif
#define MQTT_CMD_DEBUG      MQTT_BASE "/debug/send"

// ============================================================================
// Hood State
// ============================================================================
struct HoodState {
  bool lightUp = false;
  bool lightDown = false;
  uint8_t fanSpeed = 0;
  bool nachlauf = false;
#if HOOD_HAS_COVER
  const char* position = "Oben";
  const char* coverState = "up";
#endif
  bool bleConnected = false;
  uint8_t raw[9] = {0};
};

// ============================================================================
// Onboard LED
// ============================================================================
#define LED_PIN 2
#define LED_BLINK_MS 500

// ============================================================================
// Globals
// ============================================================================
NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pNotifyChar = nullptr;
volatile bool deviceConnected = false;
bool oldDeviceConnected = false;

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
WebServer webServer(80);
HoodState hood;
unsigned long lastMqttReconnect = 0;
bool discoveryPublished = false;
bool wifiStarted = false;
bool otaReady = false;
bool hoodStateValid = false;

volatile bool newStatusReceived = false;
uint8_t pendingStatus[9];

struct CmdEntry {
  uint8_t code;
  char name[16];
};
#define CMD_QUEUE_SIZE 16
#define CMD_DELAY_MS 300
CmdEntry cmdQueue[CMD_QUEUE_SIZE];
int cmdQueueHead = 0;
int cmdQueueTail = 0;
unsigned long lastCmdSent = 0;

// ============================================================================
// Raw Advertising Data
// ============================================================================
static uint8_t raw_adv_data[] = {
  0x02, 0x01, 0x05,
  0x12, 0x21,
  0x6c, 0x65, 0x62, 0x72, 0x65, 0x62, 0x43, 0x80,
  0x53, 0x40, 0x45, 0x57, 0x00, 0xf0, 0x00, 0xf0,
  0x01
};

// ============================================================================
// Web UI HTML (stored in flash)
// ============================================================================
static const char WEB_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="de">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Berbel Steuerung</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=DM+Sans:wght@300;400;500;600&family=DM+Mono:wght@400;500&display=swap');

  :root {
    --bg: #0f0f11;
    --surface: #18181c;
    --surface2: #222228;
    --border: #2a2a32;
    --accent: #e8c97a;
    --accent2: #7ac4e8;
    --text: #e8e8ec;
    --muted: #6b6b78;
    --on: #4ade80;
    --off: #ef4444;
    --radius: 14px;
  }

  * { box-sizing: border-box; margin: 0; padding: 0; }

  body {
    font-family: 'DM Sans', sans-serif;
    background: var(--bg);
    color: var(--text);
    min-height: 100vh;
    padding: 24px 16px 48px;
  }

  header {
    max-width: 480px;
    margin: 0 auto 32px;
    display: flex;
    align-items: center;
    justify-content: space-between;
  }

  .brand {
    display: flex;
    flex-direction: column;
  }

  .brand h1 {
    font-size: 1.4rem;
    font-weight: 600;
    letter-spacing: -0.02em;
    color: var(--accent);
  }

  .brand span {
    font-size: 0.75rem;
    color: var(--muted);
    font-family: 'DM Mono', monospace;
    margin-top: 2px;
  }

  .ble-badge {
    display: flex;
    align-items: center;
    gap: 6px;
    padding: 6px 12px;
    border-radius: 20px;
    background: var(--surface);
    border: 1px solid var(--border);
    font-size: 0.75rem;
    font-family: 'DM Mono', monospace;
    color: var(--muted);
    transition: all 0.3s;
  }

  .ble-badge.connected {
    border-color: var(--on);
    color: var(--on);
  }

  .ble-dot {
    width: 7px; height: 7px;
    border-radius: 50%;
    background: var(--muted);
    transition: background 0.3s;
  }

  .ble-badge.connected .ble-dot {
    background: var(--on);
    box-shadow: 0 0 8px var(--on);
    animation: pulse 2s infinite;
  }

  @keyframes pulse {
    0%, 100% { opacity: 1; }
    50% { opacity: 0.4; }
  }

  .container { max-width: 480px; margin: 0 auto; display: flex; flex-direction: column; gap: 16px; }

  .card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    padding: 20px;
  }

  .card-label {
    font-size: 0.68rem;
    font-family: 'DM Mono', monospace;
    text-transform: uppercase;
    letter-spacing: 0.1em;
    color: var(--muted);
    margin-bottom: 16px;
  }

  /* Fan Speed */
  .fan-grid {
    display: grid;
    grid-template-columns: repeat(5, 1fr);
    gap: 8px;
  }

  .fan-btn {
    background: var(--surface2);
    border: 1px solid var(--border);
    border-radius: 10px;
    color: var(--text);
    font-family: 'DM Sans', sans-serif;
    font-size: 0.8rem;
    font-weight: 500;
    padding: 12px 4px;
    cursor: pointer;
    transition: all 0.15s;
    text-align: center;
  }

  .fan-btn:hover { border-color: var(--accent); color: var(--accent); }
  .fan-btn.active {
    background: var(--accent);
    border-color: var(--accent);
    color: #0f0f11;
    font-weight: 600;
  }

  /* Lights */
  .light-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }

  .light-btn {
    background: var(--surface2);
    border: 1px solid var(--border);
    border-radius: 10px;
    color: var(--muted);
    font-family: 'DM Sans', sans-serif;
    font-size: 0.85rem;
    font-weight: 500;
    padding: 16px;
    cursor: pointer;
    transition: all 0.15s;
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 8px;
  }

  .light-btn svg { opacity: 0.4; transition: opacity 0.15s; }
  .light-btn:hover { border-color: var(--accent2); color: var(--text); }
  .light-btn:hover svg { opacity: 0.8; }

  .light-btn.on {
    border-color: var(--accent);
    color: var(--accent);
    background: rgba(232, 201, 122, 0.07);
  }
  .light-btn.on svg { opacity: 1; }

  /* Action Row */
  .action-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }

  .action-btn {
    background: var(--surface2);
    border: 1px solid var(--border);
    border-radius: 10px;
    color: var(--text);
    font-family: 'DM Sans', sans-serif;
    font-size: 0.82rem;
    font-weight: 500;
    padding: 14px;
    cursor: pointer;
    transition: all 0.15s;
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 8px;
  }

  .action-btn:hover { border-color: var(--accent2); }

  .action-btn.nachlauf-on {
    border-color: var(--accent2);
    color: var(--accent2);
    background: rgba(122, 196, 232, 0.07);
  }

  .action-btn.power {
    border-color: #3a1a1a;
    color: var(--off);
  }
  .action-btn.power:hover { border-color: var(--off); background: rgba(239,68,68,0.08); }

  /* Cover Position */
  .cover-grid {
    display: grid;
    grid-template-columns: 1fr auto 1fr;
    gap: 10px;
    align-items: center;
  }

  .cover-btn {
    background: var(--surface2);
    border: 1px solid var(--border);
    border-radius: 10px;
    color: var(--text);
    font-family: 'DM Sans', sans-serif;
    font-size: 0.82rem;
    font-weight: 500;
    padding: 14px 10px;
    cursor: pointer;
    transition: all 0.15s;
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 8px;
  }

  .cover-btn:hover { border-color: var(--accent2); color: var(--accent2); }
  .cover-btn:hover svg { stroke: var(--accent2); }

  .cover-btn.active {
    border-color: var(--accent2);
    color: var(--accent2);
    background: rgba(122, 196, 232, 0.07);
  }

  .cover-state-badge {
    text-align: center;
    font-size: 0.7rem;
    font-family: 'DM Mono', monospace;
    color: var(--muted);
    padding: 6px 4px;
    line-height: 1.4;
  }

  /* Pairing */
  .pair-section {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    padding: 20px;
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 16px;
  }

  .pair-info h3 { font-size: 0.9rem; font-weight: 500; margin-bottom: 4px; }
  .pair-info p { font-size: 0.75rem; color: var(--muted); line-height: 1.5; }

  .pair-btn {
    flex-shrink: 0;
    background: var(--accent);
    border: none;
    border-radius: 10px;
    color: #0f0f11;
    font-family: 'DM Sans', sans-serif;
    font-size: 0.82rem;
    font-weight: 600;
    padding: 10px 18px;
    cursor: pointer;
    transition: opacity 0.15s;
    white-space: nowrap;
  }

  .pair-btn:hover { opacity: 0.85; }
  .pair-btn:disabled { opacity: 0.4; cursor: default; }

  /* Status footer */
  .status-bar {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    padding: 14px 18px;
    display: flex;
    align-items: center;
    justify-content: space-between;
  }

  .status-bar span {
    font-size: 0.72rem;
    font-family: 'DM Mono', monospace;
    color: var(--muted);
  }

  .raw-bytes { color: var(--surface2); font-size: 0.65rem !important; letter-spacing: 0.05em; }

  /* Toast */
  .toast {
    position: fixed;
    bottom: 24px; left: 50%;
    transform: translateX(-50%) translateY(80px);
    background: var(--surface2);
    border: 1px solid var(--border);
    border-radius: 20px;
    padding: 10px 20px;
    font-size: 0.8rem;
    color: var(--text);
    transition: transform 0.3s cubic-bezier(0.34,1.56,0.64,1);
    z-index: 99;
    white-space: nowrap;
  }

  .toast.show { transform: translateX(-50%) translateY(0); }

  /* Disconnected overlay */
  .disconnected-hint {
    text-align: center;
    padding: 12px;
    background: rgba(239,68,68,0.07);
    border: 1px solid rgba(239,68,68,0.2);
    border-radius: 10px;
    font-size: 0.8rem;
    color: #f87171;
    display: none;
  }

  .disconnected-hint.show { display: block; }
</style>
</head>
<body>

<header>
  <div class="brand">
    <h1>Berbel</h1>
    <span>Skyline · berbel-remote.local</span>
  </div>
  <div class="ble-badge" id="bleBadge">
    <div class="ble-dot"></div>
    <span id="bleText">Verbinde...</span>
  </div>
</header>

<div class="container">

  <div class="disconnected-hint" id="discHint">
    Haube nicht verbunden — Befehle werden in die Warteschlange gestellt.
  </div>

  <!-- Fan Speed -->
  <div class="card">
    <div class="card-label">Lüfter</div>
    <div class="fan-grid">
      <button class="fan-btn" id="fan0" onclick="setFan('Aus')">Aus</button>
      <button class="fan-btn" id="fan1" onclick="setFan('Stufe 1')">Stufe 1</button>
      <button class="fan-btn" id="fan2" onclick="setFan('Stufe 2')">Stufe 2</button>
      <button class="fan-btn" id="fan3" onclick="setFan('Stufe 3')">Stufe 3</button>
      <button class="fan-btn" id="fan4" onclick="setFan('Power')">Power</button>
    </div>
  </div>

  <!-- Lights -->
  <div class="card">
    <div class="card-label">Beleuchtung</div>
    <div class="light-grid">
      <button class="light-btn" id="lightUp" onclick="toggleLight('up')">
        <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8">
          <circle cx="12" cy="12" r="4"/><path d="M12 2v2M12 20v2M4.93 4.93l1.41 1.41M17.66 17.66l1.41 1.41M2 12h2M20 12h2M6.34 17.66l-1.41 1.41M19.07 4.93l-1.41 1.41"/>
        </svg>
        Oberlicht
      </button>
      <button class="light-btn" id="lightDown" onclick="toggleLight('down')">
        <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8">
          <path d="M9 18h6M10 22h4M12 2v2"/><path d="M5 12a7 7 0 1 0 14 0 7 7 0 0 0-14 0z"/>
        </svg>
        Unterlicht
      </button>
    </div>
  </div>

  <!-- Actions -->
  <div class="card">
    <div class="card-label">Aktionen</div>
    <div class="action-grid">
      <button class="action-btn" id="nachlaufBtn" onclick="toggleNachlauf()">
        <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
          <circle cx="12" cy="12" r="10"/><polyline points="12 6 12 12 16 14"/>
        </svg>
        Nachlauf
      </button>
      <button class="action-btn power" onclick="powerOff()">
        <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
          <path d="M18.36 6.64a9 9 0 1 1-12.73 0"/><line x1="12" y1="2" x2="12" y2="12"/>
        </svg>
        Ausschalten
      </button>
    </div>
  </div>

  <!-- Cover Position -->
  <div class="card">
    <div class="card-label">Position</div>
    <div class="cover-grid">
      <button class="cover-btn" id="coverUp" onclick="moveHood('up')">
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
          <polyline points="17 11 12 6 7 11"/><polyline points="17 18 12 13 7 18"/>
        </svg>
        Hochfahren
      </button>
      <div class="cover-state-badge" id="coverState">–</div>
      <button class="cover-btn" id="coverDown" onclick="moveHood('down')">
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
          <polyline points="7 13 12 18 17 13"/><polyline points="7 6 12 11 17 6"/>
        </svg>
        Herunterfahren
      </button>
    </div>
  </div>

  <!-- BLE Pairing -->
  <div class="pair-section">
    <div class="pair-info">
      <h3>BLE Verbindung</h3>
      <p>Haube in Pairing-Modus versetzen,<br>dann hier koppeln.</p>
    </div>
    <button class="pair-btn" id="pairBtn" onclick="startPairing()">Koppeln</button>
  </div>

  <!-- Raw Status -->
  <div class="status-bar">
    <span>RAW</span>
    <span class="raw-bytes" id="rawBytes">-- -- -- -- -- -- -- -- --</span>
  </div>

</div>

<div class="toast" id="toast"></div>

<script>
let state = {};

function showToast(msg) {
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.classList.add('show');
  setTimeout(() => t.classList.remove('show'), 2200);
}

function cmd(url) {
  fetch(url).then(r => r.json()).then(d => {
    if (d.ok) showToast('✓ ' + d.msg);
    else showToast('✗ ' + d.msg);
  }).catch(() => showToast('✗ Fehler'));
}

function setFan(level) { cmd('/api/fan?level=' + encodeURIComponent(level)); }
function toggleLight(which) { cmd('/api/light?which=' + which); }
function toggleNachlauf() { cmd('/api/nachlauf'); }
function powerOff() { cmd('/api/power'); }
function moveHood(dir) { cmd('/api/cover?dir=' + dir); }
function startPairing() {
  const btn = document.getElementById('pairBtn');
  btn.disabled = true;
  btn.textContent = 'Suche...';
  fetch('/api/pair').then(r => r.json()).then(d => {
    showToast(d.msg);
    btn.disabled = false;
    btn.textContent = 'Koppeln';
  });
}

function updateUI(s) {
  state = s;

  // BLE badge
  const badge = document.getElementById('bleBadge');
  const bleText = document.getElementById('bleText');
  const hint = document.getElementById('discHint');
  if (s.ble) {
    badge.className = 'ble-badge connected';
    bleText.textContent = 'Verbunden';
    hint.className = 'disconnected-hint';
  } else {
    badge.className = 'ble-badge';
    bleText.textContent = 'Getrennt';
    hint.className = 'disconnected-hint show';
  }

  // Fan buttons
  const fanMap = {'Aus':'fan0','Stufe 1':'fan1','Stufe 2':'fan2','Stufe 3':'fan3','Power':'fan4'};
  Object.values(fanMap).forEach(id => document.getElementById(id).classList.remove('active'));
  if (fanMap[s.fan]) document.getElementById(fanMap[s.fan]).classList.add('active');

  // Lights
  const lu = document.getElementById('lightUp');
  const ld = document.getElementById('lightDown');
  lu.className = 'light-btn' + (s.light_up ? ' on' : '');
  ld.className = 'light-btn' + (s.light_down ? ' on' : '');

  // Nachlauf
  const nb = document.getElementById('nachlaufBtn');
  nb.className = 'action-btn' + (s.nachlauf ? ' nachlauf-on' : '');

  // Cover position
  const cu = document.getElementById('coverUp');
  const cd = document.getElementById('coverDown');
  const cs = document.getElementById('coverState');
  if (cs) {
    const posMap = {
      'up': 'Oben', 'down': 'Unten',
      'moving up': '▲ fährt\nhoch', 'moving down': '▼ fährt\nrunter'
    };
    cs.textContent = posMap[s.cover_state] || s.position || '–';
    cu.className = 'cover-btn' + (s.cover_state === 'moving up' ? ' active' : '');
    cd.className = 'cover-btn' + (s.cover_state === 'moving down' ? ' active' : '');
  }

  // Raw bytes
  document.getElementById('rawBytes').textContent = s.raw || '-- -- -- -- -- -- -- -- --';
}

// Poll state every 2 seconds
function pollState() {
  fetch('/api/state')
    .then(r => r.json())
    .then(updateUI)
    .catch(() => {});
}

pollState();
setInterval(pollState, 2000);
</script>
</body>
</html>
)rawhtml";

// ============================================================================
// Forward Declarations
// ============================================================================
void sendButton(uint8_t code, const char* name);
void queueButton(uint8_t code, const char* name);
void processCmdQueue();
void publishState();
void publishDiscovery();
void startAdvertising();
void setupWebServer();

// ============================================================================
// Start BLE Advertising
// ============================================================================
void startAdvertising() {
  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->stop();

  NimBLEAdvertisementData advData;
  advData.addData(std::string(reinterpret_cast<const char*>(raw_adv_data), sizeof(raw_adv_data)));
  pAdvertising->setAdvertisementData(advData);

  NimBLEAdvertisementData scanData;
  pAdvertising->setScanResponseData(scanData);

  pAdvertising->setMinInterval(0x20);
  pAdvertising->setMaxInterval(0x40);
  pAdvertising->start();
  Serial.println("[BLE] Advertising started");
}

// ============================================================================
// BLE Callbacks
// ============================================================================
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("[BLE] Hood connected!");
  }

  void onDisconnect(NimBLEServer* pServer) override {
    deviceConnected = false;
    Serial.println("[BLE] Hood disconnected");
    delay(100);
    startAdvertising();
  }
};

class WriteCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar) override {
    std::string value = pChar->getValue();
    Serial.printf("[HOOD] Status (%d bytes): ", value.length());
    for (size_t i = 0; i < value.length(); i++) Serial.printf("%02X ", (uint8_t)value[i]);
    Serial.println();

    if (value.length() == 9) {
      memcpy(pendingStatus, (const uint8_t*)value.data(), 9);
      newStatusReceived = true;
    }
  }
};

// ============================================================================
// Button Send / Queue
// ============================================================================
void sendButton(uint8_t code, const char* name) {
  if (!deviceConnected || !pNotifyChar) {
    Serial.printf("[BTN] Cannot send %s - not connected\n", name);
    return;
  }
  Serial.printf("[BTN] Sending: %s (0x%02X)\n", name, code);
  uint8_t press[] = {code, 0x00};
  pNotifyChar->setValue(press, 2);
  pNotifyChar->notify();
  delay(100);
  uint8_t release[] = {0x00, 0x00};
  pNotifyChar->setValue(release, 2);
  pNotifyChar->notify();
}

void queueButton(uint8_t code, const char* name) {
  int next = (cmdQueueHead + 1) % CMD_QUEUE_SIZE;
  if (next == cmdQueueTail) {
    Serial.printf("[CMD] Queue full, dropping: %s\n", name);
    return;
  }
  cmdQueue[cmdQueueHead].code = code;
  strncpy(cmdQueue[cmdQueueHead].name, name, sizeof(cmdQueue[cmdQueueHead].name) - 1);
  cmdQueue[cmdQueueHead].name[sizeof(cmdQueue[cmdQueueHead].name) - 1] = '\0';
  cmdQueueHead = next;
  Serial.printf("[CMD] Queued: %s (0x%02X)\n", name, code);
}

void processCmdQueue() {
  if (cmdQueueHead == cmdQueueTail) return;
  if (!deviceConnected || !pNotifyChar) return;
  unsigned long now = millis();
  if (now - lastCmdSent < CMD_DELAY_MS) return;
  CmdEntry& cmd = cmdQueue[cmdQueueTail];
  sendButton(cmd.code, cmd.name);
  cmdQueueTail = (cmdQueueTail + 1) % CMD_QUEUE_SIZE;
  lastCmdSent = millis();
}

// ============================================================================
// MQTT State Publishing
// ============================================================================
static const char* fanPresetName(uint8_t speed) {
  switch (speed) {
    case 1: return "Stufe 1";
    case 2: return "Stufe 2";
    case 3: return "Stufe 3";
    case 4: return "Power";
    default: return "Aus";
  }
}

void publishState() {
  if (!mqtt.connected()) return;
  if (!hoodStateValid) {
    char json[64];
    snprintf(json, sizeof(json), "{\"ble\":\"%s\"}", hood.bleConnected ? "ON" : "OFF");
    mqtt.publish(MQTT_STATE, json, true);
    return;
  }
  char json[384];
  snprintf(json, sizeof(json),
    "{"
    "\"light_up\":\"%s\","
    "\"light_down\":\"%s\","
    "\"fan_preset\":\"%s\","
    "\"nachlauf\":\"%s\","
#if HOOD_HAS_COVER
    "\"position\":\"%s\","
    "\"cover_state\":\"%s\","
#endif
    "\"ble\":\"%s\","
    "\"status_raw\":\"%02X %02X %02X %02X %02X %02X %02X %02X %02X\""
    "}",
    hood.lightUp ? "ON" : "OFF",
    hood.lightDown ? "ON" : "OFF",
    fanPresetName(hood.fanSpeed),
    hood.nachlauf ? "ON" : "OFF",
#if HOOD_HAS_COVER
    hood.position,
    hood.coverState,
#endif
    hood.bleConnected ? "ON" : "OFF",
    hood.raw[0], hood.raw[1], hood.raw[2], hood.raw[3], hood.raw[4],
    hood.raw[5], hood.raw[6], hood.raw[7], hood.raw[8]);
  mqtt.publish(MQTT_STATE, json, true);
}

// ============================================================================
// HA MQTT Discovery
// ============================================================================
static const char DISCOVERY_DEVICE[] =
  ",\"avty_t\":\"" MQTT_AVAIL "\""
  ",\"dev\":{\"ids\":[\"berbel_hood\"]"
  ",\"name\":\"Berbel Hood\""
  ",\"mf\":\"Berbel\",\"mdl\":\"BFB 6bT\"}}";

void publishDiscoveryMsg(const char* topic, const char* fields) {
  char buf[768];
  snprintf(buf, sizeof(buf), "{%s%s", fields, DISCOVERY_DEVICE);
  mqtt.publish(topic, buf, true);
  delay(50);
}

void cleanupOldDiscovery() {
  const char* oldTopics[] = {
    "homeassistant/fan/berbel_hood/fan/config",
    "homeassistant/binary_sensor/berbel_hood/nachlauf/config",
    "homeassistant/cover/berbel_hood/cover/config",
    nullptr
  };
  for (int i = 0; oldTopics[i] != nullptr; i++) {
    mqtt.publish(oldTopics[i], "", true);
    delay(50);
  }
}

void publishDiscovery() {
  Serial.println("[MQTT] Publishing HA discovery...");
  cleanupOldDiscovery();

  publishDiscoveryMsg(
    "homeassistant/light/berbel_hood/light_up/config",
    "\"name\":\"Oberlicht\","
    "\"uniq_id\":\"berbel_light_up\","
    "\"stat_t\":\"" MQTT_STATE "\","
    "\"cmd_t\":\"" MQTT_CMD_LIGHT_UP "\","
    "\"stat_val_tpl\":\"{{ value_json.light_up }}\","
    "\"ic\":\"mdi:ceiling-light\""
  );

  publishDiscoveryMsg(
    "homeassistant/light/berbel_hood/light_down/config",
    "\"name\":\"Unterlicht\","
    "\"uniq_id\":\"berbel_light_down\","
    "\"stat_t\":\"" MQTT_STATE "\","
    "\"cmd_t\":\"" MQTT_CMD_LIGHT_DOWN "\","
    "\"stat_val_tpl\":\"{{ value_json.light_down }}\","
    "\"ic\":\"mdi:desk-lamp\""
  );

  publishDiscoveryMsg(
    "homeassistant/select/berbel_hood/fan/config",
    "\"name\":\"L\\u00fcfter\","
    "\"uniq_id\":\"berbel_fan\","
    "\"stat_t\":\"" MQTT_STATE "\","
    "\"val_tpl\":\"{{ value_json.fan_preset }}\","
    "\"cmd_t\":\"" MQTT_CMD_FAN_PRESET "\","
    "\"ops\":[\"Aus\",\"Stufe 1\",\"Stufe 2\",\"Stufe 3\",\"Power\"],"
    "\"ic\":\"mdi:fan\""
  );

#if HOOD_HAS_COVER
  publishDiscoveryMsg(
    "homeassistant/select/berbel_hood/position/config",
    "\"name\":\"Position\","
    "\"uniq_id\":\"berbel_position\","
    "\"stat_t\":\"" MQTT_STATE "\","
    "\"val_tpl\":\"{{ value_json.position }}\","
    "\"cmd_t\":\"" MQTT_CMD_POSITION "\","
    "\"ops\":[\"Oben\",\"Unten\"],"
    "\"ic\":\"mdi:arrow-up-down\""
  );
#endif

  publishDiscoveryMsg(
    "homeassistant/binary_sensor/berbel_hood/ble/config",
    "\"name\":\"BLE Verbindung\","
    "\"uniq_id\":\"berbel_ble\","
    "\"stat_t\":\"" MQTT_STATE "\","
    "\"val_tpl\":\"{{ value_json.ble }}\","
    "\"dev_cla\":\"connectivity\","
    "\"ent_cat\":\"diagnostic\""
  );

  publishDiscoveryMsg(
    "homeassistant/button/berbel_hood/power/config",
    "\"name\":\"Ausschalten\","
    "\"uniq_id\":\"berbel_power\","
    "\"cmd_t\":\"" MQTT_CMD_POWER "\","
    "\"ic\":\"mdi:power\""
  );

  publishDiscoveryMsg(
    "homeassistant/switch/berbel_hood/nachlauf/config",
    "\"name\":\"Nachlauf\","
    "\"uniq_id\":\"berbel_nachlauf\","
    "\"stat_t\":\"" MQTT_STATE "\","
    "\"val_tpl\":\"{{ value_json.nachlauf }}\","
    "\"cmd_t\":\"" MQTT_CMD_NACHLAUF "\","
    "\"ic\":\"mdi:timer-sand\""
  );

#if HOOD_HAS_COVER
  publishDiscoveryMsg(
    "homeassistant/button/berbel_hood/move_up/config",
    "\"name\":\"Hochfahren\","
    "\"uniq_id\":\"berbel_move_up\","
    "\"cmd_t\":\"" MQTT_CMD_MOVE_UP "\","
    "\"ic\":\"mdi:arrow-up\""
  );

  publishDiscoveryMsg(
    "homeassistant/button/berbel_hood/move_down/config",
    "\"name\":\"Herunterfahren\","
    "\"uniq_id\":\"berbel_move_down\","
    "\"cmd_t\":\"" MQTT_CMD_MOVE_DOWN "\","
    "\"ic\":\"mdi:arrow-down\""
  );

  publishDiscoveryMsg(
    "homeassistant/sensor/berbel_hood/cover_state/config",
    "\"name\":\"Cover State\","
    "\"uniq_id\":\"berbel_cover_state\","
    "\"stat_t\":\"" MQTT_STATE "\","
    "\"val_tpl\":\"{{ value_json.cover_state }}\","
    "\"ent_cat\":\"diagnostic\","
    "\"ic\":\"mdi:arrow-up-down\""
  );
#endif

  publishDiscoveryMsg(
    "homeassistant/sensor/berbel_hood/status_raw/config",
    "\"name\":\"Status Raw\","
    "\"uniq_id\":\"berbel_status_raw\","
    "\"stat_t\":\"" MQTT_STATE "\","
    "\"val_tpl\":\"{{ value_json.status_raw }}\","
    "\"ent_cat\":\"diagnostic\","
    "\"ic\":\"mdi:bug\""
  );

  discoveryPublished = true;
  Serial.println("[MQTT] Discovery published!");
}

// ============================================================================
// Restore state from retained MQTT
// ============================================================================
static bool jsonGetValue(const char* json, const char* key, char* out, size_t outLen) {
  char search[32];
  snprintf(search, sizeof(search), "\"%s\":\"", key);
  const char* start = strstr(json, search);
  if (!start) return false;
  start += strlen(search);
  const char* end = strchr(start, '"');
  if (!end || (size_t)(end - start) >= outLen) return false;
  memcpy(out, start, end - start);
  out[end - start] = '\0';
  return true;
}

void restoreStateFromMqtt(const char* json) {
  char val[32];
  if (jsonGetValue(json, "light_up", val, sizeof(val)))
    hood.lightUp = (strcmp(val, "ON") == 0);
  if (jsonGetValue(json, "light_down", val, sizeof(val)))
    hood.lightDown = (strcmp(val, "ON") == 0);
  if (jsonGetValue(json, "nachlauf", val, sizeof(val)))
    hood.nachlauf = (strcmp(val, "ON") == 0);
  if (jsonGetValue(json, "fan_preset", val, sizeof(val))) {
    if (strcmp(val, "Stufe 1") == 0)      hood.fanSpeed = 1;
    else if (strcmp(val, "Stufe 2") == 0)  hood.fanSpeed = 2;
    else if (strcmp(val, "Stufe 3") == 0)  hood.fanSpeed = 3;
    else if (strcmp(val, "Power") == 0)    hood.fanSpeed = 4;
    else                                   hood.fanSpeed = 0;
  }
#if HOOD_HAS_COVER
  if (jsonGetValue(json, "position", val, sizeof(val)))
    hood.position = (strcmp(val, "Unten") == 0) ? "Unten" : "Oben";
  if (jsonGetValue(json, "cover_state", val, sizeof(val))) {
    if (strcmp(val, "down") == 0)             hood.coverState = "down";
    else if (strcmp(val, "moving up") == 0)   hood.coverState = "moving up";
    else if (strcmp(val, "moving down") == 0) hood.coverState = "moving down";
    else                                      hood.coverState = "up";
  }
#endif
  hoodStateValid = true;
  mqtt.unsubscribe(MQTT_STATE);
  Serial.println("[MQTT] State restored from retained message");
}

// ============================================================================
// MQTT Callback
// ============================================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char msg[384];
  unsigned int copyLen = length < sizeof(msg) - 1 ? length : sizeof(msg) - 1;
  memcpy(msg, payload, copyLen);
  msg[copyLen] = '\0';
  Serial.printf("[MQTT] %s = %s\n", topic, msg);

  String t(topic);

  if (t == MQTT_STATE && !hoodStateValid) {
    restoreStateFromMqtt(msg);
    return;
  }

  if (t == MQTT_CMD_LIGHT_UP) {
    bool wantOn = (strcmp(msg, "ON") == 0);
    if (wantOn == hood.lightUp) return;
    queueButton(BTN_LIGHT_UP, "Light Up");
  }
  else if (t == MQTT_CMD_LIGHT_DOWN) {
    bool wantOn = (strcmp(msg, "ON") == 0);
    if (wantOn == hood.lightDown) return;
    queueButton(BTN_LIGHT_DOWN, "Light Down");
  }
  else if (t == MQTT_CMD_POWER) {
    queueButton(BTN_POWER, "Power Off");
  }
  else if (t == MQTT_CMD_NACHLAUF) {
    bool wantOn = (strcmp(msg, "ON") == 0);
    if (wantOn == hood.nachlauf) return;
    queueButton(BTN_TIMER, "Timer");
  }
  else if (t == MQTT_CMD_FAN_PRESET) {
    uint8_t targetSpeed = 0;
    uint8_t btnCode = BTN_POWER;
    const char* btnName = "Fan Off";
    if (strcmp(msg, "Stufe 1") == 0)      { targetSpeed = 1; btnCode = BTN_FAN_1; btnName = "Fan 1"; }
    else if (strcmp(msg, "Stufe 2") == 0)  { targetSpeed = 2; btnCode = BTN_FAN_2; btnName = "Fan 2"; }
    else if (strcmp(msg, "Stufe 3") == 0)  { targetSpeed = 3; btnCode = BTN_FAN_3; btnName = "Fan 3"; }
    else if (strcmp(msg, "Power") == 0)    { targetSpeed = 4; btnCode = BTN_FAN_P; btnName = "Fan Power"; }
    if (targetSpeed == hood.fanSpeed) return;
    queueButton(btnCode, btnName);
  }
#if HOOD_HAS_COVER
  else if (t == MQTT_CMD_POSITION) {
    if (strcmp(msg, "Oben") == 0)       queueButton(BTN_MOVE_UP, "Move Up");
    else if (strcmp(msg, "Unten") == 0) queueButton(BTN_MOVE_DOWN, "Move Down");
  }
  else if (t == MQTT_CMD_MOVE_UP)   queueButton(BTN_MOVE_UP, "Move Up");
  else if (t == MQTT_CMD_MOVE_DOWN) queueButton(BTN_MOVE_DOWN, "Move Down");
#endif
  else if (t == "homeassistant/status" && strcmp(msg, "online") == 0) {
    publishDiscovery();
    publishState();
  }
  else if (t == MQTT_CMD_DEBUG) {
    uint8_t code = (uint8_t)strtol(msg, NULL, 16);
    if (code >= 0x01 && code <= 0x0D) {
      char name[16];
      snprintf(name, sizeof(name), "Debug 0x%02X", code);
      queueButton(code, name);
    }
  }
}

// ============================================================================
// Web Server Setup
// ============================================================================
void setupWebServer() {
  // Main UI
  webServer.on("/", HTTP_GET, []() {
    webServer.sendHeader("Content-Encoding", "identity");
    webServer.send_P(200, "text/html", WEB_HTML);
  });

  // GET /api/state — returns current hood state as JSON
  webServer.on("/api/state", HTTP_GET, []() {
    char json[320];
    snprintf(json, sizeof(json),
      "{"
      "\"ble\":%s,"
      "\"fan\":\"%s\","
      "\"light_up\":%s,"
      "\"light_down\":%s,"
      "\"nachlauf\":%s,"
      "\"position\":\"%s\","
      "\"cover_state\":\"%s\","
      "\"raw\":\"%02X %02X %02X %02X %02X %02X %02X %02X %02X\""
      "}",
      hood.bleConnected ? "true" : "false",
      fanPresetName(hood.fanSpeed),
      hood.lightUp ? "true" : "false",
      hood.lightDown ? "true" : "false",
      hood.nachlauf ? "true" : "false",
      hood.position,
      hood.coverState,
      hood.raw[0], hood.raw[1], hood.raw[2], hood.raw[3], hood.raw[4],
      hood.raw[5], hood.raw[6], hood.raw[7], hood.raw[8]
    );
    webServer.sendHeader("Access-Control-Allow-Origin", "*");
    webServer.send(200, "application/json", json);
  });

  // GET /api/fan?level=Stufe+1
  webServer.on("/api/fan", HTTP_GET, []() {
    if (!webServer.hasArg("level")) {
      webServer.send(400, "application/json", "{\"ok\":false,\"msg\":\"Missing level\"}");
      return;
    }
    String level = webServer.arg("level");
    uint8_t btnCode = BTN_POWER;
    const char* btnName = "Fan Off";
    uint8_t targetSpeed = 0;

    if (level == "Stufe 1")      { targetSpeed = 1; btnCode = BTN_FAN_1; btnName = "Fan 1"; }
    else if (level == "Stufe 2")  { targetSpeed = 2; btnCode = BTN_FAN_2; btnName = "Fan 2"; }
    else if (level == "Stufe 3")  { targetSpeed = 3; btnCode = BTN_FAN_3; btnName = "Fan 3"; }
    else if (level == "Power")    { targetSpeed = 4; btnCode = BTN_FAN_P; btnName = "Fan Power"; }

    if (targetSpeed == hood.fanSpeed) {
      webServer.send(200, "application/json", "{\"ok\":true,\"msg\":\"Bereits aktiv\"}");
      return;
    }
    queueButton(btnCode, btnName);
    webServer.send(200, "application/json", "{\"ok\":true,\"msg\":\"Lüfter gesetzt\"}");
  });

  // GET /api/light?which=up|down
  webServer.on("/api/light", HTTP_GET, []() {
    if (!webServer.hasArg("which")) {
      webServer.send(400, "application/json", "{\"ok\":false,\"msg\":\"Missing which\"}");
      return;
    }
    String which = webServer.arg("which");
    if (which == "up")        queueButton(BTN_LIGHT_UP, "Light Up");
    else if (which == "down") queueButton(BTN_LIGHT_DOWN, "Light Down");
    else {
      webServer.send(400, "application/json", "{\"ok\":false,\"msg\":\"Invalid which\"}");
      return;
    }
    webServer.send(200, "application/json", "{\"ok\":true,\"msg\":\"Licht umgeschaltet\"}");
  });

  // GET /api/cover?dir=up|down
  webServer.on("/api/cover", HTTP_GET, []() {
    if (!webServer.hasArg("dir")) {
      webServer.send(400, "application/json", "{\"ok\":false,\"msg\":\"Missing dir\"}");
      return;
    }
    String dir = webServer.arg("dir");
    if (dir == "up")        queueButton(BTN_MOVE_UP, "Move Up");
    else if (dir == "down") queueButton(BTN_MOVE_DOWN, "Move Down");
    else {
      webServer.send(400, "application/json", "{\"ok\":false,\"msg\":\"Invalid dir\"}");
      return;
    }
    webServer.send(200, "application/json", "{\"ok\":true,\"msg\":\"Position gesendet\"}");
  });

  // GET /api/nachlauf
  webServer.on("/api/nachlauf", HTTP_GET, []() {
    queueButton(BTN_TIMER, "Nachlauf");
    webServer.send(200, "application/json", "{\"ok\":true,\"msg\":\"Nachlauf umgeschaltet\"}");
  });

  // GET /api/power
  webServer.on("/api/power", HTTP_GET, []() {
    queueButton(BTN_POWER, "Power Off");
    webServer.send(200, "application/json", "{\"ok\":true,\"msg\":\"Ausschalten gesendet\"}");
  });

  // GET /api/pair — restart advertising so hood can reconnect/pair
  webServer.on("/api/pair", HTTP_GET, []() {
    Serial.println("[WEB] Pairing requested via web interface");
    if (deviceConnected) {
      webServer.send(200, "application/json", "{\"ok\":true,\"msg\":\"Bereits verbunden\"}");
      return;
    }
    startAdvertising();
    webServer.send(200, "application/json", "{\"ok\":true,\"msg\":\"Advertising neu gestartet – Haube koppeln\"}");
  });

  webServer.onNotFound([]() {
    webServer.send(404, "application/json", "{\"ok\":false,\"msg\":\"Not found\"}");
  });

  webServer.begin();
  Serial.println("[WEB] Webserver started on port 80");
  Serial.printf("[WEB] URL: http://berbel-remote.local  (IP: http://%s)\n",
    WiFi.localIP().toString().c_str());
}

// ============================================================================
// WiFi Setup
// ============================================================================
void setupWiFi() {
  Serial.printf("[WiFi] Connecting to %s...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("berbel-remote");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  ArduinoOTA.setHostname("berbel-remote");
  ArduinoOTA.onStart([]() {
    Serial.println("[OTA] Starting, switching to WiFi priority...");
    esp_coex_preference_set(ESP_COEX_PREFER_WIFI);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\n[OTA] Done, restoring BLE priority...");
    esp_coex_preference_set(ESP_COEX_PREFER_BT);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("[OTA] %u%%\r", progress * 100 / total);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] Error %u\n", error);
  });

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
    ArduinoOTA.begin();
    otaReady = true;
    Serial.println("[OTA] Ready");
    setupWebServer();
  } else {
    Serial.println("\n[WiFi] Failed, will retry in loop");
  }
}

// ============================================================================
// MQTT Reconnect
// ============================================================================
void mqttReconnect() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connected()) return;
  unsigned long now = millis();
  if (now - lastMqttReconnect < 5000) return;
  lastMqttReconnect = now;

  Serial.printf("[MQTT] Connecting to %s:%d...\n", MQTT_HOST, MQTT_PORT);
  if (mqtt.connect("berbel-remote", MQTT_USER, MQTT_PASS,
                    MQTT_AVAIL, 0, true, "offline")) {
    Serial.println("[MQTT] Connected!");
    mqtt.publish(MQTT_AVAIL, "online", true);
    mqtt.subscribe(MQTT_CMD_LIGHT_UP);
    mqtt.subscribe(MQTT_CMD_LIGHT_DOWN);
    mqtt.subscribe(MQTT_CMD_POWER);
    mqtt.subscribe(MQTT_CMD_NACHLAUF);
    mqtt.subscribe(MQTT_CMD_FAN_PRESET);
#if HOOD_HAS_COVER
    mqtt.subscribe(MQTT_CMD_POSITION);
    mqtt.subscribe(MQTT_CMD_MOVE_UP);
    mqtt.subscribe(MQTT_CMD_MOVE_DOWN);
#endif
    mqtt.subscribe(MQTT_CMD_DEBUG);
    mqtt.subscribe("homeassistant/status");
    if (!hoodStateValid) {
      mqtt.subscribe(MQTT_STATE);
    }
    publishDiscovery();
    publishState();
  } else {
    Serial.printf("[MQTT] Failed, rc=%d\n", mqtt.state());
  }
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.println("\n============================================");
  Serial.println("  BERBEL REMOTE - HA Bridge + Web UI");
  Serial.println("============================================\n");

  // BLE first (MAC spoofing must happen before WiFi)
  Serial.println("[MAC] Setting Texas Instruments OUI...");
  uint8_t ti_mac[6] = {0x88, 0x01, 0xF9, 0xAA, 0xBB, 0xCC};
  esp_base_mac_addr_set(ti_mac);

  Serial.println("[BLE] Initializing NimBLE...");
  NimBLEDevice::init("");
  Serial.printf("[BLE] MAC: %s\n", NimBLEDevice::getAddress().toString().c_str());

  NimBLEDevice::setSecurityAuth(true, false, true);   // bonding=true, mitm=false, sc=true
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);             // maximale BLE Sendeleistung
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  NimBLEDevice::setSecurityInitKey(BLE_SM_PAIR_KEY_DIST_ENC);
  NimBLEDevice::setSecurityRespKey(BLE_SM_PAIR_KEY_DIST_ENC);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // DevInfo
  NimBLEService* pDevInfoService = pServer->createService(NimBLEUUID((uint16_t)0x180A));
  NimBLECharacteristic* pManufacturer = pDevInfoService->createCharacteristic(
      NimBLEUUID((uint16_t)0x2A29), NIMBLE_PROPERTY::READ);
  pManufacturer->setValue("Texas Instruments");
  NimBLECharacteristic* pPnpId = pDevInfoService->createCharacteristic(
      NimBLEUUID((uint16_t)0x2A50), NIMBLE_PROPERTY::READ);
  uint8_t pnpId[] = {0x01, 0x0D, 0x00, 0x00, 0x00, 0x10, 0x00};
  pPnpId->setValue(pnpId, 7);
  pDevInfoService->start();

  // Battery
  NimBLEService* pBatteryService = pServer->createService(NimBLEUUID((uint16_t)0x180F));
  NimBLECharacteristic* pBatteryLevel = pBatteryService->createCharacteristic(
      NimBLEUUID((uint16_t)0x2A19),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  uint8_t batteryLevel = 90;
  pBatteryLevel->setValue(&batteryLevel, 1);
  pBatteryService->start();

  // HID
  NimBLEService* pHidService = pServer->createService(NimBLEUUID((uint16_t)0x1812));
  NimBLECharacteristic* pHidInfo = pHidService->createCharacteristic(
    NimBLEUUID((uint16_t)0x2A4A), NIMBLE_PROPERTY::READ);
  uint8_t hidInfo[] = {0x11, 0x01, 0x00, 0x01};
  pHidInfo->setValue(hidInfo, 4);
  pHidService->createCharacteristic(NimBLEUUID((uint16_t)0x2A4C), NIMBLE_PROPERTY::WRITE_NR);
  NimBLECharacteristic* pProtocol = pHidService->createCharacteristic(
    NimBLEUUID((uint16_t)0x2A4E),
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE_NR);
  uint8_t protocolMode = 0x01;
  pProtocol->setValue(&protocolMode, 1);
  NimBLECharacteristic* pReportMap = pHidService->createCharacteristic(
    NimBLEUUID((uint16_t)0x2A4B), NIMBLE_PROPERTY::READ);
  const uint8_t reportMap[] = {
    0x05, 0x0C, 0x09, 0x01, 0xA1, 0x01, 0x85, 0x01,
    0x09, 0xE0, 0x15, 0xE8, 0x25, 0x18, 0x75, 0x08,
    0x95, 0x01, 0x81, 0x06, 0xC0
  };
  pReportMap->setValue(reportMap, sizeof(reportMap));
  NimBLECharacteristic* pReport = pHidService->createCharacteristic(
    NimBLEUUID((uint16_t)0x2A4D),
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  NimBLEDescriptor* pReportRef = pReport->createDescriptor(
    NimBLEUUID((uint16_t)0x2908), NIMBLE_PROPERTY::READ);
  uint8_t reportRef[] = {0x01, 0x01};
  pReportRef->setValue(reportRef, 2);
  pHidService->start();

  // Berbel Custom Service
  NimBLEService* pBerbelService = pServer->createService(BERBEL_SERVICE_UUID);
  pNotifyChar = pBerbelService->createCharacteristic(
    BERBEL_NOTIFY_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::WRITE_NR);
  uint8_t initVal[] = {0x00, 0x00};
  pNotifyChar->setValue(initVal, 2);
  NimBLECharacteristic* pWriteChar = pBerbelService->createCharacteristic(
    BERBEL_WRITE_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE_NR);
  pWriteChar->setCallbacks(new WriteCallbacks());
  pWriteChar->setValue(initVal, 2);
  pBerbelService->start();

  startAdvertising();
  Serial.println("[BLE] Services started, advertising...");

  esp_coex_preference_set(ESP_COEX_PREFER_BT);

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(1024);
  mqtt.setCallback(mqttCallback);

  Serial.printf("[SYS] Free heap before WiFi: %u bytes\n", esp_get_free_heap_size());
  setupWiFi();
  Serial.printf("[SYS] Free heap after WiFi: %u bytes\n", esp_get_free_heap_size());
  wifiStarted = true;

  Serial.println("\n============================================");
  Serial.println("  Ready! Webserver: http://berbel-remote.local");
  Serial.println("============================================\n");
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
  static unsigned long lastHeapLog = 0;
  unsigned long now = millis();
  if (now - lastHeapLog > 30000) {
    lastHeapLog = now;
    Serial.printf("[SYS] Free heap: %u  BLE: %s  WiFi: %s\n",
      esp_get_free_heap_size(),
      deviceConnected ? "connected" : "waiting",
      WiFi.status() == WL_CONNECTED ? "connected" : "disconnected");
  }

  // OTA
  if (wifiStarted && WiFi.status() == WL_CONNECTED) {
    if (!otaReady) { ArduinoOTA.begin(); otaReady = true; }
    ArduinoOTA.handle();
    webServer.handleClient();  // <-- Webserver requests
  }

  // WiFi reconnect
  if (wifiStarted && WiFi.status() != WL_CONNECTED) {
    static unsigned long lastWifiRetry = 0;
    if (now - lastWifiRetry > 30000) {
      lastWifiRetry = now;
      Serial.println("[WiFi] Reconnecting...");
      WiFi.reconnect();
    }
  }

  // MQTT
  if (wifiStarted) {
    if (!mqtt.connected()) mqttReconnect();
    else mqtt.loop();
  }

  // LED
  if (deviceConnected) {
    digitalWrite(LED_PIN, LOW);
  } else {
    static unsigned long lastLedToggle = 0;
    if (now - lastLedToggle >= LED_BLINK_MS) {
      lastLedToggle = now;
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
  }

  // BLE state change
  if (deviceConnected != oldDeviceConnected) {
    hood.bleConnected = deviceConnected;
    if (!deviceConnected) cmdQueueHead = cmdQueueTail = 0;
    publishState();
    oldDeviceConnected = deviceConnected;
  }

  // Process command queue
  processCmdQueue();

  // Process hood status
  if (newStatusReceived) {
    newStatusReceived = false;
    bool isSync = true;
    for (int i = 0; i < 9; i++) {
      if (pendingStatus[i] != 0x11) { isSync = false; break; }
    }
    if (isSync) {
      Serial.println("[HOOD] Sync packet ignored");
    } else {
      hoodStateValid = true;
      memcpy(hood.raw, pendingStatus, 9);
      hood.lightUp   = (hood.raw[2] & 0x10);
      hood.lightDown = (hood.raw[4] & 0x10);
      if (hood.raw[2] & 0x09)      hood.fanSpeed = 4;
      else if (hood.raw[1] & 0x10) hood.fanSpeed = 3;
      else if (hood.raw[1] & 0x01) hood.fanSpeed = 2;
      else if (hood.raw[0] & 0x10) hood.fanSpeed = 1;
      else                         hood.fanSpeed = 0;
      hood.nachlauf = (hood.raw[5] & 0x90);
#if HOOD_HAS_COVER
      if (hood.raw[4] & 0x01) {
        hood.coverState = "moving up"; hood.position = "Oben";
      } else if (hood.raw[6] & 0x01) {
        hood.coverState = "moving down"; hood.position = "Unten";
      } else if (strcmp(hood.coverState, "moving up") == 0) {
        hood.coverState = "up";
      } else if (strcmp(hood.coverState, "moving down") == 0) {
        hood.coverState = "down";
      }
#endif
      publishState();
    }
  }

  delay(10);
}