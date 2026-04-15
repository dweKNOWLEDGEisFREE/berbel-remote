# Berbel BFB 6bT — Remote Control Emulator & Home Assistant Bridge

> Intelligent control of a Berbel range hood via ESP32-S3 with BLE Dual-Role Bridge, Web Interface, Home Assistant Integration, and Alexa Support.

---

## 🏗 Architecture

```
Original Remote ──BLE──▶ ESP32-S3 ──BLE──▶ Berbel Hood
                          │
                         WiFi
                          │
               ┌──────────┴──────────┐
               │                     │
            MQTT                Webserver
               │           berbel-remote.local
               │
      Home Assistant
               │
            Alexa
```

The ESP32-S3 operates simultaneously in **two BLE roles**:
- **Peripheral** — appears to the original remote as the hood
- **Central** — connects as a remote to the actual hood

This allows the **original remote, web UI, Home Assistant, and Alexa to be used simultaneously**.

---

## 🚀 Features

### BLE Dual-Role Bridge
- Original remote remains fully functional
- Commands from the remote are transparently forwarded to the hood
- Hood status is mirrored in real-time to the remote + MQTT
- Persistent BLE bonding (survives reboots)
- MAC spoofing on Texas Instruments OUI (required by the hood)
- Correct GATT service layout (DevInfo → Battery → HID → Berbel Custom)

### Web Interface
- Modern, mobile-optimized interface at `http://berbel-remote.local`
- **Control Page** (`/`) — Fan, light, position, after-run, BLE pairing
- **Settings Page** (`/settings`) — Configure WiFi + MQTT
- **Update Page** (`/update`) — OTA firmware update via drag & drop in the browser
- Real-time status display (2-second polling)
- Dual BLE status badges (hood + remote)

### Setup Wizard on First Start
- No hardcoded WiFi password in the code
- ESP32 opens an Access Point **"Berbel-Setup"** on first start
- Captive portal opens automatically on iOS/Android
- Configure WiFi + MQTT via browser → Settings saved in flash (NVS)
- Factory reset function via web interface

### Home Assistant Integration
- MQTT Auto-Discovery — All entities appear automatically
- Real-time status feedback (even with manual operation on the hood)
- Alexa support via Nabu Casa or Emulated Hue

### OTA Updates
- Wireless via PlatformIO (`pio run -e ota --target upload`)
- Alternatively: Upload firmware `.bin` directly in the browser (`/update`)

---

## 🛠 Hardware

| Part | Details |
|---|---|
| **ESP32-S3 N16R8 DevKitC-1** | 16MB Flash, 8MB PSRAM, BLE 5.0 Dual-Role |
| USB-C Cable (Data Cable) | For initial flash |
| USB Power Supply 5V/1A | Continuous operation in the kitchen |
| Berbel Skyline | Model with BLE remote (from approx. 2020) |

> **Why ESP32-S3?** The classic ESP32 WROOM-32 does not stably support BLE Central + Peripheral simultaneously. The S3 has significantly better BLE/WiFi coexistence and sufficient RAM for dual-role operation.

---

## 📦 Home Assistant Entities

| Entity | Type | Function |
|---|---|---|
| `light.oberlicht` | Light | Top light on/off |
| `light.unterlicht` | Light | Cooktop lighting |
| `select.luefter` | Select | Off / Level 1 / Level 2 / Level 3 / Power |
| `switch.nachlauf` | Switch | After-run timer |
| `button.ausschalten` | Button | Turn off hood (starts after-run) |
| `button.hochfahren` | Button | Lift hood up (lift models) |
| `button.herunterfahren` | Button | Lower hood (lift models) |
| `select.position` | Select | Up / Down (lift models) |
| `binary_sensor.ble_haube` | Sensor | BLE connection ESP32 → Hood |
| `binary_sensor.ble_fernbedienung` | Sensor | BLE connection Remote → ESP32 |
| `sensor.status_raw` | Sensor | Raw 9-byte status data (diagnostics) |

---

## ⚙️ Setup

### 1. Prerequisites

- [VSCode](https://code.visualstudio.com) + [PlatformIO Extension](https://platformio.org/install/ide?install=vscode)
- Home Assistant with [Mosquitto MQTT Broker Add-on](https://github.com/home-assistant/addons/tree/master/mosquitto)
- Create an MQTT user in HA: *Settings → People → Users*

### 2. Clone the Project

```bash
git clone https://github.com/your-user/berbel-remote.git
cd berbel-remote
```

### 3. Initial Flash (USB)

Enter the USB port in `platformio.ini`:

```ini
[env:esp32s3dev]
upload_port = /dev/cu.usbmodem1234   # macOS
# upload_port = COM4                 # Windows
# upload_port = /dev/ttyUSB0        # Linux
```

Then flash:

```bash
pio run -e esp32s3dev --target upload
```

### 4. Setup Wizard

1. Connect to the **"Berbel-Setup"** WiFi network (no password)
2. The browser opens automatically — or manually: `http://192.168.4.1`
3. Enter WiFi name, password, MQTT server, and credentials
4. Save → ESP32 reboots and connects

### 5. Pair the Hood

```
http://berbel-remote.local → "Connections" tab → "Search for Hood"
```

Put the hood in pairing mode (hold the Bluetooth button for about 5-10 seconds).

### 6. Pair the Remote (Optional)

```
http://berbel-remote.local → "Connections" tab → "Pair Remote"
```

Turn on the remote or press the reset button.

---

## 🔄 OTA Updates

**Via PlatformIO (recommended for developers):**
```bash
pio run -e ota --target upload
```

**Via Browser (for everyone):**
```
http://berbel-remote.local/update
```
Upload the firmware file (`.bin`) via drag & drop. The binary is located after the build at:
```
.pio/build/esp32s3dev/firmware.bin
```

---

## 🏠 Alexa Setup

**Option A — Nabu Casa** (~7 €/month, recommended):
1. HA → *Settings → Home Assistant Cloud* → Enable Nabu Casa
2. Activate and link the "Home Assistant" Alexa skill
3. Discover devices

**Option B — Emulated Hue** (free):
```yaml
# configuration.yaml
emulated_hue:
  expose_by_default: false
  entities:
    fan.berbel_luefter:
      name: "Range Hood"
      hidden: false
```

**Example Voice Commands:**
```
"Alexa, turn on the range hood"
"Alexa, set the range hood to 66 percent"   → Level 2
"Alexa, turn on the kitchen light"
```

---

## 🔧 Technical Details

### BLE Protocol (Reverse Engineered)

```
Button Press:   [code, 0x00] → Characteristic f004f002
Button Release: [0x00, 0x00] → Characteristic f004f002
Hood Status:    9-Byte Write  ← Characteristic f004f001
Sync Packet:    [0x11 × 9]   (ignored)
```

**Hood Status Bytes (9 Bytes, Bitmask):**

| Byte | Bit | Meaning |
|---|---|---|
| 0 | 0x10 | Fan Level 1 |
| 1 | 0x01 | Fan Level 2 |
| 1 | 0x10 | Fan Level 3 |
| 2 | 0x09 | Fan Power |
| 2 | 0x10 | Top light on |
| 4 | 0x10 | Bottom light on |
| 4 | 0x01 | Hood lifting up |
| 5 | 0x90 | After-run active |
| 6 | 0x01 | Hood lowering |

### Critical Requirements

- **MAC OUI**: Hood only accepts Texas Instruments MACs (`88:01:F9` or `30:AF:7E`)
- **GATT Order**: DevInfo → Battery → HID → Berbel Custom Service
- **Legacy Pairing**: LTK only, no IRK
- **BLE Priority**: `esp_coex_preference_set(ESP_COEX_PREFER_BT)` for stable connection
- **NimBLE Stack**: ~100KB heap savings compared to Bluedroid

### MQTT Topics

```
berbel/hood/available          → online / offline
berbel/hood/state              → JSON with current state
berbel/hood/light_up/set       → ON / OFF
berbel/hood/light_down/set     → ON / OFF
berbel/hood/fan/preset/set     → Off / Level 1 / Level 2 / Level 3 / Power
berbel/hood/nachlauf/set       → ON / OFF
berbel/hood/power/set          → (any value)
berbel/hood/position/set       → Up / Down
berbel/hood/move_up/set        → (any value)
berbel/hood/move_down/set      → (any value)
```

---

## 🐛 Troubleshooting

| Problem | Cause | Solution |
|---|---|---|
| `[MQTT] Failed rc=5` | Wrong password | Check MQTT user in HA |
| `[MQTT] Failed rc=-2` | Mosquitto unreachable | Is the add-on running? Correct IP? |
| Hood not connecting | Old bonding keys | Power off hood + "Search for Hood" |
| Connection drops immediately | Keys not persistent | `NVS_PERSIST=1` in platformio.ini |
| `Connecting......` hangs | ESP32 not in flash mode | Hold BOOT button during upload |
| berbel-remote.local missing | ESP32 in setup AP mode | Configure WiFi at 192.168.4.1 |
| OTA fails | BLE/WiFi coexistence | Retry, increase timeout |
| Remote not connecting | Old bonding keys | "Pair Remote" deletes keys automatically |

### LED Status

| LED | Meaning |
|---|---|
| Steady on | Hood + remote both connected ✅ |
| Slow blink (1s) | Only hood connected |
| Fast blink (300ms) | Nothing connected / searching |
| Very fast blink (200ms) | Setup AP mode active |

### Factory Reset

```
http://berbel-remote.local/settings → "Factory Reset"
```

Or temporarily in code:
```cpp
NimBLEDevice::deleteAllBonds();
clearConfig();
ESP.restart();
```

---

## 📁 Project Structure

```
berbel-remote/
├── src/
│   ├── main.cpp          ← Firmware (BLE + WiFi + MQTT + Webserver)
│   └── config.h          ← Empty, configuration via NVS
├── platformio.ini        ← Board + build configuration
├── .gitignore
└── README.md
```

---

## 🔌 Compatibility

| Berbel Series | BLE | Tested |
|---|---|---|
| Skyline Frame / Edge / Curve | ✅ BLE (from Nov. 2020) | ✅ |
| Skyline Sound / Light / Round | ✅ BLE | ⚠️ not officially |
| Ergoline / Glassline / Blockline | ✅ BLE | ⚠️ not officially |
| Smartline / Formline / Firstline | ✅ BLE | ⚠️ not officially |
| Older models (IR remote) | ❌ no BLE | ❌ |

---

## 📜 License

MIT License — Community project, not officially supported by Berbel Ablufttechnik GmbH.

Based on reverse engineering of the BLE protocol of the Berbel BFB 6bT remote.
Original protocol research: [tfohlmeister/berbel-remote](https://github.com/tfohlmeister/berbel-remote)