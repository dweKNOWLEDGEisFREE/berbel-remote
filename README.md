# Berbel BFB 6bT — Remote Control Emulator & Home Assistant Bridge

> Intelligente Steuerung einer Berbel Dunstabzugshaube über ESP32-S3 mit BLE Dual-Role Bridge, Web-Interface, Home Assistant Integration und Alexa-Unterstützung.

---

## 🏗 Architektur

```
Originale FB ──BLE──▶ ESP32-S3 ──BLE──▶ Berbel Haube
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

Der ESP32-S3 läuft gleichzeitig in **zwei BLE-Rollen**:
- **Peripheral** — erscheint für die originale Fernbedienung als Haube
- **Central** — verbindet sich selbst als Fernbedienung mit der echten Haube

Damit können **originale FB, Web-UI, Home Assistant und Alexa gleichzeitig** genutzt werden.

---

## 🚀 Features

### BLE Dual-Role Bridge
- Originale Fernbedienung bleibt vollständig nutzbar
- Befehle der FB werden transparent zur Haube weitergeleitet
- Hauben-Status wird in Echtzeit an FB + MQTT zurückgespiegelt
- Persistentes BLE-Bonding (überlebt Neustarts)
- MAC-Spoofing auf Texas Instruments OUI (von Haube vorausgesetzt)
- Korrektes GATT-Service-Layout (DevInfo → Battery → HID → Berbel Custom)

### Web-Interface
- Moderne, mobile-optimierte Oberfläche unter `http://berbel-remote.local`
- **Steuerungsseite** (`/`) — Lüfter, Licht, Position, Nachlauf, BLE-Pairing
- **Einstellungsseite** (`/settings`) — WiFi + MQTT nachträglich konfigurieren
- **Update-Seite** (`/update`) — OTA Firmware-Update per Drag & Drop im Browser
- Echtzeit-Statusanzeige (2-Sekunden-Polling)
- Dual BLE-Status-Badges (Haube + Fernbedienung)

### Setup-Wizard beim ersten Start
- Kein hardcodiertes WiFi-Passwort im Code
- ESP32 öffnet beim ersten Start einen AccessPoint **„Berbel-Setup"**
- Captive Portal öffnet sich auf iOS/Android automatisch
- WLAN + MQTT über Browser konfigurieren → Einstellungen im Flash gespeichert (NVS)
- Werksreset-Funktion über Web-Interface

### Home Assistant Integration
- MQTT Auto-Discovery — alle Entitäten erscheinen automatisch
- Echtzeit-Statusrückmeldung (auch bei manueller Bedienung an der Haube)
- Alexa-Unterstützung über Nabu Casa oder Emulated Hue

### OTA Updates
- Kabellos über PlatformIO (`pio run -e ota --target upload`)
- Alternativ: Firmware `.bin` direkt im Browser hochladen (`/update`)

---

## 🛠 Hardware

| Teil | Details |
|---|---|
| **ESP32-S3 N16R8 DevKitC-1** | 16MB Flash, 8MB PSRAM, BLE 5.0 Dual-Role |
| USB-C Kabel (Datenkabel) | Für ersten Flash |
| USB-Netzteil 5V/1A | Dauerbetrieb in der Küche |
| Berbel Skyline | Modell mit BLE-Fernbedienung (ab ca. 2020) |

> **Warum ESP32-S3?** Der klassische ESP32 WROOM-32 unterstützt BLE Central + Peripheral nicht stabil gleichzeitig. Der S3 hat deutlich bessere BLE/WiFi Coexistence und ausreichend RAM für den Dual-Role Betrieb.

---

## 📦 Home Assistant Entitäten

| Entität | Typ | Funktion |
|---|---|---|
| `light.oberlicht` | Licht | Oberes Licht ein/aus |
| `light.unterlicht` | Licht | Kochfeld-Beleuchtung |
| `select.luefter` | Select | Aus / Stufe 1 / Stufe 2 / Stufe 3 / Power |
| `switch.nachlauf` | Schalter | Nachlauf-Timer |
| `button.ausschalten` | Button | Haube ausschalten (startet Nachlauf) |
| `button.hochfahren` | Button | Haube hochfahren (Lift-Modelle) |
| `button.herunterfahren` | Button | Haube herunterfahren (Lift-Modelle) |
| `select.position` | Select | Oben / Unten (Lift-Modelle) |
| `binary_sensor.ble_haube` | Sensor | BLE-Verbindung ESP32 → Haube |
| `binary_sensor.ble_fernbedienung` | Sensor | BLE-Verbindung FB → ESP32 |
| `sensor.status_raw` | Sensor | Rohe 9-Byte Statusdaten (Diagnose) |

---

## ⚙️ Einrichtung

### 1. Voraussetzungen

- [VSCode](https://code.visualstudio.com) + [PlatformIO Extension](https://platformio.org/install/ide?install=vscode)
- Home Assistant mit [Mosquitto MQTT Broker Add-on](https://github.com/home-assistant/addons/tree/master/mosquitto)
- MQTT-Benutzer in HA anlegen: *Einstellungen → Personen → Benutzer*

### 2. Projekt klonen

```bash
git clone https://github.com/dein-user/berbel-remote.git
cd berbel-remote
```

### 3. Ersten Flash (USB)

USB-Port in `platformio.ini` eintragen:

```ini
[env:esp32s3dev]
upload_port = /dev/cu.usbmodem1234   # macOS
# upload_port = COM4                 # Windows
# upload_port = /dev/ttyUSB0        # Linux
```

Dann flashen:

```bash
pio run -e esp32s3dev --target upload
```

### 4. Setup-Wizard

1. Mit WLAN **„Berbel-Setup"** verbinden (kein Passwort)
2. Browser öffnet sich automatisch — oder manuell: `http://192.168.4.1`
3. WLAN-Name, Passwort, MQTT-Server und Zugangsdaten eingeben
4. Speichern → ESP32 startet neu und verbindet sich

### 5. Haube koppeln

```
http://berbel-remote.local → Karte "Verbindungen" → "Haube suchen"
```

Haube in Pairing-Modus versetzen (Bluetooth-Taste ca. 5-10 Sekunden halten).

### 6. Fernbedienung koppeln (optional)

```
http://berbel-remote.local → Karte "Verbindungen" → "FB koppeln"
```

Fernbedienung einschalten oder Reset-Taste drücken.

---

## 🔄 OTA Updates

**Via PlatformIO (empfohlen für Entwickler):**
```bash
pio run -e ota --target upload
```

**Via Browser (für alle):**
```
http://berbel-remote.local/update
```
Firmware-Datei (`.bin`) per Drag & Drop hochladen. Die Binary liegt nach dem Build unter:
```
.pio/build/esp32s3dev/firmware.bin
```

---

## 🏠 Alexa einrichten

**Option A — Nabu Casa** (~7 €/Monat, empfohlen):
1. HA → *Einstellungen → Home Assistant Cloud* → Nabu Casa aktivieren
2. Alexa-Skill „Home Assistant" aktivieren und verknüpfen
3. Geräte suchen lassen

**Option B — Emulated Hue** (kostenlos):
```yaml
# configuration.yaml
emulated_hue:
  expose_by_default: false
  entities:
    fan.berbel_luefter:
      name: "Dunstabzugshaube"
      hidden: false
```

**Beispiel-Sprachbefehle:**
```
"Alexa, schalte die Dunstabzugshaube ein"
"Alexa, setze die Dunstabzugshaube auf 66 Prozent"   → Stufe 2
"Alexa, schalte das Küchenlicht ein"
```

---

## 🔧 Technische Details

### BLE Protokoll (Reverse Engineered)

```
Button Press:   [code, 0x00] → Characteristic f004f002
Button Release: [0x00, 0x00] → Characteristic f004f002
Hood Status:    9-Byte Write  ← Characteristic f004f001
Sync Packet:    [0x11 × 9]   (wird ignoriert)
```

**Hood Status Bytes (9 Bytes, Bitmask):**

| Byte | Bit | Bedeutung |
|---|---|---|
| 0 | 0x10 | Lüfter Stufe 1 |
| 1 | 0x01 | Lüfter Stufe 2 |
| 1 | 0x10 | Lüfter Stufe 3 |
| 2 | 0x09 | Lüfter Power |
| 2 | 0x10 | Oberlicht an |
| 4 | 0x10 | Unterlicht an |
| 4 | 0x01 | Haube fährt hoch |
| 5 | 0x90 | Nachlauf aktiv |
| 6 | 0x01 | Haube fährt runter |

### Kritische Anforderungen

- **MAC OUI**: Haube akzeptiert nur Texas Instruments MACs (`88:01:F9` oder `30:AF:7E`)
- **GATT Reihenfolge**: DevInfo → Battery → HID → Berbel Custom Service
- **Legacy Pairing**: LTK only, kein IRK
- **BLE Priorität**: `esp_coex_preference_set(ESP_COEX_PREFER_BT)` für stabile Verbindung
- **NimBLE Stack**: ~100KB Heap-Ersparnis gegenüber Bluedroid

### MQTT Topics

```
berbel/hood/available          → online / offline
berbel/hood/state              → JSON mit aktuellem Zustand
berbel/hood/light_up/set       → ON / OFF
berbel/hood/light_down/set     → ON / OFF
berbel/hood/fan/preset/set     → Aus / Stufe 1 / Stufe 2 / Stufe 3 / Power
berbel/hood/nachlauf/set       → ON / OFF
berbel/hood/power/set          → (beliebiger Wert)
berbel/hood/position/set       → Oben / Unten
berbel/hood/move_up/set        → (beliebiger Wert)
berbel/hood/move_down/set      → (beliebiger Wert)
```

---

## 🐛 Troubleshooting

| Problem | Ursache | Lösung |
|---|---|---|
| `[MQTT] Failed rc=5` | Falsches Passwort | MQTT-User in HA prüfen |
| `[MQTT] Failed rc=-2` | Mosquitto nicht erreichbar | Add-on läuft? IP korrekt? |
| Haube verbindet nicht | Alte Bonding-Keys | Haube stromlos + „Haube suchen" |
| Verbindung bricht sofort ab | Keys nicht persistent | `NVS_PERSIST=1` in platformio.ini |
| `Connecting......` hängt | ESP32 nicht im Flash-Modus | BOOT-Taste halten beim Upload |
| berbel-remote.local fehlt | ESP32 im Setup-AP Modus | WiFi konfigurieren unter 192.168.4.1 |
| OTA schlägt fehl | BLE/WiFi Coexistence | Nochmal versuchen, Timeout erhöhen |
| FB verbindet nicht | Alte Bonding-Keys | „FB koppeln" löscht Keys automatisch |

### LED-Status

| LED | Bedeutung |
|---|---|
| Dauerhaft an | Haube + FB beide verbunden ✅ |
| Langsam blinkt (1s) | Nur Haube verbunden |
| Schnell blinkt (300ms) | Nichts verbunden / Suche läuft |
| Sehr schnell blinkt (200ms) | Setup-AP Modus aktiv |

### Werksreset

```
http://berbel-remote.local/settings → "Werksreset"
```

Oder im Code temporär:
```cpp
NimBLEDevice::deleteAllBonds();
clearConfig();
ESP.restart();
```

---

## 📁 Projektstruktur

```
berbel-remote/
├── src/
│   ├── main.cpp          ← Firmware (BLE + WiFi + MQTT + Webserver)
│   └── config.h          ← Leer, Konfiguration läuft über NVS
├── platformio.ini        ← Board + Build-Konfiguration
├── .gitignore
└── README.md
```

---

## 🔌 Kompatibilität

| Berbel-Serie | BLE | Getestet |
|---|---|---|
| Skyline Frame / Edge / Curve | ✅ BLE (ab Nov. 2020) | ✅ |
| Skyline Sound / Light / Round | ✅ BLE | ⚠️ nicht offiziell |
| Ergoline / Glassline / Blockline | ✅ BLE | ⚠️ nicht offiziell |
| Smartline / Formline / Firstline | ✅ BLE | ⚠️ nicht offiziell |
| Ältere Modelle (IR-Fernbedienung) | ❌ kein BLE | ❌ |

---

## 📜 Lizenz

MIT License — Community-Projekt, nicht offiziell von Berbel Ablufttechnik GmbH unterstützt.

Basierend auf Reverse Engineering des BLE-Protokolls der Berbel BFB 6bT Fernbedienung.
Ursprüngliches Protokoll-Research: [tfohlmeister/berbel-remote](https://github.com/tfohlmeister/berbel-remote)
