Berbel BFB 6bT - Remote Control Emulator & Home Assistant Bridge
Dieses Projekt ermöglicht die intelligente Steuerung einer Berbel Dunstabzugshaube (z. B. Skyline-Serie) über einen ESP32. Es emuliert die originale Bluetooth-Fernbedienung und fungiert als Brücke (Bridge) zu modernen Smart-Home-Systemen.

🚀 Was es macht
Fernbedienungs-Emulation: Der ESP32 gibt sich gegenüber der Haube als originale Fernbedienung aus (inkl. MAC-Spoofing und korrektem GATT-Service-Layout).

Home Assistant Integration: Dank MQTT Auto-Discovery erscheinen alle Funktionen (Licht, Lüfterstufen, Lift-Position) automatisch als Entitäten in Home Assistant.

Web-Interface: Bietet eine moderne, mobile-optimierte Weboberfläche zur direkten Steuerung und zum Pairing im Browser.

Status-Feedback: Durch Reverse-Engineering des Protokolls werden Statusänderungen der Haube (z. B. manuelles Schalten an der Haube selbst) in Echtzeit an MQTT/HA zurückgemeldet.

🛠 Technische Highlights
Protokoll: Bluetooth Low Energy (BLE) unter Verwendung des effizienten NimBLE-Stacks (~100KB Heap-Ersparnis).

Features: * Steuerung von Ober- und Unterlicht.

Schaltung der Lüfterstufen (1-3 + Power) und des Nachlaufs.

Unterstützung für Lift-Funktion (Hoch-/Runterfahren bei Skyline-Modellen).

OTA (Over-the-Air) Updates für einfache Wartung.

Spezial-Anforderungen: Implementiert spezifische Timeouts, Radio-Priorisierung und TI-OUI MAC-Adressen, um eine stabile Verbindung zur Berbel-Steuerung zu garantieren.

📦 HA Entitäten
light.oberlicht / light.unterlicht

select.luefter (Aus, Stufe 1-3, Power)

button.ausschalten / switch.nachlauf

binary_sensor.ble_verbindung (Status-Diagnose)
