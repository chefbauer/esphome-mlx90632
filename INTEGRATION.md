# MLX90632 ESPHome Component Integration

## Übersicht

Diese ESPHome-Komponente integriert den Melexis MLX90632 Far Infrared Temperature Sensor mit vollständiger Unterstützung der Adafruit-Library.

## Komponenten-Architektur

```
┌─────────────────────────────────────────────────────────┐
│            ESPHome MLX90632 Component                    │
├─────────────────────────────────────────────────────────┤
│  mlx90632.cpp / mlx90632.h (ESPHome Integration)        │
│  - PollingComponent für zyklische Messungen             │
│  - I2CDevice für ESPHome I2C-Bus Integration            │
│  - Sensor für Wertveröffentlichung                      │
├─────────────────────────────────────────────────────────┤
│            ESPHomeI2CAdapter                             │
├─────────────────────────────────────────────────────────┤
│  - TwoWire-Adapter für Arduino-Kompatibilität           │
│  - Konvertiert ESPHome I2C zu Arduino Wire-Protokoll    │
├─────────────────────────────────────────────────────────┤
│         Adafruit_MLX90632 Library                        │
├─────────────────────────────────────────────────────────┤
│  - Vollständige Sensor-Kontrolle                        │
│  - Kalibrierungskonstanten-Verwaltung                   │
│  - Temperaturberechnungen (Stefan-Boltzmann)            │
│  - EEPROM-Zugriff                                       │
├─────────────────────────────────────────────────────────┤
│        Adafruit_BusIO (extern)                           │
├─────────────────────────────────────────────────────────┤
│       ESPHome I2C Bus (native)                           │
└─────────────────────────────────────────────────────────┘
```

## Dateistruktur

```
components/mlx90632/
├── __init__.py                    # Python Komponenten-Manifest
├── sensor.py                      # Python Konfigurationsschema
├── mlx90632.h                     # ESPHome C++ Header
├── mlx90632.cpp                   # ESPHome C++ Implementierung
├── ESPHomeI2CAdapter.h            # I2C Adapter Header
├── ESPHomeI2CAdapter.cpp          # I2C Adapter Implementierung
├── Adafruit_MLX90632.h            # Adafruit Library Header
├── Adafruit_MLX90632.cpp          # Adafruit Library Implementation
├── CMakeLists.txt                 # Build-Konfiguration
├── README.md                      # Dokumentation
└── INTEGRATION.md                 # Diese Datei
```

## ESPHomeI2CAdapter

Der `ESPHomeI2CAdapter` ist ein kritisches Bindeglied:

### Zweck
- Implementiert die Arduino `TwoWire`-Schnittstelle
- Übersetzt Arduino-I2C-Operationen in ESPHome-I2C-Operationen
- Erlaubt der Adafruit-Library, mit ESPHome's I2C-Bus zu kommunizieren

### Implementierte Methoden
```cpp
// Wire Schreiboperationen
beginTransmission(addr)     // Start I2C-Transaktion
write(data)                 // Schreibe Byte
write(data, size)           // Schreibe Buffer
endTransmission(sendStop)   // Ende I2C-Transaktion

// Wire Leseoperationen
requestFrom(addr, qty)      // Fordere Daten vom Gerät an
read()                       // Lies nächstes Byte
peek()                       // Schaue auf nächstes Byte
flush()                      // Leere Buffer
```

## Adafruit_MLX90632 Library

Die Library wurde von der Adafruit Version angepasst:
- Entfernt Arduino-spezifische Serial.print() Debug-Ausgaben
- Integriert ESPHome-Logging via ESP_LOGD/ESP_LOGE
- Unterstützt alle Messungsmodi und Kalibrierungen

### Wichtige Funktionen
- `begin()` - Initialisierung und Kalibrierungsladen
- `getAmbientTemperature()` - Umgebungstemperatur
- `getObjectTemperature()` - Objekttemperatur
- `setMode()` - Messmodus konfigurieren
- `setRefreshRate()` - Aktualisierungsrate setzen
- `isNewData()` - Neue Daten verfügbar?

## ESPHome Komponente Integration

### MLX90632Sensor Klasse
```cpp
class MLX90632Sensor : public sensor::Sensor, 
                       public PollingComponent, 
                       public i2c::I2CDevice
```

#### setup()
- Erstellt I2C-Adapter
- Initialisiert Adafruit_MLX90632 Bibliothek
- Konfiguriert Sensor-Parameter
- Lädt Kalibrierungskonstanten

#### update()
- Liest aktuelle Temperaturwerte
- Veröffentlicht Objekttemperatur als Sensor-Wert
- Loggt Umgebungstemperatur zur Diagnose

#### dump_config()
- Zeigt Konfigurationsinformationen
- Listet I2C-Adresse und Update-Intervall auf

## Python-Konfiguration (sensor.py)

```python
CONFIG_SCHEMA = (
    sensor.sensor_schema(
        MLX90632Sensor,
        unit_of_measurement=UNIT_TEMPERATURE,
        icon=ICON_EMPTY,
        accuracy_decimals=2,
    )
    .extend(cv.polling_component_schema("60s"))  # Standard: 60 Sekunden
    .extend(i2c.i2c_device_schema(0x3A))        # Standard-I2C-Adresse
)
```

## Verwendungsbeispiel

```yaml
sensor:
  - platform: mlx90632
    name: "Raum Temperatur"
    update_interval: 30s
    address: 0x3A
    
i2c:
  sda: 21
  scl: 22
  frequency: 100kHz
```

## Bekannte Abhängigkeiten

### Externe Libraries
1. **Adafruit_BusIO** - Basis I2C Register-Abstraktion
   - Muss in ESPHome verfügbar sein
   - Implementiert `Adafruit_I2CDevice` und `Adafruit_BusIO_Register`

2. **Adafruit_MLX90632** - Original Adafruit Library
   - Integriert direkt in Komponente
   - Enthält Sensor-Arithmetik und -Logik

### ESPHome-Komponenten
- `i2c` - I2C-Bus-Verwaltung
- `sensor` - Sensor-Basisklasse

## Debugging

Aktiviere Debugging in `Adafruit_MLX90632.cpp`:
```cpp
#define MLX90632_DEBUG  // Entkommentieren für Debug-Ausgaben
```

ESPHome-Logging nutzt standardmäßig `ESP_LOGD` für Debug-Meldungen.

## Erweiterungsmöglichkeiten

1. **Zusätzliche Sensor-Ausgänge**
   - Umgebungstemperatur als separater Sensor
   - Messstatus/Zyklus-Position als binärer Sensor

2. **Konfigurierbare Parameter**
   - Messmodus (Medical/Extended Range)
   - Aktualisierungsrate
   - I2C-Adresse

3. **Diagnostik**
   - EEPROM-Version auslesen
   - Gerätestatus überwachen
   - Beschäftigungsflag prüfen

## Technische Referenzen

- [MLX90632 Datenblatt](https://www.melexis.com/en/product/MLX90632/Far-Infrared-Thermometer)
- [Adafruit MLX90632 Library](https://github.com/adafruit/Adafruit_MLX90632)
- [Adafruit BusIO](https://github.com/adafruit/Adafruit_BusIO)
- [ESPHome I2C Documentation](https://esphome.io/components/i2c.html)
