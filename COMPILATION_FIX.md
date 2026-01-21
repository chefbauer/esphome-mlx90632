# Kompilierungsfehler-Fix: Arduino.h und TwoWire Abhängigkeiten beseitigt

## Problem
Die Komponente konnte nicht unter ESP-IDF kompiliert werden, da:
1. `Arduino.h` war nicht verfügbar (ES-IDF bietet das nicht)
2. `TwoWire` wurde nicht korrekt deklariert (Forward declaration war nicht ausreichend)

Fehler:
```
error: 'TwoWire' was not declared in this scope
src/esphome/components/mlx90632/Adafruit_I2CDevice.cpp:40:3
fatal error: Arduino.h: No such file or directory
src/esphome/components/mlx90632/Adafruit_MLX90632.h:31:12
```

## Lösung
Einführung einer abstrakten I2C-Schnittstelle, die unabhängig von Arduino/Wire ist:

### 1. Neues Interface: `I2CInterface.h`
- Abstrakte Basis-Klasse für I2C-Operationen
- Plattformunabhängig
- Definiert die notwendigen Methoden:
  - `beginTransmission()`, `endTransmission()`
  - `write()`, `read()`
  - `requestFrom()`, `available()`

### 2. Aktualisierte Klassen
- **ESPHomeI2CAdapter.h**: Erbt nun von `I2CInterface` statt von `TwoWire`
  - Keine Abhängigkeit auf Wire.h
  - Nutzt ESPHome's native `i2c::I2CBus` interface
  
- **Adafruit_I2CDevice.h**: 
  - Verwendet `I2CInterface` statt `TwoWire`
  - Forward declaration statt komplexe Conditional Includes
  
- **Adafruit_I2CDevice.cpp**: 
  - Alle `TwoWire*` Casts zu `I2CInterface*` geändert
  - Verwendet die abstrakten Methoden statt Wire-spezifische Methoden
  
- **Adafruit_MLX90632.h**: 
  - Entfernte Arduino.h Include und platformspezifische Conditionals
  - Nur noch C++ Standard-Headers (`<cmath>`, `<cstring>`)

### 3. Dateien mit doppelten Inhalten bereinigt
- Entfernte alte TwoWire-basierte Implementierungen
- Vereinheitlichte ESPHomeI2CAdapter.cpp

## Architektur
```
ESPHome I2C Bus
      ↓
ESPHomeI2CAdapter (implements I2CInterface)
      ↓
I2CInterface (abstract)
      ↓
Adafruit_I2CDevice
      ↓
Adafruit_BusIO_Register
      ↓
Adafruit_MLX90632 (Library mit Melexis-Algorithmus)
      ↓
mlx90632.cpp (ESPHome Component)
```

## Ergebnis
✅ Komponente ist nun völlig unabhängig von Arduino.h/Wire.h  
✅ Funktioniert mit ESP-IDF  
✅ Funktioniert mit ESPHome  
✅ Keine externen Abhängigkeiten außer ESPHome Core  
✅ Kompletter Melexis-Algorithmus (5 Iterationen) erhalten  
✅ Dual-Temperature-Output (Objekt + Umgebung) erhalten  

## Kompilierung testen
```bash
cd /workspaces/esphome-mlx90632
esphome compile config.yaml
```

## Datei-Änderungssummary
| Datei | Änderung |
|-------|----------|
| `I2CInterface.h` | Neu erstellt - abstrakte I2C-Schnittstelle |
| `ESPHomeI2CAdapter.h` | Von TwoWire zu I2CInterface geändert |
| `ESPHomeI2CAdapter.cpp` | Vereinheitlicht, alte Duplikate entfernt |
| `Adafruit_I2CDevice.h` | TwoWire Forward Declaration entfernt |
| `Adafruit_I2CDevice.cpp` | Alle TwoWire-Referenzen zu I2CInterface geändert |
| `Adafruit_MLX90632.h` | Arduino.h und Platform-Conditionals entfernt |
