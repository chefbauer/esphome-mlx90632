# esphome-mlx90632

## English

A fast and easy-to-use ESPHome component for the Melexis MLX90632 Far Infrared Temperature Sensor. This component provides seamless I2C integration with ESPHome, including support for dual temperature outputs (object IR and ambient reference temperatures), configurable measurement modes, and EEPROM-optimized refresh rate settings.

### Features

- **Dual Temperature Outputs**: Measure both infrared (object) and ambient (reference) temperatures simultaneously
- **Measurement Modes**: Switch between Medical (±50°C) and Extended Range modes at runtime
- **Configurable Refresh Rates**: 0.5Hz to 64Hz with intelligent EEPROM protection to minimize wear
- **Easy Integration**: Works as an external ESPHome component with minimal setup
- **I2C Support**: Native ESPHome I2C integration with the Adafruit MLX90632 library

### Quick Start

Add to your ESPHome configuration:

```yaml
external_components:
  - source: github://chefbauer/esphome-mlx90632@main
    components: [mlx90632]

sensor:
  - platform: mlx90632
    name: "IR Temperature Sensor"
    update_interval: 30s
    object_temperature:
      name: "Object Temperature"
    ambient_temperature:
      name: "Ambient Temperature"

i2c:
  sda: GPIO4
  scl: GPIO5
```

See [components/mlx90632/README.md](components/mlx90632/README.md) for detailed documentation.

---

## Deutsch

Eine schnelle und benutzerfreundliche ESPHome-Komponente für den Melexis MLX90632 Far-Infrarot-Temperatursensor. Diese Komponente bietet nahtlose I2C-Integration mit ESPHome, einschließlich Unterstützung für duale Temperaturausgaben (Infrarot-Objekt- und Umgebungsreferenztemperaturen), konfigurierbare Messmodi und EEPROM-optimierte Aktualisierungsrateneinstellungen.

### Features

- **Duale Temperaturausgaben**: Messe gleichzeitig Infrarot- (Objekt-) und Umgebungs- (Referenz-) Temperaturen
- **Messmodi**: Wechsle zur Laufzeit zwischen Medical (±50°C) und Extended Range Modi
- **Konfigurierbare Aktualisierungsraten**: 0,5Hz bis 64Hz mit intelligentem EEPROM-Schutz zur Minimierung von Verschleiß
- **Einfache Integration**: Funktioniert als externe ESPHome-Komponente mit minimalem Setup
- **I2C-Unterstützung**: Native ESPHome I2C-Integration mit der Adafruit MLX90632-Bibliothek

### Schnellstart

Füge dies zu deiner ESPHome-Konfiguration hinzu:

```yaml
external_components:
  - source: github://chefbauer/esphome-mlx90632@main
    components: [mlx90632]

sensor:
  - platform: mlx90632
    name: "IR-Temperatursensor"
    update_interval: 30s
    object_temperature:
      name: "Objekttemperatur"
    ambient_temperature:
      name: "Umgebungstemperatur"

i2c:
  sda: GPIO4
  scl: GPIO5
```

Siehe [components/mlx90632/README.md](components/mlx90632/README.md) für detaillierte Dokumentation.

---

**Development Note**: This component was rapidly prototyped and developed using GitHub Copilot and Claude Haiku, enabling quick iteration and efficient integration of the Adafruit MLX90632 library with ESPHome's I2C architecture.

**Entwicklungshinweis**: Diese Komponente wurde schnell mit GitHub Copilot und Claude Haiku prototypisiert und entwickelt, was eine schnelle Iteration und effiziente Integration der Adafruit MLX90632-Bibliothek mit ESPHomes I2C-Architektur ermöglichte.
