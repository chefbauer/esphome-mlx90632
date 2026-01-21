# MLX90632 Far Infrared Temperature Sensor Component

This ESPHome component provides support for the Melexis MLX90632 Far Infrared Temperature Sensor over I2C.

## Features

- Full MLX90632 Far Infrared Temperature Sensor support
- Integrated Adafruit MLX90632 library with ESPHome I2C adapter
- Automatic calibration constant loading from EEPROM
- Support for Medical and Extended Range measurement modes
- Continuous measurement mode
- Adjustable refresh rates (0.5Hz to 64Hz)
- **Dual temperature outputs: Object (IR) and Ambient (Reference) temperatures**

## Dependencies

- [Adafruit BusIO](https://github.com/adafruit/Adafruit_BusIO)
- [Adafruit MLX90632](https://github.com/adafruit/Adafruit_MLX90632)

## How It Works

The component uses:
1. **ESPHomeI2CAdapter** - A wrapper that adapts ESPHome's I2C interface to Arduino's TwoWire format required by Adafruit BusIO
2. **Adafruit_MLX90632** - The official Adafruit library for sensor control
3. **ESPHome PollingComponent** - For periodic sensor updates

The sensor can publish both object temperature (measured IR) and ambient temperature (reference temperature) as separate sensor values.

## Measurement Modes

The MLX90632 supports two measurement modes that can be switched via the `measurement_select` configuration:

### Medical Mode (Default)
- **Range**: ±50°C around ambient temperature
- **Use case**: Standard temperature measurements
- **Accuracy**: High precision within the range

### Extended Range Mode
- **Range**: Wider measurement range
- **Use case**: When you need measurements beyond ±50°C deviation from ambient
- **Note**: Can be used on both medical and standard hardware versions

**Important**: Both modes can be used on any MLX90632, regardless of whether it's labeled as "medical" or "standard" version. The hardware version primarily affects the factory calibration, but the measurement select mode can be switched in software.

## EEPROM Protection

The component is designed to minimize EEPROM writes:

- **measurement_select**: Uses RAM-only control register (no EEPROM write)
- **refresh_rate**: Only writes to EEPROM if different from current value (change detection)
- This prevents excessive wear on the sensor's EEPROM over time

## Example Configuration

```yaml
sensor:
  - platform: mlx90632
    name: MLX90632 Temperature Sensor
    update_interval: 30s
    address: 0x3A                    # Optional (default: 0x3A)
    measurement_select: medical      # Optional: medical | extended_range
    refresh_rate: 2hz                # Optional: 0.5hz | 1hz | 2hz | 4hz | 8hz | 16hz | 32hz | 64hz
    emissivity: 0.95                 # Optional (default: 1.0, range: 0.0 - 1.0)
    
    object_temperature:
      name: "Object Temperature"
      unit_of_measurement: "°C"
      
    ambient_temperature:
      name: "Ambient Temperature"
      unit_of_measurement: "°C"

i2c:
  sda: 4
  scl: 5
  frequency: 100kHz
```

### Configuration Variables

- **update_interval** (*Optional*, time): How often to update the sensor from ESPHome. Defaults to 60s.
- **address** (*Optional*, int): The I2C address of the sensor. Defaults to 0x3A.
- **measurement_select** (*Optional*, string): The measurement mode. Can be `medical` (default) or `extended_range`.
  - `medical`: Standard medical mode with ±50°C range
  - `extended_range`: Extended range measurement mode (can be used even with medical hardware version)
- **refresh_rate** (*Optional*, string): How fast the sensor takes measurements. Defaults to `2hz`.
  - Available rates: `0.5hz` (2000ms), `1hz` (1000ms), `2hz` (500ms), `4hz` (250ms), 
    `8hz` (125ms), `16hz` (62ms), `32hz` (31ms), `64hz` (16ms)
  - **Note**: This is the sensor's internal measurement rate, independent of `update_interval`
- **emissivity** (*Optional*, float): Software-based emissivity correction factor. Defaults to `1.0`.
  - **Range**: 0.0 to 1.0
  - **Description**: Emissivity corrects the temperature reading based on the material being measured
  - **Examples**: 
    - 1.0 = Perfect blackbody (default, for most materials)
    - 0.95 = Most painted surfaces, non-metals
    - 0.85-0.95 = Most common materials
    - 0.6-0.8 = Reflective metals, polished surfaces
    - 0.3-0.6 = Shiny metals, foil
- **object_temperature** (*Optional*): The object (IR) temperature sensor. At least one temperature sensor must be configured.
  - **name** (*Optional*, string): The name of the object temperature sensor.
- **ambient_temperature** (*Optional*): The ambient (reference) temperature sensor. At least one temperature sensor must be configured.
  - **name** (*Optional*, string): The name of the ambient temperature sensor.

## Using as External Component

Add this to your ESPHome configuration to use this component:

```yaml
external_components:
  - source: github://chefbauer/esphome-mlx90632@main
    components: [mlx90632]

sensor:
  - platform: mlx90632
    name: "IR Temperature"
    update_interval: 30s
    object_temperature:
      name: "Object Temperature"
    ambient_temperature:
      name: "Ambient Temperature"

i2c:
  sda: 4
  scl: 5
```

## Als External Component verwenden

Füge dies zu deiner ESPHome-Konfiguration hinzu:

```yaml
external_components:
  - source: github://chefbauer/esphome-mlx90632@main
    components: [mlx90632]

sensor:
  - platform: mlx90632
    name: "IR-Temperatur"
    update_interval: 30s
    object_temperature:
      name: "Objekttemperatur"
    ambient_temperature:
      name: "Umgebungstemperatur"

i2c:
  sda: 4
  scl: 5
```
