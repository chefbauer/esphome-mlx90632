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

## Example Configuration

```yaml
sensor:
  - platform: mlx90632
    name: MLX90632 Temperature Sensor
    update_interval: 30s
    address: 0x3A  # Optional, default is 0x3A
    
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

- **update_interval** (*Optional*, int): How often to update the sensor. Defaults to 60s.
- **address** (*Optional*, int): The I2C address of the sensor. Defaults to 0x3A.
- **object_temperature** (*Optional*): The object (IR) temperature sensor. At least one temperature sensor must be configured.
  - **name** (*Optional*, string): The name of the object temperature sensor.
- **ambient_temperature** (*Optional*): The ambient (reference) temperature sensor. At least one temperature sensor must be configured.
  - **name** (*Optional*, string): The name of the ambient temperature sensor.
