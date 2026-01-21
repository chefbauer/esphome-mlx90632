# MLX90632 Far Infrared Temperature Sensor Component

This ESPHome component provides support for the Melexis MLX90632 Far Infrared Temperature Sensor over I2C.

## Features

- Full MLX90632 Far Infrared Temperature Sensor support
- Integrated Adafruit MLX90632 library with ESPHome I2C adapter
- Automatic calibration constant loading from EEPROM
- Support for Medical and Extended Range measurement modes
- Continuous measurement mode
- Adjustable refresh rates (0.5Hz to 64Hz)
- Object and ambient temperature readings

## Dependencies

- [Adafruit BusIO](https://github.com/adafruit/Adafruit_BusIO)
- [Adafruit MLX90632](https://github.com/adafruit/Adafruit_MLX90632)

## Example Configuration

```yaml
sensor:
  - platform: mlx90632
    name: MLX90632 Temperature
    update_interval: 60s
    address: 0x3A  # Optional, default is 0x3A

i2c:
  sda: 4
  scl: 5
  frequency: 100kHz
```

## How It Works

The component uses:
1. **ESPHomeI2CAdapter** - A wrapper that adapts ESPHome's I2C interface to Arduino's TwoWire format required by Adafruit BusIO
2. **Adafruit_MLX90632** - The official Adafruit library for sensor control
3. **ESPHome PollingComponent** - For periodic sensor updates

The sensor publishes the object temperature as the main value, with ambient temperature available for logging.

## API Reference

The underlying Adafruit_MLX90632 library provides access to:
- `getObjectTemperature()` - Read object temperature
- `getAmbientTemperature()` - Read ambient temperature  
- `setMode()` - Set measurement mode (Halt, Sleeping Step, Step, Continuous)
- `setMeasurementSelect()` - Set measurement type (Medical or Extended Range)
- `setRefreshRate()` - Set measurement refresh rate
- `isNewData()` - Check if new measurement data is available
