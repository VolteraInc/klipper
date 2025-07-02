# XGZP6847D Pressure Sensor

This document describes the XGZP6847D vacuum pressure sensor module for Klipper. The XGZP6847D is an I2C-based digital pressure sensor commonly used in vacuum applications and fluid dispensing systems.

## Configuration

To use the XGZP6847D pressure sensor, add the following configuration section to your printer configuration file:

```
[XGZP6847D_sensor]
i2c_bus: i2c0e
#i2c_mcu: mcu
#   The name of the micro-controller that the I2C bus is connected to.
#   The default is "mcu".
#status_update_ms: 200
#   How often to update the sensor status (in milliseconds).
#   The default is 200ms.
```

### Configuration Parameters

- **i2c_bus**: The I2C bus name that the sensor is connected to (required)
- **i2c_mcu**: The micro-controller managing the I2C bus (optional, default: "mcu")  
- **status_update_ms**: Status update interval in milliseconds (optional, default: 200)

## G-Code Commands

The following G-code commands are available when the XGZP6847D sensor is configured:

### DO_PT_MEASURE
`DO_PT_MEASURE`

Initiates an asynchronous pressure and temperature measurement cycle. The sensor will begin sampling and the results will be available after the measurement completes.

### TELL_PT_MEASURE  
`TELL_PT_MEASURE`

Returns the latest pressure and temperature readings from the sensor. This command provides immediate access to the most recent measurements without triggering a new measurement cycle.

Example output:
```
XGZP6847D sensor readings: Pressure=-2500 Pa, Temperature=23.45 C
```

## Status Information

The sensor publishes status information that can be queried via the Moonraker API or WebSocket interface.

### API Endpoint
```
GET http://<printer-ip>:7125/printer/objects/query?XGZP6847D_sensor
```

### Status Object Format
```json
{
  "result": {
    "eventtime": 443.538806217,
    "status": {
      "XGZP6847D_sensor": {
        "Sampling Status": true,
        "Pressure": -2500,
        "Temperature": 23.45
      }
    }
  }
}
```

### Status Fields

- **Sampling Status**: Boolean indicating if a measurement is currently in progress
  - `true`: Sensor is actively taking a measurement
  - `false`: Sensor is ready for new commands
- **Pressure**: Current pressure reading in Pascals (Pa)
  - Negative values indicate vacuum (below atmospheric pressure)
  - Positive values indicate pressure above atmospheric
- **Temperature**: Current temperature reading in degrees Celsius

## Hardware Setup

### Wiring

The XGZP6847D sensor requires the following connections:

- **VCC**: 3.3V or 5V power supply
- **GND**: Ground connection  
- **SDA**: I2C data line
- **SCL**: I2C clock line

### I2C Address

The sensor uses the factory default I2C address `0x6D`. This address is fixed and cannot be changed.

### Supported Pressure Ranges

The XGZP6847D sensor supports various pressure ranges. The module is configured for:

- **Model**: XGZP6847D100KPG
- **Range**: -100kPa to +100kPa (vacuum to positive pressure)
- **Resolution**: High precision digital output

## Usage Examples

### Basic Vacuum Monitoring

```gcode
# Take a pressure measurement
DO_PT_MEASURE

# Wait for measurement to complete (typically 5-20ms)
G4 P20

# Read the results
TELL_PT_MEASURE
```

### Automated Vacuum Control

The sensor can be integrated with other Klipper modules for automated vacuum control in pick-and-place or dispensing applications:

```gcode
# Start vacuum pump
M106 P255

# Monitor vacuum buildup
DO_PT_MEASURE
G4 P20
TELL_PT_MEASURE

```

## Version History

### Version 0.1 (Current)
- Initial implementation
- Basic pressure and temperature measurement
- I2C communication support
- Moonraker API integration
- Asynchronous measurement capability

## Related Documentation

- [Configuration example](config/example-voltera-XGZP6847D_sensor.cfg)
