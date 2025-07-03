# Suction Controller

This document describes the suction controller module for Klipper. The suction controller provides PWM-based vacuum pump control designed for pick-and-place and fluid dispensing applications. It is built upon the normal fan.py module.

## Configuration

To use the suction controller, add the following configuration section to your printer configuration file:

```
[suction_controller]
pin: gpio18
# + other fan.py optional cfgs
```

### Required Parameters

- **pin**: The PWM-capable GPIO pin controlling the vacuum pump (required)

### Optional Parameters

- **max_power**: Maximum power output (0.0-1.0, default: 1.0)
- **shutdown_speed**: Emergency shutdown speed (0.0-1.0, default: 0)
- **cycle_time**: PWM cycle time in seconds (default: 0.010)
- **hardware_pwm**: Use hardware PWM instead of software PWM (default: False)
- **kick_start_time**: Full power startup duration (default: 0.1s)
- **off_below**: Minimum speed threshold (default: 0.0)
- **enable_pin**: Optional pump enable control pin

## G-Code Commands

The suction controller is compatible with standard fan control G-code commands:

### M106 - Set Pump Power
`M106 [P<value>]`

Sets the vacuum pump power level.

Parameters:
- **P**: Power level from 0-255 (0 = off, 255 = maximum power)

Examples:
```gcode
M106 P255    ; Full pump power (100%)
M106 P128    ; 50% pump power  
M106 P64     ; 25% pump power
M106 P0      ; Turn off pump
```

### M107 - Turn Off Pump
`M107`

Turns off the vacuum pump (equivalent to `M106 P0`).

Example:
```gcode
M107         ; Turn off vacuum pump
```

## Status Information

The suction controller publishes status information that can be queried via the Moonraker API or WebSocket interface.

### API Endpoint
```
GET http://<printer-ip>:7125/printer/objects/query?suction_controller
```

### Status Object Format
```json
{
  "result": {
    "eventtime": 359.007609619,
    "status": {
      "suction_controller": {
        "Pump Power": 1.0
      }
    }
  }
}
```

### Status Fields

- **Pump Power**: Current pump power setting (0.0-1.0)
  - `0.0`: Pump is off
  - `1.0`: Pump at maximum power
  - Values between 0.0-1.0 indicate proportional power levels


## Usage Examples

### Basic Vacuum Control

```gcode
# Turn on vacuum pump at 75% power
M106 P191

# Wait for vacuum to build up
G4 P2000

# Turn off vacuum
M107
```

### Graduated Vacuum Control

```gcode
# Gentle vacuum for delicate components
M106 P64         ; 25% power - delicate parts

# Standard vacuum for normal components  
M106 P128        ; 50% power - standard parts

# Strong vacuum for heavy components
M106 P255        ; 100% power - heavy parts
```

## Integration with Pressure Monitoring

When used with the XGZP6847D pressure sensor, you can monitor vacuum levels:

```gcode
# Start vacuum pump
M106 P255

# Monitor vacuum buildup
DO_PT_MEASURE
G4 P100
TELL_PT_MEASURE

# Proceed when adequate vacuum achieved
```

Query both modules simultaneously:
```
GET http://<printer-ip>:7125/printer/objects/query?suction_controller&XGZP6847D_sensor
```

## Version History

### Version 0.1 (Current)
- Initial implementation
- Standard M106/M107 G-code compatibility  
- PWM-based pump control with configurable parameters
- Moonraker API integration
- Status reporting functionality


## Related Documentation
- [Config example](config/example-voltera-suction_controller.cfg)
- [XGZP6847D Pressure Sensor](docs/voltera-XGZP6847D_sensor.md)
- [Klipper Config Reference](Config_Reference.md)
- [PWM Output Pins](Config_Reference.md#pwm_output_pin)