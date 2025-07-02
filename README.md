Welcome to the Klipper project!

[![Klipper](docs/img/klipper-logo-small.png)](https://www.klipper3d.org/)

https://www.klipper3d.org/

The Klipper firmware controls 3d-Printers. It combines the power of a
general purpose computer with one or more micro-controllers. See the
[features document](https://www.klipper3d.org/Features.html) for more
information on why you should use the Klipper software.

Start by [installing Klipper software](https://www.klipper3d.org/Installation.html).

Klipper software is Free Software. See the [license](COPYING) or read
the [documentation](https://www.klipper3d.org/Overview.html). We
depend on the generous support from our
[sponsors](https://www.klipper3d.org/Sponsors.html).

##
## Voltera added Features:

### XGZP6847D Pressure Sensor V0.1
#### Commands
- **`DO_PT_MEASURE`** - Start an asynchronous pressure and temperature measurement cycle
- **`TELL_PT_MEASURE`** - Return the latest pressure and temperature readings

### Status Objects Example
```json
{
  "XGZP6847D_sensor": {
    "Sampling Status": false,
    "Pressure": -2500,
    "Temperature": 23.45
  }
}
```
See [XGZP6847D_sensor.md](docs/XGZP6847D_sensor.md) for detailed documentation.
##
### Suction Controller V0.1
#### Commands
- **`M106 [P<value>]`** - Set vacuum pump power (P: 0-255, where 255 = 100% power)
- **`M107`** - Turn off vacuum pump (equivalent to M106 P0)

### Status Objects Example
{
  "suction_controller": {
    "Pump Power": 0.75
  }
}

See [suction_controller.md](docs/suction_controller.md) for detailed documentation.