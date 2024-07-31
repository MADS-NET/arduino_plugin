# Arduino plugin for MADS

This is a source plugin for MADS that reads a JSON string published by an Arduino connected via serial port on the MADS network.

## Installation

```bash
cmake -Bbuild -DCMAKE_INSTALL_PREFIX=$(mads -p)
cmake --build build -j4
cmake --install build
```

## INI settings

```ini
[arduino]
port = /dev/ttyUSB0
baudrate = 115200
silent = true
# the following can be omitted
cfg_cmd = ""
```

## Arduino code

The `arduino` folder contains a sketch that can be customized to read different analog or digital pin and publish a corresponding JSON string on serial port.

The script supports a simple command parsing interface that allows to configure some settings on the Arduino side. Look at the arduino code for details, but this allows to set the `cfg_cmg` setting as `250p20t`, for example, to set the sampling period to 250 ms and the threshold to 20 (in the range 0–1023).