# LoRa single channel gateway

Synchronous wireless star network.

Use case:
* wireless messages are initiated by end devices (sensors)
* gateway always turned on in a fixed location, gateway is always receiving

See [embedded gateway page](https://os.mbed.com/users/dudmuck/code/LoRaWAN_singlechannel_gateway) for more detailed description of network.
This project uses [raspberry Pi](https://en.wikipedia.org/wiki/Raspberry_Pi) I2C to LoRa radio.

Operates with I2C slave device [i2c_lora_slave](https://os.mbed.com/users/dudmuck/code/i2c_lora_slave/) connected to raspberry pi.

Wireless end devices use [single channel endnode](https://os.mbed.com/users/dudmuck/code/LoRaWAN_singlechannel_endnode/) project.

# build instructions
for raspberry pi, initial installation:
* Clone this repository with ``--recurse-submodule``
* if ``/dev/i2c-1`` doesnt exist, use ``raspi-config`` to enable it, or edit ``/boot/config.txt`` to uncomment ``dtparam=i2c_arm=on``
* [install wiringPi](http://wiringpi.com/download-and-install/) library
* ``sudo apt install libjson-c-dev libi2c-dev swig``
* if ``i2c_lora_master`` directory empty, then you didnt clone this repository with ``--recurse-submodule``, then:
  * ``cd i2c_lora_master``
  * ``git submodule init``
  * ``git submodule update``
  * ``cd ..``

From project root directory, build:
* ``mkdir build``
* ``cd build``
* ``cmake ..`` 
* ``make``

# wiring connections
see [lib_i2c_slave_block](https://os.mbed.com/users/dudmuck/code/lib_i2c_slave_block/) for wiring connections to slave.

4 pins required: SDA, SCL, interrupt and ground.   Additional interrupt needed for each slave device.

![RPi lora](/pi_lora_.svg?raw=true)

# gateway configuration
Configuration file ``gw_conf.json``:
* provision end devices
* assign I2C slave address and interrupt pin
* radio channel setup (frequencies and datarate)

# gateway application program
Gateway implemented as library.  User payload is handled in application layer, ``app_scgw`` for demonstration.

Library ``scgw`` implements MAC layer.  Gateway application layer includes header ``scgw.h`` and links to ``scgw`` library.  Application layer implements user interface and user payload handling.

### using ``app_scgw``
``app_scgw`` listens to keyboard input during operation. Use ``? <enter>`` for list of commands.
