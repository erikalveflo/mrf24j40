MRF24J40 Arduino Library
============

This is an Arduino library for the [MRF24J40](http://www.microchip.com/wwwproducts/Devices.aspx?product=MRF24J40) radio. It is known to work with the *MRF24J40MA* module, *Teensy 3.2*, *Arduino Nano*, and *Arduino Pro Mini*, but is written to work with any Arduino compatible device.

## Features

* Reception
* Transmission
* Security (at the MAC level)

This library is mainly a rewrite from the [C-library by bitgamma](https://github.com/bitgamma/mrf24j40) to a C++ class compatible with Arduino.

Its primary use is to send and receive broadcast messages using raw *IEEE 802.15.4 MAC frames*. There is **no support** for ZigBee or other high-level protocols (unless you implement them ;) However, in theory, it can be used with any IEEE 802.15.4 based protocol.

## Notes

This driver makes use of an undocumented feature of the MRF24J40 chip: namely, when writing to any FIFO, you can write the address once, and then write as many data bytes as needed until you release the CS signal. The same is also true for reading (just continue sending zeros and read the answer). This should improve performance a little.

The same trick cannot be applied to Control Registers, or at least I have not been able to do it.

## Installation

Download and extract [the .ZIP file](https://github.com/erikalveflo/mrf24j40/archive/master.zip) to ``Documents\Arduino\libraries\``