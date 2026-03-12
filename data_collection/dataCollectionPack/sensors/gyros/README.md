# ISM330DHCX Interface

This repo is an interface for logging data from the [SparkFun ISM330DHCX 9DoF Accel/Gyro/Mag](https://www.sparkfun.com/sparkfun-6dof-imu-breakout-ism330dhcx-qwiic.html) on a linux PC using the [WWZMDiB CH341A USB to UART/IIC/SPI/TTL/ISP Adapter](https://manuals.plus/asin/B0BVH6BCF5). It contains an adapted version of the [ISM330DHCX Arduino interface](https://github.com/sparkfun/SparkFun_6DoF_ISM330DHCX_Arduino_Library/tree/main) which removes the requirement of an Arduino. 

With this setup, two sensors can collect gyro data on the same bus at a rate of 380Hz. The Arduino approach appears to max out at 100Hz with a single sensor. The theoretical max rate of the sensor is 6667Hz (though I've never seen an example above 200Hz, so I'm not convinced this is possible given the breakout board.)

This project provides a cross-platform interface for the ISM330DHCX 6-axis IMU sensor, supporting both Windows and Linux platforms with different I2C interfaces.

## Platform Support

### Windows
- **I2C Interface**: CH341 USB-to-I2C adapter using DLL
- **Driver**: CH341DLLA64.dll (included in lib/ directory)
- **Requirements**: CH341 driver from [WCH website](https://www.wch-ic.com/downloads/CH341PAR_EXE.html)

### Linux  
- **I2C Interface**: Native Linux I2C (/dev/i2c-*)
- **Driver**: Custom i2c-ch341-usb driver (included in driver/ directory)
- **Requirements**: I2C permissions and CH341 driver installation

## Setup Instructions 
Clone this repository 
```
git clone git@github.com:spencerteetaert/sparkfun_ism330dhcx_interface.git
cd sparkfun_ism330dhcx_interface
```

### Hardware setup 
Ensure that the CH341A adapter is set to i2c mode via the spi/i2c selector pins and jumper on the side of the USB port. 

The ISM330DHCX has two possible i2c addresses, 0x6b (default, ISM330DHCX_ADDRESS_HIGH) and 0x6a (bridged, ISM330DHCX_ADDRESS_LOW). The are selected via some soldered pads on the flip side of the chip. In order to use two sensors on the same bus, they must have different addresses. 

### Driver installation 
Some additional steps were required to get the [USB to serial adapters](https://www.amazon.ca/gp/product/B0B72K6VTW/?AssociateTag=sacsite05-20&tag=sacsite05-20) working. Specifically, an adapted i2c-ch341-usb driver. This is not native to linux and the current version is outdated, using an old GPIO library. After removing it, the driver appears to build and load properly, albeit losing any GPIO functionality. This updated driver is included in the driver folder. To install, 
```
cd driver
sudo make install
```
To confirm it installed properly, run 
```
sudo dmesg -w
```
and plug in the adapter. You should see something along the lines of 
```
[ 6932.953061] i2c-ch341-usb 1-1:1.0: ch341_cfg_probe: output gpio0 gpio=0 irq=0 
[ 6932.953072] i2c-ch341-usb 1-1:1.0: ch341_cfg_probe: output gpio1 gpio=1 irq=1 
[ 6932.953076] i2c-ch341-usb 1-1:1.0: ch341_cfg_probe: output gpio2 gpio=2 irq=2 
[ 6932.953078] i2c-ch341-usb 1-1:1.0: ch341_cfg_probe: output gpio3 gpio=3 irq=3 
[ 6932.953081] i2c-ch341-usb 1-1:1.0: ch341_cfg_probe: input  gpio4 gpio=4 irq=4 (hwirq)
[ 6932.953085] i2c-ch341-usb 1-1:1.0: ch341_cfg_probe: input  gpio5 gpio=5 irq=5 
[ 6932.953088] i2c-ch341-usb 1-1:1.0: ch341_cfg_probe: input  gpio6 gpio=6 irq=6 
[ 6932.953090] i2c-ch341-usb 1-1:1.0: ch341_cfg_probe: input  gpio7 gpio=7 irq=7 
[ 6932.953382] i2c-ch341-usb 1-1:1.0: ch341_i2c_probe: created i2c device /dev/i2c-16
[ 6932.953386] i2c-ch341-usb 1-1:1.0: ch341_i2c_set_speed: Change i2c bus speed to 100 kbps
[ 6932.953596] i2c-ch341-usb 1-1:1.0: ch341_usb_probe: connected
```
Make note of the address that the device is attached to (in this example, /dev/i2c-16). This is passed as a parameter in the code itself to establish the i2c connection. 

### Software setup
First, ensure that in main.cpp the correct address is passed to the GyroAPI constructor (default /dev/i2c-16). Also edit the gyro_api.add_device() function calls to reflect the sensors you have connected to the adapter. 

To build the project, 
```
mkdir build && cd build 
cmake ..
cmake --build .
```
and to run the project 
```
sudo ./sparkfun_ism330dhcx <output_folder> <frequency> 
```
where <output_folder> is the location you wish to log data and <frequency> is the rate you wish to log at. sudo is required here to acces the /dev/i2c-* port that your device is attached to. 

## Additional Links
- [I2C protocol description](https://www.ti.com/lit/an/slva704/slva704.pdf?ts=1756996414251)
- [Original Linux driver](https://www.wch-ic.com/downloads/CH341SER_LINUX_ZIP.html)
