# LSM6DS33 + LIS3MDL Libraries v0.1.0 ALPHA

These files are used together to communicate with the two chipsets via the I2C interface. 
The libraries are written for the STM32L4xx, to use it on an F series of any other STM32 platform, change the includes for the HALs.

## Disclaimer

The files were tested using the STM32CubeIDE, as such any attempts to use the files on another platform should be done at your own discression. 
The code has only been tested on a STM32L433cbt6 chipset using the J-link interface to both program and debug.
A UART was included in the main.c file to allow printing out onto a terminal.

This project is unfinished! The algorithm and libraries work but styling is a mess. This will be fixed in future revisions.
The libraries follow a 'DIY' approach to configuring the various registers in an intuitive way. Most libraries simply give a few default settings and then if the user wants to get more control over their device, they should make their own library or read the datasheet.
This library aims to make it easier to understand what knobs you're turning when you shift bits into different registers (let me know if this sort of approach is helpful or not).

## Versioning

We use [SemVer](http://semver.org/) for versioning.

## Authors

* **Harry Nowakowski**

## License

Copyright 2019 Harry Nowakowski

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.



## Acknowledgments

* The DCM algorithm used to test the library was provided by Pololu Corporation, which allows the redistribution and/or modification under the terms of the GNU Lesser General Public License as published by the Free Software Foundation. The code was modified to work with the C99 standard (so a lot less globals than the arduino code). The original files may be found [here](https://github.com/pololu/minimu-9-ahrs-arduino/blob/master/MinIMU9AHRS/DCM.ino).
