**Components:**

**ESP32:** The microcontroller that controls everything.

Pins: GPIO19, GPIO18, GPIO17, 3v3, GND, VBUS, SCL, SDA, SPKR+, SPKR-, GPS+, GPS-.

**12V to 5V Buck Converter:** To step down the input voltage for the ESP32 and LED strip.

Pins: Input +, Input -, Output +, Output -.

**XL6009E1 Boost Converter:** To step up the 5V to 10V for the motor.

Pins: Input +, Input -, Output +, Output -.

**WS2812B 48 LED Strip:** The addressable LED lights.

Pins: +, -, Data

**iFlight iPower GM2804 Motor:** The brushless gimbal motor with the built-in ESC.

Pins: +, -, PWM

**MMC5603 Magnetometer:** A sensor for magnetic fields.

Pins: (UART) +, -, SDA, SCL

**AS5048A Encoder:** A sensor for rotational position.

Pins: (UART) +, -, SDA, SCL

**Schottky Diodes (x2):** For protecting the two power inputs.

Pins: Input, Output

**GPS:** Can be ignored as it's just the antenna with a regular RF wire. Included for clarification.







**Connection List:**

**ESP32 Connector:**

* GPIO19 to Encoder.PWM
* GPIO18 to Motor.PWM
* GPIO17 to LED\_Strip.Data
* 3v3 to 3V3\_Rail
* GND to GND\_Rail
* VBUS to 5V\_Rail
* SCL to MMC5603.SCL
* SDA to MMC5603.SDA
* SPKR+, SPKR-, GPS+, GPS- are Not Connected



**12V to 5V Buck Converter:**

* Input+ to 12V\_Rail
* Input- to GND\_Rail
* Output+ to 5V\_Rail
* Output- to GND\_Rail



**XL6009E1 Boost Converter:**

* Input+ to 5V\_Rail
* Input- to GND\_Rail
* Output+ to Motor.+
* Output- to GND\_Rail



**WS2812B 48 LED Strip:**

* \+ to 5V\_Rail
* \- to GND\_Rail
* Data to ESP32.GPIO17



**iFlight iPower GM2804 Motor:**

* \+ to Boost\_Converter.Output+
* \- to GND\_Rail
* PWM to ESP32.GPIO18



**MMC5603 Magnetometer:**

* \+ to 3V3\_Rail
* \- to GND\_Rail
* SDA to ESP32.SDA
* SCL to ESP32.SCL



**AS5048A Encoder (3-pin PWM version):**

* \+ to 3V3\_Rail
* \- to GND\_Rail
* PWM to ESP32.GPIO19



**Schottky Diode 1:**

* Input to Barrel\_Jack.+
* Output to 12V\_Rail



**Schottky Diode 2:**

* Input to Direct\_Connect.+
* Output to 12V\_Rail



**Barrel Jack Input:**

* \+ to Schottky\_Diode\_1.Input
* \- to GND\_Rail



**Direct Connect Input:**

* \+ to Schottky\_Diode\_2.Input
* \- to GND\_Rail







**Wiring:**

This section describes the physical connections to each module, all module pins have female connectors.



**ESP32 Connector:**

* GPIO19**:** M-M wire to Encoder PWM
* GPIO18: M-M wire to Motor PWM
* GPIO17: M-M wire to LED Strip Data
* 3v3: M-M wire to 3V3\_Rail splitter
* GND: M-M wire to GND\_Rail splitter
* VBUS: M-M wire to 5V\_Rail splitter
* SCL: M-M wire to MMC5603 SCL
* SDA: M-M wire to MMC5603 SDA



**12V to 5V Buck Converter:**

* Input+: M-M wire to 12V\_Rail splitter
* Input-: M-M wire to GND\_Rail splitter
* Output+: M-M wire to 5V\_Rail splitter
* Output-: M-M wire to GND\_Rail splitter



**XL6009E1 Boost Converter:**

* Input+: M-M wire to 5V\_Rail splitter
* Input-: M-M wire to GND\_Rail splitter
* Output+: M-M wire to Motor +
* Output-: M-M wire to GND\_Rail splitter



**WS2812B 48 LED Strip:**

* +: M-M wire to 5V\_Rail splitter
* -: M-M wire to GND\_Rail splitter
* Data: M-M wire to ESP32 GPIO17



**iFlight iPower GM2804 Motor:**

* +: M-M wire to Boost Converter Output+
* -: M-M wire to GND\_Rail splitter
* PWM: M-M wire to ESP32 GPIO18



**MMC5603 Magnetometer:**

* +: M-M wire to 3V3\_Rail splitter
* -: M-M wire to GND\_Rail splitter
* SDA: M-M wire to ESP32 SDA
* SCL: M-M wire to ESP32 SCL



**AS5048A Encoder:**

* +: M-M wire to 3V3\_Rail splitter
* -: M-M wire to GND\_Rail splitter
* PWM: M-M wire to ESP32 GPIO19







**Wires Needed:**

This is an inventory of the jumper wires required to assemble the project.



**Splitter Cables (to create power/ground rails):**

* **(x4) Male to 3x Male Splitter:**

&nbsp;	- One for the 5V\_Rail (connects Buck, Boost, ESP32, LED).

&nbsp;	- Three linked together for the GND\_Rail (connects all 11 ground points).



* **(x2) Male to 2x Male Splitter:**

&nbsp;	- One for the 3V3\_Rail (connects ESP32, Encoder, MMC5603).

&nbsp;	- One for the 12V\_Rail (connects both Diodes, Buck Input).



**Standard Point-to-Point Wires:**

* **(x10) Male to Male Wires:**

&nbsp;	- 2 for the 12V inputs to the diodes.

&nbsp;	- 1 for ESP32.GPIO17 to LED.Data.

&nbsp;	- 1 for ESP32.GPIO18 to Motor.PWM.

&nbsp;	- 1 for ESP32.GPIO19 to Encoder.PWM.

&nbsp;	- 1 for Boost.Output+ to Motor.+.

&nbsp;	- 2 for the SDA/SCL lines between the ESP32 and MMC5603.

&nbsp;	- 2 for connecting the main power sources to the project.





