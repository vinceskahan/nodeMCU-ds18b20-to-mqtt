
## publish a DS18B20 temperature to MQTT

This program is for the nodeMCU but should work on any typical Arduino card.

## how to wire

The DS18B20 sensor has three pins left-to-right with the 'flat' side toward you:
  GROUND
  DATA
  VCC

* Connect 3.3V power to the VCC pin

* Connect ground to the GROUND pin

* Connect a data connection to the DATA pin
      (nodeMCU hardware pin D2 = arduino software pin 4)

* Make sure you have the resistor between DATA and VCC

    Note: do not get the VCC/GROUND connections reversed, or the sensor
      will get 'very' hot 'very' quickly.

## debugging

If you use a different nodeMCU data pin, edit the .ino file here accordingly.
Important, the 'software' pin numbers do 'not' match the 'hardware' pin numbers.
Consult the docs for your Arduino card for the appropriate mapping.

If you see a temperature of -185 degF then you aren't getting data from the sensor.
This is likely either a loose/disconnected DATA jumper, or you are connected to
a pin on the 'hardware' that does not match the pin in the .ino file you loaded.

