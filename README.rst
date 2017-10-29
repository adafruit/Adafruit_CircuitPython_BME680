
Introduction
============

.. image:: https://readthedocs.org/projects/adafruit-circuitpython-bme680/badge/?version=latest
    :target: https://circuitpython.readthedocs.io/projects/bme680/en/latest/
    :alt: Documentation Status

.. image :: https://img.shields.io/discord/327254708534116352.svg
    :target: https://discord.gg/nBQh6qu
    :alt: Discord

TODO

Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://github.com/adafruit/Adafruit_CircuitPython_Bundle>`_.

Usage Example
=============

.. code-block:: python
	import gc
	from busio import I2C
	import adafruit_bme680
	import time
	import board

	gc.collect()
	print("Free mem:",gc.mem_free())

	# Create library object using our Bus I2C port
	i2c = I2C(board.SCL, board.SDA)
	bme280 = adafruit_bme680.Adafruit_BME680_I2C(i2c)

	# change this to match the location's pressure (hPa) at sea level
	bme280.seaLevelhPa = 1013.25

	while True:
	    print("\nTemperature: %0.1f C" % bme280.temperature)
	    print("Gas: %d ohm" % bme280.gas)
	    print("Humidity: %0.1f %%" % bme280.humidity)
	    print("Pressure: %0.1f hPa" % bme280.pressure)
	    print("Altitude = %0.2f meters" % bme280.altitude)

	    time.sleep(2)


Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_bme680/blob/master/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.

API Reference
=============

.. toctree::
   :maxdepth: 2

   api
