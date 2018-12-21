
Introduction
============

.. image:: https://readthedocs.org/projects/adafruit-circuitpython-bme680/badge/?version=latest
    :target: https://circuitpython.readthedocs.io/projects/bme680/en/latest/
    :alt: Documentation Status

.. image :: https://img.shields.io/discord/327254708534116352.svg
    :target: https://discord.gg/nBQh6qu
    :alt: Discord

.. image:: https://travis-ci.com/adafruit/Adafruit_CircuitPython_BME680.svg?branch=master
    :target: https://travis-ci.com/adafruit/Adafruit_CircuitPython_BME680
    :alt: Build Status

CircuitPython driver for BME680 sensor over I2C

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
	bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c)

	# change this to match the location's pressure (hPa) at sea level
	bme680.sea_level_pressure = 1013.25

	while True:
	    print("\nTemperature: %0.1f C" % bme680.temperature)
	    print("Gas: %d ohm" % bme680.gas)
	    print("Humidity: %0.1f %%" % bme680.humidity)
	    print("Pressure: %0.3f hPa" % bme680.pressure)
	    print("Altitude = %0.2f meters" % bme680.altitude)

	    time.sleep(2)


Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_bme680/blob/master/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.

Building locally
================

To build this library locally you'll need to install the
`circuitpython-build-tools <https://github.com/adafruit/circuitpython-build-tools>`_ package.

.. code-block:: shell

    python3 -m venv .env
    source .env/bin/activate
    pip install circuitpython-build-tools

Once installed, make sure you are in the virtual environment:

.. code-block:: shell

    source .env/bin/activate

Then run the build:

.. code-block:: shell

    circuitpython-build-bundles --filename_prefix adafruit-circuitpython-bme680 --library_location .

Sphinx documentation
-----------------------

Sphinx is used to build the documentation based on rST files and comments in the code. First,
install dependencies (feel free to reuse the virtual environment from above):

.. code-block:: shell

    python3 -m venv .env
    source .env/bin/activate
    pip install Sphinx sphinx-rtd-theme

Now, once you have the virtual environment activated:

.. code-block:: shell

    cd docs
    sphinx-build -E -W -b html . _build/html

This will output the documentation to ``docs/_build/html``. Open the index.html in your browser to
view them. It will also (due to -W) error out on any warning like Travis will. This is a good way to
locally verify it will pass.