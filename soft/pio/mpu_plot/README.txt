

MPU Plotter
===========

Live plotting utility for streaming MPU6050 (or similar) data in the form:
	Seq,Micros,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,Roll,Pitch,Yaw

Quick Start
-----------

1. Create / activate a virtual environment (recommended).
2. Install dependencies:

	 pip install -r requirements.txt

3. Run:

	 `````````````python mpu_plotter.py -p /dev/ttyUSB0 --window-seconds 15````````````

Common Options
--------------
	-p / --port            Serial device (default /dev/ttyUSB0)
	-b / --baud            Baud rate (default 115200)
	-n / --points          Max buffered points (default 400)
	--window-seconds       Visible rolling time window (seconds)
	--debug                Print raw and first parsed lines
	--no-header-wait       Skip waiting for header line

Troubleshooting
---------------
AttributeError: module 'serial' has no attribute 'Serial'
	Cause: pyserial not installed OR wrong package named 'serial' OR name shadowing.
	Fix:
		pip uninstall serial -y  # only if mistakenly installed
		pip install --upgrade pyserial
	Ensure no local file/folder named serial.py or serial/ in the working path.

If plots don't show:
	- Ensure an X server / display is available (e.g., SSH with -X or use a non-GUI backend and adapt code).
	- Check that data is actually streaming (use --debug to inspect incoming lines).

Keyboard Shortcuts
------------------
	Space : Pause / resume
	q     : Quit

License
-------
Internal project utility; adapt as needed.
