# Bicopter_heckx
Flight controller firmware for an esp controlled bicopter.

The roll_tuner.ino & tuner_gui.py are made specifically to fine tune PID for roll. 

About:

The gui helps to tune the pid values in realtime without reflashing the code.
updated: 2 march 2026
currently testing and tuning the roll behaviour + pitch behaviour.
based on code from roll_test(the logic remains the same).
roll_tuner.ino is the main code for this tuning
added failsafe by ch6 on pin 36
rc roll limits increased to 70 
pitch pid tuning required.

if any errors happen, roll_test will be the start point for redoing.




USAGE:

Upload bicopter_tuner.ino to your board (USB serial) using Arduino IDE. Make sure correct board/port selected. (It prints READY when initialized.)
Connect the board with serial cable to your PC.
Run the Python GUI: python tuner_gui.py.
Select the COM port and press Connect.
Move sliders — each slider change immediately sends the command:
AKP<value> (e.g. AKP4.200) etc.
Press Save to EEPROM to persist current gains on the board.
Telemetry will appear live on labels and in the plot (roll_angle vs time).
Use Load from EEPROM on boot or when you want to restore saved values.
