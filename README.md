# Remote-Monitoring-System
Precision remote temperature &amp; pressure monitoring system with PID calibration

File: Remote_Monitoring_DAQ.ino

Features:
• Dual PID controllers (pressure + strain)
• Kalman filter + FIR + EMA noise rejection
• EEPROM calibration storage
• Serial protocol: SET_PRESS:1025.0
• Wire compensation for 50m+ cables

Libraries:
HX711 (bogde)
Adafruit BMP085
PID v1

Usage:
1. Upload code to Arduino Nano
2. Wire BMP085 (A4/A5), HX711 (D2/D3)
3. Serial Monitor: SET_PRESS:1025

Output:
DATA:1025.12,48.3,24.8,1025.0,152

Recreated Jan 2025 instrumentation project.
