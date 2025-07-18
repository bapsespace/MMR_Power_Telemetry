# Power Telemetry
"""
Legge can:
ID    byte: 1    2    3    4    5    6    7    8 
225         xx   xx   xx   xx   00   xx   xx   xx

IF byte_5 == 0:
  LED Acceso
  Traco OFF
IF byte_5 >= 1:
  LED Spento
  Traco ON 5V
"""
