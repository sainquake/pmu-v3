# PMU v3

C8051F120 - mcu

# IDE requerments

silabs for c8051 old version
silabs programmer

# pinout

# DATA PROTOCOL

packet structure: 1byte ADDRESS, Xbyte PAYLOAD, 1byte CRC

ADDRESS:
MSB - address bit
6bit - start of transmit
0-5 - address bits

PAYLOAD:
MSB = 1
0-6bit - payload

CRC:
XOR of packet