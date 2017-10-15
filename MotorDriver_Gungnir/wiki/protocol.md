# CAN Protocol
SJW = 1 tq
BS1 = 4 tq
BS2 = 1 tq
Rate: 1Mbps

## Packing method
The first byte is usually the command byte. 
Latter byte are of higher order. In case of 3 bytes data (like PWM), the first bit of the resulting bit pattern is the sign bit.
Example:

Packing:
```
msg.data[1] = pos;
msg.data[2] = pos >> 8;
msg.data[3] = pos >> 16;
msg.data[4] = pos >> 24;
```

Unpacking:
```
pos = msg->Data[1] | msg->Data[2] << 8 | msg->Data[3] << 16 | msg->Data[4] << 24;
```

## Message ID and Command ID
The CAN message id used for sending commands to the motor driver can be obtained by adding the motor id offset to 0x0D0.
Command | ID
--- | ---
Velocity | 0xAA
Position | 0xBC
Acceleration | 0x45
Maximum velocity | 0x49
Lock | 0xEE

The CAN message id used for receiving feedback from the motor driver can be obtained by adding the motor id offset to 0x0E0.
Feedback type | ID
--- | ---
Encoder count, state and pwm feedback | 0x22 with masks
Lower limit switch hit | 0x30
Upper limit switch hit | 0x31
Path finished acceleration | 0x40
Path beginning deceleration | 0x41
Path finished deceleration | 0x42
