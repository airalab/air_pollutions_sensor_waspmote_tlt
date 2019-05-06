Waspmote sensor communication
=============================

This protocol implements ECDSA signatures for Liberium sensors.

Protocol
--------

1. Sensor open TCP connection
2. Server accept TCP connection
3. Sensor send `Data frame`
4. Server send `0x01` response 

Data frame structure
--------------------

```
+------+------------------------+
| 0x00 | Ed25519 signature      |
| 0x40 | Sensor data length     | 
| 0x42 | Sensor data string     |
| ...  |                        |
+------+------------------------+
```
