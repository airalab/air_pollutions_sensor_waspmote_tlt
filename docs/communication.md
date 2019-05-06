Waspmote sensor communication
=============================

This protocol implements signed by ECDSA UDP frames for Liberium sensors.

Data frame structure
--------------------

```
+------+--------------------------------+
| 0x00 | Hex encoded Ed25519 signature  |
| 0x80 | Sensor data string             | 
| ...  |                                |
+------+--------------------------------+
```
