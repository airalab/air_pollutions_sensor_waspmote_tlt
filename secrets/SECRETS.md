Secret files:
--
1. `server.crt` with SSL certificate for sensor gateway TCP server
1. `server.key` with SSL certificate private key
1. `otp.key` with one-time password secret key in base32 format
1. `secrets.h` for sensor firmware with two arrays:
* `hmacKey` with hex HMAC key values for one-time password
* `certificate` with sensor gateway TCP server SSL certificate same as one in `server.crt`
