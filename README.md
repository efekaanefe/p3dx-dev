# p3dx-main

```main```
---
The file to run from the computer. Currently just for testing purposes. When run, the user is asked to choose a function registered in. Current functions (these are just for testing/changing the mode of the robot at initialization purposes):
- `tryMovement`: Tries to move the robot according to the given parameters, reads the response and gives 0 velocity. (Timer yok şu an ani başlayıp durabilir)
- `sendCommands`: Şu an işlevsiz, tryMovement'ın parametresi hali gibi takılıyor.
- ```functions.json```. When chosen, the main will first establish the connection, then run the function automatically. At last, it will disconnect.


```p3dxRobotClass```
---
This is our base class for the robot. An object from this class needs to be initialized to use the p3dx-centered functions. 

Current attributes:
 - ``port``: The port the robot is connected to. By default `/dev/ttyUSB0`.
 - `baudrate`: Default `9600`. (Yanlış olabilir)
 - `timeout`: Timeout for the serial connection
 - ``serial_conn``: The connection opened via `connect` function.


Current functionalities:
 - ``connect``: Connects via the serial port (socket implementation can be added in the future)
 - ``send_velocity``: Sends a translation and a rotation speed, both are scalar (an option for timing may be added)
 - ``read_response``: Read a single line from the robot (if available).
 - ``disconnect``: Disconnects from the serial port.