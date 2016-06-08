# System APIs
######################################################

### getRecorderStatus()
This function works on the ProMotion board with and requests the status of the on-chip recorder. The returned status is either off, busy recording, or busy doing playback.

### getMotionStatus()
This function requests the status of the motion engine on Neblina. The returned status will indicate what motion features are currently active on Neblina.

### getFirmwareVersion()
This function requests for Neblina's device ID, firmware version numbers for the KL26 and Nordic chips, as well as the API release version. This information will be returned by this function call.

### getDataPortState()
This function inquires the status of all the data streaming interfaces. These are the interface ports, where Neblina streams its data to. On the ProMotion board this includes BLE and UART. The returned status will determine whether the BLE/UART port is open or closed for streaming.

### setDataPortState()
This function sets a data streaming interface state to either open or closed. The interfaces on the ProMotion board include BLE and UART.

### setInterface()
This function opens a single interface port and closes the other one. If the input argument is BLE, then the UART port will be closed, and the BLE will be opened for streaming. Otherwise with the input argument set to UART, the BLE port will be closed for streaming, and the UART port will be opened.
