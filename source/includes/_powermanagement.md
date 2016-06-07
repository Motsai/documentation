# Power management

Commands associated with power management system.

## GetTemperature()

This command returns the temperature in degrees celsius sensed by the main processor integrated circuit on the ProMotion board.

<aside class="notice">
Note: When requested over BLE, this command will not get a response out of the Neblina Custom Characteristic.  Bluetooth standardizes the temperature information.  To get the temperature via BLE, simply read it from the standard org.bluetooth.characteristic.temperature characteristic.
</aside>


## GetBatteryLevel()

Returns the remaining battery life as a percentage (0-100%). It is returned with a precision a tenth of a percent.

<aside class="notice">
When requested over BLE, this command will not get a response out of the Neblina Custom Characteristic.  Bluetooth standardizes the battery information.  To get the battery level via BLE, simply read it from the standard org.bluetooth.characteristic.battery_level characteristic (0x2A19).
</aside>


