iNav coprocessor

# Compiling in platformio

# Debugging on wsl
Need to ensure access to the usb port when on wsl.  First figure out the jtag debug port -- could use usbipd list -- also might need to change the mode of the device in WSL to 0666 -- also need to do the same for the usb port for the usb console output

```powershell
PS C:\Users\gcmcn> usbipd attach --wsl --busid 4-4 -a
usbipd: info: Using WSL distribution 'Ubuntu' to attach; the device will be available in all WSL 2 distributions.
usbipd: info: Detected networking mode 'nat'.
usbipd: info: Using IP address 172.17.32.1 to reach the host.
usbipd: info: Starting endless attach loop; press Ctrl+C to quit.
WSL Monitoring host 172.17.32.1 for BUSID: 4-4
WSL 2025-07-07 18:09:25 Device 4-4 is available. Attempting to attach...
WSL 2025-07-07 18:09:25 Attach command for device 4-4 succeeded.
WSL 2025-07-07 18:09:33 Device 4-4 is now attached.
```

# Settings to use iNav
- msp_override_channels = 15 (or 7 if just aet)
- msp override as a mode configured to a switch
- failsafe to RTH most likely
- consider allow rearm in flight

# Other
- BTLE code uploader
- Storage event logs in local storage

# Displaying inav blackbox data in google earth
## First time setup
- this tool converts the logfiles to kml files:
  - https://www.daria.co.uk/bbl2kml/#flightlog2kml
  - https://github.com/stronnag/bbl2kml
- it uses backbox-tools: 
  - https://github.com/iNavFlight/blackbox-tools
- displaying the kml works best in google earth
- install both utilities

## Convert a flight log
- download flight log from the flight controller
- determine which logfile to convert
- do a conversion:
```powershell
PS C:\Users\gcmcn\OneDrive\Documents\Dev> ..\Navigation\fl2x\win64-x86_64\bin\flightlog2kml -attributes altitude -cli ..\Navigation\blackbox-tools-win64\bin\blackbox_decode.exe -extrude -index 20 -interval 200 ..\Navigation\HB1-orange-configs\blackbox_log_2025-03-09_203731.TXT       
Size     : 966.4 KB
Log      : blackbox_log_2025-03-09_203731.TXT / 20
Flight   :  on 2025-03-09 13:16:53
Firmware : INAV 7.1.2 (4e1e59eb) MATEKF722MINI of Jun  1 2024 00:00:40
Altitude : 71.7 m at 01:31
Speed    : 26.8 m/s at 02:22
Range    : 177 m at 01:11
Distance : 2591 m
Duration : 03:42
Disarm   : Switch
Output   : C:\Users\gcmcn\OneDrive\Documents\Dev\blackbox_log_2025-03-09_203731.20.kmz
```

- then 'import' the resulting kmz into google earth