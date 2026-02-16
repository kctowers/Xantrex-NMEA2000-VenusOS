# Xantrex XC Pro Marine Inverter/Charger
## Venus OS D-Bus Driver (Cerbo) NMEA 2000


**Date Updated:** Feb 20, 2025  
**Kevin Towers**

It is a Python service that reads Xantrex Freedom Pro Marine (NMEA 2000) (CAN) frames and publishes decoded values to Victron Venus OS D-Bus, running on a PI or Cerbo, as inverter service.

**Target:** Venus OS - I have tested with v3.67 (Python 3.12 on Cerbo GX). 

-------------------------------------------------------------------------------

## FEATURES

- Decodes propietary Xantrex PGNs and maps them to Venus D-Bus paths which integrate with the Cerbo GUI.
- Posts notications to GUI when Xantrex alarms on over/under range values
- Heartbeat updates, /State path, and derived power calculations (e.g., P = V × I).
- CAN frames are filtered in hardware
- Written in Python

-------------------------------------------------------------------------------

## REQUIREMENTS

- Venus OS device with Python 3.12 and D-Bus/GLib available.
- SocketCAN interface configured (default: vecan0, 250 kbps).
- Permission to access CAN sockets (root or appropriate capabilities).

-------------------------------------------------------------------------------

## INSTALL (VENUS OS)

**1) Place files on the device**
```text
/data/xantrex-monitor/
├─ xantrex_service.py    # this script
└─ velib_python/         # optional vendored copy; I still have to confirm this.  Do not recall at the moment
```
Note, you will have to download the velib_python from Github at: https://github.com/victronenergy/velib_python

**2) Run the service**
```bash
# Use this command line if you want to test any changes you made.  Ctrl-C will exit the script and allow you to
# make further changes
cd /data/xantrex-monitor
python3 xantrex_service.py

When you're ready for prime time, execute this:
./start.sh
```

**3) Setup for automatic run at boot time**  
Modify the file _/data/rc.local_ to add this line:
```
/data/xantrex-monitor/start.sh
```
This will automatically run the _start.sh_ script on reboot.  
Note: A new Cerbo may not have a _/data/rc.local_ file.  You will have to create it with these commands:
```
nano /data/rc.local
   #!/bin/bash
   /data/xantrex-monitor/start.sh
chmod +x /data/rc.local
```

**Optional Program parameters**
```text
--can IFACE    SocketCAN interface (default: vecan0)
--log LEVEL    Enable logging (DEBUG, INFO, WARNING, ERROR, default: WARNING)
--verbose      Dump all logs to the terminal as well as log files
```

**Examples**
```bash
# Use any combination of flags. During initial testing, enable --log (and optionally --verbose).
python3 /data/xantrex-monitor/xantrex_service.py --can vecan1
python3 /data/xantrex-monitor/xantrex_service.py --log INFO
python3 /data/xantrex-monitor/xantrex_service.py --log INFO --verbose
python3 /data/xantrex-monitor/xantrex_service.py --v
```

-------------------------------------------------------------------------------

## ARCHITECTURE NOTES

**Main loop:**
- GLib main loop drives CAN receive and D-Bus exports.

**Source selection:**
- The service retrieves the CAN device ID from the Cerbo CAN NMEA driver.  This is used for all transmissions.

**Mapping/decoding:**
- PGNs are decoded from frame payloads and mapped to canonical Venus paths. Derived values are computed where useful (e.g., power).

**Logging:**
- Structured logging supports --log and --verbose. Frame counts and source IDs aid traceability. Logs default to /data/xantrex-monitor/logs/xantrex.log.

**Clean shutdown:**
- Signal handling exits the main loop, unregisters D-Bus objects, and closes CAN sockets to avoid stale bus names on restart.

-------------------------------------------------------------------------------

## CONTRIBUTIONS / FEEDBACK

- You are welcome to modify and adapt this code for your own setup.
- Please send improvements back (pull requests or patches) so others can benefit.
- Open an issue for bugs, new DGN decoders, or Venus OS path mapping suggestions.
- Unless otherwise noted, contributions are accepted under the same license (MIT).



-------------------------------------------------------------------------------


## LICENSE

MIT  See LICENSE.
