# AgOpenGPS firmwares (OTA & RawGNSS)

AgOpenGPS Teensy 4.1 firmwares with additions for AgIO:

| Firmware | For |
|---|---|
| `All-In-One/AIO v4 (OTA & RawGNSS)` | AiO v4.x boards (single F9P / UM982, or dual F9P) |
| `All-In-One/AIO v2.5 (OTA & RawGNSS)` | AiO v2.5 boards |
| `CANBUS/CANBUS All-In-One (OTA & RawGNSS)` | CANBUS steering tractors, AiO board |
| `CANBUS/CANBUS CommonRail (OTA & RawGNSS)` | CANBUS steering tractors, Tony's (CommonRail) board |
| `Keya/Keya (OTA & RawGNSS)` | Keya steering motor (CAN), AiO board |

Each works with a single F9P or UM982, a UM982 dual antenna and two F9Ps for dual antenna, and finds out which as
it runs: dual mode while a fixed dual heading comes, the single antenna way otherwise. The receiver is set up on its
own (Config-O-Matic or AgIO's module configurator); the firmware does not need to know.

What is added to each:

- raw measurements of the position receiver (RTCM 3 MSM and ephemerides, u-blox RXM-RAWX / RXM-SFRBX) go to AgIO
  for TDCP (carrier phase movement) in AgOpenGPS;
- the receivers can be configured over Ethernet from AgIO (Settings > Module configurator), their answers come
  back (PGN 0xB0); the heading receiver (GPS2) gets its commands on UDP port 2234;
- the UM982's dual antenna heading ($GNHPR) is used as the dual heading;
- firmware update over Ethernet from AgIO (UDP port 2235): once one of these is on the Teensy (over USB), later
  firmwares go on without opening the box. The new one is received apart from the running program and installed only
  after its CRC-32 checks out; do not switch off during the last ~30 seconds (installing).

Otherwise they work as the original firmwares. Not tried on a tractor yet.

Source and build script (GPL-3.0): [`FirmwareSource`](../../FirmwareSource) in this repository.
Built with the Teensy package 1.57.3.
