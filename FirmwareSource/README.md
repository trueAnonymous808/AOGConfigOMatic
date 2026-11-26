# Firmware sources (modified)

The sources of the firmwares in [`../Firmwares/AgOpenGPS-OTA-RawGNSS`](../Firmwares/AgOpenGPS-OTA-RawGNSS): AgOpenGPS Teensy 4.1
firmwares with a GNSS bridge and a network firmware update added for AgIO. Their own behaviour is unchanged
otherwise.

| Folder | Original | Built as |
|---|---|---|
| `AIO_v4_Firmware` | [AgOpenGPS Boards](https://github.com/AgOpenGPS-Official/Boards) `TeensyModules/AIO v4 Firmware`, commit `c8a3da7` | `All-In-One/AIO v4 (OTA & RawGNSS).hex` |
| `Autosteer_gps_teensy_v2_5` | AgOpenGPS Boards `TeensyModules/AIO v2.5/Firmware`, commit `c8a3da7` | `All-In-One/AIO v2.5 (OTA & RawGNSS).hex` |
| `AIO_v4_CANBUS_Firmware` | AgOpenGPS Boards `CANBUS/CANBUS Firmware/AIO_v4_CANBUS_Firmware`, commit `c8a3da7` | `CANBUS/CANBUS All-In-One (OTA & RawGNSS).hex`, and with `isAllInOneBoard` off `CANBUS/CANBUS CommonRail (OTA & RawGNSS).hex` (Tony's board) |
| `Keya_chriskinal` | [chriskinal/AOG_Teensy_UM982](https://github.com/chriskinal/AOG_Teensy_UM982) branch `Dual-Autoconf-KeyaV2` (see `SOURCE.txt`) | `Keya/Keya (OTA & RawGNSS).hex` |

All of them are GPL-3.0, as their originals; these modified sources are published with the firmwares built from them.

The other firmwares Config-O-Matic offers (Keya variants, the TM171 CANBUS betas, the brand fixes, the utilities)
have no published source that could be found, so there is no modified version of them.

## What was added

- `common/AogGnssBridge.h` sits between the GNSS receiver's serial port and the firmware's NMEA parser:
  - NMEA text goes on to the parser as before;
  - raw measurement frames of the position receiver (RTCM 3 MSM and ephemerides, u-blox RXM-RAWX / RXM-SFRBX)
    go to AgIO over UDP (port 9999) unchanged, for TDCP in AgOpenGPS;
  - the receivers' answers to configuration commands (Unicore `$command,...`, `#...` logs; u-blox ACK/NAK, MON,
    CFG) go to AgIO in PGN 0xB0: `80 81 78 B0 len port bytes... crc`;
  - a Unicore `$xxHPR` sentence (UM982 dual antenna) becomes the dual heading and roll, as an F9P RELPOSNED does
    (in the AgOpenGPS Boards firmwares; the Keya firmware handles HPR itself).
- `common/AogNetUpdate.h`: firmware update over Ethernet (UDP port 2235, answers in PGN 0xB1). The new image is
  received into the upper half of the flash (0x60400000) while the program runs on, checked (CRC-32, Teensy 4 boot
  header) and only then copied over the program from RAM, after which the Teensy restarts. Power lost during that
  copy (a few seconds) leaves the Teensy to be programmed over USB again; its bootloader is a separate chip.
- `zGnssBridge.ino` in each sketch: the glue. Bytes for the position receiver come on the NTRIP port 2233 as before;
  for the heading receiver (GPS2, dual F9P) on UDP port 2234.
- The position receiver's serial receive buffer is 8 KB (raw measurements make the stream several times larger),
  and the loop reads all waiting bytes (up to 1024) per pass instead of one.
- AIO_v4_CANBUS_Firmware: only the Panda / dual GPS modes use the bridge; the forwarding modes are unchanged.
- Every firmware works with any of the receivers, single or dual antenna, and finds out which as it runs: a single
  F9P or UM982 (GGA/VTG), a UM982 dual antenna (HPR), or two F9Ps (the second one's NAV-RELPOSNED on GPS2). Dual
  mode only while a fixed dual heading comes; without one for 1.5 s (a single antenna, or the heading lost) the
  position goes on as with a single antenna (PANDA), so it never stops.
- Keya: from chriskinal's UM982 Keya firmware, made universal: the single / dual mode is found as it runs (it was
  fixed to dual), and a second F9P on Serial2 is read as in the AIO firmware. Its automatic UM982 set-up (when
  switched on) turns the raw measurements for TDCP on as well.

## UM982 dual antenna (AgOpenGPS Boards firmwares)

The receiver is expected to be set up as Config-O-Matic's `ConfigUM982D.txt` does (`CONFIG HEADING OFFSET 90.0`,
`CONFIG HEADING TRACTOR`, `GPHPR COM1 0.1`): the HPR heading is the tractor's heading already. The roll is the HPR
roll field, or the pitch when the roll field is 0. **Not tried on a tractor yet** - check heading and roll sign.

## Building

The shared parts (`common/AogGnssBridge.h`, `common/AogNetUpdate.h`) are kept only in `common/`; `build.ps1`
copies them into each sketch before it builds (open a sketch in the Arduino IDE after one `build.ps1` run, or copy
them by hand).

`build.ps1` (PowerShell) builds them all and writes the .hex files to `../Firmwares/AgOpenGPS-OTA-RawGNSS`. Once those are
committed and pushed, the AgIO module configurator (AgIO > Settings > Module configurator) downloads them.

It needs `arduino-cli` with the Teensy package **1.57.3** (these firmwares do not build with 1.58 or later:
`addMemoryForRead` moved) and the `SimpleKalmanFilter` library for the UM982 firmware:

```
arduino-cli config add board_manager.additional_urls https://www.pjrc.com/teensy/package_teensy_index.json
arduino-cli core install teensy:avr@1.57.3
arduino-cli lib install SimpleKalmanFilter
```
