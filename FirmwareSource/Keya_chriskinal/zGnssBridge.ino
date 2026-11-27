// GNSS bridge glue (added, not in the original firmware):
//  - raw RTCM / UBX-RXM measurement frames of the position receiver -> AgIO (TDCP)
//  - receiver answers to configuration commands -> AgIO in PGN 0xB0 (position receiver: commands on the NTRIP
//    port 2233; heading receiver: UDP port 2234)
//  - a second F9P on Serial2 (dual antenna with two F9Ps): its NAV-RELPOSNED becomes the dual heading and roll,
//    as in the AgOpenGPS AIO firmware
//  - firmware update over Ethernet from AgIO (AogNetUpdate.h, UDP port 2235)
// The firmware's own NMEA parser keeps handling GGA / VTG / HPR (UM982). Single or dual antenna is found as it runs.

AogGnssBridge gnss1, gnss2;
AogNetUpdate netUpdate;                   // firmware update over Ethernet (UDP 2235)
HardwareSerial* SerialGPS2 = &Serial2;    // the heading F9P of a two-F9P dual antenna (AiO board GPS2)
EthernetUDP Eth_udpGps2Cmd;               // In port 2234: bytes for the heading receiver
const unsigned int AOGGps2CmdPort = 2234;
char Eth_GPS2_packetBuffer[512];

void gnssBridgeSend(const uint8_t* data, size_t len)
{
    if (!Ethernet_running) return;
    Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
    Eth_udpPAOGI.write(data, len);
    Eth_udpPAOGI.endPacket();
}

// IMU summary (added): the I2C BNO085 sends two reports now (rotation and acceleration) - a few packets read, the
// newest of each kept
bool imuSummaryI2cAvailable()
{
    bool any = false;
    for (int k = 0; k < 3 && bno08x.dataAvailable(); k++) any = true;
    return any;
}

void gnssBridgeText1(char c)
{
    parser << c;
}

static int32_t relPosI32(const uint8_t* f, int o)
{
    return (int32_t)f[o] | ((int32_t)f[o + 1] << 8) | ((int32_t)f[o + 2] << 16) | ((int32_t)f[o + 3] << 24);
}

// UBX-NAV-RELPOSNED of the heading F9P (offsets in the whole frame, as the AIO firmware reads it)
void gnssBridgeRelPos(const uint8_t* f, size_t len)
{
    if (len < 72 || f[2] != 0x01 || f[3] != 0x3C) return;
    uint32_t flags = f[66];
    bool gnssFixOk = flags & 1, diffSoln = flags & 2, relPosValid = flags & 4;
    int carrSoln = (flags & 24) >> 3;
    // only a fixed solution makes the dual mode; without one the single antenna way goes on
    if (!gnssFixOk || !diffSoln || !relPosValid || carrSoln != 2) return;

    double h = relPosI32(f, 30) * 0.00001 + 90;   // antennas left / right: the tractor's heading is 90 degrees on
    while (h >= 360) h -= 360;
    while (h < 0) h += 360;
    double baseline = relPosI32(f, 26) + (int8_t)f[41] * 0.01;   // cm
    double relPosD = relPosI32(f, 22) + (int8_t)f[40] * 0.01;
    if (baseline == 0) baseline = 0.01;
    double ratio = relPosD / baseline;
    if (ratio > 1) ratio = 1;
    if (ratio < -1) ratio = -1;

    heading = h;
    rollDual = asin(ratio) * -RAD_TO_DEG;
    headingcorr = 900;   // antennas left / right (the IMU fusion corrects its roll for that)
    dualSeenMs = systick_millis_count;
    makeOGI = true;
    digitalWrite(GPSRED_LED, LOW);
    digitalWrite(GPSGREEN_LED, HIGH);
    if (useBNO08x && baseLineCheck)
    {
        imuDualDelta();          // the IMU runs, corrected by this dual reading
        dualReadyRelPos = false;
    }
    else
    {
        imuHandler();            // the dual data direct
        dualReadyRelPos = true;
    }
}

uint8_t GPS2bridgeRx[2048];

void gnssBridgeSetup()
{
    gnss1.begin(1, true, gnssBridgeSend, gnssBridgeText1, nullptr);
    gnss2.begin(2, false, gnssBridgeSend, nullptr, nullptr, gnssBridgeRelPos);
    SerialGPS2->begin(baudGPS);
    SerialGPS2->addMemoryForRead(GPS2bridgeRx, sizeof(GPS2bridgeRx));
    if (Ethernet_running)
    {
        netUpdate.begin(FIRMWARE_NAME ", " __DATE__);
        if (Eth_udpGps2Cmd.begin(AOGGps2CmdPort))
        {
            Serial.print("Ethernet GPS2 command UDP listening to port: ");
            Serial.println(AOGGps2CmdPort);
        }
    }
}

void gnssBridgeFeed1(uint8_t b)
{
    gnss1.feed(b);
}

void gnssBridgeReceive()
{
    // the heading F9P, when there is one
    for (int n = 0; n < 512 && SerialGPS2->available(); n++)
        gnss2.feed(SerialGPS2->read());

    // parsePacket() blocks when Ethernet is not running
    if (!Ethernet_running) return;
    netUpdate.poll();
    unsigned int len = Eth_udpGps2Cmd.parsePacket();
    if (len == 0) return;
    if (len > sizeof(Eth_GPS2_packetBuffer)) len = sizeof(Eth_GPS2_packetBuffer);
    Eth_udpGps2Cmd.read(Eth_GPS2_packetBuffer, len);
    SerialGPS2->write(Eth_GPS2_packetBuffer, len);
}
