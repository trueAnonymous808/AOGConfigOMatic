// GNSS bridge glue (added, not in the original AgOpenGPS Boards firmware):
//  - raw RTCM / UBX-RXM measurement frames of the position receiver -> AgIO (TDCP)
//  - receiver answers to configuration commands -> AgIO in PGN 0xB0
//  - commands for the heading receiver (GPS2) <- AgIO on UDP port 2234 (GPS1 keeps using the NTRIP port 2233)
//  - firmware update over Ethernet from AgIO (AogNetUpdate.h, UDP port 2235)
//  - UM982 dual antenna: $GNHPR heading / roll used as the dual heading, as an F9P RELPOSNED would be

AogGnssBridge gnss1, gnss2;
AogNetUpdate netUpdate;                   // firmware update over Ethernet (UDP 2235)
EthernetUDP Eth_udpGps2Cmd;               // In port 2234: bytes for the heading receiver
const unsigned int AOGGps2CmdPort = 2234;
char Eth_GPS2_packetBuffer[serial_buffer_size];

void gnssBridgeSend(const uint8_t* data, size_t len)
{
    Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
    Eth_udpPAOGI.write(data, len);
    Eth_udpPAOGI.endPacket();
}

// IMU summary (added): every serial BNO085 reading (100 Hz) into the summary sent with the next fix
void imuSummaryRvc()
{
    float h = bnoData.yawX100 * 0.01f;   // the raw yaw, 0.01 degree steps
    if (h < 0) h += 360;
    float r = (steerConfig.IsUseY_Axis ? bnoData.rollX10 : bnoData.pitchX10) * 0.1f;   // the roll as the PANDA has it
    imuSummary.add(h, r, 1, bnoData.accX * MILLI_G_TO_MS2, bnoData.accY * MILLI_G_TO_MS2, bnoData.accZ * MILLI_G_TO_MS2);
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

// UM982 HPR: the receiver turns its heading to the tractor's already (CONFIG HEADING OFFSET 90 / TRACTOR).
// Only a fixed dual solution makes the dual mode; without one (a single antenna, or the heading lost) the position
// goes on as with a single antenna - see gnssBridgeDualWatch.
bool hprDual = false;
uint32_t hprSeenMs = 0;

void gnssBridgeHpr(float headingDeg, float pitchDeg, float rollDeg, int quality)
{
    if (quality != 4) return;
    hprSeenMs = millis();
    hprDual = true;
    useDual = true;
    double h = headingDeg;
    while (h >= 360) h -= 360;
    while (h < 0) h += 360;
    heading = h;
    // tractor mode reports the side tilt as roll; without it the baseline's pitch is the side tilt
    rollDual = rollDeg != 0 ? rollDeg : pitchDeg;
#ifdef GPSGREEN_LED
    digitalWrite(GPSGREEN_LED, HIGH);
#endif
    imuHandler();
    dualReadyRelPos = true;
}

// no fixed dual heading from the HPR for 1.5 s: back to the single antenna way, so the position never stops
void gnssBridgeDualWatch()
{
    if (hprDual && millis() - hprSeenMs > 1500)
    {
        hprDual = false;
        useDual = false;
        dualReadyRelPos = false;
        dualReadyGGA = false;
    }
}

void gnssBridgeSetup()
{
    gnss1.begin(1, true, gnssBridgeSend, gnssBridgeText1, gnssBridgeHpr);
    gnss2.begin(2, false, gnssBridgeSend, nullptr, nullptr);
    netUpdate.begin("AIO v4 (OTA & RawGNSS), " __DATE__);
    if (Eth_udpGps2Cmd.begin(AOGGps2CmdPort))
    {
        Serial.print("Ethernet GPS2 command UDP listening to port: ");
        Serial.println(AOGGps2CmdPort);
    }
}

void gnssBridgeFeed1(uint8_t b)
{
    gnss1.feed(b);
}

void gnssBridgeFeed2(uint8_t b)
{
    gnss2.feed(b);
}

// commands from AgIO for the heading receiver
void gnssBridgeReceive()
{
    gnssBridgeDualWatch();
    netUpdate.poll();
    unsigned int len = Eth_udpGps2Cmd.parsePacket();
    if (len == 0) return;
    if (len > serial_buffer_size) len = serial_buffer_size;
    Eth_udpGps2Cmd.read(Eth_GPS2_packetBuffer, len);
    SerialGPS2->write(Eth_GPS2_packetBuffer, len);
}
