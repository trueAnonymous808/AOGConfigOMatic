// IMU summary (added, not in the original AgOpenGPS Boards firmware):
// the PANDA sentence carries one IMU reading per GNSS fix (10 Hz), while the IMU is read 50-100 times a second.
// What happens between two fixes - how fast it turned, the roll on average and its peaks, how fast the roll changed
// at most (a pothole) - is summed up here and sent just before each PANDA in PGN 0xB2. The AgIO forwards it to
// AgOpenGPS like any module PGN; an AgOpenGPS that does not know it ignores it, so nothing changes for it.
//
//   0x80 0x81 0x7F 0xB2 len(18) payload crc
//   payload, little endian:
//     int16  yaw rate, deg/s x100 (heading increasing = +), averaged over the interval
//     int16  roll average, deg x100        int16 roll min, deg x100      int16 roll max, deg x100
//     int16  roll of the last sample, deg x100
//     int16  fastest roll change over 50 ms or more, deg/s x10 (signed)
//     int16  heading of the last sample, deg x10
//     uint16 interval covered, ms
//     uint8  samples                       uint8 IMU: 1 BNO085 serial (RVC), 2 BNO085 I2C
//   crc: the low byte of the sum of the bytes from the third on (as every AgOpenGPS PGN)

#pragma once
#include <Arduino.h>

class AogImuSummary
{
public:
    // one IMU reading: heading and roll in degrees (the roll as it goes in the PANDA: the axis chosen already)
    void add(float headingDeg, float rollDeg, uint8_t imuType)
    {
        uint32_t now = micros();
        type = imuType;
        if (havePrev)
        {
            float dt = (now - prevUs) * 1e-6f;
            if (dt > 0.0005f && dt < 0.5f)
            {
                float dh = headingDeg - prevHeading;
                while (dh > 180) dh -= 360;
                while (dh < -180) dh += 360;
                turned += dh;
                spanS += dt;
            }
        }
        // the roll's rate over 50 ms at least: from one reading to the next (10 ms) a single 0.1 degree step of the
        // IMU would already look like 10 deg/s
        float refDt = (now - refUs) * 1e-6f;
        if (!haveRef || refDt > 0.5f)
        {
            haveRef = true;
            refUs = now;
            refRoll = rollDeg;
        }
        else if (refDt >= 0.05f)
        {
            float rate = (rollDeg - refRoll) / refDt;
            if (fabsf(rate) > fabsf(rollRatePeak)) rollRatePeak = rate;
            refUs = now;
            refRoll = rollDeg;
        }
        havePrev = true;
        prevUs = now;
        prevHeading = headingDeg;
        prevRoll = rollDeg;

        // (255 readings at most: no fix for 2.5 s - the average stays that of the ones counted)
        if (count == 255) return;
        rollSum += rollDeg;
        if (count == 0 || rollDeg < rollMin) rollMin = rollDeg;
        if (count == 0 || rollDeg > rollMax) rollMax = rollDeg;
        count++;
    }

    // with every fix, before the PANDA: what came since the last one, then it starts again
    template <typename SendFn>
    void send(SendFn sendFn)
    {
        if (count == 0) return;   // no IMU (or a dual antenna in its place): nothing
        uint8_t p[24];
        p[0] = 0x80;
        p[1] = 0x81;
        p[2] = 0x7F;
        p[3] = 0xB2;
        p[4] = 18;
        int i = 5;
        put16(p, i, spanS > 0.005f ? turned / spanS * 100 : 0);
        put16(p, i, rollSum / count * 100);
        put16(p, i, rollMin * 100);
        put16(p, i, rollMax * 100);
        put16(p, i, prevRoll * 100);
        put16(p, i, rollRatePeak * 10);
        put16(p, i, prevHeading * 10);
        float ms = spanS * 1000;
        uint16_t span = ms > 65000 ? 65000 : (uint16_t)ms;
        p[i++] = (uint8_t)span;
        p[i++] = (uint8_t)(span >> 8);
        p[i++] = count;
        p[i++] = type;
        uint8_t crc = 0;
        for (int j = 2; j < i; j++) crc += p[j];
        p[i++] = crc;
        sendFn(p, (size_t)i);
        clear();
    }

    // the interval forgotten (a dual antenna in use: its roll and heading go to AgOpenGPS, not the IMU's)
    void clear()
    {
        count = 0;
        rollSum = 0;
        turned = 0;
        spanS = 0;
        rollRatePeak = 0;
    }

private:
    bool havePrev = false, haveRef = false;
    uint32_t prevUs = 0, refUs = 0;
    float prevHeading = 0, prevRoll = 0, refRoll = 0;
    float turned = 0, spanS = 0, rollSum = 0, rollMin = 0, rollMax = 0, rollRatePeak = 0;
    uint8_t count = 0, type = 0;

    static void put16(uint8_t* p, int& i, float v)
    {
        if (v > 32767) v = 32767;
        if (v < -32767) v = -32767;
        int16_t s = (int16_t)lroundf(v);
        p[i++] = (uint8_t)s;
        p[i++] = (uint8_t)((uint16_t)s >> 8);
    }
};
