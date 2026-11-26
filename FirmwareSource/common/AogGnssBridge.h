// AogGnssBridge - added to the AgOpenGPS Teensy firmwares (not part of the original AgOpenGPS Boards code).
//
// Sits between a GNSS receiver's serial port and the firmware's NMEA parser, byte by byte:
//  - NMEA / ASCII text goes on to the parser unchanged, so the firmware works as before;
//  - raw measurement frames (RTCM 3, u-blox RXM-RAWX / RXM-SFRBX) of the position receiver go to AgIO
//    unchanged over UDP, for TDCP (carrier phase movement) in AgOpenGPS;
//  - the receiver's answers to configuration commands (Unicore "$command,...", u-blox ACK/NAK/MON/CFG)
//    go to AgIO in PGN 0xB0, so AgIO's configurator sees whether a setting was taken;
//  - a Unicore $xxHPR sentence (UM982 dual antenna heading / pitch) is handed to the firmware.
//
// Header only, no Arduino dependencies: the firmware gives it a function to send UDP and one to feed its parser.
//
// GPL-3.0, as the firmware it is part of.

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

class AogGnssBridge
{
public:
    static const uint8_t PgnReply = 0xB0;        // receiver answer: [port][bytes...]
    static const uint8_t SourceGps = 0x78;       // 120: the GPS module
    static const size_t MaxFrame = 2200;         // RXM-RAWX with 64 signals fits
    static const size_t MaxLine = 240;
    static const size_t MaxReply = 240;          // what fits in one PGN (length is one byte)

    typedef void (*SendFn)(const uint8_t* data, size_t len);
    typedef void (*TextFn)(char c);
    typedef void (*HprFn)(float headingDeg, float pitchDeg, float rollDeg, int quality);
    typedef void (*UbxFn)(const uint8_t* frame, size_t len);   // any other checked UBX frame (e.g. NAV-RELPOSNED)

    // port: 1 position receiver, 2 heading receiver. forwardRaw: send raw measurement frames (port 1 only).
    void begin(uint8_t port, bool forwardRaw, SendFn send, TextFn text, HprFn hpr, UbxFn ubx = nullptr)
    {
        ubx_ = ubx;
        port_ = port;
        forwardRaw_ = forwardRaw;
        send_ = send;
        text_ = text;
        hpr_ = hpr;
        reset();
    }

    uint32_t rawFrames() const { return rawFrames_; }
    uint32_t replies() const { return replies_; }
    uint32_t badFrames() const { return badFrames_; }

    void feed(uint8_t b)
    {
        switch (state_)
        {
        case Idle:
            if (b == 0xD3) { startFrame(b); state_ = Rtcm; }
            else if (b == 0xB5) { startFrame(b); state_ = Ubx; }
            else if (b == '$' || b == '#') { lineLen_ = 0; addLine(b); state_ = Text; passText(b); }
            else passText(b);   // stray text: the parser decides
            break;

        case Text:
            passText(b);
            if (b == '\n' || b == '\r')
            {
                endLine();
                state_ = Idle;
            }
            else if (b < 0x20 || b > 0x7E)
            {
                // not text after all (a binary frame started inside): take it from here
                state_ = Idle;
                lineLen_ = 0;
                feed(b);
            }
            else addLine(b);
            break;

        case Rtcm:
            frame_[frameLen_++] = b;
            if (frameLen_ == 2 && (b & 0xFC) != 0) { abandon(); break; }   // reserved bits must be 0
            if (frameLen_ == 3) want_ = 3 + (((size_t)(frame_[1] & 0x03) << 8) | frame_[2]) + 3;
            if (frameLen_ >= 3 && frameLen_ == want_) { rtcmDone(); state_ = Idle; }
            break;

        case Ubx:
            frame_[frameLen_++] = b;
            if (frameLen_ == 2 && b != 0x62) { abandon(); break; }
            if (frameLen_ == 6)
            {
                size_t len = (size_t)frame_[4] | ((size_t)frame_[5] << 8);
                want_ = 6 + len + 2;
                if (want_ > MaxFrame) { abandon(); break; }
            }
            if (frameLen_ >= 6 && frameLen_ == want_) { ubxDone(); state_ = Idle; }
            break;
        }
        if (state_ != Text && state_ != Idle && frameLen_ >= MaxFrame) abandon();
    }

    // CRC-24Q of RTCM 3
    static uint32_t crc24q(const uint8_t* p, size_t n)
    {
        uint32_t crc = 0;
        for (size_t i = 0; i < n; i++)
        {
            crc ^= (uint32_t)p[i] << 16;
            for (int k = 0; k < 8; k++)
            {
                crc <<= 1;
                if (crc & 0x1000000) crc ^= 0x1864CFB;
            }
        }
        return crc & 0xFFFFFF;
    }

    // a PGN for AgIO: 0x80 0x81 source pgn length data... crc (sum of source..data)
    static size_t buildPgn(uint8_t* out, uint8_t pgn, const uint8_t* data, size_t len)
    {
        if (len > 250) len = 250;
        out[0] = 0x80;
        out[1] = 0x81;
        out[2] = SourceGps;
        out[3] = pgn;
        out[4] = (uint8_t)len;
        memcpy(out + 5, data, len);
        uint8_t crc = 0;
        for (size_t i = 2; i < 5 + len; i++) crc += out[i];
        out[5 + len] = crc;
        return 6 + len;
    }

private:
    enum State { Idle, Text, Rtcm, Ubx };

    State state_ = Idle;
    uint8_t port_ = 1;
    bool forwardRaw_ = false;
    SendFn send_ = nullptr;
    TextFn text_ = nullptr;
    HprFn hpr_ = nullptr;
    UbxFn ubx_ = nullptr;
    uint8_t frame_[MaxFrame];
    size_t frameLen_ = 0, want_ = 0;
    char line_[MaxLine + 1];
    size_t lineLen_ = 0;
    uint8_t pgn_[MaxReply + 8];
    uint32_t rawFrames_ = 0, replies_ = 0, badFrames_ = 0;

    void reset()
    {
        state_ = Idle;
        frameLen_ = want_ = lineLen_ = 0;
    }

    void passText(uint8_t b)
    {
        if (text_) text_((char)b);
    }

    void startFrame(uint8_t b)
    {
        frameLen_ = 0;
        want_ = 0;
        frame_[frameLen_++] = b;
    }

    // a frame that turned out not to be one: its bytes are text for the parser (it finds its own way)
    void abandon()
    {
        badFrames_++;
        for (size_t i = 0; i < frameLen_; i++) passText(frame_[i]);
        frameLen_ = 0;
        state_ = Idle;
    }

    void addLine(uint8_t b)
    {
        if (lineLen_ < MaxLine) line_[lineLen_++] = (char)b;
    }

    void rtcmDone()
    {
        size_t body = frameLen_ - 3;
        uint32_t crc = ((uint32_t)frame_[body] << 16) | ((uint32_t)frame_[body + 1] << 8) | frame_[body + 2];
        if (crc24q(frame_, body) != crc) { badFrames_++; frameLen_ = 0; return; }
        if (forwardRaw_ && send_) { send_(frame_, frameLen_); rawFrames_++; }
        frameLen_ = 0;
    }

    void ubxDone()
    {
        uint8_t a = 0, b = 0;
        for (size_t i = 2; i < frameLen_ - 2; i++) { a += frame_[i]; b += a; }
        if (a != frame_[frameLen_ - 2] || b != frame_[frameLen_ - 1]) { badFrames_++; frameLen_ = 0; return; }
        uint8_t cls = frame_[2], id = frame_[3];
        if (cls == 0x02 && (id == 0x15 || id == 0x13))
        {
            // RXM-RAWX / RXM-SFRBX: carrier phase and navigation data for TDCP
            if (forwardRaw_ && send_) { send_(frame_, frameLen_); rawFrames_++; }
        }
        else if (cls == 0x05 || cls == 0x0A || cls == 0x06 || cls == 0x27)
        {
            // ACK/NAK, MON (version, ...), CFG answers, SEC (unique id): for the configurator
            reply(frame_, frameLen_);
        }
        else if (ubx_) ubx_(frame_, frameLen_);
        frameLen_ = 0;
    }

    void endLine()
    {
        line_[lineLen_] = 0;
        if (lineLen_ < 6) { lineLen_ = 0; return; }
        // Unicore: $GNHPR,utc,heading,pitch,roll,quality,...
        if (line_[0] == '$' && line_[3] == 'H' && line_[4] == 'P' && line_[5] == 'R' && hpr_) parseHpr();
        // Unicore answers ($command,...,response: OK) and its own logs (#VERSIONA, ...); u-blox $PUBX
        else if (strncmp(line_, "$command", 8) == 0 || line_[0] == '#' || strncmp(line_, "$PUBX", 5) == 0)
            reply((const uint8_t*)line_, lineLen_);
        lineLen_ = 0;
    }

    void reply(const uint8_t* data, size_t len)
    {
        if (!send_) return;
        uint8_t body[MaxReply];
        if (len > MaxReply - 1) len = MaxReply - 1;
        body[0] = port_;
        memcpy(body + 1, data, len);
        size_t n = buildPgn(pgn_, PgnReply, body, len + 1);
        send_(pgn_, n);
        replies_++;
    }

    void parseHpr()
    {
        // fields after the talker: 1 utc, 2 heading, 3 pitch, 4 roll, 5 quality
        const char* f[8] = { 0 };
        int n = 0;
        for (size_t i = 0; i < lineLen_ && n < 8; i++)
            if (line_[i] == ',') f[n++] = line_ + i + 1;
        if (n < 5 || !f[1] || !f[2] || !f[3] || !f[4]) return;
        if (*f[1] == ',' || *f[2] == ',') return;   // no heading yet
        float heading = (float)atof(f[1]);
        float pitch = (float)atof(f[2]);
        float roll = (float)atof(f[3]);
        int quality = atoi(f[4]);
        hpr_(heading, pitch, roll, quality);
    }
};
