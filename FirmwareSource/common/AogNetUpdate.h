// AogNetUpdate - firmware update over Ethernet for the AgOpenGPS Teensy 4.1 firmwares (added, not part of the
// original AgOpenGPS Boards code).
//
// AgIO sends the new firmware image to UDP port 2235 in pieces. It goes into the upper half of the flash, away from
// the running program, which keeps working meanwhile. Only when the whole image is there, its CRC-32 matches and it
// looks like a Teensy 4 image (FlexSPI configuration block "FCFB", boot data) is it copied over the program and the
// Teensy restarted. The copy takes a few seconds: power lost exactly then leaves no working firmware, and the
// Teensy must be programmed over USB again (its bootloader is a separate chip and always there).
//
// Messages (UDP, AgIO -> module), all little endian, after the 5-byte magic "AOGFW":
//   'Q'                                  query: answer with the firmware's name and the largest image
//   'B' size(u32) crc32(u32)             begin: erase the space for an image of this size
//   'D' offset(u32) length(u16) data...  a piece (offset a multiple of 256, at most 1024 bytes)
//   'E'                                  end: check the image and, if right, install it and restart
// Answers (to the sender) in PGN 0xB1: 80 81 78 B1 len, then the command letter in lower case, a status byte
// (0 = ok) and, for 'q', the protocol version and the name, for 'd', the offset.
//
// GPL-3.0, as the firmware it is part of.

#pragma once
#include <Arduino.h>
#include <NativeEthernetUdp.h>

extern "C" {
void eepromemu_flash_write(void* addr, const void* data, uint32_t len);
void eepromemu_flash_erase_sector(void* addr);
void eepromemu_flash_erase_64K_block(void* addr);
}

class AogNetUpdate
{
public:
    static const uint16_t Port = 2235;
    static const uint8_t PgnReply = 0xB1;
    static const uint8_t Version = 1;
    static const uint32_t FlashBase = 0x60000000;
    static const uint32_t BufferAddr = 0x60400000;     // upper half of the 8 MB flash
    static const uint32_t MaxImage = 0x00400000;       // the image must stay below the buffer
    static const uint32_t BufferEnd = 0x607C0000;      // the EEPROM emulation starts here

    enum Status : uint8_t { Ok = 0, BadMessage = 1, TooBig = 2, NotStarted = 3, OutOfOrder = 4, BadCrc = 5, NotTeensy4 = 6, Busy = 7 };

    void begin(const char* name)
    {
        name_ = name;
        udp_.begin(Port);
    }

    // in the loop: one message at most per call
    void poll()
    {
        int len = udp_.parsePacket();
        if (len <= 0) return;
        if (len > (int)sizeof(buf_)) len = sizeof(buf_);
        len = udp_.read(buf_, len);
        if (len < 6 || memcmp(buf_, "AOGFW", 5) != 0) return;
        IPAddress ip = udp_.remoteIP();
        uint16_t port = udp_.remotePort();
        const uint8_t* p = buf_ + 6;
        int n = len - 6;
        switch (buf_[5])
        {
        case 'Q': query(ip, port); break;
        case 'B': start(ip, port, p, n); break;
        case 'D': piece(ip, port, p, n); break;
        case 'E': finish(ip, port); break;
        default: break;
        }
    }

    static uint32_t crc32(const uint8_t* data, uint32_t len, uint32_t crc = 0)
    {
        crc = ~crc;
        for (uint32_t i = 0; i < len; i++)
        {
            crc ^= data[i];
            for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
        return ~crc;
    }

private:
    EthernetUDP udp_;
    const char* name_ = "";
    uint8_t buf_[1100];
    uint8_t page_[256];
    bool started_ = false;
    uint32_t size_ = 0, crc_ = 0, next_ = 0;

    static uint32_t u32(const uint8_t* p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

    void reply(IPAddress ip, uint16_t port, char cmd, uint8_t status, const uint8_t* extra = nullptr, uint8_t extraLen = 0)
    {
        uint8_t out[80];
        uint8_t len = 2 + extraLen;
        out[0] = 0x80;
        out[1] = 0x81;
        out[2] = 0x78;
        out[3] = PgnReply;
        out[4] = len;
        out[5] = (uint8_t)cmd;
        out[6] = status;
        if (extraLen) memcpy(out + 7, extra, extraLen);
        uint8_t crc = 0;
        for (int i = 2; i < 5 + len; i++) crc += out[i];
        out[5 + len] = crc;
        udp_.beginPacket(ip, port);
        udp_.write(out, 6 + len);
        udp_.endPacket();
    }

    void query(IPAddress ip, uint16_t port)
    {
        uint8_t extra[64];
        extra[0] = Version;
        extra[1] = (uint8_t)(MaxImage >> 16);   // in 64 KB
        size_t l = strlen(name_);
        if (l > sizeof(extra) - 2) l = sizeof(extra) - 2;
        memcpy(extra + 2, name_, l);
        reply(ip, port, 'q', Ok, extra, (uint8_t)(2 + l));
    }

    void start(IPAddress ip, uint16_t port, const uint8_t* p, int n)
    {
        if (n < 8) { reply(ip, port, 'b', BadMessage); return; }
        uint32_t size = u32(p), crc = u32(p + 4);
        if (size < 0x2000 || size > MaxImage || BufferAddr + size > BufferEnd) { reply(ip, port, 'b', TooBig); return; }
        // the space for it, in 64 KB blocks
        for (uint32_t a = BufferAddr; a < BufferAddr + size; a += 0x10000)
            eepromemu_flash_erase_64K_block((void*)a);
        size_ = size;
        crc_ = crc;
        next_ = 0;
        started_ = true;
        reply(ip, port, 'b', Ok);
    }

    void piece(IPAddress ip, uint16_t port, const uint8_t* p, int n)
    {
        uint8_t off[4];
        if (n < 6) { reply(ip, port, 'd', BadMessage); return; }
        uint32_t offset = u32(p);
        uint16_t len = (uint16_t)(p[4] | (p[5] << 8));
        memcpy(off, p, 4);
        if (!started_) { reply(ip, port, 'd', NotStarted, off, 4); return; }
        if (offset + len <= next_) { reply(ip, port, 'd', Ok, off, 4); return; }   // a repeat: taken already
        if (offset != next_ || (offset & 0xFF) != 0 || len > 1024 || n < 6 + len || offset + len > size_)
        {
            reply(ip, port, 'd', OutOfOrder, off, 4);
            return;
        }
        // flash pages are 256 bytes: written one page at a time
        for (uint32_t done = 0; done < len; done += 256)
        {
            uint32_t chunk = len - done < 256 ? len - done : 256;
            memcpy(page_, p + 6 + done, chunk);
            eepromemu_flash_write((void*)(BufferAddr + offset + done), page_, chunk);
        }
        next_ = offset + len;
        reply(ip, port, 'd', Ok, off, 4);
    }

    void finish(IPAddress ip, uint16_t port)
    {
        if (!started_ || next_ != size_) { reply(ip, port, 'e', NotStarted); return; }
        const uint8_t* image = (const uint8_t*)BufferAddr;
        if (crc32(image, size_) != crc_) { reply(ip, port, 'e', BadCrc); started_ = false; return; }
        // a Teensy 4 image: the FlexSPI configuration block at 0 ("FCFB"), the boot data (IVT, tag D1) at 0x1000
        if (memcmp(image, "FCFB", 4) != 0 || image[0x1000] != 0xD1) { reply(ip, port, 'e', NotTeensy4); started_ = false; return; }
        reply(ip, port, 'e', Ok);
        delay(200);   // the answer goes out before the network stops
        install(size_);
    }

    // over the running program: everything this needs runs from RAM (Teensy 4 runs its code from ITCM)
    FASTRUN void install(uint32_t size)
    {
        __disable_irq();
        for (uint32_t off = 0; off < size; off += 4096)
        {
            eepromemu_flash_erase_sector((void*)(FlashBase + off));
            for (uint32_t pg = 0; pg < 4096 && off + pg < size; pg += 256)
            {
                uint32_t chunk = size - (off + pg) < 256 ? size - (off + pg) : 256;
                memcpy(page_, (const void*)(BufferAddr + off + pg), chunk);
                eepromemu_flash_write((void*)(FlashBase + off + pg), page_, chunk);
                __disable_irq();
            }
            __disable_irq();
        }
        // restart into the new firmware
        SCB_AIRCR = 0x05FA0004;
        while (1) ;
    }
};
