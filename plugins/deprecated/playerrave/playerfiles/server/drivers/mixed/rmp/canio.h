#ifndef _CANIO_H_
#define _CANIO_H_

#if HAVE_CONFIG_H
  #include <config.h>
#endif
#if HAVE_STDINT_H
  #include <stdint.h>
#endif

#include <sys/types.h>
#include <string.h>
#include <stdio.h>

// Copied this from <canlib.h>. I assume that it's portable across CAN
// implementations.
#ifndef canMSG_STD
  #define canMSG_STD              0x0002
#endif


class CanPacket
{
  public:
    long id;
    uint8_t msg[8];
    uint32_t dlc;
    uint32_t flags;

    CanPacket()
    {
      memset(msg,0,sizeof(msg));

      flags = canMSG_STD;
      dlc = 8;
    }

    uint16_t GetSlot(int s)  const 
    {
      return (uint16_t) ((msg[s*2] << 8) | (msg[s*2+1]));
    }

    void PutSlot(const int slot, const uint16_t val) 
    {
      msg[slot*2] = (val >> 8) & 0xFF;
      msg[slot*2+1] = val & 0xFF;
    }

    void PutByte(const int byte, const uint16_t val) 
    {
      msg[byte] = val & 0xFF;
    }

    char* toString() 
    {
      static char buf[256];
      sprintf(buf, "id:%04lX %02X %02X %02X %02X %02X %02X %02X %02X",
              id, msg[0], msg[1], msg[2], msg[3], msg[4], msg[5], 
              msg[6], msg[7]);

      return buf;
    }
};


/* this class encapsulates the low level CAN stuff.... so it deals
   with reading and writing packets on the CAN channels.
   We make the assumption that we only have to read off of one
   channel though (looking at rmi_demo, it appears that this is
   OK.)
   A higher level entity will make sense of the packets, and call
   the read/write methods with the required timing.

   It wouldn't take much to make this an abstract base class so that
   the SegwayIO can use it, and then have different CAN hardwares
   implement the virtual methods.  So then we can just drop in 
   a new CAN hardware driver class and everything would still work.
   Would also be able to split up the files, so we could keep
   canio.[cc,h] in player, and the CAN hardware specific files
   can be local.
*/

class CANIO
{
  public:
    CANIO() {}
    virtual ~CANIO() {}
    virtual int Init() = 0;
    virtual int ReadPacket(CanPacket *pkt) = 0;
    virtual int ReadFrame(CanPacket *pkt) {printf("in the wrong readframe\n"); return 0;};
    virtual int WritePacket(CanPacket &pkt) = 0;
    virtual int Shutdown() = 0;
};

#endif
