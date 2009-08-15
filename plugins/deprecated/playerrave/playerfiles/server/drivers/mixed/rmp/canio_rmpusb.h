#ifndef _RMPUSB_CANLIB_
#define _RMPUSB_CANLIB_

#include "ftd2xx.h"
#include "canio.h"

class CANIOrmpusb : public CANIO
{
  private:
	FT_STATUS	ftStatus;
    static const unsigned int RX_BUFFER_SIZE = 1000;
    static const unsigned int RX_WARNING_SIZE = 360;
    static const unsigned int RX_KEEP_SIZE = 360;
    unsigned char rxbuf[RX_BUFFER_SIZE];
    
    bool ready;
    int rcount;
    int timeouts;
    int writecount, readcount;
    int usbreadcount, nomsgreadcount;
    int purgecount;
    unsigned int rxbufcount;
    class rmpUsbPacket
    {
    public:
        rmpUsbPacket();
        rmpUsbPacket(CanPacket &cpkt);
        ~rmpUsbPacket();
        void InitPacket();
        unsigned char computeChecksum();
        void addChecksum();
        char *toString();

        unsigned char bytes[18];
        unsigned char *pID;
        unsigned char *pDATA;
    };

    CanPacket packet_to_send;

    int SendPacket();
    int ExtractPacketsFromBuffer(CanPacket *pkt,int max_packets,bool *got_packet_0x400);    
  public:
    static const unsigned int MAX_EXTRACTED_PACKETS = 20;
	FT_HANDLE	ftHandle;

    CANIOrmpusb();
    virtual ~CANIOrmpusb();
    virtual int Init();
    virtual int ReadPacket(CanPacket *pkt);
    virtual int ReadFrame(CanPacket *pkt);
    virtual int WritePacket(CanPacket &pkt);
    virtual int Shutdown();
};

#endif // _RMPUSB_CANLIB_
