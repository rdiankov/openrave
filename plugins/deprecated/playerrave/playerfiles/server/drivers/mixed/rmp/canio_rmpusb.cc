
#include "canio_rmpusb.h"
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <list>
#include <map>

CANIOrmpusb::CANIOrmpusb() : ready(false)
{
}

#define STATS
#ifdef STATS
struct ReceiveStats {
    double time;
    int id;
    unsigned int bytes;
};

std::list<ReceiveStats> rslist;
long sleeptotal=0;
int sleepcount =0;
#endif

static int writeslots=0;
static int busyslots=0;
static int busybytes=0;


CANIOrmpusb::~CANIOrmpusb()
{
#ifdef STATS
    printf("%d write slots were open\n",writeslots);
    printf("%d busy slots, avg %d bytes each\n",busyslots,busyslots?busybytes/busyslots:-1);
    //    printf("Average sleep time: %3.3f usecs\n",(double)sleeptotal/(double)sleepcount);
    FILE *fout;
    fout = fopen("packet_stats","w");
    double totaltime=0;
    unsigned int totalbytes=0;
    std::map<int,int> histograph;
    if (fout) {
        std::list<ReceiveStats>::iterator it = rslist.begin();
        while (it != rslist.end()) {
            fprintf(fout,"Packet %d inside %d bytes, read %3.3f msecs after write\n",it->id,it->bytes,it->time);
            int msecs = ((int) (it->time * 10.0 + 0.5)); // store as 10* msecs
            histograph[msecs]++;
            totaltime+=it->time;
            totalbytes+=it->bytes;
            ++it;
        }
        //        fprintf(fout,"%d bytes in %3.3f seconds = %3.3f packets/second\n",
        //                totalbytes,totaltime,totalbytes/18/totaltime*1000.0);
        fclose(fout);
        printf("Summary of write windows:\n");
        for (std::map<int,int>::iterator hit=histograph.begin(); hit!=histograph.end(); ++hit) {
            printf("  %2.1f msecs : %d occurances\n",hit->first/10.0,hit->second);
        }
    }
    printf("read packet statistics written to file packet_stats\n");
#endif
}
        


int
CANIOrmpusb::Init()
{
  DWORD iVID = 0x0403;  // Vendor ID for Future Technology Devices Inc
  DWORD iPID = 0xE729;  // Product ID for FTD245BM chip in the Segway RMP
  
  ftStatus = FT_SetVIDPID(iVID, iPID);	// use our VID and PID
  if(ftStatus != FT_OK) {
    printf("Unable to set appropriate VID/PID for USB device\n");
    return ftStatus;
  }
  
  char *desc = strdup("Robotic Mobile Platform");
  ftStatus = FT_OpenEx(desc,FT_OPEN_BY_DESCRIPTION,&ftHandle);
  delete(desc);
  if(ftStatus != FT_OK) {
    /* 
       This can fail if the ftdi_sio driver is loaded.
       Use lsmod to check this and "rmmod ftdi_sio" or "modeprobe -r ftdi_sio"
       to remove.  Also remove usbserial.
    */
    printf("FT_Open(0) failed\n");
    return ftStatus;
  }
    
    // Boost the baud rate
    ftStatus = FT_SetBaudRate(ftHandle,FT_BAUD_460800);
	if(ftStatus != FT_OK) {
		printf("Unable to increase USB baud rate\n");
        FT_Close(ftHandle);
		return ftStatus;
	}

    // Decrease the internal latency timer
    ftStatus = FT_SetLatencyTimer(ftHandle,2);
    if (ftStatus != FT_OK) {
        printf("Unable to decrease latency timer...continuing anyway\n");
    }
    

    // Set a timeout value of 10ms for reads and writes
    ftStatus = FT_SetTimeouts(ftHandle,10,10);
    if (ftStatus != FT_OK) {
        printf("FT_SetTimeouts failed\n");
        FT_Close(ftHandle);
        return ftStatus;
    }
    
    DWORD rsize;
    ftStatus = FT_GetQueueStatus(ftHandle,&rsize);
    if (ftStatus != FT_OK) {
        printf("Unable to get Queue status\n");
    } else {
        printf("At Init there are %d characters in read queue\n",rsize);
    }

    ftStatus = FT_ResetDevice(ftHandle);
    if (ftStatus != FT_OK) {
        printf("Unable to reset USB FIFO\n");
        FT_Close(ftHandle);
        return ftStatus;
    }
 
    ftStatus = FT_Purge(ftHandle,FT_PURGE_RX | FT_PURGE_TX);
    if (ftStatus != FT_OK) {
        printf("Unable to clear USB read/write buffers\n");
        FT_Close(ftHandle);
        return ftStatus;
    }

    ftStatus = FT_ResetDevice(ftHandle);
    if (ftStatus != FT_OK) {
        printf("Unable to reset USB FIFO\n");
        FT_Close(ftHandle);
        return ftStatus;
    }

    ftStatus = FT_GetQueueStatus(ftHandle,&rsize);
    if (ftStatus != FT_OK) {
        printf("Unable to get Queue status\n");
    } else {
        printf("After purge/reset there are %d characters in read queue\n",rsize);
    }
    ready = true;
    rcount = timeouts = 0;
    rxbufcount = 0;
    writecount = readcount = 0;
    usbreadcount = nomsgreadcount = 0;
    purgecount = 0;
    return 0;
}

/* Closes the CAN channel
 *
 * returns: 0 on success, nonzero error code otherwise
 */
int
CANIOrmpusb::Shutdown()
{
    ftStatus = FT_Close(ftHandle);
	if(ftStatus != FT_OK) {
		printf("FT_Close failed\n");
		return ftStatus;
	}
    ready = false;
    return 0;
}
  
/* The driver has to transmit the packet during a specific timeslot
   on the CANbus, so just queue the packet for sending when ready.
*/

int
CANIOrmpusb::WritePacket(CanPacket &cpkt)
{
    packet_to_send = cpkt;
    return 0;
}

/* Writes the given packet
 *
 * returns: 0 on success, nonzero error code otherwise
 */
int
CANIOrmpusb::SendPacket()
{
    DWORD bytes_written;
    DWORD bytes_to_write;

    writecount++;
    // #define PRINTSTATS
#ifdef PRINTSTATS
    if ((writecount % 10) == 0) {
        printf("writes %d  read calls %d  device reads %d  aborted reads %d  purged bytes %d  \n",writecount,readcount,usbreadcount,nomsgreadcount,purgecount);
        writecount=readcount=usbreadcount=nomsgreadcount=purgecount=0;
    }
#endif

    if (!ready) {
        return -1;
    }

    static struct timeval *last=NULL;
    struct timeval curr;
    
    // initialize the first timeval if necessary
    if (!last) {
        last = new struct timeval;
        gettimeofday(last,NULL);
    }

    // get the current time
    gettimeofday(&curr,NULL);
    
    // calculate how long since the last write
    double msecs = (curr.tv_sec - last->tv_sec)*1000 + (curr.tv_usec - last->tv_usec) / 1000;
    
    // if it's been less than 30 milliseconds, sleep so that we don't
    // overload the CAN bus
    if (msecs < 30) {
        return 0;
        // usleep((30-msecs)*1000);
    }

    
    //    printf("CAN packet created: %s\n", packet_to_send.toString());

    //    if (packet_to_send.id == 0x413) {
    //        int vel = packet_to_send.GetSlot(0);
    //        if (vel) {
    //            printf("non-zero velocity\n");
    //      }
    //    }

    // create a USB packet from our CAN packet
    CANIOrmpusb::rmpUsbPacket packet(packet_to_send);

    //    printf("USB packet created: %s\n",packet.toString());
    
    bytes_to_write = 18;
    unsigned char *pData = packet.bytes;
    while (bytes_to_write > 0) {
        ftStatus = FT_Write(ftHandle,pData,bytes_to_write,&bytes_written);
        if (ftStatus != FT_OK) {
            printf("Error while trying to write packet to USB port\n");
            return ftStatus;
        }
        bytes_to_write -= bytes_written;
        pData += bytes_written;
    }
    //    printf("USB packet sent: %s\n",packet.toString());

    // record the time
    gettimeofday(last,NULL);

    return 0;
}
        
static ReceiveStats rs;

int CANIOrmpusb::ExtractPacketsFromBuffer(CanPacket *pkt,int max_packets,bool *got_packet_0x400) {
    // printf("read %d bytes\n",bytes_read);

    int packnum=0;
    int i;
    CANIOrmpusb::rmpUsbPacket packet;
    // Search for the next valid packet
    for ( i = 0; (i < ((int)rxbufcount-17)) && (packnum < max_packets); ) {
        if (rxbuf[i] != 0xF0) { // all usb messages start with F0
            i++;
            continue;
        }
        // move bytes into 18-byte USB packet
        memcpy(packet.bytes,rxbuf+i,18);
        rs.id = ((packet.bytes[4] << 3) | ((packet.bytes[5] >> 5) & 7)) & 0xfff;
        if (rxbuf[i+1] == 0) {  // heartbeat
            rs.id = 9999;
            rslist.push_back(rs);
            i+=18;
            continue;
        }
        if (rxbuf[i+2] == 0xBB) {  // side-b
            rs.id += 1000; // mark the side-b messages differently
            rslist.push_back(rs);
            i+=18;
            continue;
        }
        if ((rxbuf[i+1] != 0x55) || (rxbuf[i+2] != 0xAA)) {
            // only pay attention to non-heartbeat, side A messages
            rs.id = 5555;
            rslist.push_back(rs);
            i+=18;
            continue;
        }
        
        // do Checksum
        if(packet.computeChecksum() != packet.bytes[17]) {
            i+=18;
            rs.id = 5555;
            rslist.push_back(rs);
            continue;
        }
        rslist.push_back(rs);
        
        // extract CAN packet
        // this is what the Segway RMI_DEMO does for received packets.  It uses all 8
        // bits from byte 4 and the first 3 bits from byte 5.
        pkt[packnum].id = ((packet.bytes[4] << 3) | ((packet.bytes[5] >> 5) & 7)) & 0xfff;
        memcpy(pkt[packnum].msg,packet.pDATA,8);
        if (pkt[packnum].id == 0x400) {
            *got_packet_0x400=true;
        }
        //  printf("CAN packet extracted: %s\n", pkt[packnum].toString());
        // increment our pointer
        i+=18;
        // increment the packet number
        packnum++;

        
    }
    if (i<rxbufcount) {
        // shift the receive buffer
        memcpy(rxbuf,rxbuf+i,rxbufcount-i);
    }
    rxbufcount -= i;

    return packnum;
}

int
CANIOrmpusb::ReadFrame(CanPacket *pkt)
{
    static struct timeval *last=NULL;
    // initialize the first timeval if necessary
    if (!last) {
        last = new struct timeval;
        gettimeofday(last,NULL);
    }
    int packnum=0;
    
#ifdef STATS
    rs.bytes=0;
    struct timeval curr;
#endif

    
    DWORD rsize;
    ftStatus = FT_GetQueueStatus(ftHandle,&rsize);
    if (ftStatus != FT_OK) {
        printf("Unable to get Queue status\n");
        return -1;
    }
    DWORD bytes_read;
    static bool got_packet_0x400 = false; 

    // as long as there's more data to read, keep reading it.  the only
    // exception is that we don't want to read just a single byte, since it
    // seems to provoke the ftdi chip into producing garbage.
    while (rsize > 1) {
        if (rsize > (RX_BUFFER_SIZE - rxbufcount)) {
            rsize = RX_BUFFER_SIZE - rxbufcount;
        }
        ftStatus = FT_Read(ftHandle,rxbuf+rxbufcount,rsize,&bytes_read);
        if (ftStatus != FT_OK) {
            printf("Error reading from USB device\n");
            return -1;
        }
#ifdef STATS
        // get the time of the read
        gettimeofday(&curr,NULL);
        rs.time = (curr.tv_sec - last->tv_sec)*1000.0 + (curr.tv_usec - last->tv_usec) / 1000.0;
        rs.bytes+=bytes_read;
#endif
        rxbufcount += bytes_read;
        usbreadcount++;
        if (rxbufcount > RX_WARNING_SIZE) {
            // Player is falling behind in processing the messages, so throw out
            // the oldest bytes
            memcpy(rxbuf,rxbuf+rxbufcount-RX_KEEP_SIZE,RX_KEEP_SIZE);
            printf("Warning: purged %d old bytes from USB interface\n", rxbufcount-RX_KEEP_SIZE);
            purgecount+=rxbufcount-RX_KEEP_SIZE;
            rxbufcount = RX_KEEP_SIZE;
        }
        
        packnum += ExtractPacketsFromBuffer(pkt+packnum,MAX_EXTRACTED_PACKETS-packnum,&got_packet_0x400);
        if (packnum > 9) { // if we've queued up more than a full frame's worth, get rid of the old ones
            memcpy(pkt,pkt+packnum-9,9);
            packnum =9;
        }

        ftStatus = FT_GetQueueStatus(ftHandle,&rsize);
        if (ftStatus != FT_OK) {
            printf("Unable to get Queue status\n");
            return -1;
        }
    }
    if (got_packet_0x400) {
        writeslots++;
        got_packet_0x400 = false;
        SendPacket();
        gettimeofday(last,NULL);
    }
    return packnum;
}


/* Reads a packet from the USB bus, and extracts the CAN data.
 */
int
CANIOrmpusb::ReadPacket(CanPacket *pkt)
{
    // We'll read as much as we possibly can, since the read call is slow.
    // The extra bytes will be kept around until next time, and just processed
    // from memory.

    readcount++;
    DWORD bytes_read;
    
    static struct timeval *last=NULL;
    // initialize the first timeval if necessary
    if (!last) {
        last = new struct timeval;
        gettimeofday(last,NULL);
    }
    
    if (!ready) {
        return -1;
    }

    CANIOrmpusb::rmpUsbPacket packet;
#ifdef STATS
    static ReceiveStats rs;
#endif


    if (rxbufcount > RX_WARNING_SIZE) {
        // Player is falling behind in processing the messages, so throw out
        // the oldest bytes
        memcpy(rxbuf,rxbuf+rxbufcount-RX_KEEP_SIZE,RX_KEEP_SIZE);
        printf("Warning: purged %d old bytes from USB interface\n", rxbufcount-RX_KEEP_SIZE);
        purgecount+=rxbufcount-RX_KEEP_SIZE;
        rxbufcount = RX_KEEP_SIZE;
    }
        
    if (rxbufcount < 180) {
        // only read more from the device if we're running low
        DWORD rsize;
        ftStatus = FT_GetQueueStatus(ftHandle,&rsize);
        if (ftStatus != FT_OK) {
            printf("Unable to get Queue status\n");
            return -1;
        }
        
        // Limit what we will read to the space left in the buffer
        if (rsize > (RX_BUFFER_SIZE - rxbufcount)) {
            rsize = RX_BUFFER_SIZE - rxbufcount;
        }
        
        //        if (rsize < 18) {
        if (rsize < 2) {
            // not a full packet available
            // it seems that if we try to read something now, it
            // provokes the FTDI chip into producing garbage, so instead
            // we'll wait until later to get at least 1 full packet
            nomsgreadcount++;
            return 0;
        }
        ftStatus = FT_Read(ftHandle,rxbuf+rxbufcount,rsize,&bytes_read);

        usbreadcount++;
        if (ftStatus != FT_OK) {
            printf("Error reading from USB device\n");
            return -1;
        }
        rxbufcount += bytes_read;

#ifdef STATS
        struct timeval curr;
        // get the current time
        gettimeofday(&curr,NULL);
        rs.time = (curr.tv_sec - last->tv_sec)*1000.0 + (curr.tv_usec - last->tv_usec) / 1000.0;
        rs.bytes=bytes_read;
        last->tv_sec=curr.tv_sec; last->tv_usec=curr.tv_usec;
#endif 
        //        rcount++;
        //        if (rcount == 500) {
        //            rcount = 0;
        //            printf("queue: %d characters; buf %d, initcount=%d\n\n",rsize, rxbufcount,initcount);
        //        }
        

    }
    
    int i;
    // Now, search for the next valid packet
    for ( i = 0; i < ((int)rxbufcount-17); i++) {
        if ((rxbuf[i] != 0xF0) || (rxbuf[i+1] != 0x55) || (rxbuf[i+2] != 0xAA)) {
            continue;
        }
        
        // move bytes into 18-byte USB packet and check
        memcpy(packet.bytes,rxbuf+i,18);
        if(packet.computeChecksum() != packet.bytes[17]) {
            rs.id = 0 - ((packet.bytes[4] << 3) | ((packet.bytes[5] >> 5) & 7)) & 0xfff;
            rslist.push_back(rs);
            continue;
        }

        // extract CAN packet
        // this is what the Segway RMI_DEMO does for received packets.  It uses all 8
        // bits from byte 4 and the first 3 bits from byte 5.
        pkt->id = ((packet.bytes[4] << 3) | ((packet.bytes[5] >> 5) & 7)) & 0xfff;
        memcpy(pkt->msg,packet.pDATA,8);
        //  printf("CAN packet extracted: %s\n", pkt->toString());
        rs.id=pkt->id;
        rslist.push_back(rs);
        // shift the receive buffer
        memcpy(rxbuf,rxbuf+i+18,rxbufcount-i-18);
        rxbufcount -= (i+18);
        if (pkt->id == 1025) {
            if (rxbufcount ==0) {
                printf("write opportunity\n");
            } else {
                printf("%d bytes still in buffer\n",rxbufcount);
            }
        }
        

        // return the CAN bytes received
        return 10;
    }

    // we didn't find any valid messages
    // shift the receive buffer
    memcpy(rxbuf,rxbuf+i,rxbufcount-i);
    rxbufcount -= i;
    
    // nomsgreadcount++;
    return 0;

}


CANIOrmpusb::rmpUsbPacket::rmpUsbPacket() {
    InitPacket();
}

CANIOrmpusb::rmpUsbPacket::rmpUsbPacket(CanPacket &cpkt) {
    InitPacket();
    bytes[6] = (cpkt.id >> 8) & 0xff;
    bytes[7] = cpkt.id & 0xff;
    memcpy(pDATA,cpkt.msg,8);
    addChecksum();
}

void CANIOrmpusb::rmpUsbPacket::InitPacket() {
    bytes[0] = 0xF0;
    bytes[1] = 0x55;
    bytes[2] = 0;
    bytes[3] = 0;
    bytes[4] = 0;
    bytes[5] = 0;
    bytes[6] = 0;
    bytes[7] = 0;
    bytes[8] = 0;
    bytes[9] = 0;
    bytes[10] = 0;
    bytes[11] = 0;
    bytes[12] = 0;
    bytes[13] = 0;
    bytes[14] = 0;
    bytes[15] = 0;
    bytes[16] = 0;
    bytes[17] = 0;
    pID = bytes+6;
    pDATA = bytes+9;
}

CANIOrmpusb::rmpUsbPacket::~rmpUsbPacket() {
}

unsigned char CANIOrmpusb::rmpUsbPacket::computeChecksum() {
    // code copied from Segway RMP Interface Guide
    unsigned short checksum;
    unsigned short checksum_high;
    checksum = 0;
    for (int i = 0; i < 17; i++) {
        checksum += (short)bytes[i];
    }
    checksum_high = checksum >> 8;
    checksum &= 0xff;
    checksum += checksum_high;
    checksum_high = checksum >> 8;
    checksum &= 0xff;
    checksum += checksum_high;
    checksum = (~checksum +1) & 0xff;
    return (unsigned char) checksum;
}

void CANIOrmpusb::rmpUsbPacket::addChecksum() {
    bytes[17] = computeChecksum();
}

char * CANIOrmpusb::rmpUsbPacket::toString() {
    static char buf[256];
    buf[0] = 0;
    for (int i = 0; i < 18; i++) {
        sprintf(buf+strlen(buf),"%d:%02X ",i,bytes[i]);
    }
    return buf;
}

