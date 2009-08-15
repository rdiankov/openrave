#include <unistd.h>
// #include "rmp_frame.h"
#include "canio_rmpusb.h"
#include <list>

// things to try:
//   restart segway and see if the a/b bus delay is the same.
//   count how many times write window is open.



void MakeVelocityCommand(CanPacket* pkt, 
                               int32_t trans, 
                               int32_t rot)
{
    pkt->id = 0x0413; // RMP_CAN_ID_COMMAND;
    pkt->PutSlot(0, (uint16_t)trans);
    pkt->PutSlot(1, (uint16_t)rot);
    pkt->PutSlot(2, (uint16_t)0); // RMP_CAN_CMD_NONE
    pkt->PutSlot(3, (uint16_t)0);
}

#if 0
struct ReceiveStats {
    double time;
    unsigned int bytes;
};

std::list<ReceiveStats> rslist;

int direct_read(CANIOrmpusb &canio) {
    static struct timeval *last=NULL;
    // initialize the first timeval if necessary
    if (!last) {
        last = new struct timeval;
        gettimeofday(last,NULL);
    }
    static unsigned char rxbuf[10000];
    DWORD rsize;
    DWORD bytes_read;
	FT_STATUS	ftStatus;
    ftStatus = FT_GetQueueStatus(canio.ftHandle,&rsize);
    if (ftStatus != FT_OK) {
        printf("Unable to get Queue status\n");
        return 0;
    }
    if (rsize>10000) {
        rsize=10000;
    }
    if (rsize>0) {
        ftStatus = FT_Read(canio.ftHandle,rxbuf,rsize,&bytes_read);
    } else {
        bytes_read=0;
    }
    struct timeval curr;
    // get the current time
    gettimeofday(&curr,NULL);
    ReceiveStats rs;
    rs.time = (curr.tv_sec - last->tv_sec)*1000.0 + (curr.tv_usec - last->tv_usec) / 1000.0;
    rs.bytes=bytes_read;
    rslist.push_back(rs);
    last->tv_sec=curr.tv_sec; last->tv_usec=curr.tv_usec;
    return bytes_read/18;
}
#endif

int main() {
    int ret;
    int writecount = 0;
    CanPacket pkt, pkt2;
    CANIOrmpusb canio;
    if ((ret=canio.Init())) {
        printf("Init failed with error %d\n",ret);
        return 1;
    }

    MakeVelocityCommand(&pkt,0,0);
    canio.WritePacket(pkt);

    printf("reading 10000 packets\n");
    int state=0;
#if 0
    for(int i =0; i < 1000; ) {
        i+=direct_read(canio);
        //usleep(1000);
    }
    FILE *fout;
    fout = fopen("packet_stats","w");
    double totaltime=0;
    unsigned int totalbytes=0;
    if (fout) {
        std::list<ReceiveStats>::iterator it = rslist.begin();
        while (it != rslist.end()) {
            fprintf(fout,"%d bytes after %3.3f msecs\n",it->bytes,it->time);
            totaltime+=it->time;
            totalbytes+=it->bytes;
            ++it;
        }
        fprintf(fout,"%d bytes in %3.3f seconds = %3.3f packets/second\n",
                totalbytes,totaltime,totalbytes/18/totaltime*1000.0);
        fclose(fout);
    }
    printf("read packet statistics written to file packet_stats\n");

    return 0;
#endif
    int delaycount;
    for(int i =0; i < 10000; ) {
    
        if ((i >= 3000) && (state == 0)) {
            // start moving
            printf("===========START===============================\n");
            MakeVelocityCommand(&pkt,75,0);
            canio.WritePacket(pkt);
            delaycount=0;
            state=1;
        } else if ((i >= 4000) && (state == 2)) {
            printf("===========STOP===============================\n");
            MakeVelocityCommand(&pkt,0,0);
            canio.WritePacket(pkt);
            delaycount=0;
            state=3;
        }
        //        int ret = canio.ReadPacket(&pkt2);
        CanPacket pkt2[CANIOrmpusb::MAX_EXTRACTED_PACKETS];
        ret=canio.ReadFrame(pkt2);
        i+= ret;
        for (int j=0; j<ret; j++) {
            if (((state % 2) == 1) && (pkt2[j].id == 0x407)) {
                delaycount++;
                //                printf("%s\n",pkt2.toString());
                int cmdvel = pkt2[j].GetSlot(0);
                if ((state == 1) && (cmdvel != 0)) {
                    printf("read %d frames before start confirmation\n",delaycount);
                    state = 2;
                } else if ((state == 3) && (cmdvel == 0)) {
                    printf("read %d frames before stop confirmation\n",delaycount);
                    state = 4;
                }
            }
            //            if ((i % 10) == 0) {
            //                if (pkt2.id == 0x407) { // command velocity
            //                    printf("%s\n",pkt2.toString());
            //                }
            //            }
        }
        writecount++;
        if (writecount == 100) {
            writecount = 0;
        }
    }

    printf("done\n");


    printf("==========================================\n");

    return 0;
}



