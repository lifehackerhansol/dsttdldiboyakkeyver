/*
	iointerface.c

 Copyright (c) 2006 Michael "Chishm" Chisholm
 Copyright (c) 2006 SuperCard

 Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation and/or
     other materials provided with the distribution.
  3. The name of the author may not be used to endorse or promote products derived
     from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


// When compiling for NDS, make sure NDS is defined
#ifndef NDS
 #if defined ARM9 || defined ARM7
  #define NDS
 #endif
#endif

#ifdef NDS
 #include <nds/ndstypes.h>
#else
 #include "gba_types.h"
#endif

#define BYTES_PER_READ 512

#ifndef NULL
 #define NULL 0
#endif


// Card bus
#define REG_AUXSPICNTH      (*(vu8*)0x040001A1)
#define REG_ROMCTRL       (*(vu32*)0x040001A4)
#define REG_CARD_COMMAND   ((vu8*)0x040001A8)

#define REG_CARD_DATA_RD   (*(vu32*)0x04100010)

#define CARD_CR1_ENABLE  0x80  // in byte 1, i.e. 0x8000
#define CARD_CR1_IRQ     0x40  // in byte 1, i.e. 0x4000

// 3 bits in b10..b8 indicate something
// read bits
#define CARD_BUSY       (1<<31)  // when reading, still expecting incomming data?
#define CARD_DATA_READY (1<<23)  // when reading, REG_CARD_DATA_RD or CARD_DATA has another word of data and is good to go

#define	sdmode_sdhc		(*(vu32*)0x02FFFC24)	//FIXME: DLDI could not alloc global memory ??

void sd_readpage(unsigned int addr,unsigned int dst)
{
	REG_CARD_COMMAND[0] = 0x53;
	REG_CARD_COMMAND[1] = (u8)(addr >> 24);
	REG_CARD_COMMAND[2] = (u8)(addr >> 16);
	REG_CARD_COMMAND[3] = (u8)(addr >> 8);
	REG_CARD_COMMAND[4] = (u8)(addr);
	REG_ROMCTRL = 0xA7180000;
	while (!(REG_ROMCTRL & CARD_DATA_READY)) ;
	REG_CARD_DATA_RD;

    do{
        REG_CARD_COMMAND[0] = 0x80;
        REG_ROMCTRL = 0xA7180000;
        while (!(REG_ROMCTRL & CARD_DATA_READY));
    }while(REG_CARD_DATA_RD);

    REG_CARD_COMMAND[0] = 0x81;
    REG_ROMCTRL=0xA1180000;
    do {
        if (REG_ROMCTRL & CARD_DATA_READY) {
            *(vu32*)dst=REG_CARD_DATA_RD;
            dst+=4;
        }
    } while (REG_ROMCTRL & CARD_BUSY);
}

void sd_setmode(u8 mode,u8 cmd,u32 arg)
{
	REG_CARD_COMMAND[0] = 0x51;
	REG_CARD_COMMAND[1] = (u8)(arg >> 24);
	REG_CARD_COMMAND[2] = (u8)(arg >> 16);
	REG_CARD_COMMAND[3] = (u8)(arg >> 8);
	REG_CARD_COMMAND[4] = (u8)(arg);
	REG_CARD_COMMAND[5] = cmd;
	REG_CARD_COMMAND[6] = mode;
	REG_CARD_COMMAND[7] = 0;
	REG_ROMCTRL = 0xA7180000;
	while (!(REG_ROMCTRL & CARD_DATA_READY)) ;
	REG_CARD_DATA_RD;
}
u32 sd_isbusy()
{
	REG_CARD_COMMAND[0] = 0x50;
	REG_ROMCTRL = 0xA7180000;
	while (!(REG_ROMCTRL & CARD_DATA_READY)) ;
	return REG_CARD_DATA_RD;
}
u32 sd_getresp()
{
	REG_CARD_COMMAND[0] = 0x52;
	REG_ROMCTRL = 0xA7180000;
	while (!(REG_ROMCTRL & CARD_DATA_READY)) ;
	return REG_CARD_DATA_RD;
}

u32 sd_cmd_r1(u8 cmd,u32 arg)//4
{
	sd_setmode(1,cmd,arg);
	while(sd_isbusy());
	return sd_getresp();
}

void sd_readmultipage(unsigned int addr,unsigned int* dst, unsigned int len)
{
    u32 data;

    if(len != 0) {
        REG_CARD_COMMAND[0] = 0x54;
        REG_CARD_COMMAND[1] = (u8)(addr >> 24);
        REG_CARD_COMMAND[2] = (u8)(addr >> 16);
        REG_CARD_COMMAND[3] = (u8)(addr >> 8);
        REG_CARD_COMMAND[4] = (u8)(addr);
        REG_ROMCTRL = 0xA7180000;
        while (!(REG_ROMCTRL & CARD_DATA_READY)) ;
        REG_CARD_DATA_RD;

        while(1) {
            do{
                REG_CARD_COMMAND[0] = 0x80;
                REG_ROMCTRL = 0xA7180000;
                while (!(REG_ROMCTRL & CARD_DATA_READY));
            }while(REG_CARD_DATA_RD);

            REG_CARD_COMMAND[0] = 0x81;
            REG_ROMCTRL=0xA1180000;
            do {
                if (REG_ROMCTRL & CARD_DATA_READY) {
                    data = REG_CARD_DATA_RD;
                    if((u32)dst & 3) {
                        ((uint8*)dst)[0] = data & 0xff;
                        ((uint8*)dst)[1] = (data >> 8) & 0xff;
                        ((uint8*)dst)[2] = (data >> 16) & 0xff;
                        ((uint8*)dst)[3] = (data >> 24) & 0xff;
                    }
                    else {
                        *dst=data;
                    }
                    dst++;
                }
            }while (REG_ROMCTRL & CARD_BUSY);
            len--;
            if (len == 0) break;
            sd_setmode(7,0,0);
        };
        sd_cmd_r1(12, 0);
    }
}

void sd_writepage(unsigned int addr,unsigned int dst)
{
    sd_cmd_r1(24,addr);
    REG_CARD_COMMAND[0] = 0x82;
    REG_ROMCTRL = 0xE1180000;
    do {
        if (REG_ROMCTRL & CARD_DATA_READY) {
                REG_CARD_DATA_RD=*(vu32*)dst;
                dst+=4;
            }
    } while (REG_ROMCTRL & CARD_BUSY);
	while(sd_isbusy());
    REG_CARD_COMMAND[0] = 0x56;
    REG_ROMCTRL = 0xA7180000;
    while (!(REG_ROMCTRL & CARD_DATA_READY));
    REG_CARD_DATA_RD;
	while(sd_isbusy());

}

void sd_writedata(unsigned int addr,unsigned int* dst, unsigned int len)
{
    bool stop_writing = false;
    u32 data;

    if (len != 0) {
        if(len == 1) {
            sd_cmd_r1(24,addr);
        } else {
            sd_cmd_r1(25, addr);
        }
        do {
            REG_CARD_COMMAND[0] = 0x82;
            REG_ROMCTRL = 0xE1180000;
            do {
                if (REG_ROMCTRL & CARD_DATA_READY) {
                    if((u32)dst & 3) {
                        data = ((uint8*)dst)[0] | (((uint8*)dst)[1] << 8) | (((uint8*)dst)[2] << 16) | (((uint8*)dst)[3] << 24);
                    } else {
                        data = *dst;
                    }
                    REG_CARD_DATA_RD=data;
                    dst++;
                }
            } while (REG_ROMCTRL & CARD_BUSY);
            while(sd_isbusy());
            len--;
            if(len == 0) {
                stop_writing = true;
            }
            if(stop_writing) {
                sd_cmd_r1(12, 0);
            }
            REG_CARD_COMMAND[0] = 0x56;
            REG_ROMCTRL = 0xA7180000;
            while (!(REG_ROMCTRL & CARD_DATA_READY));
            REG_CARD_DATA_RD;
            while(sd_isbusy());
        } while(len);
    }
}

/*-----------------------------------------------------------------
startUp
Initialize the interface, geting it into an idle, ready state
returns true if successful, otherwise returns false
-----------------------------------------------------------------*/
bool startup(void) {
	return true;
}

/*-----------------------------------------------------------------
isInserted
Is a card inserted?
return true if a card is inserted and usable
-----------------------------------------------------------------*/
bool isInserted (void) {
	return true;
}

/*-----------------------------------------------------------------
readSectors
Read "numSectors" 512-byte sized sectors from the card into "buffer", 
starting at "sector".
return true if it was successful, false if it failed for any reason
-----------------------------------------------------------------*/
bool readSectors (u32 sector, u32 numSectors, void* buffer) {
	REG_AUXSPICNTH = CARD_CR1_ENABLE | CARD_CR1_IRQ;
	if(sdmode_sdhc)
    {
        sd_readmultipage(sector,buffer, numSectors);
    }
	else
    {
        sd_readmultipage(sector << 9,buffer, numSectors);
    }
	return true;
}

/*-----------------------------------------------------------------
writeSectors
Write "numSectors" 512-byte sized sectors from "buffer" to the card, 
starting at "sector".
return true if it was successful, false if it failed for any reason
-----------------------------------------------------------------*/
bool writeSectors (u32 sector, u32 numSectors, void* buffer) {
	REG_AUXSPICNTH = CARD_CR1_ENABLE | CARD_CR1_IRQ;
	if(sdmode_sdhc)
    {
        sd_writedata(sector, buffer, numSectors);
    }
	else
    {
        sd_writedata(sector << 9, buffer, numSectors);
    }
	return true;
}

/*-----------------------------------------------------------------
clearStatus
Reset the card, clearing any status errors
return  true if the card is idle and ready
-----------------------------------------------------------------*/
bool clearStatus (void) {
	return true;
}

/*-----------------------------------------------------------------
shutdown
shutdown the card, performing any needed cleanup operations
-----------------------------------------------------------------*/
bool shutdown(void) {
	return true;
}
