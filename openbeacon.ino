/***************************************************************
 *
 * OpenBeacon.org - OnAir protocol specification and definition
 *
 * Copyright 2006 Milosch Meriac <meriac@openbeacon.de>
 *
/***************************************************************

/*
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; version 2.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*/

#include <Arduino.h>
#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

#include "openbeacon.h"

volatile const u_int32_t oid = 0xFFFFFFFF, seed = 0xFFFFFFFF;

static const long xxtea_key[4] =
{ 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF };
static u_int32_t seq = 0;
static u_int16_t code_block;

static u_int16_t crc;
TBeaconEnvelope env;

static unsigned long
htonl (unsigned long src)
{
    unsigned long res;

    ((unsigned char *) &res)[0] = ((unsigned char *) &src)[3];
    ((unsigned char *) &res)[1] = ((unsigned char *) &src)[2];
    ((unsigned char *) &res)[2] = ((unsigned char *) &src)[1];
    ((unsigned char *) &res)[3] = ((unsigned char *) &src)[0];

    return res;
}

static unsigned short
htons (unsigned short src)
{
    unsigned short res;

    ((unsigned char *) &res)[0] = ((unsigned char *) &src)[1];
    ((unsigned char *) &res)[1] = ((unsigned char *) &src)[0];

    return res;
}

#define SHUFFLE(a,b) 	tmp=env.datab[a];\
			env.datab[a]=env.datab[b];\
			env.datab[b]=tmp;

static void
shuffle_tx_byteorder (void)
{
    unsigned char tmp;

    SHUFFLE (0 + 0, 3 + 0);
    SHUFFLE (1 + 0, 2 + 0);
    SHUFFLE (0 + 4, 3 + 4);
    SHUFFLE (1 + 4, 2 + 4);
    SHUFFLE (0 + 8, 3 + 8);
    SHUFFLE (1 + 8, 2 + 8);
    SHUFFLE (0 + 12, 3 + 12);
    SHUFFLE (1 + 12, 2 + 12);
}

static unsigned short
crc16 (const unsigned char *buffer, unsigned char size)
{
    unsigned short crc = 0xFFFF;
    if (buffer)
    {
        while (size--)
        {
            crc = (crc >> 8) | (crc << 8);
            crc ^= *buffer++;
            crc ^= ((unsigned char) crc) >> 4;
            crc ^= crc << 12;
            crc ^= (crc & 0xFF) << 5;
        }
    }
    return crc;
}

static void
store_incremented_codeblock (void)
{
    if (code_block < 0xFFFF)
    {
        code_block++;
        //EEPROM.write (0, (u_int8_t) (code_block));
        //delay (10);
        //EEPROM.write (1, (u_int8_t) (code_block >> 8));
        //delay (10);
    }
}

static void
xxtea_encode (void)
{
    u_int32_t z, y, sum;
    u_int8_t p, q, e;

    /* prepare first XXTEA round */
    z = env.data[TEA_ENCRYPTION_BLOCK_COUNT - 1];
    sum = 0;

    /* setup rounds counter */
    q = (6 + 52 / TEA_ENCRYPTION_BLOCK_COUNT);

    /* start encryption */
    while (q--)
    {
        sum += 0x9E3779B9UL;
        e = (sum >> 2) & 3;

        for (p = 0; p < TEA_ENCRYPTION_BLOCK_COUNT; p++)
        {
            y = env.data[(p + 1) & (TEA_ENCRYPTION_BLOCK_COUNT - 1)];
            z = env.data[p] + ((z >> 5 ^ y << 2) +
                               (y >> 3 ^ z << 4) ^ (sum ^ y) +
                               (xxtea_key[p & 3 ^ e] ^ z));
            env.data[p] = z;
        }
    }
}

// TODO: move it to another file

#define CONFIG_DEFAULT_CHANNEL 81
#define CONFIG_MAX_POWER_LEVELS 4

#define NRF_RFOPTIONS 0x09

// end of TODO

void
nRFCMD_Init ()
{
    Serial.begin(9600);
    Mirf.spi = &MirfHardwareSpi;
    Mirf.init ();
    Mirf.configRegister (CONFIG, 0x00); // stop nRF
    Mirf.configRegister (EN_AA, 0x00); // disable ShockBurst(tm)
    //Mirf.configRegister (EN_RXADDR, 0x01); // enable RX pipe address 0
    //Mirf.configRegister (SETUP_AW, NRF_MAC_SIZE - 2); // setup MAC address width to NRF_MAC_SIZE
    //Mirf.configRegister (RF_CH, CONFIG_DEFAULT_CHANNEL); // set channel to 2480MHz
    Mirf.channel = CONFIG_DEFAULT_CHANNEL;
    Mirf.configRegister (RF_SETUP, NRF_RFOPTIONS); // update RF options
    Mirf.configRegister (STATUS, 0x87); // reset status register
    Mirf.setRADDR ((uint8_t*)"BEACO"); // set RX_ADDR_P0 to "BEACO"
//	Mirf.setTADDR ((uint8_t*)"\x1\x2\x3\x2\x1"); // set TX_ADDR
    Mirf.setTADDR ((uint8_t*)"serv1"); // set TX_ADDR
    // set payload width of pipe 0 to sizeof(TRfBroadcast)
    Mirf.payload = 16;
    Mirf.config ();
}

void
setup ()
{
    nRFCMD_Init ();

    // increment code block after power cycle
    //((unsigned char *) &code_block)[0] = EEPROM_READ (0);
    //((unsigned char *) &code_block)[1] = EEPROM_READ (1);
    code_block = 0;
    store_incremented_codeblock ();

    seq = code_block ^ oid ^ seed;
    srand (crc16 ((unsigned char *) &seq, sizeof (seq)));

    // increment code blocks to make sure that seq is higher or equal after battery
    // change
    seq = ((u_int32_t) (code_block - 1)) << 16;
}

u_int8_t i = 0;

void
loop ()
{
    // u_int8_t i;

    if (code_block != 0xFFFF)
    {
        Serial.println(i);
        //g_MacroBeacon.rf_setup = NRF_RFOPTIONS | ((i & 3) << 1);
        env.pkt.hdr.proto = RFBPROTO_BEACONTRACKER;
        env.pkt.hdr.flags =
            //CONFIG_PIN_SENSOR ? 0 : RFBFLAGS_SENSOR;
            0;
        env.pkt.hdr.oid = htons ((u_int16_t) oid);
        env.pkt.oid_last_seen = 0;
        env.pkt.powerup_count =
            htons ((u_int16_t) code_block);
        env.pkt.strength = i;
        env.pkt.seq = htonl (seq);
        env.pkt.reserved = 0;
        crc = crc16 (env.datab,
                     sizeof (env.pkt) -
                     sizeof (env.pkt.crc));
        env.pkt.crc = htons (crc);

        // update code_block so on next power up
        // the seq will be higher or equal
        crc = seq >> 16;
        /*    if (crc == 0xFFFF)
              break;*/
        if (crc == code_block)
            store_incremented_codeblock ();

        // encrypt my data
        shuffle_tx_byteorder ();
        xxtea_encode ();
        shuffle_tx_byteorder ();

        // change tx power
        Mirf.configRegister(RF_SETUP, NRF_RFOPTIONS | ((i & 3) << 1));

        // send it away
        Mirf.send((uint8_t*)&env);

        while(Mirf.isSending()) {
        }

        if (++i >= CONFIG_MAX_POWER_LEVELS)
        {
            i = 0;
            seq++;
        }
    }
}
