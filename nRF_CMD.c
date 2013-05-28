/***************************************************************
 *
 * OpenBeacon.org - nRF24L01 hardware support / macro functions
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

#include <htc.h>
#include "config.h"
//#include "timer.h"
#include "nRF_HW.h"
#include "nRF_CMD.h"

#include <Arduino.h>

//#assert NRF_MAC_SIZE==5

#define NRF_CONFIG_BYTE (NRF_CONFIG_EN_CRC)

// first byte payload size+1, second byte register, 3..n-th byte payload
const unsigned char g_MacroInitialization[] = {
	0x01, OP_NOP,
	0x02, NRF_REG_CONFIG | WRITE_REG, 0x00,										// stop nRF
	0x02, NRF_REG_EN_AA | WRITE_REG, 0x00,										// disable ShockBurst(tm)
	0x02, NRF_REG_EN_RXADDR | WRITE_REG, 0x01,									// enable RX pipe address 0
	0x02, NRF_REG_SETUP_AW | WRITE_REG, NRF_MAC_SIZE - 2,						// setup MAC address width to NRF_MAC_SIZE
	0x02, NRF_REG_RF_CH | WRITE_REG, CONFIG_DEFAULT_CHANNEL,					// set channel to 2480MHz
	0x02, NRF_REG_RF_SETUP | WRITE_REG, NRF_RFOPTIONS,							// update RF options
	0x02, NRF_REG_STATUS | WRITE_REG, 0x78,										// reset status register
	0x06, NRF_REG_RX_ADDR_P0 | WRITE_REG, 'O', 'C', 'A', 'E', 'B',				// set RX_ADDR_P0 to "BEACO"
	0x06, NRF_REG_TX_ADDR | WRITE_REG, 0x01, 0x02, 0x03, 0x02, 0x01,			// set TX_ADDR
	0x02, NRF_REG_RX_PW_P0 | WRITE_REG, 16,										// set payload width of pipe 0 to sizeof(TRfBroadcast)
	0x00																		// termination
};

// first byte payload size+1, second byte register, 3..n-th byte payload
const unsigned char g_MacroStart[] = {
	0x02, NRF_REG_CONFIG | WRITE_REG, NRF_CONFIG_BYTE | NRF_CONFIG_PWR_UP,
	0x02, NRF_REG_STATUS | WRITE_REG, 0x70,										// reset status
	0x00
};

// first byte payload size+1, second byte register, 3..n-th byte payload
const unsigned char g_MacroStop[] = {
	0x02, NRF_REG_CONFIG | WRITE_REG, NRF_CONFIG_BYTE,
	0x02, NRF_REG_STATUS | WRITE_REG, 0x70,										// reset status
	0x00
};

unsigned char
nRFCMD_XcieveByte (unsigned char byte)
{
	unsigned char idx;

	for (idx = 0; idx < 8; idx++)
	{
		//CONFIG_PIN_MOSI = (byte & 0x80) ? 1 : 0;
                digitalWrite(CONFIG_PIN_MOSI, (byte & 0x80) ? HIGH : LOW);
		byte <<= 1;
		//CONFIG_PIN_SCK = 1;
                digitalWrite(CONFIG_PIN_SCK, HIGH);
                byte |= CONFIG_PIN_MISO;
		//byte |= digitalRead(CONFIG_PIN_MISO); // TODO: powinno dzialac, bo #define HIGH 0x1
		digitalWrite(CONFIG_PIN_SCK, LOW);
	}
	//CONFIG_PIN_MOSI = 0;
        digitalWrite(CONFIG_PIN_MOSI, LOW);
        
	return byte;
}

unsigned char
nRFCMD_RegWrite (unsigned char reg, const unsigned char *buf,
				 unsigned char count)
{
	unsigned char res;

	//CONFIG_PIN_CSN = 0;
        digitalWrite(CONFIG_PIN_CSN, LOW);
	res = nRFCMD_XcieveByte (reg);
	if (buf)
		while (count--)
			nRFCMD_XcieveByte (*buf++);
	//CONFIG_PIN_CSN = 1;
        digitalWrite(CONFIG_PIN_CSN, HIGH);

	return res;
}

unsigned char
nRFCMD_RegRead (unsigned char reg, unsigned char *buf, unsigned char count)
{
	unsigned char res;

	//CONFIG_PIN_CSN = 0;
        digitalWrite(CONFIG_PIN_CSN, LOW);
	res = nRFCMD_XcieveByte (reg);
	if (buf)
		while (count--)
			*buf++ = nRFCMD_XcieveByte (0);
	//CONFIG_PIN_CSN = 1;
        digitalWrite(CONFIG_PIN_CSN, HIGH);

	return res;
}

void
nRFCMD_Macro (const unsigned char *macro)
{
	unsigned char size;

	while ((size = *macro++) != 0)
	{
		nRFCMD_RegWrite (*macro, macro + 1, size - 1);
		macro += size;
	}
}

void
nRFCMD_Stop (void)
{
	nRFCMD_Macro (g_MacroStop);
}

void
nRFCMD_Execute (void)
{
	nRFCMD_Macro (g_MacroStart);
	//sleep_2ms ();
	//CONFIG_PIN_CE = 1;
   	delay(2);
        digitalWrite(CONFIG_PIN_CE, HIGH);
        //sleep_2ms ();
     	//CONFIG_PIN_CE = 0;
   	delay(2);
        digitalWrite(CONFIG_PIN_CE, HIGH);
	nRFCMD_Stop ();
}

unsigned char
nRFCMD_RegExec (unsigned char reg)
{
	unsigned char res;

	//CONFIG_PIN_CSN = 0;
        digitalWrite(CONFIG_PIN_CSN, LOW);
	res = nRFCMD_XcieveByte (reg);
	//CONFIG_PIN_CSN = 1;
        digitalWrite(CONFIG_PIN_CSN, HIGH);

	return res;
}

unsigned char
nRFCMD_RegReadWrite (unsigned char reg, unsigned char value)
{
	unsigned char res;

	//CONFIG_PIN_CSN = 0;
        digitalWrite(CONFIG_PIN_CSN, LOW);
	res = nRFCMD_XcieveByte (reg);
	nRFCMD_XcieveByte (value);
	//CONFIG_PIN_CSN = 1;
        digitalWrite(CONFIG_PIN_CSN, HIGH);

	return res;
}

void
nRFCMD_Init (void)
{
	/* update CPU default pin settings */
	/*CONFIG_PIN_CSN = 1;
	CONFIG_PIN_CE = 0;
	CONFIG_PIN_MOSI = 0;
	CONFIG_PIN_SCK = 0;*/
        digitalWrite(CONFIG_PIN_CSN, HIGH);
	digitalWrite(CONFIG_PIN_CE, LOW);
	digitalWrite(CONFIG_PIN_MOSI, LOW);
	digitalWrite(CONFIG_PIN_SCK, LOW);

	//sleep_2ms ();
        delay(2);

	/* send initialization macro to RF chip */
	nRFCMD_Macro (g_MacroInitialization);
}

