//*****************************************************************************
//
//! \file DS18B20.c
//! \brief Driver for digital thermometer DS18B20.
//! \version V2.1.1.0
//! \date 10/18/2011
//! \author CooCox
//! \copy
//!
//! Copyright (c)  2011, CooCox 
//! All rights reserved.
//! 
//! Redistribution and use in source and binary forms, with or without 
//! modification, are permitted provided that the following conditions 
//! are met: 
//! 
//!     * Redistributions of source code must retain the above copyright 
//! notice, this list of conditions and the following disclaimer. 
//!     * Redistributions in binary form must reproduce the above copyright
//! notice, this list of conditions and the following disclaimer in the
//! documentation and/or other materials provided with the distribution. 
//!     * Neither the name of the <ORGANIZATION> nor the names of its 
//! contributors may be used to endorse or promote products derived 
//! from this software without specific prior written permission. 
//! 
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
//! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
//! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
//! CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
//! SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//! INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
//! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
//! ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
//! THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//#include "xhw_types.h"
//#include "xhw_memmap.h"
//#include "xhw_ints.h"
//#include "xdebug.h"
//#include "xsysctl.h"
//#include "xgpio.h"
#include "hw_DS18B20.h"
#include "DS18B20.h"

static unsigned long ulHclk;
static GPIO_InitTypeDef GPIO_Def;

#if (DS18B20_SEARCH_ROM_EN > 0)
//
// global search state
//
static unsigned char ucROM[8];
static int LastDiscrepancy;
static int LastFamilyDiscrepancy;
static int LastDeviceFlag;
static unsigned char ucCrc8;
#endif

void __attribute__((used, naked))
SysCtlDelay(unsigned long ulCount);

//*****************************************************************************
//
//! \internal
//! \brief Delay some time for n us.
//!
//! \param ulNus determines delay time.
//!
//! The parameter of ulNus can be: all values with ulNus > 0
//!
//! \return None.
//
//*****************************************************************************
static void DS18B20DelayNus(unsigned long ulNus) {
	SysCtlDelay(ulHclk * ulNus);
}

//*****************************************************************************
//
//! \brief Initializes the DS18B20 device.
//!
//! \param psDev is a pointer which contains a DS18B20 device information.
//!
//! \return None.
//
//*****************************************************************************
void DS18B20Init(tDS18B20Dev *psDev) {
	unsigned char i = 1;
	ulHclk = SystemCoreClock / 1000000 / 3;
	//
	// Enable the GPIOx port which is connected with DS18B20
	//
	GPIO_Def.GPIO_Pin = psDev->ulPin;
	GPIO_Def.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Def.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Def.GPIO_OType = GPIO_OType_PP;
	GPIO_Def.GPIO_PuPd = GPIO_PuPd_UP;

	//Init Port
	GPIO_Init(psDev->ulPort, &GPIO_Def);

	//Start clock to the selected port
	RCC_AHB1PeriphClockCmd(psDev->uBus, ENABLE);

	//
	// check
	//
	while (i) {
		//
		// DS18B20 dq pin be set as output
		//
		GPIO_Def.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_Init(psDev->ulPort, &GPIO_Def);

		//
		// DS18B20 dq_pin be set to high
		//
		GPIO_SetBits(psDev->ulPort, psDev->ulPin);

		DS18B20DelayNus(5);

		//
		// DS18B20 dq_pin be set to low
		//
		GPIO_ResetBits(psDev->ulPort, psDev->ulPin);

		DS18B20DelayNus(600);

		//
		// DS18B20 dq_pin be set to high
		//
		GPIO_SetBits(psDev->ulPort, psDev->ulPin);

		DS18B20DelayNus(45);

		//
		// DS18B20 dq pin be set as input
		//
		GPIO_Def.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(psDev->ulPort, &GPIO_Def);

		i = GPIO_ReadInputDataBit(psDev->ulPort, psDev->ulPin);
	}

	DS18B20DelayNus(500);

	//
	// DS18B20 dq pin be set as output
	//
	GPIO_Def.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(psDev->ulPort, &GPIO_Def);

#if (DS18B20_SEARCH_ROM_EN > 0)
	//
	// reset the search state
	//
	LastDiscrepancy = 0;
	LastDeviceFlag = 0;
	LastFamilyDiscrepancy = 0;
#endif
}

//*****************************************************************************
//
//! \brief Reset the DS18B20 device.
//!
//! \param psDev is a pointer which contains a DS18B20 device information.
//!
//! \return None.
//
//*****************************************************************************
uint8_t DS18B20Reset(tDS18B20Dev *psDev) {
	unsigned long i = 1;

	//Declare pin
	// DS18B20 dq pin be set as output
	GPIO_Def.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(psDev->ulPort, &GPIO_Def);

	//
	// DS18B20 dq_pin be set to high
	//
	GPIO_SetBits(psDev->ulPort, psDev->ulPin);
	//xGPIOPinWrite(psDev->ulPort, psDev->ulPin, 1);

	DS18B20DelayNus(5);

	//
	// DS18B20 dq_pin be set to low
	//
	GPIO_ResetBits(psDev->ulPort, psDev->ulPin);
	//xGPIOPinWrite(psDev->ulPort, psDev->ulPin, 0);

	DS18B20DelayNus(600);

	//
	// DS18B20 dq_pin be set to high
	//
	GPIO_SetBits(psDev->ulPort, psDev->ulPin);
	//xGPIOPinWrite(psDev->ulPort, psDev->ulPin, 1);

	DS18B20DelayNus(100);

	//
	// DS18B20 dq pin be set as input
	//
	GPIO_Def.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(psDev->ulPort, &GPIO_Def);
	//xGPIODirModeSet(psDev->ulPort, psDev->ulPin, xGPIO_DIR_MODE_IN);

	i = GPIO_ReadInputDataBit(psDev->ulPort, psDev->ulPin);
	//i = xGPIOPinRead(psDev->ulPort, psDev->ulPin);

	//
	// DS18B20 dq pin be set as output
	//
	GPIO_Def.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(psDev->ulPort, &GPIO_Def);
	//xGPIODirModeSet(psDev->ulPort, psDev->ulPin, xGPIO_DIR_MODE_OUT);

	DS18B20DelayNus(500);

	if (i) {
		return 0;
	} else {
		return 1;
	}
}

//*****************************************************************************
//
//! \brief Read a bit from the DS18B20.
//!
//! \param psDev is a pointer which contains a DS18B20 device information.
//!
//! \return 0 or 1.
//
//*****************************************************************************
unsigned char DS18B20BitRead(tDS18B20Dev *psDev) {
	unsigned char ucData = 0;

	//
	// DS18B20 dq pin be set as output
	//
	GPIO_Def.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(psDev->ulPort, &GPIO_Def);

	//
	// DS18B20 dq_pin be set to high
	//
	GPIO_SetBits(psDev->ulPort, psDev->ulPin);

	DS18B20DelayNus(2);

	//
	// DS18B20 dq_pin be set to low
	//
	GPIO_ResetBits(psDev->ulPort, psDev->ulPin);

	DS18B20DelayNus(6);

	//
	// DS18B20 dq_pin be set to high
	//
	GPIO_SetBits(psDev->ulPort, psDev->ulPin);

	DS18B20DelayNus(3);

	//
	// DS18B20 dq pin be set as input
	//
	GPIO_Def.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(psDev->ulPort, &GPIO_Def);

	if (GPIO_ReadInputDataBit(psDev->ulPort, psDev->ulPin)) {
		ucData = 1;
	} else {
		ucData = 0;
	}

	DS18B20DelayNus(60);

	//
	// DS18B20 dq_pin be set to high
	//
	GPIO_Def.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(psDev->ulPort, &GPIO_Def);
	GPIO_SetBits(psDev->ulPort, psDev->ulPin);

	DS18B20DelayNus(10);
	return ucData;
}

//*****************************************************************************
//
//! \brief Read a byte from the DS18B20.
//!
//! \param psDev is a pointer which contains a DS18B20 device information.
//!
//! \return a byte of data.
//
//*****************************************************************************
unsigned char DS18B20ByteRead(tDS18B20Dev *psDev) {
	unsigned char i, ucData;
	ucData = 0;

	//
	// DS18B20 dq_pin be set to high
	//
	GPIO_Def.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(psDev->ulPort, &GPIO_Def);
	GPIO_SetBits(psDev->ulPort, psDev->ulPin);

	DS18B20DelayNus(2);

	for (i = 0; i < 8; i++) {
		ucData = ucData >> 1;
		if (DS18B20BitRead(psDev))
			ucData |= 0x80;
	}

	return ucData;
}

//*****************************************************************************
//
//! \brief Write a bit from the DS18B20.
//!
//! \param psDev is a pointer which contains a DS18B20 device information.
//!
//! \return None.
//
//*****************************************************************************
void DS18B20BitWrite(tDS18B20Dev *psDev, unsigned char ucBit) {
	//
	// DS18B20 dq_pin be set to low
	//
	GPIO_ResetBits(psDev->ulPort, psDev->ulPin);
	DS18B20DelayNus(5);

	if (ucBit) {
		//
		// DS18B20 dq_pin be set to high
		//
		GPIO_SetBits(psDev->ulPort, psDev->ulPin);
		DS18B20DelayNus(5);
	} else {
		//
		// DS18B20 dq_pin be set to low
		//
		GPIO_ResetBits(psDev->ulPort, psDev->ulPin);
	}
	DS18B20DelayNus(65);

	//
	// DS18B20 dq_pin be set to high
	//
	GPIO_SetBits(psDev->ulPort, psDev->ulPin);
	DS18B20DelayNus(5);
}

//*****************************************************************************
//
//! \brief Write a byte from the DS18B20.
//!
//! \param psDev is a pointer which contains a DS18B20 device information.
//!
//! \return None.
//
//*****************************************************************************
void DS18B20ByteWrite(tDS18B20Dev *psDev, unsigned char ucByte) {
	unsigned char i;

	//
	// DS18B20 dq pin be set as output
	//
	GPIO_Def.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(psDev->ulPort, &GPIO_Def);

	for (i = 0; i < 8; i++) {
		DS18B20BitWrite(psDev, ((ucByte >> i) & 0x1));
	}
}

#if (DS18B20_SEARCH_ROM_EN > 0)
//*****************************************************************************
//
//! \brief Perform the 1-Wire Search Algorithm on the 1-Wire bus using the 
//! existing search state.
//!
//! \param psDev is a pointer which contains a DS18B20 device information.
//!
//! \return 1  : device found, ROM number in ucROM buffer
//!         0 : device not found, end of search
//
//*****************************************************************************
uint8_t DS18B20ROMSearch(tDS18B20Dev *psDev) {
	int IdBitNumber, i;
	int LastZero, ROMByteNumber, SearchResult;
	int IDBit, ComplementIDBitR;
	unsigned char ROMByteMask, SearchDirection;
	//
	// initialize for search
	//
	IdBitNumber = 1;
	LastZero = 0;
	ROMByteNumber = 0;
	ROMByteMask = 1;
	SearchResult = 0;
	ucCrc8 = 0;
	//
	// if the last call was not the last one
	//
	if (!LastDeviceFlag) {
		//
		// 1-Wire reset
		//
		if (DS18B20Reset(psDev)) {
			//
			// reset the search
			//
			LastDiscrepancy = 0;
			LastDeviceFlag = 0;
			LastFamilyDiscrepancy = 0;
			return 0;
		}
		//
		// issue the search command
		//
		DS18B20ByteWrite(psDev, DS18B20_SEARCH);
		//
		// loop to do the search
		//
		do {
			//
			// read a bit and its complement
			//
			IDBit = DS18B20BitRead(psDev);
			ComplementIDBitR = DS18B20BitRead(psDev);
			//
			// check for no devices on 1-wire
			//
			if ((IDBit == 1) && (ComplementIDBitR == 1)) {
				break;
			} else {
				//
				// all devices coupled have 0 or 1
				//
				if (IDBit != ComplementIDBitR) {
					//
					// bit write value for search
					//
					SearchDirection = IDBit;
				} else {
					//
					// if this discrepancy if before the Last Discrepancy
					// on a previous next then pick the same as last time
					//
					if (IdBitNumber < LastDiscrepancy) {
						SearchDirection = ((ucROM[ROMByteNumber] & ROMByteMask)
								> 0);
					} else {
						//
						// if equal to last pick 1, if not then pick 0
						//
						SearchDirection = (IdBitNumber == LastDiscrepancy);
					}
					//
					// if 0 was picked then record its position in LastZero
					//
					if (SearchDirection == 0) {
						LastZero = IdBitNumber;
						//
						// check for Last discrepancy in family
						//
						if (LastZero < 9)
							LastFamilyDiscrepancy = LastZero;
					}
				}
				//
				// set or clear the bit in the ROM byte ROMByteNumber
				// with mask ROMByteMask
				//
				if (SearchDirection == 1)
					ucROM[ROMByteNumber] |= ROMByteMask;
				else
					ucROM[ROMByteNumber] &= ~ROMByteMask;
				//
				// serial number search direction write bit
				//
				DS18B20BitWrite(psDev, SearchDirection);
				//
				// increment the byte counter IdBitNumber
				// and shift the mask ROMByteMask
				//
				IdBitNumber++;
				ROMByteMask <<= 1;
				//
				// if the mask is 0 then go to new SerialNum byte ROMByteNumber
				// and reset mask
				//
				if (ROMByteMask == 0) {
					//
					// accumulate the CRC
					//
					//doucCrc8(ucROM[ROMByteNumber]);
					ROMByteNumber++;
					ROMByteMask = 1;
				}
			}
		}
		//
		// loop until through all ROM bytes 0-7
		//
		while (ROMByteNumber < 8);
		//
		// if the search was successful then
		//
		if (!((IdBitNumber < 65) || (ucCrc8 != 0))) {
			//
			// search successful so set LastDiscrepancy,LastDeviceFlag,SearchResult
			//
			LastDiscrepancy = LastZero;
			//
			// check for last device
			//
			if (LastDiscrepancy == 0)
				LastDeviceFlag = 1;
			SearchResult = 1;
			for (i = 0; i < 8; i++) {
				psDev->ucROM[i] = ucROM[i];
			}
		}
	}
	//
	// if no device found then reset counters so next 'search' will be like a first
	//
	if (!SearchResult || !ucROM[0]) {
		LastDiscrepancy = 0;
		LastDeviceFlag = 0;
		LastFamilyDiscrepancy = 0;
		SearchResult = 0;
	}
	return SearchResult;
}

//*****************************************************************************
//
//! \brief Verify the device with the ROM number in ROM buffer is present. 
//!
//! \param psDev is a pointer which contains a DS18B20 device information.
//!
//! \return 1  : device verified present
//!         0 : device not present
//
//*****************************************************************************
uint8_t DS18B20Verify(tDS18B20Dev *psDev) {
	unsigned char ucROMBackup[8];
	int i, result, LDBackup, LDFBackup, LFDBackup;
	//
	// keep a backup copy of the current state
	//
	for (i = 0; i < 8; i++)
		ucROMBackup[i] = ucROM[i];
	LDBackup = LastDiscrepancy;
	LDFBackup = LastDeviceFlag;
	LFDBackup = LastFamilyDiscrepancy;
	//
	// set search to find the same device
	//
	LastDiscrepancy = 64;
	LastDeviceFlag = 0;
	if (DS18B20ROMSearch(psDev)) {
		//
		// check if same device found
		//
		result = 1;
		for (i = 0; i < 8; i++) {
			if (ucROMBackup[i] != ucROM[i]) {
				result = 0;
				break;
			}
		}
	} else
		result = 0;
	//
	// restore the search state
	//
	for (i = 0; i < 8; i++)
		ucROM[i] = ucROMBackup[i];
	LastDiscrepancy = LDBackup;
	LastDeviceFlag = LDFBackup;
	LastFamilyDiscrepancy = LFDBackup;
	//
	// return the result of the verify
	//
	return result;
}

//*****************************************************************************
//
//! \brief Setup the search to find the device type 'family_code' on the 
//! next call to DS18B20ROMSearch() if it is present. 
//!
//! \param psDev is a pointer which contains a DS18B20 device information.
//!
//! \return None
//
//*****************************************************************************
void DS18B20TargetSetup(tDS18B20Dev *psDev, unsigned char ucFamily) {
	int i;
	//
	// set the search state to find SearchFamily type devices
	//
	ucROM[0] = ucFamily;
	for (i = 1; i < 8; i++)
		ucROM[i] = 0;
	LastDiscrepancy = 64;
	LastFamilyDiscrepancy = 0;
	LastDeviceFlag = 0;
}

//*****************************************************************************
//
//! \brief Setup the search to skip the current device type on the next call
//! to DS18B20ROMSearch().
//!
//! \param psDev is a pointer which contains a DS18B20 device information.
//! \param ucFamily.
//!
//! \return None
//
//*****************************************************************************
void DS18B20FamilySkipSetup(tDS18B20Dev *psDev, unsigned char ucFamily) {
	//
	// set the Last discrepancy to last family discrepancy
	//
	LastDiscrepancy = LastFamilyDiscrepancy;
	//
	// check for end of list
	//
	if (LastDiscrepancy == 0)
		LastDeviceFlag = 1;
}
#endif

//*****************************************************************************
//
//! \brief Get the DS18B20's alarm flag is set or not.
//!
//! \param psDev is a pointer which contains a DS18B20 device information.
//!
//! \return None
//! \note This command can only be used when there is one slave on the bus.
//
//*****************************************************************************
void DS18B20AlarmSearch(tDS18B20Dev *psDev) {
	//DS18B20ByteWrite(psDev, DS18B20_ALARM_SEARCH);

	//if (DS18B20BitRead(psDev))
	//{
	//    return 1;
	//}
	//else
	//{
	//    return 0;
	//}
}
//*****************************************************************************
//
//! \brief Read the slave��s 64-bit ROM code without using the Search ROM 
//! procedure.
//!
//! \param psDev is a pointer which contains a DS18B20 device information.
//!
//! \return None
//! \note This command can only be used when there is one slave on the bus.
//
//*****************************************************************************
void DS18B20ROMRead(tDS18B20Dev *psDev) {
	int i;
	DS18B20ByteWrite(psDev, DS18B20_READ);
	for (i = 0; i < 8; i++) {
		psDev->ucROM[i] = DS18B20ByteRead(psDev);
	}
}

//*****************************************************************************
//
//! \brief The match ROM command followed by a 64-bit ROM code sequence allows
//! the bus master to address a specific slave device on a multidrop or 
//! single-drop bus.
//!
//! \param psDev is a pointer which contains a DS18B20 device information.
//!
//! Only the slave that exactly matches the 64-bit ROM code sequence will 
//! respond to the function command issued by the master;
//!
//! \return None
//
//*****************************************************************************
void DS18B20ROMMatch(tDS18B20Dev *psDev) {
	int i;
	DS18B20ByteWrite(psDev, DS18B20_MATCH);
	for (i = 0; i < 8; i++) {
		DS18B20ByteWrite(psDev, psDev->ucROM[i]);
	}
}

//*****************************************************************************
//
//! \brief The master can use this command to address all devices on the bus
//! simultaneously without sending out any ROM code information.
//!
//! \param psDev is a pointer which contains a DS18B20 device information.
//!
//! \return None
//! \note that the Read Scratchpad [BEh] command can follow the Skip ROM command
//! only if there is a single slave device on the bus. In this case, time is 
//! saved by allowing the master to read from the slave without sending the 
//! device��s 64-bit ROM code. A Skip ROM command followed by a Read Scratchpad
//! command will cause a data collision on the bus if there is more than one
//! slave since multiple devices will attempt to transmit data simultaneously.
//
//*****************************************************************************
void DS18B20ROMSkip(tDS18B20Dev *psDev) {
	DS18B20ByteWrite(psDev, DS18B20_SKIP);
}

//*****************************************************************************
//
//! \brief Initiates temperature conversion.
//!
//! \param psDev is a pointer which contains a DS18B20 device information.
//!
//! \return None
//
//*****************************************************************************
void DS18B20TempConvert(tDS18B20Dev *psDev) {
	DS18B20ByteWrite(psDev, DS18B20_CONVERT);
	DS18B20DelayNus(750000);
}

//*****************************************************************************
//
//! \brief copies the contents of the scratchpad TH, TL and configuration 
//! registers (bytes 2, 3 and 4) to EEPROM.
//!
//! \param psDev is a pointer which contains a DS18B20 device information.
//!
//! \return None
//
//*****************************************************************************
void DS18B20ScratchpadCopy(tDS18B20Dev *psDev) {
	DS18B20ByteWrite(psDev, DS18B20_COPY_SCRATCHPAD);
	DS18B20DelayNus(10);
}

//*****************************************************************************
//
//! \brief Set the Th and Tl register.
//!
//! \param psDev is a pointer which contains a DS18B20 device information.
//! \param cHigh The data is written into the TH register (byte 2 of the 
//! scratchpad).
//! \param cLow The data is written into the TL register (byte 3).
//! \param ucBitConfig The data is written into the configuration register 
//! (byte 4).
//!
//! \return None
//
//*****************************************************************************
void DS18B20ScratchpadSet(tDS18B20Dev *psDev, char cHigh, char cLow,
		unsigned char ucBitConfig) {
	DS18B20ByteWrite(psDev, DS18B20_WRITE_SCRATCHPAD);
	DS18B20ByteWrite(psDev, cHigh);
	DS18B20ByteWrite(psDev, cLow);
	DS18B20ByteWrite(psDev, ucBitConfig);
}

//*****************************************************************************
//
//! \brief Get the Temperature of DS18B20.
//!
//! \param psDev is a pointer which contains a DS18B20 device information.
//! \param pfTemp The flaot Temperature value.
//!
//! \return None
//
//*****************************************************************************
void DS18B20TempRead(tDS18B20Dev *psDev, float *pfTemp) {
	unsigned short ulTemp = 0;
	DS18B20ByteWrite(psDev, DS18B20_READ_SCRATCHPAD);
	ulTemp = DS18B20ByteRead(psDev);
	ulTemp |= (DS18B20ByteRead(psDev) << 8);
	if (ulTemp > 2097) {
		ulTemp = 65536 - ulTemp;
		*pfTemp = -(((ulTemp & 0x7F0) >> 4) * 1.0 + (ulTemp & 0xf) * 0.0625);
	} else {
		*pfTemp = ((ulTemp & 0x7F0) >> 4) * 1.0 + (ulTemp & 0xf) * 0.0625;
	}
}

//*****************************************************************************
//
//! \brief Recalls TH, TL, and configuration register data from EEPROM to the
//! scratchpad..
//!
//! \param psDev is a pointer which contains a DS18B20 device information.
//!
//! \return None
//
//*****************************************************************************
void DS18B20EEROMRecall(tDS18B20Dev *psDev) {
	DS18B20ByteWrite(psDev, DS18B20_RECALL);
	//
	// DS18B20 dq_pin be set to high
	//
	GPIO_Def.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(psDev->ulPort, &GPIO_Def);
	GPIO_SetBits(psDev->ulPort, psDev->ulPin);

	DS18B20DelayNus(5);
	//
	// Wait utill the recall is done.
	//
	while (DS18B20BitRead(psDev))
		;
}

//*****************************************************************************
//
//! \brief Get DS18B20s's power supply.
//!
//! \param psDev is a pointer which contains a DS18B20 device information.
//!
//! \return 1  : externally powered
//!         0 : parasite powered
//
//*****************************************************************************
uint8_t DS18B20PowerSupplyRead(tDS18B20Dev *psDev) {
	DS18B20ByteWrite(psDev, DS18B20_READ_POWER_SUPPLY);
	if (DS18B20BitRead(psDev)) {
		return 1;
	} else {
		return 0;
	}
}

//*****************************************************************************
//
//! \brief Get the Scratchpad of DS18B20.
//!
//! \param psDev is a pointer which contains a DS18B20 device information.
//! \param pucTemp The point of Scratchpad.
//!
//! \return None
//
//*****************************************************************************
void DS18B20ScratchpadRead(tDS18B20Dev *psDev, unsigned char *pucTemp) {
	int i;
	DS18B20ByteWrite(psDev, DS18B20_READ_SCRATCHPAD);
	for (i = 0; i < 8; i++) {
		*pucTemp++ = DS18B20ByteRead(psDev);
	}

}

/**
 * The loop takes 10 cycles/loop.
 */
void __attribute__ ((used, naked)) SysCtlDelay(unsigned long ulCount) {
	__asm("subs    r0, #1\n"
			"bne     SysCtlDelay\n"
			"bx      lr\n");
}
