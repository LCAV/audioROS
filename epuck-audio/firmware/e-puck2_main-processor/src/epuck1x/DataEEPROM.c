#include "DataEEPROM.h"

int ReadEE(int Page, int Offset, int* DataOut, int Size) {
	(void)Size;
	if(Page==0x7F && Offset==0xFFFE) {
		*DataOut = 0xFFF7;	// The last two bytes of the EEPROM, at address 0x7FFFFE, were used in the e-puck1.x to store the camera model and related orientation.
							// The e-puck2 has no EEPROM storage and has only one camera model, that is the PO8030 with no rotation, thus return 0xFFF7.
	} else {
		*DataOut = 0; 		// No EEPROM storage available on e-puck2, thus return 0 when other sections are requested.
	}
	return 0;
}

int EraseEE(int Page, int Offset, int Size) {
	(void)Page;
	(void)Offset;
	(void)Size;
	return 0; // No EEPROM storage available on e-puck2, thus no operation is performed.
}

int WriteEE(int* DataIn, int Page, int Offset, int Size) {
	(void)DataIn;
	(void)Page;
	(void)Offset;
	(void)Size;
	return 0; // No EEPROM storage available on e-puck2, thus no operation is performed.
}


