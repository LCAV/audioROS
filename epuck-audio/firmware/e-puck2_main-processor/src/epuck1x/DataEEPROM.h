#ifndef DATA_EEPROM
#define DATA_EEPROM

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Function available only for legacy software compatibility, there is no EEPROM on e-puck2.
 * Parameters Definition:
 * Page:        is the 8 most significant bits of the source address in EEPROM
 * Offset:      is 16 least significant bits of the source address in EEPROM
 * DataOut:     is the 16-bit address of the destination RAM location or array
 * Size:        is the number of words to read from EEPROM and is a value of 1 or 16
 * Return Value:
 * Function returns 0
 */
int ReadEE(int Page, int Offset, int* DataOut, int Size);

/*
 * Function available only for legacy software compatibility, there is no EEPROM on e-puck2.
 * Parameters Definition:
 * Page:        is the 8 most significant bits of the address in EEPROM to be erased
 * Offset:      is 16 least significant bits of the address in EEPROM to be erased
 * Size:        is the number of words to read from EEPROM and is a value of 1, 16 or
 *              0xFFFF (for erasing ALL EEPROM memory)
 * Return Value:
 * Function returns 0
 */
int EraseEE(int Page, int Offset, int Size);

/*
 * Function available only for legacy software compatibility, there is no EEPROM on e-puck2.
 * Parameters Definition:
 * Page:        is the 8 most significant bits of the destination address in EEPROM
 * Offset:      is 16 least significant bits of the destination address in EEPROM
 * DataIn:      is the 16-bit address of the source RAM location or array
 * Size:        is the number of words to read from EEPROM and is a value of 1 or 16
 * Return Value:
 * Function returns 0
 */
int WriteEE(int* DataIn, int Page, int Offset, int Size);

#ifdef __cplusplus
}
#endif

#endif
