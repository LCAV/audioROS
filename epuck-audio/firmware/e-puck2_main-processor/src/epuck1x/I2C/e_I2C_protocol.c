/********************************************************************************

			I2C master module
			Version 1.0 may 2005 Davis Daidie


This file is part of the e-puck library license.
See http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45

(c) 2005-2007 Davis Daidie

Robotics system laboratory http://lsro.epfl.ch
Laboratory of intelligent systems http://lis.epfl.ch
Swarm intelligent systems group http://swis.epfl.ch
EPFL Ecole polytechnique federale de Lausanne http://www.epfl.ch

**********************************************************************************/

/*! \file
 * \ingroup i2c
 * \brief Manage I2C protocole.
 *
 * This module manage the I2C protocole. The function's module are 
 * made to directly send or receive data from or to a specified slave.
 * \warning This file must be include to communicate with an PO3030K
 * camera throught the I2C communication protocol
 * \author Code: Davis Daidie \n Doc: Jonathan Besuchet
 */

#include "e_I2C_protocol.h"
#include "../i2c_bus.h"

/*! \brief Initialize the microcontroller for I2C uses
 * \return 1 to confirme the oparation and 0 for an error
 */
void e_i2cp_init(void)
{
	return;
}

void e_i2cp_deinit(void)
{
	return;
}

/*! \brief Enable special I2C interrupt
 * \return 1 to confirme the oparation and 0 for an error
 */
void e_i2cp_enable(void)
{
	return;
}

/*! \brief Disable special I2C interrupt
 * \return 1 to confirme the oparation and 0 for an error
 */
void e_i2cp_disable(void)
{
	return;
}

/*! \brief Read a specific register on a device
 * \param device_add The address of the device you want information
 * \param reg The register address you want read on the device
 * \return The readed value
 */
char e_i2cp_read(char device_add, char reg)
{
	uint8_t value = 0;
	read_reg(device_add>>1, reg, &value);
	return value;
}

/*! \brief Write a specific register on a device
 * \param device_add The address of the device you want information
 * \param reg The register address you want read on the device
 * \param value The data you want to write
 * \return 1 to confirme the oparation and 0 for an error
 */
char e_i2cp_write (char device_add, char reg, char value)
{
	int8_t err = 0;
	if((err = write_reg(device_add>>1, reg, value)) != MSG_OK) {
		return 0;
	} else {
		return 1;
	}
}

