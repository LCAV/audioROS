#ifndef SPI_COMM_H
#define SPI_COMM_H

#include <stdint.h>
#include <hal.h>

#define SPI_COMMAND_SIZE 64
#define SPI_DATA_HEADER_SIZE 4
#define SPI_DATA_PAYLOAD_SIZE 4092
#define SPI_DELAY 1200

#define SPI_PACKET_MAX_SIZE 4092

void spi_comm_start(void);

#endif /* SPI_COMM_H */
