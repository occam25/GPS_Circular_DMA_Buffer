

#include <string.h>
#include "ringbuff/ringbuff.h"
#include "main.h"
#include "gps.h"

uint8_t GPS_Buffer[GPS_BUFFER_SIZE];

uint8_t processing_buffer[GPS_BUFFER_SIZE];

static ringbuff_t buff;


void process_data(uint8_t *buffer, size_t len)
{
	HAL_UART_Transmit(&huart2, buffer, len, 100);
	HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 100);

	return;
}

GPS_STATUS GPS_init(void)
{
	ringbuff_init(&buff, GPS_Buffer, sizeof(GPS_Buffer));
	return GPS_OK;
}

size_t GPS_store_data(uint8_t *data, size_t len)
{
	return ringbuff_write(&buff, data, len);
}

GPS_STATUS GPS_process_data(void)
{
	static size_t index = 0;
	uint16_t i;
	size_t nbytes;
	static uint8_t *begin = NULL;
	uint8_t *end = NULL;

	nbytes = ringbuff_read(&buff, processing_buffer + index, sizeof(processing_buffer) - index);


	// Process data
	for(i = index; i < nbytes + index; i++){
		if(processing_buffer[i] == '$'){
			begin = processing_buffer + i;
		//}else if((begin != NULL)&&(processing_buffer[i] == '\n' || processing_buffer[i] == '\r')){
		}else if((begin != NULL)&&(processing_buffer[i] == '#')){
			end = processing_buffer + i;
			// process data
			process_data(begin, end - begin);
			begin = NULL;
			end = NULL;
		}
	}

	// begin character not processed (no end character foud yet) -> update index and move data
	if(begin != NULL){ // && end == NULL){
		index += nbytes - (begin - processing_buffer);
		if(begin != processing_buffer){
			for(i = 0; i < index; i++){
				processing_buffer[i] = *(begin + i);
			}
			begin = processing_buffer;
		}
	}else{
		index = 0;
	}

	return GPS_OK;
}
