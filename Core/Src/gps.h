
#ifndef SRC_GPS_H_
#define SRC_GPS_H_


#define GPS_BUFFER_SIZE		256

typedef enum
{
	GPS_OK = 0,
	GPS_ERROR = 1
}GPS_STATUS;


GPS_STATUS GPS_init(void);
size_t GPS_store_data(uint8_t *data, size_t len);
GPS_STATUS GPS_process_data(void);

#endif /* SRC_GPS_H_ */
