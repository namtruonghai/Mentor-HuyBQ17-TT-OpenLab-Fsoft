/*
 * dht11.h
 *
 *  Created on: Jul 18, 2024
 *      Author: ADMIN
 */
#include <stdint.h>
#ifndef DHT11_H_
#define DHT11_H_
typedef uint32_t sl_status_t;

sl_status_t dht22_getRHTdata(int16_t* TEMP, int16_t* RH);
void dht22_init(void);


#endif /* DHT11_H_ */
