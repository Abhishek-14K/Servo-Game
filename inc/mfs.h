/*
 * mfs.h
 *
 *  Created on: Apr 3, 2022
 *      Author: abhis
 */

#ifndef INC_MFS_H_
#define INC_MFS_H_

void set_led_1( uint32_t on );
void set_led_2( uint32_t on );
void set_led_3( uint32_t on );
void shiftOut(uint8_t val);
void WriteNumberToSegment(int Segment, int Value);
void segdisplay(int x);

#endif /* INC_MFS_H_ */
