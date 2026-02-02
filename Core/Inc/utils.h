/*
 * utils.h
 *
 *  Created on: Apr 1, 2024
 *      Author: PaganLuc
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include "main.h"
#include "stdint.h"

#include <stdio.h>
#include <string.h>

void 	hex2ascii(char* esadecimale, char* ascii);


extern uint8_t	 msg_per_printf[512];
#define    PRONO_PRINT(...)	{\
	memset(msg_per_printf, 0x00, sizeof(msg_per_printf));\
	sprintf((char*)&msg_per_printf, __VA_ARGS__);\
	HAL_UART_Transmit(&huart3, (const uint8_t*) msg_per_printf, strlen((char*)msg_per_printf), 10000);\
}


/*
* Very basic boolean typedef
*/
typedef 	uint8_t 		boolean_t;
#define		FALSE			((boolean_t)0)
#define		TRUE			((boolean_t)1)

/**
* Bitmasks
*/ 
#define		SET_FLAG( entity, bmask ) 		( entity |= bmask )
#define		CLEAR_FLAG( entity, bmask ) 	( entity &= ~bmask )
#define		TOGGLE_FLAG( entity, bmask ) 	( entity ^= bmask )
#define		IS_FLAG_SET( entity, bmask )	( entity & bmask )
#define		IS_FLAG_CLEAR( entity, bmask )	!( entity & bmask )

/**
* IOs
*/
#define		IS_IO_PIN_SET( port, pin )		( HAL_GPIO_ReadPin( port, pin ) == GPIO_PIN_SET )
#define		IS_IO_PIN_CLEAR( port, pin )	( HAL_GPIO_ReadPin( port, pin ) == GPIO_PIN_CLEAR )
#define		TOGGLE_PIN( port, pin )			HAL_GPIO_TogglePin( port, pin )
#define		SET_PIN( port, pin )			HAL_GPIO_WritePin( port, pin, GPIO_PIN_SET )
#define		CLEAR_PIN( port, pin )			HAL_GPIO_WritePin( port, pin, GPIO_PIN_RESET )

/**
* Bitfields
*/
#define		SET_BIT_bf( bitfield, bit_position ) 		bitfield.bit_position = 1
#define		CLEAR_BIT_bf( bitfield, bit_position ) 		bitfield.bit_position = 0
#define		TOGGLE_BIT_bf( bitfield, bit_position )		bitfield.bit_position ^= 1
#define		IS_BIT_SET_bf( bitfield, bit_position ) 	( bitfield.bit_position == 1 )
#define		IS_BIT_CLEAR_bf( bitfield, bit_position ) 	( bitfield.bit_position == 0 )








typedef struct
{
	uint8_t 	b0 :1;
	uint8_t 	b1 :1;
	uint8_t 	b2 :1;
	uint8_t 	b3 :1;
	uint8_t 	b4 :1;
	uint8_t 	b5 :1;
	uint8_t 	b6 :1;
	uint8_t 	b7 :1;
} bitfield8_t;


typedef struct
{
	uint8_t 	b0  :1;
	uint8_t 	b1  :1;
	uint8_t 	b2  :1;
	uint8_t 	b3  :1;
	uint8_t 	b4  :1;
	uint8_t 	b5  :1;
	uint8_t 	b6  :1;
	uint8_t 	b7  :1;	
	uint8_t 	b8  :1;
	uint8_t 	b9  :1;
	uint8_t 	b10 :1;
	uint8_t 	b11 :1;
	uint8_t 	b12 :1;
	uint8_t 	b13 :1;
	uint8_t 	b14 :1;
	uint8_t 	b15 :1;
} bitfield16_t;


typedef struct
{
	uint8_t 	b0  :1;
	uint8_t 	b1  :1;
	uint8_t 	b2  :1;
	uint8_t 	b3  :1;
	uint8_t 	b4  :1;
	uint8_t 	b5  :1;
	uint8_t 	b6  :1;
	uint8_t 	b7  :1;	
	uint8_t 	b8  :1;
	uint8_t 	b9  :1;
	uint8_t 	b10 :1;
	uint8_t 	b11 :1;
	uint8_t 	b12 :1;
	uint8_t 	b13 :1;
	uint8_t 	b14 :1;
	uint8_t 	b15 :1;	
	uint8_t 	b16 :1;
	uint8_t 	b17 :1;
	uint8_t 	b18 :1;
	uint8_t 	b19 :1;
	uint8_t 	b20 :1;
	uint8_t 	b21 :1;
	uint8_t 	b22 :1;
	uint8_t 	b23 :1;	
	uint8_t 	b24 :1;
	uint8_t 	b25 :1;
	uint8_t 	b26 :1;
	uint8_t 	b27 :1;
	uint8_t 	b28 :1;
	uint8_t 	b29 :1;
	uint8_t 	b30 :1;
	uint8_t 	b31 :1;
} bitfield32_t;


#define		BIT_0 			0 
#define		BIT_1 			1 
#define		BIT_2 			2 
#define		BIT_3 			3 
#define		BIT_4 			4 
#define		BIT_5 			5 
#define		BIT_6 			6 
#define		BIT_7 			7 
#define		BIT_8 			8 
#define		BIT_9 			9 
#define		BIT_10			10
#define		BIT_11			11
#define		BIT_12			12
#define		BIT_13			13
#define		BIT_14			14
#define		BIT_15			15
#define		BIT_16			16
#define		BIT_17			17
#define		BIT_18			18
#define		BIT_19			19
#define		BIT_20			20
#define		BIT_21			21
#define		BIT_22			22
#define		BIT_23			23
#define		BIT_24			24
#define		BIT_25			25
#define		BIT_26			26
#define		BIT_27			27
#define		BIT_28			28
#define		BIT_29			29
#define		BIT_30			30
#define		BIT_31			31


#define		BMASK_0 		( 1 << 0  )
#define		BMASK_1 		( 1 << 1  )
#define		BMASK_2 		( 1 << 2  )
#define		BMASK_3 		( 1 << 3  )
#define		BMASK_4 		( 1 << 4  )
#define		BMASK_5 		( 1 << 5  )
#define		BMASK_6 		( 1 << 6  )
#define		BMASK_7 		( 1 << 7  )
#define		BMASK_8 		( 1 << 8  )
#define		BMASK_9 		( 1 << 9  )
#define		BMASK_10		( 1 << 10 )
#define		BMASK_11		( 1 << 11 )
#define		BMASK_12		( 1 << 12 )
#define		BMASK_13		( 1 << 13 )
#define		BMASK_14		( 1 << 14 )
#define		BMASK_15		( 1 << 15 )
#define		BMASK_16		( 1 << 16 )
#define		BMASK_17		( 1 << 17 )
#define		BMASK_18		( 1 << 18 )
#define		BMASK_19		( 1 << 19 )
#define		BMASK_20		( 1 << 20 )
#define		BMASK_21		( 1 << 21 )
#define		BMASK_22		( 1 << 22 )
#define		BMASK_23		( 1 << 23 )
#define		BMASK_24		( 1 << 24 )
#define		BMASK_25		( 1 << 25 )
#define		BMASK_26		( 1 << 26 )
#define		BMASK_27		( 1 << 27 )
#define		BMASK_28		( 1 << 28 )
#define		BMASK_29		( 1 << 29 )
#define		BMASK_30		( 1 << 30 )
#define		BMASK_31		( 1 << 31 )


#endif /* INC_UTILS_H_ */
