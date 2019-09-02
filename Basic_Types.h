#ifndef BASIC_TYPES_H
#define BASIC_TYPES_H
/*This file is containing the global APIs that are used for any standard project*/
/*********************************Global Symbols******************************/
/*
Symbol name: F_CPU
Symbol Description: MCU CPU Freq as defined by AVR/io.h
Symbol Range: for ATMega16 : 0 .. 16000000
*/
#define F_CPU 8000000UL

/*
Symbol name: OP_ON
Symbol Description: enable a specific feature
Symbol Range: 1U
*/
#define OP_ON 1U

/*
Symbol name: OP_OFF
Symbol Description: disable a specific feature
Symbol Range: 0U
*/
#define OP_OFF 0U
/*********************************Global Types********************************/
/*
Type name: u8
Type Description: unsigned char type
Type Range: 0 .. 255
*/

typedef unsigned char u8;

/*
Type name: u16
Type Description: unsigned short int type
Type Range: 0 .. 65535
*/

typedef unsigned short int u16;

/*
Type name: u32
Type Description: unsigned long int type
Type Range: 0 .. 4294967295
*/

typedef unsigned long int u32;

/*
Type name: BasicRet
Type Description: Basic Return type
Type Range: OK,NOK
*/
typedef enum { OK,NOK } BasicRet;

/*
Type name: bool
Type Description: bool type
Type Range: TRUE,FALSE
*/
typedef enum { TRUE,FALSE } bool;

#endif

