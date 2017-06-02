#ifndef _UTILS_H
#define _UTILS_H

#define LOW( val ) ((val) & 0xFF)

// Utility macros to set, clear, and flip specified bits
#define SET_BITS( port, bits )            	((port) |= (bits))
#define CLR_BITS( port, bits )            	((port) &= LOW(~(bits)))
#define FLIP_BITS( port, bits )           	((port) ^= (bits))

// Utility macro to force the bits identified by mask to take on the values of the matching bits in value
#define FORCE_BITS( port, mask, value)		((port) = ((port) & LOW(~(mask))) | ((value) & (mask)))


// Macro to calculate the vector # given the vector address
// - take 1's complement of address to get a positive value
// - mask off lower byte since that's the only meaningful data - rest should be 0
// - right shift by 1 (divide by 2) to get the vector number since each vector address is 2 bytes wide
// - (The LSB will be 1 before shift.  This will get shifted off on the divide by 2.  e.g. 3/2 = 1, 5/2= 2)
#define VECTOR_NUM( vector_addr ) 			(((~(vector_addr)) & 0xFF) >> 1)

#define BIT_IS_SET( port, bits )			((port) & (bits))
#define BIT_IS_CLR( port, bits )			((~(port))&(bits))

#define LO_NYBBLE( value )  ((value) & 0x0F)
#define HI_NYBBLE( value )  (((value) >> 4) & 0x0F)

#endif