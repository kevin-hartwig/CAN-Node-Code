#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include <stdio.h>
#include <stdlib.h>
#include "utils.h"  
//#include "candefinitions.h"  

// CAR CONTROLLER //

/*//////////////////////////////////////////////////////////////////////////////////////////////// 
 *    Global Constants and Macros
 *////////////////////////////////////////////////////////////////////////////////////////////////
 
/* Acceptance Code Definitions */
/* 0x100 Defines */
#define ACC_CODE_ID100 0x2000
#define ACC_CODE_ID100_HIGH ((ACC_CODE_ID100 & 0xFF00)>>8)
#define ACC_CODE_ID100_LOW (ACC_CODE_ID100 & 0x00FF)  

#define ACC_CODE_ID200 0x4000
#define ACC_CODE_ID200_HIGH ((ACC_CODE_ID200 & 0xFF00)>>8)
#define ACC_CODE_ID200_LOW (ACC_CODE_ID200 & 0x00FF)  

/* Mask Code Definitions */
#define MASK_CODE_ST_ID 0x0007
#define MASK_CODE_ST_ID_HIGH ((MASK_CODE_ST_ID & 0xFF00)>>8)
#define MASK_CODE_ST_ID_LOW (MASK_CODE_ST_ID & 0xFF)

#define ST_ID_100 0x100 
#define ST_ID_200 0x200

#define MY_NODE_ID CC

// local typedefs
typedef unsigned char	U8;
typedef signed char	S8;
typedef unsigned int	U16;
typedef signed int	S16;
typedef unsigned long	U32;
typedef signed long	S32;  

//Structure for Getting the Message from Registers (Buffer)
typedef struct {
  U32 id;
  U8 length;
  U8 data[8];
} CANmsg_t;

/*//////////////////////////////////////////////////////////////////////////////////////////////// 
 *    Global Variables
 *////////////////////////////////////////////////////////////////////////////////////////////////

unsigned char txbuffer = 0;

/*//////////////////////////////////////////////////////////////////////////////////////////////// 
 *    Function Prototypes
 *////////////////////////////////////////////////////////////////////////////////////////////////
void CANInit (void);
int MSCAN_GotMsg( void ); 
void interrupt CAN0RxISR(void); 
unsigned char CANSendFrame (unsigned long id, char priority, char length, char *txdata );
unsigned char CANReceiveFrame (unsigned long id, char priority, char length, char *txdata );
void MSCAN_Getd( CANmsg_t * thisMsg );
void MSCAN_ListenInit();


/*//////////////////////////////////////////////////////////////////////////////////////////////// 
 *    CANInit
 *
 *     - Initialize CAN 
 *
 *////////////////////////////////////////////////////////////////////////////////////////////////
void CANInit (void) {

  CANCTL0 = 0x01; /* Enter Initialization Mode */
  while (! (CANCTL1&0x01)) {}; /* Wait for Initialization Mode
                                  acknowledge (INITRQ bit = 1) */

  //CANCTL1 = 0xA0;        //this enables loop back mode functioning
  CANCTL1 = 0b11000000;    // no loopback, bus clock   , third bit 1 for loopback

  //CANBTR0 = 0xC7;
  CANBTR0 = 0b00000111;
  CANBTR1 = 0b00100011;
  //CANBTR1 = 0b00110010;
  //CANBTR1 = 0x3A;        //set timing segment 1 = 11 time quanta, segment 2 = 4 time quanta  
  
  
  /* Acceptance Filters */
  CANIDAC = 0x10; /* Set four 16-bit Filters     */
  
  CANIDAR0 = ACC_CODE_ID100_HIGH;   //|\ 16-bit Filter 0
  CANIDMR0 = MASK_CODE_ST_ID_HIGH;  //| \__ Accepts Standard Data Frame Msg
  CANIDAR1 = ACC_CODE_ID100_LOW;    //| / with ID 0x100
  CANIDMR1 = MASK_CODE_ST_ID_LOW;   //|/

  /* Acceptance Filters */
  CANIDAC = 0x10; /* Set four 16-bit Filters   */

  CANIDAR2 = ACC_CODE_ID200_HIGH;   //|\ 16-bit Filter 0
  CANIDMR2 = MASK_CODE_ST_ID_HIGH;  //| \__ Accepts Standard Data Frame Msg
  CANIDAR3 = ACC_CODE_ID200_LOW;    //| / with ID 0x200
  CANIDMR3 = MASK_CODE_ST_ID_LOW;   //|/

  CANIDAR4 = 0x00;                  //|\    16-bit Filter 2
  CANIDMR4 = MASK_CODE_ST_ID_HIGH;  //| \__ Accepts Standard Data Frame Msg
  CANIDAR5 = 0x00;                  //| /   with ID 0x100
  CANIDMR5 = MASK_CODE_ST_ID_LOW;   //|/

  CANIDAR6 = 0x00;                  //|\    16-bit Filter 3
  CANIDMR6 = MASK_CODE_ST_ID_HIGH;  //| \__ Accepts Standard Data Frame Msg
  CANIDAR7 = 0x00;                  //| /   with ID 0x100
  CANIDMR7 = MASK_CODE_ST_ID_LOW;   //|/

  CANCTL0 = 0x00; /* Exit Initialization Mode Request */

  while ((CANCTL1&0x00) != 0) {}; /* Wait for Normal Mode */


}

/*//////////////////////////////////////////////////////////////////////////////////////////////// 
 *    CANSendFrame
 *
 *     - Transmits message over CAN bus
 *
 *////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char CANSendFrame (unsigned long id, char priority, char length, char *txdata ) {
   
  int index = 0; 
  U32 myIdBfr = 0;	// local copy of 32 bit message ID - msg ID built here and then copied to outpu buffer

  if (!CANTFLG) // Is Transmit Buffer full?? 
    return -1;

  CANTBSEL = CANTFLG; /* Select lowest empty buffer */
  txbuffer = CANTBSEL; /* Backup selected buffer */

   /* Load Id to IDR Register */
 	myIdBfr = ((id & 0x07ff) << (32-11))
				| (((U32)FALSE & 0x01) << (32 - 11 - 1));

 *((unsigned long *) ((unsigned long)(&CANTXIDR0))) = myIdBfr;
 
 for (index=0;index<length;index++) {
   *(&CANTXDSR0 + index) = txdata[index]; //Load data to Tx buffer
   // Data Segment Registers      
 }
 
 CANTXDLR = length; // Set Data Length Code 
 CANTXTBPR = priority; // Set Priority 
 CANTFLG = txbuffer; // Start transmission 
 while ( (CANTFLG & txbuffer) != txbuffer); // Wait for Transmission
 // completion    
}


/*//////////////////////////////////////////////////////////////////////////////////////////////// 
 *    MSCAN_Getd
 *
 *     - Read data from CAN bus
 *
 *////////////////////////////////////////////////////////////////////////////////////////////////
void MSCAN_Getd( CANmsg_t * thisMsg ) {

int i;
int j = 0;
U32 id2 = 0;
U8 id3 = 0;
U8 * dataPtr;

//thisMsg->id = *(U32 *)(&CANRXIDR0);
thisMsg->id = (*(U16 *)(&CANRXIDR0)) >> 5;   //bit shift by 5 because of how data is stored in registers.
//id2 = (U16)thisMsg->id;
//id3 = (U8)thisMsg->id;

  if(j == 0 & id2 == 0){
  //thisMsg->id = *((unsigned long *) ((unsigned long)(&CANTXIDR0))); 
  //id2 = CANRXIDR0;

  }

//id2 = thisMsg->id; 

thisMsg->length = CANRXDLR;
dataPtr = (U8 *)(&CANRXDSR0);

for (i = 0; i < thisMsg->length; i++)
	thisMsg->data[i] = *dataPtr++;
	
SET_BITS( CANRFLG, CANRFLG_RXF_MASK );
	
} // MSCAN_Getd()


/*//////////////////////////////////////////////////////////////////////////////////////////////// 
 *    MSCAN_GotMsg
 *
 *     - Check CAN Rx flag for message waiting
 *
 *////////////////////////////////////////////////////////////////////////////////////////////////
int MSCAN_GotMsg( void ) {
  return ( BIT_IS_SET( CANRFLG, CANRFLG_RXF_MASK ) );	
}   

/*//////////////////////////////////////////////////////////////////////////////////////////////// 
 *    MSCAN_ListenForMsg
 *
 *     - Setup 2 filters: 1 for standard and 1 for extended for the same message
 *     - Look for exact match
 *
 *////////////////////////////////////////////////////////////////////////////////////////////////
void MSCAN_ListenInit() {

  // Force module into init mode via sleep mode (to ensure xmit/rcv complete
  SET_BITS( CANCTL0, CANCTL0_SLPRQ_MASK );		// put module into sleep mode
  while( BIT_IS_CLR( CANCTL1, CANCTL1_SLPAK_MASK ) );	// wait for module to fall asleep
  SET_BITS( CANCTL0, CANCTL0_INITRQ_MASK );		// put module into init mode
  while( BIT_IS_CLR( CANCTL1, CANCTL1_INITAK_MASK ) );	// wait for module to enter init mode

  // all done - leave init mode and enter normal mode
  CLR_BITS( CANCTL0, CANCTL0_INITRQ_MASK );		// turn off the init mode request flag
  while( BIT_IS_SET( CANCTL1, CANCTL1_INITAK_MASK ) );	// wait for module to exit init mode
  CLR_BITS( CANCTL0, CANCTL0_SLPRQ_MASK );		// turn off sleep mode flag
  while( BIT_IS_SET( CANCTL1, CANCTL1_SLPAK_MASK ) );	// wait for module to wake up
	
} // MSCAN_ListenForMsg()



/*//////////////////////////////////////////////////////////////////////////////////////////////// 
 *    MAIN Application Loop (Car Controller)
 *
 *      - Performs all Car Controller Functions
 *
 *////////////////////////////////////////////////////////////////////////////////////////////////
void main(void) {
  // put your own code here 
  
   int i = 0, k = 0;
   unsigned char newMessage = 0;
   unsigned char errorflag = -1;
   unsigned char txbuff[] = "ABCDEFGH";
   int check = 1;
   CANmsg_t thisMsg;
   CANInit();
   //CANCTL0 = 0x10;
   while (!(CANCTL0&0x10));
   CANRFLG = 0xC3;
   CANRIER = 0x01;
   EnableInterrupts;
   
   DDRA = 0xFF;
   PORTA = 0;
      
   errorflag = CANSendFrame((0x100), 0x00, sizeof(txbuff)-1, txbuff);   
  
   MSCAN_ListenInit();
  
   for(;;){
    
      if(MSCAN_GotMsg())  {   // check for new message
        if (check) {
          PORTA = 1;
          check = 0;
        } else {
          check = 1;
          PORTA = 0;
        }   
       
        newMessage = 1;
        MSCAN_Getd(&thisMsg);
        newMessage++;
      }   
   } 

}

/*//////////////////////////////////////////////////////////////////////////////////////////////// 
 *    CAN0RxISR  (NOT USED)
 *
 *     - ISR for receiving message 
 *
 *////////////////////////////////////////////////////////////////////////////////////////////////
void interrupt CAN0RxISR(void) {
 
 unsigned char length, index;
 unsigned char rxdata[8];
 length = (CANRXDLR & 0x0F);
 for (index=0; index<length; index++)
 rxdata[index] = *(&CANRXDSR0 + index); // Get received data 
 CANRFLG = 0x01; // Clear RXF  
  
}  
   