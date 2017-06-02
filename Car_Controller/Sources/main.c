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
#define ACC_CODE_ID101 0x2020
#define ACC_CODE_ID101_HIGH ((ACC_CODE_ID101 & 0xFF00)>>8)
#define ACC_CODE_ID101_LOW (ACC_CODE_ID101 & 0x00FF)  

#define ACC_CODE_ID100 0x2000
#define ACC_CODE_ID100_HIGH ((ACC_CODE_ID100 & 0xFF00)>>8)
#define ACC_CODE_ID100_LOW (ACC_CODE_ID100 & 0x00FF)  

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
 *    MAIN Application Loop (Car Controller)
 *
 *      - Performs all Car Controller Functions
 *
 *////////////////////////////////////////////////////////////////////////////////////////////////
void main(void) {

  
   int i = 0;
   int k = 0;
   int delay;
   int delay2;
   int floorSelected = 0;
   int pastPosition = 0;
   int doorState = 0;       //0 = CLOSED , 1 = OPEN
   int targetFloor = 0;
   unsigned char newMessage = 0;
   unsigned char errorflag = -1;
   unsigned char txbuff[] = "\x0";
   U8 CarPosition = 0;
   
   int check = 1;     
   CANmsg_t thisMsg = {0};

  //PORT SET UP FOR INPUTS 
  DDRT = 0x00;
 
  //PORTA SET UP FOR OUTPUTS
  DDRA = 0xFF;
  PORTA = 0;
  
  //BUILT IN BUTTONS CONFIGURED FOR INPUTS
  DDRJ = 0x00;
  
/* //TEST FOR FLASHING LEDS
  for(;;) 
  {
    if(PTT == 0b00000001)
      PORTA = 0b00000001;
    
    else if(PTT == 0b00000010)
      PORTA = 0b00000010;
    
    else if(PTT == 0b00000100)
      PORTA = 0b00000100;
    
    else if(PTJ == 0b01000000)
      PORTA = 0b00001000;
    
    else if(PTJ == 0b10000000)
      PORTA = 0b00010000;
  }   */
           
    
   CANInit();
   //CANCTL0 = 0x10;
   while (!(CANCTL0&0x10));
   CANRFLG = 0xC3;
   CANRIER = 0x01;
   //EnableInterrupts;
  
  // errorflag = CANSendFrame((0x100), 0x00, sizeof(txbuff)-1, txbuff);   
     MSCAN_ListenInit();
  
   PORTA = 0b00001000;
   
   //GET INITIAL CAR POSITION
   while(!MSCAN_GotMsg());
   MSCAN_Getd(&thisMsg);
   CarPosition = (thisMsg.data[0] & 0b00000011);  //STORE DATA INTO CarPosition
   pastPosition = CarPosition;
   
   for(;;){
   
     /***********************************************************
     *
     *      LISTENING TO ELEVATOR CONTROLLER
     *
     ***********************************************************/
     
    // LISTEN FOR MESSAGE
      if(MSCAN_GotMsg())  {   // check for new message
         
         MSCAN_Getd(&thisMsg);
   
         
         //IF MESSAGE IS FROM THE SUPERVISORY CONTROLLER
         if(thisMsg.id == 0x100){
           targetFloor = (thisMsg.data[0] & 0b00000011);
           
           if(CarPosition == targetFloor) {    
            PORTA &= 0b00000111;
            PORTA |= 0b00010000; //turn on open door led
            doorState = 1;
           }
         }
         
         
         
         //IF MESSAGE IS FROM THE ELEVATOR CONTROLLER
         if(thisMsg.id == 0x101){
          
         CarPosition = (thisMsg.data[0] & 0b00000011);  //STORE DATA INTO CarPosition 
           if(thisMsg.data[0] <= 2)
            PORTA |= (thisMsg.data[0] & 0b00011111);  
           else  //handling for floor 3 led
            PORTA |= ((thisMsg.data[0]+1) & 0b00011111);
           
           
         if(CarPosition == 1){
              PORTA |= 0b00000001;
              PORTA &= 0b00011001;
         }
         if(CarPosition == 2){
              PORTA |= 0b00000010;
              PORTA &= 0b00011010;
         }
         if(CarPosition == 3){
              PORTA |= 0b00000100;
              PORTA &= 0b00011100;
         }
         if(doorState == 0){
              PORTA |= 0b00001000;
              PORTA &= 0b00001111;
                       }
         if(doorState == 1){
              PORTA |= 0b00010000;
              PORTA &= 0b00010111;
         }
         
         //IF STATE CHANGE
         if(CarPosition != pastPosition) {
          
            //if car starts moving
             if(CarPosition == 0){
              PORTA &= 0b11101111; //turn off door open light
              PORTA |= 0b00001000; //turn on close door light 
              doorState = 0;
             }
             
             //car at floor 1
             if(CarPosition == 1){
              PORTA &= 0b00011001;
              
              //CHECK TARGET FOR DOOR OPEN  
              if(targetFloor == CarPosition){ 
                PORTA = 0b00010001;  //turn on floor 3 light
                doorState = 1;       //door flag OPEN 
                targetFloor = 0;
              }
             }
              
             if(CarPosition == 2){
              PORTA &= 0b00011010;
              
              //CHECK TARGET FOR DOOR OPEN  
              if(targetFloor == CarPosition){ 
                PORTA = 0b00010010;  //turn on floor 3 light
                doorState = 1;       //door flag OPEN 
                targetFloor = 0;
              }
             }
             
             if(CarPosition == 3){
                PORTA &= (0b00011100);
              
              //CHECK TARGET FOR DOOR OPEN  
              if(targetFloor == CarPosition){ 
                PORTA = 0b00010100;  //turn on floor 3 light
                doorState = 1;       //door flag OPEN 
                targetFloor = 0;
              }
             }
         }
         
         pastPosition = CarPosition;     
         
         } //end of receive from Elevator Controller
         
         
       
      }
      
     
     /***********************************************************
     *
     *      DOOR CLOSE BUTTON
     *
     ***********************************************************/
      
      //if elevator is not moving, door is closed, and floor is requested
      if(CarPosition && PTJ == 0b01000000) {
        
        //while ((PTJ &= 0b01000000)); //wait for button release
        for(delay=0;delay < 9999; delay++)
          for(delay2=0; delay2 < 4; delay2++);
        
        
        //DOOR CLOSE LIGHT ON
        PORTA &= 0b11101111;  //turn off door open light
        PORTA |= 0b00001000;
        
        doorState = 0; //set door flag to CLOSED
        
        //NEED TO EDIT TXBUFF WITH CORRECT DATA HERE
        if(floorSelected == 1)
          txbuff[0] = 0x5;
        else if(floorSelected == 2)
          txbuff[0] = 0x6;
        else if (floorSelected == 3)
          txbuff[0] = 0x7;
        else
          txbuff[0] = 0x4;
        
        
        //send floor request to supervisory        
        errorflag = CANSendFrame((0x200), 0x00, sizeof(txbuff)-1, txbuff);
        
        floorSelected = 0;  //set floor request flag off
         
      }
      
      
      
     /***********************************************************
     *
     *      DOOR OPEN BUTTON
     *
     ***********************************************************/
      //if car is NOT moving and door open button is pressed
      if(CarPosition && PTJ == 0b10000000) {
      
      //  while ((PTJ &= 0b10000000)); //wait for button release
        for(delay=0;delay < 9999; delay++)
          for(delay2=0;delay2 < 4; delay2++);
        
        //DOOR OPEN LIGHT ON
        PORTA = 0b00010000;
        doorState = 1;
        
        //send message to supervisory of door state open (NO FLOOR REQUEST)
         txbuff[0] = 0x0;
         errorflag = CANSendFrame((0x200), 0x00, sizeof(txbuff)-1, txbuff);
        
      }
      
      
     /***********************************************************
     *
     *      FLOOR 1 BUTTON PRESSED
     *
     ***********************************************************/
      //if car position is NOT moving and floor 1 button is pressed
      if(CarPosition && (PTT == 0b00000001)) {
        
        while ((PTT &= 0b00000001)); //wait for button release
        
        //FLOOR 1 REQUESTED
        txbuff[0] = 0x1;
       // PORTA = 0b00000001;
        floorSelected = 1;
        errorflag = CANSendFrame((0x200), 0x00, sizeof(txbuff)-1, txbuff);
 
      }
      
     /***********************************************************
     *
     *      FLOOR 2 BUTTON PRESSED
     *
     ***********************************************************/
      //if car position is NOT moving and floor 2 button is pressed
      if(CarPosition && (PTT == 0b00000010)) {
        
        while ((PTT &= 0b00000010)); //wait for button release
        
        //FLOOR 1 REQUESTED
        txbuff[0] = 0x2;
      //  PORTA = 0b00000010;
        floorSelected = 2;
        errorflag = CANSendFrame((0x200), 0x00, sizeof(txbuff)-1, txbuff);
 
      }
      
      /***********************************************************
     *
     *      FLOOR 3 BUTTON PRESSED
     *
     ***********************************************************/
      //if car position is NOT moving and floor 3 button is pressed
      if(CarPosition && (PTT == 0b00000100)) {
        
        while ((PTT &= 0b00000100)); //wait for button release
        
        //FLOOR 1 REQUESTED
        txbuff[0] = 0x3;
      //  PORTA = 0b00000100;
        floorSelected = 3;
        errorflag = CANSendFrame((0x200), 0x00, sizeof(txbuff)-1, txbuff);
 
      }
       
   /*    
       
//TOGGLE LED ON MESSAGE RECEIVE
        if (check) {
          PORTA = 0b00000001;
          check = 0;
        } else {
          check = 1;
          PORTA = 0; 
        }   
       
        newMessage = 1;
        MSCAN_Getd(&thisMsg);
        newMessage++;
      } 
      
     if(PTJ == 0b10000000){
      
      PORTA = 0b00010000;                
      errorflag = CANSendFrame((0x100), 0x00, sizeof(txbuff)-1, txbuff);
     } */
      
   }     
}



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
  
  CANIDAR0 = ACC_CODE_ID101_HIGH;   //|\ 16-bit Filter 0
  CANIDMR0 = MASK_CODE_ST_ID_HIGH;  //| \__ Accepts Standard Data Frame Msg
  CANIDAR1 = ACC_CODE_ID101_LOW;    //| / with ID 0x100
  CANIDMR1 = MASK_CODE_ST_ID_LOW;   //|/

  /* Acceptance Filters */
  CANIDAC = 0x10; /* Set four 16-bit Filters   */

  CANIDAR2 = ACC_CODE_ID100_HIGH;   //|\ 16-bit Filter 0
  CANIDMR2 = MASK_CODE_ST_ID_HIGH;  //| \__ Accepts Standard Data Frame Msg
  CANIDAR3 = ACC_CODE_ID100_LOW;    //| / with ID 0x200
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
   