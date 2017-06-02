#include <stdio.h>
#include <unistd.h>
#include <asm/types.h>
#include <iostream>

#include "../Headers/PCANBasic.h"
#include "../Headers/candefinitions.h"

#define PCAN_DEVICE   PCAN_USBBUS1      //USB Port that PCAN dongle connected to
#define BAUD_RATE     PCAN_BAUD_125K    //CAN Bus baud rate
#define MY_ID         0x100             //ID for CAN messages
#define MESSAGE_LEN   1                 //Length of data portion of CAN message

//Message defines
#define SC_Enable             0b00000100
#define SC_Enable_False       0b00000000

#define SC_Floor_Cmd_None     0b00000000
#define SC_Floor_Cmd_F1       0b00000001
#define SC_Floor_Cmd_F2       0b00000010
#define SC_Floor_Cmd_F3       0b00000011

/* Elevator Controller Signals */
#define EC_Status_Disable     0b00000000
#define EC_Status_Enable      0b00000100

#define LOW_2BIT_MASK         0b00000011
#define EC_Car_Pos_Moving     0b00000000
#define EC_Car_Pos_F1         0b00000001
#define EC_Car_Pos_F2         0b00000010
#define EC_Car_Pos_F3         0b00000011

/* Car Controller signals */
#define BIT_2_MASK            0b00000100
#define CC_Door_State_Open    0b00000000
#define CC_Door_State_Closed  0b00000100

#define CC_Floor_Req_None     0b00000000
#define CC_Floor_Req_F1       0b00000001
#define CC_Floor_Req_F2       0b00000010
#define CC_Floor_Req_F3       0b00000011

/* Floor Node Signals */
#define F1_Call_Req_Open      0b00000000
#define F1_Call_Req_Request   0b00000001
#define F2_Call_Req_Open      0b00000000
#define F2_Call_Req_Request   0b00000001
#define F3_Call_Req_Open      0b00000000
#define F3_Call_Req_Request   0b00000001

/* Global Variables */
unsigned char FloorQueue[3];               /* Queue of floor requests        */
unsigned char FloorQueueIndex;             /* Points to next empty Q space   */
#define EMPTY           0                  /* in the FloorQueue array        */

unsigned char DoorState;                   /* Door state: 0 open, 1 closed   */
#define DOOR_OPEN       0                  /*   Possible Door States         */
#define DOOR_CLOSED     1
unsigned char CarPosition;                 /* Car Position                   */
unsigned char TargetPosition;              /* Target Car Positions           */
#define CAR_MOVING      0                  /*   Possible Car Positions       */
#define CAR_AT_FLOOR_1  1
#define CAR_AT_FLOOR_2  2
#define CAR_AT_FLOOR_3  3

unsigned char EC_En_Status;                /* Elevator Enable Status (EC):   */
#define EC_CAR_DISABLED 0                  /*   Elevator Car Enable Statuses */
#define EC_CAR_ENABLED  1
unsigned char SC_En_Status;                /* Elevator Enable Status:        */
#define SC_CAR_DISABLED 0                  /*   Supervisor Car Status        */
#define SC_CAR_ENABLED  1


void queueFloor(unsigned char floor);      /* Queue a floor request          */
unsigned char dequeueFloor(void);          /* Dequeue next floor request     */

int readCAN(TPCANMsg *mssg);               /* Read next message from CAN bus */
int writeCAN(TPCANMsg *mssg);              /* Write a message to the CAN bus */
void fillCANFrame(TPCANMsg *mssg,          /* Fill a CAN frame               */
                  char data);
int init(void);                            /* Initialize CAN and globals     */

/* State Machine Definitions */
int waiting_state(void);
int door_closed_state(void);
int moving_state(void);
int door_open_state(void);
int lookup_transitions(int current_state, int condition);

/* Array of to state functions of each state */
int (* state[])(void) = { waiting_state, door_closed_state, \
                          moving_state, door_open_state};

/* Enumeration of each state (for references) */
enum state_codes { waiting, door_closed, moving, door_open };

/* Enumeration of transition conditions (returns from state functions)*/
enum ret_codes { FloorArrival, DoorClosed, DequeueValid,
                 DequeueInvalid, DoorOpened, Repeat };

/* Structure definition for present-condition-next */
struct transition {
    enum state_codes  src_state;
    enum ret_codes    ret_code;
    enum state_codes  dst_state;
};

/* State Transition Table */
struct transition state_transitions[] {
/*  Present State     Condition/return    Next State      Transition Number */
    {waiting,         DoorOpened  ,       door_open},     // 1
    {waiting,         Repeat,             waiting},       // 2
    {door_open,       DoorClosed,         door_closed},   // 3
    {door_open,       Repeat,             door_open},     // 4
    {door_closed,     DequeueInvalid,     waiting},       // 5
    {door_closed,     DequeueValid,       moving},        // 6
    {waiting,         DequeueValid,       moving},        // 7
    {moving,          Repeat,             moving},        // 8
    {moving,          FloorArrival,       door_open}      // 9
};

#define ENTRY_STATE waiting

/* main application: Main entry-point
 */
int main(int argc, char* argv[]){
    TPCANMsg Message;                           /* Message Container */

    int cur_state = ENTRY_STATE;                /* State tracking variable */
    int prev_state = moving;
    int condition;                              /* Condition variable      */
    int (* state_fun)(void);                    /* State function pointer   */

    if (init()) {
        /* Initialization failed */
        printf("Exiting...\n");
        return(0);
    }

    /* Main application loop */
    for(;;) {
      if (cur_state != prev_state){
        prev_state = cur_state;
        switch (cur_state) {
            case waiting:
                printf("\nIn Waiting State...\n\n");
                break;
            case door_open:
                printf("\nIn Open Door State...\n\n");
                break;
            case door_closed:
                printf("\nIn Closed Door State...\n\n");
                break;
            case moving:
                printf("\nIn Moving State...\n\n");
                break;
        }
      }

      /* Check for new CAN messages */
      if (readCAN(&Message))
          printf("CAN read failed.\n");

      fflush(stdout);

      /* Move through state machine */

      state_fun = state[cur_state];
      condition = state_fun();
      cur_state = lookup_transitions(cur_state, condition);


    }
}

/* int lookup_transitions(int current_state, int condition)
 *    This function cycles through the state_transitions table to determine
 *    the next state.  The return value is the next state.
 */
int lookup_transitions(int current_state, int condition) {
      int index, index2;
      for (index = 0 ; index < 9 ; index++) {
          if (state_transitions[index].src_state == current_state) {
            if (state_transitions[index].ret_code == condition)
              return((int)state_transitions[index].dst_state);
          }
      }
      /* Execution should never reach this point */
      printf("Error in 'lookup_transitions' function: table access invalid\n");
      return(0);
}

/* int init(void)
 *    Initialize CAN Bus interface and global variables.
 *
 *    Returns 0 on success, else 1
 */
int init(void) {
    int Status = 0;
    /* Initialize CAN Interface */
    Status = CAN_Initialize(PCAN_DEVICE, BAUD_RATE, 0, 0, 0);
    if ((int)Status) {
        printf("CAN Intialization failed: %i", (int)Status);
        return (1);
    }

    /* Initialize global variables */
    FloorQueueIndex = EMPTY;
    DoorState = DOOR_CLOSED;
    CarPosition = CAR_AT_FLOOR_1;
    TargetPosition = CAR_AT_FLOOR_1;
    EC_En_Status = EC_CAR_ENABLED;
    SC_En_Status = SC_CAR_ENABLED;
    FloorQueue[0] = 0;
    FloorQueue[1] = 0;
    FloorQueue[2] = 0;

    return (0);
}

/* int waiting_state(void)
 *    This is the function executed while the state machine is in Waiting state
 *    The return value is the condition that will affect the next transition.
 */
int waiting_state(void){
    if (FloorQueueIndex != EMPTY) {
        /* There is a floor request in the Queue */
        unsigned char nextFloor = dequeueFloor();
        TPCANMsg mssg;
        fillCANFrame(&mssg, (SC_Enable | nextFloor));
        if (writeCAN(&mssg))
            printf("CAN write failed.\n");
        TargetPosition = nextFloor;
        CarPosition = CAR_MOVING;
        return (DequeueValid);
    }

    if (DoorState == DOOR_OPEN) {
        /* Door was opened */
        return (DoorOpened);
    }

    if (FloorQueueIndex == EMPTY && DoorState == DOOR_CLOSED) {
        /* Take no action */
        return (Repeat);
    }

    /* Should never reach this point */
    printf("'waiting_state' state function error: unable to process. \
             State machine is lost.\n");

    return(0);
}

/* int door_closed_state(void)
 *    This is the function executed while the state machine is in Closed Door
 *    state.
 *    The return value is the condition that will affect the next transition.
 */
int door_closed_state(void) {
    if (FloorQueueIndex != EMPTY) {
        /* There is a floor request in the Queue */
        unsigned char nextFloor = dequeueFloor();
        TPCANMsg mssg;
        fillCANFrame(&mssg, (SC_Enable | nextFloor));
        if (writeCAN(&mssg))
            printf("CAN write failed.\n");
        TargetPosition = nextFloor;
        CarPosition = CAR_MOVING;
        return (DequeueValid);
    }

    if (FloorQueueIndex == 0) { /* No floor request in the queue */
        return (DequeueInvalid);
    }

    /* Should never reach this point */
    printf("'door_state_closed' state function error: unable to process. \
             State machine is lost.\n");

    return(0);
}

/* int moving_state(void)
 *    This is the function executed while the state machine is in Moving state.
 *    The return value is the condition that will affect the next transition.
 */
int moving_state(void) {
    if (CarPosition == CAR_MOVING) {
        /* Take no action */
        return (Repeat);
    }

    if (CarPosition != CAR_MOVING && CarPosition != TargetPosition) {
        /* Take no action - we are at a floor but not the desired floor */
        return (Repeat);
    }

    if (CarPosition == TargetPosition) {
        /* We have arrived at our destination */
        DoorState = DOOR_OPEN;
        return (FloorArrival);
    }

    printf("'moving_state' state function error: unable to process. \
             State machine is lost\n");

    return(0);
}

/* int door_open_state(void)
 *    This is the function executed while the state machine is in Door Opened
 *    state.
 *    The return value is the condition that will affect the next transition.
 */
int door_open_state(void) {
    if (DoorState == DOOR_OPEN) {
        /* Take no action */
        return(Repeat);
    }

    if (DoorState == DOOR_CLOSED) {
        DoorState = DOOR_CLOSED;
        return(DoorClosed);
    }

    printf("'door_open_state' state function error: unable to process. \
             State machine is lost\n");

    return(0);
}

/* void parseCANFrame(TPCANMsg *mssg)
 *    Parse a CAN message.  Update flags as necessary for use in state machine.
 */
 void parseCANFrame(TPCANMsg *mssg) {
    unsigned char data = (unsigned char)mssg->DATA[0];

    switch(mssg->ID){
      case F1:
          printf("  Car requested from Floor 1...");
          queueFloor(1);
          break;
      case F2:
          printf("  Car requested from Floor 2...");
          queueFloor(2);
          break;
      case F3:
          printf("  Car requested from Floor 3...");
          queueFloor(3);
          break;
      case CC:
          {
            if (data & CC_Floor_Req_None) {/* Do nothing */}
            else if ((data & LOW_2BIT_MASK) == CC_Floor_Req_F1) {
                printf("  Floor 1 requested from Car Controller...");
                queueFloor(1);
            }
            else if ((data & LOW_2BIT_MASK) == CC_Floor_Req_F2) {
                printf("  Floor 2 requested from Car Controller...");
                queueFloor(2);
            }
            else if ((data & LOW_2BIT_MASK) == CC_Floor_Req_F3) {
                printf("  Floor 3 requested from Car Controller...");
                queueFloor(3);
            }

            if ((data & BIT_2_MASK) == CC_Door_State_Open) {
                printf("  Car controller updated door state (OPEN)\n");
                DoorState = DOOR_OPEN;
            }
            else if ((data & BIT_2_MASK) == CC_Door_State_Closed) {
                printf("  Car controller updated door state (CLOSED)\n");
                DoorState = DOOR_CLOSED;
            }
          }
          break;
      case EC:
          {
            /* Not using EC Enable/Disable
            if ((data & BIT_2_MASK) == EC_Status_Enable) {
                printf("  Elevator Controller updated status (ENABLED)\n");
                EC_En_Status = EC_Status_Enable;
            }
            else if ((data & BIT_2_MASK) == EC_Status_Disable) {
                printf("  Elevator Controller updated status (DISASBLED)\n");
                EC_En_Status = EC_Status_Disable;
            } */

            if ((data & LOW_2BIT_MASK) == EC_Car_Pos_Moving) {
                printf("  Elevator Controller updated Car Position (MOVING)\n");
                CarPosition = CAR_MOVING;
            }
            else if ((data & LOW_2BIT_MASK) == EC_Car_Pos_F1) {
                printf("  Elevator Controller updated Car Position (FLOOR 1)\n");
                CarPosition = CAR_AT_FLOOR_1;
            }
            else if ((data & LOW_2BIT_MASK) == EC_Car_Pos_F2) {
                printf("  Elevator Controller updated Car Position (FLOOR 2)\n");
                CarPosition = CAR_AT_FLOOR_2;
            }
            else if ((data & LOW_2BIT_MASK) == EC_Car_Pos_F3) {
                printf("  Elevator Controller updated Car Position (FLOOR 3)\n");
                CarPosition = CAR_AT_FLOOR_3;
            }
          }
          break;
      default:
          printf("  Error: parsing failed. ID not recognized\n");
          break;
    }
}

/* void queueFloor(unsigned char floor)
 *    Add item floor (function argument) to the queue.  Does not add floor to
 *    the queue if the queue is full, the requested floor is already
 *    present in the queue, or if the floor request is invalid
 */
void queueFloor(unsigned char floor) {
    if (floor < 1 || floor > 3) {         /* Invalid floor request */
        printf("rejected: Invlaid floor request\n");
        return;
    }
    if (FloorQueueIndex == 3) {           /* Ignore request if Queue is full */
        printf("rejected: Queue is full\n");
        return;
    }
    if (FloorQueue[0] == floor  ||         /* Ignore duplicate requests & */
        FloorQueue[1] == floor  ||         /* ignore request if we are in */
        TargetPosition == floor) {          /* transit to requested floor  */
        printf("rejected: Floor already queued.\n");
        return;
    }
    printf("accepted: Floor has been queued.\n");
    FloorQueue[FloorQueueIndex++] = floor;

}

/* unsigned char dequeueFloor(void)
 *    Return the next item in the queue.  Returns 0 if no items in the queue.
 */
unsigned char dequeueFloor(void) {
    unsigned char nextFloor = 0;

    if (FloorQueueIndex == 0) return 0;   /* There are no items in the queue */

    nextFloor = FloorQueue[--FloorQueueIndex];
    FloorQueue[FloorQueueIndex] = 0;

    return (nextFloor);
}

/* int readCAN(TPCANMsg *mssg)
 *    Read a message from the CAN Bus into the TPCANMsg pointed to by mssg
 *
 *    Returns 1 upon failure, else 0
 */
int readCAN(TPCANMsg *mssg) {
    TPCANStatus Status;
    int check = 1;
  //  while (Status != PCAN_ERROR_QRCVEMPTY) {
        Status=CAN_Read(PCAN_DEVICE, mssg, NULL);

        if (Status == PCAN_ERROR_QRCVEMPTY) { return 0; /* break; */ }     /* Receive queue empty */
        if (Status != PCAN_ERROR_OK) {                 /* Error occured       */
            printf("Error 0x%x\n", (int)Status);
            return 1;
        }

        if ((int)mssg->LEN != 1) return 0; /* continue; */             /* Message invalid     */

        /* Use for debugging
        printf("Received: ID:%4x LEN:%1x DATA:%02x\n",
            (int)mssg->ID, (int)mssg->LEN, (int)mssg->DATA[0]);

        printf("Parsing message for State Machine...\n");
        */

        parseCANFrame(mssg);
    //}

    return 0;
}

/* int writeCAN(TPCANMsg *mssg)
 *    Write a message to the CAN Bus into the TPCANMsg pointed to by mssg
 *
 *    Returns 1 upon failure, else 0 *
 */
int writeCAN(TPCANMsg *mssg) {
    TPCANStatus Status;
    unsigned long ulIndex = 0;

    if ((Status=CAN_Write(PCAN_DEVICE, mssg)) != PCAN_ERROR_OK){
        printf("Error!");
        //mssg->DATA[0]++;
        //ulIndex++;
        //if ((ulIndex & 1000) == 0)
            //printf("  - T Message %i\n", (int)ulIndex);
    }

}
/* int fillCANFrame(TPCANMsg *mssg, char data)
 *    Fill the CAN frame pointed to by mssg (first argument).
 *    All paramaters are  set as defines except for
 *    the actual message data, data (second argument)
 */
void fillCANFrame(TPCANMsg *mssg, char data) {
    mssg->ID = MY_ID;
    mssg->LEN = MESSAGE_LEN;
    mssg->MSGTYPE = PCAN_MESSAGE_STANDARD;
    mssg->DATA[0]=data;
}
