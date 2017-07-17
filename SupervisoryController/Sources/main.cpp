#include <stdio.h>
#include <unistd.h>
#include <asm/types.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <ctime>
#include <time.h>

#include "../Headers/PCANBasic.h"
#include "../Headers/candefinitions.h"
#include "../Headers/elevator_statemachine.h"

/* MySQL Includes */
#include <mysql_connection.h>

#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>

using namespace std;

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
#define EMPTY           0
#define NUM_FLOORS      3                  /* Number of floors in the system */
#define QUEUE_LEN       NUM_FLOORS         /* Q-Len is equal to num floors   */
unsigned char FloorQueue[QUEUE_LEN];       /* Queue of floor requests        */
unsigned char FloorQueueIndex;             /* Points to next empty Q space   */
                                           /* in the FloorQueue array */


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
#define TRANSITION_ENTRIES  9                   /* Nine Entries in the state
                                                   transition table */

#define ENTRY_STATE waiting                     /* Initial state is waiting */

sql::Connection *con;
sql::Driver *driver;

int connectMySQL() {
    driver = get_driver_instance();
    con = driver->connect("tcp://127.0.0.1:3306", "root", "");
    con->setSchema("elevator");

    cout << "Connecting to mysql database..." << endl;

    if (con->isValid()) {
        cout << "Connection is valid" << endl;
        return(1);
    }
    else {
        cout << "Failed to establish valid connection" << endl;
        return(0);
    }
}

int dequeueMySQL_clientQueue() {
    sql::Statement *stmt;
    sql::ResultSet *res;

    int nextAction = 0;

    stmt = con->createStatement();
    //stmt->execute("INSERT INTO clientQueue (activeID) VALUES (200)");
    res = stmt->executeQuery("SELECT activeID FROM clientQueue");
    if (res->next()) {
        printf("ActiveID: %d", res->getInt(1));
        nextAction = res->getInt(1);
        char query[100] = {0};
        char number[10] = {0};
        strcpy(query, "DELETE FROM clientQueue WHERE activeID=");
        sprintf(number, "%d", nextAction);
        strcat(query, number);
        strcat(query, ";");
        stmt->execute(query);
        switch (nextAction) {
            case 1:
            case 2:
            case 3:
                queueFloor(nextAction);
                break;
            case 4:
                queueFloor(1);
                break;
            case 5:
                queueFloor(2);
                break;
            case 6:
                queueFloor(3);
                break;
            case 7:
                if (DoorState == DOOR_CLOSED && CarPosition != CAR_MOVING){
                    DoorState = DOOR_OPEN;
                } else {
                    /* Need to immediately set this request to zero in
                       James' clientRequests table as the request is invalid */
                }
                break;
            case 8:
                DoorState = DOOR_CLOSED;
                if (DoorState == DOOR_OPEN) {
                    DoorState = DOOR_CLOSED;
                } else {
                    /* Need to immediately set this request to zero in
                       James' clientRequests table as the request is invalid */
                }
                break;
            default:
                printf("Received an invalid request from the client side FIFO queue");
                fflush(stdout);
                break;
        }
    }

    delete stmt;
    delete res;

}

int updateMySQL_currentState() {
    sql::Statement *stmt;
    sql::Statement *stmt2;

    char query[1000] = {0};
    char number[10] = {0};

    stmt = con->createStatement();
    //stmt->execute("INSERT INTO clientQueue (activeID) VALUES (200)");
    stmt->execute("TRUNCATE TABLE currentState");

    strcat(query, "INSERT INTO currentState (currentPosition, doorState) VALUES (");

    sprintf(number, "%d", CarPosition);

    strcat(query, number);
    strcat(query, ",");

    sprintf(number, "%d", DoorState);
    strcat(query, number);
    strcat(query, ");");

    puts("\n");
    puts(query);

    stmt2 = con->createStatement();
    stmt2->execute(query);

    delete stmt;
    delete stmt2;

    return(0);

}

void updateMySQL_CANLog(char signal[]) {

    sql::Statement *stmt;

    time_t rawtime;
    struct tm * timeinfo;
    char query[1000] = {0};
    char date [15] = {0};
    char time_s [15] = {0};

    time (&rawtime);
    timeinfo = localtime (&rawtime);

    /* Encode the query */
    strcat(query, "INSERT INTO CANLog (date, time, floorQueue, carPosition, targetPosition, doorState, signalID) VALUES (");

    /* Date */
    strftime (date,15,"\"%F\",",timeinfo);
    strcat(query, date);

    /* Time */
    strftime (time_s,15,"\"%T\",",timeinfo);
    strcat(query, time_s);

    /* Floor Queue */
    char floor[3] = {0};
    if (FloorQueueIndex == EMPTY) {
        strcat(query, "\"EMPTY\",");
    } else {
        strcat(query, "\"");
        for (int i = 0; i < FloorQueueIndex ; i++) {
            floor[0] = FloorQueue[i] + 48;

            if (i != FloorQueueIndex - 1)
                floor[1] = ',';
            else
                floor[1] = '\0';

            strcat(query, floor);
        }
        strcat(query, "\",");
    }

    /* Car Position */
    switch (CarPosition) {
        case CAR_MOVING:
            strcat(query, "\"MOVING\",");
            break;
        case CAR_AT_FLOOR_1:
            strcat(query, "\"FLOOR 1\",");
            break;
        case CAR_AT_FLOOR_2:
            strcat(query, "\"FLOOR 2\",");
            break;
        case CAR_AT_FLOOR_3:
            strcat(query, "\"FLOOR 3\",");
            break;
        default:
            strcat(query, "\"NULL\",");
            break;
    }

    /* Target Position */
    switch (TargetPosition) {
        case CAR_AT_FLOOR_1:
            strcat(query, "\"FLOOR 1\",");
            break;
        case CAR_AT_FLOOR_2:
            strcat(query, "\"FLOOR 2\",");
            break;
        case CAR_AT_FLOOR_3:
            strcat(query, "\"FLOOR 3\",");
            break;
        default:
            strcat(query, "\"NULL\",");
    }

    /* Door State */
    if (DoorState = DOOR_OPEN)
        strcat(query, "\"OPEN\",");
    else
        strcat(query, "\"CLOSED\",");

    /* Signal ID */
    strcat(query, "\"");
    strcat(query, signal);

    strcat(query, "\");");

    puts("\n");

    puts(query);

    stmt = con->createStatement();
    stmt->execute(query);

    delete stmt;
}


/* main application: Main entry-point
 */
int main(int argc, char* argv[]){
    TPCANMsg Message;                           /* Message Container */
    char what[10];

    int cur_state = ENTRY_STATE;                /* State tracking variable */
    int prev_state = moving;
    int condition;                              /* Condition variable      */
    int (* state_fun)(void);                    /* State function pointer   */



    if(!connectMySQL()){
        printf("Exiting...\n");
        return(0);
    }

    updateMySQL_currentState();

    //updateMySQL_CANLog("0");

    if (init()) {
        /* Initialization failed */
        printf("Exiting...\n");
        return(0);
    }

    printf("Entering main application loop");
    fflush(stdout);

    /* Main application loop */
    for(;;) {
      if (cur_state != prev_state){
        prev_state = cur_state;
        updateMySQL_currentState();
        updateMySQL_CANLog("nothing");
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

      /* Check queue from client (database) */
      dequeueMySQL_clientQueue();

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
      int index;
      for (index = 0 ; index < TRANSITION_ENTRIES ; index++) {
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
        printf("CAN Intialization failed. Error code:%i\n", (int)Status);
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
        printf("Dequeuing Floor: %d\n", nextFloor);
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
    char signalBuff[20] = {0};

    switch(mssg->ID){
      case F1:
          printf("  Car requested from Floor 1...");
          strcpy(signalBuff, "201");
          queueFloor(1);
          break;
      case F2:
          printf("  Car requested from Floor 2...");
          strcpy(signalBuff, "202");
          queueFloor(2);
          break;
      case F3:
          printf("  Car requested from Floor 3...");
          strcpy(signalBuff, "203");
          queueFloor(3);
          break;
      case CC:
          strcpy(signalBuff, "200");
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
                if (DoorState != DOOR_OPEN)
                    printf("  Car controller updated door state (OPEN)\n");
                DoorState = DOOR_OPEN;
            }
            else if ((data & BIT_2_MASK) == CC_Door_State_Closed) {
                if (DoorState != DOOR_CLOSED)
                    printf("  Car controller updated door state (CLOSED)\n");
                DoorState = DOOR_CLOSED;
            }
          }
          break;
      case EC:
          strcpy(signalBuff, "101");
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
                //if (CarPosition != CAR_MOVING)
                    printf("  Elevator Controller updated Car Position (MOVING)\n");
                CarPosition = CAR_MOVING;
            }
            else if ((data & LOW_2BIT_MASK) == EC_Car_Pos_F1) {
                if (CarPosition != CAR_AT_FLOOR_1)
                    printf("  Elevator Controller updated Car Position (FLOOR 1)\n");
                CarPosition = CAR_AT_FLOOR_1;
            }
            else if ((data & LOW_2BIT_MASK) == EC_Car_Pos_F2) {
                if (CarPosition != CAR_AT_FLOOR_2)
                    printf("  Elevator Controller updated Car Position (FLOOR 2)\n");
                CarPosition = CAR_AT_FLOOR_2;
            }
            else if ((data & LOW_2BIT_MASK) == EC_Car_Pos_F3) {
                if (CarPosition != CAR_AT_FLOOR_3)
                    printf("  Elevator Controller updated Car Position (FLOOR 3)\n");
                CarPosition = CAR_AT_FLOOR_3;
            }
          }
          break;
      default:
          printf("  Error: parsing failed. ID not recognized\n");
          break;
    }
    //updateMySQL_CANLog(signalBuff);
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
    if (FloorQueue[0] == floor  ||         /* Ignore duplicate requests & if */
        FloorQueue[1] == floor) {          /* in transit to requested floor  */
        printf("rejected: Floor already queued.\n");
        return;
    }
    if (TargetPosition == floor) {
        DoorState = DOOR_OPEN;
    }
    printf("accepted: Floor has been queued.\n");
    FloorQueue[FloorQueueIndex++] = floor;

}

/* unsigned char dequeueFloor(void)
 *    Return the next item in the queue.  Returns 0 if no items in the queue.
 */
unsigned char dequeueFloor(void) {
    unsigned char nextFloor = 0;
    unsigned char i;

    if (FloorQueueIndex == 0) {
        printf("No floors requested.  Waiting...\n");
        return 0;   /* There are no items in the queue */
    }

    nextFloor = FloorQueue[0];            /* Dequeue from front of queue */

    for (i = 0 ; i < (QUEUE_LEN-1) && (FloorQueue[i]) ; i++){
        FloorQueue[i] = FloorQueue[i+1];  /* Fix the queue */
    }

    FloorQueue[QUEUE_LEN-1] = 0;          /* Make sure last element is empty */

    FloorQueueIndex--;                    /* Decrement queue index */

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

    Status=CAN_Read(PCAN_DEVICE, mssg, NULL);

    if (Status == PCAN_ERROR_QRCVEMPTY) {  /* Receive queue empty */
        return 0; /* break; */
    }
    if (Status != PCAN_ERROR_OK) {         /* Error occured       */
        printf("Error 0x%x\n", (int)Status);
        return 1;
    }

    if ((int)mssg->LEN != 1)               /* Message invalid     */
        return 0; /* continue; */

    parseCANFrame(mssg);

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

    updateMySQL_CANLog("100");

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
