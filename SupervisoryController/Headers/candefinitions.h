#ifndef CANDEFINITIONS
#define CANDEFINITIONS

/* FloorQueueIndex Empty Value */
#define EMPTY           0                  /* in the FloorQueue array        */
//
/* Door state: 0 open, 1 closed   */
//#define DOOR_OPEN       0                  /*   Possible Door States         */
//#define DOOR_CLOSED     1

/* Car positions for use with global variables: CarPosition, TargetPosition  */
//#define CAR_MOVING      0
//#define CAR_AT_FLOOR_1  1
//#define CAR_AT_FLOOR_2  2
//#define CAR_AT_FLOOR_3  3

/* Elevator Enable Status for use with global variable EC_En_status */
//#define EC_CAR_DISABLED 0
//#define EC_CAR_ENABLED  1
//unsigned char SC_En_Status;                /* Elevator Enable Status for use with global variable       */
//#define SC_CAR_DISABLED 0                  /*   Supervisor Car Status        */
//#define SC_CAR_ENABLED  1


// Node IDs
#define CC   0x200         // Car Controller
#define SC   0x100         // Supervisory Controller
#define EC   0x101         // Elevator Controller
#define F1   0x201         // Floor 1 Controller
#define F2   0x202         // Floor 2 Controller
#define F3   0x203         // Floor 3 Controller

#endif /* CANDEFINITIONS */
