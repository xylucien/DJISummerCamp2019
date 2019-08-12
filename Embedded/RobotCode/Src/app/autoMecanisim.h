#ifndef AUTOMECANISIM_H
#define AUTOMECANISIM_H

#define ARM_RESTING_ANGLE 0.0f
#define ARM_CUP_UP_ANGLE 0.0f
#define ARM_GET_CUP_ANGLE -90.0f

#define CUP_THINGIE_RESTING_ANGLE 1100.0f
//#define CUP_THINGIE_READY_ANGLE_10 1850.0f
#define CUP_THINGIE_READY_ANGLE_5 1400.0f
//#define CUP_THINGIE_READY_ANGLE_5 1100.0f
#define CUP_THINGIE_READY_ANGLE_10 1375.0f //2030
#define CUP_THINGIE_GET_BALL 1000.0f

//#define CUP_THINGIE_GET_BALL 1470.0f

//#define CUP_THINGIE_RESTING_ANGLE 2000.0f
//#define CUP_THINGIE_READY_ANGLE_10 2000.0f
//#define CUP_THINGIE_READY_ANGLE_5 2100.0f
//#define CUP_THINGIE_GET_BALL 2000.0f

#define CLAW_READY_SET_POINT 2195.0f
#define CLAW_RELEASE_SET_POINT 2220.0f
#define CLAW_GRAB_SET_POINT 2000.0f

#define scrollWheelUpTreshold 300.0f
#define scrollWheelDownTreshold -300.0f

/*
typedef enum {
    GETTING_READY_FOR_CUP = 0,
    LOWERING_CUP = 1,
    GRABBING_CUP = 2,
    LIFTING_CUP = 3,
    NEUTRAL_POSITION = 4,
    PLACEING_CUP = 5
} AutoMecanisimStatus;
*/

typedef enum {
    GRABING_CUP = 1,
    PLACE_CUP = 2,
    REST = 3
} AutoMecanisimGoal;

typedef enum {
		DROPPING_BALL = 1,
    PREPARE_POSITION = 2,
    GRAB_CUP = 3,
    LIFT_CUP = 4
} CupGrabbingSteps;

extern void initAutoMecanisim(void);
extern void updateAutoMecanisim(void *arguments);
#endif
