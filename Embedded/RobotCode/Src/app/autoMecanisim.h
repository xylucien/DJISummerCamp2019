#ifndef AUTOMECANISIM_H
#define AUTOMECANISIM_H

#define ARM_RESTING_ANGLE 0.0f
#define ARM_CUP_UP_ANGLE 0.0f
#define ARM_GET_CUP_ANGLE -75.0f

#define CUP_THINGIE_RESTING_ANGLE 0.0f
#define CUP_THINGIE_READY_ANGLE 0.0f

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
    GRABING_CUP = 0,
    PLACE_CUP = 1,
    REST = 2
} AutoMecanisimGoal;

typedef enum {
    PREPARE_POSITION = 0,
    GRAB_CUP = 1,
    LIFT_CUP = 2
} CupGrabbingSteps;

extern void initAutoMecanisim(void);
extern void updateAutoMecanisim(void *arguments);
#endif
