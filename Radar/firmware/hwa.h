#ifndef MINIMAL_RADAR_STAGE_H
#define MINIMAL_RADAR_STAGE_H
typedef struct
{
    float x;
    float y;
    float vx;
    float vy;

} RadarObject_t;

void MinimalRadar_runStage(void);
int32_t CfarProc_configParser();
#endif