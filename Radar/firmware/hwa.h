#ifndef MINIMAL_RADAR_STAGE_H
#define MINIMAL_RADAR_STAGE_H
typedef struct
{
    float x;
    float y;
    float vx;
    float vy;

} RadarObject_t;
typedef enum
    {
        TTC_RESULT_SAFE = 0,
        TTC_RESULT_WARNING,
        TTC_RESULT_CRITICAL

    } TTC_Result_t;

void MinimalRadar_runStage(void);
int32_t CfarProc_configParser();
#endif