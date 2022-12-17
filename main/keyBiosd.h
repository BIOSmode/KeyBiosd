#ifndef __KEYBIOSD_H__
#define __KEYBIOSD_H__

/************ MACRO AND STUCTURE **************/
typedef struct
{
    uint8_t gpioPin;
    bool gpioSts;
    uint8_t hidKey;
    bool isShift;
} biosd_button_info_t;


/********* Application ***************/
void exp_send_hid_demo(void);
void ven_send_hid_demo(void);
void ven_OneTimeInit(void);
void ven_Runner(void);


#endif