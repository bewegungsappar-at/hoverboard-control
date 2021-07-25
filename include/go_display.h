#ifndef GO_DISPLAY_H
#define GO_DISPLAY_H

#include "config.h"

#ifdef ODROID_GO_HW
#include <odroid_go.h>


namespace GO_DISPLAY
{
    typedef enum
    {
        CURRENT_LEFT,
        CURRENT_RIGHT,
        PWM_LEFT,
        PWM_RIGHT,
        SPEED,
        STEER,
        PACKAGE_LOSS_UPSTREAM,
        PACKAGE_LOSS_DOWNSTREAM,
        BATTERY_VOLTAGE
    } field_value_t;

    typedef enum
    {
        MENU_SENSOR,
        MENU_PWM,
        MENU_ENABLE,
        MENU_DISABLE,
        MENU_WIRESHARKDEC,
        MENU_WIRESHARKINC,
        MENU_WRITEFLASH,
        MENU_PADDLEREAD,
        MENU_PADDLEDRAGINC,
        MENU_PADDLEDRAGDEC,
        MENU_PADDLEMULTINC,
        MENU_PADDLEMULTDEC,
        MENU_PADDLESPEEDINC,
        MENU_PADDLESPEEDDEC,
        MENU_PWM_VOLLGAS,
        MENU_PWM_HALBGAS,
        MENU_IDLE
    } menu_result;

    void setup(void);
    void set(field_value_t field, float value);
    void update(void);
    void show_internal_battery_voltage();
    void show_labels();
    void connectionSelector();
    void plot(double value);
    void plotBattery(double value);
    void plotSpeed(double value);
    menu_result menu(bool init);
}

#endif /* ODROID_GO_HW */


#endif /* GO_DISPLAY_H */