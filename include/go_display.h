#ifndef GO_DISPLAY_H
#define GO_DISPLAY_H

#include "config.h"

#ifdef ODROID_GO_HW
#include <odroid_go.h>

/*
phail monitor
L current   | R current  "
 1234 mA    |  4321 mA   "
            |            "
L PWM       | R PWM      "
 2345 uS    |  1200 uS   "
            |            "
speed       | steer      "
 666 %      | 000 %      "
            |            "
package loss|            "
U 98765     | D 99999    "
            |            "
            |            "
            |            "
battery             0000V"
*/

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

    void setup(void);
    void set(field_value_t field, float value);
    void update(void);

    void plot(double value);

}

#endif /* ODROID_GO_HW */


#endif /* GO_DISPLAY_H */