#include "config.h"
#include "go_display.h"

#ifdef ODROID_GO_HW


#include <driver/adc.h>
#include <esp_adc_cal.h>

namespace GO_DISPLAY
{
    // forward declaration
    static void show_labels(void);
    static void show_internal_battery_voltage(void);
    static void setup_internal_battery_adc(void);
    static float get_internal_battery_voltage(void);
    static esp_adc_cal_characteristics_t _adc_chars;
    static uint16_t backgroundColor;


    // impl
    void setup(void)
    {
        GO.begin();
        GO.lcd.setTextSize(2);
        GO.lcd.clearDisplay();
        GO_DISPLAY::show_labels();
        GO_DISPLAY::setup_internal_battery_adc();
        GO_DISPLAY::show_internal_battery_voltage();
    }
    void set(field_value_t field, float value)
    {
        // TODO: just for now update V_bat everytime one set a value.
        GO_DISPLAY::show_internal_battery_voltage();

        uint8_t x=0,y=0;
        switch (field)
        {
        case CURRENT_LEFT:
            x = 1;
            y = 2;
            break;
        case CURRENT_RIGHT:
            x = 15;
            y = 2;
            break;
        case PWM_LEFT:
            x = 2;
            y = 5;
            break;
        case PWM_RIGHT:
            x = 15;
            y = 5;
            break;
        case SPEED:
            x = 2;
            y = 8;
            break;
        case STEER:
            x = 15;
            y = 8;
            break;
        case PACKAGE_LOSS_UPSTREAM:
            x = 2;
            y = 11;
            break;
        case PACKAGE_LOSS_DOWNSTREAM:
            x = 15;
            y = 11;
            break;
        case BATTERY_VOLTAGE:
            x = 5;
            y = 14;
            break;
        default:
            break;
        }
        if(0 != x && 0 != y)
        {
            GO.lcd.setCharCursor(x,y);
            GO.lcd.print(value);
        }
    }

    void show_labels()
    {
        GO.lcd.clearDisplay();
        GO.lcd.setCharCursor(0, 0);
        GO.lcd.setTextColor(TFT_MAROON, backgroundColor);
        GO.lcd.println("phail monitor  V_bat:");
        GO.lcd.setTextColor(TFT_GREEN, backgroundColor);
        GO.lcd.print("L current   | R current");
        GO.lcd.setCharCursor(0, 4);
        GO.lcd.print("L PWM       | R PWM");
        GO.lcd.setCharCursor(0, 7);
        GO.lcd.print("speed       | steer");
        GO.lcd.setCharCursor(0, 10);
        GO.lcd.print("package loss|");
        GO.lcd.setCharCursor(0, 13);
        GO.lcd.print("hoverboard battery in Volt");
    }

    void show_internal_battery_voltage()
    {
        GO.lcd.setTextColor(TFT_MAROON, backgroundColor);
        GO.lcd.setCharCursor(21,0);
        GO.lcd.print(get_internal_battery_voltage());
        GO.lcd.setTextColor(TFT_GREEN, backgroundColor);
    }

    void setup_internal_battery_adc()
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, ESP_ADC_CAL_VAL_EFUSE_TP, &_adc_chars);
    }


    float get_internal_battery_voltage()
    {
        static const int BATTERY_SAMPLES = 64;
        static const int BATTERY_RESISTANCE_NUM = 2;
        uint32_t adc_reading = 0;
        for (int i = 0; i < BATTERY_SAMPLES; i++)
        {
            adc_reading += adc1_get_raw((adc1_channel_t) ADC1_CHANNEL_0);
        }
        adc_reading /= BATTERY_SAMPLES;

        return (double) esp_adc_cal_raw_to_voltage(adc_reading, &_adc_chars) * BATTERY_RESISTANCE_NUM / 1000;
    }
}

#endif /* ODROID_GO_HW */