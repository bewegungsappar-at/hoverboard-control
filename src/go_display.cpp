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

    int plotX = 0;


    // impl
    void setup(void)
    {
        GO.begin();
//        GO.lcd.setTextSize(2);
        GO.lcd.setFreeFont(&FreeMono9pt7b);
        GO.lcd.clearDisplay();
        GO_DISPLAY::show_labels();
        GO_DISPLAY::setup_internal_battery_adc();
    }
    void set(field_value_t field, float value)
    {
        GO.lcd.setCharCursor(10, 2);
        // TODO: just for now update V_bat everytime one set a value.
        GO_DISPLAY::show_internal_battery_voltage();

        uint8_t x=0,y=0;
        switch (field)
        {
        case CURRENT_LEFT:
            x = 19;
            y = 4;
            break;
        case CURRENT_RIGHT:
            x = 46;
            y = 4;
            break;
        case PWM_LEFT:
            x = 19;
            y = 6;
            break;
        case PWM_RIGHT:
            x = 46;
            y = 6;
            break;
        case SPEED:
            x = 19;
            y = 8;
            break;
        case STEER:
            x = 46;
            y = 8;
            break;
        case PACKAGE_LOSS_UPSTREAM:
            x = 46;
            y = 10;
            break;
        case PACKAGE_LOSS_DOWNSTREAM:
            x = 19;
            y = 10;
            break;
        case BATTERY_VOLTAGE:
            x = 46;
            y = 12;
            break;
        default:
            break;
        }
        if(0 != x && 0 != y)
        {
            GO.lcd.setCharCursor(x,y);
            GO.lcd.fillRect(x*6, (y*8)-11, 4*12,12, backgroundColor);
            GO.lcd.print(value);
        }
    }

    void plot(double value)
    {

        int plotY = (int) (230.0 - (value * 0.1));

        GO.lcd.drawLine(plotX, 230, plotX, (int) (230.0 - (200 * 0.4)), backgroundColor);

        if(value > 200) {
            GO.lcd.drawPixel(plotX,  (int) (230.0 - (200 * 0.4)), TFT_RED);
        } else if(value <= 0) {
            GO.lcd.drawPixel(plotX,  230, TFT_RED);
        } else {
            GO.lcd.drawPixel(plotX, plotY, TFT_ORANGE);
        }

        if(++plotX > 319) plotX = 0;
        GO.lcd.drawLine(plotX, 230, plotX, (int) (230.0 - (200 * 0.4)), TFT_DARKGREEN);
    }

    void show_labels()
    {
        GO.lcd.clearDisplay();
        GO.lcd.setCharCursor(0, 2);
        GO.lcd.setTextColor(TFT_MAROON, backgroundColor);
        GO.lcd.print("phail monitor  V_bat:");
        GO.lcd.setCharCursor(0, 4);
        GO.lcd.setTextColor(TFT_GREEN, backgroundColor);
        GO.lcd.print("L current      R current");
        GO.lcd.setCharCursor(0, 6);
        GO.lcd.print("L PWM          R PWM");
        GO.lcd.setCharCursor(0, 8);
        GO.lcd.print("speed          steer");
        GO.lcd.setCharCursor(0, 10);
        GO.lcd.print("Ping  ");
        GO.lcd.setCharCursor(0, 12);
        GO.lcd.print("hb battery in Volt");
    }

    void show_internal_battery_voltage()
    {
        GO.lcd.setTextColor(TFT_MAROON, backgroundColor);
        GO.lcd.setCharCursor(45,2);
        GO.lcd.fillRect(45*6, (2*8)-11, 4*12,12, backgroundColor);

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