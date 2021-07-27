#include "config.h"
#include "go_display.h"

#ifdef ODROID_GO_HW


#include <driver/adc.h>
#include <esp_adc_cal.h>

namespace GO_DISPLAY
{
    // forward declaration
    static void setup_internal_battery_adc(void);
    static float get_internal_battery_voltage(void);
    static esp_adc_cal_characteristics_t _adc_chars;
    static uint16_t backgroundColor;



    // impl
    void setup(void)
    {
        GO.begin();

        //    pinMode(25, INPUT);  // These mute the speaker.. Speaker is connected to pam8304a audio amplifier IN- Pin 25, IN+ Pin26
        digitalWrite(25, LOW); // The library sets Pin 26 to HIGH. Pin 25 is also connected to SD, which
                           // Activates Standby of the Audio Amplifier when Low.

        GO_DISPLAY::setup_internal_battery_adc();
    }


    void connectionSelector()
    {

        int firstline       = 4;
        int firstcolumn     = 2;

        GO.lcd.clearDisplay();

        int cursor = firstline;
        GO.lcd.setTextColor(TFT_BLUE, backgroundColor);
        GO.lcd.setTextSize(2);

        GO.lcd.setCharCursor(firstcolumn + 2, cursor++);
        GO.lcd.print("UDP paddelec");
        GO.lcd.setCharCursor(firstcolumn + 2, cursor++);
        GO.lcd.print("UDP wireshark");
        GO.lcd.setCharCursor(firstcolumn + 2, cursor++);
        GO.lcd.print("UDP panzer");
        GO.lcd.setCharCursor(firstcolumn + 2, cursor++);
        GO.lcd.print("UDP zulu");
        GO.lcd.setCharCursor(firstcolumn + 2, cursor++);
        GO.lcd.print("ESPnow wireshark");
        GO.lcd.setCharCursor(firstcolumn + 2, cursor++);
        GO.lcd.print("ESPnow panzer");

        int entries =  cursor - firstline;
        cursor = firstline;

        int timer = 600;
        uint8_t joyYold = 0;
        GO.lcd.setCharCursor(firstcolumn, cursor);
        GO.lcd.print(">");

        while ( timer > 100 )
        {
            GO.update();
            GO.lcd.setCharCursor(firstcolumn, firstline - 2);
            GO.lcd.print(timer--/100);
            GO.lcd.print("   ");
            delay(10);

            if( GO.BtnA.isPressed() ) return;

            switch (cursor - firstline)
            {
            case 0:
                sysconfig.chan_out = COMM_CHAN_UDP;
                strcpy(sysconfig.wifi_ssid, "paddelec");
                strcpy(sysconfig.wifi_pass, "t.j9c4hkopppxs");
                break;

            case 1:
                sysconfig.chan_out = COMM_CHAN_UDP;
                strcpy(sysconfig.wifi_ssid, "wireshark");
                strcpy(sysconfig.wifi_pass, "t.j9c4hkopppxs");
                break;

            case 2:
                sysconfig.chan_out = COMM_CHAN_UDP;
                strcpy(sysconfig.wifi_ssid, "panzer");
                strcpy(sysconfig.wifi_pass, "t.j9c4hkopppxs");
                break;

            case 3:
                sysconfig.chan_out = COMM_CHAN_UDP;
                strcpy(sysconfig.wifi_ssid, "zulu");
                strcpy(sysconfig.wifi_pass, "t.j9c4hkopppxs");
                break;

            case 4:
                sysconfig.chan_out = COMM_CHAN_ESPNOW;
                strcpy(sysconfig.wifi_ssid, "wireshark");
                strcpy(sysconfig.wifi_pass, "t.j9c4hkopppxs");
                break;

            case 5:
                sysconfig.chan_out = COMM_CHAN_ESPNOW;
                strcpy(sysconfig.wifi_ssid, "panzer");
                strcpy(sysconfig.wifi_pass, "t.j9c4hkopppxs");
                break;

            default:
                break;
            }

            if( GO.JOY_Y.isAxisPressed() != joyYold ) {
                joyYold = GO.JOY_Y.isAxisPressed();
                timer = 600;
                switch (joyYold)
                {

                case 2:
                    GO.lcd.setCharCursor(firstcolumn,cursor);
                    GO.lcd.print(" ");
                    if(--cursor < firstline) cursor = firstline;
                    GO.lcd.setCharCursor(firstcolumn,cursor);
                    GO.lcd.print(">");
                    break;

                case 1:
                    GO.lcd.setCharCursor(firstcolumn,cursor);
                    GO.lcd.print(" ");
                    if(++cursor >= firstline + entries) cursor = firstline + entries;
                    GO.lcd.setCharCursor(firstcolumn,cursor);
                    GO.lcd.print(">");
                    break;

                default:
                    break;
                }
            }

        }
    }


    menu_result menu(bool init)
    {
        int firstline       = 4;
        int firstcolumn     = 2;
        static int cursor;
        static int entries;
        static uint8_t joyYold;

        if(init)
        {
            GO.lcd.clearDisplay();

            cursor = firstline;
            GO.lcd.setTextColor(TFT_BLUE, backgroundColor);
            GO.lcd.setTextSize(1);
            GO.lcd.setTextFont(1);

            GO.lcd.setCharCursor(firstcolumn + 2, cursor++);
            GO.lcd.print("Subscribe to Sensor Data");
            GO.lcd.setCharCursor(firstcolumn + 2, cursor++);
            GO.lcd.print("Send PWM");
            GO.lcd.setCharCursor(firstcolumn + 2, cursor++);
            GO.lcd.print("Send Enable");
            GO.lcd.setCharCursor(firstcolumn + 2, cursor++);
            GO.lcd.print("Send Disable");
            GO.lcd.setCharCursor(firstcolumn + 2, cursor++);
            GO.lcd.print("Decrease Wireshark distance");
            GO.lcd.setCharCursor(firstcolumn + 2, cursor++);
            GO.lcd.print("Increase Wireshark distance");
            GO.lcd.setCharCursor(firstcolumn + 2, cursor++);
            GO.lcd.print("Write Flash");
            GO.lcd.setCharCursor(firstcolumn + 2, cursor++);
            GO.lcd.print("Paddle Read");
            GO.lcd.setCharCursor(firstcolumn + 2, cursor++);
            GO.lcd.print("Paddle increase drag");
            GO.lcd.setCharCursor(firstcolumn + 2, cursor++);
            GO.lcd.print("Paddle decrease drag");
            GO.lcd.setCharCursor(firstcolumn + 2, cursor++);
            GO.lcd.print("Paddle increase PWM mult");
            GO.lcd.setCharCursor(firstcolumn + 2, cursor++);
            GO.lcd.print("Paddle decrease PWM mult");
            GO.lcd.setCharCursor(firstcolumn + 2, cursor++);
            GO.lcd.print("Paddle increase gyro to speed");
            GO.lcd.setCharCursor(firstcolumn + 2, cursor++);
            GO.lcd.print("Paddle decrease gyro to speed");
            GO.lcd.setCharCursor(firstcolumn + 2, cursor++);
            GO.lcd.print("PWM no limit");
            GO.lcd.setCharCursor(firstcolumn + 2, cursor++);
            GO.lcd.print("PWM limit 400");

            entries =  cursor - firstline;
            cursor = firstline;

            joyYold = 0;
            GO.lcd.setCharCursor(firstcolumn, cursor);
            GO.lcd.print(">");
        }
        else
        {
            GO.update();
            delay(10);

            if( GO.BtnA.isPressed() ) return (menu_result) (cursor - firstline);

            if( GO.JOY_Y.isAxisPressed() != joyYold ) {
                joyYold = GO.JOY_Y.isAxisPressed();
                switch (joyYold)
                {

                case 2:
                    GO.lcd.setCharCursor(firstcolumn,cursor);
                    GO.lcd.print(" ");
                    if(--cursor < firstline) cursor = firstline;
                    GO.lcd.setCharCursor(firstcolumn,cursor);
                    GO.lcd.print(">");
                    break;

                case 1:
                    GO.lcd.setCharCursor(firstcolumn,cursor);
                    GO.lcd.print(" ");
                    if(++cursor >= firstline + entries) cursor = firstline + entries;
                    GO.lcd.setCharCursor(firstcolumn,cursor);
                    GO.lcd.print(">");
                    break;

                default:
                    break;
                }
            }
        }
        return MENU_IDLE;
    }

    void set(field_value_t field, float value)
    {
        uint8_t x=0,y=0;
        switch (field)
        {
        case CURRENT_LEFT:
            x = 17;
            y = 4;
            break;
        case CURRENT_RIGHT:
            x = 44;
            y = 4;
            break;
        case PWM_LEFT:
            x = 17;
            y = 6;
            break;
        case PWM_RIGHT:
            x = 44;
            y = 6;
            break;
        case SPEED:
            x = 17;
            y = 8;
            break;
        case STEER:
            x = 44;
            y = 8;
            break;
        case PACKAGE_LOSS_UPSTREAM:
            x = 44;
            y = 10;
            break;
        case PACKAGE_LOSS_DOWNSTREAM:
            x = 17;
            y = 10;
            break;
        case BATTERY_VOLTAGE:
            x = 44;
            y = 12;
            break;
        default:
            break;
        }
        if(0 != x && 0 != y)
        {
            GO.lcd.setCharCursor(x,y);
            GO.lcd.fillRect(x*6, (y*8)-11, 5*12,12, backgroundColor);
            GO.lcd.printf("%5.1f", value);
        }
    }

    void plot(double value)
    {
        int plotBottom = 84;
        int plotHeight = 15;
        int plotLeft   = 45;
        int plotWidth  = 56;
        double valueMin = 0.0;
        double valueMax = 200.0;
        double valueScale   = (double)plotHeight / (valueMax - valueMin);

        static int plotX = plotLeft;

        int plotY = (int) (plotBottom - ((value-valueMin) * valueScale));

        // Clear old Data
        GO.lcd.drawRect(plotX, plotBottom - plotHeight, 1, plotHeight +1, 0x39E7);

        if(value > valueMax)
        {
            GO.lcd.drawPixel(plotX, plotBottom - plotHeight, TFT_RED);
        }
        else if(value <= valueMin)
        {
            GO.lcd.drawPixel(plotX,  plotBottom, TFT_RED);
        }
        else
        {
            GO.lcd.drawPixel(plotX, plotY, TFT_ORANGE);
        }

        // Move plot forward, wrap around at end
        if(++plotX > (plotWidth + plotLeft)) plotX = plotLeft;

        // Draw 'Cursor'
        GO.lcd.drawRect(plotX, plotBottom - plotHeight, 1, plotHeight +1, TFT_DARKGREY);
    }


    void plotBattery(double value)
    {
        int plotBottom = 230;
        int plotHeight = 40;
        int plotLeft   = 0;
        int plotWidth  = 200;
        double valueMin = 30.0;
        double valueMax = 42.0;
        double valueScale   = (double)plotHeight / (valueMax - valueMin);

        static int plotX = plotLeft;

        int plotY = (int) (plotBottom - ((value-valueMin) * valueScale));

        // Clear old Data
        GO.lcd.drawRect(plotX, plotBottom - plotHeight, 1, plotHeight +1, 0x39E7);

        if(value > valueMax)
        {
            GO.lcd.drawPixel(plotX, plotBottom - plotHeight, TFT_RED);
        }
        else if(value <= valueMin)
        {
            GO.lcd.drawPixel(plotX,  plotBottom, TFT_RED);
        }
        else
        {
            GO.lcd.drawPixel(plotX, plotY, TFT_ORANGE);
        }

        // Move plot forward, wrap around at end
        if(++plotX > (plotWidth + plotLeft)) plotX = plotLeft;

        // Draw 'Cursor'
        GO.lcd.drawRect(plotX, plotBottom - plotHeight, 1, plotHeight +1, TFT_DARKGREY);
    }

    void plotSpeed(double value)
    {
        int plotBottom = 180;
        int plotHeight = 60;
        int plotLeft   = 0;
        int plotWidth  = 200;
        double valueMin = -8.0;
        double valueMax = 8.0;
        double valueScale   = (double)plotHeight / (valueMax - valueMin);

        static int plotX = plotLeft;

        int plotY = (int) (plotBottom - ((value-valueMin) * valueScale));

        // Clear old Data
        GO.lcd.drawRect(plotX, plotBottom - plotHeight, 1, plotHeight +1, 0x39E7);

        if(value > valueMax)
        {
            GO.lcd.drawPixel(plotX, plotBottom - plotHeight, TFT_RED);
        }
        else if(value <= valueMin)
        {
            GO.lcd.drawPixel(plotX,  plotBottom, TFT_RED);
        }
        else
        {
            GO.lcd.drawPixel(plotX, plotY, TFT_BLUE);
        }

        // Move plot forward, wrap around at end
        if(++plotX > (plotWidth + plotLeft)) plotX = plotLeft;

        // Draw 'Cursor'
        GO.lcd.drawRect(plotX, plotBottom - plotHeight, 1, plotHeight +1, TFT_DARKGREY);
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