//getswitchstate update
#include <FastLED.h>
#define LED_PIN     3
#define NUM_LEDS    300
#define BUTTON_PIN  8

CRGB leds[NUM_LEDS];
int sat = 255; int light = 255;
String inString = "";

void setup()
{
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    fill_solid(leds, NUM_LEDS, CRGB(0,0,0));
    Serial.begin(128000);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
}

bool toggState;
bool mySwitch = HIGH;
bool currState = HIGH;

bool getSwitchState()
{
    mySwitch = !digitalRead(BUTTON_PIN);
    if(mySwitch == HIGH)
    {
        currState = !currState;
    }
    return(currState);
}

void loop()
{
    for(int h=0; h<256; h++)
    {
        int inChar = 0;
        int wait = 0;
        
        while(inChar!= '\n' and wait < 10000)
        {
            inChar = 0; sat = 255;
            if(Serial.available() > 0)
            {
                inChar = Serial.read();
                inString += (char)inChar;
                if(inChar == '\n')
                {
                    light = inString.toInt();
                    sat -= 2*(light-220);
                    sat = max(0, min(sat, 255));
                    light = max(10, min(light, 255));
                    inString = "";
                }
            }
            else{light=255;}
            wait++;
            if(getSwitchState() == HIGH)
            {
                light = 0;
            }
        }
        fill_solid(leds, 150, CHSV(h,sat,light));
        FastLED.show();
    }
}
