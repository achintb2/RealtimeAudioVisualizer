//finalizing...
#include <FastLED.h>
#define LED_PIN     3
#define NUM_LEDS    300
#define BUTTON_PIN  8
#define onboardLED  13
CRGB leds[NUM_LEDS];
int sat = 255; int light = 255;
String inString = "";

void setup()
{
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    fill_solid(leds, NUM_LEDS, CRGB(0,0,0));
    Serial.begin(128000);
    pinMode(8, INPUT_PULLUP);
    pinMode(onboardLED, OUTPUT);
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
        digitalWrite(onboardLED, HIGH);
        delay(500);
        digitalWrite(onboardLED, LOW);
    }
    return(currState);
}

int getValue()
{
    int color = 0;
    int inChar = 0;
    int wait = 0;
    while(inChar!= '\n' and wait < 25000)
    {
        inChar = 0;
        if(Serial.available() > 0)
        {
            inChar = Serial.read();
            inString += (char)inChar;
            if(inChar == '\n')
            {
                color = inString.toInt();
                inString = "";
            }
        }
        else{color=1;}
        wait++;
    }
    return(color);
}

void HSVMode(int xtraHue)
{
    int hue = 0, sat = 0, lit = 0;
    int altHue = 0;
    hue = getValue(); sat = getValue(); lit= getValue();
    altHue = hue;
    altHue += xtraHue;

    //Change this color to altHue for a color cycling effect, otherwise fixed blue color right now.
    if(hue == 1 && sat == 1 && lit == 1)
    {hue = 151; sat = 255; lit = 255;}
    else
    {hue = altHue;}

    if(getSwitchState() == LOW)
    {
        lit = 0;
    }
    
    hue = max(0, hue); 
    lit = max(0, lit); 
    sat = max(0, sat);
    fill_solid(leds, 250, CHSV(hue,sat,lit));
    FastLED.show();
}

void loop()
{
    for(int xh = 0; xh < 255; xh++)
    {HSVMode(xh);}
}
