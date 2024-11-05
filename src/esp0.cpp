#include "IIKit.h"
#include <Arduino.h>

#define TRMNivel def_pin_R4a20_1
#define MOTBomba def_pin_RELE
#define CHNivel def_pin_D1
#define IdCanalValvula 0

bool tripValue = false;

uint32_t time_counter_sa = 0;

AsyncDelay_c delayPOT(50); // time in milliseconds
void monitoraPOT(void)
{
    if (delayPOT.isExpired())
    {
        delayPOT.repeat();
        const uint16_t vlPOT1 = analogRead(def_pin_POT1);
        ledcWrite(IdCanalValvula, vlPOT1);
        IIKit.disp.setText(2, ("P1:" + String(100.0 * vlPOT1 / 4095.0)).c_str());
        IIKit.WSerial.plot("vlPOT1", vlPOT1);
    }
}

AsyncDelay_c delay4A20(100); // time in milliseconds
uint8_t count = 0;
double vlR4a20_1 = 0.0;
int Josue = 100;
int pwm=0.0 ; // Inicializa PWM como 0

void monitora4A20(void)
{
    if (delay4A20.isExpired())
    {
        delay4A20.repeat();
        vlR4a20_1 += (double)analogRead(def_pin_R4a20_1);

        if (++count >= 20)
        {

            vlR4a20_1 = vlR4a20_1 / count; // média dos 20 valores
            vlR4a20_1 = map(vlR4a20_1,0,4096,4,20);
 
            vlR4a20_1 = (0.99517397 * vlR4a20_1) + 0.17689519;
            


            pwm = 255; // 100% de duty cycle


            IIKit.disp.setText(3, ("T1:" + String(vlR4a20_1)).c_str());
            time_counter_sa = (millis() / 1000);
            IIKit.WSerial.print((time_counter_sa));
            IIKit.WSerial.print(",");
            IIKit.WSerial.print(String(vlR4a20_1));
            IIKit.WSerial.print(",");
            IIKit.WSerial.println(pwm);

            ledcWrite(IdCanalValvula, pwm); // Atualiza o PWM
            count = 0;
            vlR4a20_1 = 0;
        }
    }
}

void IRAM_ATTR trip_func()
{
    if (digitalRead(CHNivel) == LOW) // Considerando LOW como sinal de trip
    {
        digitalWrite(MOTBomba, LOW);
        tripValue = true;
    }
    else
    {
        tripValue = false;
    }
}

void setup()
{
    IIKit.setup();
    pinMode(CHNivel, INPUT_PULLDOWN);
    pinMode(def_pin_D4, OUTPUT);
    attachInterrupt(CHNivel, trip_func, CHANGE);

    ledcAttachPin(def_pin_W4a20_1, IdCanalValvula); // Atribui o pino ao canal de PWM.
    ledcSetup(IdCanalValvula, 19000, 8);            // Configura o canal de PWM.
    ledcWrite(IdCanalValvula, 0);                   // Inicializa o PWM com duty cycle de 0%.

    IIKit.rtn_1.onValueChanged([](uint8_t status)
    {
        if (!tripValue)
        {
            digitalWrite(MOTBomba, status);
            digitalWrite(def_pin_D4, status);
            IIKit.WSerial.println(status ? "MOTBomba ON" : "MOTBomba OFF");
        }
    });
}

void loop()
{
    IIKit.loop();
    // Descomente se necessário
    // monitoraPOT();
    monitora4A20();
}
