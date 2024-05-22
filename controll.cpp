/*
 * File:          Controlador.cpp
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

#include <iostream>
#include <webots/Supervisor.hpp>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LED.hpp>
#include <cmath>
#include <cstdio>

using namespace std;
using namespace webots;

#define TIME_STEP 256
#define QtddSensoresProx 8
#define QtddLeds 10

int main(int argc, char **argv) {
    int i = 0;
    char texto[256];
    double LeituraSensorProx[QtddSensoresProx];
    double AceleradorDireito = 1.0, AceleradorEsquerdo = 1.0;
    for (i = 0; i < 256; i++) texto[i] = '0';

    Supervisor *robot = new Supervisor();
    
    // Configuração dos motores
    Motor *MotorEsquerdo = robot->getMotor("left wheel motor");
    Motor *MotorDireito = robot->getMotor("right wheel motor");
    MotorEsquerdo->setPosition(INFINITY);
    MotorDireito->setPosition(INFINITY);
    MotorEsquerdo->setVelocity(0);
    MotorDireito->setVelocity(0);

    // Configuração dos sensores de proximidade
    DistanceSensor *SensorProx[QtddSensoresProx];
    SensorProx[0] = robot->getDistanceSensor("ps0");
    SensorProx[1] = robot->getDistanceSensor("ps1");
    SensorProx[2] = robot->getDistanceSensor("ps2");
    SensorProx[3] = robot->getDistanceSensor("ps3");
    SensorProx[4] = robot->getDistanceSensor("ps4");
    SensorProx[5] = robot->getDistanceSensor("ps5");
    SensorProx[6] = robot->getDistanceSensor("ps6");
    SensorProx[7] = robot->getDistanceSensor("ps7");

    for (i = 0; i < QtddSensoresProx; i++) {
        SensorProx[i]->enable(TIME_STEP);
    }

    // Configuração dos LEDs
    LED *Leds[QtddLeds];
    Leds[0] = robot->getLED("led0");
    Leds[1] = robot->getLED("led1");
    Leds[2] = robot->getLED("led2");
    Leds[3] = robot->getLED("led3");
    Leds[4] = robot->getLED("led4");
    Leds[5] = robot->getLED("led5");
    Leds[6] = robot->getLED("led6");
    Leds[7] = robot->getLED("led7");
    Leds[8] = robot->getLED("led8");
    Leds[9] = robot->getLED("led9");

    for (i = 0; i < QtddLeds; i++) {
        Leds[i]->set(0);
    }

    Node *constCaixa = robot->getFromDef("wooden_box");
    double vector[3] = {-0.35, -0.35, 0.039995};
    
    while (robot->step(TIME_STEP) != -1) {
        for (i = 0; i < 256; i++) texto[i] = 0;
        for (i = 0; i < QtddSensoresProx; i++) {
            LeituraSensorProx[i] = SensorProx[i]->getValue() - 60;
            snprintf(texto, sizeof(texto), "%s|%d: %5.2f  ", texto, i, LeituraSensorProx[i]);
        }
        cout << texto << endl;
        
        double movimentoCoordenadas = 0.02;
        
        const double *caixa_movel = constCaixa->getPosition();
        if (fabs(vector[0] - caixa_movel[0]) > movimentoCoordenadas ||
            fabs(vector[1] - caixa_movel[1]) > movimentoCoordenadas ||
            fabs(vector[2] - caixa_movel[2]) > movimentoCoordenadas) {
            cout << "Encontrou a caixa." << endl;
            AceleradorDireito = 0;
            AceleradorEsquerdo = 0;
            MotorEsquerdo->setVelocity(0);
            MotorDireito->setVelocity(0);

            for (i = 0; i < QtddLeds; i++) {
                Leds[i]->set(1);
            }
            break;
        }

        if (LeituraSensorProx[0] > 10) {
            AceleradorDireito = -1;
            AceleradorEsquerdo = 1;
        } else {
            AceleradorDireito = 1.0;
            AceleradorEsquerdo = 1.0;
        }

        MotorEsquerdo->setVelocity(6.28 * AceleradorEsquerdo);
        MotorDireito->setVelocity(6.28 * AceleradorDireito);
    }

    delete robot;

    return 0;
}
