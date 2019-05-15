#include "main.h"
#include "Config.h"
#include "ThermocoupleSensor.h"
#include <stdbool.h>
#include <math.h>

const double Mhot[] = {0, 25.08355, 0.07860106, -0.2503131, 0.0831527, -0.01228034, 0.0009804036, -0.0000441303, 0.000001057734, -0.00000001052755};
double Temperature;

extern uint32_t ADC1_Result[3];
//******************************************************************************
void ColdJunctionCompensate(void) {
  uint32_t NtcAvg = ADC1_Result[ADC1_NTC] / ADCn_MAX_SAMPLES;
  NtcAvg = 4096 - NtcAvg;

  // convert the value to resistance
  NtcAvg = BALANCE_RESISTOR * (ADC_RESOLUTION / NtcAvg - 1);
  double tKelvin = (DEFAULT_BCOEF * (TEMPERATURENOMINAL + 273.15)) /
                   (DEFAULT_BCOEF + ((TEMPERATURENOMINAL + 273.15) * log(NtcAvg / DEFAULT_NOMINAL_RES)));
  //double tCelsius = tKelvin - 273.15;
  Temperature += (tKelvin - 273.15);
}
//******************************************************************************
double powerofXY(double X, double Y) {
  double num;
  int i;
  num = X;	//copy the value to a variable

  if (Y == 0) {
    X = 1;	//if X^0 = 1
  }
  else if (Y == 1) {
    X = num;	//X^1 = X
  }
  else {
    for (i = 2; i <= Y; i++) {	// X^Y where Y>1
      X = X * num;
    }
  }
  return X;
}
//******************************************************************************
void LinearizeTCandConvert2degC(double emf) {
  double toPower;
  double Coef;
  int counter;

  //temperature = M(i) + M_1*emf + M_2*emf^2 + ... + M_n*emf^n
  Temperature = 0;
  for (counter = 0; counter < 10; counter++) {
    toPower = powerofXY(emf, counter);
    Coef = Mhot[counter];
    Temperature = Coef * toPower + Temperature;
  }
  //return (Temperature);
}
//******************************************************************************

void MeasureThermocoupleOutput(void) {
  uint32_t Vout_code = 0;
  double emf;		//Thermocouple emf (unit mV)
  double Vout;

  Vout_code = ADC1_Result[ADC1_THERMOCOUPLE] / ADCn_MAX_SAMPLES;
  //Convert Code to Voltage
  Vout = Vout_code * (IN_ADC_VREF / ADC_RESOLUTION);
  emf  = Vout / 220;

  LinearizeTCandConvert2degC(emf);
  ColdJunctionCompensate();
}
//******************************************************************************
