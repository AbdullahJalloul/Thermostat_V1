#ifndef _ThermocoupleSensor_H_
#define _ThermocoupleSensor_H_

// NTC thermistor for cold junction 
#define DEFAULT_BCOEF        	 3950.0
#define DEFAULT_NOMINAL_RES  	10000.0
#define BALANCE_RESISTOR 	 		10000.0
#define TEMPERATURENOMINAL   	   25.0

void ColdJunctionCompensate(void);
void LinearizeTCandConvert2degC(double emf);
void MeasureThermocoupleOutput(void);

double powerofXY(double X, double Y);
#endif
