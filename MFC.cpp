#include "MFC.h"
//constructor
MFC::MFC(int channel){
  DAC = channel;
  ADC = channel;
  TankC = 0;
  GasC = 0;
  gas = 'NotSet';
  offset = 32768;
  gain = 6440;
  maxFlow = 10;
  desiredFlow = 0;
  voltIn = 32768;
  voltOut = 32768;
  stdFlow = 0;
  onOff = 0;
}