#include <genieArduino.h>
#include <string.h>
#include <analogShield.h>
#include <Wire.h>
//#include <SoftwareSerial.h>
//SoftwareSerial photonSerial(8, 9) ; //RX, TX
#define RESETLINE 4 // Change this if you are not using an Arduino Adaptor Shield Version 2
Genie genie;
//Created by Daniel Green, TwoBTech, Boulder. Some code snippets are taken from genie demos.

// uSD file saving can not be implemneted while using visi genie! Must be visi serial!
// Thus this code CANNOT be used to implement uSD file saving to display uSD.
 
/*
   Calibration Tools button pressed
      Low Med High concentration's for mfc 3 and 4 calculated and displayed
      Set Time Remaining to 0
      Concentration MFC 3:
        OFF: conMFC3C = 0
        Low: conMFC3C = 1
        Med: conMFC3C = 2
        High: conMFC3C = 3
        Increment through all: conMFC3C = 4
        Custom Incrementation: conMFC3C = -1
            currentmFC = 3
            Switch to Custom Incrementation Form
      Concentration MFC 4:
        OFF: conMFC4C = 0
        Low: conMFC4C = 1
        Med: conMFC4C = 2
        High: conMFC4C = 3
        Increment through all: conMFC4C = 4
        Custom Incrementation: conMFC4C = -1
            currentMFC = 4
            Switch to Custom Incrementation Form
      Relative Humidity:
        15 %: relHumC = 1
        40 %: relHumC = 2
        65 %: relHumC = 3
        Increment through all: relHumC = 4
      Temperature:
        5 C: tempC = 1
        20 C: tempC = 2
        35 C: tempC = 3
        Increment through all: tempC = 4
      Increment Time:
        5 minutes at each increment: timeC = 1
        15 minutes at each increment: timeC = 2
        40 minutes at each increment: timeC = 3
      Purge Between Increments:
        OFF: purgeC = 0
        ON, 1 minute: purgeC = 1
        ON, 5 minute: purgeC = 2
       Confirm and Begin:
        if calStatus = 1
          do nothing, cal in progress already
        else
          Call function_calStart
          calStatus = 1
       Stop:
        Call function_calStop
        calStatus = 0
*/
//--------NEEDS UPDATING TO SEND AND RECEIVE COMPONENTS!!!---------//
/* Components used to send information to the 4D system display
     String 0,2,4,6: Indicate the gas being used in mfc
     String 1,3,5,7: Flow through mfc
     String 8,9: Gas Config, tank and flow concentration
     String 10-12: MFC information
     String 13: Desired MFC Flow units switcher. From mL/min, to L/min as needed
     String 14-16: MFC 4 calibration Low/Med/High values
     String 17-19: MFC 3 calibration Low/MEd/High values
     String 20: Time remaining for calibration
     String 21: Initial Concentration for custom cal increments
     String 22: Final Concentration for custom cal increments
     String 23: Increment for custom cal increments
     String 24: MFC selected for custom incrementation
     Leddigits 0: realtive humidity in gas config
     Leddigits 1: Desired Full System Flow
     Leddigits 2: System Temp
     Leddigits 3: System Press
     Leddigits 4: System Humidity
     Leddigits 5: Desired MFC Flow
     Leddigits 6: Signal Offset
     Leddigits 7: Signal Gain
*/
/* Input components of 4D display sending information to Arduino
     Slider 0-3: OnOff sliders for MFCs
     WinButton 0: To gas configuration panel
     WinButton 1: To flow configuration panel
     WinButton 2: To calibration tool panel
     WinButton 3-6,19-22: MFC selection
     WinButton 7-16: Gas selection
     WinButton 23: Tech Info button
     WinButton 17,18,24: Confirm changes
     WinButton 25-29,31-35: MFC concnetration buttons
     WinButton 30,36: To custom incrementation panel
     WinButton 37-40,55: Relative humidity choices for calibration
     WinButton 41-44: Temperature choices for calibration
     WinButton 45-47: Time choices for calibration
     WinButton 48-50: Purge choices for calibration
     WinButton 51: Confirm and Begin for calibration
     WinButton 52: Stop calibration
     WinButton 53: Confirm Custom Increments for cali
     WinButton 54: Return to calibration tool panel
     WinButton 56,57: MFC 4 controller type selection
     WinButton 58: Mirror calibration selection
     WinButton 59: To PAMM ID page
     KeyBoard 1,10,11,12: +100000 Incrementor
     KeyBoard 0: +1000 Incrementor
     KeyBoard 8,9: +100 Incrementor
     KeyBoard 2,3,4,6: +10 Incrementor
     KeyBoard 5,7: +1 Incrementor
     KeyBoard 13: PAMM ID keyboard
     4DButton 0,2,3,4: Back to Main Menu
     4DButton 1: To individual mfc configuration
     4DButton 5: back to calibration setup
*/
class MFC {
  public:
    double tankC; //Tank Concentration, if needed
    double gasC; //Desired Gas Concentration, if needed
    int gas; //gas used in system from list
    double hum; //humidity if needed
    double offset; //voltage offset for mfc
    double gain; //gain of mfc
    double desiredFlow; //flow input by user
    double sigIn; // 65535 signal sent to mfc
    double voltOut; // 65535 signal mfc is reading
    double stdFlowMax; //standard flow maximum
    bool onOff; //whether mfc is on or off
    int dac; // DAC channel of mfc
    int adc; // ADC channel of mfc, should be same as DAC channel
    MFC(int channel, int gasN, double stdF);
};
//constructor for MFC class
MFC::MFC(int channel, int gasN, double stdF) {
  dac = channel;
  adc = channel;
  tankC = 0;
  gasC = 0;
  gas = gasN;
  hum = 0;
  offset = 32768;
  gain = 6440;
  desiredFlow = 0;
  sigIn = 32768;
  voltOut = 32768;
  stdFlowMax = stdF;
  onOff = 0;
}
//------global variables--------//
//4 objects of MFC class, one for each controller
//Note!: MFC are 0-3 in code, but 1-4 in display UI!!t6
  MFC mfc[4] = { {0, 0, 10}, {1, 5, 10}, {2, 1, 0.03}, {3, 6, 0.03} }; //two 10 L/min (10 SLM) mfcs and two 0.03 L/min (30 sccm) mfc
//conversion from standard flow to actual
  double conv = (1 / 0.8) * ((298) / 273); //Assumes Boudler Colorado and room temperature
//after selecting MFC, in gas or flow configuration menus, it will be saved here
  int currentMFC = -1;
//desired output flow for entire system
  double sysFlow = 0;
//temperature of system
  double sysTemp = 20; //degrees C
//pressure of system
  double sysPres = 0.8; //atm
//humidity of system
  double sysHum = 0; // % relative humidity
//calibration status
  int calStatus = 0; // seeing if calibration is in progress
//inital concentration for custom calibrations
  double iCon[2] = {0, 0};
//final concentration for custom calibrations
  double fCon[2] = {0, 0};
//increments for custom calibrations
  double increment[2] = {0, 0};
//current MFC selected for custom calibrations
  int currentCalMFC = -1;
//numeric holder for calibration MFC concentration button selections
  int calMFCSelect[2] = {0, 0};
  int calHumSelect = 0;
  int calTempSelect = 0;
  int calTimeSelect = 0;
  int calPurgeSelect = 0;
  int calMirrorSelect[2] = {0,0};
//String to hold PAMM IDs
  String pammID = "";
//current number of PAMM IDs in string  
  double pammIDcounter = 0;
//------end global variables--------//
void setup() {
  //setup code, runs once:
  Serial.begin(9600);  // Serial0 @ 9600 (96K) Baud
  Serial1.begin(9600); //Serial1, used for sending data to photon, which will send data to data base
  Wire.begin(); //Communication line from this master arduino to slave temp control arduino
  genie.Begin(Serial);   // Use Serial0 for talking to the Genie Library, and to the 4D Systems display
  genie.AttachEventHandler(myGenieEventHandler); // Attach the user function Event Handler for processing event
  pinMode(RESETLINE, OUTPUT);  // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
  digitalWrite(RESETLINE, 1);  // Reset the Display via D4
  delay(100);
  digitalWrite(RESETLINE, 0);  // unReset the Display via D4

  delay (3500); //let the display start up after the reset (This is important)
  // Set the brightness/Contrast of the Display
  // Use 0-15 for Brightness Control, where 0 = Display OFF, though to 15 = Max Brightness ON.
  genie.WriteContrast(10);
  for (int ii = 0; ii < 4; ii++)
  { //set up start menu text
    genie.WriteStr(ii*2, gasToText(mfc[ii].gas));
  }
  genie.WriteObject(GENIE_OBJ_STRINGS, 36, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
  genie.DoEvents(); // This calls the library each loop to process the queued responses from the display
}
//onOff slider flipped on main page
void OnOff(int slider, int statusOnOff)
{
  if (statusOnOff == 1) //slider pressed to ON position
  {
    //send last saved signal to mfc
    analog.write(mfc[slider].adc, (unsigned int) mfc[slider].sigIn);
    //update flow string for mfc
    genie.WriteStr(slider * 2 + 1, mfc[slider].desiredFlow, 5);
    mfc[slider].onOff = 1;
  }
  else //slider pressed to OFF position
  {
    //send 0 point setting to mfc
    analog.write(mfc[slider].adc, (unsigned int) mfc[slider].offset);
    //set flow string to 0 for mfc
    genie.WriteStr(slider * 2 + 1, 0);
    mfc[slider].onOff = 0;
  }
  genie.WriteObject(GENIE_OBJ_SLIDER, slider, statusOnOff);
}
//return to main menu button pressed
void toMain(void)
{
  //move back to main menu screen
  genie.WriteObject(GENIE_OBJ_FORM, 0, 0);
  //update Gases and flows shown to user
  for ( int ii = 0; ii < 4; ii++)
  {
    genie.WriteStr(ii*2, gasToText(mfc[ii].gas));
    genie.WriteStr(ii * 2 + 1, mfc[ii].desiredFlow, 5);
  }
  if (calStatus == 1) //calibration running
  {
    genie.WriteStr(29, "Calibration");
    genie.WriteStr(30, "In Progress");
  }
  else //cal not running
  {
    genie.WriteStr(29, " ");
    genie.WriteStr(30, " ");
  }
}
//to MFC config button pressed
void toMFCconfig(void)
{
  //reset mfc selection as to not cause problems
  mfcSelectionMC(0);
  if(mfc[3].stdFlowMax == 0.03)//if MFC 4 is a 30 SCCM controller
  {
    //update toggle buttons to show mfc 4 as a 30 SCCM controller
    genie.WriteObject(GENIE_OBJ_STRINGS, 34, 0);
    genie.WriteObject(GENIE_OBJ_WINBUTTON, 56, 1);
  }
  else //if MFC 4 is a 1000 SCCM controller
  {
    //update toggle buttons to show mfc 4 as a 1000 SCCM controller
    genie.WriteObject(GENIE_OBJ_STRINGS, 34, 1);
    genie.WriteObject(GENIE_OBJ_WINBUTTON, 57, 1);
  }
  //move to mfc config screen
  genie.WriteObject(GENIE_OBJ_FORM, 3, 0);
}
//mfc selection for gasConfig
void mfcSelectionGC(int index)
{
  //as mfc is selected, change all strings and digits to reflect selected mfc
  digitsUpdate(index, mfc[index].gas);
  genie.WriteObject(GENIE_OBJ_WINBUTTON, mfc[index].gas + 7, 1);
  currentMFC = index;
}
//update strings showing gas concentrations and humdity on gas config page
void digitsUpdate(int mfci, int gas)
{
  if (mfci == 0 || gas == 0) // if large dry air mfc selected, or mfc has dry air flowing
  {
    //No tank or flow concentration
    genie.WriteStr(8, 0);
    genie.WriteStr(9, 0);
    //no humidity
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0, 0);
  }
  else if (mfci == 1 || gas == 5) //large humid mfc selected, or mfc has humid air flowing
  {
    //no tank or flow concentration 
    genie.WriteStr(8, 0);
    genie.WriteStr(9, 0);
    //humidity input by user
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0, mfc[mfci].hum * 10);
  }
  else // smaller mfcs selected with any other gas
  {
    //show tank and flow concentrtaions input by user
    genie.WriteStr(8, mfc[mfci].tankC / 1000, 0);
    genie.WriteStr(9, mfc[mfci].gasC, 0);
    //no humidity
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0, 0);
  }
}
//mfc selection for mfcConfig
void mfcSelectionMC(int index)
{
  if (index == 0 || index == 1) //mfc 1 or 2 chosen
  {
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 5, mfc[index].desiredFlow * 100);
    genie.WriteObject(GENIE_OBJ_STRINGS, 13, 0);
  }
  else if(index == 2) //mfc 3 chosen
  {
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 5, mfc[index].desiredFlow * 10000);
    genie.WriteObject(GENIE_OBJ_STRINGS, 13, 1);
  }
  else //mfc 4 chosen, which has two possible setups
  {
    if(mfc[index].stdFlowMax = 1) //1000 SCCM mfc
    {
      genie.WriteObject(GENIE_OBJ_LED_DIGITS, 5, mfc[index].desiredFlow * 100);
      genie.WriteObject(GENIE_OBJ_STRINGS, 13, 0);
    }
    else //30 SCCM mfc
    {
      genie.WriteObject(GENIE_OBJ_LED_DIGITS, 5, mfc[index].desiredFlow * 10000);
      genie.WriteObject(GENIE_OBJ_STRINGS, 13, 1);
    }
  }
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 6, mfc[index].offset);
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 7, mfc[index].gain);
  currentMFC = index;
  collectTechInfo();
}
//display signal in, signal out, and standard flow of mfc to user on mfc config page
void collectTechInfo(void)
{
  unsigned int temp = 0;
  // take an average of 5 measurements of signal out, since it varies widely
  for (int ii = 0; ii < 5; ii++)
  {
    temp += analog.read(currentMFC);
  }
  temp = temp / 5;
  genie.WriteStr(10, temp);
  //update mfc voltage out to new value
  mfc[currentMFC].voltOut = temp;
  genie.WriteStr(11, (unsigned int) mfc[currentMFC].sigIn);
  genie.WriteStr(12, mfc[currentMFC].desiredFlow / conv, 5 );
}
// convert desired flows to signals which are then sent to MFCs
void flowToSignal(int index)
{
  // convert desired flow to a voltage
  double voltage = 5 * mfc[index].desiredFlow / (mfc[index].stdFlowMax * conv);
  if (voltage < 0)
  {
    voltage = 0;
  }
  // convert that voltage to a signal
  double sig = mfc[index].gain * voltage + mfc[index].offset;
  // save new signal to mfc
  mfc[index].sigIn = sig;
  // turn on or off mfc depending on signal. 
  // OnOff function uses mfc[index].sigIn as the on level for the mfc
  if (sig <= mfc[index].offset)
  {
    OnOff(index, 0);
  }
  else
  {
    OnOff(index, 1);
  }
}
//run through numerous checks to ensure flow is properly balanced
void checkFlowBalance()
{
  double sumPerc = 0; //summation of the percentage of flow for each mfc
  double mfc2Perc = 0; //percentage of flow coming from MFC 2
  double mfc3Perc = 0; //percentage of flow coming from MFC 3
  double mfc4Perc = 0; //percentage of flow coming from MFC 4
  //NOTE: MFC 2,3,4 are denoted as mfc[1,2,3] in this function!
  //This is due to C++ coding starting arrays at 0, not 1.
  if (sysHum == 0)
  {
    //do nothing
  }
  else
  {
    if (mfc[1].hum == 0)
    {
      //warning! no humidity possible
    }
    else
    {
      mfc2Perc = sysHum / mfc[1].hum;
    }
  }
  if (mfc[2].tankC == 0)
  {
    //do nothing
  }
  else
  {
    mfc3Perc = mfc[2].gasC / mfc[2].tankC;
  }
  if (mfc[3].tankC == 0)
  {
    //do nothing
  }
  else
  {
    mfc4Perc = mfc[3].gasC / mfc[3].tankC;
  }
  sumPerc = mfc2Perc + mfc3Perc + mfc4Perc;
  //check if these concentrations and humdities can be balanced
  if (sumPerc > 1) //flow can not be made, reduce humidity
  {
    mfc[0].desiredFlow = 0;
    double humPerc = 1 - (mfc3Perc + mfc4Perc);
    if (humPerc < 0) //gas concentrations in MFC 3 and MFC 4 cannot be done at same time
    {
      // reduce concentrtaion of gas in MFC 4
      mfc4Perc = 1 - mfc3Perc;
      mfc[3].gasC = mfc[3].tankC * mfc4Perc;
      humPerc = 0; //no humidity can occur if MFC 3 and 4 combined make entire flow
    }
    sysHum = humPerc * mfc[1].hum; //maximum humdity given the required gas concentrations.
    mfc2Perc = sysHum / mfc[1].hum;
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 4, sysHum);
    //send warning
    sumPerc = 1;
  }
  mfc[0].desiredFlow = (1 - sumPerc) * sysFlow;
  mfc[1].desiredFlow = mfc2Perc * sysFlow;
  mfc[2].desiredFlow = mfc3Perc * sysFlow;
  mfc[3].desiredFlow = mfc4Perc * sysFlow;
  //check if MFC 4 is saturated
  if (mfc[3].desiredFlow > mfc[3].stdFlowMax * conv)
  {
    genie.WriteStr(12, "MFC 4 saturated");
    mfc[3].desiredFlow = mfc[3].stdFlowMax * conv;
    sysFlow = mfc[3].desiredFlow / mfc4Perc; //reduce system flow until in line with maximum output of mfc
    mfc[2].desiredFlow = sysFlow * mfc3Perc;
    mfc[1].desiredFlow = sysFlow * mfc2Perc;
    mfc[0].desiredFlow = sysFlow - (mfc[1].desiredFlow + mfc[2].desiredFlow + mfc[3].desiredFlow);
  }
  //check if MFC 3 is saturated
  if (mfc[2].desiredFlow > mfc[2].stdFlowMax * conv)
  {
    genie.WriteStr(12, "MFC 3 saturated");
    mfc[2].desiredFlow = mfc[2].stdFlowMax * conv;
    sysFlow = mfc[2].desiredFlow / mfc3Perc; //reduce system flow until in line with maximum output of mfc
    mfc[3].desiredFlow = sysFlow * mfc4Perc;
    mfc[1].desiredFlow = sysFlow * mfc2Perc;
    mfc[0].desiredFlow = sysFlow - (mfc[1].desiredFlow + mfc[2].desiredFlow + mfc[3].desiredFlow);
  }
  //check if MFC 2 is saturated
  if (mfc[1].desiredFlow > mfc[1].stdFlowMax * conv)
  {
    genie.WriteStr(12, "MFC 2 saturated");
    mfc[1].desiredFlow = mfc[1].stdFlowMax * conv;
    sysFlow = mfc[1].desiredFlow / mfc2Perc; //reduce system flow until in line with humidity
    mfc[2].desiredFlow = sysFlow * mfc3Perc;
    mfc[3].desiredFlow = sysFlow * mfc4Perc;
    mfc[0].desiredFlow = sysFlow - (mfc[1].desiredFlow + mfc[2].desiredFlow + mfc[3].desiredFlow);
  }
  //check if MFC 1 is saturated
  if (mfc[0].desiredFlow > mfc[0].stdFlowMax * conv) //dry air can not output enough flow at given perameters to balance system
  {
    genie.WriteStr(12, "MFC 1 saturated");
    mfc[0].desiredFlow = mfc[0].stdFlowMax * conv;
    sysFlow = mfc[0].desiredFlow / (1 - sumPerc); //reduce system flow until in line with maximum of dry air
    mfc[1].desiredFlow = sysFlow * mfc2Perc;
    mfc[2].desiredFlow = sysFlow * mfc3Perc;
    mfc[3].desiredFlow = sysFlow * mfc4Perc;
  }
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 1, sysFlow * 100);
  //now that flow is balanced, send new signals to mfcs
  for ( int ii = 0; ii < 4; ii++)
  {
    flowToSignal(ii);
  }
  transmitTemp(sysTemp);
}
// update settings button pressed on gas config page
void updateSettingsGC(int index)
{
  checkFlowBalance();
}
// update settings button pressed on individual MFC config page
void updateSettingsMC()
{
  //all error checking required is done in keyboards themselves, or in checkFlowBalance function, so none is required here
  collectTechInfo();
  checkFlowBalance();
}
// update settings button pressed on system config page
void updateSettingsSys(void)
{
  conv = (1 / sysPres) * ((sysTemp + 273) / 273);
  //send temperature control information to temp control arduino
  transmitTemp(sysTemp); 
  checkFlowBalance();
}
//use gas number to pass correct gas name as string 
String gasToText(int gasN)
{
  switch (gasN) {
    case 0:
      return "Air(Dry)";
      break;
    case 1:
      return "O2";
      break;
    case 2:
      return  "N2";
      break;
    case 3:
      return  "CO";
      break;
    case 4:
      return  "R2";
      break;
    case 5:
      return  "Air(Humid)";
      break;
    case 6:
      return  "O3";
      break;
    case 7:
      return  "NO2";
      break;
    case 8:
      return  "CO2";
      break;
    case 9:
      return  "D2";
      break;
    default:
      return  "ERROR";
      break;
  }
}
//once button to select gas is pressed, gas is tied to MFC
void gasSelection(int gasB)
{
  mfc[currentMFC].gas = gasB;
  digitsUpdate(currentMFC, gasB);
  genie.WriteObject(GENIE_OBJ_WINBUTTON, mfc[currentMFC].gas + 7, 1);
}
//MFC type selected in MFC config
void mfcTypeSelection(int type)
{
  // mfc 4 can be 30 SCCM (type 0) or 1000 SCCM (type 1)
  if (type == 0)
  {
    mfc[3].stdFlowMax = 0.03;
    genie.WriteObject(GENIE_OBJ_STRINGS, 13, 1);
    genie.WriteObject(GENIE_OBJ_STRINGS, 34, 0);
    genie.WriteObject(GENIE_OBJ_STRINGS, 36, 0);
    genie.WriteObject(GENIE_OBJ_STRINGS, 38, 0);
  }
  else if (type == 1)
  {
    mfc[3].stdFlowMax = 1;
    genie.WriteObject(GENIE_OBJ_STRINGS, 13, 0);
    genie.WriteObject(GENIE_OBJ_STRINGS, 34, 1);
    genie.WriteObject(GENIE_OBJ_STRINGS, 36, 1);
    genie.WriteObject(GENIE_OBJ_STRINGS, 38, 1);
  }
}
//Temperature being sent to temperature control arduino
void transmitTemp(double T)
{
 //fails if temp control arduino is not powered 
  /*Wire.beginTransmission(8);
  Wire.write((uint8_t)T);
  Wire.endTransmission();
  */
}
//-------Calibration Functitons-------//
// function to go to calibration tool
void toCal()
{
  //Update the Low/Med/High increments for mfc 3 and 4 in calibration setup
  currentCalMFC = -1;
  genie.WriteObject(GENIE_OBJ_FORM, 4, 0);
  genie.WriteStr(17, mfc[2].tankC / 1000, 0);
  genie.WriteStr(18, 2 * mfc[2].tankC / 1000, 0);
  genie.WriteStr(19, 3 * mfc[2].tankC / 1000, 0);
  genie.WriteStr(14, mfc[3].tankC / 1000, 0);
  genie.WriteStr(15, 2 * mfc[3].tankC / 1000, 0);
  genie.WriteStr(16, 3 * mfc[3].tankC / 1000, 0);
}
// mfc concetration selection in calibration setup
void calMFCConc(int mfcIndex, int increment)
{
  if (mfcIndex == 2) //MFC 3 selected
  {
    calMFCSelect[0] = increment;
    currentCalMFC = 0;
    if (increment == 5) //if custom incrementation button pressed
    {
      calCustomInc();
    }
  }
  else if (mfcIndex == 3) //MFC 4 selected
  {
    calMFCSelect[1] = increment;
    currentCalMFC = 1;
    if (increment == 5)
    {
      calCustomInc();
    }
  }
}
//relative humidity selection in calibration setup
void calRelHum(int increment)
{
  if (increment == 18)
  {
    calHumSelect = 4;
  }
  else
  {
    calHumSelect = increment;
  }
}
//temperature selection in calibration setup
void calTemp(int increment)
{
  calTempSelect = increment;
}
//time selection in calibration setup
void calTime(int increment)
{
  calTimeSelect = increment;
}
//purge selection in calibration setup
void calPurge(int increment)
{
  calPurgeSelect = increment;
}
//Increment up then back down selection
void calMirrorInc(int index)
{
  calMirrorSelect[currentCalMFC] = index;
}
//button pressed to go from custom incrementation window back to calibration setup
void toCalFromCustom()
{
  // close to function toCal, but checks for calibration in progress
  currentCalMFC = -1;
  genie.WriteObject(GENIE_OBJ_FORM, 4, 0);
  genie.WriteStr(17, mfc[2].tankC / 1000, 0);
  genie.WriteStr(18, 2 * mfc[2].tankC / 1000, 0);
  genie.WriteStr(19, 3 * mfc[2].tankC / 1000, 0);
  genie.WriteStr(14, mfc[3].tankC / 1000, 0);
  genie.WriteStr(15, 2 * mfc[3].tankC / 1000, 0);
  genie.WriteStr(16, 3 * mfc[3].tankC / 1000, 0);
  //if calibration in progress, print stats
  if (calStatus == 1)
  {
    //Printing in progress stats
    genie.WriteStr(25, mfc[2].gasC, 0);
    genie.WriteStr(26, mfc[3].gasC, 0);
    genie.WriteStr(27, sysHum, 0);
    genie.WriteStr(28, sysTemp, 0);
  }
}
//begin calibration
void beginCal()
{
  /*
     Check MFC 3 increment selection
      if custom incrementation
        save increment set.
     Repeat for MFC 4
     Check Relative Humidity
      if increment thru all
        save full set
     Check Temperature
      if increment thru all
         save full set
     Check Increment Time
     Check Purge
     Call balancing function
      if balancing returns with errors
        stop calibration
     Begin calibration loop
        create loop
          call calLoop
     Once initial cal complete
        check if mirror cal has been selected
        mirror mfc 3 cal
        mirror mfc 4 cal
     Finish
  */
  double timeCounter = 0; //number of loops that have occurred
  double iv3; //incrementor for MFC 3, initial value
  double fv3; //incrementor for MFC 3, final value
  double cv3; //incrementor for MFC 3, counter value
  //MFC 3
  switch (calMFCSelect[0]) {
    case 0: //Off
      iv3 = 0;
      fv3 = 0;
      cv3 = 1;
      break;
    case 1: //Low
      iv3 = mfc[2].tankC / 1000;
      fv3 = mfc[2].tankC / 1000;
      cv3 = 1;
      break;
    case 2: //Medium
      iv3 = 2 * mfc[2].tankC / 1000;
      fv3 = 2 * mfc[2].tankC / 1000;
      cv3 = 1;
      break;
    case 3: //High
      iv3 = 3 * mfc[2].tankC / 1000;
      fv3 = 3 * mfc[2].tankC / 1000;
      cv3 = 1;
      break;
    case 4: //MFC increment thru all
      //four iterations of MFC concentration
      iv3 = 0;
      fv3 = 3 * mfc[2].tankC / 1000;
      cv3 = mfc[2].tankC / 1000;
      break;
    case 5: //MFC custom increment
      //user defined iterations of MFC concentration
      iv3 = iCon[0];
      fv3 = fCon[0];
      cv3 = increment[0];
      break;
    default: //default is no concentration of MFC 3
      iv3 = 0;
      fv3 = 0;
      cv3 = 1;
      break;
  }
  double iv4; //incrementor for MFC 4, initial value
  double fv4; //incrementor for MFC 4, final value
  double cv4; //incrementor for MFC 4, counter value
  //MFC 4
  switch (calMFCSelect[1]) {
    case 0: //Off
      iv4 = 0;
      fv4 = 0;
      cv4 = 1;
      break;
    case 1: //Low
      iv4 = mfc[3].tankC / 1000;
      fv4 = mfc[3].tankC / 1000;
      cv4 = 1;
      break;
    case 2: //Medium
      iv4 = 2 * mfc[3].tankC / 1000;
      fv4 = 2 * mfc[3].tankC / 1000;
      cv4 = 1;
      break;
    case 3: //High
      iv4 = 3 * mfc[3].tankC / 1000;
      fv4 = 3 * mfc[3].tankC / 1000;
      cv4 = 1;
      break;
    case 4: //MFC increment thru all
      //four iterations of MFC concentration
      iv4 = 0;
      fv4 = 3 * mfc[3].tankC / 1000;
      cv4 = mfc[3].tankC / 1000;
      break;
    case 5: //MFC custom increment
      //user defined iterations of MFC concentration
      iv4 = iCon[1];
      fv4 = fCon[1];
      cv4 = increment[1];
      break;
    default: //default is no concentration of MFC 4
      iv3 = 0;
      fv3 = 0;
      cv3 = 1;
      break;
  }
  //Relative humidity
  double ivRH; //incremetnor for Relative humidity, initial value
  double fvRH; //incremetnor for Relative humidity, final value
  double cvRH; //incremetnor for Relative humidity, counter value
  switch (calHumSelect) {
    case 0: //15%
      ivRH = 15;
      fvRH = 15;
      cvRH = 1;
      break;
    case 1: //40%
      ivRH = 40;
      fvRH = 40;
      cvRH = 1;
      break;
    case 2: //65%
      ivRH = 65;
      fvRH = 65;
      cvRH = 1;
      break;
    case 3: //increment thru all
      ivRH = 15;
      fvRH = 65;
      cvRH = 25;
      break;
    case 4: //humidity turned off
      ivRH = 0;
      fvRH = 0;
      cvRH = 1;
      break;
    default: // default is 40 %humidity
      ivRH = 40;
      fvRH = 40;
      cvRH = 1;
      break;
  }
  //temperature
  double ivT; //incremetnor for temperature, initial value
  double fvT; //incremetnor for temperature, final value
  double cvT; //incremetnor for temperature, counter value
  switch (calTempSelect) {
    case 0: //5C
      ivT = 5;
      fvT = 5;
      cvT = 1;
      break;
    case 1: //20C
      ivT = 20;
      fvT = 20;
      cvT = 1;
      break;
    case 2: //35%
      ivT = 35;
      fvT = 35;
      cvT = 1;
      break;
    case 3: //increment thru all
      ivT = 5;
      fvT = 35;
      cvT = 15;
      break;
    default: //default is 20 C
      ivT = 20;
      fvT = 20;
      cvT = 1;
      break;
  }
  //time
  uint32_t timeForInc = 0; //time to hold at each increment
  switch (calTimeSelect) {
    case 0: //5 minutes
      timeForInc = 5 * 60000;
      break;
    case 1: //15 minutes
      timeForInc = 15 * 60000;
      break;
    case 2: //40 minutes
      timeForInc = 40 * 60000;
      break;
    default: //default is 5 minutes
      timeForInc = 5 * 60000;
      break;
  }
  calStatus = 1;//calibration is beginning
  
  //send settings to cloud
  String data;
  data += String(iv3)+','+String(fv3)+','+String(cv3)+','+String(iv4)+','+String(fv4)+','+String(cv4)+','+String(ivRH)+','+String(fvRH)+','+String(cvRH)+','+String(ivT)+','+String(fvT)+','+String(cvT)+','+String(timeForInc/1000)+','+String(calMirrorSelect[0])+','+String(calMirrorSelect[1])+',';
  if(calPurgeSelect == 0)
  {
    data += String(0)+ ',';
  }
  else if(calPurgeSelect == 1)
  {
    data += String(1*60)+',';
  }
  else if(calPurgeSelect == 2)
  {
    data += String(5*60)+',';
  }
  for (int ii = 0; ii<3; ii+=1)
  {
    data += gasToText(mfc[ii].gas)+',';
  }
  data += gasToText(mfc[3].gas);
  //send calibration data to server
  /*char mander = 0;
  while(mander != '1')
  {
    Serial1.print(data);
    Serial1.print("X");
    while( Serial1.available() == 0)
    {
      //do literally nothing
    }
    mander = Serial1.read();
    Serial1.print(mander);
  }*/
  Serial1.print(data);
  Serial1.print("X");
  Serial1.print(pammID);
  Serial1.print("Y");
  //now that selections are complete, nested for loops are used to increment thru calibration
  //MFC 3
  for (double ii = iv3; ii <= fv3; ii = ii + cv3)
  {
    //MFC 4
    for (double jj = iv4; jj <= fv4; jj = jj + cv4)
    {
      //Relative humidity
      for (double kk = ivRH; kk <= fvRH; kk = kk + cvRH)
      {
        //Temperature
        for (double nn = ivT; nn <= fvT; nn = nn + cvT)
        {
          if (calStatus == 0)
          {
            for (int mm = 0; mm < 4; mm += 1)
            {
              mfc[mm].desiredFlow = 0;
              flowToSignal(mm);
            }
            return;
          }
          else
          {
            //Set flow for calibration repeatedly
            //This can change within program if parameters are ill-conditioned,
            //so should be reset incase it was changed last cycle.
            sysFlow = 3; // 3 L/min
            //using all settings, send to loop control
            calLoop(ii, jj, kk, nn, timeForInc, calPurgeSelect, timeCounter);
            timeCounter += 1;
          }
        }
      }
    }
  }
  // initial cal complete
  genie.WriteStr(32, "Intl");
  genie.WriteStr(31, "Cal");
  genie.WriteStr(20, "Done");
  // check for mirrored cal
  if(calMirrorSelect[0] == 1)
  {
    double temp = iv3;
    iv3 = fv3;
    fv3 = temp;
    cv3 = -1*cv3;
    //Mirror loop for MFC 3 if need be
    //MFC 3
    for (double ii = iv3; ii >= fv3; ii = ii + cv3)
    {
      //MFC 4
      for (double jj = iv4; jj <= fv4; jj = jj + cv4)
      {
        //Relative humidity
        for (double kk = ivRH; kk <= fvRH; kk = kk + cvRH)
        {
          //Temperature
          for (double nn = ivT; nn <= fvT; nn = nn + cvT)
          {
            if (calStatus == 0)
            {
              for (int mm = 0; mm < 4; mm += 1)
              {
                mfc[mm].desiredFlow = 0;
                flowToSignal(mm);
              }
              return;
            }
            else
            {
              //Set flow for calibration repeatedly
              //This can change within program if parameters are ill-conditioned,
              //so should be reset incase it was changed last cycle.
              sysFlow = 3; // 3 L/min
              //using all settings, send to loop control
              calLoop(ii, jj, kk, nn, timeForInc, calPurgeSelect, timeCounter);
              timeCounter += 1;
            }
          }
        }
      }
    }
  }
  if(calMirrorSelect[1] == 1)
  {
    double temp = iv3;
    iv3 = fv3;
    fv3 = temp;
    cv3 = -1*cv3;
    temp = iv4;
    iv4 = fv4;
    fv4 = temp;
    cv4 = -1*cv4;
    //Mirror loop for MFC 4 if need be
    //MFC 3
    for (double ii = iv3; ii <= fv3; ii = ii + cv3)
    {
      //MFC 4
      for (double jj = iv4; jj >= fv4; jj = jj + cv4)
      {
        //Relative humidity
        for (double kk = ivRH; kk <= fvRH; kk = kk + cvRH)
        {
          //Temperature
          for (double nn = ivT; nn <= fvT; nn = nn + cvT)
          {
            if (calStatus == 0)
            {
              for (int mm = 0; mm < 4; mm += 1)
              {
                mfc[mm].desiredFlow = 0;
                flowToSignal(mm);
              }
              return;
            }
            else
            {
              //Set flow for calibration repeatedly
              //This can change within program if parameters are ill-conditioned,
              //so should be reset incase it was changed last cycle.
              sysFlow = 3; // 3 L/min
              //using all settings, send to loop control
              calLoop(ii, jj, kk, nn, timeForInc, calPurgeSelect, timeCounter);
              timeCounter += 1;
            }
          }
        }
      }
    }
  }
  //end of calibration
  genie.WriteStr(32, "Cal");
  genie.WriteStr(31, "Comp");
  genie.WriteStr(20, "lete");
  //set all flows to zero
  for (int mm = 0; mm < 4; mm += 1)
  {
    mfc[mm].desiredFlow = 0;
    flowToSignal(mm);
  }
  calStatus = 0;
}
// loop used to seperate setup and looping execution of calibration
void calLoop(double mfc3Con, double mfc4Con, double relHum, double temperature, uint32_t timeInc, int purge, double counter)
{
  /*
   * Check if cal has been stopped by user
   * If it has
   *  stop caling
   * Update mfc 3 and 4 with new step concentrations 
   * Update conversion factor
   * Send temperature control arduino new temperature
   * update humidity
   * check flow balance and send signals to MFCs to begin new flow
   * Print out new flow data to user
   * Calculate time that has passed in cal 
   * Begin sub loop that ends when the step time - purge time has passed
   *  check if cal has been stopped
   *  Check for new inputs for user
   *    ONLY THE CAL STOP COMMAND WILL BE RECOGNIZED DURING A CAL-
   *    -this ensures multiple cals are not layered ontop of one another and stops flow configuratoins from changing
   *  Update the amount of time that has passed in the cal and display to user
   * Finish step
   * Check for purge, if present
   *  set all flows besides mfc 1 to zero
   *  update display to show that purge is in progress
   *  begin loop
   *    check for new inputs
   *    check for cal stop
   * End purge
   * End Step, leave function
   */
  if(calStatus == 0)
  {
    return;
  }
  //update gas concentrations
  mfc[2].gasC = mfc3Con;
  mfc[3].gasC = mfc4Con;
  //update conversion factor with new temperature
  conv = (1 / sysPres) * ((temperature + 273) / 273);
  sysTemp = temperature;
  //send temperature control information to temp control arduino
  transmitTemp(sysTemp);
  //update system humidity
  sysHum = relHum;
    
  checkFlowBalance();
  //Printing in progress stats
  genie.WriteStr(25, mfc[2].gasC, 0);
  genie.WriteStr(26, mfc[3].gasC, 0);
  genie.WriteStr(27, sysHum, 0);
  genie.WriteStr(28, sysTemp, 0);
  //time between each increment given time increment and purge length
  double multiplier = 0;
  if (calPurgeSelect == 1) //one minute purges
  {
    multiplier = timeInc + 60000;
  }
  else if (calPurgeSelect == 2) //5 minutes purges
  {
    multiplier = timeInc + 5 * 60000;
  }
  else //no purge
  {
    multiplier = timeInc;
  }
  //have system loop until otherwise noted
  for ( uint32_t tStart = millis(); (millis() - tStart) < timeInc; )
  {
    //print time ELAPSED
    if (calStatus == 0) //if exit status confirmed
    {
      return; //stop looping
    }
    //to ensure string does not blink insanely fast, small pauses are put in
    for ( uint32_t temp = millis(); (millis() - temp) < 500; )
    {
      genie.DoEvents(); // This calls the library to process the queued responses from the display
    }
    //format time into the more comprehensable hr:min:sec instead of seconds
    double seconds = ((millis() - tStart) + counter * multiplier) / 1000;
    int minutes = 0;
    int hours = 0;
    while (seconds > 59.9)
    {
      minutes += 1;
      seconds = seconds - 60;
    }
    while (minutes > 59)
    {
      hours += 1;
      minutes = minutes - 60;
    }
    genie.WriteStr(20, seconds, 0);
    genie.WriteStr(31, minutes, 0);
    genie.WriteStr(32, hours, 0);
  }
  if (calStatus == 0)
  {
    return;//stop function
  }
  //purge after loop as needed
  uint32_t purgeTime = 0;
  if (purge == 0)
  {
    //do nothing, no purge required
  }
  else if (purge == 1) //1 minute purges
  {
    purgeTime = 1 * 60000;
    // set mfc 2,3,4 flows to zero
    mfc[3].gasC = 0;
    mfc[2].gasC = 0;
    sysHum = 0;
    // generate flow for new configuration
    checkFlowBalance();
    //print text to string
    genie.WriteStr(32, "Purg");
    genie.WriteStr(31, "ing");
    genie.WriteStr(20, "...");
    for ( uint32_t tStart = millis(); (millis() - tStart) < purgeTime; )
    {
      genie.DoEvents(); // This calls the library to process the queued responses from the display
      if (calStatus == 0) //if exit status confirmed
      {
        return; //stop looping
      }
    }
  }
  else if (purge == 2) //5 minute purge
  {
    purgeTime = 5 * 60000;
    // set mfc 2,3,4 flows to zero
    mfc[3].gasC = 0;
    mfc[2].gasC = 0;
    sysHum = 0;
    // generate flow for new configuration
    checkFlowBalance();
    genie.WriteStr(32, "Purg");
    genie.WriteStr(31, "ing");
    genie.WriteStr(20, "...");
    for ( uint32_t tStart = millis(); (millis() - tStart) < purgeTime; )
    {
      genie.DoEvents(); // This calls the library to process the queued responses from the display
      if (calStatus == 0) //if exit status confirmed
      {
        return; //stop looping
      }
      delay(100);
    }
  }
}
//stop whatever calibration is in progress
void stopCal()
{
  calStatus = 0;
}
//custom increment button pressed
void calCustomInc()
{
  genie.WriteObject(GENIE_OBJ_FORM, 5, 0);
  genie.WriteObject(GENIE_OBJ_STRINGS, 24, currentCalMFC);
  genie.WriteStr(21, iCon[currentCalMFC], 0);
  genie.WriteStr(22, fCon[currentCalMFC], 0);
  genie.WriteStr(23, increment[currentCalMFC], 0);
  calMFCSelect[currentCalMFC] = 5;
}
//confirm custom increments
void calConfirmCustomInc()
{
  toCalFromCustom();
}
//delete last input pamm id
void deletePAMM()
{
  //if fully formed id is the last input thing
  if(pammID.substring(pammID.length()-1) == ",")
  {
    pammID.remove(pammID.length()-1);
    while((pammID.length() != 0) && (pammID.substring(pammID.length()-1) != ","))
    {
      pammID.remove(pammID.length()-1);
      printPID();
    }
    pammIDcounter -= 1;
  }
  //full pamm id not yet input. basically deleting the id the user has yet to confirm
  else
  {
    while((pammID.length() != 0) && (pammID.substring(pammID.length()-1) != ","))
    {
      pammID.remove(pammID.length()-1);
      printPID();
    }
  }
  genie.WriteObject(GENIE_OBJ_LED_DIGITS,8,pammIDcounter);
  printPID();
}
//go to pamm id screen
void toPAMMid()
{
  genie.WriteObject(GENIE_OBJ_FORM, 6, 0);
  printPID();
  genie.WriteObject(GENIE_OBJ_LED_DIGITS,8,pammIDcounter);
}
// print pamm IDs in a sereis of strings
void printPID()
{
  int lengthInc = 0;
  if(pammID.length() > 200)
  {
    lengthInc = 4;
  }
  else if(pammID.length() > 150)
  {
    lengthInc = 3;
  }
  else if(pammID.length() > 100)
  {
    lengthInc = 2;
  }
  else if(pammID.length() > 50)
  {
    lengthInc = 1;
  }
  else
  {
    //do nothing
  }
  String temp;
  if(pammID.length() == 0)
  {
    for(int ii = 33; ii<42 ; ii+=2)
    {
      genie.WriteStr(ii,"");  
    }
  }
  //each print section can only hold 50 characters
  while(pammID.length()-lengthInc*50 > 50)
  {
    //segment pammID into 50 character sections, and print in a decending list of strings.
    temp = pammID.substring(lengthInc*50,lengthInc*50 + 50);
    genie.WriteStr(33+2*lengthInc,temp);
    lengthInc += 1; 
  }
  //final print section has upto 50 characters, not exactly 50
  temp = pammID.substring(lengthInc*50,lengthInc*50 + (pammID.length()-lengthInc*50));
  genie.WriteStr(33+2*lengthInc,temp);
  //clear all unused print sections
  for(int ii= 35+2*lengthInc; ii< 42; ii +=2)
  {
    genie.WriteStr(ii,"");
  }
}
//------------KeyBoard Functions---------------//
// keyboard connected to tank concentration pressed
void tankConcentration(int keyValue)
{
  double tankCon = mfc[currentMFC].tankC;
  switch (keyValue) {
    //tankC is displayed in PPM, while gas is displayed in PPB. Both are stored in PPB.
    case 0:
      tankCon = tankCon + 100000;
      break;
    case 1:
      tankCon = tankCon + 10000;
      break;
    case 2:
      tankCon = tankCon + 1000;
      break;
    case 3:
      tankCon = tankCon - 1000;
      break;
    case 4:
      tankCon = tankCon - 10000;
      break;
    case 5:
      tankCon = tankCon - 100000;
      break;
    case 6:
      tankCon = 0;
      break;
    case 7:
      tankCon = tankCon + 1000000;
      break;
    default:
      //nothing
      break;
  }
  if (tankCon < 0) //concentration can not be negative
  {
    tankCon = 0;
  }
  else if (tankCon > 999999000)
  {
    tankCon = 999999000;
  }
  genie.WriteStr(8, tankCon / 1000, 0);
  mfc[currentMFC].tankC = tankCon;
}
// keyboard connected to gas concentration, or concentration in flow, pressed
void flowConcentration(int keyValue)
{
  double flowCon = mfc[currentMFC].gasC;
  switch (keyValue) {
    case 0:
      flowCon = flowCon + 100000;
      break;
    case 1:
      flowCon = flowCon + 10000;
      break;
    case 2:
      flowCon = flowCon + 1000;
      break;
    case 3:
      flowCon = flowCon + 100;
      break;
    case 4:
      flowCon = flowCon + 10;
      break;
    case 5:
      flowCon = flowCon - 10;
      break;
    case 6:
      flowCon = flowCon - 100;
      break;
    case 7:
      flowCon = flowCon - 1000;
      break;
    case 8:
      flowCon = flowCon - 10000;
      break;
    case 9:
      flowCon = 0;
      break;
    default:
      //nothing
      break;
  }
  if (flowCon < 0) //concentration can not be negative
  {
    flowCon = 0;
  }
  else if (flowCon > mfc[currentMFC].tankC)//concentration in flow cannnot be greater than tank concentration
  {
    flowCon = mfc[currentMFC].tankC;
  }
  genie.WriteStr(9, flowCon, 0);
  mfc[currentMFC].gasC = flowCon;
}
// keyboard connected to humdity of air into mfc pressed
void humidityGC(int keyVal)
{
  double relHum = mfc[currentMFC].hum;
  switch (keyVal) {
    case 0:
      relHum = relHum + 10;
      break;
    case 1:
      relHum = relHum + 1;
      break;
    case 2:
      relHum = relHum + 0.1;
      break;
    case 3:
      relHum = relHum - 0.1;
      break;
    case 4:
      relHum = relHum - 1;
      break;
    case 5:
      relHum = relHum - 10;
      break;
    case 6:
      relHum = 0;
      break;
    default:
      //nothing
      break;
  }
  if (relHum < 0) //humidity can not be negative
  {
    relHum = 0;
  }
  else if (relHum < sysHum) //humidity cannot be less than system humidity
  {
    relHum = sysHum;
  }
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0, relHum * 10);
  mfc[currentMFC].hum = relHum;
}
// keyboard connected to system flow pressed
void fullFlow(int keyVal)
{
  switch (keyVal) {
    case 0:
      sysFlow = sysFlow + 1;
      break;
    case 1:
      sysFlow = sysFlow + 0.1;
      break;
    case 2:
      sysFlow = sysFlow + 0.01;
      break;
    case 3:
      sysFlow = sysFlow - 0.01;
      break;
    case 4:
      sysFlow = sysFlow - 0.1;
      break;
    case 5:
      sysFlow = sysFlow - 1;
      break;
    case 6:
      sysFlow = 0;
      break;
    default:
      //nothing
      break;
  }
  if (sysFlow < 0) //flow can not be negative
  {
    sysFlow = 0;
  }
  else if (sysFlow > 99.99) //flow cannot be over 99.99 L/min
  {
    sysFlow = 99.99;
  }
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 1, sysFlow * 100);
}
// keyboard connected to system temperature pressed
void sysT(int keyVal)
{
  switch (keyVal) {
    case 0:
      sysTemp = sysTemp + 10;
      break;
    case 1:
      sysTemp = sysTemp + 1;
      break;
    case 2:
      sysTemp = sysTemp - 1;
      break;
    case 3:
      sysTemp = sysTemp - 10;
      break;
    case 4:
      sysTemp = 20;
      break;
    default:
      //nothing
      break;
  }
  if (sysTemp < 0) //flow should not go below freezing point of water
  {
    sysTemp = 0;
  }
  else if (sysTemp > 99) //temp should not exceed boiling point of water
  {
    sysTemp = 99;
  }
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 2, sysTemp);
}
// keyboard connected to system pressure pressed
void sysP(int keyVal)
{
  switch (keyVal) {
    case 0:
      sysPres = sysPres + 1;
      break;
    case 1:
      sysPres = sysPres + 0.1;
      break;
    case 2:
      sysPres = sysPres + 0.01;
      break;
    case 3:
      sysPres = sysPres - 0.01;
      break;
    case 4:
      sysPres = sysPres - 0.1;
      break;
    case 5:
      sysPres = sysPres - 1;
      break;
    case 6:
      sysPres = 1;
      break;
    default:
      //nothing
      break;
  }
  if (sysPres < 0.01) //pressure should not reduce towards vaccuum
  {
    sysPres = 0.01;
  }
  else if (sysPres > 99.99) //pressure should not exceed 100 atm.
  {
    sysPres = 99.99;
  }
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 3, sysPres * 100);
}
// keyboard connected to humdity of system pressed
void humSys(int keyVal)
{
  double relHum = sysHum;
  switch (keyVal) {
    case 0:
      relHum = relHum + 10;
      break;
    case 1:
      relHum = relHum + 1;
      break;
    case 2:
      relHum = relHum + 0.1;
      break;
    case 3:
      relHum = relHum - 0.1;
      break;
    case 4:
      relHum = relHum - 1;
      break;
    case 5:
      relHum = relHum - 10;
      break;
    case 6:
      relHum = 0;
      break;
    default:
      //nothing
      break;
  }
  if (relHum < 0) //humidity can not be negative
  {
    relHum = 0;
  }
  else if (relHum > 99.9) //humidity in system cannot be greater than 100%
  {
    relHum = 99.9;
  }
  else if (relHum > mfc[1].hum)
  {
    relHum = mfc[1].hum;
  }
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 4, relHum * 10);
  sysHum = relHum;
}
// keyboard connected to individual mfc flow pressed
// Removed. checkFlowBalance does this operation automatically
/*void mfcFlow(int keyVal)
{
  //all flows saved in L/min form, but mfc 2 and 3 display flow in ml/min
  double flow = mfc[currentMFC].desiredFlow;
  if (currentMFC == 0 || currentMFC == 1) //10 L/min mfc selected
  {
    switch (keyVal)
    {
      case 0:
        flow = flow + 1;
        break;
      case 1:
        flow = flow + 0.1;
        break;
      case 2:
        flow = flow + 0.01;
        break;
      case 3:
        flow = flow - 0.01;
        break;
      case 4:
        flow = flow - 0.1;
        break;
      case 5:
        flow = flow - 1;
        break;
      case 6:
        flow = 0;
        break;
      default:
        //nothing
        break;
    }
  }
  else // 30 ml/min mfc selected
  {
    switch (keyVal)
    {
      case 0:
        flow = flow + 0.01;
        break;
      case 1:
        flow = flow + 0.001;
        break;
      case 2:
        flow = flow + 0.0001;
        break;
      case 3:
        flow = flow - 0.0001;
        break;
      case 4:
        flow = flow - 0.001;
        break;
      case 5:
        flow = flow - 0.01;
        break;
      case 6:
        flow = 0;
        break;
      default:
        //nothing
        break;
    }
  }
  if (flow > mfc[currentMFC].stdFlowMax * conv) //cannot push mfc past saturation
  {
    flow = mfc[currentMFC].stdFlowMax * conv;
  }
  mfc[currentMFC].desiredFlow = flow;
  if (currentMFC == 0 || currentMFC == 1) //10 L/min mfc selected
  {
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 5, flow * 100);
  }
  else
  {
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 5, flow * 10000);
  }
}*/
// keyboard connected to mfc signal offset pressed
void mfcOffset(int keyVal)
{
  double sOff = mfc[currentMFC].offset;
  switch (keyVal) {
    case 0:
      sOff = sOff + 100;
      break;
    case 1:
      sOff = sOff + 10;
      break;
    case 2:
      sOff = sOff + 1;
      break;
    case 3:
      sOff = sOff - 1;
      break;
    case 4:
      sOff = sOff - 10;
      break;
    case 5:
      sOff = sOff - 100;
      break;
    case 6:
      sOff = 32768;
      break;
    default:
      //nothing
      break;
  }
  if (sOff < 0) //offset cannot be negative
  {
    sOff = 0;
  }
  else if (sOff > 65535) //offset cannot be over 65535
  {
    sOff = 65535;
  }
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 6, sOff);
  mfc[currentMFC].offset = sOff;
}
// keyboard connected to mfc signal gain pressed
void mfcGain(int keyVal)
{
  double sGain = mfc[currentMFC].gain;
  switch (keyVal) {
    case 0:
      sGain = sGain + 100;
      break;
    case 1:
      sGain = sGain + 10;
      break;
    case 2:
      sGain = sGain + 1;
      break;
    case 3:
      sGain = sGain - 1;
      break;
    case 4:
      sGain = sGain - 10;
      break;
    case 5:
      sGain = sGain - 100;
      break;
    case 6:
      sGain = 6440;
      break;
    default:
      //nothing
      break;
  }
  if (sGain < 0) //offset cannot be negative
  {
    sGain = 0;
  }
  else if (sGain > 9999) //offset cannot be over 99999
  {
    sGain = 9999;
  }
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 7, sGain);
  mfc[currentMFC].gain = sGain;
}
//keyboard connected to initial concentration in custom calibration
void calICon(int keyVal)
{
  double con = iCon[currentCalMFC];
  switch (keyVal) {
    case 0:
      con = con + 100000;
      break;
    case 1:
      con = con + 10000;
      break;
    case 2:
      con = con + 1000;
      break;
    case 3:
      con = con + 100;
      break;
    case 4:
      con = con + 10;
      break;
    case 5:
      con = con - 100;
      break;
    case 6:
      con = con - 100;
      break;
    case 7:
      con = con - 1000;
      break;
    case 8:
      con = con - 10000;
      break;
    case 9:
      con = 0;
      break;
    default:
      //nothing
      break;
  }
  if (con < 0) //offset cannot be negative
  {
    con = 0;
  }
  else if (con > mfc[currentCalMFC + 2].tankC) //offset cannot be over 99999
  {
    con = mfc[currentCalMFC + 2].tankC;
  }
  else if (con > fCon[currentCalMFC])
  {
    con = fCon[currentCalMFC];
  }
  genie.WriteStr(21, con, 0);
  iCon[currentCalMFC] = con;
}
//keyboard connected to final concentration in custom calibration
void calFCon(int keyVal)
{
  double con = fCon[currentCalMFC];
  switch (keyVal) {
    case 0:
      con = con + 100000;
      break;
    case 1:
      con = con + 10000;
      break;
    case 2:
      con = con + 1000;
      break;
    case 3:
      con = con + 100;
      break;
    case 4:
      con = con + 10;
      break;
    case 5:
      con = con - 10;
      break;
    case 6:
      con = con - 100;
      break;
    case 7:
      con = con - 1000;
      break;
    case 8:
      con = con - 10000;
      break;
    case 9:
      con = 0;
      break;
    default:
      //nothing
      break;
  }
  if (con < 0) //concentration cant be negative
  {
    con = 0;
  }
  else if (con > mfc[currentCalMFC + 2].tankC) //concentration cant be greater than tank
  {
    con = mfc[currentCalMFC + 2].tankC;
  }
  genie.WriteStr(22, con, 0);
  fCon[currentCalMFC] = con;
}
//keyboard connect to concentration increment in custom calibration
void calIncrement(int keyVal)
{
  double inc = increment[currentCalMFC];
  switch (keyVal) {
    case 0:
      inc = inc + 100000;
      break;
    case 1:
      inc = inc + 10000;
      break;
    case 2:
      inc = inc + 1000;
      break;
    case 3:
      inc = inc + 100;
      break;
    case 4:
      inc = inc + 10;
      break;
    case 5:
      inc = inc - 10;
      break;
    case 6:
      inc = inc - 100;
      break;
    case 7:
      inc = inc - 1000;
      break;
    case 8:
      inc = inc - 10000;
      break;
    case 9:
      inc = 0;
      break;
    default:
      //nothing
      break;
  }
  if (inc < 0) //offset cannot be negative
  {
    inc = 0;
  }
  else if (inc > mfc[currentCalMFC + 2].tankC) //cant be larger than tank Concentration
  {
    inc = mfc[currentCalMFC + 2].tankC;
  }
  genie.WriteStr(23, inc, 0);
  increment[currentCalMFC] = inc;
}
//keyboard to PAMM IDs
void pammIDinput(int keyVal)
{
  String pamm = pammID;
  switch (keyVal) {
    case 0:
      pamm += "0";
      break;
    case 1:
      pamm += "1";
      break;
    case 2:
      pamm += "2";
      break;
    case 3:
      pamm += "3";
      break;
    case 4:
      pamm += "4";
      break;
    case 5:
      pamm += "5";
      break;
    case 6:
      pamm += "6";
      break;
    case 7:
      pamm += "7";
      break;
    case 8:
      pamm += "8";
      break;
    case 9:
      pamm += "9";
      break;
    case 10:
      pamm += ",";
      pammIDcounter += 1;
      break;
    case 11:
      if(pamm.length() == 0)//nothing in string
      {
        //do nothing
      }
      else if(pammID.substring(pammID.length()-1) == ",")
      //entire PAMM id has been deleted, would begin invading next one over
      {
        //do nothing
      }
      else
      {
        //set last character to null 
        pamm.remove(pamm.length()-1);
      }
      break;
    default:
      //nothing
      break;
  }
  pammID = pamm;
  printPID();
  genie.WriteObject(GENIE_OBJ_LED_DIGITS,8,pammIDcounter);
}
//----------end keypad function-----------//

//function that interfaces between display and arduino
void myGenieEventHandler(void)
{
  genieFrame Event;
  genie.DequeueEvent(&Event); // Remove the next queued event from the buffer, and process it below
  //check to see if cal stop button was pressed
  if (genie.EventIs(&Event, GENIE_REPORT_EVENT, GENIE_OBJ_WINBUTTON, 52))
  {
    stopCal();
  }
  //if cal running, do not allow any events to occur besides stop event, thus ensuring multiple cals do not pile ontop of each other
  else if(calStatus == 1)
  {
    return;
  }
  //allow events as usual if cal not occuring
  else 
  {
    int index = Event.reportObject.index;
    if (Event.reportObject.object == GENIE_OBJ_SLIDER) //Sliders 0,1,2,3 to turn mfcs off and on
    {
      /*
         Slider 0-3: OnOff sliders for MFCs
      */
      if (index >= 0 && index <= 3)
      {
        OnOff(index, genie.GetEventData(&Event));
      }
    }
    else if (Event.reportObject.object == GENIE_OBJ_WINBUTTON)
    {
      /*
         WinButton 0: To gas configuration panel
         WinButton 1: To flow configuration panel
         WinButton 2: To calibration tool panel
         WinButton 3-6,19-22: MFC selection
         WinButton 7-16: Gas selection
         WinButton 23: Tech Info button
         WinButton 17,18,24: Confirm changes
         WinButton 25-29,31-35: MFC concnetration buttons
         WinButton 30,36: To custom incrementation panel
         WinButton 37-40,55: Relative humidity choices for calibration
         WinButton 41-44: Temperature choices for calibration
         WinButton 45-47: Time choices for calibration
         WinButton 48-50: Purge choices for calibration
         WinButton 51: Confirm and Begin for calibration
         WinButton 52: Stop calibration
         WinButton 53: Confirm Custom Increments for cali
         WinButton 54: Return to calibration tool panel
         WinButton 56,57: MFC 4 controller type selection
         WinButton 58: Mirror calibration selection
         WinButton 59: To PAMM ID page
      */
      if (index == 0) //button to gas configuration pressed
      {
        //reset currentmfc to 0 and change buttons correspondingly
        mfcSelectionGC(0);
        gasSelection(0);
        genie.WriteObject(GENIE_OBJ_WINBUTTON, 3, 1);
        genie.WriteObject(GENIE_OBJ_WINBUTTON, 7, 1);
        genie.WriteObject(GENIE_OBJ_FORM, 1, 0);
        if( mfc[3].stdFlowMax == 0.03)
        {
          genie.WriteObject(GENIE_OBJ_STRINGS, 38, 0);
        }
        else
        {
          genie.WriteObject(GENIE_OBJ_STRINGS, 38, 1);
        }
      }
      else if (index == 1) //button flow configuration pressed
      {
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 1, sysFlow * 100);
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 2, sysTemp);
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 3, sysPres * 100);
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 4, sysHum * 10);
        genie.WriteObject(GENIE_OBJ_FORM, 2, 0);
      }
      else if (index == 2) //button to calibration tool pressed
      {
        toCal();
      }
      else if (index >= 3 && index <= 6) //button within gasConfig to select mfc
      {
        mfcSelectionGC(index - 3);
      }
      else if (index >= 7 && index <= 16) //button within gasConfig to select gas
      {
        if ( currentMFC == 0 || currentMFC == 1) // these two mfc should not have their gases changed, as they are integral to system operation
        {
          genie.WriteObject(GENIE_OBJ_WINBUTTON, 7 + currentMFC * 5, 1); //reset active gas button pre-set air buttons
        }
        else
        {
          gasSelection(index - 7);
        }
      }
      else if (index == 17) //button to confirm gas config changes
      {
        updateSettingsGC(currentMFC);
      }
      else if (index == 18) //button to confirm system config changes
      {
        updateSettingsSys();
      }
      else if ( index >= 19 && index <= 22) //button within mfcConfig to select mfc
      {
        mfcSelectionMC(index - 19);
      }
      else if (index == 23) //button to collect technical data in mfc configuration
      {
        collectTechInfo();
      }
      else if (index == 24) //button to confirm mfc config changes
      {
        updateSettingsMC();
      }
      else if (index >= 25 && index <= 30) //buttons to choose mfc 4 calibration concentration
      {
        calMFCConc(3, index - 25);
      }
      else if (index >= 31 && index <= 36) //buttons to choose mfc 3 calibration concentration
      {
        calMFCConc(2, index - 31);
      }
      else if ((index >= 37 && index <= 40) || index == 55) //buttons to choose relative humidity in calibration
      {
        calRelHum(index - 37);
      }
      else if (index >= 41 && index <= 44) //buttons to choose temperature for calibration
      {
        calTemp(index - 41);
      }
      else if (index >= 45 && index <= 47) //buttons to choose time for each increment in calibration
      {
        calTime(index - 45);
      }
      else if (index >= 48 && index <= 50) //buttons to choose if calibration will have purge
      {
        calPurge(index - 48);
      }
      else if (index == 51) //button to confirm and begin calibration
      {
        beginCal();
      }
      else if (index == 52) //button to stop calibration
      {
        stopCal();
      }
      else if (index == 53) //button to confirm custom increment calibration
      {
        calConfirmCustomInc();
      }
      else if (index == 54) //return to calibration setup
      {
        toCalFromCustom();
      }
      else if (index == 56 || index == 57) //MFC type
      {
        mfcTypeSelection(index - 56);
      }
      else if (index == 58) //mirrored cal button
      {
        calMirrorInc(genie.GetEventData(&Event));
      }
      else if (index == 59) // to pamm id input screen
      {
        toPAMMid();
      }
      else if (index == 60) //delete last pamm id
      {
        deletePAMM();
      }
    }
    else if (Event.reportObject.object == GENIE_OBJ_4DBUTTON)
    {
      /*
         4DButton 0,2,3,4: Back to Main Menu
         4DButton 1: To individual mfc configuration
         4DButton 5: back to calibration setup
      */
      if (index == 0 || index == 2 || index == 3 || index == 4 ) //button to main menu pressed
      {
        toMain();
      }
      else if (index == 1) //button to mfc config pressed
      {
        toMFCconfig();
      }
      else if (index == 5) //button to calibration setup pressed
      {
        toCal();
      }
    }
    else if (Event.reportObject.object == GENIE_OBJ_KEYBOARD)
    {
      /*
         KeyBoard 1,10,11,12: +100000 Incrementor
         KeyBoard 0: +1000 Incrementor
         KeyBoard 8,9: +100 Incrementor
         KeyBoard 2,3,4,6: +10 Incrementor
         KeyBoard 5,7: +1 Incrementor
         KeyBoard 13: PAMM ID keyboard
      */
      switch (index) {
        case 0: //gas config tank concentration
          if (mfc[currentMFC].gas == 0 || mfc[currentMFC].gas == 5) //dry of humid air do not need  tank concentration
          {
            //do nothing
          }
          else
          {
            tankConcentration(genie.GetEventData(&Event));
          }
          break;
        case 1: //gas config flow concentration
          if (mfc[currentMFC].gas == 0 || mfc[currentMFC].gas == 5) //dry or humid air do not need  tank concentration
          {
            //do nothing
          }
          else
          {
            flowConcentration(genie.GetEventData(&Event));
          }
          break;
        case 2://gas config relative humidity
          if (mfc[currentMFC].gas == 5) //used for humid air only
          {
            humidityGC(genie.GetEventData(&Event));
          }
          else
          {
            //do nothing
          }
          break;
        case 3://flow config desired full system flow
          fullFlow(genie.GetEventData(&Event));
          break;
        case 4://flow config system temp
          sysT(genie.GetEventData(&Event));
          break;
        case 5://flow config system pressure
          sysP(genie.GetEventData(&Event));
          break;
        case 6://flow config system humidity
          humSys(genie.GetEventData(&Event));
          break;
        case 7://mfc config desired mfc flow
          //REMOVED.
          //Use to change individual mfc flow, but this is now controlled by function checkFlowBalance
          break;
        case 8://mfc signal offset
          mfcOffset(genie.GetEventData(&Event));
          break;
        case 9://mfc signal gain
          mfcGain(genie.GetEventData(&Event));
          break;
        case 10://cal, custom increments, initial concentration
          calICon(genie.GetEventData(&Event));
          break;
        case 11://cal, custom increments, final concnetration
          calFCon(genie.GetEventData(&Event));
          break;
        case 12://cal, custom increments, increment
          calIncrement(genie.GetEventData(&Event));
          break;
        case 13://pamm id input
          pammIDinput(genie.GetEventData(&Event));
          break;
        default:
          //do nothing
          break;
      }
    }
  }
}


