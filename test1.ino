#include <Arduino.h>
#include <Wire.h>
#define PIN_IRQ 7
#define I2C_ADD 0x11


enum AS3935_REGS : uint8_t
{
    AS3935_REGISTER_AFE_GB = 0x00,      //Analog Frontend Gain Boost
    AS3935_REGISTER_PWD = 0x00,       //Power Down
    AS3935_REGISTER_NF_LEV = 0x01,      //Noise Floor Level
    AS3935_REGISTER_WDTH = 0x01,      //Watchdog threshold
    AS3935_REGISTER_CL_STAT = 0x02,     //Clear statistics
    AS3935_REGISTER_MIN_NUM_LIGH = 0x02,  //Minimum number of lightnings
    AS3935_REGISTER_SREJ = 0x02,      //Spike rejection
    AS3935_REGISTER_LCO_FDIV = 0x03,    //Frequency division ratio for antenna tuning
    AS3935_REGISTER_MASK_DIST = 0x03,   //Mask Disturber
    AS3935_REGISTER_INT = 0x03,       //Interrupt
    AS3935_REGISTER_S_LIG_L = 0x04,     //Energy of the Single Lightning LSBYTE
    AS3935_REGISTER_S_LIG_M = 0x05,     //Energy of the Single Lightning MSBYTE
    AS3935_REGISTER_S_LIG_MM = 0x06,    //Energy of the Single Lightning MMSBYTE
    AS3935_REGISTER_DISTANCE = 0x07,    //Distance estimation
    AS3935_REGISTER_DISP_LCO = 0x08,    //Display LCO on IRQ pin
    AS3935_REGISTER_DISP_SRCO = 0x08,   //Display SRCO on IRQ pin
    AS3935_REGISTER_DISP_TRCO = 0x08,   //Display TRCO on IRQ pin
    AS3935_REGISTER_TUN_CAP = 0x08,     //Internal Tuning Capacitors (from 0 to 120pF in steps of 8pF)
    AS3935_REGISTER_TRCO_CALIB_DONE = 0x3A, //Calibration of TRCO done (1=successful)
    AS3935_REGISTER_TRCO_CALIB_NOK = 0x3A,  //Calibration of TRCO unsuccessful (1 = not successful)
    AS3935_REGISTER_SRCO_CALIB_DONE = 0x3B, //Calibration of SRCO done (1=successful)
    AS3935_REGISTER_SRCO_CALIB_NOK = 0x3B,  //Calibration of SRCO unsuccessful (1 = not successful)
    AS3935_REGISTER_PRESET_DEFAULT = 0x3C,  //Sets all registers in default mode
    AS3935_REGISTER_CALIB_RCO = 0x3D    //Sets all registers in default mode
};

  enum AS3935_register_mask_t : uint8_t
  {
    AS3935_MASK_AFE_GB =        0b00111110, //Analog Frontend Gain Boost
    AS3935_MASK_PWD =         0b00000001, //Power Down
    AS3935_MASK_NF_LEV =        0b01110000, //Noise Floor Level
    AS3935_MASK_WDTH =          0b00001111, //Watchdog threshold
    AS3935_MASK_CL_STAT =       0b01000000, //Clear statistics
    AS3935_MASK_MIN_NUM_LIGH =      0b00110000, //Minimum number of lightnings
    AS3935_MASK_SREJ =          0b00001111, //Spike rejection
    AS3935_MASK_LCO_FDIV =        0b11000000, //Frequency division ratio for antenna tuning
    AS3935_MASK_MASK_DIST =       0b00100000, //Mask Disturber
    AS3935_MASK_INT =         0b00001111, //Interrupt
    AS3935_MASK_S_LIG_L =       0b11111111, //Energy of the Single Lightning LSBYTE
    AS3935_MASK_S_LIG_M =       0b11111111, //Energy of the Single Lightning MSBYTE
    AS3935_MASK_S_LIG_MM =        0b00001111, //Energy of the Single Lightning MMSBYTE
    AS3935_MASK_DISTANCE =        0b00111111, //Distance estimation
    AS3935_MASK_DISP_LCO =        0b10000000, //Display LCO on IRQ pin
    AS3935_MASK_DISP_SRCO =       0b01000000, //Display SRCO on IRQ pin
    AS3935_MASK_DISP_TRCO =       0b00100000, //Display TRCO on IRQ pin
    AS3935_MASK_TUN_CAP =       0b00001111, //Internal Tuning Capacitors (from 0 to 120pF in steps of 8pF)
    AS3935_MASK_TRCO_CALIB_DONE =   0b10000000, //Calibration of TRCO done (1=successful)
    AS3935_MASK_TRCO_CALIB_NOK =    0b01000000, //Calibration of TRCO unsuccessful (1 = not successful)
    AS3935_MASK_SRCO_CALIB_DONE =   0b10000000, //Calibration of SRCO done (1=successful)
    AS3935_MASK_SRCO_CALIB_NOK =    0b01000000, //Calibration of SRCO unsuccessful (1 = not successful)
    AS3935_MASK_PRESET_DEFAULT =  0b11111111, //Sets all registers in default mode
    AS3935_MASK_CALIB_RCO =     0b11111111  //Sets all registers in default mode
  };

  enum afe_setting_t : uint8_t
  {
    AS3935_INDOORS = 0b10010,
    AS3935_OUTDOORS = 0b01110
  };

/* Figure 45: Frequency Division Ratio for the Antenna Tuning */
enum division_ratio_t : uint8_t
{
    AS3935_DR_16  = 0b00,
    AS3935_DR_32  = 0b01,
    AS3935_DR_64  = 0b10,
    AS3935_DR_128 = 0b11
};

#define AS3935_DIRECT_CMD  0x96
#define AS3935_TIMEOUT  2000



uint8_t readRegister(uint8_t reg)
{
  char buf[4];
  sprintf(buf, "R|%x", reg);
  Serial.println(buf);

  Wire.beginTransmission(I2C_ADD);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(I2C_ADD, static_cast<uint8_t>(1));
  return Wire.read();
}

void writeRegister(uint8_t reg, uint8_t value)
{
  char buf[8];
  sprintf(buf, "W|%x:%x", reg, value);
  Serial.println(buf);
   
  Wire.beginTransmission(I2C_ADD);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t getMaskShift(uint8_t mask)
{
  uint8_t return_value = 0;

  //count how many times the mask must be shifted right until the lowest bit is set
  if (mask != 0)
  {
    while (!(mask & 1))
    {
      return_value++;
      mask >>= 1;
    }
  }
  return return_value;
}

uint8_t setMaskedBits(uint8_t reg, uint8_t mask, uint8_t value)
{
  //clear mask bits in register
  reg &= (~mask);
  
  //set masked bits in register according to value
  return ((value << getMaskShift(mask)) & mask) | reg;
}

void writeRegisterValue(uint8_t reg, uint8_t mask, uint8_t value)
{
  uint8_t reg_val = readRegister(reg);
  char buf[28];
  sprintf(buf, "WRV|reg:%x m:%x v:%x rv:%x", reg, mask, value, reg_val);
  Serial.println(buf);
  writeRegister(reg, setMaskedBits(reg_val, mask, value));
}

  
uint8_t getMaskedBits(uint8_t reg, uint8_t mask)
{
  //extract masked bits
  return ((reg & mask) >> getMaskShift(mask));
}

uint8_t readRegisterValue(uint8_t reg, uint8_t mask)
{
  return getMaskedBits(readRegister(reg), mask);
}

uint8_t readAFE()
{
  return readRegisterValue(AS3935_REGISTER_AFE_GB, AS3935_MASK_AFE_GB);
}

bool checkConnection()
{
  uint8_t afe = readAFE();
  char buf[8];
  sprintf(buf, "afe:%x", afe);
  Serial.println(buf);

  return ((afe == AS3935_INDOORS) || (afe == AS3935_OUTDOORS));
}

void writeDivisionRatio(uint8_t ratio)
{
  /*
   * --------------------------------------------------------------------------------   
   * ||  #REG  ||   7   |   6   |     5     |   4   |   3   |   2   |   1   |   0   |
   * ||  0x03  ||    LCO_FDIV   | MASK_DIST |       |              INT              |
   * --------------------------------------------------------------------------------
  */

  /*
   * Antenna Tuning
   * 
   * The AS3935 uses a loop antenna based on a parallel LC resonator.
   * The antenna has to be designed to have its resonance frequency at 500kHz.
   * By setting the register AS3935_REGISTER_DISP_LCO REG0x08[7] = 1 the antenna's resonance 
   * frequency is displayed on the IRQ pin as a digital signal.
   */
  writeRegisterValue(AS3935_REGISTER_LCO_FDIV, AS3935_MASK_LCO_FDIV, ratio);
}

bool checkIRQ()
{
#if 1
  int16_t target = 6250;    //500kHz / 16 * 0.1s * 2 (counting each high-low / low-high transition)
  int16_t best_diff_abs = 32767;
  uint8_t best_i = 0;
  
  writeDivisionRatio(AS3935_DR_16);
  delayMicroseconds(AS3935_TIMEOUT);

  //display LCO on IRQ
  writeRegisterValue(AS3935_REGISTER_DISP_LCO, AS3935_MASK_DISP_LCO, 1);
  
  bool irq_current = digitalRead(PIN_IRQ);
  bool irq_last = irq_current;

  int16_t counts = 0;

  Serial.println("checkIRQ::Start");

  uint32_t time_start = millis();

   //count transitions for 10ms
  while ((millis() - time_start) < 10)
  {
    irq_current = digitalRead(PIN_IRQ);

    if (irq_current != irq_last)
      counts++;

    irq_last = irq_current;
  }
  char buf[16];
  sprintf(buf, "counts:%d", counts);
  Serial.println(buf);

  return true;
#else
  




  //stop displaying LCO on IRQ
  writeRegisterValue(AS3935_REGISTER_DISP_LCO, AS3935_MASK_DISP_LCO, 0);

  delayMicroseconds(AS3935_TIMEOUT);

  //return true if at least 100 transition was detected (to prevent false positives). 
  return (counts > 100);
#endif
}

void setup()
{
  Serial.begin(9600);

  //wait for serial connection to open (only necessary on some boards)
  while (!Serial);

  //set the IRQ pin as an input pin. do not use INPUT_PULLUP - the AS3935 will pull the pin 
  //high if an event is registered.
  pinMode(PIN_IRQ, INPUT);

  Wire.begin();

  writeRegister(AS3935_REGISTER_PRESET_DEFAULT, AS3935_DIRECT_CMD);
  delayMicroseconds(AS3935_TIMEOUT);

  //check I2C connection.
  if (checkConnection())
  {
    Serial.println("checkConnection() failed. check your I2C connection and I2C Address. ");
    while (1);
  }
  else
  {
    Serial.println("I2C connection check passed. ");
  }

  if (!checkIRQ())
  {
    Serial.println("checkIRQ() failed. check if the correct IRQ pin was passed to the AS3935I2C constructor. ");
 
    while (1);
  }
  else
  {
    Serial.println("IRQ pin connection check passed. ");
  }  
}

void loop() {
  // put your main code here, to run repeatedly:

}
