#include "i2c.h"
#include "vl53l0x.h"
#include "delay.h"
#include "uart.h"
#include "stdio.h"

//uint16_t bswap(u8 b[])
//{
//	uint16_t val = ((b[0]<< 8) & b[1]);
//	return val;
//}
//
//uint16_t VL53L0X_decode_vcsel_period(short vcsel_period_reg)
//{
//	uint16_t vcsel_period_pclks = (vcsel_period_reg + 1) <<1;
//	return vcsel_period_pclks;
//}
//
/*
*发送一个字节的数据到指定寄存器
*/
void VL53L0X_Write_Byte(unsigned char reg,unsigned char dat) 				 
{ 
    start(); 
	i2c_write((VL53L0X_Add<<1)|0);	
	ack();
    i2c_write(reg);	
    ack();
	i2c_write(dat);
	ack();	 
    stop();	 
}
/*
*读指定寄存器的值
*/
unsigned char VL53L0X_Read_Byte(unsigned char reg)
{
	unsigned char  res;
    start(); 

	i2c_write(0x52);
	ack();

    i2c_write(reg);	
   	ack();

	delay();

    start(); 
	i2c_write(0x53);
    ack();

	res=i2c_read();

	scl=0;
	sda=1;
	delay();
	scl=1;
	delay();
	scl=0;

    stop();	
	return res;		
}
/*
*为了快速移植，将写一个字节的函数重命名一下
*/
void writeReg(unsigned char reg,unsigned char dat)
{
	VL53L0X_Write_Byte(reg,dat);
}

/*
*为了快速移植，将读指定寄存器的函数重命名一下
*/
unsigned char readReg(unsigned char reg)
{
	return VL53L0X_Read_Byte(reg);
}

/*
*一次性发送多个字节
*/
void VL53L0X_Write_Len(unsigned char addr,unsigned char reg,unsigned char len,unsigned char *buf)
{
	unsigned char i; 
    start(); 
	i2c_write((addr<<1)|0);	
	 ack();
    i2c_write(reg);	
    	 ack();
	for(i=0;i<len;i++)
	{
		i2c_write(buf[i]);
		  ack();
	}    
    stop();	 	
}
/*
*一次性读取多个字节
*/
void VL53L0X_Read_Len(unsigned char addr,unsigned char reg,unsigned char len,unsigned char *buf)
{ 
 	start(); 
	i2c_write((addr<<1)|0);
	 ack();
    i2c_write(reg);	
    ack();
    start();
	i2c_write((addr<<1)|1);	
    ack(); 
	while(len)
	{
		if(len==1) 
		{
		   *buf=i2c_read();
			scl=0;
			sda=1;
			delay();
			scl=1;
			delay();
			scl=0;
	      }
		else
		  {
		    *buf=i2c_read();
			scl=0;
		
			sda=0;
			delay();
			scl=1;
			delay();
			scl=0;

		  }	
		len--;
		buf++; 
	}    
    stop();
}

/*
*为了快速移植，将写多个字节的函数简单封装
*/
void WriteMulti(unsigned char reg,unsigned char *dats,unsigned char num)
{
	VL53L0X_Write_Len(VL53L0X_Add,reg,num,dats);
}
/*
*为了快速移植，将读多个字节的函数简单封装
*/
void ReadMulti(unsigned char reg, unsigned char *dats, unsigned char num){
	VL53L0X_Read_Len(VL53L0X_Add,reg,num,dats);
}
/*
*一次写一个word,两个字节
*/

unsigned char WriteWord(unsigned char reg,unsigned int word)
{
	char i=0;
	unsigned char buf[2];
	for(i=0;i<2;i++)
	{
		buf[1-i] = (word >> i*8)&0xff;
	}
	VL53L0X_Write_Len(VL53L0X_Add,reg,2,buf);
	return 0;
}
/*
*一次读一个word,两个字节
*/
long ReadWord(unsigned char reg)
{
  xdata char buf[2];
  xdata long ret = 0;
  VL53L0X_Read_Len(VL53L0X_Add,reg,2,buf);
  ret = buf[2]*256+buf[3];
  return ret;
}

/*
*-----以上为封装的工具函数
*-----以下为vl53l0x的操作
*/

#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((unsigned long)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)
typedef enum { VcselPeriodPreRange, VcselPeriodFinalRange }vcselPeriodType;
 

xdata unsigned char stop_variable;
xdata unsigned long measurement_timing_budget_us;


unsigned int makeuint16(int lsb, int msb)
{
    return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
unsigned int decodeTimeout(unsigned int reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (unsigned int)((reg_val & 0x00FF) <<
         (unsigned int)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
unsigned int encodeTimeout(unsigned int timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

  xdata unsigned long ls_byte = 0;
  xdata unsigned int ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}


// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
unsigned long timeoutMclksToMicroseconds(unsigned int timeout_period_mclks, unsigned char vcsel_period_pclks)
{
  unsigned long macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
unsigned long timeoutMicrosecondsToMclks(unsigned long timeout_period_us, unsigned char vcsel_period_pclks)
{
  unsigned long macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}


unsigned char setSignalRateLimit(float limit_Mcps)
{
  if (limit_Mcps < 0 || limit_Mcps > 512) { return -1; }
  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  WriteWord(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
  return 0;
}


unsigned char getSpadInfo(unsigned char * count, unsigned char * type_is_aperture)
{
  xdata unsigned int tmp;

  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);

  writeReg(0xFF, 0x06);
  writeReg(0x83, readReg(0x83) | 0x04);
  writeReg(0xFF, 0x07);
  writeReg(0x81, 0x01);

  writeReg(0x80, 0x01);

  writeReg(0x94, 0x6b);
  writeReg(0x83, 0x00);
  //startTimeout();
  while (readReg(0x83) == 0x00)
  {
	  delay_ms(1);
//    if (checkTimeoutExpired()) { return FALSE; }
  }
  writeReg(0x83, 0x01);
  tmp = readReg(0x92);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  writeReg(0x81, 0x00);
  writeReg(0xFF, 0x06);
  writeReg(0x83, readReg(0x83)  & ~0x04);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x01);

  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  return 0;
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
void getSequenceStepEnables(SequenceStepEnables * enables)
{
  xdata unsigned char sequence_config = readReg(SYSTEM_SEQUENCE_CONFIG);

  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based 53L0X_get_vcsel_pulse_period()
unsigned char getVcselPulsePeriod(vcselPeriodType type)
{
  if (type == VcselPeriodPreRange)
  {
    return decodeVcselPeriod(readReg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else if (type == VcselPeriodFinalRange)
  {
    return decodeVcselPeriod(readReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else { return 255; }
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{
  xdata unsigned char tmpH,tmpL;
  timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodPreRange);

  timeouts->msrc_dss_tcc_mclks = readReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us =
  timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  tmpH = readReg(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI); 
  tmpL = readReg(PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO); 
  timeouts->pre_range_mclks = decodeTimeout((u16)(tmpH<<8|tmpL));
  timeouts->pre_range_us =
  timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodFinalRange);

  tmpH = readReg(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI); 
  tmpL = readReg(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO); 

  timeouts->final_range_mclks = decodeTimeout((u16)(tmpH<<8|tmpL));

  if (enables->pre_range)
  {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us = timeoutMclksToMicroseconds(timeouts->final_range_mclks,timeouts->final_range_vcsel_period_pclks);
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
unsigned char setMeasurementTimingBudget(unsigned long budget_us)
{
  xdata SequenceStepEnables enables;
  xdata SequenceStepTimeouts timeouts;

  xdata unsigned int const StartOverhead      = 1320; // note that this is different than the value in get_
  xdata unsigned int const EndOverhead        = 960;
  xdata unsigned int const MsrcOverhead       = 660;
  xdata unsigned int const TccOverhead        = 590;
  xdata unsigned int const DssOverhead        = 690;
  xdata unsigned int const PreRangeOverhead   = 660;
  xdata unsigned int const FinalRangeOverhead = 550;

  xdata unsigned long const MinTimingBudget = 20000;
  xdata unsigned long used_budget_us;
  xdata unsigned long final_range_timeout_us;
  xdata unsigned int final_range_timeout_mclks;

  if (budget_us < MinTimingBudget) { return -1; }

  used_budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    if (used_budget_us > budget_us)
    {
      // "Requested timeout too big."
      return -1;
    }

    final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(final_range_timeout_us,
                                 timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range)
    {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

	
    WriteWord(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(final_range_timeout_mclks));

    // set_sequence_step_timeout() end

    measurement_timing_budget_us = budget_us; // store for internal reuse
  }
  return 0;
}

unsigned long getMeasurementTimingBudget(void)
{
  xdata SequenceStepEnables enables;
  xdata SequenceStepTimeouts timeouts;

  xdata unsigned int  const StartOverhead     = 1910; // note that this is different than the value in set_
  xdata unsigned int  const EndOverhead        = 960;
  xdata unsigned int  const MsrcOverhead       = 660;
  xdata unsigned int  const TccOverhead        = 590;
  xdata unsigned int  const DssOverhead        = 690;
  xdata unsigned int  const PreRangeOverhead   = 660;
  xdata unsigned int  const FinalRangeOverhead = 550;

  // "Start and end overhead times always present"
  unsigned long  budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);
  }

  measurement_timing_budget_us = budget_us; // store for internal reuse
  return budget_us;
}

// based on VL53L0X_perform_single_ref_calibration()
unsigned char performSingleRefCalibration(unsigned char vhv_init_byte)
{
  writeReg(SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

  //startTimeout();
  while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    //if (checkTimeoutExpired()) { return FALSE; }
  }

  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  writeReg(SYSRANGE_START, 0x00);

  return 0;
}

unsigned char VL53L0X_init(unsigned char io_2v8)
{
	xdata unsigned char tmp;	
	xdata unsigned char spad_count;
	xdata unsigned char spad_type_is_aperture;
	xdata unsigned char ref_spad_map[6];
	xdata unsigned char first_spad_to_enable;
	xdata unsigned char spads_enabled = 0;
	xdata unsigned char i;
	
  // VL53L0X_DataInit() begin
  
  // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
	if (io_2v8)
	{
		tmp = readReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV);
		writeReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,	 tmp| 0x01); // set bit 0
	}
  
  // "Set I2C standard mode"
  writeReg(0x88, 0x00);
  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  stop_variable = readReg(0x91);


  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);
 
  // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
  writeReg(MSRC_CONFIG_CONTROL, readReg(MSRC_CONFIG_CONTROL) | 0x12);

  // set final range signal rate limit to 0.25 MCPS (million counts per second)
  setSignalRateLimit(0.25);
  writeReg(SYSTEM_SEQUENCE_CONFIG, 0xFF);

  // VL53L0X_DataInit() end

  // VL53L0X_StaticInit() begin

  if (getSpadInfo(&spad_count, &spad_type_is_aperture)<0) { return -1; }

  // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
  // the API, but the same data seems to be more easily readable from
  // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
  ReadMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

  writeReg(0xFF, 0x01);
  writeReg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
  writeReg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
  writeReg(0xFF, 0x00);
  writeReg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

  first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
  

  for (i = 0; i < 48; i++)
  {
    if (i < first_spad_to_enable || spads_enabled == spad_count)
    {
      // This bit is lower than the first one that should be enabled, or
      // (reference_spad_count) bits have already been enabled, so zero this bit
      ref_spad_map[i / 8] &= ~(1 << (i % 8));
    }
    else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
    {
      spads_enabled++;
    }
  }


  WriteMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
  ReadMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // -- VL53L0X_set_reference_spads() end

  // -- VL53L0X_load_tuning_settings() begin
  // DefaultTuningSettings from vl53l0x_tuning.h
  
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);

  writeReg(0xFF, 0x00);
  writeReg(0x09, 0x00);
  writeReg(0x10, 0x00);
  writeReg(0x11, 0x00);

  writeReg(0x24, 0x01);
  writeReg(0x25, 0xFF);
  writeReg(0x75, 0x00);

  writeReg(0xFF, 0x01);
  writeReg(0x4E, 0x2C);
  writeReg(0x48, 0x00);
  writeReg(0x30, 0x20);
  
  
  writeReg(0xFF, 0x00);
  writeReg(0x30, 0x09);
  writeReg(0x54, 0x00);
  writeReg(0x31, 0x04);
  
  writeReg(0x32, 0x03);
  writeReg(0x40, 0x83);
  writeReg(0x46, 0x25);
  writeReg(0x60, 0x00);
  writeReg(0x27, 0x00);
  writeReg(0x50, 0x06);
  writeReg(0x51, 0x00);
  writeReg(0x52, 0x96);
  writeReg(0x56, 0x08);
  writeReg(0x57, 0x30);
  writeReg(0x61, 0x00);
  writeReg(0x62, 0x00);
  writeReg(0x64, 0x00);
  writeReg(0x65, 0x00);
  writeReg(0x66, 0xA0);
  


  writeReg(0xFF, 0x01);
  writeReg(0x22, 0x32);
  writeReg(0x47, 0x14);
  writeReg(0x49, 0xFF);
  writeReg(0x4A, 0x00);

  writeReg(0xFF, 0x00);
  writeReg(0x7A, 0x0A);
  writeReg(0x7B, 0x00);
  writeReg(0x78, 0x21);

  writeReg(0xFF, 0x01);
  writeReg(0x23, 0x34);
  writeReg(0x42, 0x00);
  writeReg(0x44, 0xFF);
  writeReg(0x45, 0x26);
  writeReg(0x46, 0x05);
  writeReg(0x40, 0x40);
  writeReg(0x0E, 0x06);
  writeReg(0x20, 0x1A);
  writeReg(0x43, 0x40);

  writeReg(0xFF, 0x00);
  writeReg(0x34, 0x03);
  writeReg(0x35, 0x44);

  writeReg(0xFF, 0x01);
  writeReg(0x31, 0x04);
  writeReg(0x4B, 0x09);
  writeReg(0x4C, 0x05);
  writeReg(0x4D, 0x04);

  writeReg(0xFF, 0x00);
  writeReg(0x44, 0x00);
  writeReg(0x45, 0x20);
  writeReg(0x47, 0x08);
  writeReg(0x48, 0x28);
  writeReg(0x67, 0x00);
  writeReg(0x70, 0x04);
  writeReg(0x71, 0x01);
  writeReg(0x72, 0xFE);
  writeReg(0x76, 0x00);
  writeReg(0x77, 0x00);

  writeReg(0xFF, 0x01);
  writeReg(0x0D, 0x01);

  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x01);
  writeReg(0x01, 0xF8);

  writeReg(0xFF, 0x01);
  writeReg(0x8E, 0x01);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);
  Uart2SendString("step 5\r\n");
  
  // -- VL53L0X_load_tuning_settings() end

  // "Set interrupt config to new sample ready"
  // -- VL53L0X_SetGpioConfig() begin
  
  writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  tmp = readReg(GPIO_HV_MUX_ACTIVE_HIGH);
  writeReg(GPIO_HV_MUX_ACTIVE_HIGH,  tmp& ~0x10); // active low
  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  // -- VL53L0X_SetGpioConfig() end
  measurement_timing_budget_us = getMeasurementTimingBudget();
  // "Disable MSRC and TCC by default"
  // MSRC = Minimum Signal Rate Check
  // TCC = Target CentreCheck
  // -- VL53L0X_SetSequenceStepEnable() begin

  writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // -- VL53L0X_SetSequenceStepEnable() end

  // "Recalculate timing budget"
  setMeasurementTimingBudget(measurement_timing_budget_us);
  // VL53L0X_StaticInit() end

  // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

  // -- VL53L0X_perform_vhv_calibration() begin

  writeReg(SYSTEM_SEQUENCE_CONFIG, 0x01);
  if (performSingleRefCalibration(0x40)) { return -1; }

  // -- VL53L0X_perform_vhv_calibration() end

  // -- VL53L0X_perform_phase_calibration() begin

  writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
  if (performSingleRefCalibration(0x00)) { return -1; }

  // -- VL53L0X_perform_phase_calibration() end

  // "restore the previous Sequence Config"
  writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // VL53L0X_PerformRefCalibration() end
   
  return 0;

}

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
void startContinuous(unsigned char period_ms)
{
  xdata unsigned char tmp[4];
  xdata unsigned int  osc_calibrate_val;
  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  writeReg(0x91, stop_variable);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  if (period_ms != 0)
  {
    // continuous timed mode

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

    osc_calibrate_val = ReadWord(OSC_CALIBRATE_VAL);

    if (osc_calibrate_val != 0)
    {
      period_ms *= osc_calibrate_val;
    }
	tmp[0] = period_ms&0xFF;
	tmp[1] = period_ms>>8&0xFF;
	tmp[2] = period_ms>>16&0xFF;
	tmp[3] = period_ms>>24&0xFF;
    WriteMulti(SYSTEM_INTERMEASUREMENT_PERIOD, tmp, 4);

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

    writeReg(SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
  }
  else
  {
    // continuous back-to-back mode
    writeReg(SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
  }
}

unsigned int readRangeContinuousMillimeters(void)
{
/*
  startTimeout();
  while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    if (checkTimeoutExpired())
    {
      did_timeout = true;
      return 65535;
    }
  }
*/
  // assumptions: Linearity Corrective Gain is 1000 (default);
  // fractional ranging is not enabled
  
  unsigned int range = 0;
  range = readReg(RESULT_RANGE_STATUS + 10)<<8;
  range |= readReg(RESULT_RANGE_STATUS + 11);

  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  return range;
}


