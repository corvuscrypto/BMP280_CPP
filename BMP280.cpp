

 #define F_CPU 12000000UL

 #include "BMP280.h"
 #include <util/delay.h>
 #include <avr/io.h>
 #include <math.h>
//Constructor for BMP280 object
BMP280::BMP280()
{

}

//Initialize BMP280 sensor and grab calibration and chip ID parameters
bool BMP280::initialize(uint8_t power_mode, uint8_t filter_mode)
{



	//bool com_result = readData(BMP280_CHIP_ID_REG, &v_byte_holder, 1);
	//chip_id = readData(BMP280_CHIP_ID_REG,1);


		PORTB &= ~(1<<PB2);

		SPIxfer(0xD0|0x80);
		chip_id = SPIxfer(0x00);



		PORTB |= (1<<PB2);

		if(chip_id!=0x58)
		{ return false;}


	bool com_result = getCalParams();

	set_standby(BMP280_STANDBY_TIME_63_MS);
	com_result &= set_work_mode(power_mode);
		com_result &= set_filter(filter_mode);
		com_result &= set_power_mode(BMP280_NORMAL_MODE);



	return com_result;

}

bool BMP280::set_standby(uint8_t v_standby_durn_u8)
{
	/* variable used to return communication result*/

	uint8_t v_data_u8 = 0;

	/* write the standby duration*/
	v_data_u8= readData(BMP280_CONFIG_REG_STANDBY_DURN__REG, 1);

	v_data_u8 =	BMP280_SET_BITSLICE(v_data_u8, BMP280_CONFIG_REG_STANDBY_DURN,	v_standby_durn_u8);
	writeData(BMP280_CONFIG_REG_STANDBY_DURN__REG,v_data_u8);


	return true;
}

bool BMP280::getCalParams()
{

	bool com_succeed = true;

	uint8_t a_data_u8 [24] = {0,0,0,0,0,0,0,0,
							0,0,0,0,0,0,0,0,
							0,0,0,0,0,0,0,0};

	//read the calibration parameters from the sensor's registers

	for(uint8_t i=0;i<24;i++)
	{

		a_data_u8[i]=readData(0x88+i,1);

	}

	//use manufacturer's bit combinations

	calibration_params.T1 = (uint16_t)((((uint16_t)((uint8_t)a_data_u8[1])) << 8)| a_data_u8[0]);
	calibration_params.T2 = (int16_t)((((int16_t)((int8_t)a_data_u8[3])) << 8)| a_data_u8[2]);
	calibration_params.T3 = (int16_t)((((int16_t)((int8_t)a_data_u8[5])) << 8)| a_data_u8[4]);
	calibration_params.P1 = (uint16_t)((((uint16_t)((uint8_t)a_data_u8[7])) << 8)| a_data_u8[6]);
	calibration_params.P2 = (int16_t)((((int16_t)((int8_t)a_data_u8[9])) << 8)| a_data_u8[8]);
	calibration_params.P3 = (int16_t)((((int16_t)((int8_t)a_data_u8[11]))<< 8)| a_data_u8[10]);
	calibration_params.P4 = (int16_t)((((int16_t)((int8_t)a_data_u8[13]))<< 8)| a_data_u8[12]);
	calibration_params.P5 = (int16_t)((((int16_t)((int8_t)a_data_u8[15]))<< 8)| a_data_u8[14]);
	calibration_params.P6 = (int16_t)((((int16_t)((int8_t)a_data_u8[17]))<< 8)| a_data_u8[16]);
	calibration_params.P7 = (int16_t)((((int16_t)((int8_t)a_data_u8[19]))<< 8)| a_data_u8[18]);
	calibration_params.P8 = (int16_t)((((int16_t)((int8_t)a_data_u8[21]))<< 8)| a_data_u8[20]);
	calibration_params.P9 = (int16_t)((((int16_t)((int8_t)a_data_u8[23]))<< 8)| a_data_u8[22]);

	return com_succeed;

}

int32_t BMP280::read_uncomp_temp()
{



	uint8_t buffer [3] = {0,0,0};

	uint32_t lghold;
	//read the data from the sensor's registers
	while((readData(0xF3,1)&0x08)!=0x08);
	lghold = readData(BMP280_TEMPERATURE_MSB_REG, 3);

	buffer[0]=(uint8_t)(lghold>>16);
	buffer[1]=(uint8_t)(lghold>>8);
	buffer[2]=(uint8_t)lghold;

	//combine bits for uncompensated temp

	int32_t result = (int32_t)((((uint32_t)(buffer[0]))  << 12)
					|(((uint32_t)(buffer[1])) << 4)
					|((uint32_t)buffer[2] >> 4));

	return result;
}

double BMP280::compensate_temp()
{

	int32_t v_uncomp_temperature = read_uncomp_temp();
	double v_x1_u32r = 0;
	double v_x2_u32r = 0;
	double temperature = 0;
	/* calculate x1*/
	v_x1_u32r  = (((double)v_uncomp_temperature) / 16384.0 -
	((double)calibration_params.T1) / 1024.0) *
	((double)calibration_params.T2);
	/* calculate x2*/
	v_x2_u32r  = ((((double)v_uncomp_temperature) / 131072.0 -
	((double)calibration_params.T1) / 8192.0) *
	(((double)v_uncomp_temperature) / 131072.0 -
	((double)calibration_params.T1) / 8192.0)) *
	((double)calibration_params.T3);
	/* calculate t_fine*/
	calibration_params.t_fine = (int32_t)(v_x1_u32r + v_x2_u32r);
	/* calculate true temperature*/
	temperature  = (v_x1_u32r + v_x2_u32r) / 5120.0;


	return temperature;

}

int32_t BMP280::read_uncomp_pressure()
{

	uint8_t buffer [3] = {0,0,0};

	uint32_t lghold;
	//read the data from the sensor's registers

	lghold = readData(BMP280_PRESSURE_MSB_REG, 3);

	buffer[0]=(uint8_t)(lghold>>16);
	buffer[1]=(uint8_t)(lghold>>8);
	buffer[2]=(uint8_t)lghold;

	//combine bits for uncompensated temp

	int32_t result = (int32_t)((((uint32_t)(buffer[0]))  << 12)
	|(((uint32_t)(buffer[1])) << 4)
	|((uint32_t)buffer[2] >> 4));

	return result;

}

double BMP280::compensate_pressure()
{

	compensate_temp();
	int32_t v_uncomp_pressure = read_uncomp_pressure();

	double v_x1_u32r = 0;
	double v_x2_u32r = 0;
	double pressure = 0;

	v_x1_u32r = ((double)calibration_params.t_fine/2.0) - 64000.0;
	v_x2_u32r = v_x1_u32r * v_x1_u32r *
	((double)calibration_params.P6) / 32768.0;
	v_x2_u32r = v_x2_u32r + v_x1_u32r *
	((double)calibration_params.P5) * 2.0;
	v_x2_u32r = (v_x2_u32r / 4.0) +
	(((double)calibration_params.P4) * 65536.0);
	v_x1_u32r = (((double)calibration_params.P3) *
	v_x1_u32r * v_x1_u32r / 524288.0 +
	((double)calibration_params.P2) * v_x1_u32r) / 524288.0;
	v_x1_u32r = (1.0 + v_x1_u32r / 32768.0) *
	((double)calibration_params.P1);
	pressure = 1048576.0 - (double)v_uncomp_pressure;
	/* Avoid exception caused by division by zero */
	if (v_x1_u32r != 0.0)
		pressure = (pressure - (v_x2_u32r / 4096.0)) *
		6250.0 / v_x1_u32r;
	else
		return 0;
	v_x1_u32r = ((double)calibration_params.P9) *
	pressure * pressure / 2147483648.0;
	v_x2_u32r = pressure * ((double)calibration_params.P8) / 32768.0;
	pressure = pressure + (v_x1_u32r + v_x2_u32r +
	((double)calibration_params.P7)) / 16.0;

	return pressure;
}

bool BMP280::set_power_mode(uint8_t v_power_mode)
{
	/* variable used to return communication result*/
	bool com_rslt = true;
	uint8_t v_byte_holder = 0;

	if (v_power_mode <= BMP280_NORMAL_MODE) {
				/* write the power mode*/
				v_byte_holder = (oversamp_temperature << 5) +
				(oversamp_pressure << 2)
				+ v_power_mode;

				com_rslt &= writeData(BMP280_CTRL_MEAS_REG_POWER_MODE__REG, v_byte_holder);
			} else {
			com_rslt = false;
			}

	return com_rslt;
}

bool BMP280::set_filter(uint8_t v_value_u8)
{
	bool com_rslt = true;
	uint8_t v_data_u8 = readData(BMP280_CONFIG_REG_FILTER__REG,1);


				v_data_u8 = BMP280_SET_BITSLICE(v_data_u8,BMP280_CONFIG_REG_FILTER,v_value_u8);
				com_rslt &=	writeData(BMP280_CONFIG_REG_FILTER__REG,v_data_u8);

	return com_rslt;
}
bool BMP280::set_work_mode(uint8_t v_work_mode_u8)
{
/* variable used to return communication result*/
bool com_rslt = true;
uint8_t v_data_u8 = 0;


	if (v_work_mode_u8 <= BMP280_ULTRA_HIGH_RESOLUTION_MODE) {
		v_data_u8= readData(BMP280_CTRL_MEAS_REG,1);
		if (com_rslt == true) {
			switch (v_work_mode_u8) {
			/* write work mode*/
			case BMP280_ULTRA_LOW_POWER_MODE:
				oversamp_temperature = BMP280_ULTRALOWPOWER_OVERSAMP_TEMPERATURE;
				oversamp_pressure =	BMP280_ULTRALOWPOWER_OVERSAMP_PRESSURE;
				break;
			case BMP280_LOW_POWER_MODE:
				oversamp_temperature = BMP280_LOWPOWER_OVERSAMP_TEMPERATURE;
				oversamp_pressure =	BMP280_LOWPOWER_OVERSAMP_PRESSURE;
				break;
			case BMP280_STANDARD_RESOLUTION_MODE:
				oversamp_temperature = BMP280_STANDARDRESOLUTION_OVERSAMP_TEMPERATURE;
				oversamp_pressure =	BMP280_STANDARDRESOLUTION_OVERSAMP_PRESSURE;
				break;
			case BMP280_HIGH_RESOLUTION_MODE:
				oversamp_temperature = BMP280_HIGHRESOLUTION_OVERSAMP_TEMPERATURE;
				oversamp_pressure =	BMP280_HIGHRESOLUTION_OVERSAMP_PRESSURE;
				break;
			case BMP280_ULTRA_HIGH_RESOLUTION_MODE:
				oversamp_temperature = BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_TEMPERATURE;
				oversamp_pressure =	BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_PRESSURE;
				break;
			}

			v_data_u8 = BMP280_SET_BITSLICE(v_data_u8,BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE,oversamp_temperature);
			v_data_u8 = BMP280_SET_BITSLICE(v_data_u8,BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE,oversamp_pressure);
			com_rslt &= writeData(BMP280_CTRL_MEAS_REG,v_data_u8);
		}
	} else {
	com_rslt = false;
	}

return com_rslt;
}

void BMP280::set_zero()
{

	pressure_base = compensate_pressure();
	_delay_ms(5);
	pressure_base += compensate_pressure();
		_delay_ms(5);
	pressure_base += compensate_pressure();
		_delay_ms(5);
	pressure_base += compensate_pressure();
		_delay_ms(5);
	pressure_base += compensate_pressure();


	pressure_base /= 5;
}

double BMP280::get_altitude()
{


	double press = compensate_pressure();


	double feet = (double)(145439.6372*(1.0-pow((press/pressure_base),0.1903)));

	return feet;

}

uint8_t BMP280::SPIxfer(uint8_t byte)
{

	SPDR = byte;

	while ( !(SPSR & _BV(SPIF)) );

	return SPDR;

}

uint32_t BMP280::readData(uint8_t addr, uint8_t length)
{

	uint8_t buffer [4];

		PORTB &= ~(1<<PB2);

	   SPIxfer(addr|0x80);

	   for(int i=0; i<length; i++){
		buffer[i] = SPIxfer(0x00);
	   }

		PORTB |= (1<<PB2);

		uint32_t result=0;

		for(int i=0; i<length;i++){

		result <<=8;
		result |= buffer[i];
		}

		return result;

}

uint16_t BMP280::readCal(uint8_t addr)
{

	uint8_t buffer [2];

	PORTB &= ~(1<<PB2);

	SPIxfer(addr|0x80);

	for(int i=0; i<2; i++){
		buffer[i] = SPIxfer(0x00);
	}

	PORTB |= (1<<PB2);

	uint16_t result=0;

	for(int i=0; i<2; i++){

		result |=  buffer[i]<<(8*i);

	}

	return result;


}


bool BMP280::writeData(uint8_t addr, uint8_t data)
{




	PORTB &= ~(1<<PB2);

	SPIxfer(addr&0x7F);
	SPIxfer(data);

	PORTB |= (1<<PB2);

	return true;

}
