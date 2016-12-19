#include "BMP180.h"

void	BMP180::begin(float Altitude){
//			подключаемся к шине I2C как мастер
			Wire.begin();
//			Читаем Chip-id (константа у всех чипов BMP180 = 0x55) из EEPROM (Electrically Erasable Programmable Read-Only Memory)
			ID  = ReadReg(1, 0xD0, false);				//	1 байт из регистра 0xD0 (без знака)
		if (ID == 0x55){								//	если ID != 0x55, значит на шине IIC устройство с адресом 0x77 отсутствует или не является нашим барометром
//			Читаем калибровочные коэффициенты (константы для корректировки показаний температуры и давления) из EEPROM
			AC1	= ReadReg(2, 0xAA);						//	2байта из регистров 0xAA 0xAB
			AC2	= ReadReg(2, 0xAC);						//	2байта из регистров 0xAC 0xAD
			AC3	= ReadReg(2, 0xAE);						//	2байта из регистров 0xAE 0xAF
			AC4	= ReadReg(2, 0xB0, false);				//	2байта из регистров 0xB0 0xB1 (без знака)
			AC5	= ReadReg(2, 0xB2, false);				//	2байта из регистров 0xB2 0xB3 (без знака)
			AC6	= ReadReg(2, 0xB4, false);				//	2байта из регистров 0xB4 0xB5 (без знака)
			B_1	= ReadReg(2, 0xB6);						//	2байта из регистров 0xB6 0xB7
			B_2	= ReadReg(2, 0xB8);						//	2байта из регистров 0xB8 0xB9
			MB	= ReadReg(2, 0xBA);						//	2байта из регистров 0xBA 0xBB
			MC	= ReadReg(2, 0xBC);						//	2байта из регистров 0xBC 0xBD
			MD	= ReadReg(2, 0xBE);						//	2байта из регистров 0xBE 0xBF
			SLP	= 0; read();							//	инициируем чтение для получения текущего давления
			SLP	= pres/pow(1-(Altitude/44330),5.255);	//	расчетное давление на уровне моря в мм.рт.ст.
		}
}

boolean	BMP180::read(uint8_t OSS){
			if(ID!=0x55){return false;}		ErrData=false;
			WriteReg(0xF4, 0x2E);			DelayFlagSCO();	UT=ReadReg(2, 0xF6);			//	Читаем "сырую" температуру:	записываем в регистр 0xF4 значение 0x2E, ждем флаг SCO, читаем 2байта из регистров 0xF6 0xF7
			WriteReg(0xF4, 0x34+(OSS<<6));	DelayFlagSCO();	UP=ReadReg(3, 0xF6)>>(8-OSS);	//	Читаем "сырое" давление:	записываем в регистр 0xF4 значение 0x34 или 0x74 или 0xB4 или 0xF4 (в зависимости от значения OSS), ждем флаг SCO, читаем 3байта из регистров 0xF6 0xF7 0xF8
			if(ErrData){return false;}
//			Расчёт промежуточных переменных
			PP1=((UT-AC6)*AC5>>15)+(MC<<11)/(((UT-AC6)*AC5>>15)+MD);
			PP2=((uint32_t)AC4*(uint32_t)(((((AC3*(PP1-4000))>>13)+((B_1*(((PP1-4000)*(PP1-4000))>>12))>>16)+2)>>2)+32768))>>15;
			PP3=((uint32_t)UP-((((AC1*4+((B_2*(((PP1-4000)*(PP1-4000))>>12))>>11)+((AC2*(PP1-4000))>>11))<<OSS)+2)>>2))*(uint32_t)(50000UL>>OSS);
			PP4=PP3<0x80000000?PP3*2/PP2:PP3/PP2*2;
//			Расчет температуры, давления, высоты
			temp=((float)PP1+8)/160;
			pres=(PP4+(((((PP4>>8)*(PP4>>8)*3038)>>16)+((-7357*PP4)>>16)+3791)>>4))/133.322;
			alti=44330*(1-pow(pres/SLP,1/5.255));
			return true;
}

/** внутренние функции для записи/чтения регистров EEPROM чипа BMP180 **/

void	BMP180::WriteReg(uint8_t Reg, uint8_t Data){
			Wire.beginTransmission(0x77);													//	указываем ID адрес сенсора на шине I2C
			Wire.write(Reg);																//	указываем адрес регистра в который будет записан байт данных
			Wire.write(Data);																//	указываем данные которые нужно записать в регистр
			Wire.endTransmission();															//	совершаем передачу по шине I2C
}

int32_t	BMP180::ReadReg(int NumBits, uint8_t Reg, boolean Uns){
int32_t		result32=0;
int16_t		result16=0;
int8_t		result8 =0;
			Wire.beginTransmission(0x77);													//	указываем ID адрес сенсора на шине I2C
			Wire.write(Reg);																//	указываем адрес регистра из которого будет производится чтение (а так же следующих за ним, если мы читаем больше 1 байта)
			Wire.endTransmission();															//	совершаем передачу по шине I2C
			Wire.requestFrom(0x77, NumBits);												//	указываем ID адрес сенсора на шине I2C и количество байт которое мы хотим прочитать
			for(int i=0; i<NumBits; i++){result32<<=8;		result32+=Wire.read();	}		//	читаем полученные данные
			if (Uns && NumBits==1)		{result8 =result32;	result32 =result8;		}		//	добавляем знак (если Uns==true)
			if (Uns && NumBits==2)		{result16=result32;	result32 =result16;		}		//	добавляем знак (если Uns==true)
			return result32;
}

void	BMP180::DelayFlagSCO(void){
int	i=0; while(ReadReg(1,0xF4)&0x20&&i<50){delay(1); i++;} if(i>=50||i==0){ErrData=true;}	//	Выходим из цикла если 5й бит регистра 0xF4 (флаг SCO) = 0, или после 50го цикла, или если циклов не было
}

