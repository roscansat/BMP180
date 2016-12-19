#ifndef BMP180_h
#define BMP180_h

#if defined(ARDUINO) && (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include "Wire.h"

class BMP180{
	public:
		void	begin(float Altitude=0);							//	Инициализация сенсора, с указанием текущей высоты над уровнем моря
		boolean	read(uint8_t OSS=0);								//	Чтение всех данных сенсора, с указанием точности OSS=0...3 (0-минимальная, 3-максимальная)
		float	temp;												//	Значение температуры в °C
		float	pres;												//	Значение давления в мм.рт.ст.
		float	alti;												//	Значение высоты над уровнем моря
	private:
		int32_t	ReadReg(int NumByte, uint8_t Reg, boolean Uns=true);//	Чтение данных от сенсора, NumByte - кол-во байт, Reg - адрес регистра, Uns - читать с учетом знака
		void	WriteReg(uint8_t Reg, uint8_t Data);				//	Запись данных в сенсор, Reg - адрес регистра, Data - данные для записи
		void	DelayFlagSCO(void);									//	Задержка по состоянию флага преобразований SCO (5бит в регистре 0xF4)
		float	SLP;												//	Расчётное давление на уровне моря в мм.рт.ст.
		uint8_t	ID=0;												//	Id чипа BMP180
		int32_t	AC1=0,AC2=0,AC3=0,AC4=0,AC5=0,AC6=0,B_1=0,B_2=0,MB=0,MC=0,MD=0,UT=0,UP=0,PP1=0,PP4=0;
		uint32_t PP2=0,PP3=0;										//	Промежуточные переменные
		boolean	ErrData=false;										//	Флаг ошибки чтения данных
};

#endif