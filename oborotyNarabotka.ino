/*
	OborotyNarabotka

	Считает обороты в минуту вращающегося вала мотора с помощью диска с
	прорезями/сверлениями и оптического датчика. Также выполняет подсчёт
	наработки мотора.

	На ATTtny85 программа выдаёт около 80k отсчётов в секунду. Для уверенного
	измерения оборотов необходимо 3-4 отсчёта на каждое состояние(прорезь/диск),
	поэтому максимальное число состояний в секунду 20k-27k. Для 24k оборотов
	вала мотора, 400 оборотов в секунду, имеем максимум в 34( (27000/400)/2 )
	прорези на диске.

	Для получения минимального количества прорезей нужно определиться с
	приемлемой погрешностью на минимальных измеряемых оборотах в заданном периоде
	сэмплирования(SMPLNG_PRD). Допустим, минимальные обороты в минуту 1k5, период
	сэмплирования 0.3с и погрешность измерения 0.5%. Оборотов в секунду - 25,
	оборотов за период сэмплирования - 7.5, погрешность - 0.0375 оборота.
	Погрешность соответствует одному(двум?) состояниям за период сэмплирования,
	соответственно:
	0.0375 * 2 * <число_прорезей> = 1
	<число_прорезей> = 1 / 0.075 = 13.(3)
	DISK_HLS = 14
	
	Если берём период сэмплирования 1с, погрешность - 0,125 оборота:
	0.125 * 2 * <число_прорезей> = 1
	<число_прорезей> = 1 / 0.25 = 4
	DISK_HLS = 4

	Для измерения оборотов мотора подсчитываются полные(!) смены состояний,
	поэтому реальный период сэмплирования будет меньше заданного, особенно на
	низких оборотах и малом числе прорезей/сверлений на диске.
	
	Наработка считается, если обороты мотора превышают заданное значение
	RPM_LIMIT. Посчитанная наработка сбрасывается, если длительность останова
	превысила заданное значение ZERO_BRK.

	Результаты отображаются на ЖКД-дисплеи типоразмера 1602 подключенного по
	протоколу i2c. Длительность выведения результатов на ATtiny85 около 1/3
	секунды.

	Если на ATtiny85 отключить функцию сброса МК(НЕ рекомендуется!!!) на пине PB5,
	то вместо наработки, при замыкании(?) этого пина на землю будет отображаться
	отладочная информация: учтённая длительность сэмплирования, количество полных
	смен состояния, общее количество отсчётов за период сэмплирования.

	kfmut, 2021г
	Лицензия: Creative Commons Zero v1.0 Universal
*/

/*
	НАСТРОЙКИ
*/

// пин датчика состояния
#define SENSOR_PIN PB1
// пин вывода отладки
#define DEBUG_PIN PB5
// длительность сэмплирования в 1/10-ых секунды( 2 = 1/5сек, 10 = 1сек и т.д.)
#define SMPLNG_PRD 2
// количество прорезей в диске
#define DISK_HLS 12
// максимальная длительность останова до сброса наработки, минут
#define ZERO_BRK 15
// максимальные обороты останова
#define RPM_LIMIT 100
// i2c-адрес ЖК-дисплея
#define I2C_ADDR 0x00

/*
	ДАЛЬШЕ НИЧЕГО НЕ МЕНЯТЬ!
	ДАЛЬШЕ НИЧЕГО НЕ МЕНЯТЬ!!
	ДАЛЬШЕ НИЧЕГО НЕ МЕНЯТЬ!!!
*/

// библиотека для работы с ЖКД
#include "Adafruit_LiquidCrystal.h"

// состояние отладочного пина для вывода отладочной информации
const uint8_t debugON = HIGH; //LOW;
// состояние отладочного пина для вывода рабочей информации
const uint8_t debugOFF = LOW; //HIGH;
// заданная длительность сэмплирования в миллисекундах
const uint16_t smplngPrdConst = SMPLNG_PRD * 100;
// заданная длительность останова в миллисекундах
const uint32_t zeroBrkConst = ZERO_BRK * 60000;
// константа для подсчёта оборотов
const float diskCrrctr = 60000000 / DISK_HLS;

// состояния работы МК
// начало цикла сэмлирования
#define FR_START 1
// сэмплирование
#define FR_SMPLNG 2
// отображение результатов
#define FR_DSPLY 3
// текущее состояние
uint8_t curState = FR_START;

// время начала сэмплирования, мсек
uint32_t smplStrtTm = 0;
// время окончания сэмплирования, мсек
uint32_t smplEndTm = 0;
// время первого учитываемого изменения состояния, мксек
uint32_t firstChangeTm = 0;
// время последнего учитываемого изменения состояния, мксек
uint32_t lastChangeTm = 0;
// признак, что текущие изменения состояния датчика является первым учитываемым
uint8_t firstChange = true;
// первое учитываемое состояние
uint8_t firstChangeState = HIGH;
// наработка мотора, накопленная часть, мсек
uint32_t nrbtkSB = 0;
// время начала крайнего цикла учёта наработки мотора, мсек
uint32_t nrbtkSBLastCycl = 0;
// время начала крайней остановки мотора, мсек
uint32_t nrbtkMtrStop = 0;
// общее число отсчётов в одном периоде сэмплирования
uint32_t cntrCntr = 0;
// число учитываемых переходов между секторами в текущем цикле сэмплирования
uint16_t changesCntr = 0;
// текущее состояние датчика
uint8_t sensorState = HIGH;
// предыдущее состояние датчика
uint8_t sensorStatePrv = sensorState;

// строка для формирования вывода на дисплей
char * str;
// длина строки для чепяти на дисплей
const size_t strBufLen = 17;

// объект для работы с ЖКД
Adafruit_LiquidCrystal lcd( I2C_ADDR );

/* Обявления функций */
void lcdPrintStr( const uint8_t, const uint8_t, const char * );
static void frStartHandler();
static void frSmplngHandler();
static void frDsplyHandler();
void frErrorHandler( const bool );

// готовимся к работе
void setup(){
	// выделяем память для строки
	str = ( char * )malloc( strBufLen );
	str[ 0 ] = 0;

	// готовим пины
	pinMode( SENSOR_PIN, INPUT );
	pinMode( DEBUG_PIN, INPUT_PULLUP );
	
	// ЖКД
	lcd.begin(16, 2);
 
	// рисуем заставку программульки
	lcd.clear();
	lcd.setBacklight(HIGH);
	frErrorHandler( false );
	delay( 3000 );
	lcd.clear();

	// ругаемся если был задан нулевой период сэмплирования
	if( !smplngPrdConst )
		frErrorHandler( true );
	}

// выводим строку на ЖКД
// row - номер строки для вывода, отсчёт от 1
// column - номер столбца для вывода, отсчёт от 1
// str - строка для вывода
void lcdPrintStr( const uint8_t row, const uint8_t column, const char * str ){
	// проверяем аргументы
	if( str == NULL || row < 1 || row > 2 || column < 1 || column > 16 )
		return;

	// бегунки 
	/*uint8_t xLcd = column - 1;
	uint8_t i = 0;
	// выводим либо до конца строки, либо до конца дисплея
	while( xLcd < 16 && str[ i ] != 0 ){
		lcd.setCursor( xLcd, row - 1 );
		lcd.write( str[ i ] );
		xLcd++;
		i++;
		}*/
	lcd.setCursor( column - 1, row - 1 );
	lcd.print( str );
	}

// обработчик состояния "начало сэмплирования"
void frStartHandler(){
	// сбрасываем счётчики
	changesCntr = 0;
	cntrCntr = 0;
	firstChange = true;
	firstChangeTm = 0;
	lastChangeTm = 0;
	// переходим в состояние "сэмплирование"
	curState = FR_SMPLNG;
	// запоминаем время начала сэмплирования
	smplStrtTm = millis();
	// получаем исходное состояние датчика
	sensorState = digitalRead( SENSOR_PIN );
	sensorStatePrv = sensorState;
	}

// обработчик состояния "сэмплирование"
void frSmplngHandler(){
	// запоминаем время окончания сэмплирования
	smplEndTm = millis();
	uint32_t changeTm = micros();

	// проверяем не превысили ли мы заданный период сэмплирования
	if( smplEndTm - smplStrtTm >= smplngPrdConst ){
		// если превысили, то переходим к отображению результатов
		curState = FR_DSPLY;
		return;
		}

	// если ещё считаем, то получаем состояние датчика
	sensorState = digitalRead( SENSOR_PIN );

	// смотрим был ли переход между секторами
	if( sensorState != sensorStatePrv ){
		// ...если был, то запоминаем предыдущее состояние датчика
		sensorStatePrv = sensorState;
		
		// если у нас не первая смена и она учитываемая
		if( !firstChange && sensorState == firstChangeState ){
			// ...запоминаем время и учитываем смену
			lastChangeTm = changeTm;
			changesCntr++;
			}
			
		// если же первая смена
		if( firstChange ){
			// ...то запоминаем время смены и первое состояние
			firstChangeTm = changeTm;
			firstChangeState = sensorState;
			firstChange = false;
			}
		}

	// считаем отсчёты
	cntrCntr++;
	}

// отображение времени на ЖКД
// row - номер строки для вывода, отсчёт от 1
// column - номер столбца для вывода, отсчёт от 1
// format - формат строки для вывода
// timeStamp - метка времени для вывода
// majorD - делитель для старших длительностей
// minorD - делитель для младших длительностей
void displayTime( const uint8_t row, const uint8_t column, const char * format,
		const uint32_t timeStamp, const uint8_t majorD, const uint16_t minorD ){
	// всего младших длительностей
	uint16_t tsS = timeStamp / minorD;
	// остатки младших длительностей
	uint8_t tsM = tsS % majorD;
	// целые старшие длительности
	tsS /= majorD;
	// получаем строку
	snprintf( str, strBufLen, format, tsS, tsM );
	// выводим на экран
	lcdPrintStr( row, column, str );
	}

// обработчик состояния "отображение результатов"
void frDsplyHandler(){
	// реальное время сэмплирования
	uint32_t actualPrd = 0;
	// проверяемся, что не переполнился счётчик микросекунд, каждые 70 минут 
	if( lastChangeTm > firstChangeTm )
		actualPrd = lastChangeTm - firstChangeTm;
	// считаем обороты в секунду в закончившемся цикле сэмплирования
	uint16_t rpm = 0;
	
	if( actualPrd )
		rpm = trunc( ( ( float )changesCntr / actualPrd ) * diskCrrctr );
	else
		actualPrd = ( uint32_t )smplngPrdConst * 1000;

	// выводим на ЖКД обороты
	snprintf( str, strBufLen, "RPM %5u", rpm );
	lcdPrintStr( 1, 1, str );
	
	// отображаем период сэмплирования
	uint8_t debugState = digitalRead( DEBUG_PIN );

	if( debugState == debugOFF )
		displayTime( 1, 10, " Ts %1u.%1u", smplngPrdConst, 10, 100 );
	
	// в отладке выводим реальное время сэмплирования
	if( debugState == debugON ){
		snprintf( str, strBufLen, "%7lu", actualPrd );
		lcdPrintStr( 1, 10, str );
		}

	// учитывем наработку мотора
	// если обороты выше заданного порога
	if( rpm >= RPM_LIMIT ){
		// ..то смотрим, это у нас первый цикл с работающим мотором или нет
		if( nrbtkSBLastCycl == 0 ){
			// если первый, то запоминаем время начала работы мотора
			nrbtkSBLastCycl = smplStrtTm;
			// и сбраысваем время начала крайней остановки мотора
			nrbtkMtrStop = 0;
			}
		}
	else{
		// ...а если меньше, то смотрим, это у нас первый цикл с остановкой или нет
		if( nrbtkSBLastCycl != 0){
			// если первый, то к накопленной наработе плюсуем текущую
			nrbtkSB += smplStrtTm - nrbtkSBLastCycl;
			// обнуляем время начала текущего цикла наработки
			nrbtkSBLastCycl = 0;
			// запоминаем время начала остановки
			nrbtkMtrStop = smplStrtTm;
			}
		}

	// выводим наработку и длительность остановки мотора
	// времянка
	uint32_t tmpUL = 0;
	
	// накопленная наработка
	tmpUL = nrbtkSB;
	// если в этом цикле сэмплирования работал
	if( nrbtkSBLastCycl )
		// ...то плюсуем наработку этого цикла к накопленной
		tmpUL += smplEndTm - nrbtkSBLastCycl;

	// выводим наработку
	if( debugState == debugOFF )
		displayTime( 2, 1, "I %3u:%02u", tmpUL, 60, 1000 );

	// длительность останова
	tmpUL = 0;
	// если стоим...
	if( nrbtkMtrStop ){
		// ...то считаем длительность от последней даты сэмплирования
		tmpUL = smplEndTm - nrbtkMtrStop;
		// если превысили максимальную длительность останова
		if( tmpUL >= zeroBrkConst ){
			// ...то обнуляемся
			nrbtkSB = 0;
			nrbtkSBLastCycl = 0;
			nrbtkMtrStop = 0;
			}
		}

	// выводим длительность останова
	if( debugState == debugOFF )
		displayTime( 2, 9, " O %2u:%02u", tmpUL, 60, 1000 );

	// в отладке выводим количество изменений состояния датчика и число отсчётов
	// за период сэмплирования
	if( debugState == debugON ){
		snprintf( str, strBufLen, "IZ%4u OT%7lu", changesCntr, cntrCntr );
		lcdPrintStr( 2, 1, str );
		}
			
	// после отображения результатов начинаем новый цикл сэмплирования
	curState = FR_START;
	}

// обработчик ошибки
void frErrorHandler( const bool error = true ){
	// хвалимся
	lcdPrintStr(1, 1, "OborotyNarabotka" );
	lcdPrintStr(2, 1, "  by kfmut '21" );

	// привлекаем внимание если что
	while( error ){
		lcd.setBacklight( LOW );
		delay( 500 );
		lcd.setBacklight( HIGH );
		delay( 500 );
		}
	}

// основной цикл работы МК
void loop() {
	// смотрим текущее состояние
	switch( curState ){
		// начало сэмплирования
		case FR_START:
			frStartHandler();
			break;
		// сэмплирование
		case FR_SMPLNG:
			frSmplngHandler();
			break;
		// отображение результатов
		case FR_DSPLY:
			frDsplyHandler();
			break;
		// ошибко!!!
		default:
			frErrorHandler();
		}
	}
