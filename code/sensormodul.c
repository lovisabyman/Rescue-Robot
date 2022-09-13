/*
 * sensormodul.c
 *
 * Created: 2021-03-29 13:42:12
 *  Author: johca194
 		// Set up timer0 in CTC mode, interrupt every 125 clk/8 cycle
	//	TCCR0A = (0<<COM0A1 | 0<<COM0A0 | 0 <<COM0B1 | 0 <<COM0B0 | 0<<FOC0A)| (0<<WGM00)|(1<<WGM01);
	//	TCCR0B = (1<<WGM02 | 2 << CS00);
	//	OCR0A = 0x7c;
	//	OCR0B = 0x7c;
		// s�tter OCR0A och OCR0B till 124 klockcykler
	//	TIMSK0 = (1<<OCIE0B | 1<<OCIE0A | 0 <<TOIE0);

	//	sei(); // SREG I-bit active.

		//Avbrottsrutin f�r brytare
	//ISR(INT1_vect)
	//{
		//V�rdet kommer inte kollas i avbrottsrutin, endast f�r test. Brytare kan endast �ndras n�r roboten �r avst�ngd.
		//V�rde 0: Manuellt, mot jtag.
		//V�rde 1: Autonomt, mot bl�.
		// Spara 0 eller 1 som typ int och l�gg sen p� best�md plats tillsammans med annat som ska skickas med datapaketet till tex styrmodulen som tolkar vilket l�ge.
	//}
 */
	#define F_CPU 8000000UL

	#include <avr/io.h>
	#include <avr/interrupt.h>
	#include <stdio.h>
	#include <math.h>
	#include <string.h>
	#include <util/delay.h>


	volatile uint16_t adc_cm_IR; // v�nster bak
	volatile uint16_t adc_1_HB; // h�ger bak
	volatile uint16_t adc_2_VF; // v�nster fram mot gripklon
	volatile uint16_t adc_3_HF; // h�ger fram
	volatile uint16_t adc_0_VB; // h�ger fram
	volatile uint8_t cm_IR = 0; // v�nster bak
	volatile uint8_t cm_HB = 0; // h�ger bak
	volatile uint16_t adc_FRAM;
	volatile uint8_t cm_VF; // v�nster fram mot gripklon
	volatile uint8_t cm_VB; // v�nster fram mot gripklon
	volatile uint8_t cm_HF; // h�ger fram
	volatile uint8_t cm_FRAM; // fram
	volatile uint16_t cm_Lidar; // lidar cm
	volatile uint8_t cm_Lidar_high_byte; // lidar cm
	volatile uint8_t cm_Lidar_low_byte; // lidar cm
	volatile uint8_t SLA_W; // skriv till slav
	volatile uint8_t SLA_R; // l�s fr�n slav
	volatile uint8_t ID; //ID i pakethuvud.
	volatile double ERROR;
	volatile uint8_t DATA;
	volatile uint8_t SWITCH;
	volatile uint8_t DATA_adr;
	volatile uint16_t DATA_16;
	volatile uint8_t DATA_high;
	volatile uint8_t DATA_low;
	volatile uint8_t MT_DATA_ACK;
	volatile uint8_t MT_SLA_ACK;
	volatile uint8_t MT_DATA_NACK;
	volatile uint8_t control_register;
	volatile uint16_t COUNTER;
	volatile uint8_t start_button = 0;
	volatile uint32_t clock;
	volatile uint32_t clock1;
	volatile uint32_t clock2;
	volatile uint32_t clock3;
	volatile uint32_t clock4;
	volatile uint32_t clock5;
	volatile uint32_t clock6;
	volatile uint8_t clock_laps;
	volatile uint32_t clock_sum;
	volatile uint64_t clock_sum_64;
	volatile int index;
	volatile int value_high;
	volatile int value_low;
	volatile uint8_t BITS = 0;
	int IR_array_HF[] = {32000, 27520, 22400, 20096, 17664, 15744, 14400, 13056, 11584, 10816, 10304, 9536, 9152, 8256, 7744, 7360, 7232, 6592, 6272, 5952, 5824, 5568, 5312, 5056, 4928, 4608, 4408};
	int IR_array_VB[] = {34496, 28160, 23680, 20352, 17600, 15296, 13632, 12480, 11200, 10304, 9088, 8384, 7872, 7360, 6912, 6464, 6080, 5376, 5184, 4928, 4672, 4544, 4224, 4160, 3776, 3520, 3072};
	int IR_array_HB[] = {32448, 27776, 24192, 21248, 18880, 17216, 15360, 14080, 12800, 12160, 11456, 10816, 9792, 9536, 8960, 8448, 8128, 7744, 7360, 7168, 6720, 6208, 6144, 6080, 5824, 5504, 5248};
	int IR_array_VF[] = {36672, 30464,26304,22656,19584,18176,16384,14720,13504,12416,11392,11264,10240,9664,6344,8768,8192,8064,7616,7040,6784,6528,6464,6016,5696,5760,5568};
	int IR_array_cm[] = {4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30};
	//int IR_array_FRAM[] = {};
	int IR_array[27] = {0};
	volatile int64_t rate_gyro;
	volatile int32_t rate_gyro_signed;
	volatile uint32_t rate_gyro_unsigned;
	volatile uint64_t value_gyro_dir;

	volatile uint16_t angle_gyro;

	volatile uint64_t value_gyro;
	volatile int16_t value_gyro_signed;
	volatile uint64_t rate_gyro_unsigned_64;
	volatile int64_t rate_gyro_signed_64;

	volatile uint8_t gyro_counter;
	volatile uint8_t reflex_counter;

	volatile uint16_t adc_reflex;
	volatile uint16_t adc_reflex_4;
	volatile uint16_t adc_reflex_5;
	volatile uint16_t adc_reflex_6;
	volatile uint16_t adc_reflex_7;
	volatile uint16_t adc_reflex_8;
	volatile uint8_t reflex_pin_in;
	volatile uint8_t reflex_pin_out;
	volatile uint16_t adc_sum_4;
	volatile uint16_t adc_sum_5;
	volatile uint16_t adc_sum_6;
	volatile uint16_t adc_sum_7;
	volatile uint16_t adc_sum_8;
	volatile uint16_t adc_sum_all;
	volatile uint16_t adc_sum_null;

	volatile uint16_t adc_wheel;
	volatile uint16_t black;
	volatile uint16_t white;
	volatile uint16_t previous_value = 0;
	volatile uint16_t distance_cm;


	void correct_value_lidar();
	void ADC_IR_0_VB();
	void ADC_IR_1_HB();
	void ADC_IR_2_VF();
	void ADC_IR_3_HF();
	void ADC_IR_FRAM();
	void IR_sensors();
	void I2C_init();
	void send_data_control_module();
	void LidarLiteV3();
	void start_cond();
	void repeted_start_cond();
	void error();
	void write_16data_control_module();
	void read_from_slave();
	void send_adress_to_slave();
	void stop_cond();
	void write_to_sensor();
	void check_done_measurement_lidar();
	void IR_check_cm_value();
	void Gyro();
	void correct_value_gyro();
	void compare_gyro_IR();
	void compute_gyro_degrees();
	void check_switch();
	void check_button();
	void Reflex_sensor();
	void get_reflex_adc();
	void adc_to_binary();
	void get_position_reflex_visionen();
	void get_position_reflex_muxen();
	void Wheel_sensor();

	int main()
	{
		//konfigurera portar.
		DDRD = 0x00; // set portd as input.
		DDRB = 0xff;  //set port b as output.
		PORTD = 0xff; //aktiverar pull-up resitors
		PORTB = 0x00;

		// Gr� knapp (ej reset)
		EICRA = (1<<ISC00 | 1<<ISC01); //Aktverar avbrott p� h�g flank.
		EIMSK = (1<<INT0); // enable external interrupts p� int0.

		//IR-sensorer
		ADCSRA = (1<<ADEN | 0<< ADIE | 1<<ADPS2 | 1<<ADPS1 | 0<<ADPS0 ); // enables ADC, enables interrupt, divisionfactor 64
		ADCSRB = (1<<ADTS0);
		ADMUX = (1<<REFS0 | 0<<REFS1 | 1<<ADLAR); // AREF used, Internal Vref turned off, left adjusted result from ADLAR

		I2C_init();
		start_button = 0;
		_delay_ms(1000);
		check_switch();

		while(1)
		{
		cli();
		if (start_button == 1)
		{
			check_button();
		}
		IR_sensors();

		TCCR0B = 0x00;  //St�nger av klocka.
		clock1 = TCNT0;
		TCNT0 = 0;		// Nollst�ller klockan
		TCCR0B = 0x05;  // Normal mode och clk/1024 = 1 ms S�TTER P� KLOCKAN

		Reflex_sensor();

		TCCR0B = 0x00;  //St�nger av klocka.
		clock2 = TCNT0;
		TCNT0 = 0;		// Nollst�ller klockan
		TCCR0B = 0x05;  // Normal mode och clk/1024 = 1 ms S�TTER P� KLOCKAN

		Wheel_sensor();


		TCCR0B = 0x00;  //St�nger av klocka.
		clock3 = TCNT0;
		TCNT0 = 0;		// Nollst�ller klockan
		TCCR0B = 0x05;  // Normal mode och clk/1024 = 1 ms S�TTER P� KLOCKAN

		Gyro();


		TCCR0B = 0x00;  //St�nger av klocka.
		clock4 = TCNT0;
		TCNT0 = 0;		// Nollst�ller klockan
		TCCR0B = 0x05;  // Normal mode och clk/1024 = 1 ms S�TTER P� KLOCKAN

		Reflex_sensor();

		TCCR0B = 0x00;  //St�nger av klocka.
		clock5 = TCNT0;
		TCNT0 = 0;		// Nollst�ller klockan
		TCCR0B = 0x05;  // Normal mode och clk/1024 = 1 ms S�TTER P� KLOCKAN

		LidarLiteV3();

		TCCR0B = 0x00;  //St�nger av klocka.
		clock6 = TCNT0;
		TCNT0 = 0;		// Nollst�ller klockan
		TCCR0B = 0x05;  // Normal mode och clk/1024 = 1 ms S�TTER P� KLOCKAN

		sei();
		}
	}

	//Avbrottsrutin f�r knapp och kollar v�rde p� brytare.
ISR(INT0_vect) // 0 �r manuellt l�ge, 1 �r autonomtl�ge ej startad, 2 �r autonomt ej startad.
	{
		SWITCH = PIND&(1<<PD3); //Autonom ger v�rdet 1, manuell ger v�rdet 0.
		start_button = 1;
	}

void check_button()
{
	ID = 0x09; // startknapp och brytare
	DATA_high = 0x00;
	if (SWITCH == 8)
	{
		_delay_ms(1000);
		DATA_low = 0x02;			// autonomt l�ge och knappen tryckt
		send_data_control_module();
	}
	start_button = 0;
}

void check_switch()
{
	DATA_high = 0x00;
	ID = 0x09; // startknapp och brytare
	SWITCH = PIND&(1<<PD3); //Autonom ger v�rdet 1, manuell ger v�rdet 0.
	if (SWITCH == 8)		// betyder en etta p� pin 3
	{
		DATA_low = 0x01;    //Autonomt l�ge, knapp ej tryckt
	}
	else
	{
		DATA_low = 0x00;	//Manuellt l�ge, knapp ej tryckt
	}
	send_data_control_module();
}

void IR_sensors()
{
	DATA_high = 0x00;
	DATA_low = 0x00;
	ADC_IR_0_VB();
	ID = 0x5;
	send_data_control_module();
	ADC_IR_1_HB();
	ID = 0x4;
	send_data_control_module();
	ADC_IR_2_VF();
	ID = 0x3;
	send_data_control_module();
	ADC_IR_3_HF();
	ID = 0x2;
	send_data_control_module();
	/*
	ID = 0x28;
	ADC_IR_FRAM();
	send_data_control_module();
	*/
}
/*
void ADC_IR_FRAM()
{
	ADMUX = (1<<REFS0 | 0<<REFS1 | 1<<ADLAR); //reset old channel
	ADMUX |= (1<<MUX2 | 1<<MUX1 | 0<<MUX0);  //new channel
	ADCSRA |= (1 << ADSC);
	while(ADCSRA & (1<<ADSC));
	adc_cm_IR = ADC; // v�rde att matcha med
	adc_FRAM = adc_cm_IR;
	memcpy(IR_array, IR_array_FRAM, sizeof(IR_array));
	IR_check_cm_value();
	DATA_low = cm_IR;
	cm_VB = cm_IR;
}
*/
void ADC_IR_0_VB() // FEL
{
	ADMUX = (1<<REFS0 | 0<<REFS1 | 1<<ADLAR); //reset old channel
	ADMUX |= (0<<MUX2 | 0<<MUX1 | 0<<MUX0);  //new channel
	ADCSRA |= (1 << ADSC);
	while(ADCSRA & (1<<ADSC));
	adc_cm_IR = ADC; // v�rde att matcha med
	adc_0_VB = adc_cm_IR;
	memcpy(IR_array, IR_array_VB, sizeof(IR_array));
	IR_check_cm_value();
	DATA_low = cm_IR;
	cm_VB = cm_IR;
}

void ADC_IR_1_HB() // FEL
{
	ADMUX = (1<<REFS0 | 0<<REFS1 | 1<<ADLAR); //reset old channel
	ADMUX |= (0<<MUX2 | 0<<MUX1 | 1<<MUX0); //new channel
	ADCSRA |= (1 << ADSC);
	while(ADCSRA & (1<<ADSC));
	adc_cm_IR = ADC;
	adc_1_HB = adc_cm_IR;
	memcpy(IR_array, IR_array_HB, sizeof(IR_array));
	IR_check_cm_value();
	DATA_low = cm_IR;
	cm_HB = cm_IR;
}

void ADC_IR_2_VF()
{
	ADMUX = (1<<REFS0 | 0<<REFS1 | 1<<ADLAR); //reset old channel
	ADMUX |= (0<<MUX2 | 1<<MUX1 | 0<<MUX0); //new channel
	ADCSRA |= (1 << ADSC);
	while(ADCSRA & (1<<ADSC));
	adc_cm_IR = ADC;
	adc_2_VF = adc_cm_IR;
	memcpy(IR_array, IR_array_VF, sizeof(IR_array));
	IR_check_cm_value();
	DATA_low = cm_IR;
	cm_VF = cm_IR;
}

void ADC_IR_3_HF()
{
	ADMUX = (1<<REFS0 | 0<<REFS1 | 1<<ADLAR); //reset old channel
	ADMUX |= (0<<MUX2 | 1<<MUX1 | 1<<MUX0); //new channel
	ADCSRA |= (1 << ADSC);
	while(ADCSRA & (1<<ADSC));
	adc_cm_IR = ADC;
	adc_3_HF = adc_cm_IR;
	memcpy(IR_array, IR_array_HF, sizeof(IR_array));
	IR_check_cm_value();
	DATA_low = cm_IR;
	cm_HF = cm_IR;
}

void IR_check_cm_value()
{
	value_high = 0;
	value_low = 0;
	index = 26;
	while(IR_array[index] < adc_cm_IR | index == 0)
	{
		index -= 1;
	}
	if (index != 26)
	{
		value_high = IR_array[index] - adc_cm_IR;
		value_low = adc_cm_IR - IR_array[index + 1];
	}
	if (value_high < value_low)
	{
		cm_IR = IR_array_cm[index];
	}
	else if (adc_cm_IR < IR_array[26])
	{
		cm_IR = 0;
	}
		else if (index == 26)
	{
		cm_IR = IR_array_cm[index];
	}
	else
	{
		cm_IR = IR_array_cm[index + 1];
	}
}

void LidarLiteV3()
{
	SLA_W = 0xC4;  //Slavadress: 0x, write W=0.
	SLA_R = 0xC5;
	COUNTER = 0;
	ID = 0x06;
	BITS = 16;
	start_cond();
	DATA_adr = 0x00;							// Register-adress
	DATA = 0x04;
	write_to_sensor();
	stop_cond();
	DATA = 0x01;
	while (COUNTER != 10)
	{
		COUNTER += 1;
	}
	COUNTER = 0;
	start_cond();
	control_register = 0x8f;
	send_adress_to_slave();        //Skicka slavanrop och sensorns valt kontrollregister.
	stop_cond();
	start_cond();
	read_from_slave();             //Tar emot data fr�n valt kontrollregister.
	stop_cond();
	if (DATA_16 != 0)
	{
		correct_value_lidar();
		cm_Lidar = DATA_16;
		DATA_high = DATA_16>>8;   //Skift 8 bitar h�ger.
		DATA_low = DATA_16;
		send_data_control_module();
	}
}

void Gyro()  //GAMMAL KOD
{
	// gain/sensitivity 8.75 mdps/digit dvs
	// Z-axeln �r den enda axeln att kolla p�, plockar ut h�gra v�rdet och f�r v�nstra som negativa v�nstra

	SLA_W = 0xD6; // deFENITIVT D6
	SLA_R = 0xD7;  //Ev 0xD1 enligt datablad
	COUNTER = 0;
	ID = 0x07;
	BITS = 16;
	start_cond();
	DATA = 0x40;
	DATA_adr = 0x20;				// startar/s�tter p� Normal-mode
	DATA = 0x0C;					// skriver ettor till PD och Zen i kontrollregister 1
	write_to_sensor();
	stop_cond();
	while (COUNTER != 5)
	{
		COUNTER += 1;
	}
	COUNTER = 0;
	start_cond();
	control_register = 0x2D;		// h�ger vinkels kontrollregistreradress (Z-axeln)
	send_adress_to_slave();			// skickar kontrollregisteraderssen till slavadressen
	repeted_start_cond();
	read_from_slave();              // tar emot data fr�n valt kontrollregister i DATA_16
	stop_cond();
	correct_value_gyro();			// g�r om DATA_16 till vinkel, h�r skickas �ven data
	compare_gyro_IR();				// �terst�ller vinkeln
	send_data_control_module();		// Skickar data till styrmodul.
}

void correct_value_gyro()
{
	gyro_counter += 1;
	value_gyro_signed = (int16_t)DATA_16;					//Vinkelhastighet fr�n gyro

	if (value_gyro_signed < 0)
	{
		rate_gyro_signed = value_gyro_signed;				//G�r om till samma tal fast till positivt
		rate_gyro_signed = rate_gyro_signed - 1;
		rate_gyro_signed = ~rate_gyro_signed;

		rate_gyro_unsigned = (uint32_t)rate_gyro_signed;	//G�r om till unsigned

		TCCR0B = 0x00;  //St�nger av klocka.
		clock = TCNT0;
		TCNT0 = 0;		// Nollst�ller klockan
		TCCR0B = 0x05;  // Normal mode och clk/1024 = 1 ms S�TTER P� KLOCKAN

		rate_gyro_unsigned_64 = (uint64_t) rate_gyro_unsigned * clock;  //Area under stapel
		rate_gyro -= rate_gyro_unsigned_64;								// Dra bort arean under stapeln
	}
	else
	{

		TCCR0B = 0x00;  //St�nger av klocka.
		clock = TCNT0;		//Antalet hela clk/1024
		TCNT0 = 0; // Nollst�ller klockan
		TCCR0B = 0x05; // Normal mode och clk/1024 = 1 ms S�TTER P� KLOCKAN

		rate_gyro_unsigned = value_gyro_signed;
		rate_gyro_unsigned_64 = (uint64_t) rate_gyro_unsigned * clock;  //Area under stapel
		rate_gyro += rate_gyro_unsigned_64;								//  L�gg till arean under stapeln
	}

	//Ovan: summerar rate_gyro 10 g�nger.
	if(gyro_counter == 10)
	{
		compute_gyro_degrees();
		if (value_gyro_dir == 0)							// value_gyro �r det f�rra utr�knade v�rdet som ska anv�ndas f�r att f� korrigerat v�rde p� rate_gyro
		{
			if (value_gyro > 360)
			{
				value_gyro -= 360;
			}
		}
		else if (value_gyro_dir == 90)
		{
			value_gyro = value_gyro_dir + value_gyro;
			if (value_gyro > 360)
			{
				value_gyro -= 360;
			}
		}
		else if (value_gyro_dir == 180)
		{
			value_gyro = value_gyro_dir + value_gyro;
			if (value_gyro > 360)
			{
				value_gyro -= 360;
			}
		}
		else if (value_gyro_dir == 270)
		{
			value_gyro = value_gyro_dir + value_gyro;
			if (value_gyro > 360)
			{
				value_gyro -= 360;
			}
		}
		gyro_counter = 0;
	}


}


void compute_gyro_degrees()
{
	//R�knar ut en vinkel utefter summan rate_gyro
	if (rate_gyro < 0 || rate_gyro == 0)
	{
		rate_gyro_signed_64 = rate_gyro;					// g�r om vinkelhastighetssumman till positivt
		rate_gyro_signed_64 = rate_gyro_signed_64 - 1;
		rate_gyro_signed_64 = ~rate_gyro_signed_64;

		rate_gyro_unsigned_64 = (uint64_t)rate_gyro_signed_64;	//G�r om till unsigned
		value_gyro = rate_gyro_unsigned_64 * 950 / (100000 * 7812);

	}
	else
	{
		value_gyro = rate_gyro * 875 / (100000 * 7812);
		value_gyro = 360 - value_gyro;
		if (value_gyro == 360)
		{
			value_gyro == 0;
		}
	}
}


void compare_gyro_IR()
{
	if ((cm_HF == cm_HB && cm_HB != 0) && (cm_VB == cm_VF && cm_VB != 0))
	{
		if (value_gyro < 25)
		{
			value_gyro_dir = 0;
			value_gyro = 0;
			rate_gyro = 0;
		}
		else if (65 < value_gyro && value_gyro < 125)
		{
			value_gyro_dir = 90;
			value_gyro = 90;
			rate_gyro = 0;
		}
		else if (155 < value_gyro && value_gyro < 205)
		{
			value_gyro_dir = 180;
			value_gyro = 180;
			rate_gyro = 0;
		}
		else if (245 < value_gyro && value_gyro < 295)
		{
			value_gyro_dir = 270;
			value_gyro = 270;
			rate_gyro = 0;
		}
		else if (335 < value_gyro)
		{
			value_gyro_dir = 0;
			value_gyro = 0;
			rate_gyro = 0;
		}
		//rate_gyro = 0;
	}

	DATA_low = value_gyro;
	DATA_high = value_gyro >> 8;
}


void Reflex_sensor()				// channel 4 �r start
{
	ID = 0x1;
	get_reflex_adc();
	get_position_reflex_visionen();
	//get_position_reflex_muxen();
}

void get_reflex_adc()
{
	// s�tt en etta p� pin 15 p� mulitplexerna f�r att sriva till kanal 4
	for (reflex_counter = 1; reflex_counter < 2; ++reflex_counter)
	{
		reflex_pin_in = 0x04;
		reflex_pin_out = 0;
		while (reflex_pin_out < 5)
		{
			ADMUX = (1<<REFS0 | 0<<REFS1 | 0<<ADLAR);	 //reset old channel
			ADMUX |= (1<<MUX2 | 0<<MUX1 | 0<<MUX0);		 //s�tter PA4 till AD omvandling
			ERROR = 1;
			PORTB = reflex_pin_in;										 	 // kopierar reflex_pin_in till port B
			//Utskick PORTB: A2 Y4 Y3 Y2 Y1 Y0 A1 A0 4: 0 00001 00 5: 0 00010 01 6: 0 00100 10 7: 0 01000 11 8: 1 10000 00
			if (reflex_pin_out < 4)
			{
				PORTB |= reflex_pin_out;
			}
			else
			{
				PORTB |= 0x80;
			}
			ADCSRA |= (1 << ADSC);
			while(ADCSRA & (1<<ADSC));
			adc_reflex = ADC;							// ger v�rdet p� reflex_pin_out
			if (reflex_pin_out == 0)
			{
				adc_reflex_4 += adc_reflex;

			}
			else if (reflex_pin_out == 1)
			{
				adc_reflex_5 += adc_reflex;

			}
			else if (reflex_pin_out == 2)
			{
				adc_reflex_6 += adc_reflex;

			}
			else if (reflex_pin_out == 3)
			{
				adc_reflex_7 += adc_reflex;

			}
			else if (reflex_pin_out == 4)
			{
				adc_reflex_8 += adc_reflex;

			}
			reflex_pin_in *= 0x2;
			++reflex_pin_out;
		}
	}
	reflex_counter = 0;
	adc_reflex_4 = adc_reflex_4/1;
	adc_reflex_5 = adc_reflex_5/1;
	adc_reflex_6 = adc_reflex_6/1;
	adc_reflex_7 = adc_reflex_7/1;
	adc_reflex_8 = adc_reflex_8/1;
}

void get_position_reflex_visionen() // lite tvek p� noggrannheten h�r. �M�ste ev kalibreras i visionen.2
{
	DATA_high = 0x00;
	if((adc_reflex_5 > 580 && adc_reflex_6 > 580 && adc_reflex_7 > 580) && (adc_reflex_4 > 580 || adc_reflex_8 > 580))
	//Tejp p� alla sensorer samtidigt.
	{
		DATA_low = 0x1F;
		send_data_control_module();
	}
	else if (600 < adc_reflex_4 && adc_reflex_5 < 601)	// mitt p� 4
	{
		DATA_low = 4;
		send_data_control_module();
	}
	else if (600 < adc_reflex_4 && 600 < adc_reflex_5 ) // mellan 4 och 5
	{
		DATA_low = 45;
		send_data_control_module();
	}
	else if (600 < adc_reflex_5 && adc_reflex_4 < 601 && adc_reflex_6 < 601) // mitt p� 5
	{
		DATA_low = 5;
		send_data_control_module();
	}
		else if (600 < adc_reflex_5 && 600 < adc_reflex_6 ) // mellan 5 och 6
	{
		DATA_low = 56;
		send_data_control_module();
	}
	else if (600 < adc_reflex_6 && adc_reflex_5 < 601 && adc_reflex_7 < 601) // mitt p� 6
	{
		DATA_low = 6;
		send_data_control_module();
	}
		else if (600 < adc_reflex_6 && 600 < adc_reflex_7 ) // mellan 6 och 7
	{
		DATA_low = 67;
		send_data_control_module();
	}
	else if (600 < adc_reflex_7 && adc_reflex_6 < 601 && adc_reflex_8 < 601 )	// mitt p� 7
	{
		DATA_low = 7;
		send_data_control_module();
	}
		else if (600 < adc_reflex_7 && 600 < adc_reflex_8 ) // mellan 7 och 8
	{
		DATA_low = 78;
		send_data_control_module();
	}
	else if (700 < adc_reflex_8 && adc_reflex_7 < 601 )	// mitt p� 8
	{
		DATA_low = 8;
		send_data_control_module();
	}

	else
	{
		DATA_low = 0;
		send_data_control_module();
	}
	adc_reflex_4 = 0;
	adc_reflex_5 = 0;
	adc_reflex_6 = 0;
	adc_reflex_7 = 0;
	adc_reflex_8 = 0;
}

void I2C_init()
{
	TWBR = (1<<TWBR1);       // Set bit rate to f_clk/26 ~ 40 kbps. SCL:s frekvens ska vara 16 g�nger mindre �n slavens CPU frekvens
	TWSR = 0xf8;     // Set TWPS=0
	TWCR = (1<<TWEN);
}

void send_data_control_module() //skicka data till styrmodul
{
	start_cond();
	SLA_W = 0x02;				// s�tter adressen till 0x01 och W = 0
	write_16data_control_module();
	stop_cond();
}

void start_cond()
{
	TWCR =  (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	while (!(TWCR & (1<<TWINT)));				// kollar s� att flaggan till TWINT �r satt dvs �r lika med ett och startvilkoret ovan d�rmed �r skickat
	if ((TWSR & 0xf8) != 0x08)					// maskar ur de f�rsta bitarna och kollar om de �r lika med statuskoden f�r START = 0x08
	{
		ERROR = 1;
	}
}

void repeted_start_cond()
{
	TWCR =  (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	while (!(TWCR & (1<<TWINT)));				// kollar s� att flaggan till TWINT �r satt dvs �r lika med ett och startvilkoret ovan d�rmed �r skickat
	if ((TWSR & 0xf8) != 0x10)					// maskar ur de f�rsta bitarna och kollar om de �r lika med statuskoden f�r START = 0x08
	{
		ERROR = 1;
	}
}

void write_16data_control_module()  //Skickar alltid 16 bitar till styrmodulen
{
	for(;;)
	{
	MT_SLA_ACK = 0x18;						// SLA+W �r skickad och ACK �r mottagen
	MT_DATA_ACK = 0x28;						// DATA �r skickad och ACK �r mottagen
	TWDR = SLA_W;
	TWCR = (1<<TWINT | 1<<TWEN);			// laddar variablen med slavadressen till TWCR-registret
	while ( !(TWCR & (1<<TWINT)));			// v�ntar p� flaggan s� att vi ser att SLA_W �r skickad
	if ((TWSR & 0xf8)  != MT_SLA_ACK)
	{
		ERROR = 1;
		break;
	}
	TWDR = ID;							// laddar in den data vi vill skicka
	TWCR = (1<<TWINT | 1<<TWEN);			// clear TWINT bit f�r att starta data�verf�ringen
	while ( !(TWCR & (1<<TWINT)));			// v�ntar p� flaggan s� att vi ser att DATA �r skickad och ACK/NACK har mottagits
	if ((TWSR & 0xf8) != MT_DATA_ACK)
	{
		ERROR= 1;
		break;
	}
	//Implemmentera skicka checksum

	TWDR = DATA_high;							// laddar in den data vi vill skicka
	TWCR = (1<<TWINT | 1<<TWEN);			// clear TWINT bit f�r att starta data�verf�ringen
	while ( !(TWCR & (1<<TWINT)));			// v�ntar p� flaggan s� att vi ser att DATA �r skickad och ACK/NACK har mottagits
	if ((TWSR & 0xf8) != MT_DATA_ACK)
	{
		ERROR = 1;
		break;
	}

	TWDR = DATA_low;							// laddar in den data vi vill skicka
	TWCR = (1<<TWINT | 1<<TWEN);			// clear TWINT bit f�r att starta data�verf�ringen
	while ( !(TWCR & (1<<TWINT)));			// v�ntar p� flaggan s� att vi ser att DATA �r skickad och ACK/NACK har mottagits
	if ((TWSR & 0xf8) != MT_DATA_ACK)
	{
		ERROR = 1;
		break;
	}
	break;
	}
}

void send_adress_to_slave()					//Skicka slavanrop och sensorns valt kontrollregister.
{
	for (;;)
	{
		MT_SLA_ACK = 0x18;						// SLA+W �r skickad och ACK �r mottagen
		MT_DATA_ACK = 0x28;						// DATA �r skickad och ACK �r mottagen

		TWDR = SLA_W;							// v�ljer slavadress samt att skriva till denna
		TWCR = (1<<TWINT | 1<<TWEN);			// laddar variablen med slavadressen till TWCR-registret
		while ( !(TWCR & (1<<TWINT)));			// v�ntar p� flaggan s� att vi ser att SLA_W �r skickad
		if ((TWSR & 0xf8) != MT_SLA_ACK)		// kollar om statuskoden indikerar korrekt avl�sning
			{
				ERROR = 1;
				break;
			}
		TWDR = control_register;				// skickar kontrollregister f�r avst�nd i cm
		TWCR = (1<<TWINT | 1<<TWEN);			// clear TWINT bit f�r att starta data�verf�ringen
		while ( !(TWCR & (1<<TWINT)));			// v�ntar p� flaggan s� att vi ser att DATA �r skickad och ACK/NACK har mottagits
		if ((TWSR & 0xf8) != MT_DATA_ACK)
			{
				ERROR = 1;
				break;
			}
		break;
	}

}

void read_from_slave()  //Tar emot data fr�n valt kontrollregister.
{
	for (;;)
	{
		MT_SLA_ACK = 0x40;						// SLA+W �r skickad och ACK �r mottagen
		MT_DATA_NACK = 0x58;
		MT_DATA_ACK = 0x50;

		TWDR = SLA_R;							// v�ljer slavadress samt att skriva till denna
		TWCR = (1<<TWINT | 1<<TWEN);			// laddar variablen med slavadressen till TWCR-registret
		while ( !(TWCR & (1<<TWINT)));			// v�ntar p� flaggan s� att vi ser att SLA_W �r skickad
		if ((TWSR & 0xf8) != MT_SLA_ACK)		// kollar om statuskoden indikerar korrekt avl�sning
		{
			ERROR = 1;
			break;
		}
		TWCR = (1<<TWINT | 1<<TWEN);			// laddar variablen med slavadressen till TWCR-registret
		TWCR |= (1<<TWEA);
		while ( !(TWCR & (1<<TWINT)));			// v�ntar p� flaggan s� att vi ser att SLA_W �r skickad
		if ((TWSR & 0xf8) != MT_DATA_ACK)		// kollar om statuskoden indikerar korrekt avl�sning
		{
			ERROR = 1;
			break;
		}
		DATA_16 = TWDR;
		if (BITS == 16)
		{
			DATA_16 = DATA_16<<8;
			TWCR = (1<<TWINT | 1<<TWEN);			// laddar variablen med slavadressen till TWCR-registret
			while ( !(TWCR & (1<<TWINT)));			// v�ntar p� flaggan s� att vi ser att SLA_W �r skickad
			if ((TWSR & 0xf8) != MT_DATA_NACK)		// kollar om statuskoden indikerar korrekt avl�sning
			{
				ERROR = 1;
				break;
			}
			DATA_16 |= TWDR;
		}
		break;
	}

}

void write_to_sensor()						// skickar DATA till DATA_adr som �r en adress p� sensorn
{
	for (;;)
	{
		MT_SLA_ACK = 0x18;						// SLA+W �r skickad och ACK �r mottagen
		MT_DATA_ACK = 0x28;						// DATA �r skickad och ACK �r mottagen

		TWDR = SLA_W;							// v�ljer slavadress samt att skriva till denna
		TWCR = (1<<TWINT | 1<<TWEN);			// laddar variablen med slavadressen till TWCR-registret
		while ( !(TWCR & (1<<TWINT)));			// v�ntar p� flaggan s� att vi ser att SLA_W �r skickad
		if ((TWSR & 0xf8) != MT_SLA_ACK)		// kollar om statuskoden indikerar korrekt avl�sning
		{
			ERROR = 1;
			break;
		}
		TWDR = DATA_adr;						// l�gger in adressen vi vill skriva till i sensorn p� TWDR
		TWCR = (1<<TWINT | 1<<TWEN);			// laddar variablen med slavadressen till TWCR-registret
		while ( !(TWCR & (1<<TWINT)));			// v�ntar p� flaggan s� att vi ser att SLA_W �r skickad
		if ((TWSR & 0xf8) != MT_DATA_ACK)		// kollar om statuskoden indikerar korrekt avl�sning
		{
			ERROR = 1;
			break;
		}
		TWDR = DATA;							// l�gger in datat vi vill skriva till sensorn p� TWDR
		TWCR = (1<<TWINT | 1<<TWEN);			// laddar variablen med slavadressen till TWCR-registret
		while ( !(TWCR & (1<<TWINT)));			// v�ntar p� flaggan s� att vi ser att SLA_W �r skickad
		if ((TWSR & 0xf8) != MT_DATA_ACK)		// kollar om statuskoden indikerar korrekt avl�sning
		{
			ERROR = 1;
			break;
		}
		break;
	}

}

void stop_cond()
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);	// stoppvillkor f�r �verrf�ring satt
}

void correct_value_lidar()
{
	if (9 < DATA_16 < 32)
	{
		DATA_16 = DATA_16 - 4;
	}
	else if(31 < DATA < 67)
	{
		DATA_16 = DATA_16 - 10;
	}
	else if(66 < DATA_16 < 97)
	{
		DATA_16 = DATA_16 - 7;
	}
}


void Wheel_sensor()
{
	ID = 0x8;
	ADMUX = (1<<REFS0 | 0<<REFS1 | 1<<ADLAR);   //reset old channel
	ADMUX |= (1<<MUX2 | 0<<MUX1 | 1<<MUX0);		//new channel
	ADCSRA |= (1 << ADSC);
	while(ADCSRA & (1<<ADSC));
	adc_wheel = ADC;							// White, adc_wheel < 20000 & Black, adc_wheel > 20000


	if ((adc_wheel > 20000 && previous_value < 20000) || (adc_wheel < 20000 && previous_value > 20000))
	{
		// Fr�n vitt till svart eller svart till vitt.
		distance_cm += 2;
	}
	if (distance_cm > 65000 || distance_cm == 65000)
	{
		distance_cm = 0;
	}
	previous_value = adc_wheel;
	DATA_low = distance_cm;
	DATA_high = distance_cm >> 8;
	send_data_control_module();
}
