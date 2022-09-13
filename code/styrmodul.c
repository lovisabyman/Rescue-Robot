/*
 * styrmodul.c
 *
 * Created: 2021-05-24 08:45:45
 *  Author: Gunnar Arctaedius och Lovisa Byman
 */

#define F_CPU 16000000UL


#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <util/delay.h>


//Main-variabler
volatile uint8_t phase = 0;

bool control_mode_done = false;
uint8_t control_mode = 0;
bool first_run_control_mode = true;

uint8_t planned_control_modes[1000];
uint16_t planned_control_modes_length;
bool plan_exists = false;

uint8_t current_shortest_path[1000];
uint16_t current_shortest_path_length = 0;
uint16_t current_shortest_dist = UINT16_MAX;

uint8_t map[51][101]; // 0 = normal, 1 = visited, 2 = no walls, 3 = search visited
uint8_t updated_segments[2][100]; //row sen col
uint8_t updated_segments_length = 0;

uint8_t delivery_row = 255; // 255 utanför. 0-100 jämna = väggar, 1-99 udda = rutor
uint8_t delivery_col = 255; // 255 utanför. 0-100 jämna = väggar, 1-99 udda = rutor
uint8_t curr_pos_row = 255; // 255 utanför. 0-100 jämna = väggar, 1-99 udda = rutor
uint8_t curr_pos_col = 255; // 255 utanför. 0-100 jämna = väggar, 1-99 udda = rutor
uint8_t direction = 0;

uint16_t right_thrust;
uint16_t left_thrust;
uint8_t right_direction;
uint8_t left_direction;
uint8_t claw = 30; //19 = 1ms pulser = helt stängd, 30 = 2ms pulser = helt öppen, värden mellan 15 och 32 okej

volatile bool tape_ready = false;
volatile bool spi_connected = false;

//Mapping
bool delivery_found = false;
bool can_turn_right = false;
bool can_turn_left = false;
bool can_go_straight = false;

//Execute_competition
uint16_t control_mode_index = 0;

//Mätvärden
volatile uint8_t tape = 0;
volatile uint16_t dist_right_front = 0;
volatile uint16_t dist_left_front = 0;
volatile uint16_t dist_right_back = 0;
volatile uint16_t dist_left_back = 0;
volatile uint16_t dist_forward = 0;
volatile uint16_t angle = 0;
volatile uint16_t distance_traveled = 0;
volatile uint8_t buttons = 0;
volatile uint8_t command = 0;

//Konstanter
const uint8_t sensor_width = 14;
const uint8_t module_distance = 39;
int8_t lidar_stop_distance = 15;

const uint8_t start_pos_row = 1;
const uint8_t start_pos_col = 51;

const uint8_t normal_map_index = 0;
const uint8_t visited_map_index = 1;
const uint8_t no_walls_map_index = 2;
const uint8_t search_visited_map_index = 3;

const uint16_t max_thrust = 1023;

//ID:n för dataöverföring
uint8_t tape_id = 1;
uint8_t dist_right_front_id = 2;
uint8_t dist_left_front_id = 3;
uint8_t dist_right_back_id = 4;
uint8_t dist_left_back_id = 5;
uint8_t dist_forward_id = 6;
uint8_t angle_id = 7;
uint8_t distance_traveled_id = 8;
uint8_t buttons_id = 9;
uint8_t sc_Kp_id = 10;
uint8_t sc_Ka_id = 11;
uint8_t t_Kp_id = 12;
uint8_t t_Kd_id = 13;
uint8_t tape_Kp_id = 14;
uint8_t command_id = 16;
uint8_t right_thrust_id = 17;
uint8_t left_thrust_id = 18;
uint8_t planned_control_modes_id = 19;
uint8_t current_shortest_path_id = 20;
uint8_t map_id = 21;
uint8_t delivery_row_id = 22;
uint8_t delivery_col_id = 23;
uint8_t curr_pos_row_id = 24;
uint8_t curr_pos_col_id = 25;
uint8_t direction_id = 26;
uint8_t updated_segments_id = 27;

//Storlekar på datapaket
const uint8_t buttons_data_size = 2;
const uint8_t parameters_data_size = 11;
const uint8_t sensor_data_size = 21;
const uint8_t thrust_data_size = 4;
const uint8_t planned_control_modes_data_size = 11;
const uint16_t map_data_size = 5152;
const uint8_t delivery_placement_data_size = 4;
const uint8_t robot_placement_data_size = 4;
const uint8_t direction_data_size = 2;
const uint8_t stop_data_size = 1;

// Tider för delay för displayen
const double t_su1 = 0.05;
const double t_w = 0.25;
const double t_c = 50;


//Variabler för köra rakt (straight_corridor)
volatile uint8_t sc_Kp = 50;
volatile uint8_t sc_Kp_one_wall = 50;
volatile uint8_t sc_Ka = 45;
volatile uint8_t sc_Ka_one_wall = 45;

int16_t sc_last_error = 0;

int16_t sc_max_thrust = 750; //1023 max

uint16_t local_dist_forward;
uint16_t local_distance_traveled;
int16_t local_angle;
uint16_t goal_distance_traveled;
uint16_t distance_since_back_open = 0;

uint16_t last_distance_traveled;
uint8_t last_dist_right_front;
uint8_t last_dist_right_back;
uint8_t last_dist_left_front;
uint8_t last_dist_left_back;

bool disable_left = false;
bool disable_right = false;

bool right_open_on_start = false;
bool left_open_on_start = false;

uint8_t use_sensor = 3;


//Variabler för svänga (turn)
volatile uint8_t t_Kp = 16;
volatile uint16_t t_Kd = 0;

int16_t t_last_error = 0;

uint16_t t_max_thrust = 511;

uint16_t control_mode_done_ctr = 0;

//Variabler för tejpföljning
volatile uint8_t tape_Kp = 150;

uint16_t tape_max_thrust = 300;

//Variabler för open
bool delivery_done = false;

//I2C och SPI
volatile uint8_t id_spi;
volatile uint8_t id_i2c;
volatile uint8_t i2c_counter = 0;

volatile bool dist_forward_done = false;
volatile int16_t sending_dist_forward = 0;
volatile bool angle_done = false;
volatile bool distance_traveled_done = false;

volatile uint8_t spi_recieve_counter= 0;
volatile uint16_t spi_send_counter = 0;
volatile uint8_t spi_send_data_size_counter = 0;
volatile uint16_t data_size = 0;
volatile bool should_recieve_data = false;
volatile bool data_is_id = true;
volatile bool gotten_stop = true;
volatile bool should_send_data = false;
volatile bool should_send_data_size = false;

volatile uint16_t package_starting_index = 0;
volatile bool package_done = true;
volatile uint8_t package_send_counter = 0;

volatile bool send_map_data = false;
volatile bool send_control_mode_data = false;
volatile bool send_new_phase_data = false;

volatile bool should_send_buttons = false;
volatile bool should_send_parameters = false;
volatile bool should_send_sensor_data = false;
volatile bool should_send_thrust = false;
volatile bool should_send_planned_control_modes = false;
volatile bool should_send_empty_planned_control_modes = false;
volatile bool should_send_updated_segments = false;
volatile uint8_t send_updated_segments_cnt;
volatile bool should_send_shortest_path = false;
volatile bool should_send_delivery_placement = false;
volatile bool should_send_robot_placement = false;
volatile bool should_send_direction = false;

//Manuell körning
uint16_t manual_speed = 1023;
uint32_t time_since_last_recieved_command = 0;

uint16_t claw_cnt = 0;



//Funktionsdeklarationer
void engine_setup();
void claw_setup();
void i2c_setup();
void spi_setup();
void map_setup();
void display_setup();
void clock_setup();
void clock_setdown();

void set_display(uint8_t display_row, uint8_t display_col, uint16_t display_value);

void manual_drive();

void start();
void mapping();
void stop_mapping();
void go_out();
void get_object();
void plan_competition();
void execute_competition();

void update_map();
void update_control_mode();
void follow_plan();
bool visited_right();
bool visited_left();
bool visited_straight();
void plan_new_route();
uint16_t get_dist(uint8_t map_index, uint8_t start_col, uint8_t start_row, uint8_t destination_col, uint8_t destination_row);
uint8_t get_path(bool use_current_shortest, uint8_t start_col, uint8_t start_row, uint8_t destination_col, uint8_t destination_row, uint8_t start_direction);
bool is_valid(uint8_t map_index, uint8_t row, uint8_t col);
void run_control_mode();

void straight_corridor();
void turn_right();
void turn_left();
void turn_around();
void turn(int16_t curr_error, int8_t direction_after_turn);
void follow_tape();
void open();
void slow_corridor(uint8_t wheel_direction);
void close();
void leave_maze();
void enter_maze();

void assign_recieved_data_i2c(uint8_t data);
void recieve_data();

void assign_recieved_data_spi(uint8_t data);
void spi_send_data_size();
void spi_send_data();
void send_buttons();
void send_parameters();
void send_sensor_data();
void send_thrust();
void send_planned_control_modes();
void send_empty_planned_control_modes();
void send_updated_segments();
void send_map();
void send_shortest_path();
void send_delivery_placement();
void send_robot_placement();
void send_direction();




int main(void) {

	engine_setup();
	claw_setup();
	spi_setup();
	map_setup();
	display_setup();
	i2c_setup();
	sei();


	_delay_ms(1000);
	spi_connected = true; //test


	if (buttons == 0) {
		clock_setup();
	}
	else {

	}

 	while(1)
    {
		if (control_mode_done) {
			set_display(1,1,curr_pos_row);
			set_display(2,1,curr_pos_col);
			set_display(1,5,phase);
			set_display(2,10,angle);
		}

		if (buttons == 0) {
			manual_drive();
		}
		else {
			switch(phase) {
				case 0:
				break;

				case 1:
				start();
				break;

				case 2:
				mapping();
				break;

				case 3:
				stop_mapping();
				break;

				case 4:
				go_out();
				break;

				case 5:
				get_object();
				break;

				case 6:
				plan_competition();
				break;

				case 7:
				execute_competition();
				break;
			}
		}
    }
}



void engine_setup() {
	//PWM-signal A=höger, B=vänster
	DDRD = (1<<DDD4) | (1<<DDD5);
	TCCR1A = (0<<COM1A0) | (1<<COM1A1) | (0<<COM1B0) | (1<<COM1B1) | (1<<WGM11) | (1<<WGM10); //PWM, Phase Correct, 10-bit, TOP = 0x03FF = 1023, prescale = 8
	TCCR1B = (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (1<<CS11) | (0<<CS10);
	OCR1A = 0; //Denna ändras för att ändra spänning. Värde mellan 0-1023. Höger motorpar
	OCR1B = 0; //Denna ändras för att ändra spänning. Värde mellan 0-1023. Vänster motorpar

	//DIR-signal D2 = vänster D3 = höger
	DDRD = DDRD | (1<<DDD2) | (1<<DDD3);
	PORTD = (1<<PORTD2) | (1<<PORTD3); //Riktining på hjulen, 1 = fram, 0 = bak
}

void claw_setup() {
	//PWM-signal gripklo
	DDRD |= (1<<DDD6);
	TCCR2A = (0<<COM2B0) | (1<<COM2B1) | (1<<WGM21) | (1<<WGM20);
	TCCR2B = (0<<WGM22) | (1<<CS22) | (1<<CS21) | (1<<CS20); //PWM, Fast, prescale 1024, Topp 255, borde ge frekvens 61 Hz, vilket ger bredd 16,4 ms
	OCR2B = 30; //stänger klon från början
}

void i2c_setup() {
	//I2C-setup
	TWAR = 3; //Adress 1, Enabled general calls
	TWCR = (0<<TWINT) | (1<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWWC) | (1<<TWEN) | (1<<TWIE);
}

void spi_setup() {
	//SPI-setup
	DDRB = (0<<DDB7) | (1<<DDB6) | (0<<DDB5) | (0<<DDB4);
	SPCR = (1<<SPIE) | (1<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA); //Just nu MSB först, byt DORD för att ändra
}

void map_setup() {
	//Sätter allt till 0 i alla kartor utom no_walls, där allt utom kanterna sätts till 1.
	for (uint8_t r = 0; r<=50; r++) {
		for (uint8_t c = 0; c<=100; c++) {
			if (r == 0 || r == 50 || c == 0 || c == 100 || (r % 2 == 0 && c % 2 == 0)) {
				map[r][c] = 0;
			}
			else {
				map[r][c] = (1<<no_walls_map_index);
			}
		}
	}
	//sätter ingången till 1 på vanliga kartan
	map[0][51] = 1;
	map[1][51] = 5;
}

void display_setup() {
	DDRA = 0xff;
	DDRB |= (1<<DDB0 | (1<<DDB1) | (1<<DDB2));
	// vänta 30 ms (ev mer)
	_delay_ms(30);
	//Function set
	PORTB &= ~(1<<PORTB2 | 1<<PORTB1);
	_delay_us(t_su1);
	PORTB |= (1<<PORTB0);
	PORTA = (0<<PORTA7 | 0<<PORTA6 | 1<<PORTA5 | 1<<PORTA4 | 1<<PORTA3 | 0<<PORTA2); //Function set, N=2-line mode, F = 5x8 Dots
	_delay_us(t_w);
	PORTB &= ~(1<<PORTB0);

	// vänta 39 us
	_delay_us(39);

	//Display ON/OFF Control
	PORTB &= ~(1<<PORTB2 | 1<<PORTB1);
	_delay_us(t_su1);
	PORTB |= (1<<PORTB0);
	PORTA = (0<<PORTA7 | 0<<PORTA6 | 0<<PORTA5 | 0<<PORTA4 | 1<<PORTA3 | 1<<PORTA2 | 1<<PORTA1 | 0<<PORTA0); //Display ON/OFF Control, Display ON, Cursor ON, Blink OFF
	_delay_us(t_w);
	PORTB &= ~(1<<PORTB0);

	//vänta 39 us
	_delay_us(39);

	//Display clear
	PORTB &= ~(1<<PORTB2 | 1<<PORTB1);
	_delay_us(t_su1);
	PORTB |= (1<<PORTB0);
	PORTA = (0<<PORTA7 | 0<<PORTA6 | 0<<PORTA5 | 0<<PORTA4 | 0<<PORTA3 | 0<<PORTA2 | 0<<PORTA1 | 1<<PORTA0); //Display clear
	_delay_us(t_w);
	PORTB &= ~(1<<PORTB0);

	//vänta 1.53 ms
	_delay_ms(1.6);

	//Entry mode set
	PORTB &= ~(1<<PORTB2 | 1<<PORTB1);
	_delay_us(t_su1);
	PORTB |= (1<<PORTB0);
	PORTA = (0<<PORTA7 | 0<<PORTA6 | 0<<PORTA5 | 0<<PORTA4 | 0<<PORTA3 | 1<<PORTA2 | 1<<PORTA1 | 0<<PORTA0); //Entry mode set, Increment mode, Entire shift off
	_delay_us(t_w);
	PORTB &= ~(1<<PORTB0);
	_delay_us(t_c);
}

void clock_setup() { //Ger avbrott 62ggr per sekund
	DDRB |= (1<<DDB3);
	TCCR0A |= (1<<COM0A1)|(0<<COM0A0)|(1<<WGM01)|(0<<WGM00);
	TCCR0B |= (0<<WGM02)|(0<<FOC0A)|(0<<FOC0B)|(1<<CS02)|(0<<CS01)|(1<<CS00);
	OCR0A = 255;
	TIMSK0 = (1<<OCIE0A);
}

void clock_setdown() {
	TIMSK0 = (0<<OCIE0A);
}



void set_display(uint8_t display_row, uint8_t display_col, uint16_t display_value) {

	//Set DDRAM Address
	uint8_t display_adress;

	if (display_row == 1) {
		display_adress = display_col - 1;
	}
	else {
		display_adress = 0x40 + display_col - 1;
	}
	PORTB &= ~(1<<PORTB2 | 1<<PORTB1);
	_delay_us(t_su1);
	PORTB |= (1<<PORTB0);
	PORTA = (1<<PORTA7) + display_adress;
	_delay_us(t_w);
	PORTB &= ~(1<<PORTB0);

	_delay_us(t_c);


	//Write data to RAM
	uint8_t display_character_code;

	PORTB &= ~(1<<PORTB1);
	PORTB |= (1<<PORTB2);
	_delay_us(t_su1);
	PORTB |= (1<<PORTB0);
	display_character_code = 0x30 + ((display_value % 1000) / 100); //hundratals-siffran
	PORTA = display_character_code;
	_delay_us(t_w);
	PORTB &= ~(1<<PORTB0);

	_delay_us(t_c);

	PORTB &= ~(1<<PORTB1);
	PORTB |= (1<<PORTB2);
	_delay_us(t_su1);
	PORTB |= (1<<PORTB0);
	display_character_code = 0x30 + ((display_value % 100) / 10); //tiotalssiffran-siffran
	PORTA = display_character_code;
	_delay_us(t_w);
	PORTB &= ~(1<<PORTB0);

	_delay_us(t_c);

	PORTB &= ~(1<<PORTB1);
	PORTB |= (1<<PORTB2);
	_delay_us(t_su1);
	PORTB |= (1<<PORTB0);
	display_character_code = 0x30 + (display_value % 10); //entals-siffran
	PORTA = display_character_code;
	_delay_us(t_w);
	PORTB &= ~(1<<PORTB0);

	_delay_us(t_c);
}



void manual_drive() {
	if (time_since_last_recieved_command++ > 160000) {
		time_since_last_recieved_command = 0;
		command = 0;
	}
	switch(command) {
		case 0: //Stilla
		right_thrust = 0;
		left_thrust = 0;
		break;
		case 1: //Framåt
		right_direction = 1;
		left_direction = 1;
		right_thrust = manual_speed;
		left_thrust = manual_speed;
		break;
		case 2: //Bakåt
		right_direction = 0;
		left_direction = 0;
		right_thrust = manual_speed;
		left_thrust = manual_speed;
		break;
		case 3: //Rotera höger
		right_direction = 0;
		left_direction = 1;
		right_thrust = manual_speed;
		left_thrust = manual_speed;
		break;
		case 4: //Rotera vänster
		right_direction = 1;
		left_direction = 0;
		right_thrust = manual_speed;
		left_thrust = manual_speed;
		break;
		case 5: //Öppna gripklo
		right_thrust = 0;
		left_thrust = 0;
		claw_cnt ++;
		if (claw_cnt >= 15000){
			if (claw < 30) {
				claw ++;
			}
			claw_cnt = 0;
		}
		break;
		case 6: //Stäng gripklo
		right_thrust = 0;
		left_thrust = 0;
		claw_cnt ++;
		if (claw_cnt >= 15000){
			if (claw > 19) {
				claw --;
			}
			claw_cnt = 0;
		}
		break;
		case 7: //Fram höger
		right_thrust = manual_speed/4;
		left_thrust = manual_speed;
		right_direction = 1;
		left_direction = 1;
		break;
		case 8: //Fram vänster
		right_thrust = manual_speed;
		left_thrust = manual_speed/4;
		right_direction = 1;
		left_direction = 1;
		break;
	}
	PORTD = (left_direction<<PORTD2) | (right_direction<<PORTD3);
	OCR1A = right_thrust;
	OCR1B = left_thrust;
	OCR2B = claw;
}

ISR(TIMER0_COMPA_vect) {
	time_since_last_recieved_command++;
}



void start() {

	if (spi_connected && tape_ready) {
		if(control_mode_done) {
			control_mode_done = true;
			plan_exists = false;
			phase = 2;
			return;
		}

		left_direction = 1;
		right_direction = 1;
		left_thrust = sc_max_thrust;
		right_thrust = sc_max_thrust;

		PORTD = (left_direction<<PORTD2) | (right_direction<<PORTD3); //Riktning: framåt
		OCR1A = left_thrust;
		OCR1B = right_thrust;

		cli();
		if (distance_traveled_done) {
			local_distance_traveled = distance_traveled;
		}
		sei();

		if(first_run_control_mode) {
			goal_distance_traveled = local_distance_traveled + 51; //kör från toppen av "T" till mitten av första modulen
			first_run_control_mode = false;
		}

		if (local_distance_traveled >= goal_distance_traveled) {
			control_mode_done = true;
			first_run_control_mode = true;
			send_control_mode_data = true;
			if (curr_pos_row == 255) {
				curr_pos_col = 51;
				curr_pos_row = 1;
			}
		}

	}
}

void mapping() {

	//kolla om nödställd funnen
	if (tape == 0x1F) {
		delivery_found = true;
	}

	if(control_mode_done) {
		first_run_control_mode = true;
		if (!plan_exists) {//om det redan finns en plan
			if (delivery_found) {
				delivery_row = curr_pos_row;
				delivery_col = curr_pos_col;
				plan_exists = false;
 				planned_control_modes_length = 0;
 				send_new_phase_data = true;
				control_mode_done = true;
 				phase = 3;
				return;
			}
			update_map();
			update_control_mode();
		}

		follow_plan(); //kör efter plan (sätt rätt control_mode)
		control_mode_done = false;
	}

	run_control_mode();
}

void stop_mapping() {
	if(control_mode_done) {
		first_run_control_mode = true;
		if (!plan_exists) {
			update_map();
			uint16_t curr_dist = get_dist(normal_map_index, delivery_col, delivery_row, start_pos_col, start_pos_row);
			if (curr_dist < current_shortest_dist) {
				current_shortest_dist = curr_dist;
				current_shortest_path_length = 0;
				get_path(true, start_pos_col, start_pos_row, delivery_col, delivery_row, 0);
			}
			if (curr_dist <= get_dist(no_walls_map_index, delivery_col, delivery_row, start_pos_col, start_pos_row)) {
				plan_exists = false;
				planned_control_modes_length = 0;
				send_new_phase_data = true;
				control_mode_done = true;
				phase = 4;
				return;
			}
			update_control_mode();
		}
		follow_plan(); //kör efter plan (sätt rätt control_mode)
		control_mode_done = false;
	}

	run_control_mode();
}

void go_out() {
	if (control_mode_done) {
		first_run_control_mode = true;
		if (!(plan_exists)) {
			if ((curr_pos_col == start_pos_col) && (curr_pos_row == start_pos_row)) { //om vi inte har en plan och inte är framme skapas planen
				switch (direction) {
					case 0:
					planned_control_modes[planned_control_modes_length++] = 3;
					plan_exists = true;
					break;
					case 1:
					planned_control_modes[planned_control_modes_length++] = 1;
					plan_exists = true;
					break;
					case 2:
					plan_exists = false;
					planned_control_modes_length = 0;
					phase = 5;
					return;
					break;
					case 3:
					planned_control_modes[planned_control_modes_length++] = 2;
					plan_exists = true;
					break;
				}
			}
			else { //om vi är vid startrutan, vänder oss till öppningen. Då byts fasen.
				get_path(false, curr_pos_col, curr_pos_row, start_pos_col, start_pos_row, direction);
				plan_exists = true;
			}
		}
		follow_plan();
		control_mode_done = false;
	}
	run_control_mode();
}

void get_object() {
	if (!plan_exists) {
		if (tape != 0) { //Om det finns tejp
			planned_control_modes[planned_control_modes_length++] = 4;
			planned_control_modes[planned_control_modes_length++] = 5;
			planned_control_modes[planned_control_modes_length++] = 3;
			plan_exists = true;
			control_mode_done = true;
			follow_plan();
		}
		else { //Om det inte finns tejp, kör framåt
			control_mode = 0;
		}
	}
	else if (control_mode_done){
		first_run_control_mode = true;
		if (direction == 0) { //Kollar mot ingången, objektet upplockat
			plan_exists = false;
			phase = 6;
			return;
		}
		follow_plan();
		plan_exists = true;
		control_mode_done = false;
	}
	run_control_mode();

}

void plan_competition() {
	uint8_t new_direction;
	planned_control_modes_length = 0;
	planned_control_modes[planned_control_modes_length ++] = 8; //kör in i labyrint
	new_direction = get_path(false, start_pos_col, start_pos_row, delivery_col, delivery_row, direction); //kör till nödställd
	planned_control_modes_length--; //ta bort sista så vi stannar en ruta framför nödställd
	planned_control_modes[planned_control_modes_length ++] = 6;	//"open" alltså kör fram, lämna, backa
	uint8_t one_before_del_row = delivery_row;
	uint8_t one_before_del_col = delivery_col;
	switch(new_direction) {
		case 0:
		one_before_del_row = delivery_row - 2;
		break;
		case 1:
		one_before_del_col = delivery_col + 2;
		break;
		case 2:
		one_before_del_row = delivery_row + 2;
		break;
		case 3:
		one_before_del_col = delivery_col - 2;
		break;
	}
	new_direction = get_path(false, one_before_del_col, one_before_del_row, start_pos_col, start_pos_row, new_direction); //kör från en innan nödställd till start

	switch(new_direction) { //vänd mot ingång
		case 0:
		planned_control_modes[planned_control_modes_length ++] = 3;
		break;
		case 1:
		planned_control_modes[planned_control_modes_length ++] = 1;
		break;
		case 3:
		planned_control_modes[planned_control_modes_length ++] = 2;
		break;
	}
	planned_control_modes[planned_control_modes_length ++] = 7; //kör ut ur labyrint
	control_mode_done = true;
	phase = 7;
}

void execute_competition() {
	if (control_mode_done) {
		first_run_control_mode = true;
		if (control_mode_index == planned_control_modes_length) {
			control_mode_done = false;
			plan_exists = false;
			planned_control_modes_length = 0;
			first_run_control_mode = true;
			phase = 0;
			return;
		}
		else {
			control_mode = planned_control_modes[control_mode_index++];
		}
		control_mode_done = false;
	}
	run_control_mode();
}



void run_control_mode() {
	switch (control_mode)
	{
		case 0:
		straight_corridor();
		break;
		case 1:
		turn_right();
		break;
		case 2:
		turn_left();
		break;
		case 3:
		turn_around();
		break;
		case 4:
		follow_tape();
		break;
		case 5:
		close();
		break;
		case 6:
		open();
		break;
		case 7:
		leave_maze();
		break;
		case 8:
		enter_maze();
		break;
	}
}

void update_map() {

	cli();
	if (dist_forward_done) {
		local_dist_forward = dist_forward;
	}
	uint8_t local_dist_left_front = dist_left_front;
	uint8_t local_dist_right_front = dist_right_front;
	sei();


	//uppdatera besökt karta
	switch (direction) {
		case 0: //syd
		map[curr_pos_row-1][curr_pos_col] |= (1<<visited_map_index); //sätter andra biten till 1
		updated_segments[0][updated_segments_length] = curr_pos_row - 1; //lägger till segmentet i updated segments så att det skickas nästa gång kartdata skickas
		updated_segments[1][updated_segments_length++] = curr_pos_col;
		break;
		case 1: //väst
		map[curr_pos_row][curr_pos_col+1] |= (1<<visited_map_index);
		updated_segments[0][updated_segments_length] = curr_pos_row;
		updated_segments[1][updated_segments_length++] = curr_pos_col + 1;
		break;
		case 2: //norr
		map[curr_pos_row+1][curr_pos_col] |= (1<<visited_map_index);
		updated_segments[0][updated_segments_length] = curr_pos_row + 1;
		updated_segments[1][updated_segments_length++] = curr_pos_col;
		break;
		case 3: //öst
		map[curr_pos_row][curr_pos_col-1] |= (1<<visited_map_index);
		updated_segments[0][updated_segments_length] = curr_pos_row;
		updated_segments[1][updated_segments_length++] = curr_pos_col - 1;
		break;
	}
	map[curr_pos_row][curr_pos_col] |= (1<<visited_map_index);
	updated_segments[0][updated_segments_length] = curr_pos_row;
	updated_segments[1][updated_segments_length++] = curr_pos_col;


	//uppdaterar normal map och no walls map om det är öppet
	if (local_dist_right_front == 0 || local_dist_right_front > 20) { //öppet till höger
		can_turn_right = true;
		switch (direction) {
			case 0: //tittar åt syd, dvs öppet åt väst
			map[curr_pos_row][curr_pos_col-1] |= (1<<normal_map_index | 1<<no_walls_map_index);
			map[curr_pos_row][curr_pos_col-2] |= (1<<normal_map_index | 1<<no_walls_map_index);
			updated_segments[0][updated_segments_length] = curr_pos_row;
			updated_segments[1][updated_segments_length++] = curr_pos_col - 1;
			updated_segments[0][updated_segments_length] = curr_pos_row;
			updated_segments[1][updated_segments_length++] = curr_pos_col - 2;
			break;

			case 1: //tittar åt väst, dvs öppet åt norr
			if (curr_pos_row >1) {
				map[curr_pos_row-1][curr_pos_col] |= (1<<normal_map_index | 1<<no_walls_map_index);
				map[curr_pos_row-2][curr_pos_col] |= (1<<normal_map_index | 1<<no_walls_map_index);
				updated_segments[0][updated_segments_length] = curr_pos_row - 1;
				updated_segments[1][updated_segments_length++] = curr_pos_col;
				updated_segments[0][updated_segments_length] = curr_pos_row - 2;
				updated_segments[1][updated_segments_length++] = curr_pos_col;
			}
			break;

			case 2: //tittar åt norr, dvs öppet åt öst
			map[curr_pos_row][curr_pos_col+1] |= (1<<normal_map_index | 1<<no_walls_map_index);
			map[curr_pos_row][curr_pos_col+2] |= (1<<normal_map_index | 1<<no_walls_map_index);
			updated_segments[0][updated_segments_length] = curr_pos_row;
			updated_segments[1][updated_segments_length++] = curr_pos_col + 1;
			updated_segments[0][updated_segments_length] = curr_pos_row;
			updated_segments[1][updated_segments_length++] = curr_pos_col + 2;
			break;

			case 3: //tittar åt öst, dvs öppet åt syd
			map[curr_pos_row+1][curr_pos_col] |= (1<<normal_map_index | 1<<no_walls_map_index);
			map[curr_pos_row+2][curr_pos_col] |= (1<<normal_map_index | 1<<no_walls_map_index);
			updated_segments[0][updated_segments_length] = curr_pos_row + 1;
			updated_segments[1][updated_segments_length++] = curr_pos_col;
			updated_segments[0][updated_segments_length] = curr_pos_row + 2;
			updated_segments[1][updated_segments_length++] = curr_pos_col;
			break;
		}
	}
	else {
		can_turn_right = false;
	}

	if (local_dist_left_front == 0 || local_dist_left_front > 20) { //öppet till vänster
		can_turn_left = true;
		switch (direction) {
			case 0: //tittar åt syd, dvs öppet åt öst
			map[curr_pos_row][curr_pos_col+1] |= (1<<normal_map_index | 1<<no_walls_map_index);
			map[curr_pos_row][curr_pos_col+2] |= (1<<normal_map_index | 1<<no_walls_map_index);
			updated_segments[0][updated_segments_length] = curr_pos_row;
			updated_segments[1][updated_segments_length++] = curr_pos_col + 1;
			updated_segments[0][updated_segments_length] = curr_pos_row;
			updated_segments[1][updated_segments_length++] = curr_pos_col + 2;
			break;

			case 1: //tittar åt väst, dvs öppet åt syd
			map[curr_pos_row+1][curr_pos_col] |= (1<<normal_map_index | 1<<no_walls_map_index);
			map[curr_pos_row+2][curr_pos_col] |= (1<<normal_map_index | 1<<no_walls_map_index);
			updated_segments[0][updated_segments_length] = curr_pos_row + 1;
			updated_segments[1][updated_segments_length++] = curr_pos_col;
			updated_segments[0][updated_segments_length] = curr_pos_row + 2;
			updated_segments[1][updated_segments_length++] = curr_pos_col;
			break;

			case 2: //tittar åt norr, dvs öppet åt väst
			map[curr_pos_row][curr_pos_col-1] |= (1<<normal_map_index | 1<<no_walls_map_index);
			map[curr_pos_row][curr_pos_col-2] |= (1<<normal_map_index | 1<<no_walls_map_index);
			updated_segments[0][updated_segments_length] = curr_pos_row;
			updated_segments[1][updated_segments_length++] = curr_pos_col - 1;
			updated_segments[0][updated_segments_length] = curr_pos_row;
			updated_segments[1][updated_segments_length++] = curr_pos_col - 2;
			break;

			case 3: //tittar åt öst, dvs öppet åt norr
			if (curr_pos_row > 1) {
				map[curr_pos_row-1][curr_pos_col] |= (1<<normal_map_index | 1<<no_walls_map_index);
				map[curr_pos_row-2][curr_pos_col] |= (1<<normal_map_index | 1<<no_walls_map_index);
				updated_segments[0][updated_segments_length] = curr_pos_row - 1;
				updated_segments[1][updated_segments_length++] = curr_pos_col;
				updated_segments[0][updated_segments_length] = curr_pos_row - 2;
				updated_segments[1][updated_segments_length++] = curr_pos_col;
			}
			break;
		}
	}
	else {
		can_turn_left = false;
	}


	uint8_t open_modules_forward = local_dist_forward / 40;
	uint8_t next_wall = 2*open_modules_forward + 1;

	if (open_modules_forward >= 1) { //öppet en modul eller mer framåt
		can_go_straight = true;
		for (uint8_t i = 1; i < next_wall; i++)
		{
			switch (direction) {
				case 0: //tittar åt syd, dvs öppet åt syd
				map[curr_pos_row+i][curr_pos_col] |= (1<<normal_map_index | 1<<no_walls_map_index);
				updated_segments[0][updated_segments_length] = curr_pos_row + i;
				updated_segments[1][updated_segments_length++] = curr_pos_col;
				break;

				case 1: //tittar åt väst, dvs öppet åt väst
				map[curr_pos_row][curr_pos_col-i] |= (1<<normal_map_index | 1<<no_walls_map_index);
				updated_segments[0][updated_segments_length] = curr_pos_row;
				updated_segments[1][updated_segments_length++] = curr_pos_col - i;
				break;

				case 2: //tittar åt norr, dvs öppet åt norr
				if (curr_pos_row >= i) {
					map[curr_pos_row-i][curr_pos_col] |= (1<<normal_map_index | 1<<no_walls_map_index);
					updated_segments[0][updated_segments_length] = curr_pos_row - i;
					updated_segments[1][updated_segments_length++] = curr_pos_col;
				}
				break;

				case 3: //tittar åt öst, dvs öppet åt öst
				map[curr_pos_row][curr_pos_col+i] |= (1<<normal_map_index | 1<<no_walls_map_index);
				updated_segments[0][updated_segments_length] = curr_pos_row;
				updated_segments[1][updated_segments_length++] = curr_pos_col + i;
				break;
			}
		}
	}
	else {
		can_go_straight = false;
	}

	//uppdaterar normal map och no walls map om det är stängt
	if (local_dist_right_front != 0) { //stängt till höger
		switch (direction) {
			case 0: //tittar åt syd, dvs stängt åt väst
			map[curr_pos_row][curr_pos_col-1] &= ~(1<<normal_map_index | 1<<no_walls_map_index); // sätter tredje biten till 0
			break;

			case 1: //tittar åt väst, dvs stängt åt norr
			map[curr_pos_row-1][curr_pos_col] &= ~(1<<normal_map_index | 1<<no_walls_map_index);
			break;

			case 2: //tittar åt norr, dvs stängt åt öst
			map[curr_pos_row][curr_pos_col+1] &= ~(1<<normal_map_index | 1<<no_walls_map_index);
			break;

			case 3: //tittar åt öst, dvs stängt åt syd
			map[curr_pos_row+1][curr_pos_col] &= ~(1<<normal_map_index | 1<<no_walls_map_index);
			break;
		}
	}

	if (local_dist_left_front != 0) { //stängt till vänster
		switch (direction) {
			case 0: //tittar åt syd, dvs stängt åt öst
			map[curr_pos_row][curr_pos_col+1] &= ~(1<<normal_map_index | 1<<no_walls_map_index);
			break;

			case 1: //tittar åt väst, dvs stängt åt syd
			map[curr_pos_row+1][curr_pos_col] &= ~(1<<normal_map_index | 1<<no_walls_map_index);
			break;

			case 2: //tittar åt norr, dvs stängt åt väst
			map[curr_pos_row][curr_pos_col-1] &= ~(1<<normal_map_index | 1<<no_walls_map_index);
			break;

			case 3: //tittar åt öst, dvs stängt åt norr
			map[curr_pos_row-1][curr_pos_col] &= ~(1<<normal_map_index | 1<<no_walls_map_index);
			break;
		}
	}

	switch (direction) { //stängt framåt
		case 0: //tittar åt syd, dvs stängt åt syd
		map[curr_pos_row+next_wall][curr_pos_col] &= ~(1<<normal_map_index | 1<<no_walls_map_index);
		break;

		case 1: //tittar åt väst, dvs stängt åt väst
		map[curr_pos_row][curr_pos_col-next_wall] &= ~(1<<normal_map_index | 1<<no_walls_map_index);
		break;

		case 2: //tittar åt norr, dvs stängt åt norr
		if (curr_pos_row >= next_wall) {
			map[curr_pos_row-next_wall][curr_pos_col] &= ~(1<<normal_map_index | 1<<no_walls_map_index);
		}
		break;

		case 3: //tittar åt öst, dvs stängt åt öst
		map[curr_pos_row][curr_pos_col+next_wall] &= ~(1<<normal_map_index | 1<<no_walls_map_index);
		break;
	}


	// Uppdatera visited för att sätta "väggar" till visited om vi vet att det inte finns någon vägg där och vi varit på båda sidor av väggen
	if (curr_pos_row + 2 <= 50 && (map[curr_pos_row+2][curr_pos_col] & (1<<visited_map_index)) != 0 && (map[curr_pos_row+1][curr_pos_col] & (1<<normal_map_index)) != 0 && (map[curr_pos_row+1][curr_pos_col] & (1<<visited_map_index)) == 0)  {
		map[curr_pos_row+1][curr_pos_col] |= (1<<visited_map_index);
		updated_segments[0][updated_segments_length] = curr_pos_row + 1;
		updated_segments[1][updated_segments_length++] = curr_pos_col;
	}
	if (curr_pos_col - 2 >= 0 && (map[curr_pos_row][curr_pos_col-2] & (1<<visited_map_index)) != 0 && (map[curr_pos_row][curr_pos_col-1] & (1<<normal_map_index)) != 0 && (map[curr_pos_row][curr_pos_col-1] & (1<<visited_map_index)) == 0)  {
		map[curr_pos_row][curr_pos_col-1] |= (1<<visited_map_index);
		updated_segments[0][updated_segments_length] = curr_pos_row;
		updated_segments[1][updated_segments_length++] = curr_pos_col - 1;
	}
	if (curr_pos_row - 2 >= 0 && (map[curr_pos_row-2][curr_pos_col] & (1<<visited_map_index)) != 0 && (map[curr_pos_row-1][curr_pos_col] & (1<<normal_map_index)) != 0 && (map[curr_pos_row-1][curr_pos_col] & (1<<visited_map_index)) == 0)  {
		map[curr_pos_row-1][curr_pos_col] |= (1<<visited_map_index);
		updated_segments[0][updated_segments_length] = curr_pos_row - 1;
		updated_segments[1][updated_segments_length++] = curr_pos_col;
	}
	if (curr_pos_col + 2 <= 100 && (map[curr_pos_row][curr_pos_col+2] & (1<<visited_map_index)) != 0 && (map[curr_pos_row][curr_pos_col+1] & (1<<normal_map_index)) != 0 && (map[curr_pos_row][curr_pos_col+1] & (1<<visited_map_index)) == 0)  {
		map[curr_pos_row][curr_pos_col+1] |= (1<<visited_map_index);
		updated_segments[0][updated_segments_length] = curr_pos_row;
		updated_segments[1][updated_segments_length++] = curr_pos_col + 1;
	}


	// skicka kart-data till kommunikationsmodul i nästa avbrott
	send_map_data = true;
}

void update_control_mode() {
	//kör rakt om kan och inte varit där.
	if (can_go_straight && !visited_straight()) {
		planned_control_modes[planned_control_modes_length++]  = 0; //kör rakt
	}
	//annars höger
	else if (can_turn_right && !visited_right() ) {
		planned_control_modes[planned_control_modes_length++]  = 1; // rotera höger
		planned_control_modes[planned_control_modes_length++]  = 0;
	}
	//annars vänster
	else if (can_turn_left && !visited_left()) {
		planned_control_modes[planned_control_modes_length++]  = 2; // rotera vänster
		planned_control_modes[planned_control_modes_length++]  = 0;
	}
	//annars planera om genom att köra till närmsta korsning där det finns vägar kvar att köra på
	else {
		plan_new_route();
	}
	plan_exists = true;
}

void plan_new_route() {

 	uint8_t choises_col[100];
 	uint8_t choises_row[100];
	uint8_t index = 0;
	bool should_add = false;
	for (uint8_t r = 1; r<50; r+=2) {
		for (uint8_t c = 1; c<100; c+=2) {
			if ((map[r][c] & (1<<visited_map_index)) != 0) {
				//kollar om man kan åka till grannarna och om dom är visited (r > 1 i första är för att inte lägga till utgången som ett val.)
				if(((r > 1 && map[r-1][c] & (1<<normal_map_index)) != 0 && (map[r-1][c] & (1<<visited_map_index)) == 0) || ((map[r+1][c] & (1<<normal_map_index)) != 0 && (map[r+1][c] & (1<<visited_map_index)) == 0) || ((map[r][c-1] & (1<<normal_map_index)) != 0 && (map[r][c-1] & (1<<visited_map_index)) == 0) || ((map[r][c+1] & (1<<normal_map_index)) != 0 && (map[r][c+1] & (1<<visited_map_index)) == 0)) {
					should_add = true;
				}
				if (should_add) {
					choises_col[index] = c;
					choises_row[index] = r;
					index++;
					should_add = false;
				}
			}
		}
	}

	uint16_t curr_dist;
	uint16_t shortest_dist = UINT16_MAX;
	uint8_t plan_point_col = start_pos_col;
	uint8_t plan_point_row = start_pos_row;
	for (uint8_t i = 0; i<index; i++) {
		curr_dist = get_dist(normal_map_index, curr_pos_col, curr_pos_row, choises_col[i], choises_row[i]);
		if (curr_dist <= shortest_dist) {
			shortest_dist = curr_dist;
			plan_point_col = choises_col[i];
			plan_point_row = choises_row[i];
		}
	}

	//Tar fram styrmoderna och uppdaterar planned_control_modes
	get_path(false, curr_pos_col, curr_pos_row, plan_point_col, plan_point_row, direction);
}

uint16_t get_dist(uint8_t map_index, uint8_t start_col, uint8_t start_row, uint8_t destination_col, uint8_t destination_row) {

	//sätt search_visited till 0
	for (uint8_t r = 0; r<=50; r++) {
		for (uint8_t c = 0; c<=100; c++) {
			map[r][c] &= ~(1<<search_visited_map_index);
		}
	}

	struct Node{
		uint8_t col;
		uint8_t row;
		uint16_t dist;
		};

	//skapa en tom kö
	struct Node queue[1250];
	uint16_t queue_first = 0;
	uint16_t queue_last = 1;

	//markera startmodulen som besökt och köa denna
	map[start_row][start_col] |= (1<<search_visited_map_index);

	struct Node start_node = {start_col,start_row,0};
	queue[0] = start_node;

	//sparar längden av den längsta vägen från startmodulen till slutmodulen
	uint16_t min_dist = UINT16_MAX;

	//går igenom kön
	while(queue_first != queue_last) {
		struct Node node = queue[queue_first];

		uint8_t i = node.col, j = node.row, dist = node.dist;

		//slutar gå igenom kön om destinationen är funnen
		if (i == destination_col && j == destination_row) {
			min_dist = dist;
			break;
		}

		//lägger till tillgängliga grannmoduler på kön och ökar avståndet med ett
		if (is_valid(map_index,j+1,i) && is_valid(map_index,j+2,i)) { //undersöker noden till syd
			map[j+1][i] |= (1<<search_visited_map_index);
			map[j+2][i] |= (1<<search_visited_map_index);
			struct Node next_node = {i,j+2,dist+1};
			queue[queue_last] = next_node;
			queue_last++;
		}
		if (is_valid(map_index,j,i-1) && is_valid(map_index,j,i-2)) { //undersöker noden till väst
			map[j][i-1] |= (1<<search_visited_map_index);
			map[j][i-2] |= (1<<search_visited_map_index);
			struct Node next_node = {i-2,j,dist+1};
			queue[queue_last] = next_node;
			queue_last++;
		}
		if (is_valid(map_index,j,i+1) && is_valid(map_index,j,i+2)) { //undersöker noden till öst
			map[j][i+1] |= (1<<search_visited_map_index);
			map[j][i+2] |= (1<<search_visited_map_index);
			struct Node next_node = {i+2,j,dist+1};
			queue[queue_last] = next_node;
			queue_last++;
		}
		if (is_valid(map_index,j-1,i) && is_valid(map_index,j-2,i)) { //undersöker noden till norr
			map[j-1][i] |= (1<<search_visited_map_index);
			map[j-2][i] |= (1<<search_visited_map_index);
			struct Node next_node = {i,j-2,dist+1};
			queue[queue_last] = next_node;
			queue_last++;
		}
		//gå vidare till nästa nod
		queue_first++;

	}
	return min_dist;
}

uint8_t get_path(bool use_current_shortest, uint8_t start_col, uint8_t start_row, uint8_t destination_col, uint8_t destination_row, uint8_t start_direction) {
	uint8_t new_direction = start_direction;

	//sätt search_visited till 0
	for (uint8_t r = 0; r<=50; r++) {
		for (uint8_t c = 0; c<=100; c++) {
			map[r][c] &= ~(1<<search_visited_map_index);
		}
	}

	struct Node{
		uint8_t col;
		uint8_t row;
		uint16_t last_queue_index;
	};

	//skapa en tom kö
	struct Node queue[1250];
	uint16_t queue_first = 0;
	uint16_t queue_last = 1;

	//markera startmodulen som besökt och köa denna
	map[destination_row][destination_col] |= (1<<search_visited_map_index);

	struct Node start_node = {destination_col,destination_row,UINT16_MAX};
	queue[0] = start_node;

	//går igenom kön
	while(queue_first != queue_last) {

		struct Node node = queue[queue_first];

		uint8_t i = node.col;
		uint8_t j = node.row;

		if (i == start_col && j == start_row) {

			//Tar fram vägen från slutposition till startposition, så att den sista noden på kön är startpositionen, och planned_control_modes tas fram genom att nästla upp kön utifrån det.
			uint8_t curr_module_index = queue_first;
			uint8_t next_path_direction = 0;
			while (queue[curr_module_index].last_queue_index != UINT16_MAX) {
				uint16_t previous_module_index = queue[curr_module_index].last_queue_index;
				if (queue[curr_module_index].col == queue[previous_module_index].col + 2) {
					next_path_direction = 1;
				}
				if (queue[curr_module_index].col == queue[previous_module_index].col - 2) {
					next_path_direction = 3;
				}
				if (queue[curr_module_index].row == queue[previous_module_index].row + 2) {
					next_path_direction = 2;
				}
				if (queue[curr_module_index].row == queue[previous_module_index].row - 2) {
					next_path_direction = 0;
				}

				if (use_current_shortest) {
					if(new_direction == next_path_direction) { //ska åka rakt
						current_shortest_path[current_shortest_path_length++] = 0;
					}
					else if ((new_direction+1) % 4 == next_path_direction) { //ska åka höger
						current_shortest_path[current_shortest_path_length++] = 1;
						current_shortest_path[current_shortest_path_length++] = 0;
						new_direction = (new_direction+1) % 4;
					}
					else if ((new_direction+2) % 4 == next_path_direction) { //ska åka bakåt
						current_shortest_path[current_shortest_path_length++] = 3;
						current_shortest_path[current_shortest_path_length++] = 0;
						new_direction = (new_direction+2) % 4;
					}
					else { //ska åka vänster
						current_shortest_path[current_shortest_path_length++] = 2;
						current_shortest_path[current_shortest_path_length++] = 0;
						new_direction = (new_direction+3) % 4;
					}
				}
				else {
					if(new_direction == next_path_direction) { //ska åka rakt
						planned_control_modes[planned_control_modes_length++] = 0;
					}
					else if ((new_direction+1) % 4 == next_path_direction) { //ska åka höger
						planned_control_modes[planned_control_modes_length++] = 1;
						planned_control_modes[planned_control_modes_length++] = 0;
						new_direction = (new_direction+1) % 4;
					}
					else if ((new_direction+2) % 4 == next_path_direction) { //ska åka bakåt
						planned_control_modes[planned_control_modes_length++] = 3;
						planned_control_modes[planned_control_modes_length++] = 0;
						new_direction = (new_direction+2) % 4;
					}
					else { //ska åka vänster
						planned_control_modes[planned_control_modes_length++] = 2;
						planned_control_modes[planned_control_modes_length++] = 0;
						new_direction = (new_direction+3) % 4;
					}
				}
				curr_module_index = previous_module_index;
			}
			break;
		}

		if (is_valid(normal_map_index,j+1,i) && is_valid(normal_map_index,j+2,i)) {
			map[j+1][i] |= (1<<search_visited_map_index);
			map[j+2][i] |= (1<<search_visited_map_index);
			struct Node next_node = {i,j+2,queue_first};
			queue[queue_last] = next_node;
			queue_last++;
		}
		if (is_valid(normal_map_index,j,i-1) && is_valid(normal_map_index,j,i-2)) {
			map[j][i-1] |= (1<<search_visited_map_index);
			map[j][i-2] |= (1<<search_visited_map_index);
			struct Node next_node = {i-2,j,queue_first};
			queue[queue_last] = next_node;
			queue_last++;
		}
		if (is_valid(normal_map_index,j,i+1) && is_valid(normal_map_index,j,i+2)) {
			map[j][i+1] |= (1<<search_visited_map_index);
			map[j][i+2] |= (1<<search_visited_map_index);
			struct Node next_node = {i+2,j,queue_first};
			queue[queue_last] = next_node;
			queue_last++;
		}
		if (is_valid(normal_map_index,j-1,i) && is_valid(normal_map_index,j-2,i)) {
			map[j-1][i] |= (1<<search_visited_map_index);
			map[j-2][i] |= (1<<search_visited_map_index);
			struct Node next_node = {i,j-2,queue_first};
			queue[queue_last] = next_node;
			queue_last++;
		}
		queue_first++;

	}
	return new_direction;
}

bool is_valid(uint8_t map_index, uint8_t row, uint8_t col) {
	return (row > 0) && (row < 50) && (col > 0) && (col < 100) && ((map[row][col] & (1<<map_index)) != 0) && ((map[row][col] & (1<<search_visited_map_index)) == 0);
}

bool visited_right() {
	bool visited_right = false;
	switch (direction) {
		case 0:
		if ((map[curr_pos_row][curr_pos_col-1] & (1<<visited_map_index)) != 0) {
			visited_right = true;
		}
		break;

		case 1:
		if ((map[curr_pos_row-1][curr_pos_col] & (1<<visited_map_index)) != 0) {
			visited_right = true;
		}
		break;

		case 2:
		if ((map[curr_pos_row][curr_pos_col+1] & (1<<visited_map_index)) != 0) {
			visited_right = true;
		}
		break;

		case 3:
		if ((map[curr_pos_row+1][curr_pos_col] & (1<<visited_map_index)) != 0) {
			visited_right = true;
		}
		break;
	}
	return visited_right;
}

bool visited_left() {
	bool visited_left = false;
	switch (direction) {
		case 0:
		if ((map[curr_pos_row][curr_pos_col+1] & (1<<visited_map_index)) != 0) {
			visited_left = true;
		}
		break;

		case 1:
		if ((map[curr_pos_row+1][curr_pos_col] & (1<<visited_map_index)) != 0) {
			visited_left = true;
		}
		break;

		case 2:
		if ((map[curr_pos_row][curr_pos_col-1] & (1<<visited_map_index)) != 0) {
			visited_left = true;
		}
		break;

		case 3:
		if ((map[curr_pos_row-1][curr_pos_col] & (1<<visited_map_index)) != 0) {
			visited_left = true;
		}
		break;
	}
	return visited_left;
}

bool visited_straight() {
	bool visited_straight = false;
	switch (direction) {
		case 0:
		if ((map[curr_pos_row+1][curr_pos_col] & (1<<visited_map_index)) != 0) {
			visited_straight = true;
		}
		break;

		case 1:
		if ((map[curr_pos_row][curr_pos_col-1] & (1<<visited_map_index)) != 0) {
			visited_straight = true;
		}
		break;

		case 2:
		if ((map[curr_pos_row-1][curr_pos_col] & (1<<visited_map_index)) != 0) {
			visited_straight = true;
		}
		break;

		case 3:
		if ((map[curr_pos_row][curr_pos_col+1] & (1<<visited_map_index)) != 0) {
			visited_straight = true;
		}
		break;
	}
	return visited_straight;
}

void follow_plan() {
	control_mode = planned_control_modes[0];
	planned_control_modes_length--;
	for (uint8_t i = 0; i < planned_control_modes_length; i++) {
		planned_control_modes[i] = planned_control_modes[i+1];
	}
	if (planned_control_modes_length == 0) {
		plan_exists = false;
	}
}



void straight_corridor() {
	//PD

	control_mode_done = false;

	cli();
	uint8_t local_dist_right_front = dist_right_front;
	uint8_t local_dist_right_back = dist_right_back;
	uint8_t local_dist_left_front = dist_left_front;
	uint8_t local_dist_left_back = dist_left_back;
	if (dist_forward_done) {
		local_dist_forward = dist_forward;
	}
	if (angle_done) {
		local_angle = angle;
	}
	if (distance_traveled_done) {
		local_distance_traveled = distance_traveled;
	}
	sei();

	if (first_run_control_mode) {
		first_run_control_mode = false;
		if (local_distance_traveled + module_distance < 65000) { //TODO
			goal_distance_traveled = local_distance_traveled + module_distance;
		}
		else {
			goal_distance_traveled = local_distance_traveled - 65000 + module_distance;
		}
		if (local_dist_right_back == 0 || local_dist_right_front == 0) {
			right_open_on_start = true;
		}
		if (local_dist_left_back == 0 || local_dist_left_front == 0) {
			left_open_on_start = true;
		}
		last_dist_right_front = 20 - sensor_width/2;
		last_dist_left_front = 20 - sensor_width/2;
		last_dist_right_back = 20 - sensor_width/2;
		last_dist_left_back = 20 - sensor_width/2;
	}

	//kolla om front == 0 och sätt då use sensor = 2
	if ((local_dist_right_front == 0 || local_dist_right_front > 20) && distance_since_back_open == 0 && !right_open_on_start) {
		use_sensor = 2;
	}
	else if ((local_dist_left_front == 0 || local_dist_left_front > 20) && distance_since_back_open == 0 && !left_open_on_start) {
		use_sensor = 2;
	}

	//kollar när bakre IR-sensorerna visar att det är öppet åt sidorna. Då börjar avstånd att mätas.
	if ((local_dist_right_back == 0 || local_dist_right_back > 20) && distance_since_back_open == 0 && !right_open_on_start) {
		distance_since_back_open = local_distance_traveled;
	}
	else if ((local_dist_left_back == 0 || local_dist_left_back > 20) && distance_since_back_open == 0 && !left_open_on_start) {
		distance_since_back_open = local_distance_traveled;
	}

	//kollar om det är en vägg precis framför och använder då lidar.
	if (local_dist_forward < 30 && use_sensor != 2 && local_distance_traveled + 20 >= goal_distance_traveled) {
		use_sensor = 1;
	}


	//Den av lidar och IR som blir klar först används om båda kan användas.
	if ((use_sensor == 1 || use_sensor == 2) && local_dist_forward < 40 && local_distance_traveled + 20 >= goal_distance_traveled && local_dist_forward <= lidar_stop_distance) {
		control_mode_done = true;
	}
	else if (use_sensor == 2 && local_distance_traveled + 20 >= goal_distance_traveled && local_distance_traveled > distance_since_back_open + 2 && distance_since_back_open != 0) {
		control_mode_done = true;
	}
	else if (use_sensor == 3 && local_distance_traveled >= goal_distance_traveled) {
		control_mode_done = true;
	}


	if (control_mode_done) {
		first_run_control_mode = true;
		send_control_mode_data = true;
		disable_left = false;
		disable_right = false;
		distance_since_back_open = 0;
		right_open_on_start = false;
		left_open_on_start = false;
		use_sensor = 3;
		//uppdatera nuvarande position
		if (curr_pos_row == 255) {
			curr_pos_col = 51;
			curr_pos_row = 1;
		}
		else {
			switch (direction)
			{
				case 0: //syd
				curr_pos_row += 2;
				break;
				case 1: //väst
				curr_pos_col -= 2;
				break;
				case 2: //norr
				curr_pos_row -= 2;
				break;
				case 3: //öst
				curr_pos_col += 2;
				break;
			}
		}
		return;
	}

	int16_t curr_error;
	int16_t output;
	int16_t gyro_error;
	uint8_t Kp = sc_Kp;
	uint8_t Ka = sc_Ka;

	//Skippar en IR-sensor om dess värde ändras för fort
	if (local_distance_traveled > last_distance_traveled + 1) {
		if (local_dist_left_front != 0) {
			last_dist_left_front = local_dist_left_front;
		}
		if (local_dist_right_back != 0) {
			last_dist_right_back = local_dist_right_back;
		}
		if (local_dist_right_front != 0) {
			last_dist_right_front = local_dist_right_front;
		}
		if (local_dist_left_back != 0) {
			last_dist_left_back = local_dist_left_back;
		}
		last_distance_traveled = local_distance_traveled;
	}
	if (local_dist_left_front > last_dist_left_front + 1 && last_dist_left_front != 0) {
		local_dist_left_front = 0;
	}
	if (local_dist_right_front > last_dist_right_front + 1 && last_dist_right_front != 0) {
		local_dist_right_front = 0;
	}
	if (local_dist_left_back > last_dist_left_back + 1 && last_dist_left_back != 0) {
		local_dist_left_back = 0;
	}
	if (local_dist_right_back > last_dist_right_back + 1 && last_dist_right_back != 0) {
		local_dist_right_back = 0;
	}

	//Om vi har roterat används IR-sensorerna inte på den sidan vi har roterat åt
	if (disable_left) {
		local_dist_left_back = 0;
		local_dist_left_front = 0;
	}
	if (disable_right) {
		local_dist_right_back = 0;
		local_dist_right_front = 0;
	}

	//Beräknar felet utifrån de IR-sensorer som kan användas
	if (local_dist_left_front != 0 && local_dist_right_front != 0) {
		curr_error = (local_dist_right_front - local_dist_left_front);
	}
	else if (local_dist_left_back != 0 && local_dist_right_back != 0) {
		curr_error = (local_dist_right_back - local_dist_left_back);
	}
	else if (local_dist_left_front != 0) {
		curr_error = (last_dist_left_front - local_dist_left_front);
		Kp = sc_Kp_one_wall;
		Ka = sc_Ka_one_wall;
	}
	else if (local_dist_right_front != 0) {
		curr_error = (local_dist_right_front - last_dist_right_front);
		Kp = sc_Kp_one_wall;
		Ka = sc_Ka_one_wall;
	}
	else if (local_dist_left_back != 0) {
		curr_error = (last_dist_left_back - local_dist_left_back);
		Kp = sc_Kp_one_wall;
		Ka = sc_Ka_one_wall;
	}
	else if (local_dist_right_back != 0) {
		curr_error = (local_dist_right_back - last_dist_left_back);
		Kp = sc_Kp_one_wall;
		Ka = sc_Ka_one_wall;
	}
	else {
		curr_error = 0;
	}

	//Beräknar vinkelfelet
	if (direction * 90 - local_angle < -180) {
		gyro_error = direction * 90 - local_angle + 360;
	}
	else {
		gyro_error = direction * 90 - local_angle;
	}

	//Kör roboten reglerat med PD
	output = Kp*curr_error + Ka*gyro_error;

	left_direction = 1;
	right_direction = 1;
	PORTD = (left_direction<<PORTD2) | (right_direction<<PORTD3); //Riktning: framåt

	if (output > 0) //sväng höger
	{
		if (output > sc_max_thrust){ //stänger av motorn ifall felet är väldigt stort.
			right_thrust = 0;
		}
		else {
			right_thrust = sc_max_thrust - output;
		}
		left_thrust = sc_max_thrust; //sätter alltid andra motorn till max
	}
	else{ //sväng vänster
		if (-output > sc_max_thrust){ //stänger av motorn ifall felet är väldigt stort.
			left_thrust = 0;
		}
		else {
			left_thrust = sc_max_thrust - (-output);
		}
		right_thrust = sc_max_thrust; //sätter alltid andra motorn till max
	}
	sc_last_error = curr_error;

	OCR1A = right_thrust;
	OCR1B = left_thrust;
}

void turn_right() {
	cli();
	if (angle_done){
		local_angle = angle;
	}
	sei();

	int16_t curr_error;

	int8_t direction_after_turn;
	if (direction == 3) {
		direction_after_turn = 0;
	}
	else {
		direction_after_turn = direction + 1;
	}

	// För att förhindra problem när vinkeln går från 359 till 0
	if (direction_after_turn * 90 - local_angle < -180) {
		curr_error = direction_after_turn * 90 - local_angle + 360;
	}
	else {
		curr_error = (direction_after_turn * 90 - local_angle);
	}

	turn(curr_error, direction_after_turn);
}

void turn_left() {
	cli();
	if (angle_done){
		local_angle = angle;
	}
	sei();

	int16_t curr_error;

	int8_t direction_after_turn;
	if (direction == 0) {
		direction_after_turn = 3;
	}
	else {
		direction_after_turn = direction - 1;
	}

	// För att förhindra problem när vinkeln går från 359 till 0
	if (direction_after_turn * 90 - local_angle < -180) {
		curr_error = direction_after_turn * 90 - local_angle + 360;
	}
	else if (direction_after_turn * 90 - local_angle > 180) {
		curr_error = direction_after_turn * 90 - local_angle - 360;
	}
	else {
		curr_error = (direction_after_turn * 90 - local_angle);
	}

	turn(curr_error, direction_after_turn);
}

void turn_around() {
	cli();
	if (angle_done){
		local_angle = angle;
	}
	sei();

	int16_t curr_error;

	int8_t direction_after_turn;
	direction_after_turn = (direction + 2) % 4;

	// För att förhindra problem när vinkeln går från 359 till 0
	if (direction_after_turn * 90 - local_angle < -180) {
		curr_error = direction_after_turn * 90 - local_angle + 360;
	}
	else if (direction_after_turn * 90 - local_angle > 180) {
		curr_error = direction_after_turn * 90 - local_angle - 360;
	}
	else {
		curr_error = (direction_after_turn * 90 - local_angle);
	}

	turn(curr_error, direction_after_turn);
}

bool turn_done = false;
bool can_use_right = false;
bool can_use_left = false;

void turn(int16_t curr_error, int8_t direction_after_turn) {
	//P

	control_mode_done = false;

	cli();
	uint8_t local_dist_right_front = dist_right_front;
	uint8_t local_dist_right_back = dist_right_back;
	uint8_t local_dist_left_front = dist_left_front;
	uint8_t local_dist_left_back = dist_left_back;
	sei();

	if (turn_done) { //Ställer in oss efter väggarna om gyrosvängen är klar
		if (control_mode_done_ctr > 750) {//under den tiden borde vi få ungefär IR värdena ungefär 4 ggr
			control_mode_done = true;
		}
		if (can_use_right) {
			if (local_dist_right_front == local_dist_right_back) {
				control_mode_done_ctr++;
				curr_error = 0;
			}
			else {
//				control_mode_done_ctr = 0;
				if (local_dist_right_front > local_dist_right_back) {
					curr_error = 1; //ok att bara ta ett för den sätter till min automatiskt
				}
				else {
					curr_error = -1;
				}
			}
		}
		else if (can_use_left) {
			if (local_dist_left_front == local_dist_left_back) {
				control_mode_done_ctr++;
				curr_error = 0;
			}
			else {
//				control_mode_done_ctr = 0;
				if (local_dist_left_front > local_dist_left_back) {
					curr_error = -1;
				}
				else {
					curr_error = 1;
				}
			}
		}
		else {
			control_mode_done = true;
		}
	}
	else if (curr_error < 4 && curr_error > -4) { //Kollar när vi är klara med gyrosvängen
		if(control_mode_done_ctr++ > 10000) {
			control_mode_done_ctr = 0;
			turn_done = true;
			if (local_dist_right_front != 0 && local_dist_right_front < 20 && local_dist_right_back != 0 && local_dist_right_back < 20) {
				can_use_right = true;
			}
			if (local_dist_left_front != 0 && local_dist_left_front < 20 && local_dist_left_back != 0 && local_dist_left_back < 20) {
				can_use_left = true;
			}
		}
	}
	else {
		control_mode_done_ctr = 0;
	}

	if (control_mode_done) {
		first_run_control_mode = true;
		turn_done = false;
		can_use_left = false;
		can_use_right = false;
		control_mode_done_ctr = 0;
		direction = direction_after_turn;
		t_last_error = 0;
		send_control_mode_data = true;
		if (direction - direction_after_turn == 1 || direction - direction_after_turn == -3) {
			disable_left = true;
		}
		else if (direction - direction_after_turn == -1 || direction - direction_after_turn == 3) {
			disable_right = true;
		}
	}

	int16_t output = t_Kp*curr_error;

	if (output > 0 && output < 250) {
		output = 250;
	}
	else if (output < 0 && output > -250) {
		output = -250;
	}

	if (output > 0) //rotera höger
	{
		right_direction = 0;
		left_direction = 1;
		PORTD = (left_direction<<PORTD2) | (right_direction<<PORTD3); //Riktning: höger
	}
	else{ //rotera vänster
		right_direction = 1;
		left_direction = 0;
		PORTD = (left_direction<<PORTD2) | (right_direction<<PORTD3); //Riktning: vänster
	}

	if (abs(output) > t_max_thrust){
		right_thrust = t_max_thrust;
		left_thrust = t_max_thrust;
	}
	else{
		right_thrust = abs(output);
		left_thrust = abs(output);
	}

	t_last_error = curr_error;
	OCR1A = right_thrust;
	OCR1B = left_thrust;
}

void follow_tape() {
	//P

	control_mode_done = false;

	cli();
	uint8_t local_tape = tape;
	sei();
	int8_t tape_error = 0;
	switch (local_tape) {
		case 4:
		tape_error = 4;
		break;
		case 45:
		tape_error = 3;
		break;
		case 5:
		tape_error = 2;
		break;
		case 56:
		tape_error = 1;
		break;
		case 6:
		tape_error = 0;
		break;
		case 67:
		tape_error = -1;
		break;
		case 7:
		tape_error = -2;
		break;
		case 78:
		tape_error = -3;
		break;
		case 8:
		tape_error = -4;
		break;


	}

	if (tape == 0x1F) {
		control_mode_done = true;
		first_run_control_mode = true;
		send_control_mode_data = true;
		curr_pos_col = 255;
		curr_pos_row = 255;
	}

	int16_t output;
	output = tape_Kp * tape_error;

	//Kör just nu utan deriverande del
	//int16_t tape_derivative;
	//output = tape_Kp*tape_error + tape_Kd*tape_derivative;

	left_direction = 1;
	right_direction = 1;

	PORTD = (left_direction<<PORTD2) | (right_direction<<PORTD3); //Riktning: framåt

	if (output > 0) //sväng höger
	{
		if (output > tape_max_thrust){ //stänger av motorn ifall felet är väldigt stort.
			right_thrust = 0;
		}
		else {
			right_thrust = tape_max_thrust - output;
		}
		left_thrust = tape_max_thrust; //sätter alltid andra motorn till max
	}
	else{ //sväng vänster
		if (-output > tape_max_thrust){ //stänger av motorn ifall felet är väldigt stort.
			left_thrust = 0;
		}
		else {
			left_thrust = tape_max_thrust - (-output);
		}
		right_thrust = tape_max_thrust; //sätter alltid andra motorn till max
	}

	OCR1A = right_thrust;
	OCR1B = left_thrust;

}

void open() {

	cli();
	if (dist_forward_done) {
		local_dist_forward = dist_forward;
	}
	sei();

	if (delivery_done) {
		if(local_dist_forward % 40 <= 20 && local_dist_forward % 40 >= 8) {
			control_mode_done = true;
			right_thrust = 0;
			left_thrust = 0;
			right_direction = 1;
			left_direction = 1;
 			OCR1A = right_thrust;
 			OCR1B = left_thrust;
			delivery_done = false;
		}
		else {
			slow_corridor(0);
		}
	}
	else if (tape == 0x1F) {
		right_thrust = 0;
		left_thrust = 0;
		OCR1A = right_thrust;
		OCR1B = left_thrust;
		claw = 30;
		OCR2B = claw;
		_delay_ms(250);
		delivery_done = true;
	}
	else {
		slow_corridor(1);
	}
	PORTD = (left_direction<<PORTD2) | (right_direction<<PORTD3);
	OCR1A = right_thrust;
	OCR1B = left_thrust;
}

void slow_corridor(uint8_t wheel_direction) {
	cli();
	uint8_t local_dist_right_front = dist_right_front;
	uint8_t local_dist_right_back = dist_right_back;
	uint8_t local_dist_left_front = dist_left_front;
	uint8_t local_dist_left_back = dist_left_back;
	if (angle_done) {
		local_angle = angle;
	}
	sei();
	int16_t curr_error;
	int16_t output;
	int16_t gyro_error;

	if (disable_left) {
		local_dist_left_back = 0;
		local_dist_left_front = 0;
	}
	if (disable_right) {
		local_dist_right_back = 0;
		local_dist_right_front = 0;
	}

	if (local_dist_left_front != 0 && local_dist_right_front != 0) {
		curr_error = (local_dist_right_front - local_dist_left_front);
	}
	else if (local_dist_left_back != 0 && local_dist_right_back != 0) {
		curr_error = (local_dist_right_back - local_dist_left_back);
	}
	else {
		curr_error = 0;
	}

	if (direction * 90 - local_angle < -180) {
		gyro_error = direction * 90 - local_angle + 360;
	}
	else {
		gyro_error = direction * 90 - local_angle;
	}

	output = sc_Kp*curr_error + sc_Ka*gyro_error;

	left_direction = wheel_direction;
	right_direction = wheel_direction;

	PORTD = (left_direction<<PORTD2) | (right_direction<<PORTD3); //Riktning: framåt

	if (output > 0) //sväng höger
	{
		if (output > sc_max_thrust){ //stänger av motorn ifall felet är väldigt stort.
			right_thrust = 0;
		}
		else {
			right_thrust = sc_max_thrust - output;
		}
		left_thrust = sc_max_thrust; //sätter alltid andra motorn till max
	}
	else{ //sväng vänster
		if (-output > sc_max_thrust){ //stänger av motorn ifall felet är väldigt stort.
			left_thrust = 0;
		}
		else {
			left_thrust = sc_max_thrust - (-output);
		}
		right_thrust = sc_max_thrust; //sätter alltid andra motorn till max
	}
	sc_last_error = curr_error;


	right_thrust = sc_max_thrust;
	left_thrust = sc_max_thrust;
	OCR1A = right_thrust;
	OCR1B = left_thrust;
}

void close() {
	right_thrust = 0;
	left_thrust = 0;
	OCR1A = right_thrust;
	OCR1B = left_thrust;
	_delay_ms(250);
	claw = 21;
	OCR2B = claw;
	_delay_ms(250);
	control_mode_done = true;
}

void leave_maze() {
	right_thrust = max_thrust;
	left_thrust = max_thrust;
	right_direction = 1;
	left_direction = 1;
	PORTD = (left_direction<<PORTD2) | (right_direction<<PORTD3);
	OCR1A = right_thrust;
	OCR1B = left_thrust;
	cli();
	if (distance_traveled_done) {
		local_distance_traveled = distance_traveled;
	}
	sei();
	if (first_run_control_mode) {
		goal_distance_traveled = local_distance_traveled + 60;
		first_run_control_mode = false;
	}

	control_mode_done = false;

	if (local_distance_traveled >= goal_distance_traveled) {
		OCR1A = 0;
		OCR1B = 0;
		control_mode_done = true;
		first_run_control_mode = true;
		plan_exists = false;
		planned_control_modes_length = 0;
		current_shortest_path_length = 0;
		current_shortest_dist = UINT16_MAX;
		curr_pos_row = 255;
		curr_pos_col = 255;
		delivery_found = false;
		control_mode_index = 0;
		map_setup();
		phase = 0;
	}
}

void enter_maze() {
// 	right_thrust = sc_max_thrust;
// 	left_thrust = sc_max_thrust;
// 	right_direction = 1;
// 	left_direction = 1;
// 	PORTD = (left_direction<<PORTD2) | (right_direction<<PORTD3);
// 	OCR1A = right_thrust;
// 	OCR1B = left_thrust;

	cli();
	uint8_t local_dist_right_front = dist_right_front;
	uint8_t local_dist_right_back = dist_right_back;
	uint8_t local_dist_left_front = dist_left_front;
	uint8_t local_dist_left_back = dist_left_back;
	if (angle_done) {
		local_angle = angle;
	}
	if (distance_traveled_done) {
		local_distance_traveled = distance_traveled;
	}
	sei();

	if (first_run_control_mode) {
		goal_distance_traveled = local_distance_traveled + 25;
		first_run_control_mode = false;
	}

	control_mode_done = false;

	if (local_distance_traveled >= goal_distance_traveled) {
		first_run_control_mode = true;
		control_mode_done = true;
		send_control_mode_data = true;
		curr_pos_col = 51;
		curr_pos_row = 1;
		return;
	}

	int16_t curr_error;
	int16_t output;
	int16_t gyro_error;
	uint8_t Kp = sc_Kp;
	uint8_t Ka = sc_Ka;

	//Skippar en IR-sensor om dess värde ändras för fort
	if (local_distance_traveled > last_distance_traveled + 1) {
		if (local_dist_left_front != 0) {
			last_dist_left_front = local_dist_left_front;
		}
		if (local_dist_right_back != 0) {
			last_dist_right_back = local_dist_right_back;
		}
		if (local_dist_right_front != 0) {
			last_dist_right_front = local_dist_right_front;
		}
		if (local_dist_left_back != 0) {
			last_dist_left_back = local_dist_left_back;
		}
		last_distance_traveled = local_distance_traveled;
	}
	if (local_dist_left_front > last_dist_left_front + 1 && last_dist_left_front != 0) {
		local_dist_left_front = 0;
	}
	if (local_dist_right_front > last_dist_right_front + 1 && last_dist_right_front != 0) {
		local_dist_right_front = 0;
	}
	if (local_dist_left_back > last_dist_left_back + 1 && last_dist_left_back != 0) {
		local_dist_left_back = 0;
	}
	if (local_dist_right_back > last_dist_right_back + 1 && last_dist_right_back != 0) {
		local_dist_right_back = 0;
	}

	//Beräknar felet utifrån de IR-sensorer som kan användas
	if (local_dist_left_front != 0 && local_dist_right_front != 0) {
		curr_error = (local_dist_right_front - local_dist_left_front);
	}
// 	else if (local_dist_left_back != 0 && local_dist_right_back != 0) {
// 		curr_error = (local_dist_right_back - local_dist_left_back);
// 	}
// 	else if (local_dist_left_front != 0) {
// 		curr_error = (last_dist_left_front - local_dist_left_front);
// 		Kp = sc_Kp_one_wall;
// 		Ka = sc_Ka_one_wall;
// 	}
// 	else if (local_dist_right_front != 0) {
// 		curr_error = (local_dist_right_front - last_dist_right_front);
// 		Kp = sc_Kp_one_wall;
// 		Ka = sc_Ka_one_wall;
// 	}
// 	else if (local_dist_left_back != 0) {
// 		curr_error = (last_dist_left_back - local_dist_left_back);
// 		Kp = sc_Kp_one_wall;
// 		Ka = sc_Ka_one_wall;
// 	}
// 	else if (local_dist_right_back != 0) {
// 		curr_error = (local_dist_right_back - last_dist_left_back);
// 		Kp = sc_Kp_one_wall;
// 		Ka = sc_Ka_one_wall;
// 	}
	else {
		curr_error = 0;
	}

	//Beräknar vinkelfelet
	if (direction * 90 - local_angle < -180) {
		gyro_error = direction * 90 - local_angle + 360;
	}
	else {
		gyro_error = direction * 90 - local_angle;
	}

	//Kör roboten reglerat med PD
	output = Kp*curr_error + Ka*gyro_error;

	left_direction = 1;
	right_direction = 1;
	PORTD = (left_direction<<PORTD2) | (right_direction<<PORTD3); //Riktning: framåt

	if (output > 0) //sväng höger
	{
		if (output > sc_max_thrust){ //stänger av motorn ifall felet är väldigt stort.
			right_thrust = 0;
		}
		else {
			right_thrust = sc_max_thrust - output;
		}
		left_thrust = sc_max_thrust; //sätter alltid andra motorn till max
	}
	else{ //sväng vänster
		if (-output > sc_max_thrust){ //stänger av motorn ifall felet är väldigt stort.
			left_thrust = 0;
		}
		else {
			left_thrust = sc_max_thrust - (-output);
		}
		right_thrust = sc_max_thrust; //sätter alltid andra motorn till max
	}
	sc_last_error = curr_error;

	OCR1A = right_thrust;
	OCR1B = left_thrust;
}



ISR(TWI_vect) {
	uint8_t status_code;
	uint8_t newTWCR;
	status_code = TWSR & 0xf8;
	newTWCR = (0<<TWSTA) | (0<<TWWC) | (1<<TWEN) | (1<<TWIE);
	switch (status_code) {
		case 0x60: //Tog emot adress, skickar ACK
		newTWCR |= (0<<TWSTO)|(1<<TWINT)|(1<<TWEA);
		break;

		case 0x70: //Tog emot general call, skickar ACK
		newTWCR |= (0<<TWSTO)|(1<<TWINT)|(1<<TWEA);
		break;

		case 0x80: //Tog emot data efter adress call
		newTWCR |= (0<<TWSTO)|(1<<TWINT)|(1<<TWEA);
		recieve_data();
		break;

		case 0x90: //Tog emot data efter general call
		newTWCR |= (0<<TWSTO)|(1<<TWINT)|(1<<TWEA);
		recieve_data();
		break;

		case 0xA0: //Stopp
		newTWCR |= (0<<TWSTA) | (0<<TWSTO)|(1<<TWINT)|(1<<TWEA);
		i2c_counter = 0;
		id_i2c = 0;
		break;
	}
	TWCR = newTWCR;
}

void assign_recieved_data_i2c(uint8_t data) {
	switch (id_i2c) {
		case 1:
		if (i2c_counter == 2) {
			tape = data;
			if (tape != 0) {
				tape_ready = true;
			}
		}
		break;

		case 2:
		if(i2c_counter == 2){
			dist_right_front = data;
		}
		break;

		case 3:
		if(i2c_counter == 2){
			dist_left_front = data;
		}
		break;

		case 4:
		if(i2c_counter == 2){
			dist_right_back = data;
		}
		break;

		case 5:
		if(i2c_counter == 2){
			dist_left_back = data;
		}
		break;

		case 6:
		if(i2c_counter == 1){

			dist_forward = (data<<8);
			dist_forward_done = false;
		}
		else{
			dist_forward |= data;
			if (dist_forward != 65533) {
				dist_forward_done = true;
			}
		}
		break;

		case 7:
		if(i2c_counter == 1){
			angle = (data<<8);
			angle_done = false;
		}
		else{
			angle |= data;
			angle_done = true;
		}
		break;

		case 8:
		if (i2c_counter == 1) {
			distance_traveled = (data<<8);
			distance_traveled_done = false;
		}
		else {
			distance_traveled |= data;
			distance_traveled_done = true;
		}
		break;

		case 9:
		if(i2c_counter == 2){
			if(buttons == 0 && data == 1) {
				send_new_phase_data = true;
				phase = 0; //Om byter från manuellt till autonomt: stå still
			}
			else if (phase == 0 && data == 2) {
				clock_setdown();
				phase = 1;	//om startar tävling
			}
			buttons = data;
		}
		break;
	}
}

void recieve_data() {
	uint8_t data;
	data = TWDR;
	if (i2c_counter == 0) {
		id_i2c = data;
	}
	else {
		assign_recieved_data_i2c(data);
	}
	i2c_counter ++;
}



ISR(SPI_STC_vect) {
	spi_connected = true;
	uint8_t data = SPDR;
	if (data_is_id) {
		id_spi = data;
		spi_recieve_counter = 0;
		if (gotten_stop) {
			data_is_id = true;
		}
		else {
			data_is_id = false;
		}

		if (id_spi == 255) //start
		{
			gotten_stop = false;
			data_is_id = true;
			should_recieve_data = true;
			spi_recieve_counter = 0;
			should_send_data = true;
			spi_send_counter = 0;
			package_send_counter = 0;
			package_starting_index = 0;
			package_done = false;
			should_send_data_size = false;
		}
		else if (id_spi == 254) { //stop
			gotten_stop = true;
			data_is_id = true;
			should_recieve_data = false;
		}
		else if (id_spi == 253) { //send_data_size
			data_is_id = false;
			should_send_data_size = true;
			should_recieve_data = false;
			should_send_data = false;
			spi_send_data_size_counter = 0;
			spi_send_counter = 0;
			package_send_counter = 0;
		}
	}
	else if (should_recieve_data){
		assign_recieved_data_spi(data); //sätter rätt data utrifån id. Om hela datan är satt så ska data_is_id sättas till true.
		spi_recieve_counter++;
	}

	if (should_send_data) {
		spi_send_data();
		spi_send_counter++;
	}
	else if (should_send_data_size) {
		spi_send_data_size();
		if (spi_send_data_size_counter == 1) {
			data_is_id = true;
		}
		spi_send_data_size_counter++;
	}

	time_since_last_recieved_command = 0;
}

void assign_recieved_data_spi(uint8_t data) {
	switch (id_spi) {
		case 10:
		sc_Kp = data;
		data_is_id = true;
		break;

		case 11:
		sc_Ka = data;
		data_is_id = true;
		break;

		case 12:
		t_Kp = data;
		data_is_id = true;
		break;

		case 13:
		if (spi_recieve_counter == 1) {
			t_Kd = (data<<8);
		}
		else {
			t_Kd |= data;
			data_is_id = true;
		}
		break;

		case 14:
		tape_Kp = data;
		data_is_id = true;
		break;

		case 15:
		if (spi_recieve_counter == 1) {
			//tape_Kd |= (data<<8);
		}
		else {
			//tape_Kd |= data;
			data_is_id = true;
		}
		break;

		case 16:
		command = data;
		data_is_id = true;
		break;

	}
}

void spi_send_data_size() {

	if (spi_send_data_size_counter == 0) {
		if (buttons == 0) { //Manuellt läge
			data_size = sensor_data_size + stop_data_size;
			should_send_sensor_data = true;
		}
		else {
			switch (phase) {
				case 0: //inget
				data_size = buttons_data_size + stop_data_size;
				should_send_buttons = true;
				if (send_new_phase_data) {
					data_size += parameters_data_size;
					should_send_parameters = true;
				}
				break;

				case 1: //start
				data_size = sensor_data_size + thrust_data_size + stop_data_size;
				should_send_sensor_data = true;
				should_send_thrust = true;
				if (send_control_mode_data) {
					data_size += direction_data_size;
					should_send_direction = true;
				}
				break;

				case 2: //mapping
				data_size = sensor_data_size + thrust_data_size + stop_data_size;
				should_send_sensor_data = true;
				should_send_thrust = true;
				if (send_map_data) {
					if (updated_segments_length != 0) {
						should_send_updated_segments = true;
						data_size += updated_segments_length * 3 + 2;
					}
				}
				if (send_control_mode_data) {
					data_size += robot_placement_data_size + direction_data_size + planned_control_modes_data_size;
					should_send_robot_placement = true;
					should_send_direction = true;
					should_send_planned_control_modes = true;
				}
				break;

				case 3: //stop mapping
				data_size = sensor_data_size + thrust_data_size + stop_data_size;
				should_send_sensor_data = true;
				should_send_thrust = true;
				if (send_map_data) {
					data_size += current_shortest_path_length + 3;
					if (updated_segments_length != 0) {
						should_send_updated_segments = true;
						data_size += updated_segments_length * 3 + 2;
					}
					should_send_shortest_path = true;
				}
				if (send_new_phase_data) {
					data_size += delivery_placement_data_size;
					should_send_delivery_placement = true;
				}
				if (send_control_mode_data) {
					data_size += robot_placement_data_size + direction_data_size + planned_control_modes_data_size;
					should_send_robot_placement = true;
					should_send_direction = true;
					should_send_planned_control_modes = true;
				}
				break;

				case 4: //go out
				data_size = sensor_data_size + thrust_data_size + stop_data_size;
				should_send_sensor_data = true;
				should_send_thrust = true;
				if (send_new_phase_data) {
					data_size += current_shortest_path_length + 3 + delivery_placement_data_size + planned_control_modes_data_size;
					should_send_shortest_path = true;
					should_send_delivery_placement = true;
					should_send_empty_planned_control_modes = true;
				}
				if (send_control_mode_data) {
					data_size += robot_placement_data_size + direction_data_size;
					should_send_robot_placement = true;
					should_send_direction = true;
				}
				break;

				case 5: //get object
				data_size = sensor_data_size + thrust_data_size + stop_data_size;
				should_send_sensor_data = true;
				should_send_thrust = true;
				if (send_control_mode_data) {
					data_size += robot_placement_data_size + direction_data_size;
					should_send_robot_placement = true;
					should_send_direction = true;
				}
				break;

				case 6: //plan competition
				data_size = stop_data_size;
				break;

				case 7: //follow plan
				data_size = sensor_data_size + thrust_data_size + stop_data_size;
				should_send_sensor_data = true;
				should_send_thrust = true;
				if (send_control_mode_data) {
					data_size += robot_placement_data_size + direction_data_size;
					should_send_robot_placement = true;
					should_send_direction = true;
				}
				break;
			}
		}

		send_map_data = false;
		send_new_phase_data = false;
		send_control_mode_data = false;

		uint8_t send_data = ((data_size & 0xff00)>>8);
		SPDR = send_data;
	}
	else {
		SPDR = data_size;
		should_send_data_size = false;
	}
}

void spi_send_data() {
	if(package_done) {
		package_starting_index = spi_send_counter;
		package_send_counter ++;
		package_done = false;
	}

	switch (package_send_counter) {
		case 0: //buttons
		if (should_send_buttons) {
			send_buttons();
			break;
		}
		else {
			package_send_counter++;
		}

		case 1: //parameters
		if (should_send_parameters) {
			send_parameters();
			break;
		}
		else {
			package_send_counter++;
		}

		case 2: //sensor data
		if (should_send_sensor_data) {
			send_sensor_data();
			break;
		}
		else {
			package_send_counter++;
		}

		case 3: //thrust
		if (should_send_thrust) {
			send_thrust();
			break;
		}
		else {
			package_send_counter++;
		}

		case 4: //planned control modes
		if (should_send_planned_control_modes) {
			send_planned_control_modes();
			break;
		}
		else {
			package_send_counter++;
		}

		case 5: //empty planned control modes
		if (should_send_empty_planned_control_modes) {
			send_empty_planned_control_modes();
			break;
		}
		else {
			package_send_counter++;
		}

		case 6: //updated segments
		if (should_send_updated_segments) {
			send_updated_segments();
			break;
		}
		else {
			package_send_counter++;
		}

		case 7: //shortest path
		if (should_send_shortest_path) {
			send_shortest_path();
			break;
		}
		else {
			package_send_counter++;
		}

		case 8: //delivery placement
		if (should_send_delivery_placement) {
			send_delivery_placement();
			break;
		}
		else {
			package_send_counter++;
		}

		case 9: //robot placement
		if (should_send_robot_placement) {
			send_robot_placement();
			break;
		}
		else {
			package_send_counter++;
		}

		case 10: //direction
		if (should_send_direction) {
			send_direction();
			break;
		}
		else {
			package_send_counter++;
		}

		case 11: //stop
		should_send_data = false;
		SPDR = 254;
		break;

	}
}

void send_buttons() {
	if (spi_send_counter - package_starting_index == 0) {
		SPDR = buttons_id;
	}
	else if (spi_send_counter - package_starting_index == 1) {
		SPDR = buttons;
		package_done = true;
		should_send_buttons = false;
	}
}

void send_parameters() {
	if (spi_send_counter - package_starting_index == 0) {
		SPDR = sc_Kp_id;
	}
	else if (spi_send_counter - package_starting_index == 1) {
		SPDR = sc_Kp;
	}
	else if (spi_send_counter - package_starting_index == 2) {
		SPDR = sc_Ka_id;
	}
	else if (spi_send_counter - package_starting_index == 3) {
		SPDR = sc_Ka;
	}
	else if (spi_send_counter - package_starting_index == 4) {
		SPDR = t_Kp_id;
	}
	else if (spi_send_counter - package_starting_index == 5) {
		SPDR = t_Kp;
	}
	else if (spi_send_counter - package_starting_index == 6) {
		SPDR = t_Kd_id;
	}
	else if (spi_send_counter - package_starting_index == 7) {
		SPDR = ((t_Kd & 0xff00)>>8); //MSB;
	}
	else if (spi_send_counter - package_starting_index == 8) {
		SPDR = t_Kd; //LSB
	}
	else if (spi_send_counter - package_starting_index == 9) {
		SPDR = tape_Kp_id;
	}
	else if (spi_send_counter - package_starting_index == 10) {
		SPDR = tape_Kp;
		package_done = true;
		should_send_parameters = false;
	}
}

void send_sensor_data() {

	if (spi_send_counter - package_starting_index == 0) {
		SPDR = tape_id;
	}
	else if (spi_send_counter - package_starting_index == 1) {
		SPDR = tape;
	}
	else if (spi_send_counter - package_starting_index == 2) {
		SPDR = dist_right_front_id;
	}
	else if (spi_send_counter - package_starting_index == 3) {
		SPDR = dist_right_front;
	}
	else if (spi_send_counter - package_starting_index == 4) {
		SPDR = dist_left_front_id;
	}
	else if (spi_send_counter - package_starting_index == 5) {
		SPDR = dist_left_front;
	}
	else if (spi_send_counter - package_starting_index == 6) {
		SPDR = dist_right_back_id;
	}
	else if (spi_send_counter - package_starting_index == 7) {
		SPDR = dist_right_back;
	}
	else if (spi_send_counter - package_starting_index == 8) {
		SPDR = dist_left_back_id;
	}
	else if (spi_send_counter - package_starting_index == 9) {
		SPDR = dist_left_back;
	}
	else if (spi_send_counter - package_starting_index == 10) {
		SPDR = dist_forward_id;
	}
	else if (spi_send_counter - package_starting_index == 11) {
		if (dist_forward_done) {
			sending_dist_forward = dist_forward;
		}
		SPDR = ((sending_dist_forward & 0xff00)>>8); //MSB;
	}
	else if (spi_send_counter - package_starting_index == 12) {
		SPDR = dist_forward; //LSB
	}
	else if (spi_send_counter - package_starting_index == 13) {
		SPDR = angle_id;
	}
	else if (spi_send_counter - package_starting_index == 14) {
		SPDR = ((angle & 0xff00)>>8); //MSB;
	}
	else if (spi_send_counter - package_starting_index == 15) {
		SPDR = angle; //LSB
	}
	else if (spi_send_counter - package_starting_index == 16) {
		SPDR = buttons_id;
	}
	else if (spi_send_counter - package_starting_index == 17) {
		SPDR = buttons;
	}
	else if (spi_send_counter - package_starting_index == 18) {
		SPDR = distance_traveled_id;
	}
	else if (spi_send_counter - package_starting_index == 19) {
		SPDR = ((distance_traveled & 0xff00)>>8); //MSB;
	}
	else if (spi_send_counter - package_starting_index == 20) {
		SPDR = distance_traveled; //LSB
		package_done = true;
		should_send_sensor_data = false;
	}
}

void send_thrust() {
	if (spi_send_counter - package_starting_index == 0) {
		SPDR = right_thrust_id;
	}
	else if (spi_send_counter - package_starting_index == 1) {
		int8_t right_thrust_procentage;
		right_thrust_procentage = right_thrust/max_thrust;
		if (right_direction == 0) {
			right_thrust_procentage = -right_thrust_procentage;
		}
		SPDR = right_thrust_procentage;
	}
	else if (spi_send_counter - package_starting_index == 2) {
		SPDR = left_thrust_id;
	}
	else if (spi_send_counter - package_starting_index == 3) {
		int8_t left_thrust_procentage;
		left_thrust_procentage = left_thrust/max_thrust;
		if (left_direction == 0) {
			left_thrust_procentage = -left_thrust_procentage;
		}
		SPDR = left_thrust_procentage;

		package_done = true;
		should_send_thrust = false;
	}
}

void send_direction() {
	if (spi_send_counter - package_starting_index == 0) {
		SPDR = direction_id;
	}
	else if (spi_send_counter - package_starting_index == 1){
		SPDR = direction;
		package_done = true;
		should_send_direction = false;
	}
}

void send_robot_placement() {
	if (spi_send_counter - package_starting_index == 0) {
		SPDR = curr_pos_row_id;
	}
	else if (spi_send_counter - package_starting_index == 1) {
		SPDR = curr_pos_row;
	}
	else if (spi_send_counter - package_starting_index == 2) {
		SPDR = curr_pos_col_id;
	}
	else if (spi_send_counter - package_starting_index == 3) {
		SPDR = curr_pos_col;
		package_done = true;
		should_send_robot_placement = false;
	}
}

void send_delivery_placement() {
	if (spi_send_counter - package_starting_index == 0) {
		SPDR = delivery_row_id;
	}
	else if (spi_send_counter - package_starting_index == 1) {
		SPDR = delivery_row;
	}
	else if (spi_send_counter - package_starting_index == 2) {
		SPDR = delivery_col_id;
	}
	else if (spi_send_counter - package_starting_index == 3) {
		SPDR = delivery_col;
		package_done = true;
		should_send_delivery_placement = false;
	}
}

void send_planned_control_modes() {
	if (spi_send_counter - package_starting_index == 0) {
		SPDR = planned_control_modes_id;
	}
	else if (spi_send_counter - package_starting_index == 1) {
		SPDR = control_mode;
	}
	else {
		if (spi_send_counter - package_starting_index - 1 > planned_control_modes_length) {
			SPDR = 0xff;
		}
		else {
			SPDR = planned_control_modes[spi_send_counter - package_starting_index - 2];
		}
		if (package_starting_index + 10 == spi_send_counter) {
			package_done = true;
			should_send_planned_control_modes = false;
		}
	}
}

void send_empty_planned_control_modes() {
	if (spi_send_counter - package_starting_index == 0) {
		SPDR = planned_control_modes_id;
	}
	else {
		SPDR = 0xff;
		if (package_starting_index + 10 == spi_send_counter) {
			package_done = true;
			should_send_empty_planned_control_modes = false;
		}
	}
}

void send_updated_segments() {

	if (spi_send_counter - package_starting_index == 0) {
		SPDR = updated_segments_id;
	}
	else if (spi_send_counter - package_starting_index == 1) {
		SPDR = updated_segments_length;
	}
	else {
		if (send_updated_segments_cnt == 0){
			SPDR = updated_segments[0][updated_segments_length - 1];
			send_updated_segments_cnt++;
		}
		else if (send_updated_segments_cnt == 1) {
			SPDR = updated_segments[1][updated_segments_length - 1];
			send_updated_segments_cnt++;
		}
		else if (send_updated_segments_cnt == 2) {
			SPDR = map[updated_segments[0][updated_segments_length - 1]][updated_segments[1][updated_segments_length - 1]];
			send_updated_segments_cnt = 0;
			updated_segments_length--;
			if (updated_segments_length == 0) {
				package_done = true;
				should_send_updated_segments = false;
			}
		}
	}
}

void send_shortest_path () {

	if (spi_send_counter - package_starting_index == 0) {
		SPDR = current_shortest_path_id;
	}
	else if (spi_send_counter - package_starting_index == 1) {
		SPDR = ((current_shortest_path_length & 0xff00)>>8);
	}
	else if (spi_send_counter - package_starting_index == 2) {
		SPDR = current_shortest_path_length;
		if (current_shortest_path_length == 0) {
			package_done = true;
			should_send_shortest_path = false;
		}
	}
	else {
		SPDR = current_shortest_path[spi_send_counter - package_starting_index - 3];
		if (spi_send_counter - package_starting_index - 2 == current_shortest_path_length) { //ser till att inte skicka mer än som finns i vektorn
			package_done = true;
			should_send_shortest_path = false;
		}
	}
}
