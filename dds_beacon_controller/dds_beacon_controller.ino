/* K3NG Beacon Controller

Copyright 2012-2020 Anthony Good, K3NG
All trademarks referred to in source code and documentation are copyright their respective owners.

    /*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    

 Are you a radio artisan ?

*/

#include <stdio.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <EEPROM.h> 
#include <dds.h>
#include <TimerOne.h>
// #include <TimerFive.h>


#define CODE_VERSION "2020.05.12.01"
#define eeprom_magic_number 2               // you can change this number to have the unit re-initialize EEPROM

// #define DEBUG_SEND_CHAR
// #define DEBUG_FORCE_RESET
// #define DEBUG_EEPROM_READ_SETTINGS
// #define DEBUG_EEPROM
// #define DEBUG_ASYNC_EEPROM_WRITE
// #define DEBUG_MEMORY_LOCATIONS


#define HELL_NOT_USING_SCHEDULER
// #define NO_HELLSCREIBER

// beacon frequencies
#define tx_1_frequency 28231000UL
#define tx_2_frequency 50040000UL
#define tx_3_frequency 0
#define tx_4_frequency 0
#define tx_5_frequency 0
#define tx_6_frequency 0

// DDS frequency calibration
#define tx_1_frequency_calibration 0.000052
#define tx_2_frequency_calibration 0.000052
#define tx_3_frequency_calibration 0
#define tx_4_frequency_calibration 0
#define tx_5_frequency_calibration 0
#define tx_6_frequency_calibration 0

#define number_of_transmitters 2



// pin settings
#define tx_1 13              // transmitter 1 keying line (high = key down/tx on)
#define tx_2 13
#define tx_3 0
#define tx_4 0
#define tx_5 0
#define tx_6 0
#define sidetone_line 4         // connect a speaker for sidetone
#define ptt_tx_1 11             // PTT ("push to talk") lines
#define ptt_tx_2 12             //   Can be used for keying fox transmitter, T/R switch, or keying slow boatanchors
#define ptt_tx_3 0              //   These are optional - set to 0 if unused
#define ptt_tx_4 0
#define ptt_tx_5 0
#define ptt_tx_6 0

#define dds_load_pin 6
#define dds_data_pin 8
#define dds_clock_pin 7

#define analog_vin1 A0
#define analog_vin2 A1
#define analog_vin3 A2

// "factory defaults" - only written upon EEPROM initialization 
// change eeprom_magic_number above to invoke EEPROM initialization
#define default_unit_state BEACON        
#define default_speed_wpm 22
#define default_sidetone_freq 600
#define default_tx 1
#define default_number_of_memories 10

// various system settings
#define char_send_buffer_size 50

#define default_length_letterspace 3
#define default_length_wordspace 7
#define initial_ptt_lead_time 10          // PTT lead time in mS
#define initial_ptt_tail_time 200         // PTT tail time in mS
#define default_ptt_hang_time_wordspace_units 0.0 
#define default_serial_baud_rate 115200
#define hell_pixel_microseconds 2700 //4025
#define analog_vin1_calibration_multiplier 0.0199 //50.19 50.76
#define analog_vin1_calibration_offset 0.0
#define eeprom_write_time_ms 30000
#define serial_program_memory_buffer_size 100
#define interrupt_handler_microseconds 500

#if !defined(NO_HELLSCREIBER)
  #define element_send_buffer_size 2048
#else
  #define element_send_buffer_size 20
#endif  

// DDS chip settings
#define dds_chip_clock_mhz 180000000UL
#define dds_chip DDS9851
#define dds_chip_clock_multiplier 1


enum unit_state_type {OFFLINE, BEACON, TEST_ROUND_ROBIN_CW_30_S};
enum key_scheduler_type {IDLE, PTT_LEAD_TIME_WAIT, KEY_DOWN, KEY_UP};
enum sidetone_mode_type {SIDETONE_OFF, SIDETONE_ON};
enum char_send_mode_type {SEND_MODE_CW, SEND_MODE_HELL};
enum sending_tye {AUTOMATIC_SENDING, MANUAL_SENDING};
enum element_buffer_type {HALF_UNIT_KEY_UP, ONE_UNIT_KEY_DOWN_1_UNIT_KEY_UP, THREE_UNITS_KEY_DOWN_1_UNIT_KEY_UP,
  ONE_UNIT_KEYDOWN_3_UNITS_KEY_UP, THREE_UNIT_KEYDOWN_3_UNITS_KEY_UP,
  ONE_UNIT_KEYDOWN_7_UNITS_KEY_UP, THREE_UNIT_KEYDOWN_7_UNITS_KEY_UP,
  SEVEN_UNITS_KEY_UP, KEY_UP_LETTERSPACE_MINUS_1, KEY_UP_WORDSPACE_MINUS_4,
  KEY_UP_WORDSPACE, HELL_PIXEL_0, HELL_PIXEL_1};

#define SERIAL_SEND_BUFFER_WPM_CHANGE 200
#define SERIAL_SEND_BUFFER_PTT_ON 201
#define SERIAL_SEND_BUFFER_PTT_OFF 202
#define SERIAL_SEND_BUFFER_TIMED_KEY_DOWN 203
#define SERIAL_SEND_BUFFER_TIMED_WAIT 204
#define SERIAL_SEND_BUFFER_NULL 205
#define SERIAL_SEND_BUFFER_PROSIGN 206
#define SERIAL_SEND_BUFFER_HOLD_SEND 207
#define SERIAL_SEND_BUFFER_HOLD_SEND_RELEASE 208
#define SERIAL_SEND_BUFFER_MEMORY_NUMBER 210

#define SERIAL_SEND_BUFFER_NORMAL 0
#define SERIAL_SEND_BUFFER_TIMED_COMMAND 1
#define SERIAL_SEND_BUFFER_HOLD 2

#define NORMAL 0
#define OMIT_LETTERSPACE 1


dds ddschip(dds_chip, dds_data_pin, dds_load_pin, dds_clock_pin, dds_chip_clock_mhz);



#define memory_area_start 115

struct config_t {  // ? bytes total
  
  uint8_t unit_state;
  uint8_t sidetone_mode;
  uint8_t current_tx;
  uint8_t char_send_mode;
  uint8_t number_of_txs;

  unsigned int wpm;
  unsigned int number_of_memories;
  unsigned int hz_sidetone;

  unsigned long tx_freq[6];
  unsigned long tx_freq_calibration[6];

} configuration;


byte key_tx = 1;
unsigned int ptt_tail_time = initial_ptt_tail_time;
unsigned int ptt_lead_time = initial_ptt_lead_time;
byte key_scheduler_state = IDLE;
unsigned long next_key_scheduler_transition_time = 0;
unsigned int key_scheduler_keyup_ms;
unsigned int key_scheduler_keydown_ms;
unsigned long ptt_time;
byte ptt_line_activated = 0;
byte key_state = 0;
byte length_letterspace = default_length_letterspace;
byte length_wordspace = default_length_wordspace;
byte manual_ptt_invoke = 0;
byte last_sending_type = MANUAL_SENDING;
float ptt_hang_time_wordspace_units = default_ptt_hang_time_wordspace_units;
byte pause_sending_buffer = 0;
byte char_send_buffer_array[char_send_buffer_size];
byte char_send_buffer_bytes = 0;
byte char_send_buffer_status = SERIAL_SEND_BUFFER_NORMAL;
byte element_send_buffer_array[element_send_buffer_size];
unsigned int element_send_buffer_bytes = 0;
unsigned long beacon_cycle_count = 0;
unsigned int millis_rollover = 0;

byte sequence = 1;
unsigned long current_tx_on_frequency = 0;
unsigned long last_transition_time = 0;
byte did_first_run = 0;

byte async_eeprom_write = 0;
byte config_dirty = 0;
unsigned long last_config_write = 0;
uint16_t memory_area_end = 0;

byte incoming_serial_byte;
unsigned long serial_baud_rate;
byte serial_backslash_command;

#if !defined(NO_HELLSCREIBER)
PROGMEM const char hell_font1[] = {B00111111, B11100000, B00011001, B11000000, B01100011, B00000001, B10011100, B00111111, B11100000,    // A
                                    B00110000, B00110000, B11111111, B11000011, B00110011, B00001100, B11001100, B00011100, B11100000,    // B
                                    B00111111, B11110000, B11000000, B11000011, B00000011, B00001100, B00001100, B00110000, B00110000,    // C
                                    B00110000, B00110000, B11111111, B11000011, B00000011, B00001100, B00001100, B00011111, B11100000,    // D
                                    B00111111, B11110000, B11001100, B11000011, B00110011, B00001100, B00001100, B00110000, B00110000,    // E
                                    B00111111, B11110000, B00001100, B11000000, B00110011, B00000000, B00001100, B00000000, B00110000,    // F
                                    B00111111, B11110000, B11000000, B11000011, B00000011, B00001100, B11001100, B00111111, B00110000,    // G
                                    B00111111, B11110000, B00001100, B00000000, B00110000, B00000000, B11000000, B00111111, B11110000,    // H
                                    B00000000, B00000000, B00000000, B00000011, B11111111, B00000000, B00000000, B00000000, B00000000,    // I
                                    B00111100, B00000000, B11000000, B00000011, B00000000, B00001100, B00000000, B00111111, B11110000,    // J
                                    B00111111, B11110000, B00001100, B00000000, B01110000, B00000011, B00110000, B00111000, B11100000,    // K
                                    B00111111, B11110000, B11000000, B00000011, B00000000, B00001100, B00000000, B00110000, B00000000,    // L
                                    B00111111, B11110000, B00000001, B10000000, B00001100, B00000000, B00011000, B00111111, B11110000,
                                    B00111111, B11110000, B00000011, B10000000, B00111000, B00000011, B10000000, B00111111, B11110000,
                                    B00111111, B11110000, B11000000, B11000011, B00000011, B00001100, B00001100, B00111111, B11110000,
                                    B00110000, B00110000, B11111111, B11000011, B00110011, B00000000, B11001100, B00000011, B11110000,
                                    B00111111, B11110000, B11000000, B11000011, B11000011, B00001111, B11111100, B11110000, B00000000,
                                    B00111111, B11110000, B00001100, B11000000, B00110011, B00000011, B11001100, B00111001, B11100000,
                                    B00110001, B11100000, B11001100, B11000011, B00110011, B00001100, B11001100, B00011110, B00110000,
                                    B00000000, B00110000, B00000000, B11000011, B11111111, B00000000, B00001100, B00000000, B00110000,
                                    B00111111, B11110000, B11000000, B00000011, B00000000, B00001100, B00000000, B00111111, B11110000,
                                    B00111111, B11110000, B01110000, B00000000, B01110000, B00000000, B01110000, B00000000, B01110000,
                                    B00011111, B11110000, B11000000, B00000001, B11110000, B00001100, B00000000, B00011111, B11110000,
                                    B00111000, B01110000, B00110011, B00000000, B01111000, B00000011, B00110000, B00111000, B01110000,
                                    B00000000, B01110000, B00000111, B00000011, B11110000, B00000000, B01110000, B00000000, B01110000,
                                    B00111000, B00110000, B11111000, B11000011, B00110011, B00001100, B01111100, B00110000, B01110000};   // Z

PROGMEM const char hell_font2[] = {B00011111, B11100000, B11000000, B11000011, B00000011, B00001100, B00001100, B00011111, B11100000,   // 0
                                    B00000000, B00000000, B00000011, B00000000, B00000110, B00001111, B11111100, B00000000, B00000000,
                                    B00111000, B01100000, B11110000, B11000011, B00110011, B00001100, B01111000, B00110000, B00000000,
                                    B11000000, B00000011, B00000000, B11000110, B00110011, B00001100, B11111100, B00011110, B00000000,
                                    B00000111, B11111000, B00011000, B00000000, B01100000, B00001111, B11111100, B00000110, B00000000,
                                    B00110000, B00000000, B11000000, B00000011, B00011111, B10000110, B01100110, B00001111, B00011000,
                                    B00011111, B11110000, B11001100, B01100011, B00011000, B11001100, B01100000, B00011111, B00000000,
                                    B01110000, B00110000, B01110000, B11000000, B01110011, B00000000, B01111100, B00000000, B01110000,
                                    B00111100, B11110001, B10011110, B01100110, B00110001, B10011001, B11100110, B00111100, B11110000,
                                    B00000011, B11100011, B00011000, B11000110, B01100011, B00001100, B00001100, B00011111, B11100000};  // 9

 PROGMEM const char hell_font3[]  = {B00000011, B00000000, B00001100, B00000001, B11111110, B00000000, B11000000, B00000011, B00000000,
                                     B00000011, B00000000, B00001100, B00000000, B00110000, B00000000, B11000000, B00000011, B00000000,
                                     B00000000, B00110000, B00000000, B11001110, B01110011, B00000000, B01111100, B00000000, B00000000,
                                     B01110000, B00000000, B01110000, B00000000, B01110000, B00000000, B01110000, B00000000, B01110000,
                                     B00111000, B00000000, B11100000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
                                     B00001100, B00000001, B11110000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
                                     B00000000, B00111000, B00000011, B10000000, B00000000, B00000000, B00000000, B00000000, B00000000,
                                     B00001100, B11000000, B00110011, B00000000, B11001100, B00000011, B00110000, B00001100, B11000000,
                                     B01110000, B00111000, B01110011, B10000000, B01111000, B00000000, B00000000, B00000000, B00000000,
                                     B00000000, B00000000, B00000000, B00000000, B01111000, B00000111, B00111000, B01110000, B00111000,
                                     B00000000, B00000000, B01110011, B10000001, B11001110, B00000000, B00000000, B00000000, B00000000,
                                     0, 0, 0, 0, 0, 0, 0, 0, 0};
#endif //NO_HELLSCREIBER



//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------


void setup() {
  
  initialize_dds();
  initialize_pins();
  initialize_serial();
  initialize_unit_state();
  check_eeprom_for_initialization();

}

//-------------------------------------------------------------------------------------------------------


void loop() {
  
  check_for_dirty_configuration();
  service_async_eeprom_write();
  millis_rollover_check();
  check_serial();
  
}

//-------------------------------------------------------------------------------------------------------
void interrupt_handler(){

  if (configuration.unit_state == BEACON) {
    service_key_scheduler(); 
    check_ptt_tail();
    service_element_send_buffer();
    service_char_send_buffer();
    service_sequencer();
  }

}
//-------------------------------------------------------------------------------------------------------

void initialize_unit_state(){

  
  
  memory_area_end = EEPROM.length() - 1;

  configuration.sidetone_mode = SIDETONE_ON;
  configuration.unit_state = default_unit_state;
  configuration.number_of_memories = default_number_of_memories;
  configuration.wpm = default_speed_wpm;
  configuration.current_tx = default_tx;
  configuration.hz_sidetone = default_sidetone_freq;
  configuration.char_send_mode = SEND_MODE_CW;

  switch_to_tx(configuration.current_tx);

  #if defined(TimerOne_h_)
    Timer1.initialize(interrupt_handler_microseconds);
    Timer1.attachInterrupt(interrupt_handler);
  #else
    Timer5.initialize(interrupt_handler_microseconds);
    Timer5.attachInterrupt(interrupt_handler);
  #endif  

  
  // configuration.ptt_buffer_hold_active = 0;
  // configuration.ptt_disabled = 0;
  // configuration.ptt_lead_time[0] = initial_ptt_lead_time_tx1;
  // configuration.ptt_tail_time[0] = initial_ptt_tail_time_tx1;
  // configuration.ptt_lead_time[1] = initial_ptt_lead_time_tx2;
  // configuration.ptt_tail_time[1] = initial_ptt_tail_time_tx2;
  // configuration.ptt_lead_time[2] = initial_ptt_lead_time_tx3;
  // configuration.ptt_tail_time[2] = initial_ptt_tail_time_tx3;
  // configuration.ptt_lead_time[3] = initial_ptt_lead_time_tx4;
  // configuration.ptt_tail_time[3] = initial_ptt_tail_time_tx4;
  // configuration.ptt_lead_time[4] = initial_ptt_lead_time_tx5;
  // configuration.ptt_tail_time[4] = initial_ptt_tail_time_tx5;
  // configuration.ptt_lead_time[5] = initial_ptt_lead_time_tx6;
  // configuration.ptt_tail_time[5] = initial_ptt_tail_time_tx6;  


}

//-------------------------------------------------------------------------------------------------------


void service_sequencer(){

  if (configuration.unit_state == TEST_ROUND_ROBIN_CW_30_S) { 
    if (((millis() - last_transition_time) >= 30000) || (!did_first_run)){
      if (!did_first_run){
        configuration.current_tx = 1;
        did_first_run = 1;
      } else {
        configuration.current_tx++;
        if (configuration.current_tx > number_of_transmitters){
          configuration.current_tx = 1;
        }
      }
      tx(0);
      switch_to_tx(configuration.current_tx);
      tx(1);
      last_transition_time = millis();
      
    }
  }

  if (keyer_is_idle() && configuration.unit_state == BEACON) {    
    switch (sequence) {
      case 1:  configuration.char_send_mode = SEND_MODE_CW; 
               send_character_string_on_tx((char*)"V",1); break;
      case 2:  send_character_string_on_tx((char*)"V",2); break;   
      case 3:  send_character_string_on_tx((char*)"V",1); break;
      case 4:  send_character_string_on_tx((char*)"V",2); break;  
      case 5:  send_character_string_on_tx((char*)"V",1); break;
      case 6:  send_character_string_on_tx((char*)"V",2); break; 

      case 7:  send_character_string_on_tx((char*)" ",2); break; 

      case 8:  send_character_string_on_tx((char*)"D",1); break;
      case 9:  send_character_string_on_tx((char*)"D",2); break; 
      case 10: send_character_string_on_tx((char*)"E",1); break;
      case 11: send_character_string_on_tx((char*)"E",2); break; 

      case 12: send_character_string_on_tx((char*)" ",2); break;      

      case 13: send_character_string_on_tx((char*)"K",1); break;
      case 14: send_character_string_on_tx((char*)"K",2); break;  
      case 15: send_character_string_on_tx((char*)"3",1); break;
      case 16: send_character_string_on_tx((char*)"3",2); break;  
      case 17: send_character_string_on_tx((char*)"N",1); break;
      case 18: send_character_string_on_tx((char*)"N",2); break;  
      case 19: send_character_string_on_tx((char*)"G",1); break;
      case 20: send_character_string_on_tx((char*)"G",2); break; 
      case 21: send_character_string_on_tx((char*)"/",1); break;
      case 22: send_character_string_on_tx((char*)"/",2); break;  
      case 23: send_character_string_on_tx((char*)"B",1); break;
      case 24: send_character_string_on_tx((char*)"B",2); break;       

      case 25: send_character_string_on_tx((char*)" ",2); break;

      case 26: send_character_string_on_tx((char*)"F",1); break;
      case 27: send_character_string_on_tx((char*)"F",2); break;  
      case 28: send_character_string_on_tx((char*)"N",1); break;
      case 29: send_character_string_on_tx((char*)"N",2); break;  
      case 30: send_character_string_on_tx((char*)"2",1); break;
      case 31: send_character_string_on_tx((char*)"2",2); break;  
      case 32: send_character_string_on_tx((char*)"0",1); break;
      case 33: send_character_string_on_tx((char*)"0",2); break;  
   
      #if !defined(NO_HELLSCREIBER)
        case 34:
        configuration.char_send_mode = SEND_MODE_HELL; 
        send_character_string_on_tx((char*)"VVV DE K3NG/B",1); break;

        case 35:
        configuration.char_send_mode = SEND_MODE_HELL; 
        send_character_string_on_tx((char*)"VVV DE K3NG/B",2); break;   
      #endif //NO_HELLSCREIBER   

      default: sequence = 0; beacon_cycle_count++; break;
    }    
    sequence++;         
  }

}  

//-------------------------------------------------------------------------------------------------------

void initialize_pins(){

  if (tx_1) {
    pinMode (tx_1, OUTPUT);
    digitalWrite (tx_1, LOW);
  }  
  if (tx_2) {    
    pinMode (tx_2, OUTPUT);
    digitalWrite (tx_2, LOW);
  }
  if (tx_3) {
    pinMode (tx_3, OUTPUT);
    digitalWrite (tx_3, LOW);
  }  
  if (tx_4) {
    pinMode (tx_4, OUTPUT);
    digitalWrite (tx_4, LOW);
  }  
  if (tx_5) {
    pinMode (tx_5, OUTPUT);
    digitalWrite (tx_5, LOW);
  }  
  if (tx_6) {
    pinMode (tx_6, OUTPUT);
    digitalWrite (tx_6, LOW);
  }   

  
  if (ptt_tx_1) {
    pinMode (ptt_tx_1, OUTPUT);
    digitalWrite (ptt_tx_1, LOW);
  }  
  if (ptt_tx_2) {    
    pinMode (ptt_tx_2, OUTPUT);
    digitalWrite (ptt_tx_2, LOW);
  }
  if (ptt_tx_3) {
    pinMode (ptt_tx_3, OUTPUT);
    digitalWrite (ptt_tx_3, LOW);
  }  
  if (ptt_tx_4) {
    pinMode (ptt_tx_4, OUTPUT);
    digitalWrite (ptt_tx_4, LOW);
  }  
  if (ptt_tx_5) {
    pinMode (ptt_tx_5, OUTPUT);
    digitalWrite (ptt_tx_5, LOW);
  }  
  if (ptt_tx_6) {
    pinMode (ptt_tx_6, OUTPUT);
    digitalWrite (ptt_tx_6, LOW);
  }  
  pinMode (sidetone_line, OUTPUT);
  digitalWrite (sidetone_line, LOW);  

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);


}




//-------------------------------------------------------------------------------------------------------

void initialize_dds(){

  ddschip.set_clock_multiplier(dds_chip_clock_multiplier);
  ddschip.init_chip();

}

//-------------------------------------------------------------------------------------------------------


void initialize_serial(){

  serial_baud_rate = default_serial_baud_rate;
  Serial.begin(serial_baud_rate);
  serial_uptime_stamp();
  Serial.print(F("K3NG Beacon Controller Version "));
  Serial.write(CODE_VERSION);
  Serial.println();

}


//-------------------------------------------------------------------------------------------------------
void switch_to_tx(byte transmitter)
{
  
  switch(transmitter) {
    case 1:
      current_tx_on_frequency = tx_1_frequency;
      ddschip.calibrate(tx_1_frequency_calibration);
      configuration.current_tx = 1;
      break;
    case 2:
      current_tx_on_frequency = tx_2_frequency;
      ddschip.calibrate(tx_2_frequency_calibration);
      configuration.current_tx = 2;
      break;
    case 3:
      current_tx_on_frequency = tx_3_frequency;
      ddschip.calibrate(tx_3_frequency_calibration);
      configuration.current_tx = 1;
      break;
    case 4:
      current_tx_on_frequency = tx_4_frequency;
      ddschip.calibrate(tx_4_frequency_calibration);
      configuration.current_tx = 2;
      break;
    case 5:
      current_tx_on_frequency = tx_5_frequency;
      ddschip.calibrate(tx_5_frequency_calibration);
      configuration.current_tx = 1;
      break;
    case 6:
      current_tx_on_frequency = tx_6_frequency;
      ddschip.calibrate(tx_6_frequency_calibration);
      configuration.current_tx = 2;
      break;            
  }  
  
}

//-------------------------------------------------------------------------------------------------------
byte keyer_is_idle() {
  
  if ((!char_send_buffer_bytes) && (!element_send_buffer_bytes) && (!ptt_line_activated) && (key_scheduler_state == IDLE)) {
    return 1;
  } else {
    return 0;
  }
  
}

//-------------------------------------------------------------------------------------------------------

void send_character_string_on_tx(char* string_to_send, byte transmitter) {

  switch_to_tx(transmitter);
  send_character_string(string_to_send);

}

//-------------------------------------------------------------------------------------------------------
void send_character_string(char* string_to_send) {
  
  for (int x = 0;x < 32;x++) {
    if (string_to_send[x] != 0) {
      add_to_char_send_buffer(string_to_send[x]);
    } else {
      x = 33;
    }
  }
}
//-------------------------------------------------------------------------------------------------------
void tx(byte state)
{
  switch (configuration.current_tx){
    case 1: digitalWrite (tx_1, state); break;
    case 2: digitalWrite (tx_2, state); break;
    case 3: digitalWrite (tx_3, state); break;
    case 4: digitalWrite (tx_4, state); break;
    case 5: digitalWrite (tx_5, state); break;
    case 6: digitalWrite (tx_6, state); break;
  }  


  if (state){
    ddschip.setfrequency(current_tx_on_frequency);
  } else {
    ddschip.setfrequency(0);
  }

}


//-------------------------------------------------------------------------------------------------------

void ptt(byte state)
{
  switch (configuration.current_tx){
    case 1: if (ptt_tx_1) {digitalWrite (ptt_tx_1, state);} break;
    case 2: if (ptt_tx_2) {digitalWrite (ptt_tx_2, state);} break;
    case 3: if (ptt_tx_3) {digitalWrite (ptt_tx_3, state);} break;
    case 4: if (ptt_tx_4) {digitalWrite (ptt_tx_4, state);} break;
    case 5: if (ptt_tx_5) {digitalWrite (ptt_tx_5, state);} break;
    case 6: if (ptt_tx_6) {digitalWrite (ptt_tx_6, state);} break;
  }  
}

//-------------------------------------------------------------------------------------------------------

void tx_and_sidetone_key (int state)
{
  if ((state) && (key_state == 0)) {
    if (key_tx) {
      ptt_key();      
      tx(HIGH);
    }
    if (configuration.sidetone_mode == SIDETONE_ON){
      tone(sidetone_line, configuration.hz_sidetone);
    }
    key_state = 1;
  } else {
    if ((state == 0) && (key_state)) {
      if (key_tx) {
        tx(LOW);
      }
      if (configuration.sidetone_mode == SIDETONE_ON) {
        noTone(sidetone_line);
      }
      key_state = 0;
    }          
  }
}  

//-------------------------------------------------------------------------------------------------------
void check_ptt_tail()
{ 
  if ((key_state) || (key_scheduler_state == PTT_LEAD_TIME_WAIT)) {
    ptt_time = millis();
  } else {
    if ((ptt_line_activated) && (manual_ptt_invoke == 0) && ((millis() - ptt_time) > ptt_tail_time)){
      ptt_unkey();
    }
  }
}

//-------------------------------------------------------------------------------------------------------


void ptt_key()
{
  if (ptt_line_activated == 0) {   // if PTT is currently deactivated, bring it up and insert PTT lead time delay
    ptt(HIGH);
    ptt_line_activated = 1;      
  }
  ptt_time = millis();
}

//-------------------------------------------------------------------------------------------------------

void ptt_unkey()
{
  if (ptt_line_activated) {
    ptt(LOW);
    ptt_line_activated = 0;      
  }  
}

//-------------------------------------------------------------------------------------------------------

void schedule_keydown_keyup (unsigned int keydown_ms, unsigned int keyup_ms)
{
  if (keydown_ms) {
    if ((ptt_lead_time) && (!ptt_line_activated)) {
      ptt_key();
      key_scheduler_state = PTT_LEAD_TIME_WAIT;
      next_key_scheduler_transition_time = millis() + ptt_lead_time;
      key_scheduler_keydown_ms = keydown_ms;
      key_scheduler_keyup_ms = keyup_ms;
    } else {
      tx_and_sidetone_key(1);
      key_scheduler_state = KEY_DOWN;
      next_key_scheduler_transition_time = millis() + keydown_ms;
      key_scheduler_keyup_ms = keyup_ms;      
    }
  } else {
    tx_and_sidetone_key(0);
    key_scheduler_state = KEY_UP;
    next_key_scheduler_transition_time = millis() + keyup_ms;
  }
  
  
}

//-------------------------------------------------------------------------------------------------------

void service_key_scheduler()
{
  
  switch (key_scheduler_state) {
    case PTT_LEAD_TIME_WAIT:
      if (millis() >= next_key_scheduler_transition_time) {
        tx_and_sidetone_key(1);
        key_scheduler_state = KEY_DOWN;
        next_key_scheduler_transition_time = (millis() + key_scheduler_keydown_ms);
      }
      break;        
    case KEY_DOWN:
      if (millis() >= next_key_scheduler_transition_time) {
        tx_and_sidetone_key(0);
        key_scheduler_state = KEY_UP;
        if (key_scheduler_keyup_ms) {
          next_key_scheduler_transition_time = (millis() + key_scheduler_keyup_ms);
        } else {
          key_scheduler_state = IDLE;
        }
      }
      break;
    case KEY_UP:
      if (millis() >= next_key_scheduler_transition_time) {
        key_scheduler_state = IDLE;
      }
      break;    
  }
}

//-------------------------------------------------------------------------------------------------------

void service_char_send_buffer()
{
  // send one character out of the send buffer

  if ((char_send_buffer_bytes > 0) && (pause_sending_buffer == 0) && (element_send_buffer_bytes == 0)) {
    //send_char(char_send_buffer_array[0],NORMAL);
    send_char(char_send_buffer_array[0],OMIT_LETTERSPACE);
    remove_from_char_send_buffer();    
  } 
  
}

//-------------------------------------------------------------------------------------------------------

void remove_from_char_send_buffer()
{
  if (char_send_buffer_bytes > 0) {
    char_send_buffer_bytes--;
  }
  if (char_send_buffer_bytes > 0) {
    for (int x = 0;x < char_send_buffer_bytes;x++) {
      char_send_buffer_array[x] = char_send_buffer_array[x+1];
    }
  }
}

//-------------------------------------------------------------------------------------------------------

void add_to_char_send_buffer(byte incoming_serial_byte)
{
//  if ((incoming_serial_byte == SERIAL_SEND_BUFFER_HOLD_SEND_RELEASE) && (char_send_buffer_status == SERIAL_SEND_BUFFER_HOLD)) {
//    char_send_buffer_status = SERIAL_SEND_BUFFER_NORMAL;
//  } else {
    if (char_send_buffer_bytes < char_send_buffer_size) {
      if (incoming_serial_byte != 127) {
        char_send_buffer_bytes++;
        char_send_buffer_array[char_send_buffer_bytes - 1] = incoming_serial_byte;
      } else {  // we got a backspace
        char_send_buffer_bytes--;
      }
    } 
//  }
}


//-------------------------------------------------------------------------------------------------------

void send_char(char cw_char, byte omit_letterspace)
{
  #ifdef DEBUG_SEND_CHAR
    Serial.write("\nsend_char: called with cw_char:");
    Serial.print(cw_char);
    if (omit_letterspace) {
      Serial.print (" OMIT_LETTERSPACE");
    }
    Serial.write("\n\r");
  #endif
  
  if ((cw_char == 10) || (cw_char == 13)) { return; }  // don't attempt to send carriage return or line feed
  
  if (configuration.char_send_mode == SEND_MODE_CW) {
    switch (cw_char) {
      case 'A': send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); break;
      case 'B': send_dah(AUTOMATIC_SENDING); send_dits(3); break;
      case 'C': send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); break;
      case 'D': send_dah(AUTOMATIC_SENDING); send_dits(2); break;
      case 'E': send_dit(AUTOMATIC_SENDING); break;
      case 'F': send_dits(2); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); break;
      case 'G': send_dahs(2); send_dit(AUTOMATIC_SENDING); break;
      case 'H': send_dits(4); break;
      case 'I': send_dits(2); break;
      case 'J': send_dit(AUTOMATIC_SENDING); send_dahs(3); break;
      case 'K': send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); break;
      case 'L': send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dits(2); break;
      case 'M': send_dahs(2); break;
      case 'N': send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); break;
      case 'O': send_dahs(3); break;
      case 'P': send_dit(AUTOMATIC_SENDING); send_dahs(2); send_dit(AUTOMATIC_SENDING); break;
      case 'Q': send_dahs(2); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); break;
      case 'R': send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); break;
      case 'S': send_dits(3); break;
      case 'T': send_dah(AUTOMATIC_SENDING); break;
      case 'U': send_dits(2); send_dah(AUTOMATIC_SENDING); break;    
      case 'V': send_dits(3); send_dah(AUTOMATIC_SENDING); break;
      case 'W': send_dit(AUTOMATIC_SENDING); send_dahs(2); break;
      case 'X': send_dah(AUTOMATIC_SENDING); send_dits(2); send_dah(AUTOMATIC_SENDING); break;
      case 'Y': send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dahs(2); break;
      case 'Z': send_dahs(2); send_dits(2); break;
          
      case '0': send_dahs(5); break;
      case '1': send_dit(AUTOMATIC_SENDING); send_dahs(4); break;
      case '2': send_dits(2); send_dahs(3); break;
      case '3': send_dits(3); send_dahs(2); break;
      case '4': send_dits(4); send_dah(AUTOMATIC_SENDING); break;
      case '5': send_dits(5); break;
      case '6': send_dah(AUTOMATIC_SENDING); send_dits(4); break;
      case '7': send_dahs(2); send_dits(3); break;
      case '8': send_dahs(3); send_dits(2); break;
      case '9': send_dahs(4); send_dit(AUTOMATIC_SENDING); break;
      
      case '=': send_dah(AUTOMATIC_SENDING); send_dits(3); send_dah(AUTOMATIC_SENDING); break;
      case '/': send_dah(AUTOMATIC_SENDING); send_dits(2); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); break;
      case ' ': add_to_element_send_buffer(KEY_UP_WORDSPACE_MINUS_4); break;
      case '*': send_dah(AUTOMATIC_SENDING); send_dits(3); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); break;    // using asterisk for BK
      case '.': send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); break;
      case ',': send_dahs(2); send_dits(2); send_dahs(2); break;
      case '\'': send_dit(AUTOMATIC_SENDING); send_dahs(4); send_dit(AUTOMATIC_SENDING); break;                   // apostrophe
      case '!': send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dahs(2); break;
      case '(': send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dahs(2); send_dit(AUTOMATIC_SENDING); break;
      case ')': send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dahs(2); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); break;
      case '&': send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dits(3); break;
      case ':': send_dahs(3); send_dits(3); break;
      case ';': send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); break;
      case '+': send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); break;
      case '-': send_dah(AUTOMATIC_SENDING); send_dits(4); send_dah(AUTOMATIC_SENDING); break;
      case '_': send_dits(2); send_dahs(2); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); break;
      case '"': send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dits(2); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); break;
      case '$': send_dits(3); send_dah(AUTOMATIC_SENDING); send_dits(2); send_dah(AUTOMATIC_SENDING); break;
      case '@': send_dit(AUTOMATIC_SENDING); send_dahs(2); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); break;
      case '<': send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); break;     // AR
      case '>': send_dits(3); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); break;               // SK
      case '\n': break;
      case '\r': break;
      case '|': add_to_element_send_buffer(HALF_UNIT_KEY_UP); return; break;  
      default: send_dits(2); send_dahs(2); send_dits(2); break;
    }  
    if (omit_letterspace != OMIT_LETTERSPACE) {
      add_to_element_send_buffer(KEY_UP_LETTERSPACE_MINUS_1); //this is minus one because send_dit and send_dah have a trailing element space
    }
  } else {
    if (configuration.char_send_mode == SEND_MODE_HELL){
      #if !defined(NO_HELLSCREIBER)
        transmit_hell_char(cw_char);
      #endif
    }

  }
  
}

//-------------------------------------------------------------------------------------------------------
void send_dit(byte sending_type) {
  add_to_element_send_buffer(ONE_UNIT_KEY_DOWN_1_UNIT_KEY_UP);
}

//-------------------------------------------------------------------------------------------------------
void send_dah(byte sending_type) {
  add_to_element_send_buffer(THREE_UNITS_KEY_DOWN_1_UNIT_KEY_UP);
}

//-------------------------------------------------------------------------------------------------------

void send_dits(int dits){

  for (;dits > 0;dits--) {
    send_dit(AUTOMATIC_SENDING);
  } 
  
}

//-------------------------------------------------------------------------------------------------------

void send_dahs(int dahs){

  for (;dahs > 0;dahs--) {
    send_dah(AUTOMATIC_SENDING);
  } 
  
}

//-------------------------------------------------------------------------------------------------------

void add_to_element_send_buffer(byte element_byte){

  if (element_send_buffer_bytes < element_send_buffer_size) {
    element_send_buffer_array[element_send_buffer_bytes] = element_byte;
    element_send_buffer_bytes++;
  } 

}

//-------------------------------------------------------------------------------------------------------

void remove_from_element_send_buffer(){

  if (element_send_buffer_bytes > 0) {
    element_send_buffer_bytes--;
  }
  if (element_send_buffer_bytes > 0) {
    for (int x = 0;x < element_send_buffer_bytes;x++) {
      element_send_buffer_array[x] = element_send_buffer_array[x+1];
    }
  }

}

//-------------------------------------------------------------------------------------------------------


void service_element_send_buffer(){
  
  /*
  enum element_buffer_type {HALF_UNIT_KEY_UP, ONE_UNIT_KEY_DOWN_1_UNIT_KEY_UP, THREE_UNITS_KEY_DOWN_1_UNIT_KEY_UP,
  ONE_UNIT_KEYDOWN_3_UNITS_KEY_UP, THREE_UNIT_KEYDOWN_3_UNITS_KEY_UP,
  ONE_UNIT_KEYDOWN_7_UNITS_KEY_UP, THREE_UNIT_KEYDOWN_7_UNITS_KEY_UP, 
  SEVEN_UNITS_KEY_UP, KEY_UP_LETTERSPACE_MINUS_1, KEY_UP_WORDSPACE_MINUS_1};
  */
  
  if ((key_scheduler_state == IDLE) && (element_send_buffer_bytes > 0)) {
    switch(element_send_buffer_array[0]) {
 
      case HALF_UNIT_KEY_UP:                   schedule_keydown_keyup(0,0.5*(1200/configuration.wpm));                    break;
      case ONE_UNIT_KEY_DOWN_1_UNIT_KEY_UP:    schedule_keydown_keyup(1200/configuration.wpm,1200/configuration.wpm);                   break;
      case THREE_UNITS_KEY_DOWN_1_UNIT_KEY_UP: schedule_keydown_keyup(3*(1200/configuration.wpm),1200/configuration.wpm);               break;
      case KEY_UP_LETTERSPACE_MINUS_1:         schedule_keydown_keyup(0,(length_letterspace-1)*(1200/configuration.wpm)); break;         
      case KEY_UP_WORDSPACE_MINUS_4:           schedule_keydown_keyup(0,(length_wordspace-4)*(1200/configuration.wpm));   break;  
      case KEY_UP_WORDSPACE:                   schedule_keydown_keyup(0,length_wordspace*(1200/configuration.wpm));       break;    
      #if !defined(NO_HELLSCREIBER)     
        case HELL_PIXEL_0:                       schedule_keydown_keyup(0,3);                                 break;
        case HELL_PIXEL_1:                       schedule_keydown_keyup(3,0);                                 break;
      #endif
    }
    remove_from_element_send_buffer();
  }
  
  
}

//---------------------------------------------------------------------


void check_serial(){
  
  while (Serial.available() > 0) {  
    if (serial_backslash_command == 0) {
      incoming_serial_byte = Serial.read(); 
      incoming_serial_byte = uppercase(incoming_serial_byte);
      if (incoming_serial_byte != 92) {
      } else {                                // we're getting a backslash command
        serial_backslash_command = 1;
        Serial.write(incoming_serial_byte);          
      }
    } else {       
        incoming_serial_byte = Serial.read();
        Serial.write(incoming_serial_byte);
        incoming_serial_byte = uppercase(incoming_serial_byte);
        process_serial_command(incoming_serial_byte);
        serial_backslash_command = 0; 
        Serial.write("\n\r");        
    }
  }  //while (Serial.available() > 0)
}



//---------------------------------------------------------------------


void process_serial_command(byte incoming_serial_byte) {

  Serial.print(F("\n\r"));
  
  switch (incoming_serial_byte) {   
    case 'P': serial_program_memory(); break;
    case 'S': serial_status(); break;
    case 'W': serial_wpm_set();break; 
    //case 126: wdt_enable(WDTO_30MS); while(1) {} ; break;  // ~ - reset unit  DO NOT SCREW WITH THIS - Bootloader bug issues

    default: Serial.write("\nUnknown command\n\r"); break;
  }
  
          
}

//---------------------------------------------------------------------


void serial_program_memory()
{


  /*

  CLI memory programming test string

  WE LOVE RADIO AND SMALL COMPUTING DEVICES AND WE COMBINE THE TWO TO AUTOMATE EXPERIMENT OR JUST SEE IF SOMETHING CAN BE DONE WE BELIEVE ITS BETTER TO BUILD SOMETHING WITH OUR OWN HANDS HOWEVER SMALL OR IMPERFECT AND IMPROVE AND EXPAND IT OVER TIME WE SUPPORT EXPERIMENTERS OF ALL LEVELS AND EXCHANGE IDEAS ABOUT AMATEUR RADIO HARDWARE HOMEBREWING AND SOFTWARE DEVELOPMENT

  */

  uint8_t incoming_serial_byte = 0;
  uint8_t memory_number = 0;
  uint8_t looping = 1;
  int memory_index = 0;
  uint8_t memory_number_entered = 0;
  uint8_t memory_data_entered = 0;
  uint8_t error_flag = 0;
  uint8_t memory_1_or_1x_flag = 0;

  
  uint8_t incoming_serial_byte_buffer[serial_program_memory_buffer_size];
  unsigned int incoming_serial_byte_buffer_size = 0;


  while (looping){

    while ((Serial.available()) && (incoming_serial_byte_buffer_size < serial_program_memory_buffer_size)){  // get serial data if available
      incoming_serial_byte_buffer[incoming_serial_byte_buffer_size] = uppercase(Serial.read()); 
      incoming_serial_byte_buffer_size++;
    }

    if (incoming_serial_byte_buffer_size){
      incoming_serial_byte = incoming_serial_byte_buffer[0];
      Serial.write(incoming_serial_byte);
      for (unsigned int x = 0;x < (incoming_serial_byte_buffer_size - 1);x++){
        incoming_serial_byte_buffer[x] = incoming_serial_byte_buffer[x+1];
      }
      incoming_serial_byte_buffer_size--;

      if ((memory_1_or_1x_flag) && ((incoming_serial_byte < 48) || (incoming_serial_byte > 57))){  // do we have something other than a number?
        memory_1_or_1x_flag = 0;
        memory_number_entered = 1;
      }

      if (!memory_number_entered) {
        if ((incoming_serial_byte > 47) && (incoming_serial_byte < 58)) {  // do we have a number?
          if (memory_1_or_1x_flag){    
            memory_number = incoming_serial_byte - 48 + 10;
            memory_1_or_1x_flag = 0;
            memory_number_entered = 1;
          } else {
            memory_number = incoming_serial_byte - 48;
            if ((memory_number == 1) && (configuration.number_of_memories > 9)) {
              memory_1_or_1x_flag = 1;
            } else {
              memory_number_entered = 1;
            }
          }
          // memory number out of range check
          if (memory_number > configuration.number_of_memories){
            looping = 0;
            error_flag = 1;
          }
        } else {
          looping = 0;
          error_flag = 1;
        }

      } else {

        if (incoming_serial_byte == 13){  // we got a carriage return
          looping = 0;
          EEPROM.write((memory_start(memory_number-1)+memory_index),255);
        } else {  // looking for memory data
          memory_data_entered = 1;
          #if !defined(OPTION_SAVE_MEMORY_NANOKEYER)
            while ((Serial.available()) && (incoming_serial_byte_buffer_size < serial_program_memory_buffer_size)){  // get serial data if available
              incoming_serial_byte_buffer[incoming_serial_byte_buffer_size] = uppercase(Serial.read()); 
              incoming_serial_byte_buffer_size++;
            }  
          #endif           
          EEPROM.write((memory_start(memory_number-1)+memory_index),incoming_serial_byte);
          #if !defined(OPTION_SAVE_MEMORY_NANOKEYER)
            while ((Serial.available()) && (incoming_serial_byte_buffer_size < serial_program_memory_buffer_size)){  // get serial data if available
              incoming_serial_byte_buffer[incoming_serial_byte_buffer_size] = uppercase(Serial.read()); 
              incoming_serial_byte_buffer_size++;
            }   
          #endif              
          #ifdef DEBUG_EEPROM
            Serial.print(F("serial_program_memory: wrote "));
            Serial.print(incoming_serial_byte);
            Serial.print(F(" to location "));
            Serial.println((memory_start(memory_number-1)+memory_index));
          #endif
          memory_index++;
          if ((memory_start(memory_number-1) + memory_index) > (memory_end(memory_number-1)-2)) {    // are we at last memory location?
            looping = 0;
            EEPROM.write((memory_start(memory_number-1)+memory_index),255);
            Serial.println(F("Memory full, truncating."));
          }

        }

      }
    }

  }

  if ((memory_number_entered) && (memory_data_entered) && (!error_flag)){
    Serial.print(F("\n\rWrote memory "));
    Serial.println(memory_number);
  } else {
    Serial.println(F("\n\rError"));
  }

  #if defined(ARDUINO_SAMD_VARIANT_COMPLIANCE)
    EEPROM.commit();
  #endif

}


//---------------------------------------------------------------------


void serial_wpm_set() {
  int set_wpm = serial_get_number_input(3,0,1000);
  if ((set_wpm > 0) && (set_wpm < 100)){
    configuration.wpm = set_wpm;
    config_dirty = 1;
    Serial.write("\r\nSetting WPM to ");
    Serial.println(set_wpm,DEC);
  }
}

//---------------------------------------------------------------------


int serial_get_number_input(byte places,int lower_limit, int upper_limit)
{
  byte incoming_serial_byte = 0;
  byte looping = 1;
  byte error = 0;
  String numberstring = "";
  byte numberindex = 0;
  int numbers[6];
  
  while (looping) {
    if (Serial.available() == 0) {        // wait for the next keystroke

    } else {  
      incoming_serial_byte = Serial.read();
      if ((incoming_serial_byte > 47) && (incoming_serial_byte < 58)) {    // ascii 48-57 = "0" - "9")
        numberstring = numberstring + incoming_serial_byte;
        numbers[numberindex] = incoming_serial_byte;
        numberindex++;
        if (numberindex > places){
            looping = 0;
            error = 1;
        }
      } else {
        if (incoming_serial_byte == 13) {   // carriage return - get out
          looping = 0;
        } else {                 // bogus input - error out
          looping = 0;
          error = 1;
        }
      }
    }
  }
  if (error) {
    Serial.write("Error...\n\r");
    while (Serial.available() > 0) { incoming_serial_byte = Serial.read(); }  // clear out buffer
    return(-1);
  } else {
    int y = 1;
    int return_number = 0;
    for (int x = (numberindex - 1); x >= 0 ; x = x - 1) {
      return_number = return_number + ((numbers[x]-48) * y);
      y = y * 10;
    }
    if ((return_number > lower_limit) && (return_number < upper_limit)) {
      return(return_number); 
    } else {
      Serial.write("Error...\n\r");  
      return(-1);
    }
  }
}


//---------------------------------------------------------------------


void serial_status() {
  

  serial_uptime_stamp();
  //memorycheck();
  Serial.print(F(" beacon_cycle_count="));
  Serial.print(beacon_cycle_count);
  Serial.print(F(" millis="));
  Serial.print(millis());
  Serial.print(F(" millis_rollover="));
  Serial.print(millis_rollover);
  Serial.print(F(" Vin1="));
  Serial.println((((float)analogRead(analog_vin1)*(float)analog_vin1_calibration_multiplier)) + (float)analog_vin1_calibration_offset );
  
  Serial.println();
  Serial.print("WPM: ");
  Serial.println(configuration.wpm);

  Serial.print(F("Sidetone: "));
  switch (configuration.sidetone_mode) {
    case SIDETONE_ON:  Serial.println(F("On")); break;
    case SIDETONE_OFF: Serial.println(F("Off")); break;
  }
  Serial.print("Sidetone_Freq: ");
  Serial.println(configuration.hz_sidetone,DEC);

  serial_status_memories();


}

//---------------------------------------------------------------------


void serial_status_memories(){
  int last_memory_location;

  // #if defined(OPTION_PROSIGN_SUPPORT)
  //   byte eeprom_temp = 0;
  //   static char * prosign_temp = 0;
  // #endif

  for (int x = 0; x < configuration.number_of_memories; x++) {
    last_memory_location = memory_end(x) + 1 ;
    Serial.write("Memory ");
    Serial.print(x+1);
    Serial.write(":");
    if ( EEPROM.read(memory_start(x)) == 255) {
      Serial.write("{empty}");
    } else {
      for (int y = (memory_start(x)); (y < last_memory_location); y++) {
        if (EEPROM.read(y) < 255) {
          // #if defined(OPTION_PROSIGN_SUPPORT)
          //   eeprom_temp = EEPROM.read(y);
          //   if ((eeprom_temp > PROSIGN_START) && (eeprom_temp < PROSIGN_END)){
          //     prosign_temp = convert_prosign(eeprom_temp);
          //     Serial.print(prosign_temp[0]);
          //     Serial.print(prosign_temp[1]);
          //   } else {
          //     Serial.write(eeprom_temp);
          //   }
          // #else         
            if ((EEPROM.read(y) == 32) && ((EEPROM.read(y+1) == 255) || ((y+1) >= last_memory_location))){
              Serial.write("_");
            } else {
              Serial.write(EEPROM.read(y));
            }
          // #endif //OPTION_PROSIGN_SUPPORT
        } else {
          //Serial.write("$");
          y = last_memory_location;
        }
      }
    }

    #if defined(DEBUG_MEMORY_LOCATIONS)
      Serial.print("  start: ");
      Serial.print(memory_start(x));
      Serial.print(" end: ");
      Serial.print(memory_end(x));
      Serial.print(" size: ");
      Serial.print(memory_end(x)-memory_start(x));
    #endif

    Serial.println();
  }
}


//---------------------------------------------------------------------
void serial_uptime_stamp() {
  
  unsigned long days = 0;
  unsigned long hours = 0;
  unsigned long minutes = 0;
  unsigned long seconds = 0;
  
  seconds = (millis() /1000);
  minutes = seconds /60;
  hours = minutes /60;
  days = hours /24;
  seconds = seconds - ( minutes * 60);
  minutes = minutes - ( hours * 60);
  hours = hours - ( days * 24);
  
  // add rollovers
  days = days + (millis_rollover * 49);
  hours = hours + (millis_rollover * 17);
  minutes = minutes + (millis_rollover * 2);
  seconds = seconds + (millis_rollover * 47);
  
  if (seconds > 59) {
    minutes = minutes + int(seconds / 60);
    seconds = seconds - (int(seconds/60) * 60);
  }
    
  if (minutes > 59) {
    hours = hours + int(minutes / 60);
    minutes = minutes - (int(minutes/60) * 60);
  }
   
  if (hours > 24) { 
    days = days + (int(hours / 24));
    hours = hours - (int(hours/24) * 24);
  }
  
  Serial.print(days);
  Serial.print(F(":"));
  if (hours < 10) {
    Serial.print(F("0"));
  }
  Serial.print(hours);
  Serial.print(F(":"));
  if (minutes < 10) {
    Serial.print(F("0"));
  }
  Serial.print(minutes);
  Serial.print(F(":"));
  if (seconds < 10) {
    Serial.print(F("0"));
  }  
  Serial.print(seconds);
  Serial.print(F(" - "));
  
}

//---------------------------------------------------------------------
void memorycheck()
{
  void* HP = malloc(4);
  if (HP)
    free (HP);
    
  unsigned long free = (unsigned long)SP - (unsigned long)HP;
  
  Serial.print(F("heap="));
  Serial.print((unsigned long)HP,HEX);
  Serial.print(F(" stack="));
  Serial.print((unsigned long)SP,HEX);
  Serial.print(F(" free_memory_bytes="));
//  Serial.print((unsigned long)free,HEX);
//  Serial.print("  ");
  if (free > 2048) {
    free = 0;
  }
  Serial.print((unsigned long)free,DEC);
}

//---------------------------------------------------------------------

void millis_rollover_check() {
  
  static unsigned long last_millis = 0;
  
  if (millis() < last_millis) {
    millis_rollover++;
  }
  last_millis = millis();
}

//-------------------------------------------------------------------------------------------------------

int uppercase (int charbytein)
{
  if ((charbytein > 96) && (charbytein < 123)) {
    charbytein = charbytein - 32;
  }
  return charbytein;
}

//-------------------------------------------------------------------------------------------------------
#if !defined(NO_HELLSCREIBER)
void hell_test ()
{
  for (byte h = 65; h < 91; h++) {
    transmit_hell_char(h);
  }
  transmit_hell_char('0');
  transmit_hell_char('1');
  transmit_hell_char('2');
  transmit_hell_char('3');
  transmit_hell_char('4');
  transmit_hell_char('5');
  transmit_hell_char('6');
  transmit_hell_char('7');
  transmit_hell_char('8');
  transmit_hell_char('9');
  transmit_hell_char('+');
  transmit_hell_char('-');
  transmit_hell_char('?');
  transmit_hell_char('/');
  transmit_hell_char('.');
  transmit_hell_char(',');
  transmit_hell_char('!');
  transmit_hell_char('=');
  transmit_hell_char(')');
  transmit_hell_char('(');
  transmit_hell_char(':');
}
#endif

//-------------------------------------------------------------------------------------------------------
#if !defined(NO_HELLSCREIBER)
void transmit_hell_char(byte hellchar)
{

  // blank column
  for (byte w = 0; w < 14; w++) {
    #if defined(HELL_NOT_USING_SCHEDULER)
      transmit_hell_pixel(0);
    #else
      add_to_element_send_buffer(HELL_PIXEL_0);
    #endif
  }

  if ((hellchar > 64) && (hellchar < 91)) {    // A - Z
    hellchar = ((hellchar - 65) * 9);
    transmit_hell_pixels(hell_font1, hellchar);
  } else {
    if ((hellchar > 47) && (hellchar < 58)) {  // 0 - 9
      hellchar = ((hellchar - 48) * 9);
      transmit_hell_pixels(hell_font2, hellchar);
    } else {
      switch (hellchar) {
        case '+': hellchar = 0; break;
        case '-': hellchar = 1; break;
        case '?': hellchar = 2; break;
        case '/': hellchar = 3; break;
        case '.': hellchar = 4; break;
        case ',': hellchar = 5; break;
        case '=': hellchar = 7; break;
        case ')': hellchar = 8; break;
        case '(': hellchar = 9; break;
        case ':': hellchar = 10; break;
        default : hellchar = 11; break;
      }
      hellchar = hellchar * 9;
      transmit_hell_pixels(hell_font3, hellchar);

    }
  }

  // blank column
  for (byte w = 0; w < 14; w++) {
    #if defined(HELL_NOT_USING_SCHEDULER)
      transmit_hell_pixel(0);
    #else
      add_to_element_send_buffer(HELL_PIXEL_0);
    #endif
  }

}
#endif

//-------------------------------------------------------------------------------------------------------
#if !defined(NO_HELLSCREIBER)
void transmit_hell_pixels(const char* hell_pixels, byte hellchar){


  for (byte x = 0; x < 9; x++) {
    for (int y = 7; y > -1; y--) {
      if ((x < 8) || ((x == 8) && (y > 1))) {  // drop the last 2 bits in byte 9
        if (bitRead(pgm_read_byte(hell_pixels + hellchar + x ),y)) {
          #if defined(HELL_NOT_USING_SCHEDULER)
            transmit_hell_pixel(1);
          #else  
            add_to_element_send_buffer(HELL_PIXEL_1);
          #endif
        } else {
          #if defined(HELL_NOT_USING_SCHEDULER)
            transmit_hell_pixel(0);
          #else
            add_to_element_send_buffer(HELL_PIXEL_0);
          #endif
        }
      }
    }
  }

}
#endif

//-------------------------------------------------------------------------------------------------------
#if defined(HELL_NOT_USING_SCHEDULER) && !defined(NO_HELLSCREIBER)

  void transmit_hell_pixel(byte hellbit){

    //sending_mode = AUTOMATIC_SENDING;
    if (hellbit) {
      tx(1);
    } else {
      tx(0);
    }
    delayMicroseconds(hell_pixel_microseconds);
  }

#endif
//-------------------------------------------------------------------------------------------------------

int read_settings_from_eeprom() {

  // returns 0 if eeprom had valid settings, returns 1 if eeprom needs initialized

  #if defined(DEBUG_FORCE_RESET)
    return 1;
  #endif
  


  #if !defined(ARDUINO_SAM_DUE) || (defined(ARDUINO_SAM_DUE) && defined(FEATURE_EEPROM_E24C1024))

    #if defined(DEBUG_EEPROM_READ_SETTINGS)
      Serial.println(F("read_settings_from_eeprom: start"));
    #endif

    if (EEPROM.read(0) == eeprom_magic_number){
    
      byte* p = (byte*)(void*)&configuration;
      unsigned int i;
      int ee = 1; // starting point of configuration struct
      for (i = 0; i < sizeof(configuration); i++){
        #if defined(DEBUG_EEPROM_READ_SETTINGS)
          Serial.print(F("read_settings_from_eeprom: read: i:"));
          Serial.print(i);
          Serial.print(F(":"));
          Serial.print(EEPROM.read(ee));
          Serial.println();
        #endif
        *p++ = EEPROM.read(ee++);  
      }
    

      config_dirty = 0;

      #if defined(DEBUG_EEPROM_READ_SETTINGS)
        Serial.println(F("read_settings_from_eeprom: read complete"));
      #endif
      return 0;
    } else {
      #if defined(DEBUG_EEPROM_READ_SETTINGS)
        Serial.println(F("read_settings_from_eeprom: eeprom needs initialized"));
      #endif      
      return 1;
    }
  
  #endif //!defined(ARDUINO_SAM_DUE) || (defined(ARDUINO_SAM_DUE) && defined(FEATURE_EEPROM_E24C1024))

  #if defined(DEBUG_EEPROM_READ_SETTINGS)
    Serial.println(F("read_settings_from_eeprom: bypassed read - no eeprom"));
  #endif
 
  return 1;

}

//-------------------------------------------------------------------------------------------------------

void check_for_dirty_configuration()
{


  if ((config_dirty) && ((millis()-last_config_write)>eeprom_write_time_ms) && (!char_send_buffer_bytes) && (!ptt_line_activated)) {
  //if ((config_dirty) && ((millis()-last_config_write)>eeprom_write_time_ms) && (!send_buffer_bytes) && (!ptt_line_activated) && (!dit_buffer) && (!dah_buffer) && (!async_eeprom_write) && (paddle_pin_read(paddle_left) == HIGH)  && (paddle_pin_read(paddle_right) == HIGH) ) {
    write_settings_to_eeprom(0);
    last_config_write = millis();
    #ifdef DEBUG_EEPROM
      Serial.println(F("check_for_dirty_configuration: wrote config\n"));
    #endif
  }

}

//-------------------------------------------------------------------------------------------------------

void write_settings_to_eeprom(int initialize_eeprom) {  
 
  #if !defined(ARDUINO_SAM_DUE) || (defined(ARDUINO_SAM_DUE) && defined(FEATURE_EEPROM_E24C1024))
  
    if (initialize_eeprom) {
      EEPROM.write(0,eeprom_magic_number);
      initialize_eeprom_memories();
      const byte* p = (const byte*)(const void*)&configuration;
      unsigned int i;
      int ee = 1;  // starting point of configuration struct
      for (i = 0; i < sizeof(configuration); i++){
        EEPROM.write(ee++, *p++);  
      }        
    } else {

      async_eeprom_write = 1;  // initiate an asyncrhonous eeprom write

    }
  
  #endif //!defined(ARDUINO_SAM_DUE) || (defined(ARDUINO_SAM_DUE) && defined(FEATURE_EEPROM_E24C1024))

  config_dirty = 0;
  
}

//-------------------------------------------------------------------------------------------------------

void initialize_eeprom_memories()
{
  for (int x = 0; x < configuration.number_of_memories; x++) {
    EEPROM.write(memory_start(x),255);
  }
}


//-------------------------------------------------------------------------------------------------------

void service_async_eeprom_write(){

  // This writes one byte out to EEPROM each time it is called

  static byte last_async_eeprom_write_status = 0;
  static int ee = 0;
  static unsigned int i = 0;
  static const byte* p;

  if ((async_eeprom_write) && (!char_send_buffer_bytes) && (!ptt_line_activated)) {  
    if (last_async_eeprom_write_status){ // we have an ansynchronous write to eeprom in progress


      #if defined(_BOARD_PIC32_PINGUINO_) || defined(ARDUINO_SAMD_VARIANT_COMPLIANCE)
        if (EEPROM.read(ee) != *p) {
          EEPROM.write(ee, *p);
        }
        ee++;
        p++;
      #else
        EEPROM.update(ee++, *p++);
      #endif

      if (i < sizeof(configuration)){
        #if defined(DEBUG_ASYNC_EEPROM_WRITE)
          Serial.print(F("service_async_eeprom_write: write: "));
          Serial.println(i);
        #endif       
        i++;
      } else { // we're done
        async_eeprom_write = 0;
        last_async_eeprom_write_status = 0;
        #if defined(ARDUINO_SAMD_VARIANT_COMPLIANCE)
          EEPROM.commit();
        #endif       

        #if defined(DEBUG_ASYNC_EEPROM_WRITE)
          Serial.println(F("service_async_eeprom_write: complete"));
        #endif    
      }

    } else { // we don't have one in progress - initialize things

      p = (const byte*)(const void*)&configuration;
      ee = 1;  // starting point of configuration struct
      i = 0;
      last_async_eeprom_write_status = 1;
      #if defined(DEBUG_ASYNC_EEPROM_WRITE)
        Serial.println(F("service_async_eeprom_write: init"));
      #endif
    }
  }

}

//-------------------------------------------------------------------------------------------------------



void check_eeprom_for_initialization(){

  // do an eeprom reset to defaults if paddles are squeezed
  // if (paddle_pin_read(paddle_left) == LOW && paddle_pin_read(paddle_right) == LOW) {
  //   while (paddle_pin_read(paddle_left) == LOW && paddle_pin_read(paddle_right) == LOW) {}
  //   initialize_eeprom();
  // }

  // read settings from eeprom and initialize eeprom if it has never been written to
  if (read_settings_from_eeprom()) {
    #if defined(HARDWARE_GENERIC_STM32F103C)
      EEPROM.init();
      EEPROM.format();
    #endif
    write_settings_to_eeprom(1);  // 1 = initialize eeprom
  }
}

//-------------------------------------------------------------------------------------------------------

int memory_start(byte memory_number) {
  return (memory_area_start + (memory_number * ((memory_area_end - memory_area_start) / configuration.number_of_memories)));
}

//-------------------------------------------------------------------------------------------------------

int memory_end(byte memory_number) {
  return (memory_start(memory_number) - 1 + ((memory_area_end - memory_area_start)/ configuration.number_of_memories));
}

//-------------------------------------------------------------------------------------------------------
