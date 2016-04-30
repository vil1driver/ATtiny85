/*
 * connectingStuff, Oregon Scientific v2.1 Emitter
 * http://connectingstuff.net/blog/encodage-protocoles-oregon-scientific-sur-arduino/
 *
 * Copyright (C) 2013 olivier.lebrun@gmail.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * V2 par vil1driver
 * sketch unique pour sonde ds18b20 ou DHT11/22
 * choix de la périodicité de transmission
 * remontée niveau de batterie
 * 
 * ajout d'au capteur PIR ou reed
 *
*/

/************************************************************

    emplacement des PIN de la puce ATtiny85
                     +----+
Ain0   (D  5)  PB5  1|*   |8   VCC
Ain3   (D  3)  PB3  2|    |7   PB2 (D  2) Ain1
Ain2   (D  4)  PB4  3|    |6   PB1 (D  1) pwm1
               GND  4|    |5   PB0 (D  0) pwm0
                     +----+ 
             
****************       Confuguration       *****************/


#define NODE_ID 0xCC // Identifiant unique de votre sonde (hexadecimal)
#define LOW_BATTERY_LEVEL 2700   // Voltage minumum (mV) avant d'indiquer batterie faible
#define WDT_COUNT  5     // Nombre de cycles entre chaque transmission (1 cycles = 8 secondes, 5x8 = 40s)

// commentez (ou supprimez) la ligne suivante si vous utilisez une sonde DHT11 ou DHT22
#define TEMP_ONLY   // sonde de température simple (ds18b20)

#define DATA_PIN 3 // pin 2 // data de la sonde
const byte TX_PIN = 4;  // pin 3 // data transmetteur

// commentez (ou supprimez) la ligne suivante si vous n'utilisez pas de capteur de mouvement
//#define PIR

const int PIR_PIN = 0; // pin 5 // wake up PIR output

#define PIR_HOUSE_CODE 1	// code maison du capteur de mouvement
#define PIR_UNIT_CODE 1		// code unite du capteur de mouvement


/****************   Fin de configuration    *****************/




// Chargement des librairies
#include <avr/sleep.h>    // Sleep Modes
#include <avr/wdt.h>      // Watchdog timer
#include <avr/interrupt.h>
#ifdef PIR
	#include  "RCSwitch.h"
#endif
	
#ifdef TEMP_ONLY
  #include "OneWire.h"
  #define DS18B20 0x28     // Adresse 1-Wire du DS18B20
  OneWire ds(DATA_PIN); // Création de l'objet OneWire ds
#else
  #include "DHT.h"
  DHT dht;
#endif

#ifndef cbi
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#ifdef PIR
	RCSwitch mySwitch = RCSwitch();
	volatile byte Motion = LOW;
#endif


volatile int count = 0;
volatile boolean lowBattery = false;
const unsigned long TIME = 512;
const unsigned long TWOTIME = TIME*2;

#define SEND_HIGH() digitalWrite(TX_PIN, HIGH)
#define SEND_LOW() digitalWrite(TX_PIN, LOW)
 
// Buffer for Oregon message
#ifdef TEMP_ONLY
  byte OregonMessageBuffer[8];
#else
  byte OregonMessageBuffer[9];
#endif
 
/**
 * \brief    Send logical "0" over RF
 * \details  azero bit be represented by an off-to-on transition
 * \         of the RF signal at the middle of a clock period.
 * \         Remenber, the Oregon v2.1 protocol add an inverted bit first 
 */
inline void sendZero(void) 
{
  SEND_HIGH();
  delayMicroseconds(TIME);
  SEND_LOW();
  delayMicroseconds(TWOTIME);
  SEND_HIGH();
  delayMicroseconds(TIME);
}
 
/**
 * \brief    Send logical "1" over RF
 * \details  a one bit be represented by an on-to-off transition
 * \         of the RF signal at the middle of a clock period.
 * \         Remenber, the Oregon v2.1 protocol add an inverted bit first 
 */
inline void sendOne(void) 
{
   SEND_LOW();
   delayMicroseconds(TIME);
   SEND_HIGH();
   delayMicroseconds(TWOTIME);
   SEND_LOW();
   delayMicroseconds(TIME);
}
 
/**
* Send a bits quarter (4 bits = MSB from 8 bits value) over RF
*
* @param data Source data to process and sent
*/
 
/**
 * \brief    Send a bits quarter (4 bits = MSB from 8 bits value) over RF
 * \param    data   Data to send
 */
inline void sendQuarterMSB(const byte data) 
{
  (bitRead(data, 4)) ? sendOne() : sendZero();
  (bitRead(data, 5)) ? sendOne() : sendZero();
  (bitRead(data, 6)) ? sendOne() : sendZero();
  (bitRead(data, 7)) ? sendOne() : sendZero();
}
 
/**
 * \brief    Send a bits quarter (4 bits = LSB from 8 bits value) over RF
 * \param    data   Data to send
 */
inline void sendQuarterLSB(const byte data) 
{
  (bitRead(data, 0)) ? sendOne() : sendZero();
  (bitRead(data, 1)) ? sendOne() : sendZero();
  (bitRead(data, 2)) ? sendOne() : sendZero();
  (bitRead(data, 3)) ? sendOne() : sendZero();
}
 
/******************************************************************/
/******************************************************************/
/******************************************************************/
 
/**
 * \brief    Send a buffer over RF
 * \param    data   Data to send
 * \param    size   size of data to send
 */
void sendData(byte *data, byte size)
{
  for(byte i = 0; i < size; ++i)
  {
    sendQuarterLSB(data[i]);
    sendQuarterMSB(data[i]);
  }
}
 
/**
 * \brief    Send an Oregon message
 * \param    data   The Oregon message
 */
void sendOregon(byte *data, byte size)
{
    sendPreamble();
    //sendSync();
    sendData(data, size);
    sendPostamble();
}
 
/**
 * \brief    Send preamble
 * \details  The preamble consists of 16 "1" bits
 */
inline void sendPreamble(void)
{
  byte PREAMBLE[]={0xFF,0xFF};
  sendData(PREAMBLE, 2);
}
 
/**
 * \brief    Send postamble
 * \details  The postamble consists of 8 "0" bits
 */
inline void sendPostamble(void)
{
#ifdef TEMP_ONLY
  sendQuarterLSB(0x00);
#else
  byte POSTAMBLE[]={0x00};
  sendData(POSTAMBLE, 1);  
#endif
}
 
/**
 * \brief    Send sync nibble
 * \details  The sync is 0xA. It is not use in this version since the sync nibble
 * \         is include in the Oregon message to send.
 */
inline void sendSync(void)
{
  sendQuarterLSB(0xA);
}
 
/******************************************************************/
/******************************************************************/
/******************************************************************/
 
/**
 * \brief    Set the sensor type
 * \param    data       Oregon message
 * \param    type       Sensor type
 */
inline void setType(byte *data, byte* type) 
{
  data[0] = type[0];
  data[1] = type[1];
}
 
/**
 * \brief    Set the sensor channel
 * \param    data       Oregon message
 * \param    channel    Sensor channel (0x10, 0x20, 0x30)
 */
inline void setChannel(byte *data, byte channel) 
{
    data[2] = channel;
}
 
/**
 * \brief    Set the sensor ID
 * \param    data       Oregon message
 * \param    ID         Sensor unique ID
 */
inline void setId(byte *data, byte ID) 
{
  data[3] = ID;
}
 
/**
 * \brief    Set the sensor battery level
 * \param    data       Oregon message
 * \param    level      Battery level (0 = low, 1 = high)
 */
void setBatteryLevel(byte *data, byte level)
{
  if(!level) data[4] = 0x0C;
  else data[4] = 0x00;
}
 
/**
 * \brief    Set the sensor temperature
 * \param    data       Oregon message
 * \param    temp       the temperature
 */
void setTemperature(byte *data, float temp) 
{
  // Set temperature sign
  if(temp < 0)
  {
    data[6] = 0x08;
    temp *= -1;  
  }
  else
  {
    data[6] = 0x00;
  }
 
  // Determine decimal and float part
  int tempInt = (int)temp;
  int td = (int)(tempInt / 10);
  int tf = (int)round((float)((float)tempInt/10 - (float)td) * 10);
 
  int tempFloat =  (int)round((float)(temp - (float)tempInt) * 10);
 
  // Set temperature decimal part
  data[5] = (td << 4);
  data[5] |= tf;
 
  // Set temperature float part
  data[4] |= (tempFloat << 4);
}
 
/**
 * \brief    Set the sensor humidity
 * \param    data       Oregon message
 * \param    hum        the humidity
 */
void setHumidity(byte* data, byte hum)
{
    data[7] = (hum/10);
    data[6] |= (hum - data[7]*10) << 4;
}
 
/**
 * \brief    Sum data for checksum
 * \param    count      number of bit to sum
 * \param    data       Oregon message
 */
int Sum(byte count, const byte* data)
{
  int s = 0;
 
  for(byte i = 0; i<count;i++)
  {
    s += (data[i]&0xF0) >> 4;
    s += (data[i]&0xF);
  }
 
  if(int(count) != count)
    s += (data[count]&0xF0) >> 4;
 
  return s;
}
 
/**
 * \brief    Calculate checksum
 * \param    data       Oregon message
 */
void calculateAndSetChecksum(byte* data)
{
#ifdef TEMP_ONLY
    int s = ((Sum(6, data) + (data[6]&0xF) - 0xa) & 0xff);
 
    data[6] |=  (s&0x0F) << 4;     data[7] =  (s&0xF0) >> 4;
#else
    data[8] = ((Sum(8, data) - 0xa) & 0xFF);
#endif
}
 
/******************************************************************/
/******************************************************************/


// Fonction récupérant la température
// Retourne true si tout va bien, ou false en cas d'erreur
boolean getTemperature(float *temp){

#ifdef TEMP_ONLY  
  byte data[9], addr[8];
  // data : Données lues depuis le scratchpad
  // addr : adresse du module 1-Wire détecté


  if (!ds.search(addr)) { // Recherche un module 1-Wire
  ds.reset_search();    // Réinitialise la recherche de module
  return false;         // Retourne une erreur
  }

  if (OneWire::crc8(addr, 7) != addr[7]) // Vérifie que l'adresse a été correctement reçue
  return false;                        // Si le message est corrompu on retourne une erreur

  if (addr[0] != DS18B20) // Vérifie qu'il s'agit bien d'un DS18B20
  return false;         // Si ce n'est pas le cas on retourne une erreur

  ds.reset();             // On reset le bus 1-Wire
  ds.select(addr);        // On sélectionne le DS18B20

  ds.write(0x44, 1);      // On lance une prise de mesure de température
  delay(1000);             // Et on attend la fin de la mesure

  ds.reset();             // On reset le bus 1-Wire
  ds.select(addr);        // On sélectionne le DS18B20
  ds.write(0xBE);         // On envoie une demande de lecture du scratchpad

  for (byte i = 0; i < 9; i++) // On lit le scratchpad
  data[i] = ds.read();       // Et on stock les octets reçus

  // Calcul de la température en degré Celsius
  *temp = ((data[1] << 8) | data[0]) * 0.0625; 
  
  // Pas d'erreur
  return true;
#else
  *temp = dht.getTemperature();

  if (isnan(*temp)) { // Failed reading temperature from DHT
    return false;
  }
  else
  {
    // Pas d'erreur
    return true;
  }
#endif

}

 
/******************************************************************/
 
void setup()
{
  
 #ifdef PIR
	 pinMode(PIR_PIN, INPUT); 
	 mySwitch.enableTransmit(TX_PIN);
	 
	 PCMSK |= bit (PCINT0); 
	 GIFR |= bit (PCIF); // clear any outstanding interrupts
	 GIMSK |= bit (PCIE); // enable pin change interrupts 
	 sei();
#endif
 
 setup_watchdog(9);
 pinMode(TX_PIN, OUTPUT);	// sortie transmetteur

  SEND_LOW();  
 
#ifdef TEMP_ONLY  
  // Create the Oregon message for a temperature only sensor (TNHN132N)
  byte ID[] = {0xEA,0x4C};
#else
  // Create the Oregon message for a temperature/humidity sensor (THGR2228N)
  byte ID[] = {0x1A,0x2D};
  dht.setup(DATA_PIN);
#endif  
 
  setType(OregonMessageBuffer, ID);
  setChannel(OregonMessageBuffer, 0x20);
  setId(OregonMessageBuffer, NODE_ID);
}


// set system into the sleep state 
// system wakes up when wtchdog is timed out
void system_sleep() {
  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();

  sleep_mode();                        // System sleeps here

  sleep_disable();                     // System continues execution here when watchdog timed out 
  sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON
}

// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {

  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;

  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCR = bb;
  WDTCR |= _BV(WDIE);
}
  
// Watchdog Interrupt Service / is executed when watchdog timed out 
ISR(WDT_vect) {   
  count--;
} 

#ifdef PIR
  ISR (PCINT0_vect) 
  {
    Motion = digitalRead(PIR_PIN);
  }
#endif


//reads internal 1V1 reference against VCC
//return number 0 .. 1023 
int analogReadInternal() {
  ADMUX = _BV(MUX3) | _BV(MUX2); // For ATtiny85
  delay(5); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  uint8_t low = ADCL;
  return (ADCH << 8) | low; 
}

//calculate VCC based on internal referrence
//return voltage in mV
int readVCC() {
  return ((uint32_t)1024 * (uint32_t)1100) / analogReadInternal();
}


void loop()
{
  
  if (count <= 0) {	// on attend que le nombre de cycle soit atteint
    	
     count=WDT_COUNT;  // reset counter
          
           
      // Get Temperature, humidity and battery level from sensors
	  
      float temp; 
      
      if (getTemperature(&temp)) {
	  
	// Get the battery state
	int vcc = readVCC();

        lowBattery = vcc < LOW_BATTERY_LEVEL;
	        
        setBatteryLevel(OregonMessageBuffer, !lowBattery);	// 0=low, 1=high
        setTemperature(OregonMessageBuffer, temp);
       
        #ifndef TEMP_ONLY
            // Set Humidity
            float humidity = dht.getHumidity();
            if (isnan(humidity)) {
                
                setHumidity(OregonMessageBuffer, 52);	// Valeur par défaut en cas de lecture erronée
            }
            else
            {
                
                setHumidity(OregonMessageBuffer, humidity);
            }    
        #endif  
       
        // Calculate the checksum
        calculateAndSetChecksum(OregonMessageBuffer);
             
        // Send the Message over RF
        sendOregon(OregonMessageBuffer, sizeof(OregonMessageBuffer));
        // Send a "pause"
        SEND_LOW();
        delayMicroseconds(TWOTIME*8);
        // Send a copie of the first message. The v2.1 protocol send the message two time 
        sendOregon(OregonMessageBuffer, sizeof(OregonMessageBuffer));
        SEND_LOW();
      }
      else {
        
      }
        
    }
	
	#ifdef PIR
		if (Motion) {
	   
			mySwitch.switchOn(PIR_HOUSE_CODE, PIR_UNIT_CODE);
			delay(1000);
                        	   
		}
	#endif

  system_sleep();
}
