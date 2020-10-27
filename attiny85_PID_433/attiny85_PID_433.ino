/*
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
 * V1 par vil1driver
 * 
 * PID heating control
 * get temp from ds18b20
 * heater control by rf transmission to DI.O wallplug
 *
*/

/************************************************************

    emplacement des PIN de la puce ATtiny8
    
                +-------+
Ain0  D5  PB5  1|*      |8   VCC
Ain3  D3  PB3  2|       |7   PB2  D2  Ain1
Ain2  D4  PB4  3|       |6   PB1  D1  pwm1
          GND  4|       |5   PB0  D0  pwm0
                +-------+ 


            cablage a realiser
                
                +-------+
               1|*      |8   (+)
  Data Sonde   2|       |7
      TX 433   3|       |6
         (-)   4|       |5
                +-------+ 

                              
****************      Confuguration     *******************/


#define DATA_PIN 3                // pin 2 // data de la sonde
#define TX_PIN 4                  // pin 3 // data transmetteur

// coeffs pid
// parents : 87 26 95
// thomas : 76 12 116
// salon : 95 16 143
const int Kp = 95;  // coefficient proportionnelle
const int Ki = 16;  // coefficient integrale
const int Kd = 143; // coefficient dérivée

// consignes températures
// parents 19.0
// thomas 19.5
// salon 20.0
const float consigne = 20.0;

#define CYCLE 75 // cycle de 10 minute ( 75 * 8s)
volatile int cycleCount = 0;

// codes Chacon DI.O
// parents 2345678
// thomas 1234567
// salon 3456789
const unsigned long remoteID = 3456789;
const byte unitID = 1;

/***************  Fin de configuration   *****************/

float temp;
float tmp[4] = {0.0, 0.0, 0.0, 0.0};
float somErrI = 1.0;
int heatTime;
boolean heat;

// Chargement des librairies
#include <avr/sleep.h>    // Sleep Modes
#include <avr/wdt.h>      // Watchdog timer
#include <avr/interrupt.h>

#include "NewRemoteTransmitter.h" // https://github.com/mattwire/arduino-dev/tree/master/libraries/NewRemoteSwitch
NewRemoteTransmitter transmitter(remoteID, TX_PIN, 249, 3);

#include "OneWire.h"
#define DS18B20 0x28     // Adresse 1-Wire du DS18B20
OneWire ds(DATA_PIN); // Création de l'objet OneWire ds

// Routines to set and claer bits (used in the sleep code)
#ifndef cbi
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Fonction récupérant la température
// Retourne true si tout va bien, ou false en cas d'erreur
boolean getTemperature(float *temp){
  byte present = 0;
  byte data[9];
  byte addr[8];
  // data : Données lues depuis le scratchpad
  // addr : adresse du module 1-Wire détecté
  if (!ds.search(addr)) { // Recherche un module 1-Wire
    ds.reset_search();    // Réinitialise la recherche de module
    delay(250);
  //return false;         // Retourne une erreur
  }
  if (OneWire::crc8(addr, 7) != addr[7]) // Vérifie que l'adresse a été correctement reçue
    return false;                        // Si le message est corrompu on retourne une erreur
  if (addr[0] != DS18B20) // Vérifie qu'il s'agit bien d'un DS18B20
    return false;         // Si ce n'est pas le cas on retourne une erreur
  ds.reset();             // On reset le bus 1-Wire
  ds.select(addr);        // On sélectionne le DS18B20
  ds.write(0x44, 1);      // On lance une prise de mesure de température
  delay(1000);             // Et on attend la fin de la mesure
  present = ds.reset();             // On reset le bus 1-Wire
  ds.select(addr);        // On sélectionne le DS18B20
  ds.write(0xBE);         // On envoie une demande de lecture du scratchpad
  for (byte i = 0; i < 9; i++) // On lit le scratchpad
    data[i] = ds.read();       // Et on stock les octets reçus
  // Calcul de la température en degré Celsius
  *temp = ((data[1] << 8) | data[0]) * 0.0625; 
  // Pas d'erreur
  return true;
}

void setup()
{
 CLKPR = (1<<CLKPCE);  
 CLKPR = B00000000;  // set the fuses to 8mhz clock-speed.
 setup_watchdog(9);
 pinMode(TX_PIN, OUTPUT); // sortie transmetteur
 digitalWrite(TX_PIN, LOW); // sendLow
 delay(1000);
 transmitter.sendUnit(unitID, true);  // appairage avec la prise DI.O
 delay(1000);
 getTemperature(&temp);
 tmp[0], tmp[1], tmp[2], tmp[3] = temp;
}

// set system into the sleep state 
// system wakes up when wtchdog is timed out
void sleep() {
  cbi(ADCSRA, ADEN); // disable adc
  cbi(ADCSRA, ADSC); // stop conversion
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();                        // Go to sleep
  sbi( ADCSRA, ADEN );  // enable adc
  sbi( ADCSRA, ADSC );  // restart conversion
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
  //wake up
  cycleCount--;
} 

void loop()
{
  compute();
  sleep();
}

void compute()
{
  // toutes les 10 min
  if (cycleCount <= 0)
  {
    cycleCount = CYCLE;  // reset counter
      
    // récupération température (lecture sonde)
    if (getTemperature(&temp))
    {

      // décallage dans table temps (suppression de la plus ancienne mesure et ajout de la nouvelle)
      //memmove(tmp, tmp+1 , 3*sizeof *tmp);
      tmp[0] = tmp[1];
      tmp[1] = tmp[2];
      tmp[2] = tmp[3];
      tmp[3] = temp;
      // calcule erreur
      float err = consigne - temp;
      // calcule somme erreurs
      float somErr = 4 * consigne - tmp[0] - tmp[1] - tmp[2] - tmp[3];
      
      if (abs(somErr) < 2) {
        somErr = somErrI;
        if (abs(err) > 0.1) {
          somErr, somErrI = constrain(somErr + err / 2, 0, 5);
        }
      }
      // calcule moyenne erreur glissante
      float moyErr = ((consigne - tmp[0]) + (consigne - tmp[1]) * 8 + (consigne - tmp[2]) * 27 + (consigne - tmp[3]) * 64) / 100;
      // clacule dérivée
      float deriv = (4 * (4 * tmp[0] + 3 * tmp[1] + 2 * tmp[2] + tmp[3]) - 10 * (tmp[0] + tmp[1] + tmp[2] + tmp[3])) / 20;
      // calcule temp de mise en route en secondes (pour un cycle de 10min)
      heatTime = round(constrain(Kp * moyErr + Ki * somErr + Kd * deriv, 0, 100) * 0.1 * 60);
      
      // définition des bords max 30s entre 2 ordres opposés
      if (heatTime >= 585) {
        heatTime = 600;
      }
      else if (heatTime >= 570) {
        heatTime = 570;
      }
      else if (heatTime >= 30) {
        heatTime = heatTime;
      }
      else if (heatTime >= 15) {
        heatTime = 30;
      }
      else {
        heatTime = 0;
      }
      
      // mise en route du chauffage (mini pour 30s)
      if (heatTime > 0) {
        heat = true;
        transmitter.sendUnit(unitID, true);;
      }
      else {
        transmitter.sendUnit(unitID, false);
      }
    }  
  }
  // arrêt chauffage
  else if (CYCLE - cycleCount >= round(heatTime / 8) and heat) {
    heat = false;
    transmitter.sendUnit(unitID, false);
  }
}
