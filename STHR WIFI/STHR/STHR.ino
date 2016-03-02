/*
  STHR.ino - v1.00 - 07/01/2015:
  - Version inicial
  
  Sketch para el m?dulo sensor de temperatura ambiental STHR
  Copyright (c) 2014 Raimundo Alfonso
  Ray Ingenier?a Electr?nica, S.L.
  
  Este sketch est? basado en software libre. Tu puedes redistribuir
  y/o modificarlo bajo los terminos de licencia GNU.

  Esta biblioteca se distribuye con la esperanza de que sea ?til,
  pero SIN NINGUNA GARANT?A, incluso sin la garant?a impl?cita de
  COMERCIALIZACI?N O PARA UN PROP?SITO PARTICULAR.
  Consulte los terminos de licencia GNU para m?s detalles.
  
  * CARACTERISTICAS GENERALES
  - Alimentaci?n a bater?as
  - Doble step-up 3.3V y 5V
  - Sensores disponibles temperatura, humedad, CO2 y presi?n atmosf?rica
  - Entradas digitales para comunicaci?n con estaci?n meteorol?gica
 

  * CARACTERISTICAS SENSOR TEMPERATURA DS18B20
  - Resoluci?n temperatura: 0.1?C
  - Rango de medida: -40?C +85?C
  - Precisi?n t?pica: +/- 0.1?C
  - Precisi?n m?xima: +/- 1?C
  - IP66

  * CARACTERISTICAS SENSOR TEMPERATURA Y HUMEDAD DHT22
  - Resoluci?n temperatura: 0.1?C
  - Rango de medida: -40?C +80?C
  - Precisi?n t?pica: +/- 0.5?C
  - Resoluci?n humedad: 1%RH
  - Rango humedad: 0 - 100%RH
  - Precisi?n t?pica a 25?C: +/- 3%RH

  * CARACTERISTICAS SENSOR CO2 POR INFRARROJOS CDM8S (sensor de CO2 por defecto)
  - Rango: 400 - 10000ppm
  - Precisi?n: +/-3%



  -------------------------------------------------------------------------------
  KEY	DESCRIPTION				UNITS		        FACTOR
  -------------------------------------------------------------------------------
  1 	temperature  			?C					x10
  2 	humidity				%RH					x10
  3 	CO2 concentration		ppm					x1
  4     battery                 V                   x100
  
*/

//#define SENSOR_DS18B20	// Sensor temperatura solo
#define SENSOR_DHT22		// Sensor temperatura y humedad
//#define SENSOR_CO2S8		// Sensor CO2 infrarrojos

// Coloca aquí el SSID y PASS de tu router wifi:
#define WIFI_SSID     "AT+CWJAP=\"SSID_WIFI\",\"PASS_WIFI\""
#define APIKEY        "APIKEY"
#define NODE          "1"

const int time_between_readings     = 10;     // in minutes
const int time_between_readings_co2 = 30;     // in minutes



const int TEMPERATURE_PRECISION = 11;        // 9 (93.8ms),10 (187.5ms) ,11 (375ms) or 12 (750ms) bits equal to resplution of 0.5C, 0.25C, 0.125C and 0.0625C
#define ASYNC_DELAY 375                      // 9bit requres 95ms, 10bit 187ms, 11bit 375ms and 12bit resolution takes 750ms

// See block comment above for library info
#include <avr/power.h>
#include <avr/sleep.h>       
#include <RFu_JeeLib.h>                                     
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DHT.h"
#include <SoftwareSerial.h>

// Hardwired STHR pin allocations
#define CDM8S_TX       14
#define CDM8S_RX       2
#define CDM8S_PWR      7
#define XBEE_SLEEP     10
#define DHT22_PWR      6
#define DS18B20_PWR    5 
#define LED            9
#define BATT_ADC       A6
#define ONE_WIRE_BUS   19
#define DHTPIN         18 
#define SHDN_5V        15
#define P_RAIN         8
#define P_ANEMOMETER   2
#define WIND_VANE      A7
#define ESP8862_PD     10


// Humidity code adapted from ladyada' example                        // emonTh DHT22 data pin
// Uncomment whatever type you're using!
// #define DHTTYPE DHT11   
#define DHTTYPE DHT22   
DHT dht(DHTPIN, DHTTYPE);
boolean DHT22_status;                                                 // create flag variable to store presence of DS18B20

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
boolean DS18B20;                                                      // create flag variable to store presence of DS18B20 

typedef struct {                                                      // RFM12B RF payload datastructure
#ifdef SENSOR_DS18B20
  int temperature;
#endif 
#ifdef SENSOR_DHT22
  int temperature;
  int humidity;    
#endif  
#ifdef SENSOR_CO2S8
  int co2;
#endif
  int battery;
} Payload;
Payload sthr;

SoftwareSerial cdm(CDM8S_RX, CDM8S_TX); // RX, TX
//SoftwareSerial cdm(11, 12); // RX, TX

char buffer_rx[15];
byte cnt = 0;

//addresses of sensors, MAX 4!!  
byte allAddress [4][8];                                              // 8 bytes per address
int numSensors; 


ISR(WDT_vect) {
  Sleepy::watchdogEvent();
}                            // Attached JeeLib sleep function to Atmega328 watchdog -enables MCU to be put into sleep mode inbetween readings to reduce power consumption 


//################################################################################################################################
//################################################################################################################################
void setup() {
//################################################################################################################################
  
  byte n;
  
  pinMode(LED, OUTPUT);
  pinMode(DHT22_PWR, OUTPUT);
  pinMode(DS18B20_PWR, OUTPUT);
  pinMode(SHDN_5V, OUTPUT);
  digitalWrite(SHDN_5V, LOW);
  pinMode(CDM8S_PWR, INPUT);
  pinMode(BATT_ADC, INPUT);
  pinMode(CDM8S_RX, INPUT);
  pinMode(CDM8S_TX, INPUT);
  pinMode(XBEE_SLEEP, OUTPUT);
  pinMode(16, INPUT_PULLUP);
  pinMode(17, INPUT_PULLUP);  
  pinMode(P_RAIN, INPUT);
  pinMode(ESP8862_PD, OUTPUT);
  digitalWrite(LED, HIGH);       
  digitalWrite(ESP8862_PD, HIGH);                
  digitalWrite(DHT22_PWR, LOW);  
  digitalWrite(DS18B20_PWR, LOW);

  pinMode(16, OUTPUT);
  digitalWrite(16, LOW); 

  Serial.begin(9600);

 
  //################################################################################################################################
  // Power Save  - turn off what we don't need - http://www.nongnu.org/avr-libc/user-manual/group__avr__power.html
  //################################################################################################################################
  ACSR |= (1 << ACD);                     // disable Analog comparator    
  //if (debug==0) power_usart0_disable();   // disable serial UART
  power_twi_disable();                    // Disable the Two Wire Interface module.
  //power_usart0_disable();  
  //power_timer0_disable();               // don't disable necessary for the DS18B20 library
  power_timer1_disable();

//led_on();

#ifdef SENSOR_DHT22
  //################################################################################################################################
  // Test for presence of DHT22
  //################################################################################################################################
  digitalWrite(DHT22_PWR,HIGH);
  dodelay(2000);                                                        // wait 2s for DH22 to warm up
  dht.begin();
  float h = dht.readHumidity();                                         // Read Humidity
  float t = dht.readTemperature();                                      // Read Temperature
  digitalWrite(DHT22_PWR,LOW);                                          // Power down
  
  if (isnan(t) || isnan(h)){                                            // check if returns are valid, if they are NaN (not a number) then something went wrong!
    Sleepy::loseSomeTime(1500); 
    float h = dht.readHumidity();  float t = dht.readTemperature();
    if (isnan(t) || isnan(h)){
      DHT22_status=0;
    } 
  }else{
    DHT22_status=1;
  }    
  digitalWrite(DHT22_PWR,LOW);
  pinMode(DHTPIN, INPUT);  
#endif  

#ifdef SENSOR_DS18B20
  digitalWrite(DS18B20_PWR, HIGH);
  delay(50); 
  sensors.begin();
  sensors.setWaitForConversion(false);                             //disable automatic temperature conversion to reduce time spent awake, conversion will be implemented manually in sleeping http://harizanov.com/2013/07/optimizing-ds18b20-code-for-low-power-applications/ 
  numSensors=(sensors.getDeviceCount()); 
  
  byte j=0;                                        // search for one wire devices and
                                                   // copy to device address arrays.
  while ((j < numSensors) && (oneWire.search(allAddress[j])))  j++;
  digitalWrite(DS18B20_PWR, LOW);
  
  if (numSensors==0){
    DS18B20=0; 
  }else{
    DS18B20=1; 
  } 
#endif

#ifdef SENSOR_CO2S8
  read_co2();
#endif  

  //led(500);
  
} // end of setup




//################################################################################################################################
//################################################################################################################################
void loop()
//################################################################################################################################
{ 
  byte n;
  static int co2_reads = 0;



#ifdef SENSOR_DS18B20  
  if (DS18B20 == 1){
    digitalWrite(DS18B20_PWR, HIGH);
    dodelay(50);     
    for(int j=0;j<numSensors;j++) sensors.setResolution(allAddress[j], TEMPERATURE_PRECISION);      // and set the a to d conversion resolution of each.
    sensors.requestTemperatures();                                        // Send the command to get temperatures
    dodelay(ASYNC_DELAY); //Must wait for conversion, since we use ASYNC mode
    float temp=(sensors.getTempC(allAddress[0]));
    digitalWrite(DS18B20_PWR, LOW);    
    if ((temp<125.0) && (temp>-40.0)){
      if (DHT22_status==0) sthr.temperature=(temp*10);            // if DHT22 is not present assume DS18B20 is primary sensor (internal)
    }
  }
#endif
  
#ifdef SENSOR_DHT22  
  if (DHT22_status == 1){ 
    pinMode(DHTPIN, OUTPUT);
    digitalWrite(DHT22_PWR,HIGH);                                                                                                  // Send the command to get temperatures
    dodelay(2000);                                             //sleep for 1.5 - 2's to allow sensor to warm up
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    sthr.humidity = ((dht.readHumidity())*10);

    float temp=(dht.readTemperature());
    if ((temp<85.0) && (temp>-40.0)) sthr.temperature = (temp*10);
    digitalWrite(DHT22_PWR,LOW); 
    pinMode(DHTPIN, INPUT); 
  }
#endif

#ifdef SENSOR_CO2S8
  if(co2_reads >= time_between_readings_co2){
    co2_reads = 0;
    read_co2();  
  }
#endif  

  sthr.battery=int(analogRead(BATT_ADC)*0.3225806);                    //read battery voltage, convert ADC to volts x100    

 // send_data();
  power_usart0_enable(); 
  pinMode(16, OUTPUT);
  digitalWrite(16, LOW); 
  dodelay(200);
  digitalWrite(ESP8862_PD, HIGH);  
  dodelay(1000);  
  SetUpWIFI(); 


  delay(1000);

  digitalWrite(ESP8862_PD, LOW);
  delay(100);    
  pinMode(16, INPUT_PULLUP);
  power_usart0_disable();  

  for (int i=0; i<time_between_readings; i++){
   co2_reads++;
   dodelay(55000); //1 minute should be 60000 but is not because of variation of internal time source
    //caution parameter cannot be more than 65000, maybe find better solution
    //due to internal time source 60000 is longer than 1 minute. so 55s is used.
  }



}






void dodelay(unsigned int ms)
{
  byte oldADCSRA=ADCSRA;
  byte oldADCSRB=ADCSRB;
  byte oldADMUX=ADMUX;
 
  Sleepy::loseSomeTime(ms); // JeeLabs power save function: enter low power mode for x seconds (valid range 16-65000 ms)
      
  ADCSRA=oldADCSRA;         // restore ADC state
  ADCSRB=oldADCSRB;
  ADMUX=oldADMUX;
}

#ifdef SENSOR_CO2S8
void read_co2(void){
    int n;
    cdm.begin(9600);
    CDM8S_enable(); 
    dodelay(5000);
    // Pregunta por el sensor de CO2...
    for(n=0;n<=5;n++){    
      buffer_rx[0] = 0;
      cnt = 0;
      cdm.write(0xFE);
      cdm.write(0x04);
      cdm.write((byte)0x00);
      cdm.write(0x03);
      cdm.write((byte)0x00);
      cdm.write(0x01);  
      cdm.write(0xD5);
      cdm.write(0xC5);    
      delay(100); 
      if(compruebaCDM()) break;    
    }
    CDM8S_disable();  
    cdm.end();
}

boolean compruebaCDM(void){
  boolean stat = false;
  while(cdm.available()){ 
    buffer_rx[cnt++] = cdm.read();
    if(cnt >= 7){
      sthr.co2 = word(buffer_rx[3], buffer_rx[4]);
      buffer_rx[0] = 0;
      cnt = 0;
      stat = true;   
    }
    delay(2);
  }
  return(stat);
}
#endif

void led(int tiempo){
  digitalWrite(LED, LOW);
  dodelay(tiempo);
  digitalWrite(LED, HIGH);   
}

void led_on(void){
  digitalWrite(LED, LOW);
}

void led_off(void){
  digitalWrite(LED, HIGH);
}


void CDM8S_enable(void){
  pinMode(SHDN_5V, INPUT);
  delay(50);  
  pinMode(17, OUTPUT);
  digitalWrite(17,LOW);
  delay(100);  
  pinMode(CDM8S_PWR, OUTPUT);
  digitalWrite(CDM8S_PWR, LOW);
  pinMode(CDM8S_TX, OUTPUT);  
}  
 
void CDM8S_disable(void){
  pinMode(CDM8S_RX, INPUT);
  pinMode(CDM8S_TX, INPUT);  
  pinMode(CDM8S_PWR, INPUT);
  pinMode(SHDN_5V, OUTPUT);
  digitalWrite(SHDN_5V, LOW);  
  pinMode(17, INPUT_PULLUP);  
  //delay(100);  
} 


String GetLineWIFI(){
  String S = "" ;
  long temp;
  temp = millis();
  while (Serial.available()){
    char c = Serial.read();
    if ( c != '\n' ){            //Hasta que el caracter sea intro
      S = S + c ;
      delay(25) ;
    }else{
      break;
    }  
  }
  return( S ) ;
}
   
void SetUpWIFI(){
  char c[10];
  long T;

  Serial.flush();
  String GET = "GET /emoncms/input/post.json?apikey=";  
  GET += APIKEY;
  GET += "&node=";
  GET += NODE;
  GET += "&csv=";
  itoa(sthr.temperature, c, 10);
  GET += c;
  GET += ",";
  itoa(sthr.humidity, c, 10);
  GET += c;
  GET += ",";  
#ifdef SENSOR_CO2S8
  itoa(sthr.co2, c, 10);
  GET += c;
  GET += ","; 
#endif 
  itoa(sthr.battery, c, 10);
  GET += c;
  GET +="HTTP/1.1\r\n";

  String ordenes[]=
    {  //"AT+RST",
       "AT+CWMODE=3",
       WIFI_SSID,
       "AT+CIFSR" ,
       "AT+CIPMUX=1",
       "AT+CIPSTART=1,\"TCP\",\"emoncms.org\",80",
       "END"        // Para reconocer el fin de los comandos AT
    };
    int index = 0;
    while(ordenes[index] != "END")
        {  Serial.println(ordenes[index++]);
           T = millis(); 
           while(true){
                 String s = GetLineWIFI();
                 if ( s.startsWith("no change"))   
                         break;
                 if ( s.startsWith("OK"))   
                         break;
                 if ( s.startsWith("ready"))   
                         break;
                 if ( s.startsWith("ALREAY CONNECT"))   
                         break;                         
                 if (millis()-T > 3000) break;  
             }
      } 
      Serial.print("AT+CIPSEND=1,");
      Serial.println(GET.length());
      delay(100);

  if(Serial.find( ">" ))  {
     Serial.println(GET);
  }else{
    Serial.println( "AT+CIPCLOSE=0" );//close TCP connection
  }  
}


