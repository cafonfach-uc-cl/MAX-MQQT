// Librerias utilizadas 

//This library allows you to communicate with I2C  devices
#include <Wire.h> 
//An Arduino Library for the MAX3015 particle sensor and MAX30102 Pulse Ox sensor
#include "MAX30105.h" 
#include "spo2_algorithm.h" 
//
#include <Client.h> 
//
#include <CooperativeMultitasking.h> 
//
#include "MQTTClient.h" 
// this library allows an Arduino board to connect to the internet.
#include "WiFi.h" 
//
#include <WiFiUdp.h> 
//JSON serialization
#include <ArduinoJson.hpp>
#include <ArduinoJson.h> 


#include <PubSubClient.h>
#include <WiFiClient.h>
#include <WiFiMulti.h> 


// Conexion Wifi
#ifndef STASSID
#define STASSID "AndroidAP8483"
#define STAPSK  "simonetilleria"
#endif

const char* ssid     = STASSID;
const char* password = STAPSK;

WiFiMulti WiFiMulti;


//INICIALIZA MAX30102  ( https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library/blob/master/examples/Example8_SPO2/Example8_SPO2.ino)
//-----------------------------------------------------------------------------------------------------------
MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

#if defined(_AVR_ATmega328P) || defined(AVR_ATmega168_) 

//Las muestras se convierten en datos de 16 bits, para disminuir uso de SRAM 
//se truncará el MSB de 16 bits de los datos muestreados. 

uint16_t irBuffer[100]; // LED infrarojo sensor data
uint16_t redBuffer[100];  // 
#else

uint32_t irBuffer[100]; // LED infrarojo sensor data
uint32_t redBuffer[100];  // LED rojo sensor data
#endif


int32_t bufferLength; //data length 

int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid 

int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

//byte pulseLED = 11; //Must be on PWM pin
//byte readLED = 13; //Blinks with each data read


//---------------------------------------------------------------------------------------------------------



/*
//INICIALIZA LCD (https://www.circuitbasics.com/how-to-set-up-a-keypad-on-an-arduino/)
//---------------------------------------------------------------

const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns 
char LCD[5];
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};


// For ESP32 Microcontroller, definir pines
byte rowPins[ROWS] = {23, 16, 2, 26}; 
byte colPins[COLS] = {19, 18, 5, 17};

// Create the Keypad
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS); 

//Función constructor, crea un objeto de la clase LiquidCrystal_I2C, con dirección, columnas y filas indicadas.
LiquidCrystal_I2C lcd(0x27, 20, 4); 

//Variables para guardar la hora
const int len_key = 5;
char attempt_key[len_key]; 

//Variables int
int z=0;
int y=-1;
int k=0;

//------------------------------------------------------------------- 

*/

//MQTT Settings
char host[] = "broker.mqttdashboard.com"; 
char clientid[] = "...";
char username[] = "..."; 
  
const int mqttPort = 1883; 
const char* mqttUser =  "...";
const char* mqttPassword =  "...";

char topicname[] = "HR"; // Topic 
char topicname_2[] = "SPO2"; // Topic 
char topicname_3[] = "Pantalla"; // Topic



//-------------------------------------------------------------------------------------------------------
CooperativeMultitasking tasks;


// Iniciamos el cliente udp para su uso con el server NTP
WiFiUDP ntpUDP;

// Funciones propias 
//-----------------------------------------------------------------------------------------------------
void wifi_init(void);
void reconnect();

/*
//Esta función de manejo se ejecutará cuando se reciba un mensaje MQTT sobre un tema suscrito. 
//nombre del tema, la carga útil (en bytes) y la longitud del mensaje recibido. 
//Now if we want to receive messages we also need to create a  function 
//---------------------------------------------------------------------------------
void callback(char* topic, byte* payload, unsigned int length) {
  
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
 
  Serial.print("Message:");
  int c = 0;
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    c = c + 1;
  }
  Serial.println(String(c));
  Serial.println("");
  
  Serial.println();
  Serial.println("-----------------------");
 
} 

//-------------------------------------------------------------------------------------


*/

//
WiFiClient wificlient;
//PubSubClient client(host,1883, callback, wificlient);  
//Una instancia del objeto cliente
PubSubClient client(wificlient);


/*
 * Codigo principal 
 * 
 */
void setup() {
  
  // Monitor Serial 
  Serial.begin(115200);

  // Inicializacion del Wifi
  Serial.println("Iniciando WiFi...");
  wifi_init();
  

  // Para subcripcion 
  //port = broker port default is 1883 
  //Now we need to create a connection to a broker.
  client.setServer(host, 1883); 
  
  //client.setCallback(callback);  


  
  
  //------------------------------------------------------------------ 
  //Mientras no esta conectado
  while (!client.connected()) 
  {
    Serial.println("Connecting to MQTT..."); 
    
    //Si la conexión fue exitosa o no
    if (client.connect("ESPClient", mqttUser, mqttPassword )) 
    {
      Serial.println("Connected"); 
    }
    else 
    {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
  
  //------------------------------------------------------------------ 


  
  //Inicialización MAX 8( https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library/blob/master/examples/Example8_SPO2/Example8_SPO2.ino))
  //pinMode(pulseLED, OUTPUT);
  //pinMode(readLED, OUTPUT);

  //Use default I2C port, 400kHz speed
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) 
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  Serial.available() == 1 ; 
  Serial.read();

 //Configuraciones de mediciones
 //La corriente del controlador para el LED rojo y el LED IR del módulo se puede programar por separado de 0 a 50 mA. 
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  
  //Configure sensor with these settings 
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); 
  //------------------------------------------------------------------------ 

  
/*
  
  //INICIALIZA LCD 
  //-----------------------------------------------------------------------
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0,0);
    lcd.print("Hora comida:");

*/
  //To subscribe to a topic 
  client.subscribe(topicname);
  client.subscribe(topicname_2); 
  client.subscribe(topicname_3);
 }

 

void loop() {
  int i = 0;
  Serial.println("");

  //------------------------------------------------------------------------------------------------------------------
  //Leer sensor MAX 
  bufferLength = 100; //25 muestras en un segundo, por 4 segundos

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  { 
    //do we have new data?
    while (particleSensor.available() == false) 
    //Check the sensor for new data
      particleSensor.check(); 

    //Mide luz roja(HIPO) e infreroja (OX)
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR(); 

    //We're finished with this sample so move to next sample
    particleSensor.nextSample(); 

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  //4 segundos, despues de las primeras 100 muestras, calcular la frecuencia cardíaca y la SpO2 
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second 

  //-----------------------------------------------------------------------------------------------------------------

  /*
  //LCD 
  //-------------------------------------------------------------------------------------------------------------

  //Reporta el valor ASCII de una tecla presionada
  char key = keypad.getKey();
  lcd.setCursor(0,1);
  if (key){  
    //guardar la tecla presionada
    LCD[k]=key;
    k = k + 1;
    attempt_key[y]= key; 
    //Mueve el cursos
    lcd.setCursor(1+y,1); 
    //Lo muestra en la LCD
    lcd.print(key);
    y = y + 1; 

    //Para poner los dos puntos
    if (y == 1){
      lcd.setCursor(2,1);
      lcd.print(":"); 
      LCD[2]=':' ;
      k = k + 1;
      y = y + 1;
    } 

    //Reinicie char de hora guardada
    if (k == 5)
    {
     k = 0;
    } 
  }
      
   
  //---------------------------------------------------------------
      */

    
 //-----------------------------------------------------------------------------------------------


     
    //ignora las primeras 25 muestras para medicion mas limpia
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      //digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR(); 
      //We're finished with this sample so move to next sample
      particleSensor.nextSample();  

     
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);

      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);

      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);

      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);
    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate); 

   //-----------------------------------------------------------------------------------------------------------------------------------
  
  
  Serial.println("");
  // Construir el json 
  //------------------------------------------------------------
  char json[256]; 
  //capacity of a StaticJsonDocument in a template parameter
  StaticJsonDocument<300> doc;
  doc["sender"] = "Signos Vitales:";
  //doc["SPO2"] = spo2; 
  //doc["HR"] = HR; 
  doc["Pantalla"] = LCD;

  //tamaño json
  size_t n = serializeJson(doc, json);
  Serial.println(json);
  Serial.println("");  
  

  //--------------------------------------------------------------
  
  if (!client.connected()) 
  {
    reconnect();
  } 
  

  //--------------------------------------------------------------
  //Publicar al topico
  client.publish(topicname,(uint8_t*) json,(int) n,true); 
  client.publish(topicname_2,(uint8_t*) json,(int) n,true); 
  client.publish(topicname_3,(uint8_t*) json,(int) n,true);
  

  //----------------------------------------------------------------
  if (!client.connected()) 
  {
    reconnect();
  }

  
  i = 0;

  //-------------------------------------------------------------------
 }
 




//-------------------------------------------------------------------
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect 
    if (client.connect("ESPClient")) {
      Serial.println("connected");

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }

  
  client.subscribe(topicname); 
  client.subscribe(topicname_2); 
  client.subscribe(topicname_3);
}

//-----------------------------------------------------------------------------------------




//---------------------------------------------------------------------------------------- 

//https://www.luisllamas.es/como-conectar-un-esp8266-a-una-red-wifi-modo-sta/
void wifi_init(void) (
{
  // We start by connecting to a WiFi network
  WiFi.mode(WIFI_STA); 
  //agregar varias redes WiFI proporcionando sus respectivas SSID y su contraseña. 
  WiFiMulti.addAP(ssid, password);

  Serial.println();
  Serial.println();
  Serial.print("Wait for WiFi... ");

  while (WiFiMulti.run() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  delay(500);
}
