#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "DHTesp.h"
#include <ArduinoJson.h>
#include <ESP8266httpUpdate.h>

//--------------DEFINICION VARIABLES--------------------------
//Datos para actualización OTA:
#define HTTP_OTA_ADDRESS      F("172.16.53.138")    // Address of OTA update server
#define HTTP_OTA_PATH         F("/esp8266-ota/update")                // Path to update firmware
#define HTTP_OTA_PORT         1880                                    // Port of update server                                                          
#define HTTP_OTA_VERSION      String(__FILE__).substring(String(__FILE__).lastIndexOf('\\')+1) + ".nodemcu"  // Name of firmware

//Funciones para progreso de OTA
void progreso_OTA(int,int);
void final_OTA();
void inicio_OTA();
void error_OTA(int);
unsigned long lastAct=0;


//Datos sensor y estructura de datos
DHTesp dht;           // Objeto sensor
ADC_MODE(ADC_VCC);    // Función para poder medir la alimentación de la placa.

struct registro_datos {   //Definimos una estructura de datos para guardar todos los datos que queremos enviar
  unsigned long tiempo;   
  float temp;
  float hum;
  float bateria;
  int valor_led;
  bool valor_switch;
  char chipid[16];
  char* SSId;
  int32_t rssi;
  IPAddress ip;
};


// GPIOs 
int LED1 = 2;         //Pin del LED PWM
int LED2 = 16;        //Pin del LED interruptor
int LED_OTA = 2;      //Pin del LED usado durante la actualización FOTA
int SENSOR = 5;       //Pin de datos del sensor

//Variables de led:
char ID_PLACA[16];        // Cadenas para ID de la placa
int PWM_led;              // Variable que controla intensidad del LED1.
bool Switch_led;
int PWM_status;           //Variable que indica estado del LED1
int Switch_status;        // Variable que indica estado del LED2

//Variables de configuración
int vel_envio=30000;
int vel_fota;
int vel_PWM;
int LED_logica=0;
int SWITHC_logica=0;

//Datos para conectar a WIFI
const char* ssid = "infind";                // Usuario del punto de acceso.
const char* password = "1518wifi";          // Contraseña del punto de acceso.
const char* mqtt_server = "172.16.53.138";  // IP del equipo.


//Definimos el mensaje que se va a enviar:
  #define MSG_BUFFER_SIZE  (256)  // "#define" permite asignarle un nombre a una constante.
  char msg[MSG_BUFFER_SIZE];      // Mensaje que se muestra por el puerto serie.
  char msg_json[MSG_BUFFER_SIZE]; // Mensaje que se manda al broker MQTT.
  unsigned long lastMsg = 0;      // Tiempo en el que se envió el último mensaje.


// Creamos un cliente MQTT denominado "espClient".
WiFiClient espClient;
PubSubClient client(espClient);



//----------------FOTA-----------------------
void intenta_OTA()
{ 
  Serial.println( "--------------------------" );  
  Serial.println( "Comprobando actualización:" );
  Serial.print(HTTP_OTA_ADDRESS);Serial.print(":");Serial.print(HTTP_OTA_PORT);Serial.println(HTTP_OTA_PATH);
  Serial.println( "--------------------------" );  
  ESPhttpUpdate.setLedPin(LED_OTA, LOW);
  ESPhttpUpdate.onStart(inicio_OTA);
  ESPhttpUpdate.onError(error_OTA);
  ESPhttpUpdate.onProgress(progreso_OTA);
  ESPhttpUpdate.onEnd(final_OTA);
  WiFiClient wClient;
  switch(ESPhttpUpdate.update(wClient, HTTP_OTA_ADDRESS, HTTP_OTA_PORT, HTTP_OTA_PATH, HTTP_OTA_VERSION)) {
    case HTTP_UPDATE_FAILED:
      Serial.printf(" HTTP update failed: Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println(F(" El dispositivo ya está actualizado"));
      break;
    case HTTP_UPDATE_OK:
      Serial.println(F(" OK"));
      break;
    }
}

//----------------FUNCIONES FOTA-------------------------------------
void final_OTA()
{
  Serial.println("Fin OTA. Reiniciando...");
}

void inicio_OTA()
{
  Serial.println("Nuevo Firmware encontrado. Actualizando...");
}

void error_OTA(int e)
{
  char cadena[64];
  snprintf(cadena,64,"ERROR: %d",e);
  Serial.println(cadena);
}

void progreso_OTA(int x, int todo)
{
  char cadena[256];
  int progress=(int)((x*100)/todo);
  if(progress%10==0)
  {
    snprintf(cadena,256,"Progreso: %d%% - %dK de %dK",progress,x/1024,todo/1024);
    Serial.println(cadena);
  }
}

//-------------------WIFI-----------------------

// En esta función intentamos conectarnos a la red WIFI definida al principio del código.
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Mientras no nos hayamos podido conectar vamos a poner un '.' cada 0.5 segundos.
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");   // Informa de que ya nos hemos podido conectar al wifi
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());     // Muestra la IP local
}

//-----------------------MQTT------------------------------

// En esta función intentamos conectarnos al broker MQTT
void reconnect() {
  // De primeras se ejecuta solo una vez el "while()".
  // Si sigue sin conectarse, se vuelve a ejecutar hasta que se reconecte.
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    // Generamos el identificador del cliente 
    String clientId = "ESP8266Client-";
    clientId += String(ID_PLACA);
    
    // Intento de conectarse
    sprintf(topic_conexion, "II12/ESP%s/conexion",ID_PLACA);
    snprintf (msg_on, MSG_BUFFER_SIZE, "{"CHIPID":"ESP1234","online":true}");
    snprintf (msg_off, MSG_BUFFER_SIZE, "{"CHIPID":"ESP1234","online":false}");
    
    if (client.connect(clientId.c_str(),topic_conexion,2,true,msg_off)) //Función connecto con LWM retenido
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("infind/GRUPO12/datos", "Conexion al topic exitosa");     // Publicamos los datos del sensor en "infind/GRUPO12/datos".
      client.publish("infind/GRUPO12/conexion", "Conexion al topic exitosa");  //Publicamos el estado de conexion en "infind/GRUPO12/conexion"
      
      // ... and resubscribe
      client.subscribe("infind/GRUPO12/led/cmd");  // Nos suscribimos al topic "infind/GRUPO12/led/cmd" para el control del LED.

      client.publish(topic_conexion,msg_on ,true);       //Publicamos el estado de la conexión, mensaje retenidp
        
    } else {           
      //En caso de no conseguir conectarse se muestra en el puerto serie los mensajes de fallo, se reintenta la conexión
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
      }
  }
}

//-------------------PROCESAR MENSAJES----------------------------
void procesa_mensaje(char* topic, byte* payload, unsigned int length) {
  
  char* mensaje = (char*)malloc(length+1);  // reservo memoria para copia del mensaje
  strncpy(mensaje, (char*)payload, length); // copio el mensaje en cadena de caracteres
  mensaje[length]='\0';                     // caracter cero marca el final de la cadena

        

  // Mostramos por pantalla el topic recibido.
  Serial.printf("Mensaje recibido [%s] %s\n", topic, mensaje);

  // MENSAJE: Configuración
  String topic_config="II12/ESP"+String(ID_PLACA)+"/config";
  if(strcmp(topic, char(topic_config))==0)   // Compara si el mensaje que llega es del mismo topic que el indicado.
  {
      StaticJsonDocument<128> root; // el tamaño del buffer tiene que ser el adecuado
      // Deserialize the JSON document
      DeserializationError error = deserializeJson(root, mensaje,length);

      // Compruebo si no hubo error
      if (error) {
      Serial.print("Error deserializeJson() failed: ");   // Si devuelve un OK no ha habido error.
      Serial.println(error.c_str());
      }
      else
      if(root.containsKey("envia"))  // comprobar si existe el campo/clave que estamos buscando
      { 
        vel_envio = root["envia"];          // Lo que hay en el campo "envia" del payload se guarda en la variable "vel_encio".
        vel_fota = root["actualiza"];       // Velocidad actualización FOTA (min)
        vel_PWM = root["velocidad"];        // Velocidad de incremento PWM
        LED_logica = root["LED"];          //Logica del LED1
        SWITCH_logica = root["SWITCH"];    //Logica del LED2
        
        Serial.printf("Mensaje OK, nueva configuración. Velocidad de publicación %d, velocidad de actualización %d, velocidad PWM %d, logicad del led PWM %d y logica del SWITCH %s",vel_envio,vel_fota,LED_logica,SWITCH_logica);

      }
    
  }
  
  // MENSAJE: Control LED PWM
  String topic_led_cmd="II12/ESP"+String(ID_PLACA)+"/led/cmd";
  if(strcmp(topic, char(topic_led_cmd))==0)   // Compara si el mensaje que llega es del mismo topic que el indicado.
  {
    StaticJsonDocument<32> root; // el tamaño del buffer tiene que ser el adecuado
    // Deserialize the JSON document
    DeserializationError error = deserializeJson(root, mensaje,length);

    // Compruebo si no hubo error
    if (error) {
      Serial.print("Error deserializeJson() failed: ");   // Si devuelve un OK no ha habido error.
      Serial.println(error.c_str());
    }
    
    else
      if(root.containsKey("level"))  // comprobar si existe el campo/clave que estamos buscando
      { 
        PWM_led = root["level"];    // Lo que hay en el campo "level" del payload se guarda en la variable "valor_led".
        Serial.print("Mensaje OK, nivel de intensidad = ");
        Serial.println(PWM_led);

        int PWM;
        if(LED_logica==0){                     // Pasamos de escala 100-0 a 255-0.
        PWM = 255 - PWM_led*(255/100);        // A 255 le restamos "valor_led*(255/100)" para darle la vuelta a la escala.
        }                                      
        else{
        PWM=PWM_led*(255/100);                // 
        }
      }
        
        analogWrite(LED2,PWM);    // Escribimos en el LED2 el valor de PWM.

        // Cuando cambie el valor de la intensidad del led, enviamos un mensaje.
        if(PWM_led != PWM_status)
        {
          // Información del LED
          snprintf (msg, MSG_BUFFER_SIZE, "{\"CHIPID\":%s\"LED\": %d}",ID_PLACA, PWM_led);
          Serial.print("Mensaje de confirmación de intensidad: ");
          Serial.println(msg);
          
          String topic_led = "II12/ESP"+String(ID_PLACA)+"/led/status";
          
          client.publish(char(topic_led), msg);

          PWM_status = PWM_led;
        }
      else
      {
        Serial.print("Error : ");
        Serial.println("\"level\" key not found in JSON");
      }
    }

  //MENSAJE: Control LED Switch
  sprintf(topic_switch, "II12/ESP%s/swithc/cmd",ID_PLACA);
 
  else if(strcmp(topic, char(topic_switch))==0)   // Compara si el mensaje que llega es del mismo topic que el indicado.
  {
 
    StaticJsonDocument<64> root; // el tamaño del buffer tiene que ser el adecuado
    
    // Deserialize the JSON document
    DeserializationError error = deserializeJson(root, mensaje,length);

    // Compruebo si no hubo error
    if (error) {
      Serial.print("Error deserializeJson() failed: ");   // Si devuelve un OK no ha habido error.
      Serial.println(error.c_str());
    }

    else
      if(root.containsKey("level"))  // comprobar si existe el campo/clave que estamos buscando
      { 
        Switch_led = root["level"];    // Lo que hay en el campo "level" del payload se guarda en la variable "valor_led".
        Serial.print("Mensaje OK, valor del switch = ");
        Serial.println(PWM_led);

        bool SWITCH;
        if(SWITCH_logica==0){
        SWITCH = not(Switch_led); 
        }
        else{
        SWITCH = Switch_led);
        }
      }                                        
                                               
        digitalWrite(LED2,SWITCH);    // Escribimos en el LED2 el valor de PWM.

        // Cuando cambie el valor de la intensidad del led, enviamos un mensaje.
        if(Switch_led != Switch_status)
        {
          // Información del LED
          snprintf (msg, MSG_BUFFER_SIZE, "{\"LED\": %d}", Switch_led);
          Serial.print("Mensaje de confirmación del switch: ");
          Serial.println(msg);

          String topic_led = "II12/ESP"+String(ID_PLACA)+"/switch/status";
          client.publish(char(topic_led), msg);

          Switch_status = Switch_led;
        }
      else
      {
        Serial.print("Error : ");
        Serial.println("\"level\" key not found in JSON");
      }
  }

  
  else
  {
    Serial.println("Error: Topic desconocido");
  }

  free(mensaje);    // Para liberar memoria.
}

//-------------SERIALIZA JSON----------------
      
char serializa_JSON (char* payload)
{
  StaticJsonDocument<300> jsonRoot;
  char jsonString;
 
  jsonRoot= payload;
  
  serializeJson(jsonRoot,jsonString);
  return jsonString;
}

//-------------SERIALIZA SPRINTF-------------------------------

void serializa_sprintf(struct registro_datos datos, char *cadena, int size)
{
  snprintf(cadena,size,"{\"CHIPID\":%s\"Uptime\": %u, \"Vcc\": %f, \"DHT11\": {\"Temperatura\": %f, \"Humedad\": %f }, \"LED\": %d, \"SWITCH\": %b, \"Wifi\": {\"SSID\": \"%s\", \"IP\": \"%s\", \"RSSI\": %d }}",
                        datos.chipid, datos.tiempo, datos.bateria, datos.temp, datos.hum, datos.valor_led, datos.valor_switch, datos.SSId, datos.ip.toString().c_str(), datos.rssi);
}

//-----------------SETUP-------------------------------

void setup() {
  
  pinMode(LED1, OUTPUT);      // Definimos el pin LED1 como un Output. Tambien se corresponde con LED_OTA
  pinMode(LED2, OUTPUT);      // Definimos el pin LED" como un Output

  digitalWrite(LED1, HIGH);   // Inicialmente el LED1 está apagado. Tambien se corresponde con LED_OTA
  digitalWrite(LED2, HIGH);   // Inicialmente el LED2 está apagado.
  
  Serial.begin(115200);       // Establecemos la velocidad del puerto serie
  
  sprintf(ID_PLACA, "ESP%d", ESP.getChipId()); // Función obtener ID de la placa
  
  dht.setup(SENSOR, DHTesp::DHT11);      // Set el pin SENSOR que se conecta al sensor DHT11 
  
  setup_wifi();     //Llamamos a la función que se conecta al wifi
  intenta_OTA();    //Llamamos a la función que comprueba si hay actualizaciones OTA
  
  client.setServer(mqtt_server, 1883);
  client.setCallback(procesa_mensaje);
  client.setBufferSize(512);

}

//-------------------------MAIN-------------------------------

void loop() {

  // Definimos la estructura de datos del tipo registro_datos
  struct registro_datos misdatos;        

  // Guardamos en el campo 'tiempo' el tiempo actual en ms
  misdatos.tiempo = millis();
  
  // Si el cliente se desconecta, llamamos a a la función de reconectar.
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  //Si pasa el tiempo de actualización se comprueba:
   if (misdatos.tiempo - lastAct > vel_fota) {
    lastAct=misdatos.tiempo;
    intenta_OTA();    //Llamamos a la función que comprueba si hay actualizaciones OTA
   }
   
  // Como queremos actualizar los datos cada 30 s, lo ponemos de condición del if.
  if (misdatos.tiempo - lastMsg > vel_envio) {
    lastMsg = misdatos.tiempo;  // Actualizamos cuando se recibió el último mensaje.

    // Actualizamos el valor de los sensores y de la batería. EL ID de la placa tambien
    sprintf( misdatos.chipid, ID_PLACA);      // Identificador de la placa.    
    misdatos.hum = dht.getHumidity();         // Datos de humedad.
    misdatos.temp = dht.getTemperature();     // Datos de temperatura.
    misdatos.bateria = ESP.getVcc();          // Medimos la alimentación de la placa.

    // Actualizamos datos de conexión.
    sprintf( misdatos.SSId, ssid);           // Datos del ssid.
    misdatos.rssi = WiFi.RSSI();             // Datos de conexión de wifi.
    misdatos.ip = WiFi.localIP();            // Datos de dirección IP (convertidos a tipo string).

    // Actualizamos valores de los leds 
    misdatos.valor_led = PWM_led;
    misdatos.valor_switch = status_led;
    
    // Generamos el mensaje:
    serializa_sprintf(misdatos, msg, MSG_BUFFER_SIZE);

    // Generamos el mensaje en JSON:
    Serial.println(serializa_JSON(msg));
    
    snprintf (msg_json, MSG_BUFFER_SIZE,serializa_JSON(msg));
    Serial.print("Datos: ");
    Serial.println(msg);

    sprintf(topic_datos, "II12/ESP%s/datos",ID_PLACA);
    client.publish(topic_datos, msg_json);   
  }

}
