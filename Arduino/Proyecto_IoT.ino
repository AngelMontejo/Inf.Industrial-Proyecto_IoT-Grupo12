#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "DHTesp.h"
#include <ESP8266httpUpdate.h>
#include <ArduinoJson.h>
#include "Button2.h"

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

struct registro_datos    //Definimos una estructura de datos para guardar todos los datos que queremos enviar
     {unsigned long tiempo;   
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
int PWM_status;           // Variable que indica estado del LED1
int valor_anterior=-1;    // Variable guarda valor PWM para funcion del boton
int Switch_status;        // Variable que indica estado del LED2

//Variables de configuración
int vel_envio=30;
int vel_fota=0;
int vel_PWM;
int LED_logica=0;
int SWITCH_logica=0;

//Variables botones
#define BUTTON_PIN 0    // Definimos el pin correspondiente al boton en la placa
Button2 button;         // Objeto de la cabecera button2.h
char tipo_pulsacion[12];   // Variable indica tipo de pulsacion

// Datos para conectar a WIFI
const char* ssid = "infind";                // Usuario del punto de acceso.
const char* password = "1518wifi";          // Contraseña del punto de acceso.
const char* mqtt_server = "172.16.53.142";  // IP del equipo.


// Definimos los mensajes que se van a publicar:
#define MSG_BUFFER_SIZE  (512)  // "#define" permite asignarle un nombre a una constante.
char msg_datos[MSG_BUFFER_SIZE];      // Mensaje de datos
char msg_on[MSG_BUFFER_SIZE];   // Mensaje de conexion activa
char msg_off[MSG_BUFFER_SIZE];  // Mensaje de conexion inactiva
char msg_led[MSG_BUFFER_SIZE];  // Mensaje con estado del LED1 y LED2
unsigned long lastMsg = 0;      // Tiempo en el que se envió el último mensaje de datos


// Creamos un cliente MQTT denominado "espClient".
WiFiClient espClient;
PubSubClient client(espClient);

// Definimos los topics:
  char topic_conexion[50];
  char topic_datos[50];
  char topic_config[50];
  char topic_led_cmd[50];
  char topic_led_status[50];
  char topic_switch_cmd[50];
  char topic_switch_status[50];
  char topic_fota[50];
  

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
    String clientId = "II12";
    
    // Definimos los mensajes de conexion y desconexion
    snprintf(msg_on, MSG_BUFFER_SIZE, "{\"CHIPID\":%s,\"online\":true}",ID_PLACA);
    snprintf(msg_off, MSG_BUFFER_SIZE, "{\"CHIPID\":%s,\"online\":false}",ID_PLACA);


    // Intento de conectarse    PONER CLIENTE BIEN Y CONTRASEÑA
    if (client.connect(clientId.c_str(),topic_conexion,2,true,msg_off)) //Función conexion con LWM retenido
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(topic_conexion, "Conexion al topic exitosa");        // Publicamos el estado de conexion en "II12/ID_PLACA/conexion"
      client.publish(topic_datos, "Conexion al topic exitosa");           //Publicamos los datos del sensor en "II12/ID_PLACA/datos"
      client.publish(topic_led_status, "Conexion al topic exitosa");      //Publicamos el estado del LED2 en "II12/ID_PLACA/led/status"
      client.publish(topic_switch_status, "Conexion al topic exitosa");   //Publicamos el estado del LED1 en "II12/ID_PLACA/switch/status"
      
      // ... and resubscribe
      client.subscribe(topic_config);       // Nos suscribimos al topic "II12/ID_PLACA/config" para obtener los parámetros de configuracion
      client.subscribe(topic_led_cmd);      // Nos suscribimos al topic "II12/ID_PLACA/led/cmd" para el control del LED2
      client.subscribe(topic_switch_cmd);   // Nos suscribimos al topic "II12/ID_PLACA/switch/cmd" para el control del LED1
      client.subscribe(topic_fota);         // Nos suscribimos al topic "II12/ID_PLACA/FOTA" para comprobar las actualizaciones
      Serial.println(topic_datos); 
      client.publish(topic_conexion,msg_on,true);       //Publicamos el estado de la conexión, mensaje retenido
        
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

//-------------------FUNCIONES BOTON-----------------------
void pressed(Button2& btn) {      // Se llama cuando se pulsa el boton
    Serial.println("pressed");
}
void released(Button2& btn) {     // Se llama cuando se libera el boton
    Serial.print("released: ");
    Serial.println(btn.wasPressedFor());
}
void click(Button2& btn) {
    Serial.println("click\n");
    sprintf(tipo_pulsacion, "simpleclick");
    //tipo_pulsacion="simpleclick";
}
void longClickDetected(Button2& btn) {
    Serial.println("long click detected\n");
}
void longClick(Button2& btn) {
    Serial.println("long click\n");
    sprintf(tipo_pulsacion, "longclick");
   // tipo_pulsacion="longclick";
}
void doubleClick(Button2& btn) {
    Serial.println("double click\n");
    sprintf(tipo_pulsacion, "doubleclick");
   // tipo_pulsacion="doubleclick";
}

//-------------------PROCESAR MENSAJES----------------------------
void procesa_mensaje(char* topic, byte* payload, unsigned int length) {
  
  char* mensaje = (char*)malloc(length+1);  // reservo memoria para copia del mensaje
  strncpy(mensaje, (char*)payload, length); // copio el mensaje en cadena de caracteres
  mensaje[length]='\0';                     // caracter cero marca el final de la cadena


  // Mostramos por pantalla el topic recibido.
  Serial.printf("Mensaje recibido [%s] %s\n", topic, mensaje);

  // MENSAJE: Configuración  
  if(strcmp(topic,topic_config)==0)   // Compara si el mensaje que llega es del mismo topic que el indicado
  {
      StaticJsonDocument<96> root;    // Indicamos el tamaño requerido para deserializar mensaje de configuracion
      // Deserialize the JSON document
      DeserializationError error = deserializeJson(root, mensaje,length);

      // Compruebo si no hubo error
      if (error) {
      Serial.print("Error deserializeJson() failed: ");   // Si devuelve un OK no ha habido error
      Serial.println(error.c_str());
      }
      else
      if(root.containsKey("envia") or root.containsKey("actualiza") or root.containsKey("velocidad") or root.containsKey("LED") or root.containsKey("SWITCH"))  // Se comprueba si existe el campo "envia"
      { 
        vel_envio = root["envia"];          // Velocidad de envio de datos
        vel_fota = root["actualiza"];       // Velocidad actualización FOTA (min)
        vel_PWM = root["velocidad"];        // Velocidad de incremento PWM
        LED_logica = root["LED"];           // Logica del LED1
        SWITCH_logica = root["SWITCH"];     // Logica del LED2
        
        Serial.printf("Mensaje OK, nueva configuración. Velocidad de publicación %d, velocidad de actualización %d, velocidad PWM %d, logicad del led PWM %d y logica del SWITCH %s",vel_envio,vel_fota,LED_logica,SWITCH_logica);

      }
      else    // Si el mensaje no incluye ninguno de los campos esperados
      {
        Serial.print("Error : ");
        Serial.println("key not found in JSON");
      }
  }
  
  // MENSAJE: Control LED PWM
  else if(strcmp(topic, topic_led_cmd)==0)   // Compara si el mensaje que llega es del mismo topic que el indicado
  {
    // Variable Json para desearializar el mensaje
    StaticJsonDocument<32> root;       // Indicamos el tamaño requerido para deserializar mensaje de comando del led
    // Deserialize the JSON document
    DeserializationError error = deserializeJson(root, mensaje,length);

    //Variables para operar:
    int PWM_led;  // Guarda contenido del campo "level" del mensaje 
    int PWM;      // Variable para el control del LED1
    char id;      // Guarda el identificador del mensaje
    
    // Compruebo si no hubo error
    if (error) {
      Serial.print("Error deserializeJson() failed: ");   // Si devuelve un OK no ha habido error.
      Serial.println(error.c_str());
    }
    
    else
      if(root.containsKey("level"))  // Se comprueba si existe el campo "level"
      { 
        PWM_led = root["level"];    // Se guarda el contenido del campo "level" 
        id = root["id"];            // Se guarda el contenido del campo "id"
        
        // Se muestra info del mensaje
        Serial.print("Mensaje OK, nivel de intensidad = ");
        Serial.println(PWM_led);
        
        // Cambio de escala de [0,100] a [0,255]:
        if(LED_logica==0){                     
        PWM = 255 - PWM_led*(255/100);         // Logica inversa
        }                                      
        else{
        PWM=PWM_led*(255/100);                // Logica directa
        }
        
        analogWrite(LED1,PWM);    // Escribimos en el LED1 el valor de PWM.
      }
      else  // En el mensaje no existe el campo "level"
      {
        Serial.print("Error : ");
        Serial.println("\"level\" key not found in JSON");
      }
  }

  //MENSAJE: Control LED Switch
  else if(strcmp(topic,topic_switch_cmd)==0)   // Compara si el mensaje que llega es del mismo topic que el indicado.
  {
    // Variable Json para desearializar el mensaje
    StaticJsonDocument<32> root;  // Indicamos el tamaño requerido para deserializar mensaje de comando del led
    // Deserialize the JSON document
    DeserializationError error = deserializeJson(root, mensaje,length);

    //Variables para operar:
     int Switch_led; // Guarda contenido del campo "level" del mensaje
     bool SWITCH;    // Variable para el control del LED1
     char id;        // Guarda el identificador del mensaje
     
    // Compruebo si no hubo error
    if (error) {
      Serial.print("Error deserializeJson() failed: ");   // Si devuelve un OK no ha habido error.
      Serial.println(error.c_str());
    }

    else
      if(root.containsKey("level"))    // Se comprueba si existe el campo "level"
      { 
        Switch_led = root["level"];    // Se guarda el contenido del campo "level" 
        id = root["id"];               // Se guarda el contenido del campo "id"

        // Se muestra info del mensaje
        Serial.print("Mensaje OK, valor del switch = ");
        Serial.println(Switch_led);

        // Se configura la salida en función del tipo de logica que se aplique
        if(SWITCH_logica==0){       // Logica inversa
        SWITCH = not(Switch_led); 
        }
        else{                       // Logica directa
        SWITCH = Switch_led;
        }
        
        digitalWrite(LED2,SWITCH);    // Escribimos en el LED2 el valor de SWITCH                                        

        // Cuando cambie el valor de la intensidad del led, enviamos un mensaje.
        if(Switch_led != Switch_status)
        {
          // Genera mensaje para publicar
          snprintf (msg_led, MSG_BUFFER_SIZE, "{\"CHIPID\":%s,\"SWITCH\": %d,\"origen\":mqtt,\"id\":%s}",ID_PLACA, Switch_led,id); 
          Serial.print("Mensaje de confirmación de intensidad: ");
          Serial.println(msg_led);
  
          // Publicamos mensaje     
          client.publish(topic_switch_status, msg_led);

          Switch_status = Switch_led;   // Guarda nuevo valor en la variable global
        }
      }
      else  // En el mensaje no existe el campo "level"
      {
        Serial.print("Error : ");
        Serial.println("\"level\" key not found in JSON");
      }
  }
    //MENSAJE: Comprueba actualizacion FOTA
  else if(strcmp(topic,topic_fota)==0)   // Compara si el mensaje que llega es del mismo topic que el indicado
  {
    // Variable Json para desearializar el mensaje
    StaticJsonDocument<16> root;  // Indicamos el tamaño requerido para deserializar mensaje de comprobar FOTA
    // Deserialize the JSON document
    DeserializationError error = deserializeJson(root, mensaje,length);
     
    // Compruebo si no hubo error
    if (error) {
      Serial.print("Error deserializeJson() failed: ");   // Si devuelve un OK no ha habido error.
      Serial.println(error.c_str());
    }
    else
      if(root.containsKey("actualiza"))    // Se comprueba si existe el campo "actualiza"
      { 
        lastAct=millis();
        intenta_OTA();    //Llamamos a la función que comprueba si hay actualizaciones OTA
      }
      else  // En el mensaje no existe el campo "actualiza"
      {
        Serial.print("Error : ");
        Serial.println("\"actualiza\" key not found in JSON");
      }
  }
  else   // Si no se reconoce el topic
  {
    Serial.println("Error: Topic desconocido");
  }

  free(mensaje);    // Para liberar memoria.
}

//-----------------SETUP-------------------------------

void setup() {
  
  pinMode(LED1, OUTPUT);      // Definimos el pin LED1 como un Output. Tambien se corresponde con LED_OTA
  pinMode(LED2, OUTPUT);      // Definimos el pin LED2 como un Output

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
  
  // Inicializa funcion del boton:
  button.begin(BUTTON_PIN);         // Pin del boton
  button.setLongClickTime(1000);    // Fijamos tiempo del long click
  button.setDoubleClickTime(400);   // Fijamos tiempo entre doble click
  Serial.println(" Longpress Time: " + String(button.getLongClickTime()) + "ms");
  Serial.println(" DoubleClick Time: " + String(button.getDoubleClickTime()) + "ms");
 
    // Funciones manejadoras:
  button.setPressedHandler(pressed);                      // Boton pulsado
  button.setReleasedHandler(released);                    // Boton liberado
  button.setClickHandler(click);                          // Single click
  button.setLongClickDetectedHandler(longClickDetected);  // Detecta click largo
  button.setLongClickHandler(longClick);                  // Long click
  button.setDoubleClickHandler(doubleClick);              // Double click

  
  // Definimos los topics que vamos a usar:
  sprintf(topic_conexion, "infind/II12/%s/conexion",ID_PLACA);
  sprintf(topic_datos, "II12/%s/datos",ID_PLACA);
  sprintf(topic_config, "II12/%s/config",ID_PLACA);
  sprintf(topic_led_cmd, "II12/%s/led/cmd",ID_PLACA);
  sprintf(topic_led_status, "II12/%s/led/status",ID_PLACA);
  sprintf(topic_switch_cmd, "II12/%s/swithc/cmd",ID_PLACA);
  sprintf(topic_switch_status, "II12/%s/switch/status",ID_PLACA);
  sprintf(topic_fota, "II12/%s/FOTA",ID_PLACA);
  
 
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

  // Si pasa el tiempo de actualización se comprueba si hay actualizacion FOTA:
   if (vel_fota==0) //Si el campo vel_fota es 0 no se actualiza periodicamente
   {
   }
   else if (misdatos.tiempo - lastAct > vel_fota*60*1000) { //vel_fota originalmente en minutos, lo pasamos a milisegundos
    lastAct=misdatos.tiempo;
    intenta_OTA();    //Llamamos a la función que comprueba si hay actualizaciones OTA
   }
   
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
    misdatos.valor_led = PWM_status;
    misdatos.valor_switch = Switch_status;

    // Manda mensaje de datos, si se supera el tiempo vel_envio
   if (misdatos.tiempo - lastMsg > vel_envio*1000) { //vel_envio originalmente en segundos, lo pasamos a milisegundos
    lastMsg = misdatos.tiempo;  // Actualizamos cuando se recibió el último mensaje.
    Serial.print("OK");
    // Generamos el mensaje JSON:
    snprintf(msg_datos,MSG_BUFFER_SIZE,"{\"CHIPID\":%s,\"Uptime\": %u, \"Vcc\": %f, \"DHT11\": {\"Temperatura\": %f, \"Humedad\": %f }, \"LED\": %d, \"SWITCH\": %b, \"Wifi\": {\"SSID\": \"%s\", \"IP\": \"%s\", \"RSSI\": %d }}",
                        misdatos.chipid, misdatos.tiempo, misdatos.bateria, misdatos.temp, misdatos.hum, misdatos.valor_led, misdatos.valor_switch, misdatos.SSId, misdatos.ip.toString().c_str(), misdatos.rssi);
    Serial.print("Mensaje JSON de datos: ");
    Serial.println(msg_datos); 

    client.publish(topic_datos, msg_datos);   //Se publica el mensaje en el topic correspondiente
  }

  // Función del boton:
    button.loop();             // Llamamos a la funcion del boton
    int PWM_led=PWM_status;    // Variable controla intensidad del LED
    
    if(strcmp(tipo_pulsacion,"simpleclick")){
          if (PWM_status>0){ 
            PWM_led=255;  // Se apaga el LED2
            valor_anterior = PWM_status;
          }
          else{
             PWM_led=valor_anterior;
             analogWrite(LED2,valor_anterior); //ponemos la intensidad del LED al ultimo valor del slider
          }
    }
    else if(strcmp(tipo_pulsacion,"doubleclick")){
          PWM_led=0;  // LED2 a nivel máximo
    }
    else if(strcmp(tipo_pulsacion,"longclick")){
          lastAct=millis();
          intenta_OTA();    //Llamamos a la función que comprueba si hay actualizaciones OTA
    }
    
    analogWrite(LED2,PWM_led); // Escribe el valor calculado
        
    // Cuando cambie el valor de la intensidad del led enviamos un mensaje.
     if(PWM_led != PWM_status)
     {
      // Genera mensaje para publicar
      snprintf (msg_led, MSG_BUFFER_SIZE, "{\"CHIPID\":%s,\"LED\": %d,\"origen\":pulsador}",ID_PLACA, PWM_led); 
      Serial.print("Mensaje de confirmación de intensidad: ");
      Serial.println(msg_led);
  
      // Publicamos mensaje     
      client.publish(topic_led_status, msg_led);

      PWM_status = PWM_led; // Guarda nuevo valor en la variable global
     }
    
}
