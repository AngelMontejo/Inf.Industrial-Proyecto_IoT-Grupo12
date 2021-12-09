#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "DHTesp.h"
#include <ArduinoJson.h>
#include <ESP8266httpUpdate.h>

//Datos para actualización OTA:
#define HTTP_OTA_ADDRESS      F(Wifi.localIP().toString().c_str())    // Address of OTA update server
#define HTTP_OTA_PATH         F("/esp8266-ota/update")                // Path to update firmware
#define HTTP_OTA_PORT         1880                                    // Port of update server                                                          
#define HTTP_OTA_VERSION      String(__FILE__).substring(String(__FILE__).lastIndexOf('\\')+1) + ".nodemcu"  // Name of firmware

//Funciones para progreso de OTA
void progreso_OTA(int,int);
void final_OTA();
void inicio_OTA();
void error_OTA(int);


//Datos sensor
DHTesp dht;           // Objeto sensor
ADC_MODE(ADC_VCC);    // Función para poder medir la alimentación de la placa.


// GPIOs y variables  
int LED1 = 2;         //Pin del LED PWM
int LED2 = 16;        //Pin del LED interruptor
int LED_OTA = 2;      //Pin del LED usado durante la actualización FOTA
int SENSOR = 5;       //Pin de datos del sensor

char ID_PLACA[16];        // Cadenas para ID de la placa
int status_led;           // Variable que indica estado del LED2
int status_anterior=-1;   // Variable que vamos a usar después para comparar el valor actual con el anterior.


//Datos para conectar a WIFI
const char* ssid = "infind";                // Usuario del punto de acceso.
const char* password = "1518wifi";          // Contraseña del punto de acceso.
const char* mqtt_server = "172.16.53.138";  // IP del equipo.
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

//-----------------------------------------------------
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
    if (client.connect(clientId.c_str(),"infind/GRUPO12/conexion",2,true,"{online:false}")) //Función connecto con LWM retenido
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("infind/GRUPO12/datos", "Conexion al topic exitosa");     // Publicamos los datos del sensor en "infind/GRUPO12/datos".
      client.publish("infind/GRUPO12/conexion", "Conexion al topic exitosa");  //Publicamos el estado de conexion en "infind/GRUPO12/conexion"
      
      // ... and resubscribe
      client.subscribe("infind/GRUPO12/led/cmd");  // Nos suscribimos al topic "infind/GRUPO12/led/cmd" para el control del LED.

      client.publish("infind/GRUPO12/conexion", "{online:true}" ,true);       //Publicamos el estado de la conexión, mensaje retenidp
        
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

//----------------------SETUP-------------------------------

void setup() {
  pinMode(LED1, OUTPUT);      // Definimos el pin LED1 como un Output. Tambien se corresponde con LED_OTA
  pinMode(LED2, OUTPUT);      // Definimos el pin LED" como un Output

  digitalWrite(LED1, HIGH);   // Inicialmente el LED1 está apagado. Tambien se corresponde con LED_OTA
  digitalWrite(LED2, HIGH);   // Inicialmente el LED2 está apagado.
  
  Serial.begin(115200);       // Establecemos la velocidad del puerto serie
  
  sprintf(ID_PLACA, "ESP_%d", ESP.getChipId()); // Función obtener ID de la placa
  
  dht.setup(SENSOR, DHTesp::DHT11);      // Set el pin SENSOR que se conecta al sensor DHT11 
  
  setup_wifi();     //Llamamos a la función que se conecta al wifi
  intenta_OTA();    //Llamamos a la función que comprueba si hay actualizaciones OTA
  
  client.setServer(mqtt_server, 1883);
  client.setCallback(procesa_mensaje);
  client.setBufferSize(512);

}

//-------------------------MAIN-------------------------------

void loop() {

  // Si el cliente se desconecta, llamamos a a la función de reconectar.
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Guardamos en una variable el tiempo actual en ms
  unsigned long now = millis();

  // Como queremos actualizar los datos cada 10ms, lo ponemos de condición del if.
  if (now - lastMsg > 30000) {
    lastMsg = now;  // Actualizamos cuando se recibió el último mensaje.

    // Actualizamos el valor de los sensores y de la batería.
    float hum = dht.getHumidity();         // Datos de humedad.
    float temp = dht.getTemperature();     // Datos de temperatura.
    float bateria = ESP.getVcc();          // Medimos la alimentación de la placa.

    // Actualizamos datos de conexión.
    int32_t rssi = WiFi.RSSI();            // Datos de conexión de wifi.
    IPAddress ip = WiFi.localIP();         // Datos de dirección IP (convertidos a tipo string).

    // Conformamos el mensaje con los datos del sensor
    snprintf (msg, MSG_BUFFER_SIZE, "{\"Uptime\": %u, \"Vcc\": %f, \"DHT11\": {\"Temperatura\": %f, \"Humedad\": %f }, \"LED\": %d, \"Wifi\": {\"SSID\": \"%s\", \"IP\": \"%s\", \"RSSI\": %d }}", now, bateria, temp, hum, valor_led, ssid, ip.toString().c_str(), rssi);

    //Serializamos:
   
    snprintf (msg_json, MSG_BUFFER_SIZE,serializa_JSON(msg));
    Serial.print("Datos: ");
    Serial.println(msg_json);
    client.publish("infind/GRUPO12/datos", msg_json);   
  }

}
