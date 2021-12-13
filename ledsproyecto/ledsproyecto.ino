#include "Button2.h"
#include "DHTesp.h"         // Incluimos la librería del sensor
#include <ESP8266WiFi.h>    // Librería para el wifi
#include <PubSubClient.h>   // Librería para publicar y suscribirse
#include <ArduinoJson.h>
#include <ESP8266httpUpdate.h>

#define HTTP_OTA_ADDRESS      F("172.16.53.138")         // Address of OTA update server
#define HTTP_OTA_PATH         F("/esp8266-ota/update") // Path to update firmware
#define HTTP_OTA_PORT         1880                     // Port of update server
                                                       // Name of firmware
#define HTTP_OTA_VERSION      String(__FILE__).substring(String(__FILE__).lastIndexOf('\\')+1) + ".nodemcu"

DHTesp dht;           // Objeto que representa el sensor
ADC_MODE(ADC_VCC);    // Esta línea hay que añadirla para poder medir la alimentación de la placa.


// Update these with values suitable for your network.
const char* ssid = "infind";        // Usuario del punto de acceso.
const char* password = "1518wifi";        // Contraseña del punto de acceso.
const char* mqtt_server = "172.16.53.138";  // IP del equipo.

// cadenas para ID
char ID_PLACA[16];

// GPIOs y variables  
int LED2 = 2;
int LED16 = 16; 
int valor_led;           // variable que indica la intensidad de la señal
int valor_anterior=-1;   // Variable que vamos a usar después para comparar el valor actual con el anterior.
bool cambio_polaridad=false;
// funciones para progreso de OTA
void progreso_OTA(int,int);
void final_OTA();
void inicio_OTA();
void error_OTA(int);

// Creamos un cliente denominado "espClient".
WiFiClient espClient;
PubSubClient client(espClient);


// Creación del mensaje que se va a enviar.
unsigned long lastMsg = 0;      // Tiempo en el que se envió el último mensaje (medido desde que la placa empezó a ejecutarse).
#define MSG_BUFFER_SIZE  (256)  // El comando "#define" permite asignarle un nombre a una constante.
char msg[MSG_BUFFER_SIZE];      // Estamos creando un msg de tamaño 256.
char msg_json[MSG_BUFFER_SIZE];
int value = 0;                  // "value" es una variable que indica el número de mensajes que se han enviado.



#define BUTTON_PIN  0
Button2 button;
char* tipo_pulsacion;

void setup() {
  pinMode(LED2, OUTPUT);      // Inicializa el LED2 como un Output
  pinMode(2, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  digitalWrite(LED2, HIGH);   // Inicialmente el LED2 está apagado.
  digitalWrite(2, LOW);   // LED on
  
  Serial.begin(115200);       // Establecemos la velocidad del puerto serie
  sprintf(ID_PLACA, "ESP_%d", ESP.getChipId());
  
  dht.setup(5, DHTesp::DHT11);      // Conectamos el sensor DHT11 al puerto GPIO 5
  
  //setup_wifi();
  //intenta_OTA();
  //client.setServer(mqtt_server, 1883);
  //client.setCallback(procesa_mensaje);
  client.setBufferSize(512);
  button.begin(BUTTON_PIN);
  button.setLongClickTime(1000);
  button.setDoubleClickTime(400);

  Serial.println(" Longpress Time: " + String(button.getLongClickTime()) + "ms");
  Serial.println(" DoubleClick Time: " + String(button.getDoubleClickTime()) + "ms");
  button.setPressedHandler(pressed);
  button.setReleasedHandler(released);

  button.setClickHandler(click);
  button.setLongClickDetectedHandler(longClickDetected);
  button.setLongClickHandler(longClick);
  
  button.setDoubleClickHandler(doubleClick);
}

unsigned long ultimo = 0;

void pressed(Button2& btn) {
    Serial.println("pressed");
}
void released(Button2& btn) {
    Serial.print("released: ");
    Serial.println(btn.wasPressedFor());
}
void click(Button2& btn) {
    Serial.println("click\n");
    tipo_pulsacion="simpleclick";
}
void longClickDetected(Button2& btn) {
    Serial.println("long click detected\n");
}
void longClick(Button2& btn) {
    Serial.println("long click\n");
    tipo_pulsacion="longclick";
}
void doubleClick(Button2& btn) {
    Serial.println("double click\n");
    tipo_pulsacion="doubleclick";
}




void procesa_mensaje(char* topic, byte* payload, unsigned int length) {
  
  char* mensaje = (char*)malloc(length+1);  // reservo memoria para copia del mensaje
  strncpy(mensaje, (char*)payload, length); // copio el mensaje en cadena de caracteres
  mensaje[length]='\0';                     // caracter cero marca el final de la cadena

  
  int valor_anterior=-1;   // Variable que vamos a usar después para comparar el valor actual con el anterior.
        

  // Mostramos por pantalla el topic recibido.
  Serial.printf("Mensaje recibido [%s] %s\n", "infind/GRUPO12/led/cmd", mensaje);
  
  // Compara si el mensaje que llega es del mismo topic que el indicado.
  if(strcmp(topic, "infind/GRUPO12/led/cmd")==0) 
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
        valor_led = root["level"];    // Lo que hay en el campo "level" del payload se guarda en la variable "valor_led".
        Serial.print("Mensaje OK, nivel de intensidad = ");
        Serial.println(valor_led); // LO QUE RECIBIMOS DEL SLIDER
        int intensidad=valor_anterior; // variable auxiliar que usamos para aumentar la intensidad cada 10ms

        while (intensidad!=valor_led)
        {
            int PWM;
            
            if (millis()-ultimo >= 10) // cambiar el 10 por mqtt!!!!!!!!!
            {
              ultimo = millis();    
              if (intensidad<valor_led)
              {
                intensidad++;
              }
              else if (intensidad>valor_led)
              {
                intensidad--;
              }
            }         
              if(cambio_polaridad==false) // boton que se pulsa en nodered(var booleana)
              {
                int PWM = 255 - intensidad*(255/100);    // Pasamos de escala 100-0 a 0-255.
              }
                                                      // A 255 le restamos "valor_led*(255/100)" para darle la vuelta a la escala.
                                                      // Es así porque el LED está encendido a nivel lógico bajo (0) y se apaga a nivel lógico alto (1).
              else
              {
                PWM=intensidad*255/100;
                analogWrite(LED16,LOW);
              }
              analogWrite(LED2,PWM);    // Escribimos en el LED2 el valor de PWM.
              analogWrite(LED16,HIGH);
              if(strcmp(tipo_pulsacion,"simpleclick"))
              {
                analogWrite(LED2,valor_anterior); //ponemos la intensidad del LED al ultimo valor del slider
              }
              if(strcmp(tipo_pulsacion,"doubleclick"))
              {
                if (cambio_polaridad==false)
                {
                  analogWrite(LED2,0);
                }
                else{
                  analogWrite(LED2,255);
                }
              }
              if(strcmp(tipo_pulsacion,"longclick"))
              {
                Serial.print("Aqui hay que mirar el fota :) ");
              }
        }
        // Cuando cambie el valor de la intensidad del led, enviamos un mensaje.
        if(valor_led != valor_anterior)
        {
          // Información del LED
          snprintf (msg, MSG_BUFFER_SIZE, "{\"LED\": %d}", valor_led);
          Serial.print("Mensaje de confirmación de intensidad: ");
          Serial.println(msg);
          client.publish("infind/GRUPO12/led/status", msg);

          valor_anterior = valor_led;
        }
      }
  }
}
void loop() {
  button.loop();

}
