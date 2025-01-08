#include "WifiCam.hpp"
#include <WiFi.h>

static const char* WIFI_SSID = "MICHITA2606";    //USER del WiFi
static const char* WIFI_PASS = "Super2630$";       //Contraseña WiFi

const int led1Pin = 13;  // Pin 1 
const int led2Pin = 12;  // Pin 2 

int option;


esp32cam::Resolution initialResolution;

WebServer server(80);

void setup()
{
  Serial.begin(115200);

// Configura los pines de salida 
  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);

  // Inicia ambos pines apagados
  digitalWrite(led1Pin, LOW);
  digitalWrite(led2Pin, LOW);

  Serial.println();
  delay(2000);

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi failure");
    delay(5000);
    ESP.restart();
  }
  Serial.println("WiFi connected");

  {
    using namespace esp32cam;

    initialResolution = Resolution::find(1024, 768);

    Config cfg;
    cfg.setPins(pins::AiThinker);
    cfg.setResolution(initialResolution);
    cfg.setJpeg(80);

    bool ok = Camera.begin(cfg);
    if (!ok) {
      Serial.println("camera inicializando");
      delay(5000);
      ESP.restart();
    }
    Serial.println("camera inicializada correctamente");
  }

  Serial.println("camera inializada");
  Serial.print("http://");
  
  Serial.println(WiFi.localIP());

  addRequestHandlers();
  server.begin();
  
  
}
void loop()
{
  server.handleClient();

  if(Serial.available()> 0) {   //Caracteres enviados
    option = Serial.read();
    Serial.println(option);
    if(option == 'U'){              //Primer  electrodomestico
      digitalWrite(led1Pin, HIGH);  //Mandar el impulso para prender 
      digitalWrite(led2Pin, LOW);   //Mandar el impulso para apagar  
    }
    if(option == 'D'){             //Segundo electrodomestico
    digitalWrite(led1Pin, LOW);    //Mandar el impulso para apagar  
    digitalWrite(led2Pin, HIGH);   //Mandar el impulso para prender 
    }
    if(option == 'E'){
    digitalWrite(led1Pin, LOW);    //Mandar el impulso para apagar  
    digitalWrite(led2Pin, LOW);    //Mandar el impulso para apagar  
    }
  }  
}
