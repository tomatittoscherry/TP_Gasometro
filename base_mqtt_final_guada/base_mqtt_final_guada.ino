#include <WiFi.h>
#include "AsyncMqttClient.h"
#include <Wire.h>
#include "time.h"
#include "Arduino.h"
#include <DNSServer.h>
#include <AsyncTCP.h>
#include "ESPAsyncWebServer.h"
#include <Preferences.h>
#include <ESP32Time.h>
#include <pitches.h>
#include <SFE_BMP180.h>

//**********************************************************//
//* DECLARO DE FUN                                        **//
//**********************************************************//
void pedir_lahora(void);
void setup_rtc_ntp(void);

struct tm timeinfo;
ESP32Time rtc;

Preferences preferences;
DNSServer dnsServer;
AsyncWebServer server(80);

String ssid_wifi_ap;
String password_wifi_ap;
String Token_tel;
String Id_tel;

char ap_ssid[30] = "abc_mirko";
char ap_password[30] = "mirko1793";

char ap_token[50];

char ap_Id_tel[20];

bool is_setup_done = false;
bool valid_ssid_received = false;
bool valid_password_received = false;
bool wifi_timeout = false;

//////wifi
const char* ssid = "ORT-IoT";
const char* password = "OrtIOTnew22$";

const char name_device = 21;  ////device numero de grupo 5A 1x siendo x el numero de grupo
///                        5B 2x siendo x el numero de grupo

// Timers auxiliar variables//////////////////////////
unsigned long now = millis(); ///valor actual
unsigned long lastMeasure1 = 0; ///variable para contar el tiempo actual
unsigned long lastMeasure2 = 0; ///variable para contar el tiempo actual

const unsigned long interval_envio = 30000;//Intervalo de envio de datos mqtt
const unsigned long interval_leeo =  60000;//Intervalo de lectura de datos y guardado en la cola
int i = 0;

///time
long unsigned int timestamp ;  // hora
const char* ntpServer = "south-america.pool.ntp.org";
const long  gmtOffset_sec = -10800;
const int   daylightOffset_sec = 0;

///variables ingresar a la cola struct
int indice_entra = 0;
int indice_saca = 0;
bool flag_vacio = 1;

/////mqqtt
#define MQTT_HOST IPAddress(10, 162, 24, 31)
#define MQTT_PORT 1883
#define MQTT_USERNAME "esp32"
#define MQTT_PASSWORD "mirko15"
char mqtt_payload[150] ;  /////

// Test MQTT Topic
#define MQTT_PUB "/esp32/datos_sensores"
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

SFE_BMP180 bmp180;

//CAMBIAR ACA 
#define BUZZER_PIN 13
#define LIGHT_SENSOR_PIN 12
#define MQ5_PIN 11
#define MQ7_PIN 10


//alarm vars
long alarmMils = 0;
int auxAlarm = 0;
bool activeAlarm = false;

// setting PWM properties
const int freq = 2000;
const int buzzerChannel = 0;
const int resolution = 8;

volatile int interruptCounter;
int totalInterruptCounter;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

int contador = 0;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  contador = contador + 1;

  if (activeAlarm) {
    alarmMils = alarmMils + 1;
  }


  portEXIT_CRITICAL_ISR(&timerMux);

}
double T, P;

typedef struct
{
  long time;
  float T1;///tempe
  float G5;///gas 5
  float G7;// gas 7
  float Presion; //presion
  float luz;
  bool Alarma;
  float ruido;
} estructura ;
/////////////////
const int valor_max_struct = 1000; ///valor vector de struct
estructura datos_struct [valor_max_struct];///Guardo valores hasta que lo pueda enviar
estructura aux2 ;

/////*********************************************************************/////
////////////////////////////setup wifi/////////////////////////////////////////
/////*********************************************************************/////
void setupmqtt()
{
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  WiFi.onEvent(WiFiEvent);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);
  connectToWifi();
}
////////////////////////////Envio de datos mqtt//////////////////////////////////////////
////////Funcion que envia valores cuando la estructura no este vacia ///////////////////
///////////////////////////////////////////////////////////////////////////////////////
void fun_envio_mqtt ()
{
  fun_saca ();////veo si hay valores nuevos
  if (flag_vacio == 0) ////si hay los envio
  {
    Serial.print("enviando");
    ////genero el string a enviar
    snprintf (mqtt_payload, 150, "%u&%ld&%.2f&%.2f&%.2f&%.2f&%.2f&%.2f&%u", name_device, aux2.time, aux2.T1, aux2.G5, aux2.G7, aux2.Presion, aux2.luz, aux2.ruido, aux2.Alarma); //random(10,50)
    aux2.time = 0; ///limpio valores
    aux2.T1 = 0;
    aux2.G5 = 0;
    aux2.G7 = 0;
    aux2.Presion = 0;
    aux2.luz = 0;
    aux2.Alarma = 0;
    aux2.ruido = 0;

    Serial.print("Publish message: ");
    Serial.println(mqtt_payload);
    // Publishes Temperature and Humidity values
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB, 1, true, mqtt_payload);
  }
  else
  {
    Serial.println("no hay valores nuevos");
  }
}///////////////////////////////////////////////////

///////////////////////////////////////////////////
void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
}///////////////////////////////////////////////////
void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}///////////////////////////////////////////////////

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}///////////////////////////////////////////////////
////////////////NO TOCAR FUNCIONES MQTT///////////////////////////////////
void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}///////////////////////////////////////////////////

////////////////NO TOCAR FUNCIONES MQTT///////////////////////////////////
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}
////////////////NO TOCAR FUNCIONES MQTT///////////////////////////////////
void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}///////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
/////////////Funcion que saca un valor de la estructura para enviar //////
///////////////////////////////////////////////////////////////////////
void fun_saca () {
  if (indice_saca != indice_entra)
  {
    aux2.time = datos_struct[indice_saca].time;
    aux2.T1 = datos_struct[indice_saca].T1;
    aux2.G5 = datos_struct[indice_saca].G5;
    aux2.G7 = datos_struct[indice_saca].G7;
    aux2.Presion = datos_struct[indice_saca].Presion;
    aux2.luz = datos_struct[indice_saca].luz;
    aux2.Alarma = datos_struct[indice_saca].Alarma;
    aux2.ruido = datos_struct[indice_saca].ruido;

    flag_vacio = 0;

    Serial.println(indice_saca);
    if (indice_saca >= (valor_max_struct - 1))
    {
      indice_saca = 0;
    }
    else
    {
      indice_saca++;
    }
    Serial.print("saco valores de la struct isaca:");
    Serial.println(indice_saca);
  }
  else
  {
    flag_vacio = 1; ///// no hay datos
  }
  return ;
}
/////////////////////////////////////////////////////////////////////
/////////////funcion que ingresa valores a la cola struct///////////
///////////////////////////////////////////////////////////////////
void fun_entra (void)
{
  if (indice_entra >= valor_max_struct)
  {
    indice_entra = 0; ///si llego al maximo de la cola se vuelve a cero
  }
  //////////// timestamp/////// consigo la hora
  Serial.print("> NTP Time:");
  timestamp =  time(NULL);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;  //// si no puede conseguir la hora
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  ///////////////////////// fin de consigo la hora
  checkBMP();
  datos_struct[indice_entra].time = timestamp;
  datos_struct[indice_entra].T1 = T; /// leeo los datos
  datos_struct[indice_entra].G5 = checkMQ5(); //// se puede pasar por un parametro
  datos_struct[indice_entra].G7 = checkMQ7();
  datos_struct[indice_entra].luz = checkLS();
  datos_struct[indice_entra].ruido = 1; ///// valores motor
  datos_struct[indice_entra].Presion = P;
  datos_struct[indice_entra].Alarma = activeAlarm;
  indice_entra++;
  Serial.print("saco valores de la struct ientra");
  Serial.println(indice_entra);
}
//**********************************************************//
//* SETUP  rtc                                           **//
//**********************************************************//
void setup_rtc_ntp(void)
{
  // init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  timestamp = time(NULL);
  rtc.setTime(timestamp + gmtOffset_sec);
}

//**********************************************************//
//* PIDE LA HORA AL SERVER O AL ESP32                     **//
//**********************************************************//
void pedir_lahora(void)
{
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("veo la hora del rtc interno ");
    timestamp = rtc.getEpoch() - gmtOffset_sec;
    timeinfo = rtc.getTimeStruct();
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  }
  else
  {
    Serial.print("NTP Time:");
    timestamp = time(NULL);
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  }

  return;
}
//**********************************************************//
//* wifi ap                                **//
//**********************************************************//
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>Captive Portal Demo</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <h3>Captive Portal Demo</h3>
  <br><br>
  <form action="/get">
    <br>
    SSID: <input type="text" name="ssid">
    <br>
    Password: <input type="text" name="password">
    <br>
    Token_tel: <input type="text" name="Token_tel">
    <br>
    Id_tel: <input type="number" name="Id_tel">
    <input type="submit" value="Submit">
  </form>
</body></html>)rawliteral";
void StartCaptivePortal(void);
class CaptiveRequestHandler : public AsyncWebHandler
{
  public:
    CaptiveRequestHandler() {}
    virtual ~CaptiveRequestHandler() {}

    bool canHandle(AsyncWebServerRequest *request)
    {
      // request->addInterestingHeader("ANY");
      return true;
    }

    void handleRequest(AsyncWebServerRequest *request)
    {
      request->send_P(200, "text/html", index_html);
    }
};

void setupServer()
{
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    request->send_P(200, "text/html", index_html);
    Serial.println("Client Connected");
  });

  server.on("/get", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    String inputMessage;
    String inputParam;

    if (request->hasParam("ssid")) {
      inputMessage = request->getParam("ssid")->value();
      inputParam = "ssid";
      ssid_wifi_ap = inputMessage;
      Serial.println(inputMessage);
      valid_ssid_received = true;
    }

    if (request->hasParam("password")) {
      inputMessage = request->getParam("password")->value();
      inputParam = "password";
      password_wifi_ap = inputMessage;
      Serial.println(inputMessage);
      valid_password_received = true;
    }


    if (request->hasParam("Token_tel")) {
      inputMessage = request->getParam("Token_tel")->value();
      inputParam = "Token_tel";
      Token_tel = inputMessage;
      Serial.println(inputMessage);
      valid_password_received = true;
    }

    if (request->hasParam("Id_tel")) {
      inputMessage = request->getParam("Id_tel")->value();
      inputParam = "Id_tel";
      Id_tel = inputMessage;
      Serial.println(inputMessage);
      valid_password_received = true;
    }

    request->send(200, "text/html", "The values entered by you have been successfully sent to the device. It will now attempt WiFi connection");
  });
}

void WiFiSoftAPSetup()
{
  WiFi.mode(WIFI_AP);
  WiFi.softAP("EL CHINO LOCO WIFI<");///puede agregarle el nombre del grupo
  Serial.print(F("AP IP address: "));
  Serial.println(WiFi.softAPIP());
}

void WiFiStationSetup(String rec_ssid, String rec_password, String rec_Id_tel, String rec_Token_tel)
{
  wifi_timeout = false;
  WiFi.mode(WIFI_STA);

  rec_ssid.toCharArray(ap_ssid, rec_ssid.length() + 1);
  rec_password.toCharArray(ap_password, rec_password.length() + 1);
  rec_Token_tel.toCharArray(ap_token, rec_Token_tel.length() + 1);
  rec_Id_tel.toCharArray(ap_Id_tel, rec_Id_tel.length() + 1);

  // Serial.print("Received SSID: "); Serial.println(ap_ssid);
  // Serial.print("And password: "); Serial.println(ap_password);
  // Serial.print("And id: "); Serial.println(rec_Id_tel);
  // Serial.print("And token: ");Serial.println(rec_Token_tel);
  WiFi.begin(ap_ssid, ap_password);

  uint32_t t1 = millis();
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(2000);
    Serial.print(F("."));
    if (millis() - t1 > 50000) // 50 seconds elapsed connecting to WiFi
    {
      // Serial.println();
      Serial.println(F("Timeout connecting to WiFi. The SSID and Password seem incorrect."));
      valid_ssid_received = false;
      valid_password_received = false;
      is_setup_done = false;
      preferences.putBool("is_setup_done", is_setup_done);

      StartCaptivePortal();
      wifi_timeout = true;
      break;
    }
  }
  if (!wifi_timeout)
  {
    is_setup_done = true;
    // Serial.println("");
    Serial.print(F("WiFi connected to: "));
    Serial.println(rec_ssid);
    Serial.print(F("IP address: "));
    Serial.println(WiFi.localIP());
    preferences.putBool("is_setup_done", is_setup_done);
    preferences.putString("rec_ssid", rec_ssid);
    preferences.putString("rec_password", rec_password);
    preferences.putString("rec_Token_tel", rec_Token_tel);
    preferences.putString("rec_Id_tel", rec_Id_tel);
  }
}

void StartCaptivePortal()
{
  Serial.println(F("Setting up AP Mode"));
  WiFiSoftAPSetup();
  Serial.println(F("Setting up Async WebServer"));
  setupServer();
  Serial.println(F("Starting DNS Server"));
  dnsServer.start(53, "*", WiFi.softAPIP());
  server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER); // only when requested from AP
  server.begin();
  dnsServer.processNextRequest();
}

void ap_wifisetup()
{
  Serial.println();
  preferences.begin("my-pref", false);
  // preferences.clear();//////////////////////////////////linea de codigo para borrar la eeprom
  is_setup_done = preferences.getBool("is_setup_done", false);
  ssid_wifi_ap = preferences.getString("rec_ssid", "Sample_SSID");
  ssid_wifi_ap.toCharArray(ap_ssid, ssid_wifi_ap.length() + 1);

  password_wifi_ap = preferences.getString("rec_password", "abcdefgh");
  password_wifi_ap.toCharArray(ap_password, password_wifi_ap.length() + 1);

  Token_tel = preferences.getString("rec_Token_tel", "abcdefgh");
  Token_tel.toCharArray(ap_token, Token_tel.length() + 1);

  Id_tel = preferences.getString("rec_Id_tel", "abcdefgh");
  Id_tel.toCharArray(ap_Id_tel, Id_tel.length() + 1);
  if (!is_setup_done)
  {
    StartCaptivePortal();
  }
  else
  {
    // Serial.println("Using saved SSID and Password to attempt WiFi Connection!");
    Serial.print(F("Saved SSID is "));
    Serial.println(ssid_wifi_ap);
    Serial.print(F("Saved Password is "));
    Serial.println(password_wifi_ap);
    Serial.print(F("Saved token is "));
    Serial.println(Token_tel);
    Serial.print(F("Saved id is "));
    Serial.println(Id_tel);

    WiFiStationSetup(ssid_wifi_ap, password_wifi_ap, Id_tel, Token_tel);
  }

  while (!is_setup_done)
  {
    dnsServer.processNextRequest();
    delay(10);
    if (valid_ssid_received && valid_password_received)
    {
      Serial.println(F("Attempting WiFi Connection!"));
      WiFiStationSetup(ssid_wifi_ap, password_wifi_ap, Id_tel, Token_tel);
    }
  }
}

////////////////////////////////////////////////////////////////////
/////////////SETUP/////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  ap_wifisetup();
  setup_rtc_ntp(); // put your setup code here, to run once:
  /////declaro pines digitales
  setupmqtt();
  //Setup de time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000, true);
  timerAlarmEnable(timer);

  // configure LED PWM functionalitites
  ledcSetup(buzzerChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(BUZZER_PIN, buzzerChannel);

}///////////////////////////////////////////////////

void loop() {
  if (contador == 2000) {
    now = millis();
    if (now - lastMeasure1 > interval_envio) {    ////envio el doble de lectura por si falla algun envio
      lastMeasure1 = now;/// cargo el valor actual de millis
      fun_envio_mqtt();///envio los valores por mqtt
    }
    if (now - lastMeasure2 > interval_leeo) {
      lastMeasure2 = now;/// cargo el valor actual de millis
      fun_entra(); ///ingreso los valores a la cola struct
    }
    pedir_lahora();

   contador = 0;
  }
  if(checkMQ5() >= 2000 || checkMQ7() >= 2000){
    playAlarm();
  }else{
    activeAlarm = false;
    auxAlarm = 0;
    alarmMils = 0;
  }
}

void playAlarm() {
  activeAlarm = true;
  switch (auxAlarm) {
    case 0:
      ledcWriteTone(buzzerChannel, NOTE_CS7);
      if (alarmMils == 500) {
        auxAlarm = 1;
        alarmMils = 0;
      }
      break;
    case 1:
      ledcWriteTone(buzzerChannel, NOTE_G7);
      if (alarmMils == 500) {
        auxAlarm = 0;
        alarmMils = 0;
      }
      break;
  }
}

int checkLS() {
  float ls = analogRead(LIGHT_SENSOR_PIN);
  int mapLS = map(ls, 0, 4095, 0, 100);
  return mapLS;
}

float checkMQ5() {
  float mq5 = analogRead(MQ5_PIN);
  return mq5;
}
float checkMQ7() {
  float mq7 = analogRead(MQ7_PIN);
  return mq7;
}

float checkBMP() {
  char status;

  status = bmp180.startTemperature();//Inicio de lectura de temperatura
  if (status != 0)
  {
    delay(status); //Pausa para que finalice la lectura
    status = bmp180.getTemperature(T); //Obtener la temperatura
    if (status != 0)
    {
      status = bmp180.startPressure(3); //Inicio lectura de presión
      if (status != 0)
      {
        delay(status);//Pausa para que finalice la lectura
        status = bmp180.getPressure(P, T); //Obtenemos la presión
      }
    }
  }
}
