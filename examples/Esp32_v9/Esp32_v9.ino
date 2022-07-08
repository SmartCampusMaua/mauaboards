/***********************************************/

/***********************************************/

/***********************************************/
String deviceName = "TesteCB"; //MUDAR ESSA LINHA PARA PERSONALIZAR SEU DISPOSITIVO ("Cxorgao" ou "Redeneural" ou "Energia")
/***********************************************/

#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include "time.h"
#include "MAUA_System.h"
#include "FS.h"
#include <EEPROM.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h> // MQTT
#define MQTT_MAX_PACKET_SIZE 1000
#include <HardwareSerial.h>


/***********************************************/
 
#define EEPROM_CONFIG_ADDRESS     10
#define TRIGGER_PIN               0
#define LED_PIN                   34
#define SENSOR_PIN                35
#define PWM_PIN                   12
#define PWM_PIN_CHANNEL           0

/***********************************************/

/***********************************************/
#define PUBLICA_VIA_SOCKET        0     // comentar para não publicar
//#define PUBLICA_VIA_MQTT          1     // comentar para não publicar
/***********************************************/
typedef struct 
{
  char host[40];
  uint16_t port_host;
  char mqtt_server[100];
  uint16_t mqtt_port;
  char mqtt_user[40];
  char mqtt_password[40];
  char mqtt_topic[40];
}Package;

typedef struct
{
    unsigned long UrmsA;
    unsigned long UrmsB;
    unsigned long UrmsC;
    unsigned long IrmsA;
    unsigned long IrmsB;
    unsigned long IrmsC;
} ATM90E36A;

typedef struct
{
    unsigned long Irms;
    unsigned long Urms;
} ATM90E26A;

typedef struct
{
  ATM90E36A atm90e36a;
  ATM90E26A atm90e26a;
  unsigned char cardstatus;
  unsigned char setPoint;
  unsigned char sensorData;
  String date = "00/00/00 00:00:00";
} Dados;

Dados dados;
Package data;

System board;
WiFiClient espClient;
PubSubClient MQTT(espClient);
HardwareSerial SerialExtern(1);
HardwareSerial SerialDevice(2);

#define SERIALExtern_RXPIN 4      //esp32
#define SERIALExtern_TXPIN 5
#define SERIALDevice_RXPIN 16     
#define SERIALDevice_TXPIN 17
/***********************************************/

// wifimanager can run in a blocking mode or a non blocking mode
// Be sure to know how to process loops with no delay() if using non blocking
bool wm_nonblocking = false; // change to true to use non blocking
WiFiManager wm;
WiFiManagerParameter parameter("parameterId", "Parameter Label", "default value", 40);
WiFiManagerParameter host("host", "host", "smartcampus.maua.br", 40);
WiFiManagerParameter port_host("port", "port", "3000", 6);
WiFiManagerParameter custom_mqtt_server("server", "mqtt server", "smartcampus.maua.br", 100); 
WiFiManagerParameter custom_mqtt_port("mqttport", "mqtt port", "1883", 6);
WiFiManagerParameter custom_mqtt_user("user", "mqtt user", "PUBLIC", 40);
WiFiManagerParameter custom_mqtt_password("password", "mqtt password", "public", 40);
WiFiManagerParameter custom_mqtt_topic("topic", "mqtt topic", deviceName.c_str() , 40);

// Set web server port number to 8080
WiFiServer wifiserver(80);      //sobre escreve a outra pagina default

/***********************************************/

String header;

// Assign output variables to GPIO pins
String        LED_PINState = "off";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -3*3600;
const int   daylightOffset_sec = 0;
struct tm   timeinfo;

char msg_buffer[1000];
char buffer_serial[1001]; // +1 allows space for the null terminator
char *p;
char aux_buffer[50];

//timers
static unsigned long mtime = 0;
static unsigned long stime = 0;
static unsigned long second_time = 0;
static unsigned long mqtt_timer = 0;
static unsigned long timeout_serial = 0; //miliseconds
String cardStatus;
String devicemacAddress;
String deviceIp;

IPAddress myIP;
/***********************************************/

void setup()
{
  //wm.resetSettings();
  // Inicializa seu board, e seus periféricos  
  board.init(); 
  // Cria o arquivo para armazenamento no cartão SD
  board.createLogOnSD(); 
  
  EEPROM.begin(sizeof(data) + EEPROM_CONFIG_ADDRESS);
  EEPROM.put(EEPROM_CONFIG_ADDRESS, data);
  EEPROM.get(EEPROM_CONFIG_ADDRESS, data);

  pinMode(LED_PIN, OUTPUT);
  pinMode(TRIGGER_PIN, INPUT);
  pinMode(SENSOR_PIN, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  ledcAttachPin(PWM_PIN, PWM_PIN_CHANNEL);
  ledcSetup(PWM_PIN_CHANNEL, 1000, 8);  //1000Hz, 8 bits
  ledcWrite(PWM_PIN_CHANNEL, 0);
  
  ArduinoOTA.begin();

  SerialExtern.begin(9600,SERIAL_8N1, SERIALExtern_RXPIN, SERIALExtern_TXPIN);      // comunicação com o mundo externo (pic)
  SerialDevice.begin(9600,SERIAL_8N1, SERIALDevice_RXPIN, SERIALDevice_TXPIN);      // comunicação com o mundo externo (gps)
 
  MQTT.setServer(data.mqtt_server, data.mqtt_port);
  MQTT.setCallback(mqtt_callback); 

  deviceIp = "";
  cardStatus = "off";  //debug martins
  delay(2000); // wait stabilish
  
  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  getTime();

  Serial.println("Starting " +  deviceName + " " + dados.date + " " + devicemacAddress.c_str());
  SerialExtern.println("Starting Serial1");
  SerialDevice.println("Starting Serial2");
}

/***********************************************/
void loop()
{    
  if(millis()-stime > 100 ) //   every 100 miliseconds
  {
      stime = millis();
      timeout_serial++;
      checkButton();
      trata_pwm();
  }          
  
  if(millis()-second_time > 1000 )// every 5 seconds
  {
    second_time = millis();
    Serial.print(".");
    //envia_dados_mqtt();    // comentar para nao enviar a cada 1s
  }

  if(millis()- mtime > 60000 ) // A CADA MINUTO POST DADOS NO HOST NA PORT_HOST
  {
    mtime = millis();  
    getTime();
    //envia_dados_socket();     // comentar para não enviar dados via socket porta 3000 default a cada 1 min
    board.logOnSDFile();      // Salva uma nova linha com dados no seu arquivo 
  }

  atualiza_pagina_AP();
  ArduinoOTA.handle();
  handleSerial();  
  MQTT.loop();
  
  if (! MQTT.connected())  //checa a conexão MQTT
  {
    MQTTConnect();
  }   
}

/***********************************************/

void trata_pwm( void )
{
    dados.sensorData = analogRead(SENSOR_PIN) / 13.65;
    ledcWrite(PWM_PIN_CHANNEL, dados.setPoint);
}

/***********************************************/

void envia_dados_mqtt( void )
{
    sprintf(msg_buffer,"{\"props\":{\"deviceName\":\"%s\",\"macAddress\":\"%s\",\"deviceIp\":\"%d.%d.%d.%d\"},\"date\":\"%s\",\"data\":{\"setPoint\":\"%s\",\"sensorData\":\"%s\"}}", \
            deviceName.c_str(), devicemacAddress.c_str(),myIP[0],myIP[1],myIP[2],myIP[3], dados.date.c_str(),((String)dados.setPoint).c_str(),((String)dados.sensorData).c_str() );
    sprintf(aux_buffer, "%s/%s/rx", deviceName.c_str(), devicemacAddress.c_str());
    MQTT.publish(aux_buffer, msg_buffer);   
    Serial.println(aux_buffer);
}

/***********************************************/

void envia_dados_socket(void)
{
    if(WiFi.status() == WL_CONNECTED) //checa a conexão wifi
    {   
        WiFiClient client_host;    
        Serial.printf( "host: %s",data.host);Serial.print(" ");
        Serial.printf( "port: %d",data.port_host);
        if (!client_host.connect(data.host, data.port_host)) 
        {
          Serial.println("Connection to host failed");
        }
        else
        {
          client_host.print(buffer_serial);
          Serial.println(buffer_serial);           
          Serial.println(" Send to host ok!");
          client_host.stop();
        }
    }
    else 
    {
      Serial.println("No Host connection");  
    }
}

void atualiza_pagina_AP( void )
{
  WiFiClient client = wifiserver.available();   // se tem cliente acessando o esp32 ele envia a pagina para navegar
  if (client) 
  {                             // If a new client connects,

    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) 
    {            // loop while the client's connected
      if (client.available()) 
      {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        //debug martins Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') 
        {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) 
          {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // turns the GPIOs on and off
            if (header.indexOf("GET /LED_PIN/on") >= 0) {
              Serial.println("GPIO LED_PIN on");
              LED_PINState = "on";
              digitalWrite(LED_PIN, HIGH);
            } else if (header.indexOf("GET /LED_PIN/off") >= 0) {
              Serial.println("GPIO LED_PIN off");
              LED_PINState = "off";
              digitalWrite(LED_PIN, LOW);
            }
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta http-equiv=\"refresh\" content=\"3\"/><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");
            
            // Web Page Heading
            client.println("<body><header><h1>Online Status " + deviceName + "</h1></header>");
            
            // Display current state, and ON/OFF buttons for GPIO 2 
            //client.println("<p>GPIO 2 - State " + output2State + "</p>");
            // If the output2 State is off, it displays the ON button       
            //if (output2State=="off") {
            //  client.println("<p><a href=\"/2/on\"><button class=\"button\">ON</button></a></p>");
            //} else {
            //  client.println("<p><a href=\"/2/off\"><button class=\"button button2\">OFF</button></a></p>");
            //} 
            client.println("<!DOCTYPE html> "\
            "<html>"\
            "<head>"\
            "    <style>"\
            "        header {"\
            "            background-color: #004588;"\
            "            padding: 20px;"\
            "            text-align: center;"\
            "            font-size: 20px;"\
            "            color: white;"\
            "        }"\
            "        table {"\
            "            font-family: sans-serif;"\
            "            border-collapse: collapse;"\
            "            width: 100%;"\
            "        }"\
            "        td,"\
            "        th {"\
            "            border-bottom: 1px solid #DDD;"\
            "            text-align: center;"\
            "            padding: 15px;"\
            "        }"\
            "        .th-center {"\
            "            text-align: center;"\
            "        }"\
            "        .td-center {"\
            "            text-align: center;"\
            "        }"\
            "        tr:hover {background-color: #b1d0fe;"\
            "        }"\
            "        footer {"\
            "            padding: 40px;"\
            "            text-align: center;"\
            "            font-size: 12px;"\
            "            color: #63636b;"\
            "        }"\
            "        div.card {"\
            "            width: 350px;"\
            "            box-shadow: 0 4px 8px 0 rgba(0, 0, 0, 0.2), 0 6px 20px 0 rgba(0, 0, 0, 0.19);"\
            "            text-align: center;"\
            "            border-radius: 2%;"\
            "        }"\
            "        div.container {"\
            "            padding: 1px;"\
            "        }"\
            "        .flex-container {"\
            "            display: flex;"\
            "            flex-direction: row;"\
            "            font-size: 12px;"\
            "            text-align: center;"\
            "            align-itens: center;"\
            "            justify-content: center;"\
            "            padding: 5px;"\
            "            gap: 10px 20px;"\
            "        }"\
            "        @media (max-width: 350px) {"\
            "            .flex-container {"\
            "            flex-direction: column;"\
            "        }"\
            "    </style> "\
            "</head> ");

            client.println( "<p>  " \
            "<div> " \
            "<div> " \
            "        <h3>Device Information</h3> " \
            "        <div class=\"flex-container\">" \
            "                <div class=\"card\">" \
            "                     <div class=\"container\">" \
            "                           <p>devidceId:</p>" \
            "                           <p><h2>"+(String)WiFi.macAddress()+"</h2></p>" \
            "                     </div>" \
            "                 </div>" \
            "               <div class=\"card\">" \
            "                     <div class=\"container\">" \
            "                           <p>cardstatus:</p>" \
            "                           <p><h2>"+(String)dados.cardstatus+"</h2></p>" \
            "                     </div>" \
            "               </div>" \
            "               <div class=\"card\">" \
            "                     <div class=\"container\">" \
            "                           <p>date:</p>" \
            "                           <p><h2>"+dados.date+"</h2></p>" \
            "                     </div>" \
            "               </div>" \
            "       </div>" \
            "</div> " \
            "<div> " \
            "        <h3>Power Quality Measurement [ATM90E36A]</h3> " \
            "        <div class=\"flex-container\">" \
            "                <div class=\"card\">" \
            "                     <div class=\"container\">" \
            "                           <p>UrmsA_e36:</p>" \
            "                           <p><h2>"+(String)dados.atm90e36a.UrmsA+"</h2></p>" \
            "                     </div>" \
            "                 </div>" \
            "               <div class=\"card\">" \
            "                     <div class=\"container\">" \
            "                           <p>UrmsB_e36:</p>" \
            "                           <p><h2>"+(String)dados.atm90e36a.UrmsB+"</h2></p>" \
            "                     </div>" \
            "               </div>" \
            "               <div class=\"card\">" \
            "                     <div class=\"container\">" \
            "                           <p>UrmsC_e36:</p>" \
            "                           <p><h2>"+(String)dados.atm90e36a.UrmsC+"</h2></p>" \
            "                     </div>" \
            "               </div>" \
            "       </div>" \
            "       <div class=\"flex-container\">" \
            "                <div class=\"card\">" \
            "                     <div class=\"container\">" \
            "                           <p>IrmsA_e36:</p>" \
            "                           <p><h2>"+(String)dados.atm90e36a.IrmsA+"</h2></p>" \
            "                     </div>" \
            "                 </div>" \
            "               <div class=\"card\">" \
            "                     <div class=\"container\">" \
            "                           <p>IrmsB_e36:</p>" \
            "                           <p><h2>"+(String)dados.atm90e36a.IrmsB+"</h2></p>" \
            "                     </div>" \
            "               </div>" \
            "               <div class=\"card\">" \
            "                     <div class=\"container\">" \
            "                           <p>IrmsC_e36:</p>" \
            "                           <p><h2>"+(String)dados.atm90e36a.IrmsC+"</h2></p>" \
            "                     </div>" \
            "               </div>" \
            "       </div>" \
            "</div> " \
            "<div> " \
            "        <h3>Power Quality Measurement [ATM90E26]</h3> " \
            "        <div class=\"flex-container\">" \
            "                <div class=\"card\">" \
            "                     <div class=\"container\">" \
            "                           <p>Irms_e26:</p>" \
            "                           <p><h2>"+(String)dados.atm90e26a.Irms+"</h2></p>" \
            "                     </div>" \
            "                 </div>" \
            "               <div class=\"card\">" \
            "                     <div class=\"container\">" \
            "                           <p>Urms_e26:</p>" \
            "                           <p><h2>"+(String)dados.atm90e26a.Urms+"</h2></p>" \
            "                     </div>" \
            "               </div>" \
            "       </div>" \
            "</div> " \
            "        <footer> " \
            "                Instituto Maua de Tecnologia" \
            "        </footer> " \
            "    </div></p> "   );
          
            client.println("</body></html>");           
            client.println(); 
            break;  
          } else { // if you got a newline, then clear currentLine 
            currentLine = ""; 
          } 
        } else if (c != '\r') {  
          currentLine += c;      
        }
      }
    }
  
       
    header = "";
    // Close the connection
    client.stop();
    //Serial.println("Client desconnected.");
  }
}
/***********************************************/

void mqtt_callback(char* topic, byte* payload, unsigned int length) 
{
  String msg;

  int i;
  char buffer[10];
  //obtem a string do payload recebido
  // aqui ainda nao de faz o tratamento se é o tópico certo!!!
  for (i = 0; i < length; i++)
  {
    char c = (char)payload[i];
    msg += c;
    buffer[i] = (char)payload[i];
  }
  buffer[i] = '\0';
  dados.setPoint = atoi(buffer);

  Serial.print("[MQTT] Mensagem recebida: ");
  Serial.println(msg);
}
/***********************************************/

void handleSerial()
{
    static int length = 0; // number of characters currently in the buffer

    if (timeout_serial >= 3) //deu timeout
    {
      timeout_serial = 0;
      length = 0;
    }
    
    if(SerialExtern.available())
    {
        timeout_serial = 0;
        char c = SerialExtern.read();
        if(c == '\n')
        {
            // end-of-line received
            if(length > 0)
            {
                handleReceivedMessage();
            }
            length = 0;
        }
        else
        {
            if(length < 1000)
            {
                buffer_serial[length++] = c; // append the received character to the array
                buffer_serial[length] = '\0'; // append the null terminator
            }
        }
    }
}

 /********************** uart routine *****************/

void handleReceivedMessage()
{
  char *ret;

  ret = strstr(buffer_serial,"AT+CIFSR?");
  if(ret)
  { 
    sprintf(msg_buffer,"{\"props\":{\"deviceName\":\"%s\",\"macAddress\":\"%s\",\"deviceIp\":\"%d.%d.%d.%d\"},\"date\":\"%s\"}", \
            deviceName, devicemacAddress.c_str(), myIP[0],myIP[1],myIP[2],myIP[3], dados.date.c_str() );
    
    SerialExtern.print(msg_buffer);
    Serial.println("AT+CIFSR rcv>");
  }
  else
  {
    ret = strstr(buffer_serial,"package");
    if(ret)
    {
      #ifdef PUBLICA_VIA_MQTT
        sprintf(aux_buffer, "%s/%s/rx", deviceName.c_str(), devicemacAddress.c_str());
        MQTT.publish(aux_buffer, buffer_serial);
        SerialExtern.println("OK");
        Serial.print(data.mqtt_server); Serial.print(": "); Serial.print(data.mqtt_port); Serial.print(" > ");  Serial.print(data.mqtt_topic); Serial.print(": ");
        Serial.println(buffer_serial);
      #endif
      #ifdef PUBLICA_VIA_SOCKET 
        envia_dados_socket();     // comentar para não enviar dados via socket porta 3000 default a cada 1 min
      #endif

    }
  }  
}

/***********************************************/

// Modificação para conectar a uma wifi
void networkConnect()
{
  // Para qualquer aplicação bluetooth que possa existir
  btStop();

  wm.addParameter(&host);
  wm.addParameter(&parameter);
  wm.addParameter(&port_host);
  wm.addParameter(&custom_mqtt_server);
  wm.addParameter(&custom_mqtt_port);
  wm.addParameter(&custom_mqtt_user);
  wm.addParameter(&custom_mqtt_password);
  wm.addParameter(&custom_mqtt_topic);

  // Começa WiFi se conectando ao SSID fornecido com a senha fornecida
  //WiFi.begin("NomeDaSuaWiFi", "senhadaSuaWiFi");
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP 
  
  if(wm_nonblocking) wm.setConfigPortalBlocking(false);
  
  // callbacks
  wm.setAPCallback(configModeCallback);
  wm.setWebServerCallback(bindServerCallback);
  wm.setSaveConfigCallback(saveWifiCallback);
  wm.setSaveParamsCallback(saveParamCallback);

  std::vector<const char *> menu = {"wifi","info","param","sep","restart","exit"};
  wm.setMenu(menu);

  // set dark theme
  wm.setClass("invert");

  Serial.println("Configuring access point...");

  devicemacAddress = WiFi.macAddress(); 
  char *p;
  p = &devicemacAddress[0];
  strcpy(aux_buffer,p);
  devicemacAddress = aux_buffer;
  
  wm.setConfigPortalTimeout(60); // auto close configportal after n seconds
  sprintf(msg_buffer, "%s-%s-CFG", deviceName, devicemacAddress.c_str());
  wm.setHostname(msg_buffer);
  
 // This is sometimes necessary, it is still unknown when and why this is needed but it may solve some race condition or bug in esp SDK/lib
  //wm.resetSettings();
  wm.disconnect();
  wm.setCleanConnect(true); // disconnect before connect, clean connect
  wm.setConnectTimeout(15);
  wm.setConnectRetries(3);

  bool res;
  res = wm.autoConnect(msg_buffer,"password"); // password protected ap

  if(!res) 
  {
    Serial.println("Failed to connect or hit timeout");
    ESP.restart();
  } 
  else 
  {
    //if you get here you have connected to the WiFi    
    Serial.println("connected...yeey :)");
    // You can remove the password parameter if you want the AP to be open.
    sprintf(msg_buffer, "%s-%s-AP", deviceName, devicemacAddress.c_str());
    WiFi.softAP(msg_buffer, "password");
    myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
    wifiserver.begin();    
  }

  // Exibe via serial o endereço IP do seu kit na rede 
  Serial.println("");
  Serial.println("WiFi conectada");
  Serial.println("IP address: ");
  myIP = WiFi.localIP();
  Serial.println(myIP);

      
  Serial.println("Parameter:");
  char s_aux[10] = "";
  strcpy(s_aux,port_host.getValue());
  data.port_host = atoi(s_aux); 
  strcpy(data.host,host.getValue());

  strcpy(data.mqtt_server,custom_mqtt_server.getValue());
  strcpy(s_aux,custom_mqtt_port.getValue());
  data.mqtt_port = atoi(s_aux); 
  strcpy(data.mqtt_user,custom_mqtt_user.getValue());
  strcpy(data.mqtt_password,custom_mqtt_password.getValue());
  
  //strcpy(data.mqtt_topic,custom_mqtt_topic.getValue());   // trava topico para evitar que haja confusao em post errados no servidor
  sprintf(msg_buffer, "%s/%s", deviceName, devicemacAddress.c_str());
  strcpy(data.mqtt_topic,msg_buffer);
 
  Serial.print(data.host); Serial.print(": "); Serial.println(data.port_host);
  Serial.print(data.mqtt_server); Serial.print(": "); Serial.print(data.mqtt_port); Serial.print(" > ");  Serial.println(data.mqtt_topic);
}
// Declaração de funções que sobrescrevem as funcionalidades padrão

/*************** SD CARD ********************************/

// Função Modificada
void createFileFirstLine(fs::FS &fs, const char * path)
{
  // Mostra o nome do arquivo
  Serial.printf("Escrevendo em: %s\n", path);
  
  //Abre o arquivo do SD para a memória RAM
  File file = fs.open(path, FILE_WRITE);
  if(!file)
  {
    Serial.println("Falha ao abrir para escrita");
    return;
  }
  // Cria a primeira linha modificada e separada por vírgulas do CSV. 
  const char * message = "tempo(ms),temperatura(C),umidade(%),co2(ppm),bateria(%)";
  
  // Escreve a mensagem criada anteriormente
  if(file.println(message))
  {
    Serial.println("Escrita Começou");
  } 
  else 
  {
    Serial.println("Falha na escrita");
  }
  // Fecha o arquivo
  file.close();
}

// Função Modicada para armazenamento
void appendFile(fs::FS &fs, const char * path, TickType_t time)
{
  //Abre o arquivo do SD para a memória RAM
  File file = fs.open(path, FILE_APPEND);
  if(!file)
  {
    Serial.println("Falha ao abrir para gravacao");
    return;
  }
  // Salva no CSV o dado, seguido de uma vírgula. 
  file.print(time);
  file.write(',');
  file.print(board.getTemperature());file.write(',');
  file.print(board.getHumidity());file.write(',');
  file.print(board.getCO2Level());file.write(',');
  file.print(board.getBattery());file.write(',');
  file.print(board.getPressure());file.write(',');
  file.print(board.getLuminosity());  file.write(',');
  file.print(board.getAccelerometer(0));file.write(',');
  file.print(board.getAccelerometer(1));file.write(',');
  file.print(board.getAccelerometer(2));file.write(',');
  file.print(board.getGyroscope(0));file.write(',');
  file.print(board.getGyroscope(1));file.write(',');
  file.print(board.getGyroscope(2));file.write(',');
  file.print(board.getMagnetometer(0));file.write(',');
  file.print(board.getMagnetometer(1));file.write(',');
  file.println(board.getMagnetometer(2));
  // Fecha o arquivo
  file.close();
}


/***************WIFI MANAGER ********************************/

String getParam(String name)
{
  //read parameter from server, for customhmtl input
  String value;
  if(wm.server->hasArg(name)) 
  {
    value = wm.server->arg(name);
  }
  return value;
}

void saveWifiCallback()
{
  Serial.println("[CALLBACK] saveCallback fired");
}

//gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) 
{
  Serial.println("[CALLBACK] configModeCallback fired");
}

void saveParamCallback(){
  Serial.println("[CALLBACK] saveParamCallback fired");
  Serial.println("PARAM customfieldid = " + getParam("customfieldid"));
}

void bindServerCallback(){
  wm.server->on("/custom",handleRoute);
  // wm.server->on("/info",handleRoute); // you can override wm!
}

void handleRoute(){
  Serial.println("[HTTP] handle route");
  wm.server->send(200, "text/plain", "hello from user code");
}

void getTime() 
{
  char buffer_aux[26];
  
  if(!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }
   strftime(buffer_aux, sizeof(buffer_aux), "%Y:%m:%d %H:%M:%S", &timeinfo);
   dados.date = String(buffer_aux);
   Serial.print("Current Date: ");    
   Serial.println(dados.date);
}

void wifiInfo()
{
  WiFi.printDiag(Serial);
  Serial.println("SAVED: " + (String)wm.getWiFiIsSaved() ? "YES" : "NO");
  Serial.println("SSID: " +  (String)wm.getWiFiSSID());
  Serial.println("PASS: " +  (String)wm.getWiFiPass());
}


/********************* BOTOES**************************/

void checkButton()
{
  static char contador = 0;
  // check for button press
  if ( digitalRead(TRIGGER_PIN) == LOW ) 
  {
    Serial.println("Button Pressed");
    if (contador >= 30)
    {
        contador = 0;
        Serial.println("Button Held");
        Serial.println("Erasing Config, restarting");
        wm.resetSettings();
        ESP.restart();
    }
    contador++;   
  }
  else
  {
    if (contador)
    {
        wifiserver.stop(); 
        Serial.println("Starting config portal");
        wm.disconnect();
        sprintf(msg_buffer, "%s-%s-CFG", deviceName, devicemacAddress.c_str());
        if (!wm.startConfigPortal(msg_buffer,"password"))
        {
          Serial.println("ondemand desconectado");
          sprintf(msg_buffer, "%s-%s-CFG", deviceName, devicemacAddress.c_str());
          WiFi.softAP(msg_buffer, "password");
          IPAddress myIP = WiFi.softAPIP();
          Serial.print("AP IP address: ");
          Serial.println(myIP);
          wifiserver.begin();    
          //ESP.restart();
        } 
        else 
        {
          //if you get here you have connected to the WiFi
          Serial.println("connected on portal ondemand :)");
        }
    }
    contador = 0; 
  }
}

/********************* MQTT **************************/
void MQTTConnect(void) 
{
  Serial.print("Conectando ao broker ");
  Serial.print(data.mqtt_server);
  Serial.println(" ...");

  if (MQTT.connect("ESP32Client", data.mqtt_user, data.mqtt_password)) 
  {
    Serial.println("Conectado ao broker");
    MQTT.setBufferSize(1000);

    // cadastra tópicos para serem recebidos
    sprintf(aux_buffer, "%s/%s/tx",deviceName.c_str(), devicemacAddress.c_str());
    MQTT.subscribe(aux_buffer);
  } 
  else 
  {
    Serial.print("Falha na conexão. Estado: ");
    Serial.println(MQTT.state());
  }
}
