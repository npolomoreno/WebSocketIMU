#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <Wire.h>


//variables del MPU------------------------------------------------------
#define MPU 0x68

//Ratios de conversion
#define A_R 16384.0 // 32768/2
#define G_R 131.0 // 32768/250
 
//Conversion de radianes a grados 180/PI
#define RAD_A_DEG = 57.295779
 
//MPU-6050 da los valores en enteros de 16 bits
//Valores RAW
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
 
//Angulos
float Acc[2];
float Gy[3];
float Angle[3];

String valores;

long tiempo_prev;
float dt;

boolean  mpuOk = false;
boolean  wifiOk = false;
//----------------------------------------------------------***


// variables del WebSocket---------------------------------------------------------
WebSocketsServer webSocket = WebSocketsServer(80);
boolean  socketConect = false;
boolean enviar = false;

unsigned long tiempo1 = 0;
unsigned long tiempo2 = 0;

char message[7];
String msgStr;
//-------------------------------Manejador de eventos del websocket----------------------------------------------------------
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Desconectado!\n", num);
            socketConect =false;
            enviar = false;
            break;
        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            Serial.printf("[%u] Conectado con %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
            // send message to client
            //webSocket.sendTXT(num, "Conectado");
            socketConect = true;
        }
            break;
        case WStype_TEXT:

            Serial.printf("[%u] Mensaje recibido: %s\n", num, payload);

            if(payload[0] == 'O' && payload[1] == 'N'){
                enviar = true;
                Serial.println("Empezando a enviar datos");
            }
            if(payload[0] == 'O' && payload[1] == 'F'){
                enviar = false;
                Serial.println("Envío de datos cancelado");
            }

            if(payload[0] == '#') {
                //uint32_t rgb = (uint32_t) strtol((const char *) &payload[1], NULL, 16);
            }
            break;
    }
}

// ------------------------------------------***

// variables de la red wifi --------------
const char* ssid = "ELECTRONICA";
const char* password = "naso19681966";


void setup() {

    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.printf("WiFi ha fallado :( \n");
        return;
    }
    else{
        wifiOk = true;
    }
    
    Serial.println();
    Serial.print("Conexión exitosa, IP: ");
    Serial.println(WiFi.localIP());
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    
    //setup MPU-------------------------------------------------------------
    Wire.begin(4,5); // D2(GPIO4)=SDA / D1(GPIO5)=SCL
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    pinMode(LED_BUILTIN, OUTPUT);

    tiempo1 = millis();

}

void loop() {
    webSocket.loop();
 //Leer los valores del Acelerometro de la IMU
   Wire.beginTransmission(MPU);
   Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true);   //A partir del 0x3B, se piden 6 registros
   AcX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   AcY=Wire.read()<<8|Wire.read();
   AcZ=Wire.read()<<8|Wire.read();
 
   //A partir de los valores del acelerometro, se calculan los angulos Y, X
   //respectivamente, con la formula de la tangente.
   Acc[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
   Acc[0] = atan((AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
 
   //Leer los valores del Giroscopio
   Wire.beginTransmission(MPU);
   Wire.write(0x43);
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true);   //A partir del 0x43, se piden 6 registros
   GyX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   GyY=Wire.read()<<8|Wire.read();
   GyZ=Wire.read()<<8|Wire.read();
 
   //Calculo del angulo del Giroscopio
   Gy[0] = GyX/G_R;
   Gy[1] = GyY/G_R;
   Gy[2] = GyZ/G_R;

   dt = (millis() - tiempo_prev) / 1000.0;
   tiempo_prev = millis();
 
   //Aplicar el Filtro Complementario
   Angle[0] = 0.98 *(Angle[0]+Gy[0]*dt) + 0.02*Acc[0];
   Angle[1] = 0.98 *(Angle[1]+Gy[1]*dt) + 0.02*Acc[1];

   //Integración respecto del tiempo paras calcular el YAW
   Angle[2] = Angle[2]+Gy[2]*dt;
 

    if(String(Angle[0]) == "nan" && mpuOk==false){
        //mpuOk = true;
        Serial.println("estado: NAN");
        ESP.reset();
    }

    if (wifiOk && String(Angle[0]) != "nan"){
        digitalWrite(LED_BUILTIN, LOW);
        wifiOk = false;
    }

    if (enviar){
        tiempo2 = millis();
        if(tiempo2 > (tiempo1+100)){  //Si ha pasado 1 segundo ejecuta el IF
            tiempo1 = millis();

            if(socketConect){
                msgStr ='#'+ String((int)Angle[0]);
                msgStr.toCharArray(message, 6);
                webSocket.sendTXT(0, message);
                Serial.println(msgStr);
            }
        }

    }
  
}
