#include <HTTP_Method.h>
#include <Uri.h>
#include <WebServer.h>
#include <Wire.h>
#include <WiFi.h>

//APÓS COMPILAR E ENVIAR O CODIGO, O SERVIDOR ESTARÁ DISPONIVEL NO NAVEGADOR, BASTA DIGITAR O IP DO ESP
//O IP DO ESP DEVE, NA TEORIA, APARECER NO MONITOR SERIAL

//SABADO, A FINS DE TESTES, MUDE PARA O WIFI DA SUA CASA
const char* ssid = "HUAWEI-5G-Zs26";
const char* password = "HcrcvARh";

WebServer server(80);//o servidor estara disponivel na porta 80.

void handleRoot(){
  server.send(200, "text/plain"," hello, world!"); //quando requisitarem algo na raiz (root) responde com hello world
 
}

void setup() {
  Serial.begin(115200);

  //CONEXAO WIFI
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED){

    delay(1000);
    Serial.println("connecting to WIFI ...");
  }
  Serial.println("connected");

  //configura a rota para raiz
  server.on("/", handleRoot);

  server.begin();

  Serial.println("server started");

}

void loop() {
  // put your main code here, to run repeatedly:
  server.handleClient();

}
