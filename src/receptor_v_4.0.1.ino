/*
    Projeto: Receptor LoRa.
    Autor: Rodrigo Rocha.
    Data: 11/2022.
    Versão: v_4.0.1
    Código de Domínio Público.

    Este código se refere ao receptor de um projeto de mestrado do curso de Ciências da Computação 
    da UFSJ(Universidade Federal de São João del-Rei) que tem por finalidade testar a viabilidade 
    da comunicação LoRaWan que é o objetivo final do projeto. Para que este funcione é preciso do 
    transmissor que envia informações relevantes ao seu posicionamento geográfico além de dados 
    sobre sua transmissão. Este dispositivo salva esses dados em uma unidade de armazenamento 
    (micro SD card). Com essas informações é possível o posterior mapeamento do alcance da comunicação 
    fazendo uso das coordenadas geográficas registradas e demais informações gravadas no arquivo DATALOG.CSV.
	
	Obs: Para que os dois dispositivos se comuniquem é imprescindível a configuração de ambos com o
	mesmo fator de espalhamento 'SF'.
	
 */

#include <SPI.h>             
#include <LoRa.h>
#include <Wire.h> 
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <avr/wdt.h>

#define LED_AMARELO   5
#define LED_VERDE     6  
#define LED_VERMELHO  7

#define SF            7     			    // Varia de SF7 a SF12, padrão é SF7.
#define TX_Power      20      				// Potência TX em dB, padrão é 17 configurável de 0 até 20 dB       
 
#define localAddress  0x02    				// Endereco deste dispositivo LoRa
#define destination   0x01    				// Endereco do dispositivo para enviar a mensagem (0xFF envia para todos devices)
#define interval      5000    				// Intervalo em ms no envio das mensagens (inicial 5s)

TinyGPSPlus gps;			 				// Cria o objeto gps	  	
SoftwareSerial ss(3, 2);     				// Biblioteca usada para simular uma porta serial para ser usada pelo módulo GPS

int8_t msgCount = 0;    
long   lastSendTime = 0; 
String mensagem;              

File myFile;								// Cria o objeto myFile para manipulação de arquivos
 
void setup(){
  wdt_enable(WDTO_8S);						//Habilita o watchdog com tempo de 8 segundos
  pinMode(LED_AMARELO, OUTPUT);
  pinMode(LED_VERDE, OUTPUT);
  pinMode(LED_VERMELHO, OUTPUT);
  ss.begin(9600);		    				// Configura o baud rate para comunicação com o módulo GPS	

  if (!LoRa.begin(915E6)){ 					// Inicializa o modem LoRa com a frequência central de 915 Mhz            
    digitalWrite(LED_AMARELO, HIGH);		// Acende o Led amarelo quando há erro no modem LoRa
    while (true);                      
  }
  if (SF == 11){
    LoRa.setSpreadingFactor(10);
  }else{
    LoRa.setSpreadingFactor(SF); 			// Configura o fator de espalhamento
  }
  LoRa.setTxPower(TX_Power);				// Configura a potência de transmissão de 0 até 20 dB 
  
/* Pisca os leds verde e vermelho indicando o funcionamento do modem LoRa */ 
  for (int i = 0; i < 10; i++) { 
   digitalWrite(LED_VERDE, HIGH);
   delay(30); 
   digitalWrite(LED_VERDE, LOW); 
   delay(30); 
   digitalWrite(LED_VERMELHO, HIGH); 
   delay(30); 
   digitalWrite(LED_VERMELHO, LOW); 
   delay(30); 
  }
  if (!SD.begin(4)) {						//Inicia cartão SD
    digitalWrite(LED_AMARELO, HIGH);		// Acende o Led amarelo quando há erro no módulo SD card
    while (1);
  }
/* Pisca os leds amarelo e vermelho indicando o funcionamento do módulo SD card */   
  for (int i = 0; i < 10; i++) { 
   digitalWrite(LED_AMARELO, HIGH);
   delay(30); 
   digitalWrite(LED_AMARELO, LOW); 
   delay(30); 
   digitalWrite(LED_VERMELHO, HIGH); 
   delay(30); 
   digitalWrite(LED_VERMELHO, LOW); 
   delay(30); 
  }
}
 
void loop(){
  wdt_reset();								// Restarta o watchdog
  mensagem = "";
  digitalWrite(LED_VERMELHO, HIGH);
  if (onReceive(LoRa.parsePacket())){
    if (mensagem == "SYN"){
      if (millis() - lastSendTime > interval){
        enviarACK();
        lastSendTime = millis();
      }
    }
  }
}
 
/* Confirma o recebimento do SYN enviado pelo transmissor para iniciar as transmissões e grava uma linha contendo zeros no arquivo */ 
void enviarACK(){
  sendMessage("ACK");						// Confirma o recebimento da mensagem SYN enviada pelo transmissor enviando um ACK
	mensagem = "";
	mensagem.concat("0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0"); // Adiciona zeros na mensagem
	salvar(mensagem);
}

/* Alerta visual de falha */
void alertaFalha(){
	digitalWrite(LED_AMARELO, HIGH);
    delay (100);
    digitalWrite(LED_AMARELO, LOW);
    delay (100);
    digitalWrite(LED_AMARELO, HIGH);
    delay (100);
    digitalWrite(LED_AMARELO, LOW);
}

/* Envia uma mensagem LoRa */
void sendMessage(String outgoing) 
{
  LoRa.beginPacket();                   	// Inicia o pacote da mensagem
  LoRa.write(destination);              	// Adiciona o endereco de destino
  LoRa.write(localAddress);             	// Adiciona o endereco do remetente
  LoRa.write(msgCount);                 	// Contador da mensagem
  LoRa.write(outgoing.length());        	// Tamanho da mensagem em bytes
  LoRa.print(outgoing);                 	// Vetor da mensagem 
  LoRa.endPacket();                     	// Finaliza o pacote e envia
  msgCount++;                           	// Contador do numero de mensagnes enviadas
}
 
/* Recebe uma mensagem LoRa */ 
int onReceive(int packetSize){
  if (packetSize == 0) return;          	// Se nenhuma mesnagem foi recebida, retorna nada
  int recipient = LoRa.read();          	// Endereco de quem ta recebendo
  byte sender = LoRa.read();            	// Endereco do remetente
  byte incomingMsgId = LoRa.read();     	// ID da mensagem recebida
  byte incomingLength = LoRa.read();    	// Tamanho da mensagem recebida
 
  mensagem = "";
 
  while (LoRa.available()){
    mensagem += (char)LoRa.read();			// Monta a mensagem recebida
  }
 
  if (incomingLength != mensagem.length()){	// Mensagem corrompida 
    alertaFalha();
    return 0;                        
  }

  if (recipient != localAddress && recipient != 0xFF){ // Esta mensagem não é para esse endereço
    alertaFalha(); 
    return 0;
  }
  if (mensagem != "SYN"){
/* Concatena dados do receptor com a mensagem recebida para salvar no arquivo */	  
    mensagem.concat(';');
    mensagem.concat(LoRa.packetSnr());
    mensagem.concat(';');
    mensagem.concat(LoRa.packetRssi());

    smartDelay(1000);						// Captura das informações vindas do módulo GPS
   
    mensagem.concat(';');
    mensagem.concat(gps.satellites.value());
    mensagem.concat(';');
    mensagem.concat(gps.hdop.hdop());
    mensagem.concat(';');
    mensagem.concat(String(gps.location.lat(), 5));
    mensagem.concat(';');
    mensagem.concat(String(gps.location.lng(), 5));
    mensagem.concat(';');
    mensagem.concat abs((gps.altitude.meters()));
    mensagem.concat(';');
    if(gps.time.hour() < 10) {
      mensagem.concat('0');
    }
    mensagem.concat(gps.time.hour() - 3);
    mensagem.concat(':');
    if(gps.time.minute() < 10) {
     mensagem.concat('0');
    }
    mensagem.concat(gps.time.minute());
    mensagem.concat(':');
    if(gps.time.second() < 10) {
     mensagem.concat('0'); 
    }
    mensagem.concat(gps.time.second());

    salvar(mensagem); /* Salva a string no arquivo com os dados do transmissor e do receptor  */
    digitalWrite(LED_VERMELHO, LOW);
    digitalWrite(LED_VERDE, HIGH);
    delay (100);
    digitalWrite(LED_VERDE, LOW);
    delay (200);
    digitalWrite(LED_VERDE, HIGH);
    delay (100);
    digitalWrite(LED_VERDE, LOW);
  } 
  return 1;
}

/* ------------------------------ Leitura do GPS------------------------------------- */
static void smartDelay(unsigned long ms){
  unsigned long start = millis();
  do {
    while (ss.available())
      gps.encode(ss.read());				// Leitura das informações do módulo GPS
  } while (millis() - start < ms);
}

void zero(){
 mensagem.concat('0');
}

void intervalo(){
 mensagem.concat(';');
}

/* ----------- Persiste o conteúdo da string no arquivo "dataset.csv" ------------ */
void salvar(String msg){
  myFile = SD.open("dataset.csv", FILE_WRITE);
  if (myFile) {
    myFile.println(msg);
    myFile.close();
  } else {
    digitalWrite(LED_AMARELO, HIGH);
    delay(30);
    digitalWrite(LED_AMARELO, LOW); 
    delay(30);
    digitalWrite(LED_AMARELO, HIGH);
    delay(30);
    digitalWrite(LED_AMARELO, LOW); 
  }
}
