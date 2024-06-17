/*
    Projeto: Transmissor LoRa
    Autor: Rodrigo Rocha
    Data: 11/2024
    Versão: v_4.0.1
    Código de Domínio Público

    Este código se refere ao transmissor de um projeto de mestrado do curso de Ciências da Computação 
    da UFSJ(Universidade Federal de São João del-Rei) que tem por finalidade testar a viabilidade da 
    comunicação LoRaWan que é o objetivo final do projeto. Para que este funcione é preciso do receptor 
    que tem por finalidade registrar as informações enviadas pela string e persisti-las em uma unidade 
    de armazenamento (micro SD card). Com essas informações é possível o posterior mapeamento do alcance 
    da comunicação fazendo uso das coordenadas geográficas registradas e demais informações gravadas 
	no arquivo DATALOG.CSV que foi salvo no dispositivo receptor.
	
	Obs: Para que os dois dispositivos se comuniquem é imprescindível a configuração de ambos com o
	mesmo fator de espalhamento 'SF'.
	
 */

#include <SPI.h>
#include <LoRa.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h> 
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <avr/wdt.h>

#define BOTAO_ENVIAR  4
#define LED_AMARELO   5
#define LED_VERDE     6  
#define LED_VERMELHO  7
#define BUZZER        A3       				// A porta analógica A3 está sendo utilizada como digital
#define LMT_DISPARO   5        				//  1 -> teste distância |  5 -> teste de perda de pacotes
#define LMT_PACOTE    20       				// 10 -> teste distância | 20 -> teste de perda de pacotes

#define SF            7        				// Varia de SF7 a SF12, padrão é SF7.
#define TX_Power      20       				// Potência TX em dB, padrão é 17 configurável de 0 até 20 dB

#define localAddress  0x01     				// Endereco deste dispositivo LoRa
#define destination   0x02     				// Endereco do dispositivo para enviar a mensagem (0xFF envia para todos devices)
#define interval      5000     				// Intervalo em ms no envio das mensagens (inicial 5s)

TinyGPSPlus gps;               				// Cria o objeto gps	
SoftwareSerial ss(3, 2);	     			// Biblioteca usada para simular uma porta serial para ser usada pelo módulo GPS	
LiquidCrystal_I2C lcd(0x27,16, 2);
 
String outgoing     = "";      				// Mensagem de saída
int8_t msgCount     = 0;       				// Contador de mensagens enviadas
long   lastSendTime = 0;       				// TimeStamp da ultima mensagem enviada
String mensagem     = "";
int8_t disparo      = 0;
int8_t pacote       = 0;
 
void setup(){
  wdt_enable(WDTO_8S);			   		// Habilita o watchdog com tempo de 8 segundos
  buzzer_on();
  pinMode(BOTAO_ENVIAR, INPUT);				// Configura o pino D4 do MCU como entrada
  pinMode(LED_AMARELO, OUTPUT);				// Configura o pino D5 do MCU como saida
  pinMode(LED_VERDE, OUTPUT);				// Configura o pino D6 do MCU como saida
  pinMode(LED_VERMELHO, OUTPUT);			// Configura o pino D7 do MCU como saida
  pinMode(BUZZER, OUTPUT);    				// Configura o pino A3 analógico para saída digital
  ss.begin(9600);				      	// Configura o baud rate para comunicação com o módulo GPS
  lcd.init();
  lcd.backlight();
  LoRa.setPins(10, 9, 8);  				// Sobrescreve os pinos NSS, RESET e DIO padrão usados pela biblioteca
  if (!LoRa.begin(915E6)){ 				// Inicializa o modem LoRa com a frequência central de 915 Mhz            
    digitalWrite(LED_AMARELO, HIGH);			// Acende o Led amarelo quando há erro no modem LoRa
    while (true){					// Trava o código até que o dispositivo seja reiniciado
      wdt_reset(); 				    	// Restarta o watchdog                    
    }                      
  }
  LoRa.setSpreadingFactor(SF); 				// Configura o fator de espalhamento SF7...SF12  
  LoRa.setTxPower(TX_Power);  				// Configura a potência de transmissão de 0 até 20 dB   
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
}

void loop(){
  digitalWrite(LED_VERMELHO, HIGH);			// Acende o LED vermelho entrando em estado de espera
  while(digitalRead(BOTAO_ENVIAR)){
    wdt_reset();					// Restarta o watchdog
    smartDelay(200);					// Coleta as informações vindas do módulo GPS
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Altitude: ");				
    lcd.print abs((gps.altitude.meters())); 		// Atualiza o valor da Altitude no LCD
    lcd.setCursor(0,1);
    lcd.print("Sat.: ");
    lcd.print(gps.satellites.value());			// Atualiza o valor dos Satélites no LCD
    lcd.print("  SF: "); 
    lcd.print(SF); 					// Atualiza o valor do SF no LCD
  }  
  delay(500);                 				// Impede o disparo múltiplo do botão enviar
  digitalWrite(LED_VERMELHO, LOW);			
  if (millis() - lastSendTime > interval){
    sendMessage("SYN");					// Envia "SYN" ao receptor 
    lastSendTime = millis();  				
  }
  if (onReceive(LoRa.parsePacket())){			// Entra em estado de escuta até receber o "ACK" do receptor
    if (mensagem == "ACK"){   				
      buzzer();               				// Confirmação sonora indicando o início do envio das mensagens  
/* Looping mais externo */	  
      for (int8_t i = 0; i < LMT_DISPARO; i++){  
        wdt_reset();					// Restarta o watchdog
        disparo++;
        if(i != 0){           				// Verifica se não é o primeiro disparo
          for(int8_t count = 0; count < 120 ; count++){ // Intervalo de 2 segundos entre os disparos
            wdt_reset();      				// Restarta o watchdog
            delay(1000);  
          }
        }
/* Looping mais interno */		
        for (int8_t j = 0; j < LMT_PACOTE; j++){ 
          wdt_reset();					// Restarta o watchdog      
          lcd_gps();     				// Lê o GPS, atualiza o LCD e monta a mensagem a ser enviada
          sendMessage(String(mensagem));		// Envio da mensagem concatenada ao receptor
          mensagem = "";
        }
      }
      disparo = 0;     
      buzzer();						// Confirmação sonora do término das menssagens enviadas
      delay(350);
      buzzer();
      delay(350);
      buzzer();
    }  
  }
}

void buzzer_on(){
  tone(BUZZER,1000,350);
  delay(350);
  noTone(BUZZER);
}

void buzzer(){							
  tone(BUZZER,2000,100);
  delay(100);
  noTone(BUZZER);
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
void sendMessage(String outgoing){
  digitalWrite(LED_VERMELHO, HIGH);
  LoRa.beginPacket();        		           	// Inicia o pacote da mensagem
  LoRa.write(destination);              		// Adiciona o endereco de destino
  LoRa.write(localAddress);      		       	// Adiciona o endereco do remetente
  LoRa.write(msgCount);                 		// Contador da mensagem
  LoRa.write(outgoing.length());   		     	// Tamanho da mensagem em bytes
  LoRa.print(outgoing);                 		// Vetor da mensagem 
  LoRa.endPacket();                     		// Finaliza o pacote e envia
  msgCount++;                 		          	// Contador do numero de mensagnes enviadas
  digitalWrite(LED_VERMELHO, LOW);
}
/* Recebe uma mensagem LoRa */ 
int8_t onReceive(int8_t packetSize){
  if (packetSize == 0) return 0;    			// Se nenhuma mesnagem foi recebida, retorna nada         
  int8_t recipient = LoRa.read(); 		    	// Endereco de quem ta recebendo         
  byte sender = LoRa.read();            		// Endereco do remetente
  byte incomingMsgId = LoRa.read();     		// ID da mensagem recebida
  byte incomingLength = LoRa.read();	 	  	// Tamanho da mensagem recebida    
  while (LoRa.available()){
    mensagem += (char)LoRa.read();		  	// Monta a mensagem recebida
  }
  if (incomingLength != mensagem.length()){ 		// Mensagem corrompida   
    void alertaFalha();
    return; 
  }                         
  if (recipient != localAddress && recipient != 0xFF){  // Esta mensagem não é para esse endereço
    void alertaFalha();
    return;
  }
/* Pisca os Leds verdes para confirmar o recebimento de uma mensagem */
  digitalWrite(LED_VERDE, HIGH);
  delay (100);
  digitalWrite(LED_VERDE, LOW);
  delay (200);
  digitalWrite(LED_VERDE, HIGH);
  delay (100);
  digitalWrite(LED_VERDE, LOW);
  return 1;
}

// ------------------------------LCD e GPS-------------------------------------
static void smartDelay(unsigned long ms){
  unsigned long start = millis();
  do {
    while (ss.available())
      gps.encode(ss.read());				// Traduz as informações do módulo GPS
  } while (millis() - start < ms);
}

void apagaLinha1(){
  lcd.setCursor(0,0);
  for(int8_t i =0; i<16; i++) lcd.print(" ");
  lcd.setCursor(0,0);
}

void zero(){
  lcd.print('0'); 
  mensagem.concat('0');
}

void intervalo(){
  mensagem.concat(';');
}

/* Atualiza dados do LCD e monta a mensagem para ser enviada */
String lcd_gps(){
  smartDelay(500);					// Coleta as informações vindas do módulo GPS
  
  int8_t satelites = gps.satellites.value();
  float  prec      = gps.hdop.hdop();
  float  lati      = gps.location.lat();
  float  longi     = gps.location.lng();
  float  alti      = gps.altitude.meters();
  int8_t dia       = gps.date.day();
  int8_t mes       = gps.date.month();
  int16_t ano      = gps.date.year();
  int8_t hora      = gps.time.hour();
  int8_t minuto    = gps.time.minute();
  int8_t segundo   = gps.time.second();
  
  mensagem = "";
  pacote ++;
  if (pacote > LMT_PACOTE) pacote = 1;  		// Reinicia o n° do pacote ao atingir o limite configurado
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print("Sat.: ");
  lcd.print(satelites);
  lcd.print("  SF: "); 
  lcd.print(SF);
  mensagem.concat(disparo); 
  intervalo();
  mensagem.concat(pacote);  
  intervalo();
  mensagem.concat(satelites);  
  intervalo();
  delay(300);
  apagaLinha1();
  lcd.print("Prec: ");
  lcd.print(prec);
  mensagem.concat(prec);  
  intervalo();
  delay(300);
  apagaLinha1();
  lcd.print("Lat: ");
  lcd.print(lati, 5);
  mensagem.concat(String(lati, 5)); 
  intervalo();
  delay(300);
  apagaLinha1();
  lcd.print("Lon.: ");
  lcd.print(longi, 5);
  mensagem.concat(String(longi, 5)); 
  intervalo();
  delay(300);
  apagaLinha1();
  lcd.print("Alt: ");
  lcd.print abs((alti));
  mensagem.concat abs((alti));  
  intervalo();
  delay(300);
  apagaLinha1();
  if(dia < 10) {
    zero();
  }
  lcd.print(dia);
  mensagem.concat(dia);
  lcd.print("/");
  mensagem.concat('/');
  if(mes < 10){
    zero();
  }
  lcd.print(mes);
  mensagem.concat(mes);
  lcd.print("/");
  mensagem.concat('/');
  lcd.print(ano);
  mensagem.concat(ano);
  intervalo();
  delay(300);
  apagaLinha1();
  if(hora < 10) {
    zero();
  }
  lcd.print(hora - 3);
  mensagem.concat(hora - 3);
  lcd.print(":");
  mensagem.concat(':');
  if(minuto < 10) {
   zero();
  }
  lcd.print(minuto);
  mensagem.concat(minuto);
  lcd.print(":");
  mensagem.concat(':');
  if(segundo < 10) {
   zero(); 
  }
  lcd.print(segundo);
  mensagem.concat(segundo); 
  intervalo();
  mensagem.concat(SF);
  return mensagem;
}
