#include "LoRaWan_APP.h"
#include <TinyGPSPlus.h>

TinyGPSPlus gps;
float latitude_x  = 0.;
float longitude_x = 0.;

/* Para OTAA */
uint8_t devEui[] = { 0x14, 0xF7, 0xC6, 0x30, 0x00, 0x00, 0x3C, 0xBB };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0xEE };

/* Para ABP */
uint8_t nwkSKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appSKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint32_t devAddr =  ( uint32_t )0x00000000;

/*license for Heltec ESP32 LoRaWan, quary your ChipID relevant license: http://resource.heltec.cn/search */
uint32_t  license[4] = {0x9418D193,0x225EAC66,0x98ACB6E8,0xE0580DD5};

/* LoraWan channelsmask */
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/* LoraWan region, select in arduino IDE tools */
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/* LoraWan Class, Class A and Class C are supported */
DeviceClass_t  loraWanClass = CLASS_C;

/* Ciclo de trabalho de transmissão de dados do aplicativo.  valor em [ms]. */
uint32_t appTxDutyCycle = 15000;

/* OTAA ou ABP */
bool overTheAirActivation = true;

/* ADR enable */
bool loraWanAdr = true;

/* Variáveis e constantes para o controle da transmissão */
#define buttonPin 0
#define ledPin    25
uint8_t buttonState = 1;

/* Indica se o nó está enviando mensagens confirmadas ou não confirmadas */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 10;
/*!
* Número de tentativas para transmitir o quadro, se a camada LoRaMAC não
* receber um reconhecimento. O MAC realiza uma adaptação da taxa de dados,
* de acordo com a Especificação LoRaWAN V1.0.2, capítulo 18.4, de acordo
* para a seguinte tabela:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Observe que se NbTrials estiver definido como 1 ou 2, o MAC não diminuirá
* a taxa de dados, caso a camada LoRaMAC não receba uma confirmação
*/
uint8_t confirmedNbTrials = 4;

int disparo = 1;
int pacote = 0;

/* Prepara a carga útil do quadro */
static void prepareTxFrame( uint8_t port )
{
  /* appData o tamanho é LORAWAN_APP_DATA_MAX_SIZE que é definido em "commissioning.h".
   * appDataSize o valor máximo é LORAWAN_APP_DATA_MAX_SIZE.
   * se habilitado AT, não modifique LORAWAN_APP_DATA_MAX_SIZE, isso pode causar suspensão ou falha do sistema.
   * se desativado AT, LORAWAN_APP_DATA_MAX_SIZE pode ser modificado, o valor máximo é referência a região de Lorawan e SF.
   * por exemplo, se usar REGION_CN470, 
   * o valor máximo para diferentes DR pode ser encontrado em MaxPayloadOfDatarateCN470 referente a Taxa de DadosCN470 e 
   * Largura de bandas CN470 iem "RegionCN470.h".
  */
    //Serial.print("Disparo:.......: ");Serial.println(disparo);
    //Serial.print("Pacote.........: ");Serial.println(pacote);  

    gps_dados();
    
    unsigned char *puc;
    appDataSize = 12;

    puc = (unsigned char *)(&latitude_x);
    appData[0]  = puc[0];
    appData[1]  = puc[1];
    appData[2]  = puc[2];
    appData[3]  = puc[3];

    puc = (unsigned char *)(&longitude_x);
    appData[4]  = puc[0];
    appData[5]  = puc[1];
    appData[6]  = puc[2];
    appData[7]  = puc[3];
     
    puc = (unsigned char *)(&disparo);
    appData[8]  = puc[0];
    appData[9]  = puc[1];

    puc = (unsigned char *)(&pacote);
    appData[10] = puc[0];
    appData[11] = puc[1];
}

RTC_DATA_ATTR bool firstrun = true;

void setup() {
  Serial2.begin(9600, SERIAL_8N1,2,17);
  Serial.begin(115200);
  //delay(1000);
 
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  
  Mcu.begin();
  if(firstrun)
  {
    LoRaWAN.displayMcuInit();
    firstrun = false;
  }
  deviceState = DEVICE_STATE_INIT;
}

void loop()
{
  switch( deviceState )
  {
    case DEVICE_STATE_INIT:
    {
#if(LORAWAN_DEVEUI_AUTO)
      LoRaWAN.generateDeveuiByChipID();
#endif
      LoRaWAN.init(loraWanClass,loraWanRegion);
      break;
    }
    case DEVICE_STATE_JOIN:
    { 
      LoRaWAN.displayJoining();
      LoRaWAN.join();
      break;
    }
    case DEVICE_STATE_SEND:
    {
      /* Aguarda o precionamento do botão PGR para enviar a mensagem.
       * Neste momento o LED Built-in se acende aguardando o botão ser precionado.
      */
      pacote ++;
      if (pacote == 21){
        pacote = 1;
        disparo ++;
      }
      if (disparo == 6){
        pacote = 0;
        disparo = 1;
        buttonState = 1;
      }
      Serial.println("Aguardando envio da mensagem.....");
      digitalWrite(ledPin, HIGH); 
      while(buttonState == HIGH){
        buttonState = digitalRead(buttonPin);  
      }
      digitalWrite(ledPin, LOW);
      Serial.println("Mensagem enviada!"); 
      //buttonState = 1;
      /* Fim do loop de espera */     
      LoRaWAN.displaySending();
      prepareTxFrame( appPort );
      LoRaWAN.send();
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }
    case DEVICE_STATE_CYCLE:
    {
      /* Agendar a próxima transmissão de pacote */
      txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }
    case DEVICE_STATE_SLEEP:
    {
      LoRaWAN.displayAck();
      LoRaWAN.sleep(loraWanClass);
      break;
    }
    default:
    {
      deviceState = DEVICE_STATE_INIT;     
      break;
    }
  }
}

void gps_dados()
{
  //******************** INICIO GPS ********************
  while (Serial2.available() > 0)
    if (gps.encode(Serial2.read()))
      gps_captura();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
  //********************** FIM GPS **********************
}

void gps_captura()
{
  if (gps.location.isValid())
  {    
    latitude_x  = gps.location.lat() * 10000;
    longitude_x = gps.location.lng() * 10000;
  }
  else
  {
    latitude_x  = 0.;
    longitude_x = 0.;
  }
}
