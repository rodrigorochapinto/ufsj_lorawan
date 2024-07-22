#include "LoRaWan_APP.h"
#include "DHTesp.h"

DHTesp dht;

/*license for Heltec ESP32 LoRaWan, quary your ChipID relevant license: http://resource.heltec.cn/search */
uint32_t  license[4] = {0x513212B6,0x8358ECC0,0x427F1DFD,0x82559887};

/* OTAA para*/
uint8_t devEui[] = { 0x2D, 0x3E, 0x1D, 0x13, 0x0A, 0xEA, 0xCC, 0xFF };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x88, 0x3B, 0x0C, 0xAC, 0x88, 0x88, 0xDD, 0xFF, 0x88, 0x88, 0xDD, 0x88, 0x88, 0x88, 0x66, 0x01 };

/* ABP para*/
uint8_t nwkSKey[] = { 0x15, 0xB1, 0xD0, 0xAE, 0xC4, 0x63, 0xFD, 0xBE, 0x3D, 0x11, 0x18, 0x1A, 0x1E, 0xC7, 0xDA,0x85 };
uint8_t appSKey[] = { 0xD7, 0x2C, 0x78, 0x75, 0x8C, 0xDC, 0xCA, 0xBF, 0x55, 0xEE, 0x4A, 0x77, 0x8D, 0x16, 0xFE,0x67 };
uint32_t devAddr =  ( uint32_t )0xEE7e6ae1;

/*LoraWan channelsmask*/
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
//uint32_t appTxDutyCycle = 15000; /* +/- 15 segundos *** Padrão inicial inicial da biblioteca ***/
//uint32_t appTxDutyCycle = 30000; /* +/- 30 segundos*/
//uint32_t appTxDutyCycle = 60000; /* +/- 1 minutos*/
//uint32_t appTxDutyCycle = 120000; /* +/- 2 minutos*/
//uint32_t appTxDutyCycle = 300000; /* +/- 5 minutos*/
uint32_t appTxDutyCycle = 600000; /* +/- 10 minutos*/

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;


/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = false;

/* Application port */
uint8_t appPort = 25;

/*--------------------------------- INÍCIO DO DOWNLINK -----------------------------------------
 * A função downLinkDataHandle(McpsIndication_t *mcpsIndication) é chamada assim que uma
 * mensagem estiver na fila do agendamento esperando um UPLINK. Após o recebimento a função 
 * decodifica o payload e chama a função app(uint8_t data) passando o payload recebido.
 * Obs: Essas duas funções não devem ser chamadas dentro do código, pois isso é feito via rádio
*/
#define LEDPinVerde    12
#define LEDPinAmarelo  13
#define LEDPinVermelho 23
 
void app(uint8_t data)
 {
   lora_printf("data:%d\r\n",data);
   Serial.print("data: ");
   Serial.println(data);
   switch(data)
     {
    case 0:
    {      
      digitalWrite(LEDPinVerde, HIGH);
      delay(5000);
      break;
    }
    case 1:
    {      
      digitalWrite(LEDPinAmarelo, HIGH);
      delay(5000);
      break;
    }
    case 2:
    {
      digitalWrite(LEDPinVermelho, HIGH);
      delay(5000);
      break;
    }
    case 3:
    {
      digitalWrite(LEDPinVerde, HIGH);
      delay (1000);
      digitalWrite(LEDPinAmarelo, HIGH);
      delay (1000);
      digitalWrite(LEDPinVermelho, HIGH);
      delay(5000);
      break;
    }
    default:
    {
      break;
    }
     }
 }

void  downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
  lora_printf("+REV DATA:%s,RXSIZE %d,PORT %d\r\n",mcpsIndication->RxSlot?"RXWIN2":"RXWIN1",mcpsIndication->BufferSize,mcpsIndication->Port);
  lora_printf("+REV DATA:");
    app(mcpsIndication->Buffer[0]);

  for(uint8_t i=0;i<mcpsIndication->BufferSize;i++)
  {
    lora_printf("%02X",mcpsIndication->Buffer[i]);
  }
  lora_printf("\r\n");
}

/*------------------------------------ FIM DO DOWNLINK -----------------------------------------
/*#define LORA_SPREADING_FACTOR 9
 */
/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
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
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

byte payload[10] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};

RTC_DATA_ATTR bool firstrun = true;

void setup() {
  dht.setup(17, DHTesp::DHT11);                       //inicializa o DHT no pino 17
  pinMode(LEDPinVerde,OUTPUT);
  pinMode(LEDPinAmarelo,OUTPUT);
  pinMode(LEDPinVermelho,OUTPUT);
  Serial.begin(115200);
  Serial.println("Iniciando......");
  Mcu.setlicense(license);                            
  Mcu.begin();
  if(firstrun)
  {
    LoRaWAN.displayMcuInit();
    firstrun = false;
  }
  deviceState = DEVICE_STATE_INIT;
}

/* Prepares the payload of the frame */
static void prepareTxFrame( uint8_t port )
{
  /*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
  *appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
  *if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
  *if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
  *for example, if use REGION_CN470, 
  *the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
  */
    float   temperatura = dht.getTemperature();
    float   umidade     = dht.getHumidity();
    int     alt         = 917; 
    
    unsigned char *puc;
     
    puc = (unsigned char *)(&temperatura);
    appDataSize = 10;
    appData[0] = puc[0];
    appData[1] = puc[1];
    appData[2] = puc[2];
    appData[3] = puc[3];

    puc = (unsigned char *)(&umidade);
    appData[4] = puc[0];
    appData[5] = puc[1];
    appData[6] = puc[2];
    appData[7] = puc[3];

    puc = (unsigned char *)(&alt);
    appData[8] = puc[0];
    appData[9] = puc[1];

    Serial.print("Temperatura: ");
    Serial.print(temperatura);
    Serial.print(" °C | Umidade: ");
    Serial.print(umidade);
    Serial.print(" % | Altitude: ");
    Serial.print(alt);
    Serial.println(" m");
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
      //digitalWrite(ledPin, HIGH);
      LoRaWAN.displaySending();
      prepareTxFrame( appPort );
      LoRaWAN.send();
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }
    case DEVICE_STATE_CYCLE:
    {
      // Schedule next packet transmission
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
