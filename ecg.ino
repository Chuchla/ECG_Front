#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <User_Setup.h>

#define WREG 0x7f
#define RREG 0x80
#define CONFIG 0x00
#define FLEX_CH1_CN 0x01
#define FLEX_CH2_CN 0x02
#define FLEX_CH3_CN 0x03
#define FLEX_PACE_CN 0x04
#define FLEX_VBAT_CN 0x05
#define LOD_CN 0x06
#define LOD_EN 0x07
#define LOD_CURRENT 0x08
#define LOD_AC_CN 0x09
#define CMDET_EN 0x0a
#define CMDET_CN 0x0b
#define RLD_CN 0x0c
#define REF_CN 0x11
#define OSC_CN 0x12
#define AFE_RES 0x13
#define AFE_SHDN_CN 0x14
#define AFE_FAULT_CN 0x15
#define AFE_PACE_CN 0x17
#define ERR_STATUS 0x19
#define ERR_SYNC 0x1D
#define MASK_ERR 0x2a
#define R2_RATE 0x21
#define R3_RATE_CH1 0x22
#define R3_RATE_CH2 0x23
#define R3_RATE_CH3 0x24
#define R1_RATE 0x25
#define DIS_EFILTER 0x26
#define DRDYB_SRC 0x27
#define SYNCB_CN 0x28
#define CH_CNFG 0x2f
#define DATA_STATUS 0x30
#define REVID 0x40
#define DATA_LOOP 0x50
#define POSITIVE_TST_SIG 0x01
#define NEGATIVE_TST_SIG 0x02
#define ZERO_TST_SIG 0x03
#define ERROR -1

#define DRDY_PIN 33
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_TX "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_RX "faaa1ce4-eca1-4a11-b9e2-715d451b5d14"

#define DIVIDE_FSAMPLE_BY 3
#define IN_MIN -3000
#define IN_MAX 6000
#define OUT_MIN 0
#define OUT_MAX 319


BLECharacteristic *pCharacteristicTX;  // TX: dane EKG
BLECharacteristic *pCharacteristicRX;  // RX: komendy suwaka
SPIClass *vspi = NULL;
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
String sliderValue = "1";  // Wartosc odbierana przez BLE

int sampleCounter{};
const int csPin = 5;
const int WIDTH = 320;
const int HEIGHT = 480;
int x = 0;
int z = 0;
long sample = 0;
long oldSample = 0;
int samples[3] = { 0, 0, 0 };
int oldSamples[3] = { 0, 0, 0 };
volatile bool dataReady = false;  // Flaga przerwania
TFT_eSPI tft = TFT_eSPI(WIDTH, HEIGHT);
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool filterSamples = false;
double coeffsHPF[5] = { 0.9944617362863962, -1.9889234725727925, 0.9944617362863962, -1.9888928005576803, 0.9889541445879048 };
double coeffsNF[5] = { 0.9826294021585108, -1.3896478273191322, 0.9826294021585108, -1.3896478273191322, 0.9652588043170216 };
double coeffsLPF[5] = { 0.5690337746913825, 1.138067549382765, 0.5690337746913825, 0.9428060277021068, 0.3333290710634233 };

int ch1Buffor[5000];
int ch2Buffor[5000];
int ch3Buffor[5000];
int buforIndex{};

//Channel 1
double hpf1[4] = { 0, 0, 0, 0 };
double nf1[4] = { 0, 0, 0, 0 };
double lpf1[4] = { 0, 0, 0, 0 };
//Channel 2
double hpf2[4] = { 0, 0, 0, 0 };
double nf2[4] = { 0, 0, 0, 0 };
double lpf2[4] = { 0, 0, 0, 0 };
//Channel 3
double hpf3[4] = { 0, 0, 0, 0 };
double nf3[4] = { 0, 0, 0, 0 };
double lpf3[4] = { 0, 0, 0, 0 };

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};

class MyRXCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    //std::string rxValue = pCharacteristic->getValue();
    String rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
      sliderValue = rxValue.c_str();
      Serial.println(sliderValue);
    }
  }
};

int32_t getECGdata(uint8_t channel, SPIClass *bus) {
  uint8_t rawData[3];
  int32_t ecgData;
  channel = channel < 1 || channel > 3 ? -1 : channel - 1;
  if (channel == -1) return -1;
  rawData[0] = ads1293ReadRegister(0x37 + (channel * 3), bus);
  rawData[1] = ads1293ReadRegister(0x38 + (channel * 3), bus);
  rawData[2] = ads1293ReadRegister(0x39 + (channel * 3), bus);
  uint32_t tempData = (uint32_t)rawData[0] << 16;
  tempData |= (uint32_t)rawData[1] << 8;
  tempData |= rawData[2];
  tempData = tempData << 8;
  ecgData = (int32_t)(tempData);
  ecgData = ecgData >> 8;
  return ecgData;
}
std::vector<int32_t> readBackLoop(SPIClass *bus) {
  uint8_t rawData[9];
  uint8_t dataToSend = (DATA_LOOP | RREG);
  digitalWrite(csPin, LOW);
  bus->transfer(dataToSend);
  for (int i = 0; i < 9; i++) {
    rawData[i] = bus->transfer(0);
  }
  digitalWrite(csPin, HIGH);
  std::vector<int32_t> ecgData;
  for (int i = 0; i < 3; i++)
    ecgData.push_back(rawDataToEcgData(rawData[0 + i * 3], rawData[1 + i * 3], rawData[2 + i * 3]));
  return ecgData;
}
int32_t rawDataToEcgData(uint8_t msb, uint8_t b, uint8_t lsb) {
  int32_t ecgData;
  uint32_t tempData = (uint32_t)msb << 16;
  tempData |= (uint32_t)b << 8;
  tempData |= lsb;
  tempData = tempData << 8;
  ecgData = (int32_t)(tempData);
  ecgData = ecgData >> 8;
  return ecgData;
}
void ads1293WriteRegister(uint8_t wrAddress, uint8_t data, SPIClass *bus) {
  uint8_t dataToSend = (wrAddress & WREG);
  digitalWrite(csPin, LOW);
  bus->transfer(dataToSend);
  bus->transfer(data);
  digitalWrite(csPin, HIGH);
}

uint8_t ads1293ReadRegister(uint8_t rdAddress, SPIClass *bus) {

  uint8_t rdData;
  uint8_t dataToSend = (rdAddress | RREG);
  digitalWrite(csPin, LOW);
  bus->transfer(dataToSend);
  rdData = bus->transfer(0);
  digitalWrite(csPin, HIGH);

  return (rdData);
}

void ICACHE_RAM_ATTR drdyInterruptHndlr() {
  dataReady = true;
}

void enableInterruptPin() {
  pinMode(DRDY_PIN, INPUT_PULLUP);  // Konfiguracja pinu DRDY jako wejścia
  attachInterrupt(digitalPinToInterrupt(DRDY_PIN), drdyInterruptHndlr, FALLING);
}

void ads1293InitSingleChannel(SPIClass *bus) {
  //Ustawienie elektrody pozytywnej kanłu 1 na IN2 i negatywnej na IN1
  ads1293WriteRegister(FLEX_CH1_CN, 0b00010001, bus);
  //Wyłączenie kanału 2 i 3
  ads1293WriteRegister(FLEX_CH2_CN, 0x00, bus);
  ads1293WriteRegister(FLEX_CH3_CN, 0x00, bus);
  //Wykrywanie Common-Mode na IN1 IN2
  ads1293WriteRegister(CMDET_EN, 0b0000011, bus);
  //Ustawienie RLD na IN3
  ads1293WriteRegister(RLD_CN, 0x03, bus);
  //Ustawienie generatora sygnału zegarowego jako moduł ekg
  ads1293WriteRegister(OSC_CN, 0x04, bus);
  //Wł. wzm.instrumentalnych i modulatorow dla kanłu 1
  ads1293WriteRegister(AFE_SHDN_CN, 0b00110110, bus);
  //Ustawienie DC_rate R2 na wartość 8 w celu zmniejszenia BW i ODR
  ads1293WriteRegister(R2_RATE, 0b00001000, bus);
  //Ustawienie DC_rate R3_CH1 na 8 w celu zmniejszenia BW i ODR i fs=400hz
  ads1293WriteRegister(R3_RATE_CH1, 0b00000100, bus);
  //Pin DRDY daje znac ze dane sa gotowe do odczytu dla kanalu 1
  ads1293WriteRegister(DRDYB_SRC, 0b00001000, bus);
  //Uruchomienie procesu konwersji danych
  ads1293WriteRegister(CONFIG, 0x01, bus);
}

void ads1293InitThreeChannels(SPIClass *bus) {
  // Lead 1 = IN2 - IN 1 (LA - RA)
  ads1293WriteRegister(FLEX_CH1_CN, 0b00010001, bus);
  // Lead 2 = IN3 - IN 1 (LL - RA)
  ads1293WriteRegister(FLEX_CH2_CN, 0b00011001, bus);
  // Lead 3 = IN3 - IN 2 (LL - LA)
  ads1293WriteRegister(FLEX_CH3_CN, 0b00011010, bus);
  //Wykrywanie Common-Mode na IN1 IN2 IN3
  ads1293WriteRegister(CMDET_EN, 0b0000111, bus);
  //Ustawienie RLD na IN 4
  ads1293WriteRegister(RLD_CN, 0b00000100, bus);
  //Ustawienie generatora sygnału zegarowego jako moduł ekg
  ads1293WriteRegister(OSC_CN, 0x04, bus);
  //Włączenie wszystkich wzmacniaczy instrumentalnych i przetworników sigma delta
  ads1293WriteRegister(AFE_SHDN_CN, 0b00000000, bus);
  //Ustawienie DC_rate R2 na wartość 8 w celu zmniejszenia BW i ODR
  ads1293WriteRegister(R2_RATE, 0b00001000, bus);
  //Ustawienie DC_rate R3_CH1 na 8 w celu zmniejszenia BW i ODR i fs=400hz
  ads1293WriteRegister(R3_RATE_CH1, 0b00000100, bus);
  //Ustawienie DC_rate R3_CH2 na 8 w celu zmniejszenia BW i ODR i fs=400hz
  ads1293WriteRegister(R3_RATE_CH2, 0b00000100, bus);
  //Ustawienie DC_rate R3_CH2 na 8 w celu zmniejszenia BW i ODR i fs=400hz
  ads1293WriteRegister(R3_RATE_CH3, 0b00000100, bus);
  //Pin DRDY daje znac ze dane sa gotowe do odczytu dla kanalu 1
  ads1293WriteRegister(DRDYB_SRC, 0b00001000, bus);
  //Ustawienie możliwości trybu Streaming'u dla EKG CH1 CH2 CH3
  ads1293WriteRegister(CH_CNFG, 0b01110000, bus);
  //Uruchomienie procesu konwersji danych
  ads1293WriteRegister(CONFIG, 0x01, bus);
}
void drawGrid() {
  tft.fillScreen(TFT_BLACK);
  for (int i = 0; i <= WIDTH; i += 20) {
    if (i % 8 == 0) {
      tft.drawFastHLine(0, i, HEIGHT, TFT_BLUE);
    }
  }
  for (int i = 0; i <= HEIGHT; i += 20) {
    if (i % 5 == 0) {
      tft.drawFastVLine(i, 0, WIDTH, TFT_BLUE);
    }
  }
  char sliderCommand = sliderValue.charAt(0);
  switch (sliderCommand) {
    case '1':
      {
        drawLeadInfo("Lead I");
        break;
      }
    case '2':
      {
        drawLeadInfo("Lead II");
        break;
      }
    case '3':
      {
        drawLeadInfo("Lead III");
        break;
      }
    case '4':
      {
        drawLeadInfo("All Leads");
        break;
      }
    default:
      {
        drawLeadInfo("Lead I");
        break;
      }
  }
}

void drawLeadInfo(String leadName) {
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(TR_DATUM);
  tft.setTextPadding(9);
  tft.drawString(leadName, HEIGHT - 5, 5);  // Odejmuje 5 pikseli dla marginesu
}

void drawTrace(long value) {
  if (z != 0) {
    z = 0;
    drawGrid();
  }
  x = x + 1;
  if (x >= HEIGHT) {
    x = 0;
    drawGrid();
  }
  oldSample = sample;
  sample = value;

  if (x > 0) {
    tft.drawLine(x - 1, WIDTH - oldSample, x, WIDTH - sample, TFT_WHITE);
  }
}

void drawTraces(int value1, int value2, int value3) {
  if (x != 0) {
    x = 0;
    drawGrid();
  }
  z = z + 1;
  if (z >= HEIGHT) {
    z = 0;
    drawGrid();
  }
  oldSamples[0] = samples[0];
  oldSamples[1] = samples[1];
  oldSamples[2] = samples[2];
  samples[0] = value1;
  samples[1] = value2;
  samples[2] = value3;
  if (z > 0) {
    tft.drawLine(z - 1, WIDTH - oldSamples[0], z, WIDTH - samples[0], TFT_WHITE);
    tft.drawLine(z - 1, WIDTH - oldSamples[1], z, WIDTH - samples[1], TFT_WHITE);
    tft.drawLine(z - 1, WIDTH - oldSamples[2], z, WIDTH - samples[2], TFT_WHITE);
  }
}

void drawCenteredString(String text) {
  int16_t x = tft.width() / 2;
  int16_t y = tft.height() / 2;
  tft.drawString(text, x, y);
}

void tftInitialization() {
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  drawCenteredString("ECG Portable Monitor");
  tft.setTextSize(1);
}

void ecgModuleInitializaton() {
  vspi = new SPIClass(VSPI);
  vspi->begin(18, 19, 23, 5);
  pinMode(csPin, OUTPUT);
  pinMode(DRDY_PIN, INPUT);
  enableInterruptPin();
  ads1293InitThreeChannels(vspi);
}

void BLEInit() {
  BLEDevice::init("ESP32");             // Wł. funkcji BLE i nadanie nazwy
  pServer = BLEDevice::createServer();  // Stworzenie serwera
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  // Tworzenie charakterystyki TX
  pCharacteristicTX = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  pCharacteristicTX->addDescriptor(new BLE2902());
  // Tworzenie charkterystyki RX
  pCharacteristicRX = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pCharacteristicRX->addDescriptor(new BLE2902());
  pCharacteristicRX->setCallbacks(new MyRXCallbacks());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
}

double biquadFilter(double sample, double *coeffs, double *arr) {
  // coeffs[5]  = {a0, a1, a2, b0, b1}
  // arr[4]     = {x1, x2, y1, y2}
  // y = a0 * x + a1 * x1 + a2 * x2 - b1 * y1 - b2 * y2
  double y = coeffs[0] * sample + coeffs[1] * arr[0] + coeffs[2] * arr[1] - coeffs[3] * arr[2] - coeffs[4] * arr[3];

  //Przesunięcie próbek
  arr[1] = arr[0];  //x2 = x1;
  arr[0] = sample;  //x1 = x;
  arr[3] = arr[2];  //y2 = y1;
  arr[2] = y;       //y1 = y;


  return y;
}

void fillingBuffers(int val1, int val2, int val3){
  if (buforIndex == 5000) {
      buforIndex = 0;
    }
    ch1Buffor[buforIndex] = val1;
    ch2Buffor[buforIndex] = val2;
    ch3Buffor[buforIndex] = val3;
    buforIndex++;
}

void setup() {
  Serial.begin(115200);
  tftInitialization();
  ecgModuleInitializaton();
  BLEInit();

  delay(3000);
  drawGrid();
}

void loop() {
  uint8_t dataStatus = ads1293ReadRegister(DATA_STATUS, vspi);
  uint8_t errStatus = ads1293ReadRegister(ERR_STATUS, vspi);
  char sliderCommand = sliderValue.charAt(0);
  if(sliderCommand == '6') filterSamples = true;
  else if(sliderCommand == '7') filterSamples = false;
  
  if (dataReady && errStatus == 0) {
    std::vector<int32_t> ecgValues = readBackLoop(vspi);
    int32_t ecgValue1 = ecgValues.at(0);
    int32_t ecgValue2 = ecgValues.at(1);
    int32_t ecgValue3 = ecgValues.at(2);
    dataReady = false;
    
    double hpfValue1 = biquadFilter(double(ecgValue1), coeffsHPF, hpf1);
    double nfValue1 = biquadFilter(hpfValue1, coeffsNF, nf1);
    double lpfValue1 = biquadFilter(nfValue1, coeffsLPF, lpf1);

    double hpfValue2 = biquadFilter(double(ecgValue2), coeffsHPF, hpf2);
    double nfValue2 = biquadFilter(hpfValue2, coeffsNF, nf2);
    double lpfValue2 = biquadFilter(nfValue2, coeffsLPF, lpf2);

    double hpfValue3 = biquadFilter(double(ecgValue3), coeffsHPF, hpf3);
    double nfValue3 = biquadFilter(hpfValue3, coeffsNF, nf3);
    double lpfValue3 = biquadFilter(nfValue3, coeffsLPF, lpf3);
    if(filterSamples == true) Serial.printf("Lead1: %d, Lead2: %d, Lead3: %d\n", lpfValue1, lpfValue2, lpfValue3);
    else if(filterSamples == false) Serial.printf("Lead1: %d, Lead2: %d, Lead3: %d\n", ecgValue1, ecgValue2, ecgValue3);
    

    fillingBuffers(int(lpfValue1), int(lpfValue2), int(lpfValue3));
    if (sliderCommand == '5') {
      if (deviceConnected && pCharacteristicTX) {
        for (int i = 0; i < 5000; i++) {
          //std::string toBeSent = std::to_string(ch1Buffor[i]) + "," + std::to_string(ch2Buffor[i]) + "," + std::to_string(ch3Buffor[i]);
          String toBeSent = String(ch1Buffor[i]) + "," + String(ch2Buffor[i]) + "," + String(ch3Buffor[i]);
          pCharacteristicTX->setValue(toBeSent);
          pCharacteristicTX->notify();
          delay(5);
        }
        sliderValue = "1";
      }
    }

  sampleCounter++;
  if (sampleCounter % DIVIDE_FSAMPLE_BY == 0) {
    switch (sliderCommand) {
      case '1':
        {
          int result1 = customMapToRange(int(lpfValue1), OUT_MIN, OUT_MAX);
          drawTrace(result1);
          break;
        }
      case '2':
        {
          int result2 = customMapToRange(int(lpfValue2), OUT_MIN, OUT_MAX);
          drawTrace(result2);
          break;
        }
      case '3':
        {
          int result3 = customMapToRange(int(lpfValue3), OUT_MIN, OUT_MAX);
          drawTrace(result3);
          break;
        }
      case '4':
        {
          int result1 = customMapToRange(int(lpfValue1), 0, 105);
          int result2 = customMapToRange(int(lpfValue2), 106, 212);
          int result3 = customMapToRange(int(lpfValue3), 213, 319);
          drawTraces(result1, result2, result3);
          break;
        }
      default:
        {
          int result1 = customMapToRange(int(lpfValue1), OUT_MIN, OUT_MAX);
          drawTrace(result1);
          break;
        }
    }
    sampleCounter = 0;
  }
}


if (!deviceConnected && oldDeviceConnected) {
  delay(500);  // Czas na reset stosu BLE
  pServer->startAdvertising();
  Serial.println("Start advertising");
  oldDeviceConnected = deviceConnected;
}
if (deviceConnected && !oldDeviceConnected) {
  oldDeviceConnected = deviceConnected;
}
}

int customMapToRange(int value, int outMin, int outMax) {
  int rangeIn = IN_MAX - IN_MIN;   // Zakres wejściowy
  int rangeOut = outMax - outMin;  // Zakres wyjściowy
  int result = 0;
  value = constrain(value, IN_MIN, IN_MAX);
  result = ((value - IN_MIN) * rangeOut) / rangeIn;
  result = result + outMin;
  return result;
}