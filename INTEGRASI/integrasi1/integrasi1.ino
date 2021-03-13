/*IMPORT DEPENDENCIES*/
#include <math.h>
#include <stdlib.h>
#include "arduinoFFT.h"
#include "FS.h"
#include <SPI.h>
#include <TFT_eSPI.h> // Hardware-specific library

/*==========INTERFACE==========*/
TFT_eSPI tft = TFT_eSPI(); // Invoke custom library

#define CALIBRATION_FILE "/TouchCalData4"
#define REPEAT_CAL false
boolean stopButton = false;
// Comment out to stop drawing black spots
#define BLACK_SPOT

// pertanyaan menu position and size
#define pertanyaan_X 50
#define pertanyaan_Y 40
#define pertanyaan_W 220
#define pertanyaan_H 40

// Menu deteksi pipa size
#define deteksiPipa_X 30
#define deteksiPipa_Y 110
#define deteksiPipa_W 120
#define deteksiPipa_H 100

// Menu deteksi kebocoran size
#define deteksiKebocoran_X 170
#define deteksiKebocoran_Y 110
#define deteksiKebocoran_W 120
#define deteksiKebocoran_H 100



/*==========PENGOLAHAN DATA==========*/
/*DEFINE PARAMETER PENGOLAHAN SINYAL*/
#define SAMPLING_FREQUENCY 44100
#define SAMPLES 1024
#define BATCH 10 //Jumlah Batch Untuk PERIODOGRAM

/*DEFINE INPUT PIN*/
#define MIC_PIN 36 //Sesuaikan Dengan konfigurasi PIN

/*DEFINE UNTUK FUNGSI OUTPUT DALAM SKALA*/
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

/*======KAMUS VARIABEL======*/
arduinoFFT FFT;

/*=====KODE SETUP=====*/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  tft.init();
  // Set the rotation before we calibrate
  tft.setRotation(1);
  // call screen calibration
  touch_calibrate();
  // clear screen
  tft.fillScreen(TFT_BLUE);
  //LED
  pinMode(27, OUTPUT);
  pinMode(26, OUTPUT);
  
  pinMode(MIC_PIN,INPUT);
//  fftSinyal(specOutReal);
//  PrintVector(specOutReal,SAMPLES/2,SCL_FREQUENCY);
}

/*==========MAIN LOOP==========*/
void loop() {
  // put your main code here, to run repeatedly:
  uint16_t x, y;

  drawMenu();
  
  // See if there's any touch data for us
  if (tft.getTouch(&x, &y))
  {
    // Draw a block spot to show where touch was calculated to be
    #ifdef BLACK_SPOT
      tft.fillCircle(x, y, 2, TFT_BLACK);
    #endif
    

      if ((x > deteksiPipa_X) && (x < (deteksiPipa_X + deteksiPipa_W))) {
        if ((y > deteksiPipa_Y) && (y <= (deteksiPipa_Y + deteksiPipa_H))) {
          Serial.println("Deteksi Pipa btn hit");
          deteksiPipa();
        }
      }
      
      if ((x > deteksiKebocoran_X) && (x < (deteksiKebocoran_X + deteksiKebocoran_W))) {
        if ((y > deteksiKebocoran_Y) && (y <= (deteksiKebocoran_Y + deteksiKebocoran_H))) {
          Serial.println("Deteksi Kebocoran btn hit");
          deteksiKebocoran();
        }
      }

  }
}

/*------------------------------------FUNCTION AND PROCEDURE---------------------------------*/

/*------------------------------------------FFTSINYAL----------------------------------------*/
void fftSinyal(double *specOutReal){
  unsigned int samplingPeriod;
  unsigned long newTime;
  unsigned long startTime,processingTime,currentTime;
  double* vReal;
  double* vImag;

  startTime = micros();
  Serial.print("UKURAN HEAP SEBELUM ALOKASI MEMORY: ");
  Serial.println(ESP.getFreeHeap());
  vReal = (double*)malloc(BATCH*SAMPLES*sizeof(double));
  vImag = (double*)malloc(BATCH*SAMPLES*sizeof(double));
  Serial.print("UKURAN HEAP SETELAH ALOKASI MEMORY: ");
  Serial.println(ESP.getFreeHeap());
  Serial.println();
  
  samplingPeriod = (1000000*(1.0/SAMPLING_FREQUENCY));

  //MEMBACA DATA
  processingTime = micros();
  for(int i=0;i<BATCH*SAMPLES;i++){
    newTime=micros();
    vReal[i]=analogRead(MIC_PIN);
    vImag[i]=0;
    while(micros() < (newTime + samplingPeriod)){
      //Menunggu periode sampling selanjutnya
    }
  }
  currentTime = micros();

  //PROCESSING DATA
  processingTime = micros();
  for(int i=0;i<BATCH;i++){
    FFT = arduinoFFT(vReal+i*SAMPLES,vImag+i*SAMPLES,SAMPLES,SAMPLING_FREQUENCY);
    FFT.DCRemoval();
    FFT.Windowing(vReal+i*SAMPLES,SAMPLES,FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal+i*SAMPLES,vImag+i*SAMPLES,SAMPLES,FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal+i*SAMPLES,vImag+i*SAMPLES,SAMPLES);
    if(i!=0){
      for(int j=0;j<SAMPLES;j++){
        vReal[j]+=vReal[j+i*SAMPLES];
        if(i==BATCH-1 && j<512){
          // OUTPUT DATA
          //Memasukkan hasil spektrum ke output
          specOutReal[j]=vReal[j]/BATCH;
        }
      }
    }
  } 
  
  free(vReal);
  free(vImag);
  Serial.print("UKURAN HEAP SETELAH FREE MEMORY: ");
  Serial.println(ESP.getFreeHeap());
}

/*-----------------------------------------PRINT VECTOR-----------------------------------------*/

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
  break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / SAMPLING_FREQUENCY);
  break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES);
  break;
    }
    Serial.print(abscissa, 6);
    if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}

/*---------------------------------------TOUCH CALIBRATE-------------------------------------*/

void touch_calibrate()
{
  uint16_t calData[5];
  uint8_t calDataOK = 0;

  // check file system exists
  if (!SPIFFS.begin()) {
    Serial.println("Formating file system");
    SPIFFS.format();
    SPIFFS.begin();
  }

  // check if calibration file exists and size is correct
  if (SPIFFS.exists(CALIBRATION_FILE)) {
    if (REPEAT_CAL)
    {
      // Delete if we want to re-calibrate
      SPIFFS.remove(CALIBRATION_FILE);
    }
    else
    {
      File f = SPIFFS.open(CALIBRATION_FILE, "r");
      if (f) {
        if (f.readBytes((char *)calData, 14) == 14)
          calDataOK = 1;
        f.close();
      }
    }
  }

  if (calDataOK && !REPEAT_CAL) {
    // calibration data valid
    tft.setTouch(calData);
  } else {
    // data not valid so recalibrate
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(20, 0);
    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    tft.println("Touch corners as indicated");

    tft.setTextFont(1);
    tft.println();

    if (REPEAT_CAL) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.println("Set REPEAT_CAL to false to stop this running again!");
    }

    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("Calibration complete!");

    // store data
    File f = SPIFFS.open(CALIBRATION_FILE, "w");
    if (f) {
      f.write((const unsigned char *)calData, 14);
      f.close();
    }
  }
}

/*----------------------------------drawMenu----------------------------------------*/
void drawMenu()
{
  
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  
  //Display section pertanyaan mau menu apa
  tft.drawString("Menu :", pertanyaan_X + 100, pertanyaan_Y + (pertanyaan_H / 2));

  //Display menu deteksi pipa
  tft.drawRect(deteksiPipa_X, deteksiPipa_Y, deteksiPipa_W, deteksiPipa_H, TFT_DARKGREY);
  tft.drawString("Deteksi", deteksiPipa_X + 60, deteksiPipa_Y + (deteksiPipa_H / 4));
  tft.drawString("Pipa", deteksiPipa_X + 60, deteksiPipa_Y + (deteksiPipa_H * 0.75));

  //Display menu deteksi kebocoran
  tft.drawRect(deteksiKebocoran_X, deteksiKebocoran_Y, deteksiKebocoran_W, deteksiKebocoran_H, TFT_DARKGREY);
  tft.drawString("Deteksi", deteksiKebocoran_X + 60, deteksiKebocoran_Y + (deteksiKebocoran_H / 4));
  tft.drawString("Kebocoran", deteksiKebocoran_X + 60, deteksiKebocoran_Y + (deteksiKebocoran_H * 0.75));
}

/*----------------------------------------------------deteksiPipa------------------------------------------------------*/
// Menu Deteksi Pipa
void deteksiPipa()
{
  /*variable lokal*/
  double vReal[512];
  uint16_t i;
  
  // Pilihan stop
  #define stopButton_X 250
  #define stopButton_Y 5
  #define stopButton_W 60
  #define stopButton_H 30

  // Hasil
  #define hasil_X 20
  #define hasil_Y 40
  #define hasil_W 290
  #define hasil_H 50

  /*--- Algoritma ---*/
  tft.fillRect(deteksiPipa_X, deteksiPipa_Y, deteksiPipa_W, deteksiPipa_H, TFT_DARKGREY);
  tft.fillRect(deteksiKebocoran_X, deteksiKebocoran_Y, deteksiKebocoran_W, deteksiKebocoran_H, TFT_BLUE);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Deteksi", deteksiPipa_X + 60, deteksiPipa_Y + (deteksiPipa_H / 4));
  tft.drawString("Pipa", deteksiPipa_X + 60, deteksiPipa_Y + (deteksiPipa_H * 0.75));

  Serial.println("Debug 0");
  //bagian deteksi kebocoran
  tft.drawRect(deteksiKebocoran_X, deteksiKebocoran_Y, deteksiKebocoran_W, deteksiKebocoran_H, TFT_DARKGREY);
  tft.drawString("Deteksi", deteksiKebocoran_X + 60, deteksiKebocoran_Y + (deteksiKebocoran_H / 4));
  tft.drawString("Kebocoran", deteksiKebocoran_X + 60, deteksiKebocoran_Y + (deteksiKebocoran_H * 0.75));
  
  delay(500);
  
  tft.fillScreen(TFT_BLUE);
  uint16_t x, y;

  //Display hasil
  Serial.println("Debug 1");
  tft.drawRect(hasil_X, hasil_Y, hasil_W, hasil_H, TFT_DARKGREY);
  tft.drawString("Tampilan hasil deteksi", hasil_X + (hasil_W / 2), hasil_Y + (hasil_H / 2));

  //Display stop button
  tft.drawRect(stopButton_X, stopButton_Y, stopButton_W, stopButton_H, TFT_DARKGREY);
  tft.drawString("Stop", stopButton_X + (stopButton_W / 2), stopButton_Y + (stopButton_H / 2));
  
  //Pengaturan LED, nanti diganti sama kode buat FFT
  Serial.println("Debug 2");
  digitalWrite (27, HIGH);
  
  //dummy vReal, nanti diganti sama FFT seharusnya
  Serial.println("Debug 3");
   
  
  //Nunggu perintah stop
  while (stopButton == false)
  {
    fftSinyal(vReal);
    //Display spektrum frekuensi
    histogram(vReal);
    if (tft.getTouch(&x, &y))
    {
      // Draw a black spot to show where touch was calculated to be
      #ifdef BLACK_SPOT
        tft.fillCircle(x, y, 2, TFT_BLACK);
      #endif
      if ((x > stopButton_X) && (x < (stopButton_X + stopButton_W))) 
      {
          if ((y > stopButton_Y) && (y <= (stopButton_Y + stopButton_H))) 
          {
            stopButton = true;
            digitalWrite (27, LOW);
          }
      }
    }
  }
  stopButton = false;
  tft.fillScreen(TFT_BLUE);
}

/*---------------------------------------------------deteksiKebocoran-------------------------------------------------------*/
// Menu Deteksi Kebocoran
void deteksiKebocoran()
{
  /*variable lokal*/
  double vReal[512];
  uint16_t i;
  
  // Pilihan stop
  #define stopButton_X 250
  #define stopButton_Y 5
  #define stopButton_W 60
  #define stopButton_H 30

  // Hasil
  #define hasil_X 20
  #define hasil_Y 40
  #define hasil_W 290
  #define hasil_H 50

  /*--- Algoritma ---*/
  tft.fillRect(deteksiKebocoran_X, deteksiKebocoran_Y, deteksiKebocoran_W, deteksiKebocoran_H, TFT_DARKGREY);
  tft.fillRect(deteksiPipa_X, deteksiPipa_Y, deteksiPipa_W, deteksiPipa_H, TFT_BLUE);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Deteksi", deteksiKebocoran_X + 60, deteksiKebocoran_Y + (deteksiKebocoran_H / 4));
  tft.drawString("Kebocoran", deteksiKebocoran_X + 60, deteksiKebocoran_Y + (deteksiKebocoran_H * 0.75));

  //bagian deteksi pipa
  tft.drawRect(deteksiPipa_X, deteksiPipa_Y, deteksiPipa_W, deteksiPipa_H, TFT_DARKGREY);
  tft.drawString("Deteksi", deteksiPipa_X + 60, deteksiPipa_Y + (deteksiPipa_H / 4));
  tft.drawString("Pipa", deteksiPipa_X + 60, deteksiPipa_Y + (deteksiPipa_H * 0.75));

  delay(500);
  
  tft.fillScreen(TFT_BLUE);
  uint16_t x, y;

  //Display hasil
  tft.drawRect(hasil_X, hasil_Y, hasil_W, hasil_H, TFT_DARKGREY);
  tft.drawString("Tampilan hasil deteksi", hasil_X + (hasil_W / 2), hasil_Y + (hasil_H / 2));

  //Display stop button
  tft.drawRect(stopButton_X, stopButton_Y, stopButton_W, stopButton_H, TFT_DARKGREY);
  tft.drawString("Stop", stopButton_X + (stopButton_W / 2), stopButton_Y + (stopButton_H / 2));

  //Pengaturan LED, nanti diganti sama kode buat FFT
  digitalWrite (26, HIGH);
   
  while (stopButton == false)
  {
    fftSinyal(vReal);
    //Display spektrum frekuensi
    histogram(vReal);
    if (tft.getTouch(&x, &y))
    {
      // Draw a block spot to show where touch was calculated to be
      #ifdef BLACK_SPOT
        tft.fillCircle(x, y, 2, TFT_BLACK);
      #endif
      if ((x > stopButton_X) && (x < (stopButton_X + stopButton_W))) 
      {
          if ((y > stopButton_Y) && (y <= (stopButton_Y + stopButton_H))) 
          {
            stopButton = true;
            digitalWrite (26, LOW);
          }
      }
    }
  }
  stopButton = false;
  tft.fillScreen(TFT_BLUE);
}

/*---------------------------------------------------HISTOGRAM-------------------------------------------------------*/
void histogram(double vData[])
{
    /*--- variable lokal ---*/
    uint16_t arraySize = 1024;
    uint16_t i = 0;
    uint16_t j = 0;
    uint16_t kelas;
    double averageMagnitude[21];
    uint16_t i_counter[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    double magnitude[]= {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    /*--- Algoritma ---*/
    
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
  
    /*--- gambar kartesian dan keterangan ---*/
    
    //sumbu vertikal
    tft.drawFastVLine(20, 115, 105, TFT_WHITE); 
    tft.drawTriangle(15, 115, 25, 115, 20, 110, TFT_WHITE);
    tft.drawString("dB", 20, 102);

    //keterangan sumbu vertikal
    //Kelas 1 : 0 - 5 dB
    tft.drawFastHLine(18, 210, 5, TFT_WHITE);
    //Kelas 2 : 5 - 10 dB
    tft.drawFastHLine(18, 200, 5, TFT_WHITE);
    tft.drawString("10", 10, 200);
    //Kelas 3 : 10 - 15 dB
    tft.drawFastHLine(18, 190, 5, TFT_WHITE);
    //Kelas 4 : 15 - 20 dB
    tft.drawFastHLine(18, 180, 5, TFT_WHITE);
    tft.drawString("20", 10, 180);
    //Kelas 5 : 20 - 25 dB
    tft.drawFastHLine(18, 170, 5, TFT_WHITE);
    //Kelas 6 : 25 - 30 dB
    tft.drawFastHLine(18, 160, 5, TFT_WHITE);
    tft.drawString("30", 10, 160);
    //Kelas 7 : 30 - 35 dB
    tft.drawFastHLine(18, 150, 5, TFT_WHITE);
    //Kelas 8 : 35 - 40 dB
    tft.drawFastHLine(18, 140, 5, TFT_WHITE);
    tft.drawString("40", 10, 140);
    //Kelas 9 : 40 - 45 dB
    tft.drawFastHLine(18, 130, 5, TFT_WHITE);
    //Kelas 10: 45 - 50 dB
    tft.drawFastHLine(18, 120, 5, TFT_WHITE);
    tft.drawString("50", 10, 120);
    
    //sumbu horizontal
    tft.drawFastHLine(20, 220, 255, TFT_WHITE); 
    tft.drawTriangle(275, 215, 275, 225, 280, 220, TFT_WHITE); 
    tft.drawString("f(kHz)", 300, 220);
    
    //keterangan sumbu horizontal
    //Kelas 1 : 0 - 0.5 kHz
    tft.drawFastVLine(32.5, 218, 5, TFT_WHITE);
    //Kelas 2 : 0.5 - 1 kHz
    tft.drawFastVLine(45, 218, 5, TFT_WHITE);
    tft.drawString("1", 45, 230);
    //Kelas 3 : 1 - 1.5 kHz
    tft.drawFastVLine(57.5, 218, 5, TFT_WHITE);
    //Kelas 4 : 1.5 - 2 kHz
    tft.drawFastVLine(70, 218, 5, TFT_WHITE);
    tft.drawString("2", 70, 230);
    //Kelas 5 : 2 - 2.5 kHz
    tft.drawFastVLine(82.5, 218, 5, TFT_WHITE);
    //Kelas 6 : 2.5 - 3 kHz
    tft.drawFastVLine(95, 218, 5, TFT_WHITE);
    tft.drawString("3", 95, 230);
    //Kelas 7 : 3 - 3.5 kHz
    tft.drawFastVLine(107.5, 218, 5, TFT_WHITE);
    //Kelas 8 : 3.5 - 4 kHz
    tft.drawFastVLine(120, 218, 5, TFT_WHITE);
    tft.drawString("4", 120, 230);
    //Kelas 9 : 4 - 4.5 kHz
    tft.drawFastVLine(132.5, 218, 5, TFT_WHITE);
    //Kelas 10 : 4.5 - 5 kHz
    tft.drawFastVLine(145, 218, 5, TFT_WHITE);
    tft.drawString("5", 145, 230);
    //Kelas 11 : 5 - 5.5 kHz
    tft.drawFastVLine(157.5, 218, 5, TFT_WHITE);
    //Kelas 12 : 5.5 - 6 kHz
    tft.drawFastVLine(170, 218, 5, TFT_WHITE);
    tft.drawString("6", 170, 230);
    //Kelas 13 : 6 - 6.5 kHz
    tft.drawFastVLine(182.5, 218, 5, TFT_WHITE);
    //Kelas 14 : 6.5 - 7 kHz
    tft.drawFastVLine(195, 218, 5, TFT_WHITE);
    tft.drawString("7", 195, 230);
    //Kelas 15 : 7 - 7.5 kHz
    tft.drawFastVLine(207.5, 218, 5, TFT_WHITE);
    //Kelas 16 : 7.5 - 8 kHz
    tft.drawFastVLine(220, 218, 5, TFT_WHITE);
    tft.drawString("8", 220, 230);
    //Kelas 17 : 8 - 8.5 kHz
    tft.drawFastVLine(232.5, 218, 5, TFT_WHITE);
    //Kelas 18 : 8.5 - 9 kHz
    tft.drawFastVLine(245, 218, 5, TFT_WHITE);
    tft.drawString("9", 245, 230);
    //Kelas 19 : 9 - 9.5 kHz
    tft.drawFastVLine(257.5, 218, 5, TFT_WHITE);
    //Kelas 20 : 9.5 - 10 kHz
    tft.drawFastVLine(270, 218, 5, TFT_WHITE);
    tft.drawString("10", 270, 230);
    
    /*--- gambar spektrum ---*/
    //membagi kelas kategori frekuensi dari array
    for (i = 0; i<= 233; i++)
    {
      if(i <= 12)
      {
        kelas = 1;
      }  

      if((i >= 13) && (i <= 24))
      {
        kelas = 2;
      }  

      if((i >= 25) && (i <= 36))
      {
        kelas = 3;
      }  

      if((i >= 37) && (i <= 47))
      {
        kelas = 4;
      }  

      if((i >= 48) && (i <= 59))
      {
        kelas = 5;
      }  

      if((i >= 60) && (i <= 70))
      {
        kelas = 6;
      }  

      if((i >= 71) && (i <= 82))
      {
        kelas = 7;
      }  

      if((i >= 83) && (i <= 94))
      {
        kelas = 8;
      }  

      if((i >= 95) && (i <= 105))
      {
        kelas = 9;
      }  

      if((i >= 106) && (i <= 117))
      {
        kelas = 10;
      }  

      if((i >= 118) && (i <= 128))
      {
        kelas = 11;
      }  

      if((i >= 129) && (i <= 140))
      {
        kelas = 12;
      }  

      if((i >= 141) && (i <= 152))
      {
        kelas = 13;
      }  

      if((i >= 153) && (i <= 163))
      {
        kelas = 14;
      }  

      if((i >= 164) && (i <= 175))
      {
        kelas = 15;
      }  

      if((i >= 176) && (i <= 187))
      {
        kelas = 16;
      }  

      if((i >= 188) && (i <= 198))
      {
        kelas = 17;
      }  

      if((i >= 199) && (i <= 210))
      {
        kelas = 18;
      }  

      if((i >= 211) && (i <= 221))
      {
        kelas = 19;
      }  

      if((i >= 222) && (i <= 233))
      {
        kelas = 20;
      }  

      magnitude[kelas] = magnitude[kelas] + vData[i];
      i_counter[kelas] = i_counter[kelas] + 1; 
    }

    //Rata-rata magnituda dari tiap kelas frekuensi
    for(j = 1; j <= 20; j++)
    {
      averageMagnitude[j] = magnitude[j]/i_counter[j];
    }

    histogramMagnitude(averageMagnitude);
}

void histogramMagnitude(double magnitude[])
{
  /*---Variabel Lokal---*/
  uint16_t i = 0;
  float x = 20;
  int32_t y;
  float w = 13;
  int32_t h;
  
  tft.drawRect(22, 119, 253, 100, TFT_BLUE);
  tft.fillRect(22, 119, 253, 100, TFT_BLUE);
  /*--- Algoritma ---*/
  for(i = 1; i <= 20; i++)
  {
    if(magnitude[i] <= 100)
    {
      y = 210;
      h = 10;
    }else if(magnitude[i] <= 20000)
    {
      y = 200;
      h = 20;
    }else if(magnitude[i] <= 30000)
    {
      y = 190;
      h = 30;
    }else if(magnitude[i] <= 40000)
    {
      y = 180;
      h = 40;
    }else if(magnitude[i] <= 50000)
    {
      y = 170;
      h = 50;
    }else if(magnitude[i] <= 60000)
    {
      y = 160;
      h = 60;
    }else if(magnitude[i] <= 70000)
    {
      y = 150;
      h = 70;
    }else if(magnitude[i] <= 80000)
    {
      y = 140;
      h = 80;
    }else if(magnitude[i] <= 90000)
    {
      y = 130;
      h = 90;
    }else
    {
      y = 120;
      h = 100;
    }
    
    tft.drawRect(x, y, w, h, TFT_WHITE);
    //tft.fillRect(x, y, w, h, TFT_WHITE);
    x = x + 12.5;
//    Serial.println(w);
//    Serial.println(x);
    Serial.print(magnitude[2]); Serial.println("Ini Magnitude ke 3");
  }
  
}
