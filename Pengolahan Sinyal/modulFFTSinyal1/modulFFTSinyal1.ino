/*IMPORT DEPENDENCIES*/
#include <math.h>
#include <stdlib.h>
#include "arduinoFFT.h"

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

/*======KAMUS VARIABEL GLOBAL======*/
double specOutReal[SAMPLES];
arduinoFFT FFT;

/*======DEKLARASI FUNGSI DAN PROSEDUR======*/
void fftSinyal(double *specOutReal){
  unsigned int samplingPeriod;
  unsigned long newTime;
  double* vReal;
  double* vImag;
  
  vReal = (double*)malloc(BATCH*SAMPLES*sizeof(double));
  vImag = (double*)malloc(BATCH*SAMPLES*sizeof(double));
  samplingPeriod = (1000000*(1.0/SAMPLING_FREQUENCY));

  //MEMBACA DATA
  for(int i=0;i<BATCH*SAMPLES;i++){
    newTime=micros();
    vReal[i]=analogRead(MIC_PIN);
    vImag[i]=0;
    while(micros() < (newTime + samplingPeriod)){
      //Menunggu periode sampling selanjutnya
    }
  }
  
  //PROCESSING DATA
  for(int i=0;i<BATCH;i++){
    FFT = arduinoFFT(vReal+i*SAMPLES,vImag+i*SAMPLES,SAMPLES,SAMPLING_FREQUENCY);
    FFT.DCRemoval();
    FFT.Windowing(vReal+i*SAMPLES,SAMPLES,FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal+i*SAMPLES,vImag+i*SAMPLES,SAMPLES,FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal+i*SAMPLES,vImag+i*SAMPLES,SAMPLES);
    if(i!=0){
      for(int j=0;j<SAMPLES;j++){
        vReal[j]+=vReal[j+i*SAMPLES];
        if(i==BATCH-1){
          // OUTPUT DATA
          //Memasukkan hasil spektrum ke output
          specOutReal[j]=vReal[j]/BATCH;
        }
      }
    }
  }
  free(vReal);
  free(vImag);
}


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


/*=====KODE SETUP=====*/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(MIC_PIN,INPUT);
  fftSinyal(specOutReal);
  PrintVector(specOutReal,512,SCL_FREQUENCY);
}

/*==========MAIN LOOP==========*/
void loop() {
  // put your main code here, to run repeatedly:
  fftSinyal(specOutReal);
}
