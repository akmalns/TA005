#include <math.h>
#include "arduinoFFT.h"
#define MIC_PIN 36
#define SAMPLING_FREQUENCY 44100
#define SAMPLES 1024
#define BATCH 6

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

double vReal[BATCH*SAMPLES];
double vImag[BATCH*SAMPLES];
unsigned long newTime=0;
unsigned long oldTime=0;
unsigned long firstTime=0;
unsigned long deltaTime=0;
unsigned int samplingPeriod;

void fftSinyal(double *vReal, double *vImag){
  for(int i=0;i<BATCH;i++){
    arduinoFFT FFT = arduinoFFT(vReal+i*SAMPLES,vImag+i*SAMPLES,SAMPLES,SAMPLING_FREQUENCY);
    FFT.DCRemoval();
    FFT.Windowing(vReal+i*SAMPLES,SAMPLES,FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal+i*SAMPLES,vImag+i*SAMPLES,SAMPLES,FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal+i*SAMPLES,vImag+i*SAMPLES,SAMPLES);
    if(i!=0){
      for(int j=0;j<SAMPLES;j++){
        vReal[j]+=vReal[j+i*SAMPLES];
        if(i==BATCH-1){
          vReal[j]/=BATCH;
        }
      }
    }
  }
}

int isPipa(double *mag){
  double threshold = 0; //ini akan diganti sesuai dengan hasil pengukuran/percobaan
  double detectionFrequency = 300; //ini akan diganti sesuai dengan hasil pengukuran/percobaan
  double freqRes = SAMPLING_FREQUENCY/SAMPLES;
  if(
  
}

int isBocor(double *mag){
  double threshold = 0; //ini akan diganti sesuai dengan hasil percobaan

  
}

void setup() {
  Serial.begin(115200);
  samplingPeriod = (1000000 * (1.0/SAMPLING_FREQUENCY));
  pinMode(MIC_PIN,INPUT);
}

void loop() {

  for(int i=0;i<BATCH*SAMPLES;i++){
    newTime=micros();
    vReal[i]=analogRead(MIC_PIN);
    vImag[i]=0;
    while(micros() < (newTime + samplingPeriod)){
      //Chill
    }
  }

  
  int detectionFrequency = 2300;
  int y = round(detectionFrequency/(44100/1024));
  double threshold = 10000;
  if(vReal[y]>threshold){
    Serial.println("Frequency Detected");
//    Serial.println(vReal[y]); 
  }
//  Serial.println(vReal[y]);
//  Serial.println("::::::");
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
