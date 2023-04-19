#include "MedianFilter.h"

MedianFilter::MedianFilter(int n): N{n} {
  values = new int[N];
  srtdValues = new int[N];

  for(int i = 0; i < N; i++){
    values[i] = 0;
    srtdValues[i] = 0;
  }
}

MedianFilter::~MedianFilter(){
  delete values;
  delete srtdValues;
}

int MedianFilter::updateFilter(int num){
  for(int i = 0; i < N - 1; i++){
    values[i] = values[i+1];
  }
  values[N - 1] = num;

  for(int i = 0; i < N; i++){
    srtdValues[i] = values[i];
  }

  for(int i=0; i< N-1; i++){
    for(int j=i+1; j < N; j++){
      if(srtdValues[j] < srtdValues[i]){
        int temp = srtdValues[i];
        srtdValues[i] = srtdValues[j];
        srtdValues[j] =  temp;
      }
    }
  }

  if(N % 2 == 0){
    return (int) ((srtdValues[N/2 - 1] + srtdValues[N/2])/2);
  } else {
    return srtdValues[(N-1)/2];
  }
}
