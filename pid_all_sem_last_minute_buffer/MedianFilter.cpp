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
}

int MedianFilter::updateFilter(int num){
  std::memmove(values,values+1,sizeof(int)*(N-1));
  values[N - 1] = num;

  for(int i = 0; i < N; i++){
    srtdValues[i] = values[i];
  }

  std::sort(srtdValues, srtdValues + N);

  if(N % 2 == 0){
    return (int) ((srtdValues[N/2 - 1] + srtdValues[N/2])/2);
  } else {
    return srtdValues[(N-1)/2];
  }
}
