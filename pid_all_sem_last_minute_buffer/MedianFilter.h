#ifndef MedianFilter_H
#define MedianFilter_H

#include "Arduino.h"
#include <iostream>
#include <bits/stdc++.h>

class MedianFilter{
  public:
    explicit MedianFilter(int n);
    ~MedianFilter();

    int updateFilter(int num);
    //int* getValues();
    //void setValues(int* vals);
    int* values;
    int N;
  private:
    int* srtdValues;
};

#endif
