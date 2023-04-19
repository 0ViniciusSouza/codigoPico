#ifndef MedianFilter_H
#define MedianFilter_H


class MedianFilter{
  public:
    explicit MedianFilter(int n);
    ~MedianFilter();

    int updateFilter(int num);
    int* values;
    int N;
  private:
    int* srtdValues;
};

#endif
