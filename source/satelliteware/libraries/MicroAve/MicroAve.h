
#ifndef MicroAve_h
#define MicroAve_h

#define HISTOGRAM_MAX 32

class MicroAve {
  public:
    // constructors:
    MicroAve();

    void setStart();
    void setStop();
    void addValue(unsigned long value);
    void displayResults(const char *msg, int showHist);
    void reset();

  private:
    int isFirstTime;
    unsigned long lastTime;

    unsigned long hist[HISTOGRAM_MAX];

    unsigned long sum;
    unsigned long count;

    unsigned long min0;
    unsigned long min1;
    unsigned long min2;

    unsigned long max0;
    unsigned long max1;
    unsigned long max2;

};

#endif
