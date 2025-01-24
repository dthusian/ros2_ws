#define AVG_SAMPLES 8

class Avg {
  float samples[AVG_SAMPLES];
  int wptr;

public:
  Avg() : samples{}, wptr{} { }

  void push(float x) {
    samples[wptr] = x;
    wptr++;
    wptr %= AVG_SAMPLES;
  }

  float sum() {
    float sum = 0;
    for(int i = 0; i < AVG_SAMPLES; i++) sum += samples[i];
    return sum;
  }

  float avg() {
    return sum() / AVG_SAMPLES;
  }

  void reset() {
    for(int i = 0; i < AVG_SAMPLES; i++) samples[i] = 0;
  }
};