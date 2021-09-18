class Timer {
  Timer(int newDt) {
    dt = newDt;
  }
  
  void start() {
    startF = true;
  }
  
  void stop() {
    startF = false;
  }
  
  boolean tick() {
    if (startF && millis() - tmr >= dt) {
      tmr = millis();
      return true;
    }
    return false;
  }

  int dt = 0, tmr = 0;
  boolean startF = false;
};
