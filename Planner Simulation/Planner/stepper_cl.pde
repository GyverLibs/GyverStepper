class Stepper {
  Stepper() {
  }

  void set(int newPos) {
    pos = newPos;
  }  

  void setTarget(int ntar, int t) {
    tar = ntar;
    dir = (pos < tar) ? 1 : -1;
    dt = (pos == tar) ? 0 : (t / abs(pos - tar));
    tmr = millis();
    startF = true;
  }

  void stepTo(int tar) {
    if (tar > pos) pos++;
    else if (tar < pos) pos--;
  }

  void step(int st) {
    pos += st;
  }  

  void setPos(int npos) {
    pos = npos;
  }

  void start() {
    startF = true;
  }

  void stop() {
    startF = false;
  }

  boolean tick() {
    if (!startF) return false;
    if (pos == tar || dt == 0) return true;
    if (millis() - tmr >= dt) {
      tmr += dt;
      pos += dir;
    }
    return false;
  }

  int tmr, dt;
  int pos = 0, tar = 0, dir = 1;
  boolean startF = false;
};
