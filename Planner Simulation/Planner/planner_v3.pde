class Planner_v3 {
  Planner_v3(int numAxles) {
    axles = numAxles;
    stepper = new Stepper[numAxles];
    for (int i = 0; i < axles; i++) stepper[i] = new Stepper();
    dS = new int[numAxles];    
    tar = new int[numAxles];
    nd = new int[numAxles];
    sn = new int[numAxles];
  }

  void setMaxSpeed(int newV) {
    V = newV;
    minUs = 1000000 / V;
  }

  void setAcceleration(int newA) {
    a = newA;
    if (a != 0) us0 = int(0.676 * 1000000 * sqrt(2.0 / a));
    else us0 = 0;
  }

  void pause() {
    status = 2;
  }

  void stop() {
    if (us == 0 || status != 1) return;   // мы и так уже остановились, успокойся
    if (a == 0) {    // нет ускорения - дёргай ручник
      brake();
      return;
    }
    if (step > s2) {  // а мы уже тормозим!
      pause();        // значит флаг на паузу
      return;
    }
    status = 4;
    step = 0;
  }  

  void brake() {
    status = 0;
    us = 0;
  }

  void resume() {
    setTarget(tar);
  }

  void setCurrent(int cur[]) {
    for (int i = 0; i < axles; i++) stepper[i].pos = cur[i];
  }

  int getCurrent(int axis) {
    return stepper[axis].pos;
  }

  int getTarget(int axis) {
    return tar[axis];
  }

  void setTarget(int target[]) {
    S = 0;
    for (int i = 0; i < axles; i++) {               // для всех осей
      if (status != 2) tar[i] = target[i];          // запоминаем цель, если она не остановка      
      dS[i] = abs(target[i] - stepper[i].pos);      // модуль ошибки по оси
      sn[i] = stepper[i].pos < target[i] ? 1 : -1;  // направление движения по оси
      if (dS[i] > S) {
        S = dS[i];
        maxAx = i;
      }
    }
    for (int i = 0; i < axles; i++) nd[i] = S / 2;  // записываем половину диагонали

    if (S == 0) {        // путь == 0, мы никуда не едем
      readyF = true;     // готовы к следующей точке
      status = 0;        // стоп машина
      return;
    }

    if (a > 0) {                        // ускорение задано
      if (us > 0) {                     // мы движемся! ААА!
        int v1 = 1000000 / us;
        if (2.0*V*V-v1*v1 > 2.0*a*S) {  // треугольник
          s1 = int((2.0*a*S - v1*v1) / (4.0*a));
          s2 = s1;
        } else {                        // трапеция
          s1 = (V*V - v1*v1) / (2 * a);
          s2 = S - V*V / (2 * a);
        }
        so1 = v1 * v1 / (2 * a);
      } else {                          // не движемся
        if (V*V > a*S) {                // треугольник
          s1 = S / 2;
          s2 = s1;
        } else {                        // трапеция
          s1 = V*V / (2 * a);
          s2 = S - s1;
        }
        so1 = 0;
        us = us0;
      }
    } else {                            // ускорение отключено
      s1 = 0;
      s2 = S;
      us = minUs;
    }

    tmr = millis();
    step = 0;
    readyF = false;
    status = 1;
  }

  // вернёт false если мотор приехал
  boolean tick() {
    if (status > 0 && millis() - tmr >= us/1000) {

      // ВЫВОД МАРШРУТА
      int vel = us==0 ? 0 : 1000000/us;
      int hue = int(map(vel, 0, V, 0, 70));
      fill(hue, 255, 255);
      circle(stepper[0].pos*mul, stepper[1].pos*mul, 4);
      // ВЫВОД МАРШРУТА

      tmr = millis();      
      step++;
      for (int i = 0; i < axles; i++) {
        // http://members.chello.at/easyfilter/bresenham.html
        if (i == maxAx) stepper[i].step(sn[i]);
        else {
          nd[i] -= dS[i];
          if (nd[i] < 0) {
            nd[i] += S;
            stepper[i].step(sn[i]);    // двигаем мотор в направлении sn
          }
        }
      }

      if (status == 4) {
        us += 2 * us / (4 * (s1-step) + 1);    // торможение
        if (step >= S || us >= us0) {
          us = 0;
          status = 0;
          return false;
        }
        return false;
      }

      // https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/
      if (step < s1) us -= 2 * us / (4 * (step + so1) + 1);      // разгон
      else if (step < s2) us = minUs;                            // постоянная
      else if (step < S) us += 2 * us / (4 * (S - step) + 1);    // торможение
      else {                                                     // мы приехали
        us = 0;
        if (status == 1) readyF = true;
        status = 0;
        return false;
      }
      us = max(us, minUs);
    }
    return status > 0;
  }

  boolean ready() {
    return (readyF && status == 0);  // если мы приехали и остановились
  }

  int[] nd, sn;
  int axles;
  int S, V, a;
  int s1, s2, so1;
  int[] dS, tar;  
  int us0, us = 0, tmr, minUs, step;
  byte status = 0;
  int maxAx;
  boolean readyF = true;

  Stepper[] stepper;
};
