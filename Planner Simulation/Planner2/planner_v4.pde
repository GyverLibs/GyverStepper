class Planner_v4 {
  Planner_v4(int n_AXLES) {
    _AXLES = n_AXLES;
    steppers = new Stepper[_AXLES];
    for (int i = 0; i < _AXLES; i++) steppers[i] = new Stepper();
    bufP = new FIFO[_AXLES];
    for (int i = 0; i < _AXLES; i++) bufP[i] = new FIFO(_BUF);
    dS = new int[2];    
    nd = new int[2];
  }

  void setDtA(float newDta) {
    dtA = newDta;
  }

  void setMaxSpeed(float newV) {
    V = newV;
    usMin = int(1000000.0 / V);
  }

  void setAcceleration(int newA) {
    a = newA;
    if (a != 0) us0 = int(0.676 * 1000000 * sqrt(2.0 / a));
    else us0 = 0;
  }

  void stop() {
    if (us == 0 || status == 0 || status == 4) return;   // мы и так уже остановились, успокойся
    if (a == 0) {    // нет ускорения - дёргай ручник
      brake();
      return;
    }
    stopStep = 1000000 / us;                      // наша скорость
    stopStep = stopStep * stopStep / (2 * a);     // дистанция остановки
    us8 = us << 8;                                // период
    status = 4;                                   // статус на стоп
  }

  void start() {    
    if (status == 0) status = 1;  // если остановлен - проверяем буфер
  }

  void brake() {    
    status = 0;
    us = 0;
    // пишем в буфер что мы остановились
    for (int i = 0; i < _AXLES; i++) bufP[i].set(0, steppers[i].pos);
    bufV.set(0, 0);
  }

  void resume() {
    if (status == 0) status = 1;  // если стоим ждём - проверить буфер
  }

  void setCurrent(int cur[]) {
    for (int i = 0; i < _AXLES; i++) steppers[i].pos = cur[i];
  }

  void setTarget() {
    tmr = millis();
    for (int i = 0; i < _AXLES; i++) {               // для всех осей
      dS[i] = abs(bufP[i].get(1) - steppers[i].pos);      // модуль ошибки по оси      
      steppers[i].dir = steppers[i].pos < bufP[i].get(1) ? 1 : -1;  // направление движения по оси
    }
    S = bufS.get(0);
    for (int i = 0; i < _AXLES; i++) nd[i] = S / 2;  // записываем половину

    if (S == 0) {        // путь == 0, мы никуда не едем
      status = 1;        // на буфер
      next();
      return;
    }

    if (a > 0) {
      int v1 = bufV.get(0);      // скорость начала отрезка
      int v2 = bufV.get(1);    

      if (2.0*V*V-v1*v1-v2*v2 > 2.0*a*S) {  // треугольник
        s1 = int((2.0*a*S + v2*v2 - v1*v1) / (4.0*a));
        s2 = 0;
      } else {          // трапеция
        s1 = int((V*V - v1*v1) / (2 * a));
        s2 = int(S - (V*V - v2*v2) / (2 * a));
      }
      so1 = v1 * v1 / (2 * a);
      so2 = v2 * v2 / (2 * a);
      if (status != 4) {
        if (v1 == 0) us = us0;
        else us = 1000000 / v1;
      }
    } else {
      so1 = so2 = 0;
      s1 = 0;
      s2 = S;
      us = usMin;
    }

    step = 0;
    readyF = false;
    if (status != 4) {    // если это не стоп
      if (bufL.get(1) == 1) status = 3;
      else status = 2;
      us8 = us << 8;
    }
  }

  // вернёт false если мотор приехал
  boolean tick() {
    checkBuffer();
    if (status > 1 && millis() - tmr >= us/1000) {
      tmr = millis(); 
      tickManual();

      // ВЫВОД МАРШРУТА
      int vel = us==0 ? 0 : 1000000/us;
      int hue = int(map(vel, 0, V, 0, 70));
      fill(hue, 255, 255);
      circle(steppers[0].pos*mul, steppers[1].pos*mul, 4);
      // ВЫВОД МАРШРУТА
    }
    return status > 1;
  }

  boolean tickManual() {
    if (status == 5) {
      return true;
    }

    step++;
    for (int i = 0; i < _AXLES; i++) {
      // http://members.chello.at/easyfilter/bresenham.html
      nd[i] -= dS[i];
      if (nd[i] < 0) {
        nd[i] += S;
        steppers[i].step();    // двигаем мотор в направлении sn
      }
    }

    if (status == 4) {
      stopStep--;
      us8 += 2 * us8 / (4 * stopStep + 1);    // торможение
      us = us8 >> 8;
      us = constrain(us, usMin, us0);
      if (step >= S) {
        next();
        setTarget();
        us = us8 >> 8;
      }
      if (stopStep <= 0 || us >= us0) {
        brake();
      }
      return true;
    }

    if (step < s1) {                          // разгон
      us8 -= 2 * us8 / (4 * (step + so1) + 1);
      us = us8 >> 8;
      us = constrain(us, usMin, us0);
    } else if (step < s2) us = usMin;         // постоянная
    else if (step < S) {                      // торможение
      us8 += 2 * us8 / (4 * (S + so2 - step) + 1);
      us = us8 >> 8;
      us = constrain(us, usMin, us0);
    } else {                                  // приехали      
      if (status == 3) {              // достигли конечной точки
        readyF = true;
        status = 0;
      } else status = 1;              // иначе проверяем буфер
      next();
    }
    return status > 1;
  }

  void checkBuffer() {
    if (status == 1) {
      if (bufV.available() > 1) {
        if (bufV.get(0) == 0) calculateBlock();
        setTarget();
      }
    }
  }

  void next() {
    for (int i = 0; i < _AXLES; i++) bufP[i].next();
    bufL.next();
    bufV.next();
    bufS.next();
  }

  boolean ready() {
    if (readyF) {
      readyF = false;
      return true;
    } 
    return false;
  }

  boolean available() {
    return bufV.availableForWrite();
  }

  void addPoint(int[] tar, int l) {
    for (int i = 0; i < _AXLES; i++) bufP[i].add(tar[i]);
    bufL.add(l);
    bufV.add(0);
    bufS.add(0);
  }

  void calculateBlock() {
    // поиск максимальной конечной скорости
    for (int i = 0; i < bufV.available() - 1; i++) {
      int sqSum = 0;
      int[] dn = new int[_AXLES];
      int[] dn1 = new int[_AXLES];
      for (int j = 0; j < _AXLES; j++) {
        dn[j] = bufP[j].get(i+1) - bufP[j].get(i);
        sqSum += dn[j] * dn[j];
      }
      int s1 = int(sqrt(sqSum));
      bufS.set(i, s1);

      if (bufL.get(i + 1) == 1) break;
      if (a == 0) {
        bufV.set(i + 1, int(V));
        continue;
      }

      if (i < bufV.available() - 2) {        
        int multSum = 0;
        sqSum = 0;
        for (int j = 0; j < _AXLES; j++) {
          dn1[j] = bufP[j].get(i + 2) - bufP[j].get(i + 1);
          sqSum += dn1[j] * dn1[j];
          multSum += dn[j] * dn1[j];
        }
        int s2 = int(sqrt(sqSum));
        if (s1 == 0 || s2 == 0) continue;
        float cosa = -(float)multSum / (s1 * s2);
        float v2;
        if (cosa < -0.9) v2 = V;
        else v2 = a * dtA / sqrt(2.0 * (1 + cosa));
        v2 = min(v2, V);
        bufV.set(i + 1, int(v2));
      }
    }

    // рекурсивно уменьшаем конечные скорости на участке
    for (int i = 0; i < bufV.available() - 1; i++) {
      int v0 = bufV.get(i);
      int v1 = bufV.get(i + 1);
      int maxV = int(sqrt(2.0 * a * bufS.get(i) + v0 * v0));
      if (v1 > v0 && maxV < v1) {
        bufV.set(i + 1, maxV);
      } else if (v1 < v0 && maxV > v1) {
        int count = 0;
        while (true) {
          int minV = int(sqrt(2.0 * a * bufS.get(i + count) + bufV.get(i + count + 1) * bufV.get(i + count + 1)));        
          if (minV >= bufV.get(i + count)) break;
          else bufV.set(i + count, minV);          
          count--;
        }
      }
    }
  }

  // status
  // 0 ожидание команды (остановлен)
  // 1 ожидание буфера
  // 2 в пути
  // 3 на паузу
  // 4 на стоп
  // 5 крутится

  int[] nd;
  int S, a;
  float V;
  int s1, s2, so1, so2;
  int[] dS;  
  int us0, us = 0, us8, tmr, usMin, step;
  int stopStep;
  byte status = 0;
  boolean readyF = false;  
  Stepper[] steppers;

  float dtA = 0.3;

  int _BUF = 10;
  int _AXLES = 0;
  FIFO[] bufP;
  FIFO bufL = new FIFO(_BUF);
  FIFO bufV = new FIFO(_BUF);
  FIFO bufS = new FIFO(_BUF);
};
