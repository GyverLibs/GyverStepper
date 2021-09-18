// простая библиотек плоттера для Processing
// v1.0 - релиз

class Plotter {
  // x, y, size x, size y, axes, resolution
  Plotter(int nx, int ny, int nxx, int nyy, int naxes, int nres) {
    x = nx;
    xx = nxx;
    y = ny;
    yy = nyy;
    res = nres;
    axes = naxes;
    size = xx / nres;
    buffer = new int[axes][size];
  }

  // автомасштаб
  void autoScale(boolean nscale) {
    scale = nscale;
  }
  
  // минимум (для фикс масштаба)
  void setMin(int nminV) {
    minV = nminV;
  }
  
  // максимум (для фикс масштаба)
  void setMax(int nmaxV) {
    maxV = nmaxV;
  }
  
  // кол-во делений  
  void setLineAmount(int nlnum) {
    lnum = nlnum;
  }
  
  // отображение минимума и максимума
  void showMinMax(boolean nminmax) {
    minmax = nminmax;
  }  
  
  // инициализация
  void init() {    
    plot = createGraphics(xx+frame*2, yy+frame*2);
    plot.beginDraw();
    plot.textSize(11);
    plot.colorMode(HSB, 255, 255, 255);
    plot.background(0, 0);
    plot.endDraw();    
    canvas = createGraphics(xx+frame*2, yy+frame*2);
    canvas.beginDraw();
    canvas.colorMode(HSB, 255, 255, 255);
    canvas.fill(255);
    canvas.stroke(0);
    canvas.strokeWeight(frame);
    canvas.rect(frame, frame, xx, yy);
    canvas.strokeWeight(1);
    for (int i = 0; i < axes; i++) {
      canvas.fill((i * hueStep) % 255, 255, 200);
      canvas.rect(8 + i * 16, 8, 12, 12);
    }
    canvas.endDraw();
    image(canvas, x, y);
  }
  
  // добавить точку (ось, значение)
  void add(int addr, float val) {
    add(addr, int(val));
  }
  
  // добавить точку (ось, значение)
  void add(int addr, int val) {
    plot.beginDraw();
    plot.strokeWeight(1.2);
    plot.stroke((addr * hueStep) % 255, 255, 200);
    buffer[addr][count % size] = val;

    tmaxV = maxV;
    tminV = minV;
    int buf[] = new int [size];

    tmaxV = -0xFFFFFF;
    tminV = 0xFFFFFF;  
    for (int a = 0; a < axes; a++) {
      for (int i = 0; i < size; i++) {
        tmaxV = max(tmaxV, buffer[a][i]);
        tminV = min(tminV, buffer[a][i]);
      }
    }

    if (!scale) {
      for (int i = 0; i < size; i++) {
        buf[i] = constrain(buffer[addr][i], minV, maxV);
      }
    }

    if (scale) delta = abs(tmaxV) + abs(tminV);
    else delta = abs(maxV) + abs(minV);
    for (int i = 0; i < size; i++) {
      if (delta != 0) buf[i] = int((buffer[addr][i] - (scale ? tminV : minV)) * (yy-frame) / delta);
      else buf[i] = 0;
    }

    for (int i = res; i < size+1; i++) {
      if (count > 0) plot.line((i-1) * res, yy - buf[(i + count - 1) % size], i * res, yy - buf[(i + count) % size]);
    }

    plot.endDraw();
  }

  // отобразить
  void update() {    
    image(canvas, x, y);
    image(plot, x, y);
    plot.beginDraw();
    plot.background(0, 0);
    plot.stroke(0, 100);   
    plot.fill(0);
    for (int i = 0; i < lnum; i++) {
      int posY = (i + 1) * yy / (lnum + 1);
      plot.line(frame, posY, xx, posY);      
      if (delta != 0) plot.text((yy-posY) * delta / yy + (scale ? tminV : minV), 10, posY - 2);
    }
    if (minmax) {           
      plot.text("max: " + tmaxV, xx/2 - 15, 14);
      plot.text("min: " + tminV, xx/2 - 15, yy-2);
    }
    plot.endDraw();
    count++;
  }

  // переменные
  int x, xx, y, yy;
  int lnum = 0;
  int count, res, axes, size;
  int buffer[][];
  int minV, maxV;
  int tminV, tmaxV;
  int delta;
  boolean scale = true, minmax = false;
  PGraphics plot, canvas;

  int frame = 2;
  int hueStep = 151;  // шаг цвета
};
