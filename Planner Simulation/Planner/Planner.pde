// СИМУЛЯЦИЯ ПЛАНИРОВЩИКА GPLANNER2

int type = 2;  // тип маршрута: 0 - ручной массив, 1 - случайный, 2 - окружность 
int dots = 20; // количество точек в маршруте

int mul = 1;    // уменьшить разрешение по шагам (например 6)
boolean show = false;  // показать сетку текущего разрешения

// массив маршрута тип 0
int points[][] = {
  {100, 0}, 
  {120, 20}, 
  {150, 50}, 
  {220, 120}, 
};

//
int WX = 800;
int HX = 800;

Planner_v3 pln = new Planner_v3(2);

Plotter plotterX = new Plotter(0, HX/2, WX/2-10, HX/2-10, 2, 2);
Plotter plotterY = new Plotter(WX/2, HX/2, WX/2-10, HX/2-10, 2, 2);

Timer timer = new Timer(10);

MyBtn pause = new MyBtn(false, 10, 10, 80, 20, "pause");
MyBtn resume = new MyBtn(false, 10, 30, 80, 20, "resume");
MyBtn stop = new MyBtn(false, 10, 50, 80, 20, "stop");
MyBtn brake = new MyBtn(false, 10, 70, 80, 20, "brake");
MyBtn next = new MyBtn(false, 10, 90, 80, 20, "next");

void settings() {
  size(WX, HX);
  smooth(8);
}

void setup() {
  frameRate(1000);
  colorMode(HSB);
  background(210);
  randomSeed(6);
  
  // забиваем маршрут
  if (type == 1) {
    points = new int[dots][4];
    for (int i = 0; i < dots; i++) {
      points[i][0] = int(random(0, width));
      points[i][1] = int(random(0, height / 2));
    }
    points[dots-1][2] = 1;
  } else if (type == 2) {
    points = new int[dots][4];
    for (int i = 0; i < dots; i++) {
      float radius = 180;
      points[i][0] = width/2+int(radius*cos((float)i/dots*2*PI));
      points[i][1] = height/4+int(radius*sin((float)i/dots*2*PI));
    }
    points[dots-1][2] = 1;
  }
  
  // даунскейлинг если надо
  for (int i = 0; i < points.length; i++) {
    points[i][0] /= mul;
    points[i][1] /= mul;
  }
  
  // начальные условия
  pln.setCurrent(points[0]);
  pln.setMaxSpeed(200);
  pln.setAcceleration(300);
  
  // запускаем плотер
  plotterX.init();
  plotterY.init();
  plotterX.showMinMax(true);
  plotterY.showMinMax(true);
  plotterY.setLineAmount(10);
  plotterX.setLineAmount(10);
  plotterX.autoScale(false);  // выключим авто масштаб (по умолч. включен)
  plotterX.setMin(0);         // минимум для фикс масштаба
  plotterX.setMax(300);       // максимум для фикс масштаба
  plotterY.autoScale(false);  // выключим авто масштаб (по умолч. включен)
  plotterY.setMin(0);         // минимум для фикс масштаба
  plotterY.setMax(300);       // максимум для фикс масштаба
  timer.start();
  
  // рисуем путь
  drawPath(show);
}

int count = 0;
void draw() {
  if (pause.pool()) pln.pause();
  if (stop.pool()) pln.stop();
  if (resume.pool()) pln.resume();
  if (brake.pool()) pln.brake();
  if (next.pool()) {
    int[] tar = {0,0};
    if (count < points.length) pln.setTarget(tar);
  }

  if (timer.tick()) {
    plotterX.add(0, pln.stepper[0].pos / 5);
    plotterX.add(1, pln.us != 0 ? int(500000/pln.us) : 0);
    plotterX.update();

    plotterY.add(0, pln.stepper[1].pos / 5);
    plotterY.add(1, pln.us != 0 ? int(500000/pln.us) : 0);
    plotterY.update();
  }
  
  pln.tick();
  if (pln.ready()) {  // приехали
    if (count < points.length) pln.setTarget(points[count]);
    count++;
  }
}

void drawPath(boolean grid) {
  stroke(150);
  if (grid) {
    for (int i = 0; i < 200; i++) {
      line(i*mul, 0, i*mul, height);
      line(0, i*mul, width, i*mul);
    }
  }
  for (int i = 0; i < points.length; i++) {
    fill(0);
    noStroke();
    circle(points[i][0]*mul, points[i][1]*mul, 8);
    stroke(0);
    strokeWeight(0.5);
    if (i > 0) line(points[i-1][0]*mul, points[i-1][1]*mul, points[i][0]*mul, points[i][1]*mul);
    text(i, points[i][0]*mul+10, points[i][1]*mul);
    //text(points[i][3], points[i][0]*mul+10, points[i][1]*mul+12);
  }
  noStroke();
}
