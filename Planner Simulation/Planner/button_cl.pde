// type 0 - momentary, type 1 - fixed
class MyBtn {
  MyBtn(boolean type, int x, int y, int a, int b, String text) {
    _type = type;
    _x = x;
    _y = y;
    _a = a;
    _b = b;
    _c1 = 180;
    _c2 = 150;
    _c3 = 120;
    _c4 = 0;
    _text = text;
  }
  MyBtn(boolean type, int x, int y, int a, int b, color c1, color c2, color c3, String text, color c4) {
    _type = type;
    _x = x;
    _y = y;
    _a = a;
    _b = b;
    _c1 = c1;
    _c2 = c2;
    _c3 = c3;
    _c4 = c4;
    _text = text;
  }
  boolean pool() {
    boolean flag = false;
    if ((mouseX > _x) && (mouseX < (_x+_a)) && (mouseY > _y) && (mouseY < (_y+_b))) {
      if (mousePressed) {
        if (!_flag) {
          if (_type) _state = !_state;
          else {
            flag = _state = true;
          }
          _flag = true;
        }
        fill(_c3);
      } else {
        if (_flag) _flag = false;
        fill(_c2);
      }
    } else {
      fill(_c1);
      if (_flag) _flag = false;
    }
    if (_state && _type) fill(_c3);
    stroke(0);
    strokeWeight(1);
    rect(_x, _y, _a, _b);
    noStroke();
    fill(_c4);
    textSize(15);
    text(_text, _x+5, _y+15);
    return flag;
  }
  boolean check() {
    if (_type) return _state;
    else {
      if (_state) {
        _state = false;
        return true;
      }
    }
    return false;
  }
  int _x, _y, _a, _b;
  color _c1, _c2, _c3, _c4;
  String _text;
  boolean _state = false, _flag = false, _type = false;
};
