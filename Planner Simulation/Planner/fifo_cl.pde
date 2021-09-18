class FIFO {
  FIFO(int nsize) {
    size = nsize;
    clear();
    buf = new int[size];
  }

  void clear() {
    head = tail = 0;
  }

  int amount() {
    return (size + head - tail) % size;
  }

  int get(int i) {
    return buf[(tail + i) % size];
  }

  void add(int p) {
    int i = (head + 1) % size;
    if (i != tail) {
      buf[head] = p;
      head = i;
    }
  }

  void set(int p) {
    buf[tail] = p;
  }

  void next() {
    tail = (tail + 1) % size;
  }

  boolean availableForWrite() {
    return ((head + 1) % size != tail);
  }

  int buf[];
  int size, head, tail;
}
