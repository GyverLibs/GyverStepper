class FIFO {
  FIFO(int nsize) {
    size = nsize;
    clear();
    buf = new int[size];
    for (int i = 0; i < size; i++) buf[i] = 0;
  }

  void clear() {
    head = tail = 0;
  }

  int available() {
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

  void set(int i, int p) {
    buf[(tail + i) % size] = p;
  }

  void next() {
    buf[tail] = 0;
    tail = (tail + 1) % size;
  }

  boolean availableForWrite() {
    return ((head + 1) % size != tail);
  }

  int buf[];
  int size, head, tail;
};
