// FIFO буфер для планировщика 2

#ifndef _FIFO_h
#define _FIFO_h
#define FIFO_WIPE 1

template <typename T, uint16_t SIZE >
class FIFO {
public:
    FIFO() {
        for (int i = 0; i < SIZE; i++) buf[i] = 0;
    }
    // очистить
    void clear() {
        head = tail = 0;
    }

    // элементов в буфере
    uint16_t available() {
        return (SIZE + head - tail) % SIZE;
    }

    // прочитать элемент под номером от начала
    T get(int16_t i = 0) {
        return buf[(SIZE + tail + i) % SIZE];
    }
    
    // прочитать крайний элемент буфера
    T getLast() {
        return buf[(SIZE + head - 1) % SIZE];
    }

    // добавить элемент с конца
    void add(T data) {
        uint16_t i = (head + 1) % SIZE;
        if (i != tail) {
            buf[head] = data;
            head = i;
        }
    }

    // установить значение элемента под номером от начала
    void set(int16_t i, T data) {
        buf[(SIZE + tail + i) % SIZE] = data;
    }

    // сдвинуть начало на 1
    void next(bool wipe = 0) {
        if (wipe) buf[tail] = 0;
        tail = (tail + 1) % SIZE;
    }

    // доступность для записи (свободное место)
    bool availableForWrite() {
        return ((head + 1) % SIZE != tail);
    }

private:
    T buf[SIZE];
    uint16_t head = 0, tail = 0;
};

#endif