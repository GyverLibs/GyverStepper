// FIFO буфер для планировщика 2

#ifndef _FIFO_h
#define _FIFO_h

template <typename T, uint16_t SIZE >
class FIFO {
public:
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
    void next() {
        buf[tail] = 0;
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