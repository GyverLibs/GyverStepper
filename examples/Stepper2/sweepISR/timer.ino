// настроить таймер
void initTimer() {
  TCCR1A = 0;
  // CTC по OCR1A, делитель /64
  TCCR1B = bit(WGM12) | 0b011;
}

// установить период
void setPeriod(uint32_t prd) {
  // один тик таймера - 4 мкс (при 16 МГц клоке)
  OCR1A = (uint32_t)prd >> 2;
}

// запустить и сбросить таймер
void startTimer() {
  TIMSK1 = bit(OCIE1A);
  TCNT1 = 0;
}

// остановить таймер
void stopTimer() {
  TIMSK1 = 0;
  TCNT1 = 0;
}
