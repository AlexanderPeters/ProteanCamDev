hw_timer_t * timer = NULL;
volatile uint8_t ledState = 0;

void IRAM_ATTR onTimer() {
  ledState = 1 - ledState; // Toggle led state
  digitalWrite(2, ledState);   // Turn the LED on or off
}

void setup() {
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);    // Turn the LED off by making the voltage LOW

  timer = timerBegin(0, 80, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer, &onTimer, true); // edge (not level) triggered
  timerAlarmWrite(timer, 1000000, true); // 1000000 * 1 us = 1 s, autoreload true
  timerAlarmEnable(timer); // enable
}

void loop() {
  // nope nothing here
  vTaskDelay(portMAX_DELAY); // wait as much as posible ...
}
