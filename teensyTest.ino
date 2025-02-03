void setup() {
  // configure PORTC, bit 5 to be a digital I/O bit
  PORTC_PCR5 = PORT_PCR_MUX(0x1);
  // configure bit 5 to be an output
  GPIOC_PDDR = 0x20;

}

void loop() {
  // turn on the bit.
  GPIOC_PDOR |= 0x20;

  // wait for 0.1 second
  delay(100);

  // turn off the bit
  GPIOC_PDOR = GPIOC_PDOR & ~0x20;

  // wait for 0.1 second
  delay(100);

}
