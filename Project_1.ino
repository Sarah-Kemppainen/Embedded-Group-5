// Define LED pin (output)
#define SWITCH_PIN 25

void setup() {
    PORTC_PCR5 = PORT_PCR_MUX(0x1);
    PORTD_PCR5 = PORT_PCR_MUX(0x1);
    PORTA_PCR5 = PORT_PCR_MUX(0x1);
    PORTB_PCR11 = PORT_PCR_MUX(0x1);
    PORTE_PCR25 = PORT_PCR_MUX(0x0);

    // Configure the LED port as output
    GPIOC_PDDR |= 0b100000;
    GPIOD_PDDR |= 0b100000;
    GPIOA_PDDR |= 0b100000;
    GPIOB_PDDR |= 0b100000000000;
   

   GPIOE_PDDR &= ~(1 << SWITCH_PIN); // Set PTE25 as input

}

void loop() {
    // The loop can be empty; if you want the LED to stay on, nothing else is needed.
    
    // Turn on LED.
    GPIOC_PDOR |= 0b100000;
    delay(500);
    GPIOC_PDOR ^= 0b100000;
    delay(500);
     GPIOB_PDOR |= 0b100000000000;
    delay(500);
    GPIOB_PDOR ^= 0b100000000000;
    delay(500);
     GPIOA_PDOR |= 0b100000;
    delay(300);
    GPIOA_PDOR ^= 0b100000;
    delay(300);
    GPIOD_PDOR |= 0b100000;
    delay(300);
    GPIOD_PDOR ^= 0b100000;
    delay(300);

    if (GPIOE_PDIR & (1 << SWITCH_PIN)) {
        // Switch open
        GPIOC_PDOR = 0x0;// Turn off LED PTC5
        GPIOB_PDOR = 0x0;
        GPIOA_PDOR = 0x0;
        GPIOD_PDOR = 0x0;
    } else {
        // Switch closed
        GPIOC_PDOR |= 0b100000;// Turn on LED PTC5
        GPIOA_PDOR |= 0b100000;
        GPIOB_PDOR |= 0b100000;
        GPIOD_PDOR |= 0b100000000000;
    }
   
}



