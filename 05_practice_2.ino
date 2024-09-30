#define PIN_LED 7
unsigned int toggle;
void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  while(!Serial) {
    ;
  }
  toggle = 1;
  digitalWrite(PIN_LED, toggle);
}

void loop() {
  for( int i = 0; i< 5; i++) {
    digitalWrite(PIN_LED, 1);
    delay(100);
    
    
    digitalWrite(PIN_LED, 0);
    delay(100);
  }
  digitalWrite(PIN_LED,0);

  while(1) {
    ;
  }
 
}
