
int incomingByte = 0;   // for incoming serial data
 
void setup() {
       pinMode(BOARD_LED_PIN, OUTPUT);
}
 
void loop() {
        if (SerialUSB.available()) {
                incomingByte = SerialUSB.read();
                SerialUSB.println("1");
                if(incomingByte == '1'){
                  digitalWrite(BOARD_LED_PIN,LOW);
                  
                }
                if(incomingByte == '0'){
                  digitalWrite(BOARD_LED_PIN,HIGH);
                }
        
        }
}


