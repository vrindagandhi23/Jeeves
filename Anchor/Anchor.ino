// Include Libraries
#include <SoftwareSerial.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#define RX 10
#define TX 11

LiquidCrystal_I2C lcd(0x27,16,2);
SoftwareSerial mySerial(RX, TX);

// ANCHOR send command
// AT+ANCHOR_SEND=<Address of TAG>,<Payload/data length>,<data in ASCII format>
String ANCHOR_SendMSG_cmd = "AT+ANCHOR_SEND=TAG1,2,HI";

const unsigned long sendInterval = 700;
unsigned long previousTime = 0;


void setup()
{
  Serial.begin(115200);          
  mySerial.begin(115200); 
  lcd.init();                      // initialize the lcd 
  lcd.init();
  lcd.backlight();  
  Serial.println("Starting....");
  Serial.println("");
  delay(2000);
}

void loop() 
{
  // put your main code here, to run repeatedly:
  unsigned long currentTime = millis();
  if (currentTime - previousTime >= sendInterval) {
    sendMsg();
    previousTime = currentTime;
  }
  checkSerial();
}

// Function for sending Anchor Send command
void sendMsg()
{
  // Send AT+ANCHOR_SEND=TAG1,2,HI
  Serial.println(ANCHOR_SendMSG_cmd);
  mySerial.println(ANCHOR_SendMSG_cmd); 
}

String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;
 
    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void checkSerial()
{
  while (mySerial.available() > 0) {
    // get the new byte:
    String inString = mySerial.readString();
    inString.remove(0,3);     //Remove +OK\n
    Serial.print("Received Data : ");
    Serial.println(inString); 
    // Data receiving format : +ANCHOR_RCV=<TAG Address>,< PAYLOAD LENGTH>,<TAG DATA>,<DISTANCE>
    // Example:   +OK
    //            +ANCHOR_RCV= TAG1,Length,Data,40 cm
    //                |          |    |      |      |
    //                0          1    2      3      4
    // So to get Distance : String DataIn = getValue(inString, ',', index number);
    // Example:    So to get Distance which is at position 4 in above response : 
    //             String DataIn = getValue(inString, ',', 4);

    String Distance = getValue(inString, ',', 3);     //--> data
    Serial.print("Distance = ");
    Serial.println(Distance);
    lcd.setCursor(0,0);
    lcd.print("Distance");
    lcd.setCursor(0,1);
    lcd.print(Distance);
  }  
}
