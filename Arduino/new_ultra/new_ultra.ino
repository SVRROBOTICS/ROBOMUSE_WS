#define RX1 16  // GPIO16 for SENSOR1 RX
#define TX1 17  // GPIO17 for SENSOR1 TX (not used)

#define RX2 4   // GPIO4 for SENSOR2 RX
#define TX2 5   // GPIO5 for SENSOR2 TX (not used)

HardwareSerial SENSOR1(2);  // Use UART2 for SENSOR1
HardwareSerial SENSOR2(1);  // Use UART1 for SENSOR2

unsigned char data[4] = {};
float distance1 = -1, distance2 = -1;
unsigned long lastReadTime = 0;

void setup() {
  Serial.begin(115200);
  SENSOR1.begin(9600, SERIAL_8N1, RX1, TX1);
  SENSOR2.begin(9600, SERIAL_8N1, RX2, TX2);
}

float readSensor(HardwareSerial &sensor) {
  while (sensor.available() >= 4) {
    data[0] = sensor.read();
    if (data[0] == 0xFF) {
      data[1] = sensor.read();
      data[2] = sensor.read();
      data[3] = sensor.read();
      
      int checksum = (data[0] + data[1] + data[2]) & 0x00FF;
      if (checksum == data[3]) {
        return ((data[1] << 8) + data[2]) / 10.0;
      }
    }
  }
  return -1; // Return -1 if no valid data
}

void loop() {
  if (millis() - lastReadTime >= 50) {  // Read every 50ms (faster than 100ms sensor update)
    float newDist1 = readSensor(SENSOR1);
    float newDist2 = readSensor(SENSOR2);

    if (newDist1 != -1) distance1 = newDist1;
    if (newDist2 != -1) distance2 = newDist2;

//    Serial.print("SENSOR1: ");
    Serial.print(distance1);
    Serial.print(",");
    Serial.println(distance2);

    lastReadTime = millis();
  }
}
