#include <HardwareSerial.h>
#include <ODriveArduino.h>

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) {
  obj.print(arg);
  return obj;
}
template<>        inline Print& operator <<(Print &obj, float arg) {
  obj.print(arg, 4);
  return obj;
}

HardwareSerial& odrive_serial = Serial1;
ODriveArduino odrive(odrive_serial);

#define FREQUENCY_HZ        100
#define INTERVAL_MS         (1000 / (FREQUENCY_HZ + 1))

static unsigned long last_interval_ms = 0;

void setup() {
    odrive_serial.begin(115200);
    Serial.begin(115200);
    Serial.println("Started");
}

void loop() {
    float power;

    odrive_serial << "r axis0.controller.electrical_power\n";
    power = odrive.readFloat();

    if (millis() > last_interval_ms + INTERVAL_MS) {
        last_interval_ms = millis();

        Serial.print(power);
        Serial.println('\t');
    }
}
