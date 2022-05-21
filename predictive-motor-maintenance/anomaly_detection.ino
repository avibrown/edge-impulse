/* Includes ---------------------------------------------------------------- */
#include <odrive_06_inferencing.h>
#include <HardwareSerial.h>
#include <ODriveArduino.h>

// Edge Impulse parameters
#define FREQUENCY_HZ        EI_CLASSIFIER_FREQUENCY
#define INTERVAL_MS         (1000 / (FREQUENCY_HZ + 1))
static float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {};
static unsigned long last_interval_ms = 0;
size_t feature_ix = 0;

HardwareSerial& odrive_serial = Serial1;
ODriveArduino odrive(odrive_serial);

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) {
  obj.print(arg);
  return obj;
}
template<>        inline Print& operator <<(Print &obj, float arg) {
  obj.print(arg, 4);
  return obj;
}

float power;
float anomaly_threshold = 1.2;

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
}

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    odrive_serial.begin(115200);
    Serial.println("Edge Impulse Inferencing Demo");

    pinMode(5, OUTPUT);
    digitalWrite(5, LOW);
}

void loop()
{
    if (millis() > last_interval_ms + INTERVAL_MS) {
        last_interval_ms = millis();

        // Read power from ODrive UART
        odrive_serial << "r axis0.controller.electrical_power\n";
        // Save power reading to variable
        power = odrive.readFloat();

        // Put power reading in feature array
        features[feature_ix++] = power;

        // Features buffer full? then classify!
        if (feature_ix == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
            ei_impulse_result_t result;

            // Create signal from features frame
            signal_t signal;
            numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);

            // Run classifier
            EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
            ei_printf("run_classifier returned: %d\n", res);
            if (res != 0) return;

            // Print predictions
            ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);

            for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                ei_printf("%s:\t%.5f\n", result.classification[ix].label, result.classification[ix].value);
            }
      
        #if EI_CLASSIFIER_HAS_ANOMALY == 1
            ei_printf("anomaly:\t%.3f\n", result.anomaly);
        #endif
            // reset features frame
            feature_ix = 0;

        // Turn on fault LED if anomaly value passes threshold value
        if (result.anomaly >= anomaly_threshold) {
          digitalWrite(5, HIGH);
          }
        else {          
          digitalWrite(5, LOW);
          }
        }
    }    
}
