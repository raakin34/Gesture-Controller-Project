#include <Gestures_inferencing.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <esp_now.h>
#include <WiFi.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9
// #define FAST_MODE
#define BNO08X_RESET -1
#define FREQUENCY_HZ        1000000
#define INTERVAL_MS         (1000 / (FREQUENCY_HZ + 1))

#define PIN_12  12  // ADC2 Channel 5
#define PIN_14  14  // ADC2 Channel 6
#define PIN_32  32  // ADC1 Channel 4
#define PIN_35  35  // ADC1 Channel 7

static unsigned long last_interval_ms = 0;

// to classify 1 frame of data you need EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE values
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
// keep track of where we are in the feature array
size_t feature_ix = 0;

float OffRoll = 0.0;
float OffPitch = 0.0;
float OffYaw = 0.0;

int wavec = 0;
int tsuc = 0;
int comsic = 0;

uint8_t broadcastAddress[] = {0x3C, 0x8A, 0x1F, 0xAF, 0x5C, 0x44}; //3C:8A:1F:AF:5C:44

int send_value;

String success; //--> Variable to store if sending data was successful.

//----------------------------------------Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
    int value;
} struct_message;

struct_message send_Data; //--> Create a struct_message to send data.
//----------------------------------------
esp_now_peer_info_t peerInfo;

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif

void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}


void setup(void) {

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(1000000); // Set I2C clock to 400kHz
  WiFi.mode(WIFI_STA); //--> Set device as a Wi-Fi Station.

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  //----------------------------------------
  
  //----------------------------------------Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  //----------------------------------------
  
  //----------------------------------------Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  //----------------------------------------
  
  //----------------------------------------Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  setReports(reportType, reportIntervalUs);

  FilterCom();

  delay(100);
}


void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}


void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void loop() {

  float yaw, pitch, roll;

  int raw12 = analogRead(PIN_12);
  int raw14 = analogRead(PIN_14);
  int raw32 = analogRead(PIN_32);
  int raw35 = analogRead(PIN_35);

// ESP32 ADC is 12-bit (0-4095), assuming 3.3V reference
  float voltage12 = raw12 * (3.3 / 4095.0);
  float voltage14 = raw14 * (3.3 / 4095.0);
  float voltage32 = raw32 * (3.3 / 4095.0);
  float voltage35 = raw35 * (3.3 / 4095.0);

  send_Data.value = send_value;
  //----------------------------------------Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &send_Data, sizeof(send_Data));

  if(raw32 > 2 && raw35 > 2){
    Serial.println("Stop");
    send_value = 0; //Forward
    send_Data.value = send_value;
  //----------------------------------------Send message via ESP-NOW
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &send_Data, sizeof(send_Data));
      return;
    }
 

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }
//f (millis() > last_interval_ms + INTERVAL_MS) {
        //last_interval_ms = millis();

  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
    static long last = 0;
    long now = micros();
    last = now;

    pitch = ypr.pitch - OffPitch;
    roll = ypr.roll - OffRoll;
    yaw = ypr.yaw - OffYaw;

    features[feature_ix++] = yaw;
    features[feature_ix++] = pitch; 
    features[feature_ix++] = roll; 

    if (feature_ix == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
      ei_impulse_result_t result;

      // create signal from features frame
      signal_t signal;
      numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);

      // run classifier
      EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
//      ei_printf("run_classifier returned: %d\n", res);
      if (res != 0) return;

      // print predictions
//      ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
//                result.timing.dsp, result.timing.classification, result.timing.anomaly);

      // print the predictions
//      for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
//        ei_printf("%s:\t%.5f\n", result.classification[ix].label, result.classification[ix].value);
//      }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
//      ei_printf("anomaly:\t%.3f\n", result.anomaly);
#endif

    // reset features frame
    feature_ix = 0;

//      Serial.print(ypr.yaw - OffYaw);                Serial.print(",");
//      Serial.print(ypr.pitch - OffPitch);              Serial.print(",");
//      Serial.println(ypr.roll - OffRoll);
//      Serial.println(wavec);
//      Serial.println(tsuc);

      if(result.classification[0].value > 0.75){
        comsic++;
        if(tsuc < 3 && wavec < 3 && comsic < 2){
          return;}   
      }  else{
        comsic = 0;
      }

      if(result.classification[2].value > 0.70){
        wavec++;
        if(wavec < 3 && tsuc < 3 && comsic < 2){
          return;}     
      } else{
        wavec = 0;
      }
      if(result.classification[1].value > 0.75){
        tsuc++;
        if(tsuc < 3 && wavec < 3 && comsic < 2){
          return;}   
      }  else{
        tsuc = 0;
      }

    
      if(tsuc < 3 && wavec < 3 && comsic < 2){
        if(ypr.pitch - OffPitch < -15){
          Serial.println("left");    
          send_value = 4; //Left
        } else if (ypr.pitch - OffPitch > 15){
          Serial.println("rigth");    
          send_value = 3; //Right
        } else if (ypr.roll - OffRoll > 15){
          Serial.println("forward");     
          send_value = 1; //Forward
        }  else if (ypr.roll - OffRoll < -15){
          Serial.println("reverse");
          send_value = 2; //Reverse
        }else{
          Serial.println("stationary"); 
          send_value = 5; //Stationary
        }
      }

      if(tsuc > 2){
        Serial.println("Tsunami");
         send_value = 6; //Forward
        delay(50);
      }

      if(wavec > 2){
        Serial.println("wave");
         send_value = 7; //Forward
        delay(50);
      }

      if(comsic > 1){
        Serial.println("comsi");
         send_value = 8; //Forward
        delay(50);
      }

    }
  }
}

void FilterCom() {
  if (bno08x.wasReset()) {
    Serial.print("Sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }
  
  OffRoll = 0.0;
  OffPitch = 0.0;
  OffYaw = 0.0;

  for (int i = 0; i < 500; i++) {
    if (bno08x.getSensorEvent(&sensorValue)) {
      switch (sensorValue.sensorId) {
        case SH2_ARVR_STABILIZED_RV:
          quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
          break;
        case SH2_GYRO_INTEGRATED_RV:
          quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
          break;
      }

      // Collect offsets properly
      OffYaw += ypr.yaw;
      OffPitch += ypr.pitch;
      OffRoll += ypr.roll;
    }
    delay(5); // Wait for new data
  }
  
  // Compute the average
  OffYaw /= 500;
  OffPitch /= 500;
  OffRoll /= 500;
}

void ei_printf(const char *format, ...) {
  static char print_buf[1024] = { 0 };

  va_list args;
  va_start(args, format);
  int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
  va_end(args);

  if (r > 0) {
    Serial.write(print_buf);
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
  //Serial.println(">>>>>");
}