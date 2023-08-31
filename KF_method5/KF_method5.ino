extern "C" {
#include "esp_heap_caps.h"
}

#include <DHT.h>

#define DHTPIN 32      // what pin we're connected to
#define DHTTYPE DHT22  // DHT 22  (AM2302)

DHT dht(DHTPIN, DHTTYPE);

#include <Adafruit_SCD30.h>
Adafruit_SCD30 scd30;

// State transition matrix F
float F[4][4] = { { 1, 0, 0, 0 },
                  { 0, 1, 0, 0 },
                  { 0, 0, 1, 0 },
                  { 0, 0, 0, 1 } };

// Observation matrix H
float H[2][4] = { { 1, 0, 0, 0 },
                  { 0, 1, 0, 0 } };

// Measurement noise covariance R
float R[2][2] = { { 0.0004, 0 },
                  { 0, 4 } };

// Process noise covariance Q
float Q[4][4] = { { 1, 0, 0, 0 },
                  { 0, 1, 0, 0 },
                  { 0, 0, 1, 0 },
                  { 0, 0, 0, 1 } };

// State covariance matrix P
float P[4][4] = { { 1, 0, 0, 0 },
                  { 0, 1, 0, 0 },
                  { 0, 0, 1, 0 },
                  { 0, 0, 0, 1 } };

void printMemInfo() {
  multi_heap_info_t info;

  size_t freeBytes = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  size_t totalBytes = ESP.getHeapSize();  // Total SRAM in ESP32
  size_t usedBytes = totalBytes - freeBytes;

  // Internal RAM
  heap_caps_get_info(&info, MALLOC_CAP_INTERNAL);
  Serial.print("Internal Total Free RAM: ");
  Serial.print(info.total_free_bytes / 1024);
  Serial.print(" KB, ");
  Serial.print(info.total_free_bytes / 1024 / 1024);
  Serial.println(" MB");

  Serial.print("Internal Total Allocated RAM: ");
  Serial.print(info.total_allocated_bytes / 1024);
  Serial.print(" KB, ");
  Serial.print(info.total_allocated_bytes / 1024 / 1024);
  Serial.println(" MB");

  // SPIRAM
  /*heap_caps_get_info(&info, MALLOC_CAP_SPIRAM);
  Serial.print("SPIRAM Total Free RAM: ");
  Serial.print(info.total_free_bytes / 1024);
  Serial.print(" KB, ");
  Serial.print(info.total_free_bytes / 1024 / 1024);
  Serial.println(" MB");

  Serial.print("SPIRAM Total Allocated RAM: ");
  Serial.print(info.total_allocated_bytes / 1024);
  Serial.print(" KB, ");
  Serial.print(info.total_allocated_bytes / 1024 / 1024);
  Serial.println(" MB");*/

  Serial.print("Total SRAM: ");
  Serial.print(totalBytes / 1024);
  Serial.println("KB");

  Serial.print("Free SRAM: ");
  Serial.print(freeBytes / 1024);
  Serial.println("KB");

  Serial.print("Used SRAM: ");
  Serial.print(usedBytes / 1024);
  Serial.println("KB");
}

// Initial state vector
float X[4] = { 20, 50, 0, 0 };

// Function to calculate heat index
float calculateHeatIndex(float temp, float humidity) {
  return 0.5 * (temp + 61.0 + ((temp - 68.0) * 1.2) + (0.094 * humidity));
}

// Function to calculate dew point
float calculateDewPoint(float temp, float humidity) {
  float h = (log(humidity / 100) + ((17.625 * temp) / (243.04 + temp))) / (17.625 - log(humidity / 100) - (17.625 * temp / (243.04 + temp)));
  return 243.04 * h;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  if (!scd30.begin()) {
    Serial.println("Failed to initialize SCD30 sensor!");
    while (1) {
      delay(10);
    }
  }

  scd30.setMeasurementInterval(2);
  dht.begin();

  delay(1000);
}


void loop() {
  if (scd30.dataReady()) {
    if (!scd30.read()) { return; }

    unsigned long startTime = micros();
    unsigned long startMillis = millis();
    unsigned long startHeap = ESP.getFreeHeap();

    // Get temperature and humidity data from DHT22 sensor
    float temp = dht.readTemperature();
    float humidity = dht.readHumidity();

    // Calculate heat index and dew point
    float heatIndex = calculateHeatIndex(temp, humidity);
    float dewPoint = calculateDewPoint(temp, humidity);

    // Set calculated values in state vector
    X[2] = heatIndex;
    X[3] = dewPoint;

    // Implement the Kalman filter here
    // prediction for state X
    float X_pred[4];
    for (int i = 0; i < 4; i++) {
      X_pred[i] = X[i];
    }

    // prediction for covariance P
    float P_pred[4][4];
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        P_pred[i][j] = P[i][j];
      }
    }
    // measurement vector
    float Z[2] = { temp, humidity };
    // Innovation or measurement residual
    float y[2];
    y[0] = Z[0] - H[0][0] * X_pred[0] - H[0][1] * X_pred[1] - H[0][2] * X_pred[2] - H[0][3] * X_pred[3];
    y[1] = Z[1] - H[1][0] * X_pred[0] - H[1][1] * X_pred[1] - H[1][2] * X_pred[2] - H[1][3] * X_pred[3];

    // Innovation (or residual) covariance
    float S[2][2];
    S[0][0] = H[0][0] * P_pred[0][0] * H[0][0] + R[0][0];
    S[0][1] = H[0][0] * P_pred[0][1] * H[0][0];
    S[1][0] = H[1][0] * P_pred[1][0] * H[1][0];
    S[1][1] = H[1][0] * P_pred[1][1] * H[1][0] + R[1][1];

    // Optimal Kalman gain
    float K[4][2];
    float S_inv[2][2] = { { 1 / S[0][0], 0 },
                          { 0, 1 / S[1][1] } };

    K[0][0] = P_pred[0][0] * H[0][0] * S_inv[0][0];
    K[0][1] = P_pred[0][1] * H[1][0] * S_inv[1][1];
    K[1][0] = P_pred[1][0] * H[0][0] * S_inv[0][0];
    K[1][1] = P_pred[1][1] * H[1][0] * S_inv[1][1];
    K[2][0] = P_pred[2][0] * H[0][0] * S_inv[0][0];
    K[2][1] = P_pred[2][1] * H[1][0] * S_inv[1][1];
    K[3][0] = P_pred[3][0] * H[0][0] * S_inv[0][0];
    K[3][1] = P_pred[3][1] * H[1][0] * S_inv[1][1];

    // Updated (a posteriori) state estimate
    float X[4];
    X[0] = X_pred[0] + K[0][0] * y[0] + K[0][1] * y[1];
    X[1] = X_pred[1] + K[1][0] * y[0] + K[1][1] * y[1];
    X[2] = X_pred[2] + K[2][0] * y[0] + K[2][1] * y[1];
    X[3] = X_pred[3] + K[3][0] * y[0] + K[3][1] * y[1];

    // Updated (a posteriori) estimate covariance
    float P[4][4];
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        P[i][j] = (1 - K[i][0] * H[0][j] - K[i][1] * H[1][j]) * P_pred[i][j];
      }
    }

    // At this point, X contains the updated state vector, and P is the updated covariance matrix.
    // For the next time step, you would use these as the new initial state vector and covariance matrix.
    // ...

    unsigned long endMillis = millis();
    unsigned long endHeap = ESP.getFreeHeap();

    // Calculate CPU Time and Memory Usage
    unsigned long cpuTime = endMillis - startMillis;
    long memoryUsage = startHeap - endHeap;

    Serial.print("CPU Time: ");
    Serial.print(cpuTime);
    Serial.println(" ms");

    Serial.print("Memory Usage: ");
    Serial.print(memoryUsage);
    Serial.println(" bytes");

    printMemInfo();


    unsigned long endTime = micros();

    unsigned long elapsedTime = endTime - startTime;

    Serial.print("Time to read from DHT sensor: ");
    Serial.println(elapsedTime);

    // Output results
    Serial.print("Temperature: ");
    Serial.println(temp);
    Serial.print("Humidity: ");
    Serial.println(humidity);
    Serial.print("Heat Index: ");
    Serial.println(heatIndex);
    Serial.print("Dew Point: ");
    Serial.println(dewPoint);

    delay(1000);
  }
}