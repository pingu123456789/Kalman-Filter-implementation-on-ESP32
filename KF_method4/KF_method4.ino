extern "C" {
#include "esp_heap_caps.h"
}

#include <Adafruit_SCD30.h>
#include <DHT.h>
#define DHTTYPE DHT22
#define dht_dpin 32
DHT dht(dht_dpin, DHTTYPE);
#define VOC_PIN 35

Adafruit_SCD30 scd30;

const int dim_x = 4;  // Number of states
const int dim_z = 4;  // Number of measurements

float x[dim_x] = { 0, 0, 0, 0 };  // Initial state
float P[dim_x][dim_x] = { { 1, 0, 0, 0 },
                          { 0, 1, 0, 0 },
                          { 0, 0, 1, 0 },
                          { 0, 0, 0, 1 } };  // Initial covariance matrix

float F[dim_x][dim_x] = { { 1, 0, 0, 0 },
                          { 0, 1, 0, 0 },
                          { 0, 0, 1, 0 },
                          { 0, 0, 0, 1 } };  // State transition matrix

float H[dim_z][dim_x] = { { 1, 0, 0, 0 },
                          { 0, 1, 0, 0 },
                          { 0, 0, 1, 0 },
                          { 0, 0, 0, 1 } };  // Measurement matrix

float R[dim_z][dim_z] = { { 0.2, 0, 0, 0 },
                          { 0, 2, 0, 0 },
                          { 0, 0, 30, 0 },
                          { 0, 0, 0, 5 } };  // Measurement noise covariance matrix

float Q[dim_x][dim_x] = { { 0.1, 0, 0, 0 },
                          { 0, 0.1, 0, 0 },
                          { 0, 0, 0.1, 0 },
                          { 0, 0, 0, 0.1 } };  // Process noise covariance matrix

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

    unsigned long startMillis = millis();
    unsigned long startHeap = ESP.getFreeHeap();

    // Get sensor readings
    float sensor_data[dim_z] = {
      dht.readTemperature(),
      dht.readHumidity(),
      scd30.CO2,
      analogRead(VOC_PIN) * (5.0 / 1023.0)  // Convert ADC value to voltage
    };

    // Kalman filter prediction step
    float x_pred[dim_x];
    for (int i = 0; i < dim_x; i++) {
      x_pred[i] = 0;
      for (int j = 0; j < dim_x; j++) {
        x_pred[i] += F[i][j] * x[j];
      }
    }

    // Predict covariance
    float P_pred[dim_x][dim_x] = { 0 };
    for (int i = 0; i < dim_x; i++) {
      for (int j = 0; j < dim_x; j++) {
        for (int k = 0; k < dim_x; k++) {
          P_pred[i][j] += F[i][k] * P[k][j];
        }
        P_pred[i][j] += Q[i][j];
      }
    }

    // Kalman filter update step
    float y[dim_z];
    for (int i = 0; i < dim_z; i++) {
      y[i] = sensor_data[i] - H[i][0] * x_pred[0] - H[i][1] * x_pred[1] - H[i][2] * x_pred[2] - H[i][3] * x_pred[3];
    }

    float S[dim_z][dim_z];
    for (int i = 0; i < dim_z; i++) {
      for (int j = 0; j < dim_z; j++) {
        S[i][j] = 0;
        for (int k = 0; k < dim_x; k++) {
          S[i][j] += H[i][k] * P_pred[k][j] * H[j][k];
        }
        S[i][j] += R[i][j];
      }
    }

    float K[dim_x][dim_z];
    for (int i = 0; i < dim_x; i++) {
      for (int j = 0; j < dim_z; j++) {
        K[i][j] = 0;
        for (int k = 0; k < dim_z; k++) {
          K[i][j] += P_pred[i][k] * H[k][j];
        }
        K[i][j] /= S[j][j];
      }
    }

    for (int i = 0; i < dim_x; i++) {
      x[i] = x_pred[i];
      for (int j = 0; j < dim_z; j++) {
        x[i] += K[i][j] * y[j];
      }
    }

    // Update covariance
    for (int i = 0; i < dim_x; i++) {
      for (int j = 0; j < dim_x; j++) {
        P[i][j] = P_pred[i][j];
        for (int k = 0; k < dim_z; k++) {
          P[i][j] -= K[i][k] * S[k][j] * K[j][i];
        }
      }
    }

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

    // Print estimated vs measured values
    Serial.print("Estimated temperature using multi variable KF: ");
    Serial.print(x[0]);
    Serial.print(", Measured temperature: ");
    Serial.println(sensor_data[0]);
    Serial.print("Estimated humidity using multi variable KF: ");
    Serial.print(x[1]);
    Serial.print(", Measured humidity: ");
    Serial.println(sensor_data[1]);
    Serial.print("Estimated CO2 ppm using multi variable KF: ");
    Serial.print(x[2]);
    Serial.print(", Measured CO2 : ");
    Serial.println(sensor_data[2]);
    Serial.print("Estimated VOC ppm using multi variable KF: ");
    Serial.print(x[3]);
    Serial.print(", Measured VOC level: ");
    Serial.println(sensor_data[3]);

    delay(1000);
  }
}