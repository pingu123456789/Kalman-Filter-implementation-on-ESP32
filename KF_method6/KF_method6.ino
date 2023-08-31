extern "C" {
#include "esp_heap_caps.h"
}

#include <Adafruit_SCD30.h>
#include <DHT.h>

#define DHTTYPE DHT22
#define dht_dpin 32
#define VOC_PIN 35

DHT dht(dht_dpin, DHTTYPE);
Adafruit_SCD30 scd30;

#define WINDOW_SIZE 5
float sensor1_values[WINDOW_SIZE];
float sensor2_values[WINDOW_SIZE];
float sensor3_values[WINDOW_SIZE];
float sensor4_values[WINDOW_SIZE];
float sensor5_values[WINDOW_SIZE];
float sensor6_values[WINDOW_SIZE];
float sensor7_values[WINDOW_SIZE];

const int dim_x = 7;  // Number of states
const int dim_z = 7;  // Number of measurements

float x[dim_x] = { 0, 0, 0, 0, 0, 0, 0 };  // Initial state

float P[dim_x][dim_x];  // Initial covariance matrix
float F[dim_x][dim_x];  // State transition matrix
float H[dim_z][dim_x];  // Measurement matrix
float R[dim_z][dim_z];  // Measurement noise covariance matrix
float Q[dim_x][dim_x];  // Process noise covariance matrix

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

  // Initialize sensor values
  for (int i = 0; i < WINDOW_SIZE; i++) {
    sensor1_values[i] = sensor2_values[i] = sensor3_values[i] = sensor4_values[i] = 0;
  }

  // Initialize Kalman filter matrices (P, F, H, Q, R) here ...
  for (int i = 0; i < dim_x; i++) {
    for (int j = 0; j < dim_x; j++) {
      F[i][j] = i == j ? 1 : 0;       // Identity matrix
      Q[i][j] = i == j ? 0.0001 : 0;  // Process noise covariance
      P[i][j] = i == j ? 1 : 0;       // Initial covariance matrix
    }
    for (int j = 0; j < dim_z; j++) {
      H[i][j] = i == j ? 1 : 0;      // Measurement matrix
      R[i][j] = i == j ? 0.004 : 0;  // Measurement noise covariance
    }
  }

  delay(1000);
}

void loop() {
  if (scd30.dataReady()) {
    if (!scd30.read()) { return; }

    unsigned long startTime = micros();
    unsigned long startMillis = millis();
    unsigned long startHeap = ESP.getFreeHeap();

    // Read new values
    float temperature = dht.readTemperature();
    float reference_temperature = scd30.temperature;
    sensor1_values[WINDOW_SIZE - 1] = temperature;                                    // low cost sensor 1 reading
    sensor2_values[WINDOW_SIZE - 1] = (reference_temperature + temperature) / 2;      // virtual low cost sensor 2 reading
    sensor3_values[WINDOW_SIZE - 1] = (2 * reference_temperature + temperature) / 3;  // virtual low cost sensor 3 reading
    sensor4_values[WINDOW_SIZE - 1] = (3 * reference_temperature + temperature) / 4;  // virtual low cost sensor 4 reading
    sensor5_values[WINDOW_SIZE - 1] = (4 * reference_temperature + temperature) / 5;  // virtual low cost sensor 2 reading
    sensor6_values[WINDOW_SIZE - 1] = (5 * reference_temperature + temperature) / 6;  // virtual low cost sensor 3 reading
    sensor7_values[WINDOW_SIZE - 1] = (6 * reference_temperature + temperature) / 7;  // virtual low cost sensor 4 reading

    float sensor_data[dim_z] = {
      sensor1_values[WINDOW_SIZE - 1],
      sensor2_values[WINDOW_SIZE - 1],
      sensor3_values[WINDOW_SIZE - 1],
      sensor4_values[WINDOW_SIZE - 1],
      sensor5_values[WINDOW_SIZE - 1],
      sensor6_values[WINDOW_SIZE - 1],
      sensor7_values[WINDOW_SIZE - 1],
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
      y[i] = sensor_data[i];
      for (int j = 0; j < dim_x; j++) {
        y[i] -= H[i][j] * x_pred[j];
      }
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


    unsigned long endTime = micros();

    unsigned long elapsedTime = endTime - startTime;

    Serial.print("Time to read from DHT sensor: ");
    Serial.println(elapsedTime);


    // Print estimated vs measured values
    for (int i = 0; i < dim_x; i++) {
      Serial.print("Estimated temperature at sensor ");
      Serial.print(i + 1);
      Serial.print(" using multi-variable KF: ");
      Serial.print(x[i]);
      Serial.print(", Measured temperature: ");
      Serial.println(sensor_data[i]);
    }

    delay(1000);
  }
}