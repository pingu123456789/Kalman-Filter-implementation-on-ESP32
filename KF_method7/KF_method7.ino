#include <Adafruit_SCD30.h>
#include <DHT.h>

#define DHTTYPE DHT22
#define dht_dpin 32
#define N_READINGS 5
#define WINDOW_SIZE 5

Adafruit_SCD30 scd30;
DHT dht(dht_dpin, DHTTYPE);

float X[N_READINGS];
float Y[N_READINGS];
float beta[2] = { 0, 0 };
float P[2][2] = { { 0, 0 }, { 0, 0 } };

float sigma_beta_sq = 0.01;     // variance of the error associated with the changing slope
float sigma_epsilon_sq = 0.01;  // residual variance of the regression model

int reading_idx = 0;

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
    while (1) {
      delay(10);
    }
  }

  scd30.getMeasurementInterval();
  dht.begin();
  delay(1000);
}

void loop() {
  if (scd30.dataReady()) {
    if (!scd30.read()) { return; }

    unsigned long startTime = micros();
    unsigned long startMillis = millis();
    unsigned long startHeap = ESP.getFreeHeap();

    float low_cost_temperature = dht.readTemperature();  // low cost sensor reading
    float reference_temperature = scd30.temperature;     // reference sensor reading

    // Store readings
    X[reading_idx] = low_cost_temperature;
    Y[reading_idx] = reference_temperature;
    reading_idx++;

    if (reading_idx < N_READINGS) {
      X[reading_idx] = dht.readTemperature();
      Y[reading_idx] = scd30.temperature;
      reading_idx++;
    }

    if (reading_idx == N_READINGS) {
      // Initial regression
      float sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
      for (int i = 0; i < WINDOW_SIZE; i++) {
        sum_x += X[i];
        sum_y += Y[i];
        sum_xy += X[i] * Y[i];
        sum_xx += X[i] * X[i];
      }
      float mean_x = sum_x / WINDOW_SIZE;
      float mean_y = sum_y / WINDOW_SIZE;
      beta[1] = (sum_xy - WINDOW_SIZE * mean_x * mean_y) / (sum_xx - WINDOW_SIZE * mean_x * mean_x);  // slope
      beta[0] = mean_y - beta[1] * mean_x;                                                            // intercept

      // Now apply the Kalman filter on the remaining data
      for (int k = WINDOW_SIZE; k < N_READINGS; k++) {
        // Predict
        float beta_k_minus_1[2] = { beta[0], beta[1] };
        float P_k_minus_1[2][2] = { { P[0][0], P[0][1] }, { P[1][0], P[1][1] + sigma_beta_sq } };

        // Measurement residual
        float x_k = X[k];
        float y_k = Y[k];
        float r_k = y_k - beta_k_minus_1[0] - beta_k_minus_1[1] * x_k;

        // Residual covariance
        float S_k = 1 + x_k * P_k_minus_1[1][1] + sigma_epsilon_sq;

        // Kalman gain
        float k_k[2] = { 0, x_k * P_k_minus_1[1][1] / S_k };

        // Update
        beta[0] = beta_k_minus_1[0] + k_k[0] * r_k;
        beta[1] = beta_k_minus_1[1] + k_k[1] * r_k;
        P[0][0] = P_k_minus_1[0][0] - k_k[0] * P_k_minus_1[0][0];  //offdiagonal element
        P[0][1] = P_k_minus_1[0][1] - k_k[0] * P_k_minus_1[0][1];
        P[1][0] = P_k_minus_1[1][0] - k_k[1] * P_k_minus_1[1][0];  //offdiagonal element
        P[1][1] = P_k_minus_1[1][1] - k_k[1] * P_k_minus_1[1][1];
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

      // Print actual and estimated values
      Serial.print("Actual X: ");
      Serial.println(X[reading_idx - 1]);
      Serial.print("Actual Y: ");
      Serial.println(Y[reading_idx - 1]);
      float estimated_Y = beta[0] + beta[1] * X[reading_idx - 1];
      Serial.print("Estimated Y: ");
      Serial.println(estimated_Y);

      // Print the final values
      Serial.print("Final estimated slope (beta): ");
      Serial.println(beta[1]);
      Serial.print("Final estimated intercept (alpha): ");
      Serial.println(beta[0]);

      // Reset the reading index for the next cycle
      reading_idx = 0;
    }
    delay(1000);
  }
}