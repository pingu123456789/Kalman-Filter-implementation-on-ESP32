extern "C" {
#include "esp_heap_caps.h"
}

#include <Adafruit_SCD30.h>
Adafruit_SCD30 scd30;

#include "DHT.h"
#define DHTTYPE DHT22
#define dht_dpin1 32
#define dht_dpin2 33
#define dht_dpin3 34
#define dht_dpin4 35
DHT dht1(dht_dpin1, DHTTYPE);
DHT dht2(dht_dpin2, DHTTYPE);
DHT dht3(dht_dpin3, DHTTYPE);
DHT dht4(dht_dpin4, DHTTYPE);

float Q = 0.01;   // process noise
float R = 0.004;  // measurement noise for VOC
float P = Q;      // initial estimation error
float X;          // initial estimate
float K;          // kalman gain

float previous_reference_humidity = 0.0;  // stores the previous reference humidity

#define WINDOW_SIZE 10  // Window size for variance calculation
float sensor1_values[WINDOW_SIZE];
float sensor2_values[WINDOW_SIZE];
float sensor3_values[WINDOW_SIZE];
float sensor4_values[WINDOW_SIZE];

// We need to store the reference sensor's values to calculate its variance
float reference_values[WINDOW_SIZE];

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
  dht1.begin();
  dht2.begin();
  dht3.begin();
  dht4.begin();
  delay(1000);

  // initialize the initial estimate
  if (scd30.dataReady()) {
    if (!scd30.read()) { return; }
    previous_reference_humidity = scd30.temperature;
    X = previous_reference_humidity;
  }
}

float variance(float* values) {
  float mean = 0.0;
  float sq_sum = 0.0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    mean += values[i];
    sq_sum += values[i] * values[i];
  }
  mean /= WINDOW_SIZE;
  float variance = sq_sum / WINDOW_SIZE - mean * mean;
  return variance;
}

void loop() {
  if (scd30.dataReady()) {

    if (!scd30.read()) { return; }

    unsigned long startMillis = millis();
    unsigned long startHeap = ESP.getFreeHeap();

    // move values to the left
    for (int i = 0; i < WINDOW_SIZE - 1; i++) {
      sensor1_values[i] = sensor1_values[i + 1];
      sensor2_values[i] = sensor2_values[i + 1];
      sensor3_values[i] = sensor3_values[i + 1];
      sensor4_values[i] = sensor4_values[i + 1];
      reference_values[i] = reference_values[i + 1];
    }

    // read new values
    sensor1_values[WINDOW_SIZE - 1] = dht1.readTemperature();                                // low cost sensor 1 reading
    sensor2_values[WINDOW_SIZE - 1] = (scd30.temperature + dht1.readTemperature()) / 2;      // virtual low cost sensor 2 reading
    sensor3_values[WINDOW_SIZE - 1] = (2 * scd30.temperature + dht1.readTemperature()) / 3;  // virtual low cost sensor 3 reading
    sensor4_values[WINDOW_SIZE - 1] = (3 * scd30.temperature + dht1.readTemperature()) / 4;  // virtual low cost sensor 4 reading
    reference_values[WINDOW_SIZE - 1] = scd30.temperature;
    float reference_humidity = scd30.temperature;  // reference sensor reading

    float var1 = variance(sensor1_values);
    float var2 = variance(sensor2_values);
    float var3 = variance(sensor3_values);
    float var4 = variance(sensor4_values);
    float var_ref = variance(reference_values);

    float w1 = 1 / var1;
    float w2 = 1 / var2;
    float w3 = 1 / var3;
    float w4 = 1 / var4;
    float w_ref = 1 / var_ref;

    // normalize the weights
    float w_sum = w1 + w2 + w3 + w4 + w_ref;
    w1 /= w_sum;
    w2 /= w_sum;
    w3 /= w_sum;
    w4 /= w_sum;
    w_ref /= w_sum;

    // compute the fused low cost sensor reading
    float fused_humidity = w1 * sensor1_values[WINDOW_SIZE - 1] + w2 * sensor2_values[WINDOW_SIZE - 1] + w3 * sensor3_values[WINDOW_SIZE - 1] + w4 * sensor4_values[WINDOW_SIZE - 1] + w_ref * reference_values[WINDOW_SIZE - 1];


    // Generate Gaussian random numbers
    float U1 = (float)random(0, 32767) / 32767;
    float U2 = (float)random(0, 32767) / 32767;
    float gaussian_random = sqrt(-2 * log(U1)) * cos(2 * PI * U2);

    // Kalman filter equations
    // Step 1: Predict
    float u = 0;
    if (previous_reference_humidity != reference_humidity) {
      u = (reference_humidity - previous_reference_humidity) + gaussian_random * Q;
    }
    X += u;
    P += Q;

    // Every 10 steps, take a measurement
    static int step = 0;
    step++;
    if (step % 10 == 0) {
      // Add measurement noise
      float z = fused_humidity + gaussian_random * R;

      // Step 2: Update
      float y = z - X;
      K = P / (P + R);
      X = X + K * y;
      P = (1 - K) * P;
    }

    previous_reference_humidity = reference_humidity;

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

    Serial.print("Reference Humidity: ");
    Serial.print(reference_humidity);
    Serial.print(",");
    Serial.print("\tFused Humidity: ");
    Serial.print(fused_humidity);
    Serial.print(",");
    Serial.print("\tEstimated Humidity: ");
    Serial.println(X);

    delay(1000);
  }
}