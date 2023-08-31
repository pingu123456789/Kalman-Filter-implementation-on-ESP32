#include <Adafruit_SCD30.h>
Adafruit_SCD30 scd30;

#include "DHT.h"
#define DHTTYPE DHT22
#define dht_dpin 32
DHT dht(dht_dpin, DHTTYPE);

float Q = 0.01;  // process noise
float R = 2;     // measurement noise for VOC
float P = Q;     // initial estimation error
float X;         // initial estimate
float K;         // kalman gain

float previous_reference_humidity = 0.0;  // stores the previous reference humidity

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

  // initialize the initial estimate
  if (scd30.dataReady()) {
    if (!scd30.read()) { return; }
    previous_reference_humidity = scd30.temperature;
    X = previous_reference_humidity;
  }
}

void loop() {
  if (scd30.dataReady()) {

    if (!scd30.read()) { return; }

    // Start of CPU and memory usage measurement
    unsigned long startMillis = millis();
    unsigned long startHeap = ESP.getFreeHeap();

    // Add the following line to allocate memory dynamically
    char* testAllocation = new char[500];

    float low_cost_humidity = dht.readTemperature();  // low cost sensor reading
    float reference_humidity = scd30.temperature;     // reference sensor reading

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
      float z = low_cost_humidity + gaussian_random * R;

      // Step 2: Update
      float y = z - X;
      K = P / (P + R);
      X = X + K * y;
      P = (1 - K) * P;
    }

    previous_reference_humidity = reference_humidity;

    Serial.print("Reference Humidity: ");
    Serial.print(reference_humidity);
    Serial.print(",");
    Serial.print("\tLow Cost Humidity: ");
    Serial.print(low_cost_humidity);
    Serial.print(",");
    Serial.print("\tEstimated Humidity: ");
    Serial.println(X);

     // And add the following line to deallocate the memory
  delete[] testAllocation;

  unsigned long endMillis = millis();
  unsigned long endHeap = ESP.getFreeHeap();

  unsigned long cpuTime = endMillis - startMillis;
  long memoryUsage = startHeap - endHeap;

  Serial.print("CPU Time: ");
  Serial.print(cpuTime);
  Serial.println(" ms");
  Serial.print("Memory Usage: ");
  Serial.print(memoryUsage);
  Serial.println(" bytes");
  
  delay(1000);
  }
}