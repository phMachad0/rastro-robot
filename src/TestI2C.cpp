/*
 * MPU6050 Test
 * Reads acceleration and gyroscope data from the MPU6050 module
 * and prints the values to the Serial Monitor.
 */

 #include <Arduino.h>
 #include <Wire.h>
 #include <MPU6050.h>
 
 // Set LED_BUILTIN if it is not defined by Arduino framework
 #ifndef LED_BUILTIN
     #define LED_BUILTIN 2
 #endif
 
 MPU6050 mpu;
 
 void testI2CSetup()
 {
    // Initialize Serial Monitor
    Serial.begin(115200);
    while (!Serial); // Wait for Serial Monitor to open (only needed for some boards)

    // Initialize I²C communication
    Wire.begin();
    
    // Initialize MPU6050
    Serial.println("Initializing MPU6050...");
    mpu.initialize();
    
    Serial.println("Init sleep...");
    sleep(2);
    Serial.println("End sleep...");
    
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250); 

    // Configure LED pin
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.println("MPU6050 initialized successfully!");
 }
 
 void testI2CLoop()
 {
    // Read acceleration data
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);

    // Read gyroscope data
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);

    const float SCALE_ACCEL = 16384.0;
    const float SCALE_GYRO = 131.0;

    // Variáveis para os dados convertidos (unidades físicas)
    float accX = ax / SCALE_ACCEL; // Resultado em g
    float accY = ay / SCALE_ACCEL; // Resultado em g
    float accZ = az / SCALE_ACCEL; // Resultado em g

    float gyroX = gx / SCALE_GYRO; // Resultado em graus/segundo (°/s)
    float gyroY = gy / SCALE_GYRO; // Resultado em graus/segundo (°/s)
    float gyroZ = gz / SCALE_GYRO; // Resultado em graus/segundo (°/s)

    // Print acceleration data
    Serial.print("Accel X: "); Serial.print(accX);
    Serial.print(" | Y: "); Serial.print(accY);
    Serial.print(" | Z: "); Serial.println(accZ);

    // Print gyroscope data
    Serial.print("Gyro X: "); Serial.print(gyroX);
    Serial.print(" | Y: "); Serial.print(gyroY);
    Serial.print(" | Z: "); Serial.println(gyroZ);

    // Blink the LED to indicate activity
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
 }