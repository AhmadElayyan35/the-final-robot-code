#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <PID_v1_bc.h>
#include "Adafruit_VL53L0X.h"

// ===========================
// إعدادات محركات الروبوت
// ===========================
const int EnableA = 27;
const int motorIn1 = 14;
const int motorIn2 = 12;
const int EnableB = 4;
const int motorIn3 = 26;
const int motorIn4 = 25;

// ===========================
// إعدادات الإنكودر
// ===========================
const int EncoderA = 32;
const int EncoderB = 34;
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

// ===========================
// معايير الحركة
// ===========================
const int ticksPerCell = 140;          // عدد نبضات الإنكودر لمسافة خلية واحدة
const int targetTurnTicks = 50;         // (مثال) عدد نبضات للالتفاف باستخدام الإنكودر
// مدى الالتفاف لليمين (توقع أن تكون زاوية Yaw سالبة)
const int minTurnAngle = -100;
const int maxTurnAngle = -70;
// مدى الالتفاف لليسار (زاوية Yaw موجبة)
const int minTurnAngleLeft = 80;
const int maxTurnAngleLeft = 90;

const int correctionSpeed = 190;      // سرعة التصحيح باستخدام MPU

// ===========================
// متغيرات PID
// ===========================
double setpoint = 0, input, output;
double Kp = 0.35, Ki = 0.4, Kd = 0.05;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, P_ON_E, DIRECT);
int baseSpeedA = 180;
int baseSpeedB = 180;

// ===========================
// إعدادات MPU6050
// ===========================
Adafruit_MPU6050 mpu;
float yaw = 0.0;
float gyroZBias = 0.0;
float gyroThreshold = 0.1;
unsigned long lastTime = 0;

// ===========================
// إعدادات حساس الليدار VL53L0X
// ===========================
// عناوين الأجهزة (يمكن تعديلها)
#define LOX1_ADDRESS 0x30   // سيتم استخدامه حسب التخصيص أدناه
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

// دبابيس التحكم بحالة التشغيل (shutdown) للحساسات
#define SHT_LOX1 13
#define SHT_LOX2 19
#define SHT_LOX3 18

// إنشاء كائنات الحساسات
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();  // سيتم تخصيصه لاحقًا
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

// معايير قياس المسافات (بالميليمتر)
const int minDistance = 15;   // أقل مسافة للكشف عن جدار
const int maxDistance = 150;  // أعلى مسافة للكشف عن جدار

// ===========================
// دوال الحساسات (LiDAR)
// ===========================

// تعيين العناوين للحساسات
void setID() {
  // إعادة تعيين جميع الحساسات
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(10);
  
  // إيقاظ جميع الحساسات
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);
  
  // تهيئة الحساسات واحدًا تلو الآخر وفقاً للترتيب المطلوب:
  // - lox2 لتصبح الحساس الأمامي (بالعنوان LOX1_ADDRESS)
  // - lox3 لتصبح الحساس اليميني (بالعنوان LOX2_ADDRESS)
  // - lox1 لتصبح الحساس اليساري (بالعنوان LOX3_ADDRESS)
  
  // تهيئة الحساس الأمامي (استخدام lox2)
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX3, LOW);
  if (!lox2.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to initialize RIGHT sensor (now FRONT)"));
    while (1);
  }
  delay(10);
  
  // تهيئة الحساس اليميني (استخدام lox3)
  digitalWrite(SHT_LOX3, HIGH);
  digitalWrite(SHT_LOX1, LOW);
  if (!lox3.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to initialize LEFT sensor (now RIGHT)"));
    while (1);
  }
  delay(10);
  
  // تهيئة الحساس اليساري (استخدام lox1)
  digitalWrite(SHT_LOX1, HIGH);
  if (!lox1.begin(LOX3_ADDRESS)) {
    Serial.println(F("Failed to initialize FRONT sensor (now LEFT)"));
    while (1);
  }
}

// دوال القراءة من الحساسات
int readFrontDistance() {
  VL53L0X_RangingMeasurementData_t measure;
  lox2.rangingTest(&measure, false);  // استخدام lox2 للحساس الأمامي
  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter;
  } else {
    return -1;
  }
}

int readRightDistance() {
  VL53L0X_RangingMeasurementData_t measure;
  lox3.rangingTest(&measure, false);  // استخدام lox3 للحساس اليميني
  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter;
  } else {
    return -1;
  }
}

int readLeftDistance() {
  VL53L0X_RangingMeasurementData_t measure;
  lox1.rangingTest(&measure, false);  // استخدام lox1 للحساس اليساري
  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter;
  } else {
    return -1;
  }
}

// دوال الكشف عن الجدار
bool wallFront() {
  int distance = readFrontDistance();
  return (distance >= minDistance && distance <= maxDistance);
}

bool wallRight() {
  int distance = readRightDistance();
  return (distance >= minDistance && distance <= maxDistance);
}

bool wallLeft() {
  int distance = readLeftDistance();
  return (distance >= minDistance && distance <= maxDistance);
}

// دالة لطباعة القراءات
void printSensorReadings() {
  int front = readFrontDistance();
  int right = readRightDistance();
  int left = readLeftDistance();
  
  Serial.print("Front: "); Serial.print(front);
  Serial.print(" mm, Right: "); Serial.print(right);
  Serial.print(" mm, Left: "); Serial.print(left);
  Serial.println(" mm");
}

// ===========================
// دوال المقاطعات (الإنكودر)
// ===========================
void IRAM_ATTR handleEncoderA() { 
  encoderCountA++; 
}

void IRAM_ATTR handleEncoderB() { 
  encoderCountB++; 
}

// ===========================
// دوال MPU6050 والمعايرة
// ===========================
void initMPU() {
    Wire.begin();
    if (!mpu.begin()) {
        Serial.println("Failed to initialize MPU6050");
        while (1);
    }
    Serial.println("MPU6050 initialized!");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void calibrateGyro() {
    Serial.println("Calibrating gyroscope... Please keep the MPU6050 stationary.");
    delay(2000);
    float sumZ = 0.0;
    const int calibrationSamples = 100;
    for (int i = 0; i < calibrationSamples; i++) {
        sensors_event_t accel, gyro, temp;
        mpu.getEvent(&accel, &gyro, &temp);
        sumZ += gyro.gyro.z;
        delay(10);
    }
    gyroZBias = sumZ / calibrationSamples;
    Serial.print("Gyroscope Z-axis bias: ");
    Serial.println(gyroZBias, 6);
    Serial.println("Calibration complete!");
    lastTime = millis();
}

void updateYaw() {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;
    float correctedGyroZ = gyro.gyro.z - gyroZBias;
    correctedGyroZ *= 180.0 / M_PI;
    if (abs(correctedGyroZ) > gyroThreshold) {
        yaw += correctedGyroZ * dt;
        // إذا رغبت في عكس الإشارة (حسب تركيب المستشعر) فاستخدم:
        // yaw -= correctedGyroZ * dt;
    }
    Serial.print("Yaw Angle: ");
    Serial.println(yaw);
}

// ===========================
// دوال حركة الروبوت
// ===========================
void moveOneCell() {
    encoderCountA = 0;
    encoderCountB = 0;

    // تفعيل PID
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-50, 50);

    // تشغيل المحركات للأمام
    analogWrite(EnableA, baseSpeedA);
    analogWrite(EnableB, baseSpeedB);
    digitalWrite(motorIn1, HIGH);
    digitalWrite(motorIn2, LOW);
    digitalWrite(motorIn3, HIGH);
    digitalWrite(motorIn4, LOW);

    while (encoderCountA < ticksPerCell || encoderCountB < ticksPerCell) {
        // حساب الفرق بين الإنكودرات لتصحيح المسار
        input = encoderCountA - encoderCountB;

        if (abs(input) > 0.5) {
            myPID.Compute();
        } else {
            output = 0;
        }

        // تصحيح السرعات بناءً على PID
        int adjustedSpeedA = baseSpeedA - output;
        int adjustedSpeedB = baseSpeedB + output;

        // تصحيح الميلان باستخدام Yaw
        updateYaw(); // تحديث زاوية Yaw أثناء الحركة
        if (yaw > 2) {
            adjustedSpeedA -= 10;
        } else if (yaw < -2) {
            adjustedSpeedB -= 10;
        }

        // ضبط الحدود المسموحة للسرعة
        adjustedSpeedA = constrain(adjustedSpeedA, 0, 255);
        adjustedSpeedB = constrain(adjustedSpeedB, 0, 255);

        // تطبيق السرعات المعدلة
        analogWrite(EnableA, adjustedSpeedA);
        analogWrite(EnableB, adjustedSpeedB);

        // طباعة معلومات التصحيح للتصحيح والتطوير
        Serial.print("Encoder A: "); Serial.print(encoderCountA);
        Serial.print(" | Encoder B: "); Serial.print(encoderCountB);
        Serial.print(" | PID Output: "); Serial.print(output);
        Serial.print(" | Yaw: "); Serial.println(yaw);

        delay(5); // تحديث سلس
    }

    // إيقاف المحركات بعد الوصول للنقطة المطلوبة
    analogWrite(EnableA, 0);
    analogWrite(EnableB, 0);
    delay(200); // تأخير بسيط للثبات
}

void turnRight() {
    encoderCountA = 0;
    encoderCountB = 0;
    yaw = 0;
    Serial.println("Turning Right...");
    
    // هنا يمكن تعديل المنطق حسب قيم yaw المناسبة للدوران لليمين
    while (yaw < 250 || yaw > 265) {
        updateYaw();
        analogWrite(EnableA, correctionSpeed);
        analogWrite(EnableB, correctionSpeed);
        digitalWrite(motorIn1, HIGH);
        digitalWrite(motorIn2, LOW);
        digitalWrite(motorIn3, LOW);
        digitalWrite(motorIn4, HIGH);
        delay(50);
    }
    
    analogWrite(EnableA, 0);
    analogWrite(EnableB, 0);
    yaw = 0;
    Serial.println("Right Turn Complete! Yaw reset to 0");
}

void turnLeft() {
    encoderCountA = 0;
    encoderCountB = 0;
    yaw = 0;
    Serial.println("Turning Left...");
    
    while (yaw < minTurnAngleLeft || yaw > maxTurnAngleLeft) {
        updateYaw();
        analogWrite(EnableA, correctionSpeed);
        analogWrite(EnableB, correctionSpeed);
        digitalWrite(motorIn1, HIGH);
        digitalWrite(motorIn2, LOW);
        digitalWrite(motorIn3, LOW);
        digitalWrite(motorIn4, HIGH);
        delay(50);
    }
    
    analogWrite(EnableA, 0);
    analogWrite(EnableB, 0);
    yaw = 0;
    Serial.println("Left Turn Complete! Yaw reset to 0");
}

// ===========================
// دالة الإعداد (setup)
// ===========================
void setup() {
    Serial.begin(115200);
    
    // إعداد مخارج المحركات
    pinMode(EnableA, OUTPUT);
    pinMode(motorIn1, OUTPUT);
    pinMode(motorIn2, OUTPUT);
    pinMode(EnableB, OUTPUT);
    pinMode(motorIn3, OUTPUT);
    pinMode(motorIn4, OUTPUT);
    
    // إعداد مداخل الإنكودر
    pinMode(EncoderA, INPUT_PULLUP);
    pinMode(EncoderB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(EncoderA), handleEncoderA, RISING);
    attachInterrupt(digitalPinToInterrupt(EncoderB), handleEncoderB, RISING);
    
    // تهيئة MPU6050 والمعايرة
    initMPU();
    calibrateGyro();
    
    // إعداد دبابيس الحساسات (shutdown)
    pinMode(SHT_LOX1, OUTPUT);
    pinMode(SHT_LOX2, OUTPUT);
    pinMode(SHT_LOX3, OUTPUT);
    
    Serial.println("Initializing LiDAR sensors...");
    setID();
    
    Serial.println("Setup complete.");
}

// ===========================
// دالة التنفيذ الرئيسية (loop)
// ===========================
void loop() {
    // 1. تحديث وطباعة قراءات الحساسات
    int frontDistance = readFrontDistance();
    int rightDistance = readRightDistance();
    int leftDistance = readLeftDistance();

    Serial.print("Front: "); Serial.print(frontDistance);
    Serial.print(" mm, Right: "); Serial.print(rightDistance);
    Serial.print(" mm, Left: "); Serial.println(leftDistance);

    // 2. التحقق مما إذا كان هناك جدار أم لا
    bool frontWall = (frontDistance >= minDistance && frontDistance <= maxDistance);
    bool rightWall = (rightDistance >= minDistance && rightDistance <= maxDistance);
    bool leftWall  = (leftDistance >= minDistance && leftDistance <= maxDistance);

    Serial.print("Front wall: "); Serial.print(frontWall);
    Serial.print(" | Right wall: "); Serial.print(rightWall);
    Serial.print(" | Left wall: "); Serial.println(leftWall);

    // 3. اتخاذ القرار بعد اكتمال جميع القراءات
    if (!leftWall) {
        Serial.println("Decision: Turn Left (left-hand rule)");
        turnLeft();
    }
    else if (!frontWall) {
        Serial.println("Decision: Go Forward (left-hand rule)");
        moveOneCell();
    }
    else if (!rightWall) {
        Serial.println("Decision: Turn Right (left-hand rule)");
        turnRight();
    }
    else {
        Serial.println("Decision: Turn Around (left-hand rule)");
        turnRight();
        delay(100);
        turnRight();
    }

    delay(100);
}
