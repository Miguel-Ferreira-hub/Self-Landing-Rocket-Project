// ================= LIBRARY IMPORTS ================
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

// ================= ESP8266 WIFI ================
#define ESP Serial3
String laptopIP = "192.168.1.101";
int laptopPort = 5001;
bool wifiOK = false;
bool tcpOK  = false;

// ================= SENSORS AND PINS ================
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // Gyro library
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085); // Barometer library
Servo para1, para2;
Servo tvc1, tvc2;
File dataFile;
const int chipSelect = 53, buzzer = 12; // SD card and buzzer pins
const int redPin = 11, greenPin = 10; // LED pins
const int yawPin = 6, pitchPin = 5; // TVC servo pins
const int relayPin = 7, landingPin = 4; // Relay pins

int para_neutral = 100; // Servo neutral position for parachute mechanism servos
float tvc_neutral_position = 1480; // Servo neutral position for TVC mount (PWM)

float altitude_filtered, vel_filtered, yaw_rate_filtered, pitch_rate_filtered; // Sensor data
float altitude, last_altitude, vel;
float input_yaw, input_pitch, output_yaw, output_pitch;
float reference_pressure = 0;
unsigned long last_time = 0;

// ================= STATE DETECTION VARIABLES ================
const int N = 50, N_alt = 10, N_vel = 20;
float values[N], values_alt[N_alt], values_vel[N_vel];
int count = 0, count_vel = 0, count_alt = 0;
float diff_vel, diff_alt;
const unsigned long launchDuration = 1500, burnDuration = 4000; // 1.5 seconds
unsigned long launchStartTime = 0, landingStartTime = 0;
bool launchActive = false, landingActive = false;

// ================= QUATERNION STRUCTURE =========
struct Q { 
  float w, x, y, z; 
};

// ================= KALMAN FILTER ==============
struct Kalman { // Increasing O and decreasing R decreases lag
  float x;   // state estimate
  float P;   // estimate covariance
  float O;   // process noise
  float R;   // measurement noise
};

// ================= PID CONTROLLER STRUCTURE ==============
struct PID {
  float error, last_error, derivative, output, Kp, Kd;
};

// ================= IMU SENSOR DATA =============
struct IMUData {
  float yaw;
  float pitch;
  float roll;
  float yaw_rate;
  float pitch_rate;
  float roll_rate;
  float ax;
  float ay;
  float az;
  float gx = 0.1738f;
  float gy = -0.0454f;
  float gz = -9.4663f;
};

// ================= FLIGHT STATE ================
enum FlightPhase {
  PRELAUNCH,
  LAUNCH,
  BURNOUT,
  APOGEE,
  DESCENT,
  LANDED,
};

FlightPhase flightPhase = PRELAUNCH;

// ================= QUATERNION FUNCTIONS ================
Q toQ(imu::Quaternion q) {
    Q out;
    out.w = q.w();
    out.x = q.x();
    out.y = q.y();
    out.z = q.z();
    return out;
}

Q qmul(const Q &a, const Q &b) {
    Q out;
    out.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    out.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    out.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    out.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    return out;
}

void rotateVectorByQuat(const Q &q, float bx, float by, float bz, float &wx, float &wy, float &wz) {
    Q v{0, bx, by, bz};
    Q qc{q.w, -q.x, -q.y, -q.z};
    Q t = qmul(q, v);
    Q r = qmul(t, qc);
    wx = r.x; 
    wy = r.y; 
    wz = r.z;
}

void quatToEuler(const Q &q, float &roll, float &pitch, float &yaw) {
    // roll (x-axis rotation)
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    roll = atan2(sinr_cosp, cosr_cosp);
    // pitch (y-axis rotation)
    float sinp = 2 * (q.w * q.y - q.z * q.x);
    if (abs(sinp) >= 1)
        pitch = copysign(PI / 2, sinp);
    else
        pitch = asin(sinp);
    // yaw (z-axis rotation)
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    yaw = atan2(siny_cosp, cosy_cosp);
    // Convert to degrees
    roll *= 180.0 / PI;
    pitch *= 180.0 / PI;
    yaw *= 180.0 / PI;
}

Q q_ref;

// ================= KALMAN FILTER FUNCTIONS ================
void kalmanInit(Kalman &kf, float initial_value, float initial_P, float q_noise, float r_noise) {
  kf.x = initial_value;
  kf.P = initial_P;
  kf.O = q_noise;
  kf.R = r_noise;
}

float kalmanUpdate(Kalman &kf, float measurement) {
  // Prediction
  kf.P += kf.O;
  // Update
  float K = kf.P / (kf.P + kf.R);
  kf.x += K * (measurement - kf.x);
  kf.P *= (1 - K);
  return kf.x;
}

Kalman altFilter;
Kalman velFilter;
Kalman yaw_rateFilter;
Kalman pitch_rateFilter;

// ================= PID CONTROLLER FUNCTIONS ================
void PIDInit(PID &p, float Kp, float Kd) {
  p.error = 0;
  p.last_error = 0;
  p.derivative = 0;
  p.output = 0;
  p.Kp = Kp;
  p.Kd = Kd;
}

float PIDupdate(PID &p, float yaw, float yaw_rate) {
  p.error = - yaw;
  p.derivative = - yaw_rate;
  p.output = p.Kp * p.error + p.Kd * p.derivative;
  p.last_error = p.error;
  return p.output;
}

PID controller_yaw;
PID controller_pitch;

// ================= SENSOR STRUCTURE ================
IMUData readIMU() {
  IMUData data;
  imu::Vector<3> accelBody = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Quaternion iq = bno.getQuat();
  Q q_raw = toQ(iq);
  Q q_ref_inv = { q_ref.w, -q_ref.x, -q_ref.y, -q_ref.z };
  Q q = qmul(q_ref_inv, q_raw);
  quatToEuler(q, data.yaw, data.pitch, data.roll);
  rotateVectorByQuat(q,
                     accelBody.x(), accelBody.y(), accelBody.z(),
                     data.ax, data.ay, data.az);
  imu::Vector<3> gyroVec = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  data.ax += data.gx;
  data.ay += data.gy;
  data.az += data.gz;
  data.yaw_rate   = gyroVec.x() * 180.0 / PI;
  data.pitch_rate = gyroVec.y() * 180.0 / PI;
  data.roll_rate  = gyroVec.z() * 180.0 / PI;
  return data;
}

float angle_conversion(float rocket_angle) {
  // Quadratic fit - Degrees
  float servo_angle = (2.16*pow(PI, 2) / pow(180, 2)) * pow(rocket_angle,2) 
                    + (2.12*PI/180) * rocket_angle;
  return servo_angle;
}

// ================= ESP8266 FUNCTIONS ================
String readESP(unsigned long timeout) {
  String resp = "";
  unsigned long start = millis();
  while (millis() - start < timeout) {
    while (ESP.available()) {
      char c = ESP.read();
      resp += c;
    }
  }
  Serial.println(resp);   // prints everything to monitor
  return resp;
}

bool sendCommand(String cmd, unsigned long timeout) {
  ESP.println(cmd);
  String resp = readESP(timeout);
  if (resp.indexOf("OK") != -1) return true;
  if (resp.indexOf("ERROR") != -1) return false;
  return false;
}

void connectWiFi() { // Connect wifi
  wifiOK = false;
  Serial.println("Connecting WiFi...");
  ESP.println("AT+CWJAP=\"Rocket\",\"gdp23wifi\"");
  String resp = readESP(10000);
  if (resp.indexOf("WIFI GOT IP") != -1) {
    Serial.println("WiFi OK");
    wifiOK = true;
  } else {
    Serial.println("WiFi FAIL");
  }
}

void scanWiFi() {
  Serial.println("Scanning WiFi networks...");
  // Send scan command to ESP
  ESP.println("AT+CWLAP");
  unsigned long start = millis();
  String line = "";
  // Wait up to 5 seconds for ESP response
  while (millis() - start < 5000) {
    while (ESP.available()) {
      char c = ESP.read();   // read one byte from the ESP
      line += c;
      if (c == '\n') {       // a full line has arrived
        line.trim();         // remove leading/trailing spaces
        if (line.length() > 0) {
          Serial.println(line); // print to serial monitor
        }
        line = "";           // reset line buffer
      }
    }
  }
  Serial.println("WiFi scan complete.");
}

void connectTCP() { // Connect tcp to laptop
  tcpOK = false;
  Serial.println("Opening TCP...");
  ESP.print("AT+CIPSTART=\"TCP\",\"");
  ESP.print(laptopIP);
  ESP.print("\",");
  ESP.println(laptopPort);
  unsigned long start = millis();
  while (millis() - start < 5000) {
    if (ESP.find("CONNECT")) {
      Serial.println("TCP OK");
      tcpOK = true;
      return;
    }
    if (ESP.find("ERROR")) break;
  }
  Serial.println("TCP FAIL");
}

void sendTCPfast(String data) { // Send information to laptop
  //if (!wifiOK) connectWiFi();
  //if (!tcpOK) connectTCP();
  ESP.print("AT+CIPSEND=");
  ESP.println(data.length());
  unsigned long start = millis();
  bool gotPrompt = false;
  while (millis() - start < 120) {
    while (ESP.available()) {
      if (ESP.read() == '>') {
        gotPrompt = true;
        break;
      }
    }
    if (gotPrompt) break;
  }
  if (!gotPrompt) {
    Serial.println("No prompt → reconnecting TCP...");
    tcpOK = false;
    return;
  }
  ESP.print(data);
}

// ================= SETUP FUNCTION ================
void setup() {
  pinMode(buzzer, OUTPUT); // Set buzzer pin
  pinMode(redPin, OUTPUT); // Set red LED pin
  digitalWrite(redPin, HIGH); // Turn on red LED
  pinMode(greenPin, OUTPUT); // Set green LED pin
  pinMode(relayPin, OUTPUT); // Set relay pin
  pinMode(landingPin, OUTPUT);
  digitalWrite(relayPin, HIGH); // Deactivate relay pin (active low)
  digitalWrite(landingPin, HIGH);
  para1.attach(13); // Set parachute servo 1 pin
  para2.attach(8); // set parachute servo 2 pin - this one goes negative e.g. 80 (blue)
  para1.write(para_neutral); // Set parachute servo 1 to 0 degrees
  para2.write(para_neutral); // Set parachute servo 2 to 0 degrees
  tvc1.attach(pitchPin); // Yaw
  tvc2.attach(yawPin); // Pitch
  tvc1.writeMicroseconds(tvc_neutral_position); // TVC neutral position
  tvc2.writeMicroseconds(tvc_neutral_position);
  tone(buzzer, 1000); // Buzzer sequence
  delay(100);
  noTone(buzzer);
  delay(100);
  tone(buzzer, 500);
  delay(100);
  noTone(buzzer);
  delay(100);
  tone(buzzer, 100);
  delay(100);
  noTone(buzzer);
  Serial.begin(115200); // Begin serial connection
  delay(1000);
  ESP.begin(115200); // Begin ESP serial connection (serial3)
  Serial.println("Configuring ESP..."); // Setup wifi
  sendCommand("AT+RST", 2000);
  readESP(5000);
  delay(500);
  sendCommand("AT+CWMODE=1", 500);
  delay(500);
  sendCommand("AT+CWAUTOCONN=0", 500);
  delay(500);
  sendCommand("AT+CWDHCP=1,1", 500);
  delay(500);
  sendCommand("AT+CWQAP", 500);
  delay(500);
  sendCommand("AT+CWLAP", 500);
  delay(5000);
  scanWiFi();
  delay(5000);
  connectWiFi();
  connectTCP();
  // ---------------- BNO055 INIT ----------------
  if (!bno.begin()) {
    Serial.println("BNO055 not detected (Gyro)");
    sendTCPfast("BNO055 not detected (Gyro)");
    while (1);
  }
  
  bno.setExtCrystalUse(true);

  delay(2000); // allow fusion to settle

  imu::Quaternion iq_ref = bno.getQuat();
  q_ref = toQ(iq_ref);

  // ---------------- BMP180 INIT ----------------
  if (!bmp.begin()) {
    Serial.println("BMP180 not detected (Barometer)");
    sendTCPfast("BMP180 not detected (Barometer)");
    while (1);
  }

  // ---------------- SD CARD INIT ----------------
  pinMode(53, OUTPUT); // Set SD card pin
  digitalWrite(53, HIGH); // Set SD card pin to high always
  Serial.println("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("SD init failed");
    sendTCPfast("SD init failed");
    while (1);
  }
  Serial.println("SD ready");

  bool newFile = !SD.exists("Flight.csv");   // Check if file exists

  dataFile = SD.open("Flight.csv", FILE_WRITE); // Open file or create if none exists
  if (!dataFile) { // If no data file exists
    Serial.println("Unable to open file"); 
    sendTCPfast("Unable to open file");
    while(1);
  }

  if (newFile) {
    dataFile.println("Time (ms),Roll (deg),Pitch (deg),Yaw (deg),Roll Rate (deg/s),Pitch Rate (deg/s),"
    "Yaw Rate (deg/s),x_acceleration (ms-2),y_acceleration (ms-2),z_acceleration (ms-2),Velocity (ms-1),Altitude (m),"
    "Temperature (C),State Detection,Pitch Response,Yaw Response,Relay State");
    dataFile.flush();
  }

  Serial.println("Logging...");
  delay(1000);
  sensors_event_t event;
  bmp.getEvent(&event);
  reference_pressure = event.pressure; // Set reference pressure as initial pressure experienced
  Serial.println("All Systems Ready");
  sendTCPfast("All Systems Ready");
  tone(buzzer, 1000);
  delay(100);
  noTone(buzzer);
  delay(100);
  tone(buzzer, 1500);
  delay(100);
  noTone(buzzer);
  last_time = millis();
  kalmanInit(altFilter, 0.0, 1.0, 0.1, 4);
  kalmanInit(velFilter, 0.0, 1.0, 0.1, 4);
  kalmanInit(yaw_rateFilter, 0.0, 1.0, 50, 1000);
  kalmanInit(pitch_rateFilter, 0.0, 1.0, 50, 1000);
  PIDInit(controller_yaw, 0.9, 0.01);
  PIDInit(controller_pitch, 0.9, 0.01);
  // PIDInit(controller_pitch, 0.5, 0.09);
  // PIDInit(controller_yaw, 0.5, 0.09);
  // original was Kp = 0.5, Kd = 0.09
  digitalWrite(redPin, LOW); // Turn red LED off
  digitalWrite(greenPin, HIGH); // Turn on green LED
}

// ================= LOOP FUNCTION ================
void loop() {
  String Communication; // Variable for state communication
  String update;
  unsigned long now = millis(); // 
  float dt = (now - last_time) * 1e-3f; // Change in time
  if (dt <= 0) return;
  last_time = now;

  IMUData imuData = readIMU();

  sensors_event_t event;
  bmp.getEvent(&event);
  
  float temperature; // Assign variable for temperature reading from bmp180
  bmp.getTemperature(&temperature); // Read temperature using bmp180

  if (event.pressure) { // Pressure
    float pressure = event.pressure; // Read pressure
    float seaLevelPressure = reference_pressure; // Assign reference pressure
    altitude = bmp.pressureToAltitude(seaLevelPressure, pressure); // Calculate altitude
  }

  // Settling time
  if (now < 2000) {
    altitude = 0;
    vel = 0;
  }

  // ================= FILTERING ================
  altitude_filtered = kalmanUpdate(altFilter, altitude);
  vel = (altitude_filtered - last_altitude) / dt;
  vel_filtered = kalmanUpdate(velFilter, vel);
  yaw_rate_filtered = kalmanUpdate(yaw_rateFilter, imuData.yaw_rate);
  pitch_rate_filtered = kalmanUpdate(pitch_rateFilter, imuData.pitch_rate);

  // ================= PID CONTROLLER ================
  output_yaw = PIDupdate(controller_yaw, imuData.yaw, yaw_rate_filtered);
  output_pitch = PIDupdate(controller_pitch, imuData.pitch, pitch_rate_filtered);
  input_yaw = angle_conversion(output_yaw);
  input_pitch = angle_conversion(output_pitch);
  input_yaw = constrain(output_yaw, -26, 26); // Constrains within TVC mount limits 26
  input_pitch = constrain(output_pitch, -50, 20); // Constrains within TVC mount limits
  float microseconds_yaw = tvc_neutral_position - (input_yaw * 10); //Approximately 1 degree for 10 microseconds
  float microseconds_pitch = tvc_neutral_position - (input_pitch * 10); //Approximately 1 degree for 10 microseconds
  tvc1.writeMicroseconds(microseconds_pitch);
  tvc2.writeMicroseconds(microseconds_yaw);

  // ================= INTERPRET COMMANDS FROM LAPTOP ================
  if (ESP.available()) {
    String raw = ESP.readStringUntil('\n'); // Read command until char on next line
    if (raw.length() > 0) {
        raw.trim(); // Trim string
        int idx = raw.indexOf(':');
        if (idx != -1) {
          String cmd = raw.substring(idx + 1);
          cmd.trim();
        if (cmd == "Launch" && flightPhase == PRELAUNCH) { // Ignite motor
          digitalWrite(relayPin, LOW);
          sendTCPfast("Motor Ignited"); // Send message to laptop
          launchStartTime = millis();
          launchActive = true;
        }
        if (cmd == "P") {
          sendTCPfast("Parachute Deployed"); // Send message to laptop
          para1.write(para_neutral - 45); // Activate parachute
          para2.write(para_neutral - 45);
        }
        if (cmd == "L") {
          sendTCPfast("Landing Legs Deployed");
          digitalWrite(landingPin, LOW);   // Active low relay ON
          landingStartTime = millis();
          landingActive = true;
        }
        // Testing Purposes - Parachute Mechanism
        if (cmd == "Reset") {
          sendTCPfast("Servo Reset"); // Send message to laptop
          para1.write(para_neutral); // Activate parachute
          para2.write(para_neutral);
        }
        // Testing purposes - Parachute mechanism with landing legs
        if (cmd == "Apogee") {
          digitalWrite(landingPin, LOW);
          delay(500);
          update = "Apogee Activated";
          sendTCPfast(update);
          para1.write(para_neutral - 45); // Activate parachute
          para2.write(para_neutral - 45);
          landingStartTime = millis();
          landingActive = true;
        }
        cmd.trim();  // remove newline and spaces
        // Check if command is numeric
        bool isNumber = true;
        for (unsigned int i = 0; i < cmd.length(); i++) {
          if (!isDigit(cmd[i])) {
            isNumber = false;
          break;
          }
        }
        if (isNumber) {
          int angle = cmd.toInt();
          para1.write(angle);
          para2.write(angle);
          tvc1.writeMicroseconds(angle);
          tvc2.writeMicroseconds(angle);
          sendTCPfast("Servo set to: " + String(angle));
        }
      }
    }
  }
  // ================= STATE DETECTION ================
  // Launch
  static unsigned long launch_time = 0; // Time variable for detection
  if (imuData.az > 3 && flightPhase == PRELAUNCH) { // Threshold for detection (20)
    if (launch_time == 0) launch_time = millis(); // Record first detection
    if (millis() - launch_time > 150) { // Difference between system start and first detection
      Communication = "Launch Detected"; // Detect launch if counter exceeds 300ms
      sendTCPfast(Communication); // Send communication to laptop
      tone(buzzer, 500, 100); // Activate buzzer
      flightPhase = LAUNCH;
      digitalWrite(relayPin, HIGH); // Deactivate motor igniter
    }
  } else { 
    launch_time = 0;
  }
  // Burnout
  static unsigned long burnout_time = 0; // Time variable for detection
  if (imuData.az < 3 && flightPhase == LAUNCH) { // Threshold and conditions for detection (20)
    if (burnout_time == 0) burnout_time = millis(); // Record first detection
    if (millis() - burnout_time > 300) { // Difference between system start and first detection
      Communication = "Burnout Detected"; // Detect burnout if counter exceeds 300ms
      sendTCPfast(Communication); // Send communication to laptop
      flightPhase = BURNOUT;
      tone(buzzer, 1000, 100); // Activate buzzer
    } 
  } else {
    burnout_time = 0;
  }
  // Apogee
  if (flightPhase == BURNOUT) {
    values_alt[count_alt] = altitude_filtered;
    count_alt++;
    if (count_alt >= N_alt) {
      count_alt = 0;  // wrap around
    }
    static bool buffer_full = false;
    if (count_alt == 0) {
      buffer_full = true;
    }
    if (buffer_full) {
      int oldest_index = count_alt;
      int newest_index = (count_alt - 1 + N_alt) % N_alt;
      diff_alt = values_alt[newest_index] - values_alt[oldest_index];
      if (diff_alt < -0.50) {
        digitalWrite(landingPin, LOW);
        flightPhase = APOGEE;
        landingStartTime = millis();
        landingActive = true;
        update = "Nichrome Wire Heating ON";
        sendTCPfast(update);
        delay(500);
        para1.write(para_neutral - 45); // Activate parachute
        para2.write(para_neutral - 45);
        Communication = "Apogee Detected";
        delay(300);
        sendTCPfast(Communication);
        tone(buzzer, 1500, 100);
      }
    }
  }
  // Descent
  if (flightPhase == APOGEE) {
    static bool buffer_full = false;
    // Store new velocity sample
    values_vel[count_vel] = vel_filtered;
    count_vel++;
    // Wrap around
    if (count_vel >= N_vel) {
      count_vel = 0;
      buffer_full = true;
    }
    if (buffer_full) {
      int oldest_index = count_vel;  // next write position = oldest
      int newest_index = (count_vel - 1 + N_vel) % N_vel;
      diff_vel = values_vel[newest_index] - values_vel[oldest_index];
      // Add threshold to avoid noise triggers
      if (diff_vel < 0) {   // adjust threshold!
        Communication = "Descent Detected";
        sendTCPfast(Communication);
        flightPhase = DESCENT;
        tone(buzzer, 1500, 100);
      }
    }
  }
  // Landing
  if (flightPhase == DESCENT) { // Condition for detection
    values[count] = altitude_filtered; // Add samples to array
    count++; // increase count for each sample
    if (count >= N) { // Condition for computing standard deviation
      float mean = 0; // mean
      for (int i=0; i < N; i++) mean += values[i]; // Iterate through samples
      mean /= N; // Calculate mean
      float sumSqDiff = 0; // Sum of squares
      for (int i = 0; i < N; i++) { // Iterate through samples
      sumSqDiff += (values[i] - mean) * (values[i] - mean); // Calculate sum of squares
    }
    float stdDev = sqrt(sumSqDiff / N); // Standard deviation of samples
    if (stdDev < 0.50) { // Condition for rocket having landed
      Communication = "Landing Detected"; // Detect landing
      sendTCPfast(Communication); // Send communication to laptop
      flightPhase = LANDED;
      tone(buzzer, 500, 100); // Activate buzzer
      tone(buzzer, 1000, 100); // Activate buzzer
    }
      count = 0; // Reset count
    }
  }
  if (landingActive && millis() - landingStartTime >= burnDuration) {
    digitalWrite(landingPin, HIGH);  // Turn OFF
    update = "Nichrome Wire Heating OFF";
    sendTCPfast(update);
    landingActive = false;
  }
  if (launchActive && millis() - launchStartTime >= launchDuration) {
    digitalWrite(relayPin, HIGH);  // Turn OFF
    update = "Igniter Deactivated";
    sendTCPfast(update);
    launchActive = false;
  }
  // ================= WRITING DATA ================
  String row = // CSV row
    String(now) + "," + String(imuData.roll) + "," + String(imuData.pitch) + "," +
    String(imuData.yaw) + "," + String(imuData.roll_rate) + "," + String(imuData.pitch_rate) + "," + 
    String(imuData.yaw_rate) + "," + String(imuData.ax) + "," + String(imuData.ay) + "," +
    String(imuData.az) + "," + String(vel_filtered) + "," + String(altitude_filtered) + "," + 
    String(temperature) + "," + Communication + "," + String(tvc1.read()) + "," + 
    String(tvc2.read()) + "," + update;
  dataFile.println(row); // Apend row
  dataFile.flush(); // Write row
  
  // Serial monitor prints
  // Serial.print(vel);
  // Serial.print(" ");
  // Serial.println(vel_filtered);
  //Serial.println("PID Angle: " + String(input_yaw));
  Serial.println(imuData.az);
  last_altitude = altitude_filtered;
}