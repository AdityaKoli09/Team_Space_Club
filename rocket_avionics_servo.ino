#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <LittleFS.h>
#include <ESP32Servo.h>   // Library Manager: "ESP32Servo" by Kevin Harrington / John K. Bennett

// ============================================================
//                 ESP32 ROCKET AVIONICS SYSTEM
// ============================================================
//
// FEATURES
// 1. ESP32-WROOM-32
// 2. SX1278 / RA-02 LoRa
// 3. BMP280
// 4. D13 wire-cut release interrupt (logged + sent in telemetry)
// 5. D27 servo signal
// 6. Servo actuates automatically on altitude difference:
//      - ground altitude is measured at boot
//      - launch is detected when altitude gain >= LAUNCH_ALTITUDE_M
//      - after launch, the peak altitude is tracked
//      - when altitude falls APOGEE_DROP_M below the peak (confirmed
//        over several consecutive samples) the servo moves to
//        SERVO_ACTUATED_ANGLE
// 7. LoRa telemetry at 2 packets/sec
// 8. Complete internal flash logging
// 9. Serial commands: 'D' dump log, 'E' erase log,
//                     'T' test servo (moves to actuated angle),
//                     'R' return servo to safe angle
//
// WIRING NOTES
// - D13: wire loop to GND. Wire connected = LOW. Wire cut = pulled
//   HIGH by INPUT_PULLUP -> RISING interrupt.
// - D27: servo signal. Power the servo from a separate 5 V supply
//   (NOT the ESP32 3.3 V pin) and join its GND to the ESP32 GND.
// - Removed: buzzer (D32), event outputs (D27/D12 test outputs) and
//   event feedback (D34). D27 is now the servo signal pin.
// ============================================================


// ============================================================
//                     PIN CONFIGURATION
// ============================================================

// LoRa SX1278
constexpr uint8_t LORA_CS   = 5;
constexpr uint8_t LORA_RST  = 14;
constexpr uint8_t LORA_DIO0 = 26;
constexpr uint8_t LORA_SCK  = 18;
constexpr uint8_t LORA_MISO = 19;
constexpr uint8_t LORA_MOSI = 23;

// BMP280
constexpr uint8_t BMP_SDA = 21;
constexpr uint8_t BMP_SCL = 22;

// Release (wire cut) input
constexpr uint8_t RELEASE_PIN = 13;

// Servo
constexpr uint8_t SERVO_PIN = 27;

// Status LED
constexpr uint8_t STATUS_LED = 2;


// ============================================================
//                    LORA PARAMETERS
// ============================================================

constexpr long LORA_FREQUENCY = 434500000L;
constexpr long LORA_BANDWIDTH = 125000L;
constexpr uint8_t LORA_SPREADING_FACTOR = 7;
constexpr uint8_t LORA_CODING_RATE = 5;
constexpr uint16_t LORA_PREAMBLE = 8;
constexpr int8_t LORA_TX_POWER = 10;


// ============================================================
//                    TIMING PARAMETERS
// ============================================================

constexpr unsigned long TELEMETRY_PERIOD = 500UL;   // 2 packets/sec
constexpr unsigned long ALT_SAMPLE_PERIOD = 50UL;   // altitude detection at 20 Hz


// ============================================================
//              SERVO / ALTITUDE DETECTION SETTINGS
// ============================================================

constexpr int SERVO_SAFE_ANGLE = 0;         // position before deployment
constexpr int SERVO_ACTUATED_ANGLE = 90;    // position after deployment
constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2400;

constexpr float LAUNCH_ALTITUDE_M = 10.0F;  // gain above ground that arms detection
constexpr float APOGEE_DROP_M = 3.0F;       // drop below peak that triggers the servo
constexpr uint8_t DROP_CONFIRM_SAMPLES = 5; // consecutive samples (5 x 50 ms = 250 ms)

constexpr uint8_t GROUND_SAMPLES = 30;      // samples averaged for ground altitude


// ============================================================
//                    BMP280 / FLASH
// ============================================================

constexpr float SEA_LEVEL_PRESSURE = 1013.25F;

const char *FLIGHT_LOG = "/flight.csv";

const char *LOG_HEADER =
  "TIME_MS,PACKET_ID,TEMPERATURE_C,PRESSURE_HPA,ALTITUDE_M,"
  "FLIGHT_STATE,WIRE_CUT,SERVO,BMP_OK,LORA_OK,LORA_STATUS,RECORD_TYPE";


// ============================================================
//                    OBJECTS
// ============================================================

Adafruit_BMP280 bmp;
Servo deployServo;


// ============================================================
//                    SYSTEM STATUS
// ============================================================

bool bmpOK = false;
bool loraOK = false;
bool flashOK = false;


// ============================================================
//                    INTERRUPT VARIABLES
// ============================================================

volatile bool releaseInterrupt = false;
volatile unsigned long releaseTimestamp = 0;


// ============================================================
//                    SYSTEM TIMERS
// ============================================================

unsigned long bootTime = 0;
unsigned long lastTelemetryTime = 0;
unsigned long lastAltSample = 0;
unsigned long packetID = 0;


// ============================================================
//                    FLIGHT STATE
// ============================================================

enum FlightState
{
  STATE_PAD,        // on the ground, waiting for launch
  STATE_ASCENT,     // launch detected, tracking peak altitude
  STATE_DEPLOYED    // servo has been actuated
};

FlightState flightState = STATE_PAD;

bool wireCutDetected = false;
bool servoActuated = false;

float groundAltitude = 0.0F;
float maxRelAltitude = 0.0F;
uint8_t dropCount = 0;

String lastLoRaStatus = "NOT_STARTED";


// ============================================================
//                    INTERRUPT SERVICE ROUTINE
// ============================================================

void IRAM_ATTR releaseISR()
{
  releaseInterrupt = true;
  releaseTimestamp = millis();
}


// ============================================================
//                    STATE TEXT
// ============================================================

const char *flightStateText()
{
  switch (flightState)
  {
    case STATE_PAD:      return "PAD";
    case STATE_ASCENT:   return "ASCENT";
    case STATE_DEPLOYED: return "SERVO_DEPLOYED";
  }

  return "UNKNOWN";
}


// ============================================================
//                    BMP280
// ============================================================

bool initializeBMP280()
{
  Wire.begin(BMP_SDA, BMP_SCL);

  if (bmp.begin(0x76))
  {
    Serial.println("[OK] BMP280 detected at 0x76");
    return true;
  }

  if (bmp.begin(0x77))
  {
    Serial.println("[OK] BMP280 detected at 0x77");
    return true;
  }

  Serial.println("[WARNING] BMP280 NOT detected.");
  return false;
}


void configureBMP280()
{
  if (!bmpOK)
    return;

  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X16,
    Adafruit_BMP280::FILTER_X16,
    Adafruit_BMP280::STANDBY_MS_1
  );
}


float getTemperature()
{
  if (!bmpOK)
    return NAN;

  return bmp.readTemperature();
}


float getPressure()
{
  if (!bmpOK)
    return NAN;

  return bmp.readPressure() / 100.0F;
}


float getAltitude()
{
  if (!bmpOK)
    return NAN;

  return bmp.readAltitude(SEA_LEVEL_PRESSURE);
}


// Average several readings at boot to get the ground reference.
void measureGroundAltitude()
{
  if (!bmpOK)
    return;

  delay(300);  // let the BMP280 filter settle

  float sum = 0.0F;

  for (uint8_t i = 0; i < GROUND_SAMPLES; i++)
  {
    sum += getAltitude();
    delay(40);
  }

  groundAltitude = sum / GROUND_SAMPLES;

  Serial.print("[OK] Ground altitude reference = ");
  Serial.print(groundAltitude, 2);
  Serial.println(" m");
}


// ============================================================
//                    LORA INITIALIZATION
// ============================================================

bool initializeLoRa()
{
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQUENCY))
  {
    Serial.println("[WARNING] LoRa initialization FAILED.");
    lastLoRaStatus = "INIT_FAILED";
    return false;
  }

  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setPreambleLength(LORA_PREAMBLE);
  LoRa.enableCrc();
  LoRa.setTxPower(LORA_TX_POWER);

  lastLoRaStatus = "READY";

  Serial.println("[OK] LoRa initialized.");
  Serial.println("Frequency : 434.500 MHz");
  Serial.println("Bandwidth : 125 kHz");
  Serial.println("Spreading : SF7");
  Serial.println("Coding    : 4/5");
  Serial.println("Preamble  : 8");
  Serial.println("CRC       : ENABLED");
  Serial.println("TX Power  : 10 dBm");

  return true;
}


// ============================================================
//                    FLASH
// ============================================================

bool initializeFlash()
{
  // false = never auto-format; existing flight data is preserved.
  if (!LittleFS.begin(false))
  {
    Serial.println("[WARNING] LittleFS mount FAILED.");
    return false;
  }

  Serial.println("[OK] ESP32 internal flash mounted.");

  if (!LittleFS.exists(FLIGHT_LOG))
  {
    File file = LittleFS.open(FLIGHT_LOG, FILE_WRITE);

    if (!file)
    {
      Serial.println("[WARNING] Cannot create flight log.");
      return false;
    }

    file.println(LOG_HEADER);
    file.flush();
    file.close();

    Serial.println("[OK] New flight log created.");
  }
  else
  {
    Serial.println("[OK] Existing flight log found.");
    Serial.println("     (If it has the old BUZZER column, send 'E' to erase it.)");
  }

  return true;
}


bool writeFlashRecord(const String &record)
{
  if (!flashOK)
    return false;

  File file = LittleFS.open(FLIGHT_LOG, FILE_APPEND);

  if (!file)
  {
    Serial.println("[FLASH ERROR] Cannot open log.");
    return false;
  }

  file.println(record);
  file.flush();
  file.close();

  return true;
}


// ============================================================
//                    LOG SYSTEM EVENT
// ============================================================

void logSystemEvent(const char *eventName)
{
  if (!flashOK)
    return;

  String record;

  record += String(millis());
  record += ",-,-,-,-,";
  record += eventName;
  record += ",";
  record += wireCutDetected ? "1" : "0";
  record += ",";
  record += servoActuated ? "1" : "0";
  record += ",";
  record += bmpOK ? "1" : "0";
  record += ",";
  record += loraOK ? "1" : "0";
  record += ",";
  record += lastLoRaStatus;
  record += ",EVENT";

  writeFlashRecord(record);
}


// ============================================================
//                    TELEMETRY RECORD / PACKET
// ============================================================

String createFlashTelemetryRecord()
{
  String record;

  record += String(millis());
  record += ",";
  record += String(packetID);
  record += ",";
  record += String(getTemperature(), 2);
  record += ",";
  record += String(getPressure(), 2);
  record += ",";
  record += String(getAltitude(), 2);
  record += ",";
  record += flightStateText();
  record += ",";
  record += wireCutDetected ? "1" : "0";
  record += ",";
  record += servoActuated ? "1" : "0";
  record += ",";
  record += bmpOK ? "1" : "0";
  record += ",";
  record += loraOK ? "1" : "0";
  record += ",";
  record += lastLoRaStatus;
  record += ",TELEMETRY";

  return record;
}


String createTelemetryPacket()
{
  float temperature = getTemperature();
  float pressure = getPressure();
  float altitude = getAltitude();

  unsigned long elapsed = millis() - bootTime;

  packetID++;

  String packet;
  packet.reserve(160);

  packet += "PKT=1";
  packet += ",ID=";
  packet += String(packetID);
  packet += ",T=";
  packet += String(temperature, 2);
  packet += ",P=";
  packet += String(pressure, 2);
  packet += ",ALT=";
  packet += String(altitude, 2);
  packet += ",TIME=";
  packet += String(elapsed);
  packet += ",SRV=";
  packet += servoActuated ? "1" : "0";  // 0 = safe, 1 = actuated

  return packet;
}


void sendTelemetry()
{
  String packet = createTelemetryPacket();

  Serial.println();
  Serial.println("[TELEMETRY]");
  Serial.println(packet);

  if (loraOK)
  {
    LoRa.beginPacket();
    LoRa.print(packet);
    int result = LoRa.endPacket();

    if (result == 1)
    {
      lastLoRaStatus = "TX_OK";
      Serial.println("LoRa: TRANSMITTED");
    }
    else
    {
      lastLoRaStatus = "TX_ERROR";
      Serial.println("LoRa: TRANSMISSION ERROR");
    }
  }
  else
  {
    lastLoRaStatus = "NOT_AVAILABLE";
    Serial.println("LoRa: NOT AVAILABLE");
  }

  if (flashOK)
  {
    if (!writeFlashRecord(createFlashTelemetryRecord()))
      Serial.println("[WARNING] Telemetry flash write failed.");
  }
}


// ============================================================
//                    SERVO
// ============================================================

void initializeServo()
{
  deployServo.setPeriodHertz(50);
  deployServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  deployServo.write(SERVO_SAFE_ANGLE);

  Serial.println("[OK] Servo attached, at safe angle.");
}


void actuateServo()
{
  deployServo.write(SERVO_ACTUATED_ANGLE);

  servoActuated = true;
  flightState = STATE_DEPLOYED;
  digitalWrite(STATUS_LED, HIGH);

  Serial.println("SERVO: ACTUATED");
  logSystemEvent("SERVO_ACTUATED");
}


// ============================================================
//                    ALTITUDE-DIFFERENCE DETECTION
// ============================================================

void processAltitudeDetection()
{
  if (!bmpOK || flightState == STATE_DEPLOYED)
    return;

  unsigned long now = millis();

  if (now - lastAltSample < ALT_SAMPLE_PERIOD)
    return;

  lastAltSample = now;

  float rel = getAltitude() - groundAltitude;

  // ---------------- PAD -> ASCENT ----------------
  if (flightState == STATE_PAD)
  {
    if (rel >= LAUNCH_ALTITUDE_M)
    {
      flightState = STATE_ASCENT;
      maxRelAltitude = rel;
      dropCount = 0;

      Serial.println("LAUNCH DETECTED - altitude detection armed");
      logSystemEvent("LAUNCH_DETECTED");
    }

    return;
  }

  // ---------------- ASCENT: track peak, look for drop ----------------
  if (rel > maxRelAltitude)
    maxRelAltitude = rel;

  if ((maxRelAltitude - rel) >= APOGEE_DROP_M)
    dropCount++;
  else
    dropCount = 0;

  if (dropCount >= DROP_CONFIRM_SAMPLES)
  {
    Serial.print("ALTITUDE DROP DETECTED: peak ");
    Serial.print(maxRelAltitude, 2);
    Serial.print(" m, now ");
    Serial.print(rel, 2);
    Serial.println(" m");

    logSystemEvent("ALTITUDE_DROP_DETECTED");
    actuateServo();
  }
}


// ============================================================
//                    RELEASE INTERRUPT PROCESSING
// ============================================================

void processReleaseInterrupt()
{
  if (!releaseInterrupt)
    return;

  noInterrupts();
  releaseInterrupt = false;
  unsigned long detectedTime = releaseTimestamp;
  interrupts();

  if (wireCutDetected)
    return;

  wireCutDetected = true;

  Serial.println();
  Serial.println("[INTERRUPT] WIRE CUT DETECTED");

  Serial.print("[INTERRUPT] Time = ");
  Serial.print(detectedTime);
  Serial.println(" ms");

  logSystemEvent("WIRE_CUT_DETECTED");
}


// ============================================================
//                    DUMP / ERASE FLASH DATA
// ============================================================

void dumpFlightLog()
{
  if (!flashOK)
  {
    Serial.println("[ERROR] Flash unavailable.");
    return;
  }

  File file = LittleFS.open(FLIGHT_LOG, FILE_READ);

  if (!file)
  {
    Serial.println("[ERROR] Cannot open flight log.");
    return;
  }

  Serial.println();
  Serial.println("========================================");
  Serial.println("          STORED FLIGHT DATA");
  Serial.println("========================================");

  while (file.available())
  {
    Serial.write(file.read());
  }

  file.close();

  Serial.println();
  Serial.println("========================================");
  Serial.println("          END OF FLIGHT DATA");
  Serial.println("========================================");
}


void eraseFlightLog()
{
  if (!flashOK)
    return;

  LittleFS.remove(FLIGHT_LOG);

  File file = LittleFS.open(FLIGHT_LOG, FILE_WRITE);

  if (file)
  {
    file.println(LOG_HEADER);
    file.close();
  }

  Serial.println("[OK] Flight log erased.");
}


// Serial commands: 'D' dump, 'E' erase, 'T' test servo, 'R' servo safe
void processSerialCommands()
{
  while (Serial.available())
  {
    char c = Serial.read();

    if (c == 'D' || c == 'd')
    {
      dumpFlightLog();
    }
    else if (c == 'E' || c == 'e')
    {
      eraseFlightLog();
    }
    else if (c == 'T' || c == 't')
    {
      Serial.println("SERVO TEST: moving to actuated angle");
      deployServo.write(SERVO_ACTUATED_ANGLE);
      logSystemEvent("SERVO_TEST");
    }
    else if (c == 'R' || c == 'r')
    {
      Serial.println("SERVO: returning to safe angle");
      deployServo.write(SERVO_SAFE_ANGLE);
      logSystemEvent("SERVO_RESET");
    }
  }
}


// ============================================================
//                    SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("========================================");
  Serial.println("      ESP32 ROCKET AVIONICS BOOT");
  Serial.println("========================================");

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  pinMode(RELEASE_PIN, INPUT_PULLUP);  // wire to GND; cut = HIGH

  bootTime = millis();

  initializeServo();

  bmpOK = initializeBMP280();
  configureBMP280();
  measureGroundAltitude();

  loraOK = initializeLoRa();

  flashOK = initializeFlash();

  logSystemEvent("BOOT");

  // Warn if the wire is already cut at boot
  if (digitalRead(RELEASE_PIN) == HIGH)
  {
    Serial.println("[WARNING] Release wire reads CUT/OPEN at boot!");
  }

  attachInterrupt(
    digitalPinToInterrupt(RELEASE_PIN),
    releaseISR,
    RISING
  );

  Serial.println("[READY] 'D' dump log, 'E' erase log, 'T' test servo, 'R' reset servo.");
}


// ============================================================
//                    LOOP
// ============================================================

void loop()
{
  processReleaseInterrupt();
  processAltitudeDetection();
  processSerialCommands();

  unsigned long now = millis();

  if (now - lastTelemetryTime >= TELEMETRY_PERIOD)
  {
    lastTelemetryTime = now;
    sendTelemetry();
  }
}
