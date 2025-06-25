/*
 * RPMControl.ino
 * 
 * Advanced example demonstrating closed-loop RPM control
 * using the EMC230x library with PID control.
 * 
 * This example shows how to:
 * - Configure the EMC230x for RPM-based control
 * - Set target RPM values
 * - Configure PID gains for optimal control
 * - Monitor RPM tracking performance
 * - Handle spin-up configuration
 * 
 * Hardware required:
 * - EMC230x fan controller (any variant)
 * - 4-wire PWM fans with tachometer feedback
 * - I2C connections to Arduino
 * 
 * Note: 3-wire fans may work but will not provide RPM feedback
 */

#include <EMC230x.h>

// Create EMC230x instance
EMC230x fanController(0x2F);

// Target RPM values for cycling
const uint16_t targetRPMs[] = {800, 1200, 1600, 2000, 2400, 1000};
const int numTargets = sizeof(targetRPMs) / sizeof(targetRPMs[0]);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("EMC230x RPM Control Example");
  Serial.println("============================");
  
  // Initialize the fan controller
  if (!fanController.begin()) {
    Serial.println("ERROR: Failed to initialize EMC230x!");
    while (1) delay(1000);
  }
  
  Serial.println("EMC230x initialized successfully!");
  
  // Display device information
  EMC230xDevice device = fanController.detectDevice();
  Serial.print("Detected device: EMC230");
  Serial.println(device);
  Serial.print("Max fans: ");
  Serial.println(fanController.getMaxFans());
  
  // Configure Fan 1 for RPM control
  Serial.println("\nConfiguring Fan 1 for RPM control...");
  
  // Clear any existing faults
  fanController.clearFaults();
  
  // Configure fan parameters
  FanConfig config;
  config.enableClosedLoop = true;        // Enable RPM control
  config.minRPM = 1;                     // 1000 RPM minimum range
  config.edges = 1;                      // 5 edges (typical for 2-pole fans)
  config.updateTime = 3;                 // 800ms update time
  config.enableRampRate = true;          // Enable smooth transitions
  config.enableGlitchFilter = true;      // Filter tach noise
  config.errorWindow = 2;                // ±100 RPM error window
  
  if (!fanController.setFanConfig(1, config)) {
    Serial.println("ERROR: Failed to configure fan!");
    while (1) delay(1000);
  }
  
  // Configure PID gains for stable control
  PIDGains pidGains;
  pidGains.kp = 1;                       // Proportional gain: 2x
  pidGains.ki = 1;                       // Integral gain: 2x  
  pidGains.kd = 0;                       // Derivative gain: 1x
  pidGains.derivativeOption = 1;         // Basic derivative
  
  if (!fanController.setPIDGains(1, pidGains)) {
    Serial.println("ERROR: Failed to set PID gains!");
    while (1) delay(1000);
  }
  
  // Configure spin-up behavior
  SpinUpConfig spinConfig;
  spinConfig.spinTime = 2;               // 1 second spin-up
  spinConfig.spinLevel = 3;              // 45% spin-up level
  spinConfig.noKick = false;             // Allow initial 100% kick
  spinConfig.driveFailCount = 2;         // 32 update periods before drive fail
  
  if (!fanController.setSpinUpConfig(1, spinConfig)) {
    Serial.println("ERROR: Failed to set spin-up config!");
    while (1) delay(1000);
  }
  
  // Set minimum drive level to prevent stalling
  fanController.setMinimumDrive(1, 30);  // 30/255 minimum PWM
  
  // Enable RPM control mode
  if (!fanController.enableRPMControl(1, true)) {
    Serial.println("ERROR: Failed to enable RPM control!");
    while (1) delay(1000);
  }
  
  Serial.println("Fan configured successfully!");
  Serial.println("\nStarting RPM control sequence...");
  Serial.println("Target RPM will change every 10 seconds");
  Serial.println("Format: Target | Actual | Error | PWM | Status");
  Serial.println("----------------------------------------------");
}

void loop() {
  // Cycle through different target RPMs
  static int rpmIndex = 0;
  static unsigned long lastRPMChange = 0;
  
  if (millis() - lastRPMChange >= 10000) { // Change target every 10 seconds
    lastRPMChange = millis();
    
    uint16_t targetRPM = targetRPMs[rpmIndex];
    Serial.print("\n>>> Setting target RPM to: ");
    Serial.println(targetRPM);
    
    if (!fanController.setTargetRPM(1, targetRPM)) {
      Serial.println("ERROR: Failed to set target RPM!");
    }
    
    rpmIndex = (rpmIndex + 1) % numTargets;
  }
  
  // Display status every second
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus >= 1000) {
    lastStatus = millis();
    
    // Read current values
    uint16_t targetRPM = fanController.getTargetRPM(1);
    uint16_t actualRPM = fanController.getActualRPM(1);
    uint8_t pwmValue = fanController.getFanSpeed(1);
    float pwmPercent = fanController.getFanSpeedPercent(1);
    
    // Calculate error
    int16_t error = actualRPM - targetRPM;
    
    // Display formatted status
    Serial.print(targetRPM);
    Serial.print(" | ");
    Serial.print(actualRPM);
    Serial.print(" | ");
    if (error >= 0) Serial.print("+");
    Serial.print(error);
    Serial.print(" | ");
    Serial.print(pwmPercent, 1);
    Serial.print("% ");
    
    // Status indicators
    String status = "";
    if (fanController.isFanStalled(1)) {
      status += "[STALL] ";
    }
    if (fanController.isFanSpinUpFailed(1)) {
      status += "[SPIN-FAIL] ";
    }
    if (fanController.isDriveFailed(1)) {
      status += "[DRIVE-FAIL] ";
    }
    if (fanController.isWatchdogTriggered()) {
      status += "[WATCHDOG] ";
    }
    if (!fanController.isRPMControlEnabled(1)) {
      status += "[RPM-DISABLED] ";
    }
    
    if (status.length() == 0) {
      status = "[OK]";
    }
    
    Serial.print("| ");
    Serial.println(status);
    
    // Clear faults periodically
    static int clearCounter = 0;
    if (++clearCounter >= 30) { // Clear every 30 seconds
      fanController.clearFaults();
      clearCounter = 0;
    }
  }
  
  delay(100);
}