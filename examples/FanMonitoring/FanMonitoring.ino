/*
 * FanMonitoring.ino
 * 
 * Comprehensive fan monitoring and fault detection example
 * for the EMC230x library.
 * 
 * This example demonstrates:
 * - Real-time fan status monitoring
 * - Fault detection and reporting
 * - Tachometer signal analysis
 * - Performance metrics tracking
 * - Automatic fault recovery
 * - Data logging capabilities
 * 
 * Hardware required:
 * - EMC230x fan controller
 * - 4-wire PWM fans with tachometer
 * - I2C connections to Arduino
 */

#include <EMC230x.h>

// Create EMC230x instance
EMC230x fanController(0x2F);

// Monitoring structure for each fan
struct FanMonitor {
  uint32_t totalRunTime;        // Total runtime in seconds
  uint32_t stallCount;          // Number of stall events
  uint32_t spinFailCount;       // Number of spin-up failures
  uint32_t driveFailCount;      // Number of drive failures
  uint16_t minRPM;              // Minimum RPM recorded
  uint16_t maxRPM;              // Maximum RPM recorded
  float avgRPM;                 // Running average RPM
  uint32_t rpmSamples;          // Number of RPM samples
  bool isRunning;               // Current running state
  unsigned long lastStallTime;  // Last stall event timestamp
  unsigned long lastStartTime;  // Last successful start timestamp
};

FanMonitor monitors[6]; // Support up to 5 fans (index 0 unused)
unsigned long startTime;
bool loggingEnabled = true;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("EMC230x Fan Monitoring System");
  Serial.println("==============================");
  
  // Initialize the fan controller
  if (!fanController.begin()) {
    Serial.println("ERROR: Failed to initialize EMC230x!");
    while (1) delay(1000);
  }
  
  Serial.println("EMC230x initialized successfully!");
  startTime = millis();
  
  // Display device information
  EMC230xDevice device = fanController.detectDevice();
  int maxFans = fanController.getMaxFans();
  
  Serial.print("Device: EMC230");
  Serial.println(device);
  Serial.print("Maximum fans: ");
  Serial.println(maxFans);
  
  // Initialize monitoring structures
  for (int i = 1; i <= maxFans; i++) {
    monitors[i].totalRunTime = 0;
    monitors[i].stallCount = 0;
    monitors[i].spinFailCount = 0;
    monitors[i].driveFailCount = 0;
    monitors[i].minRPM = 65535;
    monitors[i].maxRPM = 0;
    monitors[i].avgRPM = 0;
    monitors[i].rpmSamples = 0;
    monitors[i].isRunning = false;
    monitors[i].lastStallTime = 0;
    monitors[i].lastStartTime = 0;
  }
  
  // Configure all fans for monitoring
  Serial.println("\nConfiguring fans for monitoring...");
  setupFansForMonitoring(maxFans);
  
  // Print monitoring header
  Serial.println("\n=== Real-time Fan Monitoring ===");
  Serial.println("Commands: 's' = toggle speed, 'r' = reset stats, 'l' = toggle logging");
  Serial.println("Time | Fan | RPM  | PWM% | Status     | Faults");
  Serial.println("-----|-----|------|------|------------|--------");
  
  // Start all fans at different speeds for testing
  for (int i = 1; i <= maxFans; i++) {
    float speed = 30.0 + (i * 20.0); // 50%, 70%, 90%, etc.
    fanController.setFanSpeedPercent(i, speed);
    monitors[i].isRunning = true;
    monitors[i].lastStartTime = millis();
  }
}

void setupFansForMonitoring(int maxFans) {
  for (int fan = 1; fan <= maxFans; fan++) {
    // Configure for optimal monitoring
    FanConfig config;
    config.enableClosedLoop = false;      // PWM mode for consistent monitoring
    config.minRPM = 1;                    // 1000 RPM minimum range
    config.edges = 1;                     // 5 edges (2-pole fans)
    config.updateTime = 2;                // 400ms update time (faster monitoring)
    config.enableRampRate = false;        // No ramp rate for immediate response
    config.enableGlitchFilter = true;     // Filter noise
    config.errorWindow = 1;               // ±50 RPM error window
    
    fanController.setFanConfig(fan, config);
    
    // Configure spin-up for reliable starting
    SpinUpConfig spinConfig;
    spinConfig.spinTime = 2;              // 1 second spin-up
    spinConfig.spinLevel = 5;             // 55% spin-up level
    spinConfig.noKick = false;            // Allow initial kick
    spinConfig.driveFailCount = 1;        // 16 periods before drive fail
    
    fanController.setSpinUpConfig(fan, spinConfig);
    
    // Set minimum drive to prevent stalling
    fanController.setMinimumDrive(fan, 40); // Higher minimum for reliability
    
    // Enable interrupts for immediate fault detection
    fanController.enableInterrupt(fan, true);
    
    Serial.print("Fan ");
    Serial.print(fan);
    Serial.println(" configured for monitoring");
  }
  
  fanController.clearFaults();
}

void loop() {
  static unsigned long lastMonitorUpdate = 0;
  static unsigned long lastStatUpdate = 0;
  
  // Handle serial commands
  handleSerialCommands();
  
  // Monitor fans every 500ms
  if (millis() - lastMonitorUpdate >= 500) {
    lastMonitorUpdate = millis();
    monitorFans();
  }
  
  // Update statistics every 10 seconds
  if (millis() - lastStatUpdate >= 10000) {
    lastStatUpdate = millis();
    updateStatistics();
  }
  
  delay(50);
}

void monitorFans() {
  int maxFans = fanController.getMaxFans();
  unsigned long currentTime = millis();
  uint32_t elapsedSeconds = (currentTime - startTime) / 1000;
  
  for (int fan = 1; fan <= maxFans; fan++) {
    uint16_t currentRPM = fanController.getActualRPM(fan);
    float currentPWM = fanController.getFanSpeedPercent(fan);
    
    // Update RPM statistics
    if (currentRPM > 0) {
      if (currentRPM < monitors[fan].minRPM) monitors[fan].minRPM = currentRPM;
      if (currentRPM > monitors[fan].maxRPM) monitors[fan].maxRPM = currentRPM;
      
      // Update running average
      monitors[fan].avgRPM = (monitors[fan].avgRPM * monitors[fan].rpmSamples + currentRPM) / 
                             (monitors[fan].rpmSamples + 1);
      monitors[fan].rpmSamples++;
    }
    
    // Check for faults
    bool stalled = fanController.isFanStalled(fan);
    bool spinFailed = fanController.isFanSpinUpFailed(fan);
    bool driveFailed = fanController.isDriveFailed(fan);
    
    // Update fault counters
    if (stalled && monitors[fan].lastStallTime != currentTime) {
      monitors[fan].stallCount++;
      monitors[fan].lastStallTime = currentTime;
      monitors[fan].isRunning = false;
    }
    
    if (spinFailed) {
      monitors[fan].spinFailCount++;
      monitors[fan].isRunning = false;
    }
    
    if (driveFailed) {
      monitors[fan].driveFailCount++;
      monitors[fan].isRunning = false;
    }
    
    // Update running time
    if (monitors[fan].isRunning && currentRPM > 0) {
      monitors[fan].totalRunTime++;
    }
    
    // Display current status
    if (loggingEnabled) {
      char timeStr[10];
      sprintf(timeStr, "%5lu", elapsedSeconds);
      
      Serial.print(timeStr);
      Serial.print(" | ");
      Serial.print(fan);
      Serial.print("   | ");
      
      char rpmStr[5];
      sprintf(rpmStr, "%4d", currentRPM);
      Serial.print(rpmStr);
      Serial.print(" | ");
      
      char pwmStr[5];
      sprintf(pwmStr, "%4.1f", currentPWM);
      Serial.print(pwmStr);
      Serial.print(" | ");
      
      // Status
      String status = "";
      if (monitors[fan].isRunning && currentRPM > 100) {
        status = "Running   ";
      } else if (currentRPM == 0) {
        status = "Stopped   ";
      } else if (currentRPM < 100) {
        status = "Starting  ";
      } else {
        status = "Unknown   ";
      }
      Serial.print(status);
      Serial.print(" | ");
      
      // Faults
      String faults = "";
      if (stalled) faults += "STALL ";
      if (spinFailed) faults += "SPIN ";
      if (driveFailed) faults += "DRIVE ";
      if (faults.length() == 0) faults = "OK";
      
      Serial.println(faults);
    }
    
    // Automatic recovery for stalled fans
    if (stalled || spinFailed) {
      recoverFan(fan);
    }
  }
  
  // Clear faults after processing
  fanController.clearFaults();
}

void recoverFan(int fan) {
  static unsigned long lastRecovery[6] = {0};
  unsigned long currentTime = millis();
  
  // Limit recovery attempts to once every 30 seconds
  if (currentTime - lastRecovery[fan] < 30000) {
    return;
  }
  
  lastRecovery[fan] = currentTime;
  
  Serial.print(">>> Attempting recovery for Fan ");
  Serial.println(fan);
  
  // Stop fan briefly
  fanController.setFanSpeed(fan, 0);
  delay(1000);
  
  // Restart with higher initial speed
  float recoverySpeed = fanController.getFanSpeedPercent(fan);
  if (recoverySpeed < 50.0) {
    recoverySpeed = 60.0; // Boost to 60% for recovery
  }
  
  fanController.setFanSpeedPercent(fan, recoverySpeed);
  monitors[fan].isRunning = true;
  monitors[fan].lastStartTime = currentTime;
  
  Serial.print(">>> Fan ");
  Serial.print(fan);
  Serial.print(" restarted at ");
  Serial.print(recoverySpeed, 1);
  Serial.println("%");
}

void updateStatistics() {
  Serial.println("\n=== Fan Statistics Update ===");
  int maxFans = fanController.getMaxFans();
  
  for (int fan = 1; fan <= maxFans; fan++) {
    Serial.print("Fan ");
    Serial.print(fan);
    Serial.print(": Runtime=");
    Serial.print(monitors[fan].totalRunTime);
    Serial.print("s, RPM=");
    Serial.print(monitors[fan].minRPM);
    Serial.print("-");
    Serial.print(monitors[fan].maxRPM);
    Serial.print(" (avg ");
    Serial.print(monitors[fan].avgRPM, 0);
    Serial.print("), Faults: Stall=");
    Serial.print(monitors[fan].stallCount);
    Serial.print(", Spin=");
    Serial.print(monitors[fan].spinFailCount);
    Serial.print(", Drive=");
    Serial.println(monitors[fan].driveFailCount);
  }
  Serial.println("=============================\n");
}

void handleSerialCommands() {
  if (Serial.available()) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case 's':
      case 'S':
        // Toggle fan speeds
        toggleFanSpeeds();
        break;
        
      case 'r':
      case 'R':
        // Reset statistics
        resetStatistics();
        break;
        
      case 'l':
      case 'L':
        // Toggle logging
        loggingEnabled = !loggingEnabled;
        Serial.print("Logging ");
        Serial.println(loggingEnabled ? "enabled" : "disabled");
        break;
        
      case '?':
      case 'h':
      case 'H':
        // Help
        Serial.println("\nCommands:");
        Serial.println("  s - Toggle fan speeds");
        Serial.println("  r - Reset statistics");
        Serial.println("  l - Toggle logging");
        Serial.println("  h - Show this help");
        break;
    }
  }
}

void toggleFanSpeeds() {
  static bool highSpeed = false;
  int maxFans = fanController.getMaxFans();
  
  highSpeed = !highSpeed;
  
  for (int fan = 1; fan <= maxFans; fan++) {
    float speed = highSpeed ? 80.0 : 40.0;
    fanController.setFanSpeedPercent(fan, speed);
    monitors[fan].isRunning = true;
    monitors[fan].lastStartTime = millis();
  }
  
  Serial.print(">>> All fans set to ");
  Serial.print(highSpeed ? "high" : "low");
  Serial.println(" speed");
}

void resetStatistics() {
  int maxFans = fanController.getMaxFans();
  
  for (int fan = 1; fan <= maxFans; fan++) {
    monitors[fan].totalRunTime = 0;
    monitors[fan].stallCount = 0;
    monitors[fan].spinFailCount = 0;
    monitors[fan].driveFailCount = 0;
    monitors[fan].minRPM = 65535;
    monitors[fan].maxRPM = 0;
    monitors[fan].avgRPM = 0;
    monitors[fan].rpmSamples = 0;
  }
  
  startTime = millis();
  fanController.clearFaults();
  
  Serial.println(">>> Statistics reset");
}