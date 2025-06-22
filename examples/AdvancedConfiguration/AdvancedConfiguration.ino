/*
 * AdvancedConfiguration.ino
 * 
 * Comprehensive example showing advanced EMC230x configuration
 * including multi-fan setup, PWM configuration, and all features.
 * 
 * This example demonstrates:
 * - Multi-fan configuration (EMC2302/3/5)
 * - PWM frequency and polarity settings
 * - Advanced timing and control parameters
 * - Watchdog timer configuration
 * - Clock source configuration
 * - Software lock functionality
 * - Address selection helper
 * 
 * Hardware required:
 * - EMC230x fan controller with multiple fans
 * - Multiple 4-wire PWM fans
 * - I2C connections to Arduino
 */

#include <EMC230x.h>

// Create EMC230x instance - will auto-detect device type
EMC230x fanController(0x2F);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("EMC230x Advanced Configuration Example");
  Serial.println("======================================");
  
  // Initialize the fan controller
  if (!fanController.begin()) {
    Serial.println("ERROR: Failed to initialize EMC230x!");
    Serial.println("Checking possible I2C addresses...");
    
    // Try different common addresses
    uint8_t addresses[] = {0x2C, 0x2D, 0x2E, 0x2F, 0x4C, 0x4D};
    for (int i = 0; i < 6; i++) {
      EMC230x testController(addresses[i]);
      if (testController.isConnected()) {
        Serial.print("Found EMC230x at address 0x");
        Serial.println(addresses[i], HEX);
      }
    }
    while (1) delay(1000);
  }
  
  Serial.println("EMC230x initialized successfully!");
  
  // Device identification and capabilities
  Serial.println("\n=== Device Information ===");
  EMC230xDevice device = fanController.detectDevice();
  Serial.print("Device type: EMC230");
  Serial.println(device);
  Serial.print("Product ID: 0x");
  Serial.println(fanController.getProductID(), HEX);
  Serial.print("Manufacturer ID: 0x");
  Serial.println(fanController.getManufacturerID(), HEX);
  Serial.print("Revision: 0x");
  Serial.println(fanController.getRevision(), HEX);
  Serial.print("Maximum fans: ");
  int maxFans = fanController.getMaxFans();
  Serial.println(maxFans);
  
  // Address selection helper demonstration
  Serial.println("\n=== Address Selection Reference ===");
  float resistorValues[] = {4700, 6800, 10000, 15000, 22000, 33000};
  for (int i = 0; i < 6; i++) {
    uint8_t addr = EMC230xAddressDecoder::getAddressFromResistor(resistorValues[i]);
    bool clockAvailable = EMC230xAddressDecoder::isClockPinAvailable(resistorValues[i]);
    uint8_t defaultSpeed = EMC230xAddressDecoder::getDefaultFanSpeed(resistorValues[i]);
    
    Serial.print(resistorValues[i], 0);
    Serial.print("Ω → 0x");
    Serial.print(addr, HEX);
    Serial.print(", Clock: ");
    Serial.print(clockAvailable ? "Yes" : "No");
    Serial.print(", Default speed: ");
    Serial.print((float)defaultSpeed * 100.0 / 255.0, 1);
    Serial.println("%");
  }
  
  // Clear all faults before configuration
  fanController.clearFaults();
  
  // Configure each available fan
  Serial.println("\n=== Configuring Fans ===");
  for (int fan = 1; fan <= maxFans; fan++) {
    Serial.print("Configuring Fan ");
    Serial.println(fan);
    
    // Configure PWM settings
    configurePWM(fan);
    
    // Configure fan-specific parameters
    configureFan(fan);
    
    // Set initial speed based on fan number
    float initialSpeed = 20.0 + (fan * 15.0); // 35%, 50%, 65%, etc.
    fanController.setFanSpeedPercent(fan, initialSpeed);
    
    Serial.print("  Initial speed set to ");
    Serial.print(initialSpeed, 1);
    Serial.println("%");
  }
  
  // Configure global settings
  Serial.println("\n=== Global Configuration ===");
  configureGlobalSettings();
  
  // Display final configuration
  Serial.println("\n=== Configuration Summary ===");
  displayConfiguration();
  
  Serial.println("\n=== Starting Operation ===");
  Serial.println("All fans configured and running!");
  Serial.println("Monitor output shows: Fan | Mode | Target | Actual | PWM% | Status");
}

void configurePWM(int fan) {
  // Set PWM frequency based on fan number for demonstration
  PWMFrequency freq;
  switch (fan % 4) {
    case 1: freq = PWM_FREQ_26000_HZ; break;
    case 2: freq = PWM_FREQ_19531_HZ; break;
    case 3: freq = PWM_FREQ_4882_HZ; break;
    case 0: freq = PWM_FREQ_2441_HZ; break;
  }
  
  fanController.setPWMFrequency(fan, freq);
  fanController.setPWMDivider(fan, 1);          // No additional division
  fanController.setPWMPolarity(fan, false);     // Normal polarity
  fanController.setPWMOutputType(fan, true);    // Push-pull output
  
  Serial.print("  PWM frequency: ");
  switch (freq) {
    case PWM_FREQ_26000_HZ: Serial.println("26 kHz"); break;
    case PWM_FREQ_19531_HZ: Serial.println("19.5 kHz"); break;
    case PWM_FREQ_4882_HZ: Serial.println("4.9 kHz"); break;
    case PWM_FREQ_2441_HZ: Serial.println("2.4 kHz"); break;
  }
}

void configureFan(int fan) {
  // Configure basic fan parameters
  FanConfig config;
  config.enableClosedLoop = (fan <= 2);     // RPM control for first 2 fans
  config.minRPM = 1;                         // 1000 RPM minimum
  config.edges = 1;                          // 5 edges (2-pole fans)
  config.updateTime = 3;                     // 800ms update time
  config.enableRampRate = true;              // Smooth transitions
  config.enableGlitchFilter = true;          // Noise filtering
  config.errorWindow = 2;                    // ±100 RPM error window
  
  fanController.setFanConfig(fan, config);
  
  if (config.enableClosedLoop) {
    // Configure PID gains for RPM control
    PIDGains pidGains;
    pidGains.kp = 1;                         // 2x proportional gain
    pidGains.ki = 1;                         // 2x integral gain
    pidGains.kd = 0;                         // 1x derivative gain
    pidGains.derivativeOption = 1;           // Basic derivative
    
    fanController.setPIDGains(fan, pidGains);
    
    // Configure spin-up
    SpinUpConfig spinConfig;
    spinConfig.spinTime = 1;                 // 500ms spin-up
    spinConfig.spinLevel = 4;                // 50% spin-up level
    spinConfig.noKick = false;               // Allow initial kick
    spinConfig.driveFailCount = 2;           // 32 periods before fail
    
    fanController.setSpinUpConfig(fan, spinConfig);
    
    // Set minimum drive and control limits
    fanController.setMinimumDrive(fan, 25);  // Minimum 25/255 PWM
    fanController.setMaximumStep(fan, 20);   // Maximum step size
    fanController.setValidTachCount(fan, 2); // Valid tach count
    
    // Enable RPM control
    fanController.enableRPMControl(fan, true);
    
    // Set target RPM
    uint16_t targetRPM = 1000 + (fan * 300); // 1300, 1600, 1900, etc.
    fanController.setTargetRPM(fan, targetRPM);
    
    Serial.print("  Mode: RPM control, Target: ");
    Serial.print(targetRPM);
    Serial.println(" RPM");
  } else {
    Serial.println("  Mode: PWM control");
  }
  
  // Enable interrupts for this fan
  fanController.enableInterrupt(fan, true);
}

void configureGlobalSettings() {
  // Configure watchdog timer
  fanController.enableWatchdog(false);      // Disable continuous watchdog
  Serial.println("Watchdog timer: Disabled");
  
  // Use internal clock (most common configuration)
  fanController.useExternalClock(false);
  fanController.enableClockOutput(false);
  Serial.println("Clock source: Internal");
  
  // Note: Don't lock configuration in example code
  // fanController.lockConfiguration(); // Uncomment to lock after setup
  Serial.println("Configuration lock: Disabled (for development)");
}

void displayConfiguration() {
  int maxFans = fanController.getMaxFans();
  
  for (int fan = 1; fan <= maxFans; fan++) {
    Serial.print("Fan ");
    Serial.print(fan);
    Serial.print(": ");
    
    if (fanController.isRPMControlEnabled(fan)) {
      Serial.print("RPM mode, Target: ");
      Serial.print(fanController.getTargetRPM(fan));
      Serial.print(" RPM");
    } else {
      Serial.print("PWM mode, Speed: ");
      Serial.print(fanController.getFanSpeedPercent(fan), 1);
      Serial.print("%");
    }
    
    Serial.print(", Interrupts: ");
    Serial.println(fanController.isInterruptEnabled(fan) ? "On" : "Off");
  }
}

void loop() {
  static unsigned long lastUpdate = 0;
  
  if (millis() - lastUpdate >= 2000) { // Update every 2 seconds
    lastUpdate = millis();
    
    Serial.println();
    int maxFans = fanController.getMaxFans();
    
    for (int fan = 1; fan <= maxFans; fan++) {
      Serial.print("Fan");
      Serial.print(fan);
      Serial.print(" | ");
      
      if (fanController.isRPMControlEnabled(fan)) {
        uint16_t target = fanController.getTargetRPM(fan);
        uint16_t actual = fanController.getActualRPM(fan);
        Serial.print("RPM | ");
        Serial.print(target);
        Serial.print(" | ");
        Serial.print(actual);
      } else {
        Serial.print("PWM | ");
        Serial.print("--- | ");
        Serial.print(fanController.getActualRPM(fan));
      }
      
      Serial.print(" | ");
      Serial.print(fanController.getFanSpeedPercent(fan), 1);
      Serial.print("% | ");
      
      // Status
      String status = "";
      if (fanController.isFanStalled(fan)) status += "STALL ";
      if (fanController.isFanSpinUpFailed(fan)) status += "SPIN-FAIL ";
      if (fanController.isDriveFailed(fan)) status += "DRIVE-FAIL ";
      if (status.length() == 0) status = "OK";
      
      Serial.println(status);
    }
    
    // Global status
    if (fanController.isWatchdogTriggered()) {
      Serial.println(">>> WATCHDOG TRIGGERED <<<");
    }
    
    // Clear faults periodically
    static int clearCounter = 0;
    if (++clearCounter >= 15) { // Every 30 seconds
      fanController.clearFaults();
      clearCounter = 0;
      Serial.println("(Faults cleared)");
    }
  }
  
  delay(100);
}