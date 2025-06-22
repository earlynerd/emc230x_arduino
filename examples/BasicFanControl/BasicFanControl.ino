/*
 * BasicFanControl.ino
 * 
 * Basic example demonstrating PWM-based fan speed control
 * using the EMC230x library.
 * 
 * This example shows how to:
 * - Initialize the EMC230x controller
 * - Set fan speeds using PWM values and percentages
 * - Read current fan settings
 * - Monitor actual RPM (tachometer readings)
 * 
 * Hardware required:
 * - EMC230x fan controller (any variant)
 * - 3-wire or 4-wire PWM fans
 * - I2C connections to Arduino
 * 
 * Connections:
 * - SDA to Arduino SDA pin
 * - SCL to Arduino SCL pin
 * - VDD to 3.3V or 5V (depending on your EMC230x variant)
 * - GND to Ground
 * - ADDR pin with appropriate resistor for I2C address selection
 */

#include <EMC230x.h>

// Create EMC230x instance with default I2C address (0x2F)
// Change address based on your hardware configuration
EMC230x fanController(0x2F);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Wait for Serial on boards like Leonardo
  }
  
  Serial.println("EMC230x Basic Fan Control Example");
  Serial.println("==================================");
  
  // Initialize the fan controller
  if (!fanController.begin()) {
    Serial.println("ERROR: Failed to initialize EMC230x!");
    Serial.println("Check I2C connections and address.");
    while (1) {
      delay(1000);
    }
  }
  
  Serial.println("EMC230x initialized successfully!");
  
  // Display device information
  Serial.print("Product ID: 0x");
  Serial.println(fanController.getProductID(), HEX);
  Serial.print("Manufacturer ID: 0x");
  Serial.println(fanController.getManufacturerID(), HEX);
  Serial.print("Revision: 0x");
  Serial.println(fanController.getRevision(), HEX);
  Serial.print("Max fans supported: ");
  Serial.println(fanController.getMaxFans());
  
  Serial.println("\nStarting fan control sequence...");
  Serial.println("Fan will cycle through different speeds every 5 seconds");
  
  // Ensure we're in PWM mode (not RPM control)
  fanController.enableRPMControl(1, false);
  
  // Clear any existing faults
  fanController.clearFaults();
}

void loop() {
  // Cycle through different fan speeds
  static int step = 0;
  static unsigned long lastUpdate = 0;
  
  if (millis() - lastUpdate >= 5000) { // Update every 5 seconds
    lastUpdate = millis();
    
    switch (step) {
      case 0:
        Serial.println("\n--- Setting fan to 0% (stopped) ---");
        fanController.setFanSpeedPercent(1, 0);
        break;
        
      case 1:
        Serial.println("\n--- Setting fan to 25% speed ---");
        fanController.setFanSpeedPercent(1, 25);
        break;
        
      case 2:
        Serial.println("\n--- Setting fan to 50% speed ---");
        fanController.setFanSpeedPercent(1, 50);
        break;
        
      case 3:
        Serial.println("\n--- Setting fan to 75% speed ---");
        fanController.setFanSpeedPercent(1, 75);
        break;
        
      case 4:
        Serial.println("\n--- Setting fan to 100% speed ---");
        fanController.setFanSpeedPercent(1, 100);
        break;
        
      case 5:
        Serial.println("\n--- Setting fan using raw PWM value (128/255) ---");
        fanController.setFanSpeed(1, 128); // ~50% using raw PWM value
        break;
    }
    
    step = (step + 1) % 6; // Cycle through 6 steps
  }
  
  // Display current status every second
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus >= 1000) {
    lastStatus = millis();
    
    // Read current settings
    uint8_t pwmValue = fanController.getFanSpeed(1);
    float percentage = fanController.getFanSpeedPercent(1);
    uint16_t actualRPM = fanController.getActualRPM(1);
    
    // Display status
    Serial.print("PWM: ");
    Serial.print(pwmValue);
    Serial.print("/255 (");
    Serial.print(percentage, 1);
    Serial.print("%) | RPM: ");
    Serial.print(actualRPM);
    
    // Check for any faults
    if (fanController.isFanStalled(1)) {
      Serial.print(" | STALLED!");
    }
    if (fanController.isFanSpinUpFailed(1)) {
      Serial.print(" | SPIN-UP FAILED!");
    }
    if (fanController.isDriveFailed(1)) {
      Serial.print(" | DRIVE FAILED!");
    }
    
    Serial.println();
  }
  
  delay(100); // Small delay to prevent overwhelming the serial output
}