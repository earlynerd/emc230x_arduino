# EMC230x Arduino Library

A comprehensive Arduino library for controlling EMC2301, EMC2302, EMC2303, and EMC2305 RPM-based PWM fan controllers via I2C communication.

## Features

- **Multiple Device Support**: EMC2301 (1 fan), EMC2302 (2 fans), EMC2303 (3 fans), EMC2305 (5 fans)
- **Dual Control Modes**: Direct PWM control and closed-loop RPM control
- **Advanced Configuration**: PID tuning, spin-up settings, fault detection
- **Comprehensive Monitoring**: RPM readings, fault status, stall detection
- **Hardware Features**: Watchdog timer, clock configuration, software lock
- **Easy Integration**: Simple Arduino-style API with examples

## Installation

### Manual Installation
1. Download this library as a ZIP file
2. Open Arduino IDE
3. Go to **Sketch > Include Library > Add .ZIP Library**
4. Select the downloaded ZIP file

## Hardware Connection

Connect your EMC230x device to your Arduino via I2C:

| Arduino Pin | EMC230x Pin | Description |
|------------|-------------|-------------|
| SDA | SDA | I2C Data |
| SCL | SCL | I2C Clock |
| 3.3V or 5V | VDD | Power Supply |
| GND | GND | Ground |

**Note**: The EMC230x operates at 3.3V logic levels. Use level shifters if connecting to 5V Arduino boards.

## I2C Address Selection

The EMC230x I2C address is determined by an external resistor on the ADDR pin:

| Resistor Value | I2C Address |
|----------------|-------------|
| 4.7kΩ | 0x2E |
| 6.8kΩ | 0x2F (default) |
| 10kΩ | 0x2C |
| 15kΩ | 0x2D |
| 22kΩ | 0x4C |
| 33kΩ | 0x4D |

## Quick Start

```cpp
#include <EMC230x.h>

EMC230x fanController; // Uses default address 0x2F

void setup() {
  Serial.begin(115200);
  
  if (!fanController.begin()) {
    Serial.println("Failed to initialize EMC230x!");
    while (1);
  }
  
  Serial.println("EMC230x initialized successfully!");
  
  // Set fan 1 to 50% speed
  fanController.setFanSpeedPercent(1, 50.0);
}

void loop() {
  // Read current RPM
  uint16_t rpm = fanController.getActualRPM(1);
  Serial.print("Fan 1 RPM: ");
  Serial.println(rpm);
  
  delay(1000);
}
```

## API Reference

### Initialization

```cpp
EMC230x fanController(address, wireInterface);
bool begin();
bool isConnected();
```

### Device Information

```cpp
uint8_t getProductID();
uint8_t getManufacturerID();
uint8_t getRevision();
EMC230xDevice detectDevice();
```

### PWM Control

```cpp
bool setFanSpeed(uint8_t fan, uint8_t speed);           // 0-255
bool setFanSpeedPercent(uint8_t fan, float percent);    // 0-100%
uint8_t getFanSpeed(uint8_t fan);
float getFanSpeedPercent(uint8_t fan);
```

### RPM Control

```cpp
bool setTargetRPM(uint8_t fan, uint16_t rpm);
uint16_t getTargetRPM(uint8_t fan);
uint16_t getActualRPM(uint8_t fan);
bool enableRPMControl(uint8_t fan, bool enable);
```

### Configuration

```cpp
bool setFanConfig(uint8_t fan, const FanConfig &config);
bool setPIDGains(uint8_t fan, const PIDGains &gains);
bool setSpinUpConfig(uint8_t fan, const SpinUpConfig &config);
```

### Status and Monitoring

```cpp
bool isFanStalled(uint8_t fan);
bool isFanSpinUpFailed(uint8_t fan);
bool isDriveFailed(uint8_t fan);
uint8_t getFanStatus();
bool clearFaults();
```

## Examples

The library includes several examples:

- **BasicFanControl**: Simple PWM speed control
- **RPMControl**: Closed-loop RPM control with PID
- **AdvancedConfiguration**: Complete device configuration
- **FanMonitoring**: Status monitoring and fault detection

## Device Variants

| Device | Fans | Key Features |
|--------|------|--------------|
| EMC2301 | 1 | Single fan control |
| EMC2302 | 2 | Dual fan control |
| EMC2303 | 3 | Triple fan control, address selection |
| EMC2305 | 5 | Five fan control, address selection |

## Troubleshooting

### Common Issues

**Library not found**: Ensure the library is properly installed and restart Arduino IDE.

**I2C communication failed**: Check wiring and I2C address. Use an I2C scanner to verify device presence.

**Inaccurate RPM readings**: Verify fan specifications (poles/edges) and configure accordingly using `setFanConfig()`.

**Fan not spinning**: Check power supply, enable RPM control, and verify minimum drive settings.

### Debug Tips

Enable debug output in examples to see detailed I2C communication and register values.

## Contributing

Contributions are welcome! Please feel free to submit issues, feature requests, or pull requests.

## Support

For questions and support:
- Check the examples included with the library
- Review the EMC230x datasheet for detailed register information
- Open an issue on GitHub for bugs or feature requests
