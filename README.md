# OpenFit Bike - Smart Fitness Electronics System

A comprehensive smart bike electronics system that transforms a static indoor bike into an intelligent fitness tracking platform with AI coaching capabilities.

## 🎯 Project Vision

This project is the foundation of a holistic health system built from the ground up, starting with custom electronics for an indoor static bike. The goal is to create an integrated ecosystem that tracks workouts, syncs with platforms like Strava, and eventually incorporates AI-powered coaching.

## 🏗️ System Architecture

The system consists of two ESP32-based modules that work together to provide comprehensive bike monitoring:

### 1. Internals Module (`/internals/`)
**Hardware**: ESP32 Dev Board  
**Purpose**: Core sensor data collection and BLE communication

**Features**:
- **Cadence Sensing**: Reed/Hall sensor on crank (2 magnets per revolution)
- **Resistance Measurement**: Analog resistance knob monitoring via ADC
- **Power Calculation**: Physical model (flywheel inertia + felt-pad friction + magnetic brake), calibrated per knob position with a spin-down
- **Speed Calculation**: Virtual speed based on gear ratio and wheel circumference
- **BLE Broadcasting**: FTMS (Fitness Machine Service) protocol for app connectivity
- **Calibration System**: Interactive 3-point resistance calibration
- **Persistent Settings**: Configuration stored in ESP32 preferences

**Technical Specs**:
- Device Name: "LibrePulse Bike"
- Communication: BLE FTMS Indoor Bike Data (UUID: 0x2AD2)
- Update Rate: 10Hz sensor broadcasting
- BLE CSC (UUID: 0x1816) with exact crank revolution timing

### 2. Display Module (`/display/`)
**Hardware**: Adafruit Feather ESP32-S3 TFT  
**Purpose**: Visual feedback and user interface

**Status**: Basic PlatformIO setup (development in progress)  
**Future Plans**: 
- Real-time metrics display
- Workout session tracking
- Configuration interface
- Connectivity status indicators

## 🔧 Hardware Setup

### Components Used
- **Main Controller**: ESP32 Dev Board
- **Display Unit**: Adafruit Feather ESP32-S3 with capacitive display
- **Sensors**: 
  - Reed/Hall sensor for cadence (GPIO 27)
  - Analog resistance sensor (GPIO 34)
- **Original Bike**: Repurposed static indoor bike with custom electronics

### Wiring
- Cadence sensor: GPIO 27 (with internal pullup)
- Resistance sensor: GPIO 34 (ADC1, 12-bit resolution)
- Serial debugging: 115200 baud

*Note*: Two ESP32s were used due to connector availability constraints for the display unit.

## 🚀 Getting Started

### Prerequisites
- PlatformIO IDE
- ESP32 development environment
- Static bike with resistance knob mechanism

### Installation

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd openfit-bike
   ```

2. **Flash the internals module**
   ```bash
   cd internals
   pio run --target upload
   ```

3. **Flash the display module** (when ready)
   ```bash
   cd display/openfit-display
   pio run --target upload
   ```

### Initial Calibration

1. Connect to the internals module via serial (115200 baud)
2. Run the calibration command: `cal`
3. Follow the 3-point calibration process:
   - **FREE**: Knob fully loose (no resistance)
   - **ENGAGE**: Point where resistance just starts
   - **HIGH**: Maximum comfortable resistance level

### Configuration Commands

The internals module provides a rich command interface:

- `cal` - Run resistance calibration
- `show` - Display current settings
- `raw` - Monitor raw ADC values
- `setcirc <mm>` - Set wheel circumference (1500-3000mm)
- `setratio <ratio>` - Set gear ratio (0.3-5.0)
- `setfly <kg> <diameter_cm> <factor> <ratio>` - Flywheel mass, diameter, inertia factor (0.5 solid disc to ~0.8 rim-weighted) and flywheel revs per crank rev
- `spindown` - Measure the brake at the current knob position: pedal past 80 rpm, take your feet off, let the cranks coast to a stop
- `showpower` / `setbrake <pct> <friction_Nm> <magnetic_Nms>` / `clearpower` - Inspect, set by hand or reset the brake model
- `help` - Show all commands

## 🔗 Integration Roadmap

### Phase 1: Core Hardware ✅
- [x] ESP32 sensor integration
- [x] BLE FTMS implementation
- [x] Calibration system
- [ ] Display module completion

### Phase 2: Data Integration 🚧
- [ ] [Auuki](https://github.com/search?q=auuki) integration for workout tracking
- [ ] Heart rate monitor connectivity
- [ ] Strava synchronization
- [ ] Workout session management

### Phase 3: AI Coaching 🔮
- [ ] Machine learning workout analysis
- [ ] Personalized training recommendations
- [ ] Real-time form feedback
- [ ] Progress tracking and goals

## 📊 Data Output

The system broadcasts the following metrics via BLE:
- **Cadence**: 0.5 RPM resolution
- **Speed**: 0.01 km/h resolution (virtual)
- **Power**: 1W resolution (torque-based calculation)
- **Resistance**: 0-100% scale

## 🛠️ Development Status

**Current State**: Core sensor functionality complete with BLE broadcasting  
**Next Steps**: 
1. Complete display module interface
2. Integrate with workout tracking software
3. Implement data logging and analysis

## 🤝 Contributing

This is a personal project exploring the intersection of hardware, fitness, and AI. Contributions, suggestions, and collaborations are welcome!

## 📝 Technical Notes

- **Power Calculation**: Crank torque = I·α + friction(knob) + magnetic(knob)·ω, with I the flywheel inertia seen at the crank. Spin-downs measure friction and magnetic torque in absolute units; do one at several knob positions (e.g. 0, 30, 60, 90 %). The model lives in `internals/lib/PowerModel` and is unit-tested on the host with `pio test -e native`
- **Calibration**: 3-point system automatically handles different resistance mechanisms
- **BLE Protocol**: Standard FTMS ensures compatibility with existing fitness apps
- **Memory**: All settings persist across power cycles using ESP32 preferences

## 📸 Future Additions

- Hardware photos and assembly guides
- Display module screenshots
- Integration demonstrations
- AI coaching interface mockups

---

*This project represents the first step toward a comprehensive, AI-driven fitness ecosystem built from custom hardware up to intelligent software coaching.*