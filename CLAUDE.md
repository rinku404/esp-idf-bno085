# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an ESP-IDF project for the ESP32-S3 microcontroller that integrates a BNO085 9-axis motion sensor (IMU) using the SH2 sensor hub protocol. The project uses CMake as the build system and is designed to be developed either locally with ESP-IDF or within a Docker devcontainer.

## Build & Development Commands

### Prerequisites
- **Local Setup**: ESP-IDF v5.5.5 installed at `C:\esp\v5.5.5\esp-idf`
- **Container Setup**: Use the devcontainer (`.devcontainer/devcontainer.json`) for a pre-configured environment

### Common Build Commands
```bash
# Configure the project (creates/updates build/ directory)
idf.py set-target esp32s3
idf.py build

# Full clean rebuild
idf.py fullclean
idf.py build

# Flash to device (requires USB connection)
idf.py flash

# Monitor serial output (typically at 115200 baud)
idf.py monitor

# Combined build and flash
idf.py build flash

# Open the configuration menu
idf.py menuconfig
```

### Build Targets
- **Target**: ESP32-S3 (set in `.vscode/settings.json` via `IDF_TARGET`)
- **Build Directory**: `build/`
- **Compiler**: ESP-Clang (v19.1.2)
- **OpenOCD Configuration**: `board/esp32s3-builtin.cfg`

### Code Quality Tools
- **Clangd**: Configured for IntelliSense and language features (uses compile_commands.json from build directory)
- **Lint**: Run via clangd diagnostics in VS Code or through `idf.py check-metadata`

## Architecture & Components

### Component Structure (Registry-Ready)
```
components/
└── bno085/                         # Publishable IMU driver component
    ├── bno085.h                    # Public API (opaque types, NO sh2.h included)
    ├── bno085.c                    # Handle-based implementation
    ├── CMakeLists.txt              # Component registration (nested sh2 path)
    ├── idf_component.yml           # ESP-IDF registry metadata
    ├── LICENSE                     # Apache-2.0
    ├── README.md                   # Usage guide, wiring, quick-start
    ├── SENSOR_GUIDE.md             # All 40+ sensor types and examples
    ├── sh2/                        # SH2 library (git submodule, v1.4.0)
    │   └── (vendored from github.com/ceva-dsp/sh2)
    └── examples/
        └── basic_read/             # Self-contained example project

main/                              # Application (example of using bno085)
├── main.c                         # Demo app using handle-based API
└── CMakeLists.txt
```

### Key Design Points

**BNO085 Component (Decoupled & Professional)**:
- **Public API**: Handle-based (opaque `bno085_handle_t`), no sh2 types exposed
- **Callback**: Receives `bno085_sensor_value_t` (decoded data) with 5 sensor types decoded into named fields
- **Configuration**: Tunable via `bno085_config_t` struct (I2C timeout, reset behavior)
- **Single Instance**: Only one active BNO085 at a time (sh2_t _sh2 is global in submodule)
- **Dependencies**: I2C master driver (`driver/i2c_master.h`), GPIO, `esp_timer.h`, FreeRTOS

**Internals** (Hidden from Users):
- SH2 protocol abstraction via 5 HAL callbacks (open/close/read/write/getTimeUs)
- Soft-reset I2C packet sequence (b514b1e soft-reset payload) with retry logic
- Two-phase SHTP-over-I2C read (header-first, then full packet)
- Sensor event decoding (moved from app into component) — users never call `sh2_decodeSensorEvent()`

**Shipped Together**:
- SH2 submodule moved inside `components/bno085/sh2/` (pinned to v1.4.0)
- Ensures all dependencies ship with the component when installed from registry

**ESP-IDF Integration**:
- Targets ESP32-S3 (though chip-agnostic — works on any IDF 5.2+ board with I2C master driver)
- idf_component.yml declares Apache-2.0 license and registry metadata
- Example project at `components/bno085/examples/basic_read/` for quick-start

## Development Workflow

1. **Make Changes**: Edit source files in `components/bno085/` or `main/`
2. **Build**: Run `idf.py build` (incremental) or `idf.py fullclean && idf.py build`
3. **Flash**: Connect ESP32-S3 via USB and run `idf.py flash`
4. **Monitor**: Use `idf.py monitor` to view serial output and debug
5. **Iterate**: Clangd provides real-time diagnostics in VS Code

### VS Code Configuration
- C/C++ IntelliSense using default engine with clangd fallback
- CMake configuration disabled on open (manual control to avoid delays)
- Build and launch commands hidden from status bar
- Clangd at `C:\Espressif\tools\esp-clang\esp-19.1.2_20250312\esp-clang\bin\clangd.exe`

### Git Submodule Setup
The SH2 library is included as a git submodule. If cloning fresh:
```bash
git clone --recursive <repo-url>
# or if already cloned:
git submodule update --init --recursive
```

## API Quick Reference

**Initialize:**
```c
bno085_config_t config;
bno085_config_default(&config);  // Use defaults or customize

bno085_handle_t bno085;
bno085_init(&config, i2c_dev_handle, int_gpio, rst_gpio, &bno085);
```

**Receive Data:**
```c
void callback(bno085_handle_t h, const bno085_sensor_value_t *v, void *ctx) {
    if (v->sensor_id == BNO085_SENSOR_ROTATION_VECTOR) {
        float i = v->data.rotation_vector.i;  // No sh2 types!
        // ...
    }
}
bno085_register_sensor_callback(bno085, callback, NULL);
```

**Enable Sensors:**
```c
bno085_enable_sensor(bno085, BNO085_SENSOR_ROTATION_VECTOR, 100000);  // 10Hz
```

**Service Loop:**
```c
while (1) {
    bno085_service(bno085);
    vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz polling
}
```

**Cleanup:**
```c
bno085_deinit(bno085);
```

## Registry Publishing

The component is ready for publication:

1. Update `idf_component.yml` URL/repository fields with the actual GitHub repo URL
2. Run `compote component validate components/bno085` to lint (if compote is installed)
3. Push to a public GitHub repo with Apache-2.0 license
4. Submit to [ESP-IDF Component Registry](https://components.espressif.com/)

After publishing, users can install via:
```bash
idf.py add-dependency bno085
```
