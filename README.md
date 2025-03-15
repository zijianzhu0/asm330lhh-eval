# asm330lhh-eval - Streaming 6-axis data over serial on boot

This project enables automatic sensor data streaming upon boot using the STEVAL-MKI193V1 and STEVAL-MKI109V3 combo boards. It resolves the limitation of the MEMS Studio demo firmware and software, which typically requires manual sensor configuration after every reboot.

Sensor data packets are streamed directly through the virtual COM port of the STEVAL-MKI109V3 via USB.

---

## Getting Started

### Prerequisites

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)

### Setting Up the Project

1. **Download and install STM32CubeIDE** from the [official ST website](https://www.st.com/en/development-tools/stm32cubeide.html).

2. **Import the project**:
    - Launch STM32CubeIDE.
    - Go to `File` → `Import`.
    - Select `Existing Projects into Workspace` and browse to select your project directory.

3. **Update STM32Cube device database**:
    - Navigate to `Help` → `STM32Cube Updates` → `Connection to MyST`.
    - Log in with your MyST account.
    - Go to `Help` → `STM32Cube Updates` → `Check for target selector device database updates` and apply necessary updates.

4. **Build the project**:
    - Press `Ctrl + B` to build the binary file.

---

## Flashing Firmware

Use STM32CubeProgrammer to flash the firmware to your STEVAL-MKI109V3 board:

1. Connect your board via USB and enter DFU mode:
    - Refer to the [STEVAL-MKI109V3 User Manual](https://www.st.com/resource/en/user_manual/dm00531631-stevalmki109v3-mems-sensors-evaluation-kit-for-motion-sensors-stmicroelectronics.pdf) to understand how to enter DFU mode.

2. Open STM32CubeProgrammer and select the DFU mode from the interface options.

3. Browse and select the generated `.bin` file from your project's build output directory.

4. Click `Download` to flash the firmware.

---

## How It Works

Once flashed and rebooted, the device will automatically stream binary sensor data packets via the virtual COM port in the following format:

```c
typedef struct {
    uint8_t start;
    uint8_t type;
    uint32_t timestamp;
    float_t data[3];  // X, Y, Z for accel/gyro; [0] for temp
    uint8_t checksum;
} __attribute__((packed)) Packet;
```

A provided Python script decodes these packets, outputs them to the terminal, and allows further analysis.

---

## Limitations

- The configuration settings for the ASM330LHH IMU are hardcoded in `main.c`. If adjustments like measurement range or resolution are needed, the firmware must be recompiled and reflashed.

