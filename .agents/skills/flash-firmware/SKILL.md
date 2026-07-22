---
name: flash-firmware
description: Compiles and uploads the STM32 balancing bipedal robot firmware (main.cpp) using PlatformIO, including environment verification, bootloader setup, and COM port handling.
---

# Flash & Upload STM32 Firmware (`flash-firmware`)

When the user asks to **upload**, **flash**, **build**, or **deploy** `main.cpp` or the robot firmware, follow these instructions to ensure the PlatformIO environment (`env:bluepill_f103c8`) is properly configured and the firmware is uploaded cleanly to the STM32 Bluepill (`STM32F103C8`).

## 1. Identify Target Firmware Directory
By default, target the **active safety rework firmware** unless the user explicitly requests the legacy firmware:
- **Active Rework Firmware (Default)**: `Balance_Rework/firmware`
  - Main code: `Balance_Rework/firmware/src/main.cpp`
  - Config: `Balance_Rework/firmware/platformio.ini`
- **Legacy PlatformIO Firmware**: `PlatformIO_Firmware`
  - Main code: `PlatformIO_Firmware/src/main.cpp`
  - Config: `PlatformIO_Firmware/platformio.ini`

## 2. Verify `platformio.ini` Environment Configuration
Ensure the target directory's `platformio.ini` contains the required environment configuration for the STM32 Bluepill over Serial (USB-TTL adapter):

```ini
[env:bluepill_f103c8]
platform = ststm32
board = bluepill_f103c8
framework = arduino
upload_protocol = serial
monitor_speed = 115200
board_build.f_cpu = 72000000L
build_flags = 
    -DHAL_UART_MODULE_ENABLED
    -DSERIAL_UART_INSTANCE=1
```

## 3. Hardware Preparation Check (Serial Bootloader Mode)
Before running the upload command, remind or verify with the user:
1. **BOOT0 Jumper**: To upload via `upload_protocol = serial` (USB-TTL adapter on `PA9`/`PA10`), the STM32's **BOOT0 jumper must be set to 1 (HIGH)** and **BOOT1 set to 0 (LOW)**. Press the **Reset** button on the STM32 after moving the jumper so it enters the serial bootloader.
2. **Serial Connection**: Ensure USB-TTL adapter is connected (`TX -> PA10`, `RX -> PA9`, `GND -> GND`).
3. **COM Port Check**: Ensure no serial monitor or GUI (like `mpu_inspector_gui.py` or web serial dashboard) is actively holding the COM port open.

## 4. Execution Commands (PlatformIO CLI)
To compile and upload the firmware, execute the following PowerShell commands from the workspace root:

### Check Connected Serial Devices / COM Ports
```powershell
pio device list
```

### Build / Compile Only (Verify `main.cpp` without uploading)
```powershell
pio run -d "Balance_Rework\firmware"
```

### Build & Upload Firmware
```powershell
pio run -d "Balance_Rework\firmware" -t upload
```

*(If a specific COM port needs to be targeted, append `--upload-port COMx`, e.g.: `pio run -d "Balance_Rework\firmware" -t upload --upload-port COM3`)*

## 5. Post-Upload Return to Run Mode
After a successful upload:
1. Instruct the user to move the **BOOT0 jumper back to 0 (LOW)** on the STM32.
2. Press the **Reset** button on the STM32 (or power cycle) to boot into the newly flashed firmware (`main.cpp`).
3. Verify that the 2-second startup delay completes and the AX-12+ servos lock into their standing 90-degree position.
