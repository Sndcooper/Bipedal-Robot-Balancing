# AX-12+ Servo Control Table & Buffer Mapping

This document provides a clean, comprehensive reference for the Dynamixel **AX-12+ Control Table** and its corresponding buffer offset mapping when reading the complete table (`Address 0`, `Length 50`) in STM32 firmware.

---

## 📡 Packet Buffer Structure

When requesting 50 bytes starting from **Address 0** (`READ_DATA 0x02`), the incoming UART response packet layout is structured as follows:

| Byte Range | Field | Value / Description |
| :--- | :--- | :--- |
| `buf[0]` - `buf[1]` | **Header** | `0xFF`, `0xFF` |
| `buf[2]` | **Servo ID** | Target Servo ID (`0`, `1`, `6`, `14`, etc.) |
| `buf[3]` | **Packet Length** | `52` (Data length + Error byte + Checksum) |
| `buf[4]` | **Error Byte** | Bit flags for Overheating, Overload, Voltage, etc. |
| `buf[5]` to `buf[54]` | **Data Payload** | **Control Table Addresses `0` through `49`** |
| `buf[55]` | **Checksum** | `~(ID + Length + Error + Data...) & 0xFF` |

> [!NOTE]
> **Buffer Index Formula**: For any Control Table `Address`, its byte location in the response buffer is:
> $$\text{Buffer Index} = \text{buf}[5 + \text{Address}]$$

---

## 💾 EEPROM Area (Persistent Settings: Addr 0–23)

EEPROM values persist across power cycles. Values marked **Word** are 2 bytes (Low Byte first, High Byte second).

| Address | Parameter | Size | Access | Buffer Index (`buf[...]`) | Default / Scale | Description & Notes |
| :---: | :--- | :---: | :---: | :---: | :---: | :--- |
| **0–1** | **Model Number** | 2B (Word) | Read | `buf[5]` (L), `buf[6]` (H) | `12` (`0x000C`) | Fixed model identifier for AX-12+ |
| **2** | **Firmware Version** | 1B | Read | `buf[7]` | — | Internal firmware version of the servo |
| **3** | **Servo ID** | 1B | R/W | `buf[8]` | `1`–`252` | Bus ID of the servo (`6`, `0`, `14`, `1` in biped) |
| **4** | **Baud Rate** | 1B | R/W | `buf[9]` | `1` ($1\text{ Mbps}$) | Speed setting formula: $\text{Baud} = \frac{2000000}{\text{Value} + 1}$ |
| **5** | **Return Delay Time** | 1B | R/W | `buf[10]` | $2 \mu\text{s}$ per unit | Delay before sending response packet (0 for ultra-fast) |
| **6–7** | **CW Angle Limit** | 2B (Word) | R/W | `buf[11]` (L), `buf[12]` (H) | `0` ($0^\circ$) | Clockwise physical minimum angle limit |
| **8–9** | **CCW Angle Limit** | 2B (Word) | R/W | `buf[13]` (L), `buf[14]` (H) | `1023` ($300^\circ$) | Counter-Clockwise physical maximum angle limit |
| **11** | **Max Temperature Limit** | 1B | R/W | `buf[16]` | $70^\circ\text{C}$ | Shutdown temperature threshold |
| **12** | **Min Voltage Limit** | 1B | R/W | `buf[17]` | $60$ ($6.0\text{V}$) | Lower supply voltage error threshold ($0.1\text{V}$ steps) |
| **13** | **Max Voltage Limit** | 1B | R/W | `buf[18]` | $140$ ($14.0\text{V}$) | Upper supply voltage error threshold ($0.1\text{V}$ steps) |
| **14–15** | **Max Torque** | 2B (Word) | R/W | `buf[19]` (L), `buf[20]` (H) | `1023` ($100\%$) | Maximum torque ceiling limit |
| **16** | **Status Return Level** | 1B | R/W | `buf[21]` | `2` | `0` = No response, `1` = Read only, `2` = Respond to all |
| **17** | **Alarm LED** | 1B | R/W | `buf[22]` | `36` | Bitmask of error conditions that flash the LED |
| **18** | **Alarm Shutdown** | 1B | R/W | `buf[23]` | `36` | Bitmask of error conditions that disable motor torque |

---

## ⚡ RAM Area (Dynamic Control & Telemetry: Addr 24–49)

RAM values reset to defaults on reboot. Used for real-time control, compliance tuning, and sensor telemetry.

| Address | Parameter | Size | Access | Buffer Index (`buf[...]`) | Units / Resolution | Description & Notes |
| :---: | :--- | :---: | :---: | :---: | :---: | :--- |
| **24** | **Torque Enable** | 1B | R/W | `buf[29]` | `0` or `1` | `1` = Motor powered & locked, `0` = Free-wheeling |
| **25** | **LED Status** | 1B | R/W | `buf[30]` | `0` or `1` | `1` = LED On, `0` = LED Off |
| **26** | **CW Compliance Margin** | 1B | R/W | `buf[31]` | $0.29^\circ$ steps | Deadzone margin for CW position error |
| **27** | **CCW Compliance Margin**| 1B | R/W | `buf[32]` | $0.29^\circ$ steps | Deadzone margin for CCW position error |
| **28** | **CW Compliance Slope** | 1B | R/W | `buf[33]` | Level 1–7 | Flexibility slope for CW direction correction |
| **29** | **CCW Compliance Slope**| 1B | R/W | `buf[34]` | Level 1–7 | Flexibility slope for CCW direction correction |
| **30–31** | **Goal Position** | 2B (Word) | R/W | `buf[35]` (L), `buf[36]` (H) | `0`–`1023` ($0^\circ$–$300^\circ$) | Target position input |
| **32–33** | **Moving Speed** | 2B (Word) | R/W | `buf[37]` (L), `buf[38]` (H) | $0.111\text{ RPM}$ steps | Target rotation velocity limit (`0` = max speed) |
| **34–35** | **Torque Limit** | 2B (Word) | R/W | `buf[39]` (L), `buf[40]` (H) | `0`–`1023` ($0\%$-$100\%$) | Active maximum torque ceiling |
| **36–37** | **Present Position** | 2B (Word) | Read | `buf[41]` (L), `buf[42]` (H) | `0`–`1023` ($0.29^\circ$/unit) | Current physical shaft angle |
| **38–39** | **Present Speed** | 2B (Word) | Read | `buf[43]` (L), `buf[44]` (H) | $0.111\text{ RPM}$ steps | Current shaft velocity (`Bit 10` = Direction) |
| **40–41** | **Present Load** | 2B (Word) | Read | `buf[45]` (L), `buf[46]` (H) | `0`–`1023` (`Bit 10` = Dir) | Estimated motor load / applied torque magnitude |
| **42** | **Present Voltage** | 1B | Read | `buf[47]` | $0.1\text{V}$ steps | Supply voltage ($\text{Volts} = \text{Value} \times 0.1$) |
| **43** | **Present Temperature**| 1B | Read | `buf[48]` | $1^\circ\text{C}$ steps | Internal motor temperature in Celsius |
| **44** | **Registered Instruction**| 1B | Read | `buf[49]` | `0` or `1` | `1` if waiting for an `ACTION` execution packet |
| **46** | **Moving** | 1B | Read | `buf[51]` | `0` or `1` | `1` = Servo currently moving, `0` = In goal position |
| **47** | **Lock** | 1B | R/W | `buf[52]` | `0` or `1` | `1` = Locks EEPROM area (Addr 0–18) until power cycle |
| **48–49** | **Punch** | 2B (Word) | R/W | `buf[53]` (L), `buf[54]` (H) | `32`–`1023` | Minimum initial current threshold applied to motor |

---

## 💻 C++ Code Parsing Example (STM32 Arduino)

```cpp
// Extracting telemetry after validating buf[0] == 0xFF && buf[1] == 0xFF:
uint8_t  servoId       = buf[2];
uint8_t  errorFlags    = buf[4];

// EEPROM Data (Addr 0..23)
uint16_t modelNumber   = buf[5 + 0]  | (buf[5 + 1] << 8);  // buf[5], buf[6]
uint8_t  firmwareVer   = buf[5 + 2];                       // buf[7]
uint16_t cwAngleLimit  = buf[5 + 6]  | (buf[5 + 7] << 8);  // buf[11], buf[12]
uint16_t ccwAngleLimit = buf[5 + 8]  | (buf[5 + 9] << 8);  // buf[13], buf[14]

// Dynamic RAM & Telemetry (Addr 24..49)
uint8_t  torqueEnabled = buf[5 + 24];                      // buf[29]
uint16_t goalPosition  = buf[5 + 30] | (buf[5 + 31] << 8); // buf[35], buf[36]
uint16_t torqueLimit   = buf[5 + 34] | (buf[5 + 35] << 8); // buf[39], buf[40]
uint16_t presPosition  = buf[5 + 36] | (buf[5 + 37] << 8); // buf[41], buf[42]
uint16_t presSpeed     = buf[5 + 38] | (buf[5 + 39] << 8); // buf[43], buf[44]
uint16_t presLoad      = buf[5 + 40] | (buf[5 + 41] << 8); // buf[45], buf[46]
float    presVoltage   = buf[5 + 42] * 0.1f;               // buf[47]
uint8_t  presTemp      = buf[5 + 48 - 5 + 5];              // buf[48] (Addr 43)
uint8_t  isMoving      = buf[5 + 46];                      // buf[51]
```
