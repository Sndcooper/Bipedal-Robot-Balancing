# Wireless Radio Latency Diagnostic Suite (`mcu_ik_engine_pretest_wireless`)

This directory provides a **Pure Wireless Radio Diagnostic and Latency Benchmarking Environment**. It isolates the performance of the **3DR Telemetry Radio (433MHz/915MHz)** connected to `Serial3` by stripping away IMU polling, encoder interrupts, and AX-12 bus transactions.

---

## 🎯 Benchmark Objectives

1. **Round-Trip Delay (RTT):** Measures round-trip ping/pong latency between the PC GUI and STM32.
2. **Packet Drop Rate:** Tracks sequence numbers (`S:<seq>`) to calculate dropped or corrupted frames.
3. **Loop Time Integrity:** Reports microsecond execution time (`DT`) and body calculation time (`BD`) to verify real-time 100 Hz scheduling.

---

## ⚡ Hardware Connections

```
                    ┌───────────────────────────┐
                    │   STM32F103C8T6 BLUEPILL  │
                    ├───────────────────────────┤
  3DR Radio TX      │ PB10                 GND  │ Common Ground
  3DR Radio RX      │ PB11                 3.3V │ 3.3V Power
                    └───────────────────────────┘
```

| Component | STM32 Pin | Function | Settings |
| :--- | :---: | :--- | :--- |
| **3DR Radio TX** | `PB10` | USART3 TX | 115,200 Baud |
| **3DR Radio RX** | `PB11` | USART3 RX | 115,200 Baud |

---

## 📡 Protocol Specification

### 1. Telemetry Outbound Packet (20 Hz)
```text
S:<seq>,T:<timestamp_us>,P:<synth_val>,DT:<loop_us>,BD:<body_us>
```

### 2. Ping-Pong Benchmark
* **PC Request:** `PING:<token>`
* **MCU Response:** `PONG:<token>`

---

## 📂 File Map

* **[`firmware/src/main.cpp`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine_pretest_wireless/firmware/src/main.cpp)**: Diagnostic firmware logic with lightweight non-blocking RX parser (40 µs deadline per tick).
* **[`latency_test.py`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine_pretest_wireless/latency_test.py)**: Standalone Python diagnostic tool that pings the microcontroller, measures latency percentiles, and reports telemetry throughput.
* **[`gui/main_gui.py`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine_pretest_wireless/gui/main_gui.py)**: GUI latency display interface.

---

## 🚀 Running the Diagnostic

1. Upload pretest firmware:
   ```bash
   cd firmware
   pio run -t upload
   ```
2. Run automated latency diagnostic script:
   ```bash
   python latency_test.py --port COM3 --baud 115200
   ```
