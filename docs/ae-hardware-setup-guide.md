# Hardware Connection Guide

This document describes how to connect the **Saleae Logic Analyzer (LA)** and the **STLink-V3PWR** to each supported NUCLEO board for power and signal measurements.  
We assume you have the required hardware (Saleae Logic Pro or compatible, STLink-V3PWR, jumper wires).

---

## Common Setup

- **Logic Analyzer (LA):**
  - Connect to the **Arduino headers** on each NUCLEO board.
  - `D7` → LA Channel 0 (`TRIGGER`)
  - `D4` → LA Channel 1 (`LATENCY`)

- **STLink-V3PWR:**
  - Used to power the board and capture current consumption.
  - Connection header varies by board (see below).

- **USB connection:**  
  Each board should still be connected to the host PC via USB for flashing/debugging.

⚠️ **Important:** Use the STLink-V3PWR as the *only* power source. Do not power the board simultaneously via USB 5V.

---

## 1. NUCLEO-STM32G474

- **Power:** Connect STLink-V3PWR to the **3V3 pin** on the Arduino header.  
- **Signals:**  
  - D7 → LA Channel 0 (`TRIGGER`)  
  - D4 → LA Channel 1 (`LATENCY`)  

![STM32G474 Wiring](stm32g474-wiring.png)

---

## 2. NUCLEO-STM32H7A3ZIQ

- **Power:** Connect STLink-V3PWR to the **3V3 pin**.  
- **Signals:**  
  - D7 → LA Channel 0 (`TRIGGER`)  
  - D4 → LA Channel 1 (`LATENCY`)  

![STM32H7A3ZIQ Wiring](stm32h7a3ziq-wiring.png)

---

## 3. NUCLEO-STM32U575ZIQ

- **Power:** Connect STLink-V3PWR to the **IOREF pin** on the Arduino header.  
- **Signals:**  
  - D7 → LA Channel 0 (`TRIGGER`)  
  - D4 → LA Channel 1 (`LATENCY`)  

![STM32U575ZIQ Wiring](stm32u575ziq-wiring.png)

---

## 4. NUCLEO-STM32C092RC

- **Power:** Connect STLink-V3PWR to the **3V3 pin**.  
- **Signals:**  
  - D7 → LA Channel 0 (`TRIGGER`)  
  - D4 → LA Channel 1 (`LATENCY`)  
- **Board Modification:** SB (solder bridge) must be removed to enable external power through the 3V3 pin.  

![STM32C092RC Wiring](stm32c092rc-wiring.png)

---

## Summary Table

| Board                | Power Pin | Trigger (D7) | Latency (D4) | Notes                               |
|----------------------|-----------|--------------|--------------|-------------------------------------|
| STM32G474            | 3V3       | LA Ch0       | LA Ch1       |                                     |
| STM32H7A3ZIQ         | 3V3       | LA Ch0       | LA Ch1       |                                     |
| STM32U575ZIQ         | IOREF     | LA Ch0       | LA Ch1       |                                     |
| STM32C092RC          | 3V3       | LA Ch0       | LA Ch1       | Requires SB removal for 3V3 input   |

---

## Verification

1. In CubeMonitor-Power, verify that the board draws stable idle current after connecting.  
2. In Logic 2, confirm that `TRIGGER` toggles when a benchmark runs.  
3. Check that `LATENCY` pulses are visible during benchmark execution.
