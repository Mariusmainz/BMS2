# STM32F302 Battery Mangement System

This project reads ADC inputs (battery and temp) and sends formatted data over UART.

## Requirements
- STM32CubeIDE
- STM32F302R8
- USB or UART terminal for output
- Hardware setup as described in the report

## How to Build
1. Clone this repo
2. Open STM32CubeIDE
3. File > Import > Existing STM32CubeMX Project
4. Build & flash to device

## Code Structure 
An overview of the most relevant functions:

### `read_adc_values()`

- Reads analog voltage from two ADC channels:
  - `ADC1_IN1`: LM35 temperature sensor
  - `ADC1_IN5`: Battery voltage via voltage divider
- Converts raw ADC values into millivolt readings
- Calculates:
  - Temperature in degrees Celsius
  - Battery percentage using a linear approximation
- Returns the temperature value for threshold evaluation

---

### `scan_adc_channels()`

- Manually switches between ADC channels
- Starts and polls the ADC for each input sequentially
- Stores results in `adcBuffer[2]`:
  - `adcBuffer[0]`: Temperature
  - `adcBuffer[1]`: Battery voltage

---

### `enter_stop_mode()`

- Simulates system shutdown due to high temperature:
  - Suspends SysTick timer
  - Enters STOP mode using `HAL_PWR_EnterSTOPMode()`
  - Resumes tick and reinitializes clocks on wake-up
- Wake-up triggered by button or comparator interrupt

---

### `exit_stop_mode()`

- Called after a wake-up event
- Resumes system tick and clock configuration if necessary
- Used by both GPIO and comparator interrupt callbacks

---

### `COMP4_6_IRQHandler()`

- Interrupt handler for comparator (COMP4)
- Triggered when LM35 voltage drops below the predefined threshold
- Sends UART message to indicate wake-up from STOP mode
- Clears comparator interrupt flags

---

### `HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)`

- Handles external interrupts (e.g., B1 user button)
- If the interrupt source is `B1_Pin`, sets:
  - `wake_from_button = 1`
  - `wake_flag = 1`
  - Calls `exit_stop_mode()` to resume normal operation

---

### `HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)`

- Handles comparator-triggered wake-up
- If triggered by `COMP2`, sets:
  - `wake_from_temp = 1`
  - `wake_flag = 1`
  - Calls `exit_stop_mode()` to resume normal operation

---

### Main `while(1)` Loop

- Periodically:
  - Reads sensor data
  - Evaluates temperature threshold
  - Starts or stops comparator as needed
  - Enters STOP mode if temperature remains high long enough
- Handles wake-up logic and resets flags accordingly
