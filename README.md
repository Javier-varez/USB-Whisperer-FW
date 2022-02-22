
# USB Whisperer

Tool to debug and access various internal buses in Apple Silicon Macs.

This repository contains the firmware for the prototype version. Currently uses a Makerdiary 
nRF52840 M.2 Developer Kit for the prototype. The MCU might change but firmware is written to be 
mostly compatible with any cortex-m MCU as long as it has an i2c bus and a SysTick timer.

# Connections

  * `FUSB302 VDD` -> `3.3V supply`
  * `FUSB302 I2C SDA` -> `D15/P1.06`
  * `FUSB302 I2C SCL` -> `D14/P1.05`
  * `FUSB302 IRQ_N` -> `D2/P0.19`
  * `Vbus_on` signal -> `D3/P0.20`. This should drive a 5V regulator enable signal to toggle VBUS.
