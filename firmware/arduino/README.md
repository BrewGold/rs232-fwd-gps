# Firmware Arduino

## Firmware objetivo real

- `rs232_fwd_gps_uno_r4_wifi_rev_1_0.ino` — **Rev.1.0 para Arduino UNO R4 WiFi**

Este es el sketch que corresponde al hardware real del proyecto: **Arduino UNO R4 WiFi** programado desde Arduino IDE.

## Sketches históricos conservados

Los siguientes archivos se mantienen solo como referencia histórica y **son incompatibles con el despliegue real UNO R4 WiFi** porque dependen de APIs/pines de ESP32-S3:

- `rs232_fwd_gps_final_v_0_99.ino`
- `rs232_fwd_gps_draft_v0_9.ino`

## Arquitectura real del UNO R4 WiFi

- **`Serial1`**: GNSS UM980 en **D0/RX** y **D1/TX** a **115200**.
- **`Serial`**: USB CDC para diagnóstico o para salida Dynatest NMEA limpia.
- **`Wire`**: I2C principal en **SDA/SCL** con `Wire.begin()` y `Wire.setClock(100000)`.
- **LED1 / LED2**: por defecto **D6 / D7**, evitando UART, I2C y CAN.

## Limitación serial real

El UNO R4 WiFi no ofrece un `Serial2` equivalente al del firmware histórico ESP32. La Rev.1 usa:

- GNSS por `Serial1`.
- Dynatest por `Serial`/USB CDC cuando se requiere una salida NMEA limpia.

Eso implica que el mismo puerto USB CDC **no puede** usarse al mismo tiempo para:

1. logs de diagnóstico, y
2. `$GCGGA` limpia hacia el enlace Dynatest.

Si se necesitan dos enlaces físicos simultáneos sin cambiar de modo, será necesario hardware adicional externo.

## Funcionalidad Rev.1

- checksum XOR para GGA/RMC de entrada;
- aceptación de `$GPGGA`, `$GNGGA`, `$GCGGA`, `$GPRMC` y `$GNRMC`;
- salida **`$GCGGA`** a **10 Hz** con **CRLF**;
- propagación del **HDOP real** del campo 8;
- silencio de salida cuando la GGA deja de estar fresca;
- máquina **MOVING → AVERAGING → LOCKED**;
- confirmación de parada de **2 s**;
- promedio de **15 s**;
- detección de salida de lock a **1 m** comparando siempre coordenadas del mismo marco;
- yaw BNO085 + offset antena‑pistón de **0.55 m** con bearing **yaw + 270°**;
- declinación de **1°** como parámetro configurable;
- parser PPP/HAS estricto: solo líneas que empiezan por **`#PPPNAVA`**.

## Arnés RJ45/UTP remoto

Etiqueta fija: **`BNO085/LED — NO ETHERNET`**

- Pin 1 → `+5V` solo a `VIN/5V` del breakout Adafruit
- Pin 2 → `GND`
- Pin 3 → `SDA`
- Pin 4 → `LED1`
- Pin 5 → `LED2`
- Pin 6 → `GND`
- Pin 7 → `SCL`
- Pin 8 → `GND`

## Compilación esperada

Sketch objetivo para Arduino IDE / `arduino-cli`:

- **Board:** `arduino:renesas_uno:unor4wifi`
- **Archivo:** `firmware/arduino/rs232_fwd_gps_uno_r4_wifi_rev_1_0.ino`
