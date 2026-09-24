# RS232-FWD-GPS

Firmware y documentación para el proyecto **RS232 Forward GPS** orientado al hardware real del despliegue: **Arduino UNO R4 WiFi** programado desde **Arduino IDE**.

## Plataforma real soportada

- **MCU principal:** Renesas RA4M1 del Arduino UNO R4 WiFi.
- **ESP32-S3 integrado:** solo coprocesador de conectividad Wi‑Fi/Bluetooth; **no** es el objetivo del firmware principal.
- **IDE objetivo:** Arduino IDE con el core `arduino:renesas_uno`.

## Función del sistema

El sistema recibe GNSS desde un **simpleRTK3B Budget / UM980**, detecta movimiento/parada, promedia 15 s cuando el equipo está detenido y genera una salida **`$GCGGA` a 10 Hz** compatible con Dynatest mientras la GGA siga siendo válida y fresca.

## Enrutamiento serie real en UNO R4 WiFi

- **`Serial1` (D0/D1, 115200):** entrada GNSS real desde el UM980.
- **`Serial` (USB CDC):**
  - modo diagnóstico por USB, o
  - modo salida Dynatest limpia en NMEA.

### Limitación importante

El UNO R4 WiFi solo dispone de una UART hardware externa principal (`Serial1`) además del USB CDC. Por tanto:

- **no existe `Serial2`** para crear un segundo enlace UART físico como en el firmware histórico ESP32;
- si `Serial`/USB CDC se usa como enlace Dynatest, **no debe usarse simultáneamente para logs**;
- para disponer de dos enlaces físicos simultáneos (GNSS + salida serial dedicada a Dynatest) hace falta **hardware externo adicional** como un convertidor USB‑serial gestionado fuera del UNO R4 o una interfaz de red/serial externa.

> En este repositorio no hay una configuración previa de Ethernet Shield 2, así que la Rev.1 implementada prioriza la salida limpia por USB CDC y documenta la limitación real del hardware.

## Firmware incluido

- **Actual / objetivo real:** `firmware/arduino/rs232_fwd_gps_uno_r4_wifi_rev_1_0.ino`
- **Histórico ESP32-S3 (incompatible con el despliegue real UNO R4 WiFi):**
  - `firmware/arduino/rs232_fwd_gps_final_v_0_99.ino`
  - `firmware/arduino/rs232_fwd_gps_draft_v0_9.ino`

## Rev.1 UNO R4 WiFi

La Rev.1 mantiene y aclara estas reglas funcionales:

- parseo NMEA con checksum XOR para **GGA/RMC**;
- aceptación de **`$GPGGA`, `$GNGGA`, `$GCGGA`, `$GPRMC` y `$GNRMC`**;
- generación de **`$GCGGA`** con terminación **CRLF**;
- propagación del **HDOP real** del campo 8 de la GGA de entrada;
- uso de fallback documentado solo si el HDOP no llega o es inválido;
- silencio de salida cuando la GGA supera el timeout de frescura;
- máquina de estados **MOVING → AVERAGING → LOCKED**;
- confirmación de parada de **2 s**;
- promedio de **15 s**;
- umbrales **0.20 / 0.30 m/s** y distancia de salida de lock de **1 m**;
- comparación siempre en el mismo marco de coordenadas (**raw con raw** para detectar salida de `LOCKED`);
- yaw del **BNO085**, offset antena‑pistón de **0.55 m** con bearing **yaw + 270°** y declinación **1°** como constantes configurables;
- detección PPP/HAS limitada a mensajes que **empiezan por `#PPPNAVA`**.

## I2C y BNO085

La Rev.1 usa el bus **`Wire`** del UNO R4 WiFi con:

- `Wire.begin()`
- `Wire.setClock(100000)`

Se documenta `Wire` porque es el bus I2C principal disponible en los headers **SDA/SCL (A4/A5)**, adecuado para el arnés RJ45/UTP del módulo remoto. No se usa la firma ESP32 `Wire1.begin(sda, scl, freq)`.

## LEDs externos elegidos

Para evitar conflicto con UART/I2C, la Rev.1 usa por defecto:

- **LED1:** `D6`
- **LED2:** `D7`

Estas constantes pueden ajustarse si el cableado final cambia.

## Arnés RJ45/UTP remoto

**Etiqueta obligatoria:** `BNO085/LED — NO ETHERNET`

Pinout documentado:

1. **+5 V** solo a `VIN/5V` del breakout Adafruit, nunca a `3V3` ni al IC
2. **GND**
3. **SDA**
4. **LED1**
5. **LED2**
6. **GND**
7. **SCL**
8. **GND**

Buenas prácticas:

- cable UTP directo pin‑a‑pin;
- usar pares trenzados cuando sea práctico;
- separar el arnés del cableado de bomba/motor;
- hacer cruces a ~90° cuando no se pueda evitar;
- desacoplo local **100 nF + 10–100 µF** cerca del breakout;
- prueba de continuidad antes de energizar;
- **nunca** conectar este RJ45 a Ethernet ni a PoE.

## Changelog Rev.1 UNO R4 WiFi

- Se añade `firmware/arduino/rs232_fwd_gps_uno_r4_wifi_rev_1_0.ino` como firmware real para UNO R4 WiFi.
- Se elimina el uso de APIs y pines exclusivos de ESP32 (`Serial2`, `Wire1.begin(sda,scl,freq)`, GPIO 43/44/17/18).
- Se cambia la salida generada de `$GPGGA` a `$GCGGA`.
- Se conserva el firmware ESP32 histórico solo como referencia incompatible.
- Se documenta la limitación real de seriales del UNO R4 WiFi y la necesidad de hardware externo para dos enlaces físicos simultáneos.

## Checklist de aceptación Rev.1

- [ ] `$GCGGA` a 10 Hz en movimiento
- [ ] `$GCGGA` a 10 Hz en parada (`LOCKED`)
- [ ] HDOP variable propagado desde la GGA de entrada
- [ ] Checksum XOR y terminación CRLF correctos
- [ ] Silencio de salida al vencer el timeout de frescura
- [ ] `Wire` a 100 kHz en UNO R4 WiFi
- [ ] `Serial1` en D0/D1 para GNSS
- [ ] LEDs en pines válidos del UNO R4 (`D6`/`D7` por defecto)
- [ ] Arnés RJ45 seguro y rotulado `NO ETHERNET`
- [ ] Prueba con bomba apagada y encendida sin degradación del enlace I2C
