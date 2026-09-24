# Arquitectura del sistema

## 1. Resumen

La arquitectura real del proyecto se basa en **Arduino UNO R4 WiFi** como controlador principal, con GNSS por `Serial1`, IMU BNO085 por `Wire` y salida NMEA para Dynatest por `Serial`/USB CDC cuando se necesita un flujo limpio.

Bloques principales:

1. Receptor GNSS (UM980 / simpleRTK3B Budget)
2. Arduino UNO R4 WiFi (RA4M1)
3. IMU BNO085/BNO086
4. LEDs externos de estado
5. Interfaz de salida Dynatest sobre USB CDC

## 2. Diagrama lógico

```text
simpleRTK3B Budget / UM980
        │
        └─ NMEA GGA/RMC + #PPPNAVA ──► Serial1 (D0/D1) ──► Arduino UNO R4 WiFi (RA4M1)
                                                      │
                                                      ├─ Wire @ 100 kHz ──► BNO085 remoto
                                                      ├─ GPIO D6/D7 ──────► LED1 / LED2
                                                      └─ Serial (USB CDC) ─► Dynatest / host NMEA limpio
```

## 3. Interfaz de datos

### 3.1 GNSS ↔ UNO R4 WiFi

- Enlace: `Serial1`
- Pines: `D0/RX`, `D1/TX`
- Baudrate: `115200`
- Entrada al firmware:
  - GGA (`$GPGGA`, `$GNGGA`, `$GCGGA`)
  - RMC (`$GPRMC`, `$GNRMC`)
  - PPP/HAS (`#PPPNAVA`)

### 3.2 UNO R4 WiFi ↔ Dynatest

- Enlace base implementado: `Serial` / USB CDC
- Sentencia emitida: `$GCGGA`
- Tasa: `10 Hz`
- Checksum: XOR
- Terminación: `CRLF`

#### Limitación física

El UNO R4 WiFi no dispone de un `Serial2` equivalente al usado por el firmware histórico ESP32. Por ello:

- `Serial1` queda reservado para GNSS;
- `Serial` puede llevar diagnóstico **o** NMEA limpio, no ambos a la vez;
- para un segundo enlace físico dedicado a Dynatest se requiere hardware externo adicional.

### 3.3 UNO R4 WiFi ↔ BNO085

- Enlace: `Wire`
- Velocidad: `100 kHz`
- Bus elegido: el I2C principal expuesto en los headers `SDA/SCL`, adecuado para el arnés RJ45/UTP remoto.

## 4. Máquina de estados

### MOVING

- Se ingiere GNSS continuo.
- Se mantiene salida `$GCGGA` a 10 Hz si la GGA está fresca.
- Se aplica offset instantáneo solo si hay yaw válido.

### AVERAGING

- Se confirma parada tras 2 s bajo `0.20 m/s`.
- Se acumulan muestras durante 15 s.
- Se almacenan latitud, longitud, altitud y yaw cuando existe.

### LOCKED

- Se publica la coordenada promediada.
- Se mantiene la salida a 10 Hz mientras la GGA siga fresca.
- Se abandona el estado por velocidad `> 0.30 m/s` o por desplazamiento `> 1 m` comparando coordenadas del mismo marco (`raw` con `raw`).

## 5. Calidad GNSS y PPP/HAS

- `LOCKED` válido → prioridad máxima para la salida.
- `PPP_ESTABLE` → fixQ de salida preferente.
- `PPP_CONVERGING` → fixQ intermedio.
- autónomo válido → fixQ básico.

Para evitar falsos positivos, el parser PPP/HAS solo procesa líneas que **empiezan por `#PPPNAVA`**.

## 6. HDOP y frescura

- El HDOP se copia desde el campo 8 de la GGA recibida.
- Si el campo falta o es inválido, se usa el fallback documentado `1.0`.
- Si la GGA deja de estar fresca, se silencia la salida.

## 7. LEDS de estado

### LED1 (D6)

- OFF: sin GGA fresca
- Parpadeo: PPP convergiendo
- ON fijo: GNSS válido / PPP estable

### LED2 (D7)

- OFF: `MOVING`
- Parpadeo: `AVERAGING`
- ON fijo: `LOCKED`

## 8. Cableado RJ45 (módulo remoto)

Etiqueta obligatoria:

- **`BNO085/LED — NO ETHERNET`**

Pinout:

- Pin 1: `+5V` solo a `VIN/5V` del breakout Adafruit
- Pin 2: `GND`
- Pin 3: `SDA`
- Pin 4: `LED1`
- Pin 5: `LED2`
- Pin 6: `GND`
- Pin 7: `SCL`
- Pin 8: `GND`

Buenas prácticas:

- UTP directo pin‑a‑pin
- pares trenzados cuando sea práctico
- separado del cableado de bomba/motor
- cruces a ~90°
- desacoplo local `100 nF + 10–100 µF`
- prueba de continuidad antes de energizar
- nunca Ethernet / nunca PoE

## 9. Changelog Rev.1 UNO R4 WiFi

- Se migra la Rev.1 al hardware real Arduino UNO R4 WiFi.
- Se sustituyen referencias activas a ESP32-S3 como MCU principal por la arquitectura RA4M1 real.
- Se elimina el uso de `Serial2` y de la firma ESP32 `Wire1.begin(sda, scl, freq)`.
- Se fija la salida a `$GCGGA` a 10 Hz con HDOP real, checksum y timeout de frescura.
