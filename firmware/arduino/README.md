# Firmware Arduino

## Versiones

- `rs232_fwd_gps_final_v_0_99.ino` — **v0.9.9 MADRID FINAL** (histórico, conservado sin cambios)
- `rs232_fwd_gps_rev_1_0.ino` — **Rev.1** (sketch actual)

La v0.9.9 se mantiene como referencia de campo. Rev.1 parte de ese código y conserva la arquitectura ESP32-S3 + UM980 + MAX3232 + BNO085 + LEDs, con cambios puntuales documentados abajo.

### Hardware

- GNSS (UM980): RX=44, TX=43, BAUD=115200 (`Serial1`)
- Salida Dynatest: RX=18, TX=17, BAUD=38400 (`Serial2` → MAX3232)
- IMU (BNO085): SDA=8, SCL=9 (`Wire1`), I2C 100 kHz, dirección `0x4B`
- LED_RED: pin 4 (estado GNSS/HAS)
- LED_GREEN: pin 5 (estado de movimiento/lock)

### Máquina de estados

**Movimiento:**
`MOVING → AVERAGING` (velocidad < 0.20 m/s durante 2 s) `→ LOCKED` (promedio de 15 s) `→ MOVING` (si se desplaza > 1.0 m o velocidad > 0.30 m/s)

**PPP/HAS:**
`SIN_PPP → PPP_CONVERGING → PPP_ESTABLE`

### Corrección antena → pistón

- Offset: 0.55 m
- Bearing: `yaw + 270°` (antena a la derecha, pistón a la izquierda)
- Declinación magnética (Madrid): 1.0° (`yaw_geografico = yaw_magnetico - declinacion`)

---

## 1) Mapa de pines

### UART entrada GNSS (UM980)
- `GNSS_RX = 44` → ESP32 recibe desde TX del UM980
- `GNSS_TX = 43` → ESP32 transmite hacia RX del UM980
- `GNSS_BAUD = 115200`

### UART salida a Dynatest (vía MAX3232)
- `DYNATEST_TX = 17`
- `DYNATEST_RX = 18`
- `DYNATEST_BAUD = 38400`

### I2C IMU (BNO085)
- `I2C_SDA = 8`
- `I2C_SCL = 9`
- `I2C_FREQ = 100000`
- Dirección: `0x4B` (bus `Wire1`)

### LEDs
- `LED_RED = 4`: estado GNSS/HAS (parpadeo en `PPP_CONVERGING`, fijo en fix/`PPP_ESTABLE`)
- `LED_GREEN = 5`: estado de movimiento (parpadeo en `AVERAGING`, fijo en `LOCKED`)

---

## 2) Parámetros de navegación

- `OFFSET_M = 0.55`: distancia antena→pistón en metros.
- `DECLINATION_OFFSET = 1.0`: declinación magnética local (Madrid), en grados.
- `STOP_THRESHOLD_MS = 2000`: tiempo bajo velocidad de entrada para pasar a `AVERAGING`.
- `AVERAGING_WINDOW_MS = 15000`: ventana de promedio en parada (15 s).
- `SPEED_ENTER_STOP = 0.20` / `SPEED_EXIT_STOP = 0.30` (m/s): histéresis de detección de parada.
- `NEW_LOCATION_DIST = 1.0`: distancia (m) para salir de `LOCKED` por desplazamiento.
- `MAX_SAMPLES = 160`: tamaño de buffer de muestras para promedio (lat/lon/alt/yaw).
- `OUTPUT_PERIOD_MS = 100`: periodo de salida (10 Hz).

---

## 3) Formato de salida NMEA

Rev.1 genera y envía:
- **`$GCGGA,...*CS`**
- Frecuencia de salida: 10 Hz (`OUTPUT_PERIOD_MS = 100`) cuando existe GGA válida y fresca.
- `HDOP`: valor real del campo 8 de la GGA de entrada; si el campo llega vacío o inválido, se usa `1.0` como fallback seguro documentado.
- Separación geoidal: se conserva el campo 11 de la GGA de entrada; si falta o es inválido, se usa `0.0`.

El `fixQ` de salida conserva el `fixQ` válido recibido en la GGA de entrada.
El estado `LOCKED` no eleva artificialmente ese campo.

---

## 4) Flujo de funcionamiento

1. Lee líneas NMEA/propietarias del UM980 por `Serial1` (`$GPGGA` / `$GNGGA` / `$GCGGA`, `$GPRMC` / `$GNRMC` / `$GCRMC`, `#PPPNAVA`).
2. Valida checksum y parsea GGA (posición, altitud, satélites, fix) y RMC (velocidad, rumbo).
3. Actualiza estado PPP/HAS (`SIN_PPP` / `PPP_CONVERGING` / `PPP_ESTABLE`).
4. Lee yaw del IMU BNO085 (rotation vector) vía `Wire1`, con reintentos y detección de reset.
5. Actualiza la máquina de estados de movimiento (`MOVING` / `AVERAGING` / `LOCKED`).
6. En `AVERAGING`, acumula muestras de lat/lon/alt/yaw; al completar 15 s calcula medias recortadas (trimmed mean) y media circular del yaw, y aplica el offset antena→pistón.
7. En `LOCKED`, transmite la posición corregida; en otro caso transmite la posición instantánea (con offset si hay yaw disponible).
8. Construye la trama `$GCGGA` y la envía por `Serial2` hacia el MAX3232, a 10 Hz.
9. Actualiza LEDs de estado (`LED_RED`, `LED_GREEN`).

### Correcciones Rev.1

- La salida ya no depende de haber alcanzado `LOCKED`; se transmite en `MOVING` y `AVERAGING` siempre que GNSS siga válido y fresco.
- El parser acepta exactamente `$GPGGA`, `$GNGGA` y `$GCGGA`; evita coincidencias parciales accidentales.
- La detección PPP/HAS solo clasifica líneas `#PPPNAVA` y busca tokens delimitados `PPP_CONVERGING`, `PPP_ESTABLE` o `PPP_VALID`; evita tratar cualquier texto libre con `HAS` como estado válido.
- La salida se silencia si vence `GGA_FRESHNESS_MS`, preservando el comportamiento de seguridad ante datos obsoletos.
- La detección de reanudación de movimiento desde `LOCKED` compara la posición GNSS actual con la referencia GNSS bloqueada previa al offset, evitando mezclar coordenadas crudas con coordenadas corregidas.

---

## 5) Checklist rápido de validación

1. Ver logs por USB a `115200`.
2. Confirmar recepción de tramas `[GNSS] GGA:` y `[GNSS] RMC:`.
3. Confirmar inicialización del IMU (`[IMU] BNO085 initialized`) o reintentos periódicos si falla.
4. Verificar transición de estados en logs (`[STATE] MOVING -> AVERAGING -> LOCKED`).
5. Confirmar salida `$GCGGA` hacia el Dynatest y que este la acepta sin errores.
6. Validar variación real del `HDOP` en la salida.
7. Validar ausencia de salida tras el timeout de frescura GNSS.
8. Validar comportamiento de LEDs: `LED_RED` (GNSS/HAS) y `LED_GREEN` (movimiento/lock).

---

## 6) Arnés remoto BNO085/LED por RJ45 (custom, NO Ethernet)

**Etiqueta obligatoria:** `BNO085/LED — NO ETHERNET`

Pinout final (cable UTP directo pin-a-pin):

- Pin 1 = +5 V (**solo** a `VIN/5V` del breakout Adafruit BNO085)
- Pin 2 = GND
- Pin 3 = SDA
- Pin 4 = LED1 (`GPIO4`, `LED_RED`)
- Pin 5 = LED2 (`GPIO5`, `LED_GREEN`)
- Pin 6 = GND
- Pin 7 = SCL
- Pin 8 = GND

Guía práctica:

- Nunca aplicar +5 V directamente al IC BNO085 ni al pin `3V3`.
- No conectar este RJ45 a equipos Ethernet ni PoE.
- Usar pares trenzados donde sea práctico.
- Mantener el arnés lejos del cableado de potencia de la bomba FWD.
- Cruzar señal y potencia cerca de 90° cuando sea necesario.
- Añadir desacoplo local (100 nF + 10–100 µF) en el breakout.
- Verificar continuidad pin a pin antes de energizar.

---

## 7) Cambio resumido Rev.1

- Salida NMEA de Dynatest: `$GPGGA` histórico → `$GCGGA`.
- `HDOP` de salida: fijo `1.0` histórico → valor real de GGA campo 8, con fallback seguro.
- Bus I²C del BNO085: 400 kHz histórico → 100 kHz.
- Input GGA aceptado: `$GPGGA`, `$GNGGA`, `$GCGGA`.
- Se conserva la v0.99 histórica como referencia y no se elimina.

## 8) Checklist de aceptación Rev.1

- [ ] `$GCGGA` se transmite también en `MOVING`.
- [ ] El `HDOP` cambia dinámicamente según la GGA recibida.
- [ ] Checksum NMEA XOR y `\r\n` correctos.
- [ ] Salida estable a 10 Hz.
- [ ] No hay salida después del timeout de frescura GGA.
- [ ] BNO085 operativo a 100 kHz.
- [ ] Continuidad/pinout RJ45 verificados.
- [ ] Prueba hardware con bomba OFF y ON.

---

## 9) Notas

- Requiere la librería `Adafruit_BNO08x`.
- Si el receptor Dynatest requiere otro baudrate, ajustar `DYNATEST_BAUD`.
- La declinación magnética (`DECLINATION_OFFSET`) está calibrada para Madrid; ajustar si se despliega en otra ubicación.
- Los archivos `rs232_fwd_gps_draft_v0_9.ino` y `rs232_fwd_gps_final_v_0_99.ino` se conservan como referencia histórica.
- No se ha afirmado compilación de Rev.1 sin ejecutar el toolchain Arduino/ESP32-S3 correspondiente.
