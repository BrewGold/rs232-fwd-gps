# RS232-FWD-GPS

Sistema GNSS para Dynatest FWD con receptor ArduSimple simpleRTK3B Budget (UM980) y controlador ESP32-S3.

## Objetivo

Proporcionar al Dynatest FWD una posición GNSS mejorada mediante:

- RTK cuando exista conectividad NTRIP.
- Galileo HAS cuando esté disponible.
- SBAS/EGNOS como respaldo.
- Promedio temporal de coordenadas durante la parada.
- Corrección antena→pistón usando yaw del BNO085 cuando el IMU está disponible.
- Presentación del estado GNSS mediante LEDs externos.

No se implementará inicialmente corrección por rumbo basada en trayectoria.

## Arquitectura (alto nivel)

- GNSS: ArduSimple simpleRTK3B Budget (UM980)
- MCU: ESP32-S3 formato UNO
- IMU: Adafruit BNO085/BNO086 por I²C a 100 kHz
- Conversión RS232: MAX3232

Flujo principal:

1. ESP32-S3 recibe GGA/RMC del UM980 por UART1 (115200).
2. ESP32-S3 inyecta RTCM (NTRIP) al UM980.
3. ESP32-S3 detecta estado MOVING/STOPPED.
4. En STOPPED promedia coordenadas (15 s a 10 Hz).
5. ESP32-S3 emite `$GCGGA` a Dynatest por UART2 + MAX3232 (38400, 10 Hz) cuando la GGA es válida y reciente.
6. LEDs externos en GPIO4/GPIO5 muestran estado GNSS/HAS y movimiento/lock.

## Prioridad de solución GNSS

1. RTK FIX
2. Galileo HAS
3. SBAS (EGNOS)
4. GPS autónomo

## Estados de software

- MOVING: lectura GNSS continua y actualización de historial.
- STOPPED: promedio de coordenadas durante 15 s.
- OUTPUT: mantenimiento de salida GGA a 10 Hz al Dynatest.

## Estructura del repositorio

- `docs/functional-spec-v1.0.md`: especificación funcional completa.
- `docs/system-architecture.md`: detalle de arquitectura y comunicaciones.
- `firmware/arduino/`: sketches Arduino (`rs232_fwd_gps_final_v_0_99.ino` histórico, `rs232_fwd_gps_draft_v0_9.ino` de referencia y `rs232_fwd_gps_rev_1_0.ino` como Rev.1 actual).

## Rev.1 actual

- Base de código: `firmware/arduino/rs232_fwd_gps_rev_1_0.ino`, derivada de la v0.99 histórica.
- Entrada GNSS aceptada: `$GPGGA`, `$GNGGA`, `$GCGGA` y RMC equivalentes.
- Salida Dynatest: `$GCGGA` con checksum XOR NMEA y terminación `\r\n`.
- `HDOP`: se conserva el valor real del campo 8 de la GGA de entrada; si falta o es inválido, se usa `1.0` como fallback seguro documentado.
- Si expira el timeout de frescura de GGA, la salida se silencia para no retransmitir posiciones obsoletas.

### Arnés remoto BNO085/LED

**RJ45/8P8C custom, cable UTP directo pin-a-pin — `BNO085/LED — NO ETHERNET`**

- Pin 1 = +5 V (**solo** a `VIN/5V` del breakout Adafruit BNO085)
- Pin 2 = GND
- Pin 3 = SDA
- Pin 4 = LED1 (`GPIO4`)
- Pin 5 = LED2 (`GPIO5`)
- Pin 6 = GND
- Pin 7 = SCL
- Pin 8 = GND

No conectar este arnés a Ethernet ni PoE. Mantenerlo alejado del cableado de potencia de la bomba hidráulica, cruzar potencia/señal cerca de 90° cuando sea necesario, colocar desacoplo local (100 nF + bulk) en el breakout y verificar continuidad antes de energizar.

## Roadmap histórico

La Rev.1 ya incorpora el arnés remoto BNO085/LED y la salida `$GCGGA`; la lista siguiente se conserva como referencia de planificación original.

- Fase 1: Validar enlace RTK3B → ESP32-S3 → Dynatest (38400, GGA 10 Hz).
- Fase 2: Implementar NTRIP/RTCM y RTK FIX.
- Fase 3: Implementar promedio de 15 s en parada.
- Fase 4: Integrar BNO085, LEDs y arnés RJ45 para evaluación futura.
