# RS232-FWD-GPS

Sistema GNSS para Dynatest FWD con receptor ArduSimple simpleRTK3B Budget (UM980) y controlador ESP32-S3.

## Objetivo

Proporcionar al Dynatest FWD una posición GNSS mejorada mediante:

- RTK cuando exista conectividad NTRIP.
- Galileo HAS cuando esté disponible.
- SBAS/EGNOS como respaldo.
- Promedio temporal de coordenadas durante la parada.
- Presentación del estado GNSS mediante LEDs externos.

La fase Arduino unificada sí implementa corrección geométrica por rumbo (offset antena -> punto de referencia Dynatest) con fallback por COG cuando el magnetómetro no está disponible.

## Arquitectura (alto nivel)

- GNSS: ArduSimple simpleRTK3B Budget (UM980)
- MCU: ESP32-S3 formato UNO
- Magnetómetro: QMC5883L por I2C (con recuperación ante NACK/intermitencias)
- Conversión RS232: MAX3232

Flujo principal:

1. ESP32-S3 recibe GGA/VTG/RMC del UM980 por UART1 (115200).
2. ESP32-S3 inyecta RTCM (NTRIP) al UM980.
3. ESP32-S3 detecta estado MOVING/STOPPED.
4. En STOPPED promedia coordenadas (15 s a 10 Hz).
5. ESP32-S3 emite GGA a Dynatest por UART2 + MAX3232 (38400, 10 Hz).
6. LEDs externos muestran estado POWER/GNSS.

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
- `firmware/esp32-s3/`: base de firmware modular.

## Roadmap

- Fase 1: Validar enlace RTK3B → ESP32-S3 → Dynatest (38400, GGA 10 Hz).
- Fase 2: Implementar NTRIP/RTCM y RTK FIX.
- Fase 3: Implementar promedio de 15 s en parada.
- Fase 4: consolidar calibración de brújula (hard-iron/soft-iron), tilt compensation y arnés RJ45 de campo.

## Notas de operación de campo (Dynatest Embedded)

- Salida a Dynatest por MAX3232: **38400 baudios, 10 Hz, `$GPGGA`**.
- Geometría típica: antena/magnetómetro en mástil (~40 cm) delante del centro del plato; en firmware se representa con `offsetAlongHeadingMeters < 0`, lo que aplica corrección hacia atrás (`heading + 180°`).
- El promedio de 15 s mantiene la cadencia de 10 Hz y actualiza UTC de salida mediante avance temporal local entre tramas GGA recibidas (limitado por jitter de `millis()`).
- QMC5883L no ofrece tilt compensation completa; para pendientes pronunciadas se recomienda validar el error angular residual.
