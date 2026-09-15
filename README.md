# RS232-FWD-GPS

Sistema GNSS para Dynatest FWD con receptor ArduSimple simpleRTK3B Budget (UM980) y controlador ESP32-S3.

## Objetivo

Proporcionar al Dynatest FWD una posición GNSS mejorada mediante:

- RTK cuando exista conectividad NTRIP.
- Galileo HAS cuando esté disponible.
- SBAS/EGNOS como respaldo.
- Promedio temporal de coordenadas durante la parada.
- Presentación del estado GNSS mediante LEDs externos.

No se implementará inicialmente corrección por rumbo basada en trayectoria.

## Arquitectura (alto nivel)

- GNSS: ArduSimple simpleRTK3B Budget (UM980)
- MCU: ESP32-S3 formato UNO
- IMU: Adafruit BNO085/BNO086 (experimental)
- Salida RS232 hacia Dynatest usando el enlace serial activo del montaje

Flujo principal:

1. ESP32-S3 usa un único enlace serial activo con el UM980 sobre TX3/RX3 (UART1, 115200).
2. USB1 queda reservado para programación y logs.
3. ESP32-S3 recibe GGA/RMC del UM980 e inyecta RTCM (NTRIP) por ese mismo enlace.
4. ESP32-S3 detecta estado MOVING/STOPPED.
5. En STOPPED promedia coordenadas (15 s a 10 Hz).
6. ESP32-S3 reenvía GGA a 10 Hz por el enlace serial activo según el montaje actual.
7. LEDs externos muestran estado POWER/GNSS.

## Prioridad de solución GNSS

1. RTK FIX
2. Galileo HAS
3. SBAS (EGNOS)
4. GPS autónomo

## Estados de software

- MOVING: lectura GNSS continua y actualización de historial.
- STOPPED: promedio de coordenadas durante 15 s.
- OUTPUT: mantenimiento de salida GGA a 10 Hz sobre el enlace serial activo.

## Estructura del repositorio

- `docs/functional-spec-v1.0.md`: especificación funcional completa.
- `docs/system-architecture.md`: detalle de arquitectura y comunicaciones.
- `firmware/esp32-s3/`: base de firmware modular.

## Roadmap

- Fase 1: Validar enlace serial activo RTK3B ↔ ESP32-S3 (GGA 10 Hz según montaje actual).
- Fase 2: Implementar NTRIP/RTCM y RTK FIX.
- Fase 3: Implementar promedio de 15 s en parada.
- Fase 4: Integrar BNO085, LEDs y arnés RJ45 para evaluación futura.
