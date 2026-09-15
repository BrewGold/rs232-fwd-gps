# Especificación funcional v1.0

## Sistema GNSS para Dynatest FWD con RTK3B Budget

## Objetivo

Proporcionar al Dynatest FWD una posición GNSS mejorada mediante:

- RTK cuando exista conectividad NTRIP.
- Galileo HAS cuando esté disponible.
- SBAS/EGNOS como respaldo.
- Promedio temporal de coordenadas durante la parada.
- Presentación del estado GNSS mediante LEDs externos.

No se implementará corrección por rumbo basada en trayectoria.

## Arquitectura Hardware

### Receptor GNSS

- ArduSimple simpleRTK3B Budget
- UM980

Características:

- COM1 → UPrecise
- COM3 → Comunicación principal con ESP32-S3

### Controlador

- ESP32-S3 formato UNO

Funciones:

- Cliente NTRIP
- Recepción GNSS
- Detección de parada
- Promedio de coordenadas
- Generación GGA
- Control LEDs

### Sensor de orientación (experimental)

- Adafruit BNO085/BNO086
- Interfaz I²C
- Uso previsto: obtención de rumbo absoluto

Estado actual:

- Experimental
- No se utilizará inicialmente para corregir coordenadas

### Comunicación Dynatest

- El flujo actual reutiliza el mismo enlace serial principal del UM980 según el montaje reportado

Conexión:

- UM980 COM3 ↔ ESP32-S3 UART1 (TX3/RX3)
- USB1 reservado para programación y logs

## Comunicaciones

### UART RTK3B

- Puerto: ESP32-S3 UART1 sobre TX3/RX3
- Velocidad: 115200 baud
- Mensajes recibidos: GGA, RMC
- Mensajes transmitidos: RTCM y GGA a 10 Hz por el enlace activo según el montaje actual

### I²C IMU

- ESP32-S3 ↔ BNO085
- Velocidad: 100 kHz

## Posicionamiento

### Prioridad de soluciones

1. RTK FIX
2. Galileo HAS
3. SBAS (EGNOS)
4. GPS autónomo

### Detección de parada

Condición preliminar (cualquiera):

- Velocidad < 0,2 km/h durante 2 s
- o desplazamiento < 10 cm durante 2 s

### Promedio GNSS

Al detectar parada:

- Ventana: 15 segundos
- Muestras: 10 Hz × 15 s = 150 observaciones

Resultado:

- Latitud media
- Longitud media
- Altitud media

### Coordenada enviada

Inicialmente:

- Posición media de la antena GNSS

No se aplicará:

- Corrección por trayectoria previa

## Indicadores externos

Ubicación:

- Integrados en el módulo remoto asociado al BNO085

### LED 1 — POWER

- Apagado → Sin alimentación
- Encendido → Sistema operativo

### LED 2 — GNSS

- Apagado → Sin solución
- Parpadeo lento → GPS autónomo
- 2 destellos → SBAS
- 3 destellos → Galileo HAS
- Encendido fijo → RTK FIX

## Cableado IMU

Conector:

- RJ45 (CAT5e/CAT6)

Asignación:

- Pin 1: SDA
- Pin 2: GND
- Pin 3: SCL
- Pin 4: +3V3
- Pin 5: +3V3
- Pin 6: GND
- Pin 7: LED_POWER
- Pin 8: LED_GNSS

## Software

### Estado MOVING

- Leer GGA
- Leer RMC
- Actualizar historial

### Estado STOPPED

- Promedio 15 s

### Estado OUTPUT

- Mantener salida GGA a 10 Hz sobre el enlace serial activo

## Fases del proyecto

### Fase 1

RTK3B ↔ ESP32-S3 sobre enlace serial único

Validar:

- 115200 baud en UART1 / TX3-RX3
- GGA 10 Hz
- Topología de arranque reportada por USB1

### Fase 2

Implementar:

- NTRIP
- RTCM
- RTK FIX

### Fase 3

Implementar:

- Promedio de 15 s

### Fase 4

Instalar:

- BNO085
- LEDs
- RJ45

Para evaluación de corrección geométrica futura.
