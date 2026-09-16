# Especificación funcional v1.0

## Sistema GNSS para Dynatest FWD con RTK3B Budget

## Objetivo

Proporcionar al Dynatest FWD una posición GNSS mejorada mediante:

- RTK cuando exista conectividad NTRIP.
- Galileo HAS cuando esté disponible.
- SBAS/EGNOS como respaldo.
- Promedio temporal de coordenadas durante la parada.
- Presentación del estado GNSS mediante LEDs externos.

La versión Arduino unificada implementa corrección por rumbo con offset configurable entre antena y punto de referencia del Dynatest.

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

### Sensor de orientación

- QMC5883L
- Interfaz I²C
- Uso previsto: obtención de rumbo absoluto

Estado actual:

- Se usa heading magnético filtrado para corrección geométrica.
- Si el heading magnético falla temporalmente, se usa fallback por COG de VTG/RMC cuando la velocidad > 0.8 m/s y el COG es reciente (<= 2 s).
- Lecturas I2C con NACK/fallo activan recuperación lógica: reintentos de inicialización del QMC5883L y continuidad de salida GNSS sin bloqueo.
- El QMC5883L no tiene tilt compensation completa; requiere calibración hard-iron/soft-iron.

### Comunicación Dynatest

- Conversión RS232 con MAX3232

Conexión:

- ESP32-S3 UART2
- MAX3232
- Dynatest

## Comunicaciones

### UART RTK3B

- Puerto: ESP32-S3 UART1
- Velocidad: 115200 baud
- Mensajes recibidos: GGA, RMC
- Mensajes transmitidos: RTCM

### UART Dynatest

- Puerto: ESP32-S3 UART2
- Velocidad: 38400 baud
- Salida: GGA a 10 Hz

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

Se aplica offset desde la antena hacia el punto de referencia FWD. Para mástil delante del plato se configura offset longitudinal negativo (equivale a heading + 180°).

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

- Mantener salida GGA a 10 Hz al Dynatest

## Fases del proyecto

### Fase 1

RTK3B → ESP32-S3 → Dynatest

Validar:

- 38400 baud
- GGA 10 Hz
- Compatibilidad Dynatest

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
