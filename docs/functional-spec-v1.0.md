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
- Salida: `$GCGGA` a 10 Hz

### I²C IMU

- ESP32-S3 ↔ BNO085
- Velocidad: 100 kHz
- Dirección: `0x4B` (sin cambios)

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

### LED 1 — GPIO4 / LED_RED

- Apagado → Sin fix GNSS válido
- Parpadeo → PPP convergiendo
- Encendido → GNSS válido/PPP estable

### LED 2 — GPIO5 / LED_GREEN

- Apagado → MOVING
- Parpadeo lento → AVERAGING
- Encendido fijo → LOCKED válido

## Cableado IMU

Conector:

- RJ45 (CAT5e/CAT6)
- **Custom BNO085/LED — NO ETHERNET** (no conectar a PoE/equipos Ethernet)

Asignación:

- Pin 1: +5 V (solo a VIN/5V del breakout Adafruit BNO085)
- Pin 2: GND
- Pin 3: SDA
- Pin 4: LED1 (GPIO4 / LED_RED)
- Pin 5: LED2 (GPIO5 / LED_GREEN)
- Pin 6: GND
- Pin 7: SCL
- Pin 8: GND

Guía de instalación:

- Usar pares trenzados donde sea práctico.
- Mantener el arnés lejos de cableado de potencia de bomba/motor.
- Cruzar potencia y señal a ~90° cuando aplique.
- Añadir desacoplo local en el breakout (100 nF + 10–100 µF).
- Verificar continuidad pin-a-pin antes de energizar.

## Software

### Estado MOVING

- Leer GGA
- Leer RMC
- Actualizar historial

### Estado STOPPED

- Promedio 15 s

### Estado OUTPUT

- Mantener salida `$GCGGA` a 10 Hz al Dynatest cuando GNSS esté fresco y válido

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
