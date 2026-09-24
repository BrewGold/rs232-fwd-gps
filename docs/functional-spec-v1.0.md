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

### Sensor de orientación

- Adafruit BNO085/BNO086
- Interfaz I²C
- Uso en Rev.1: yaw para corregir el offset antena→pistón cuando el IMU está disponible

Estado actual:

- Integrado en Rev.1 con fallback a GNSS puro si no hay yaw válido
- Conserva la arquitectura base de la v0.99 de campo, pero en Rev.1 sí aplica el offset con yaw cuando el IMU está disponible

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
- Salida: `$GCGGA` a 10 Hz, con checksum XOR y `CRLF`

### I²C IMU

- ESP32-S3 ↔ BNO085
- Velocidad: 100 kHz
- Pines: SDA=8, SCL=9, dirección `0x4B`

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

- En `MOVING`: posición instantánea con offset antena→pistón si hay yaw válido.
- En `LOCKED`: posición promedio de 15 s con el mismo offset aplicado una sola vez.
- Si el BNO085 no entrega yaw válido, la salida cae a posición GNSS pura.
- No se aplica corrección por trayectoria previa.

## Indicadores externos

Ubicación:

- Integrados en el módulo remoto asociado al BNO085

### LED 1 — GPIO4 / LED_RED

- Apagado → Sin fix GNSS válido
- Parpadeo → PPP convergiendo
- Encendido → GNSS válido / PPP estable

### LED 2 — GPIO5 / LED_GREEN

- Apagado → MOVING
- Parpadeo lento → AVERAGING
- Encendido fijo → LOCKED válido

## Cableado IMU

Conector:

- RJ45 (CAT5e/CAT6)
- **BNO085/LED — NO ETHERNET**

Asignación:

- Pin 1: +5 V (solo a `VIN/5V` del breakout Adafruit)
- Pin 2: GND
- Pin 3: SDA
- Pin 4: LED1 (`GPIO4`)
- Pin 5: LED2 (`GPIO5`)
- Pin 6: GND
- Pin 7: SCL
- Pin 8: GND

Guía práctica:

- No conectar a Ethernet ni PoE.
- No aplicar +5 V directamente al IC BNO085 ni al pin `3V3`.
- Usar pares trenzados donde sea práctico.
- Separar el arnés del cableado de motor/solenoides/potencia de la FWD.
- Cruzar señal y potencia cerca de 90° cuando sea necesario.
- Añadir desacoplo local 100 nF + capacidad bulk en el breakout.
- Verificar continuidad pin a pin antes de energizar.

## Software

### Estado MOVING

- Leer GGA
- Leer RMC
- Actualizar historial
- Mantener salida `$GCGGA` si GNSS es válido y fresco

### Estado STOPPED

- Promedio 15 s
- Mantener salida `$GCGGA` mientras exista GNSS válido y fresco

### Estado OUTPUT

- Mantener salida `$GCGGA` a 10 Hz al Dynatest
- Silenciar la salida tras timeout de frescura GGA

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

## Rev.1 — cambios y aceptación

Cambios clave:

- `$GPGGA` histórico en salida sustituido por `$GCGGA` en la transmisión al Dynatest.
- `HDOP` de salida tomado del campo 8 de la GGA de entrada.
- I²C del BNO085 reducido a 100 kHz para el arnés remoto.
- Aceptación de talker IDs `$GPGGA`, `$GNGGA` y `$GCGGA`.

Checklist de aceptación Rev.1:

- [ ] `$GCGGA` se mantiene en MOVING y en parada con GNSS válido.
- [ ] El `HDOP` cambia con los valores reales recibidos.
- [ ] Checksum XOR y `CRLF` correctos.
- [ ] Salida estable a 10 Hz.
- [ ] Silencio tras timeout de frescura GGA.
- [ ] BNO085 operativo a 100 kHz.
- [ ] Continuidad/pinout del RJ45 verificados antes de energizar.
- [ ] Prueba de hardware con bomba OFF y ON.
