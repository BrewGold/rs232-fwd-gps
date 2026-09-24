# Arquitectura del sistema

## 1. Resumen

El sistema desacopla adquisición GNSS, lógica de estado y salida serial hacia Dynatest FWD.

Bloques principales:

1. Receptor GNSS (UM980 / simpleRTK3B Budget)
2. Controlador ESP32-S3
3. Interfaz RS232 (MAX3232)
4. IMU BNO085/BNO086 (experimental)
5. LEDs externos de estado

## 2. Diagrama lógico

```text
NTRIP caster (Internet)
        │
        ▼
ESP32-S3 (cliente NTRIP) ───── RTCM ─────► UM980 (RTK3B)
        │                                    │
        │                                    └─ NMEA (GGA/RMC) ──► ESP32-S3 UART1
        │
        ├─ I2C ──► BNO085 (experimental)
        │
        ├─ GPIO ──► LED1(GPIO4) / LED2(GPIO5)
        │
        └─ UART2 (38400) ─► MAX3232 ─► Dynatest FWD (GGA 10 Hz)
```

## 3. Interfaz de datos

### 3.1 GNSS ↔ ESP32-S3

- Enlace: UART1
- Baudrate: 115200
- Entrada a ESP32-S3:
  - NMEA GGA
  - NMEA RMC
- Salida desde ESP32-S3:
  - RTCM (cuando hay NTRIP)

### 3.2 ESP32-S3 ↔ Dynatest

- Enlace: UART2 + MAX3232
- Baudrate: 38400
- Trama: NMEA **$GCGGA** generada por el firmware
- Tasa: 10 Hz

### 3.3 ESP32-S3 ↔ BNO085

- Enlace: I²C (100 kHz)
- Uso actual: telemetría experimental de rumbo
- Impacto en coordenada: ninguno en v1.0

## 4. Máquina de estados

### MOVING

- Se ingiere GNSS continuo.
- Se actualiza historial para detección de parada.
- Se mantiene salida GGA 10 Hz.

### STOPPED

- Se detecta parada por umbral de velocidad o desplazamiento.
- Se activa ventana de muestreo de 15 s.
- Se calculan medias de latitud, longitud y altitud.

### OUTPUT

- Se publica GGA a 10 Hz usando la mejor coordenada disponible.
- Mientras persista parada se puede mantener la coordenada promediada.
- Al reanudar movimiento, volver a solución instantánea.

## 5. Selección de calidad GNSS

Orden de prioridad:

1. RTK FIX
2. Galileo HAS
3. SBAS (EGNOS)
4. Autónomo

Regla:

- Siempre emitir en GGA la mejor solución válida disponible según prioridad.

## 6. Detección de parada

Criterios (OR):

- Velocidad < 0,2 km/h durante al menos 2 s.
- Desplazamiento < 0,10 m durante al menos 2 s.

Notas de implementación:

- Evaluar condición a 10 Hz.
- Usar ventana deslizante temporal para evitar rebotes.

## 7. LEDs de estado

### LED1 (GPIO4 / LED_RED)

- OFF: sin fix GNSS válido
- Parpadeo: PPP convergiendo
- ON fijo: GNSS válido / PPP estable

### LED2 (GPIO5 / LED_GREEN)

- OFF: MOVING
- Parpadeo lento: AVERAGING
- ON fijo: LOCKED válido

## 8. Cableado RJ45 (módulo remoto)

**Custom BNO085/LED — NO ETHERNET** (no conectar a PoE/equipos Ethernet).

- Pin 1: +5 V (solo a VIN/5V del breakout Adafruit BNO085)
- Pin 2: GND
- Pin 3: SDA
- Pin 4: LED1 (GPIO4 / LED_RED)
- Pin 5: LED2 (GPIO5 / LED_GREEN)
- Pin 6: GND
- Pin 7: SCL
- Pin 8: GND

Notas de instalación:

- I2C del BNO085 se opera a 100 kHz.
- Mantener el arnés lejos de cableado de potencia de bomba/motor.
- Cruzar potencia y señal a ~90° cuando sea necesario.
- Añadir desacoplo local en el breakout (100 nF + 10–100 µF).
- Verificar continuidad pin a pin antes de energizar.

## 9. Fases y criterio de aceptación

### Fase 1

Objetivo:

- Validar enlace serial con Dynatest.

Aceptación:

- GGA estable a 10 Hz, 38400 baud.
- Dynatest recibe y parsea sin errores.

### Fase 2

Objetivo:

- Integrar NTRIP y flujo RTCM.

Aceptación:

- Entrada RTCM efectiva al UM980.
- Transiciones de calidad reflejadas en LED1 (GPIO4).

### Fase 3

Objetivo:

- Activar promedio de 15 s al detenerse.

Aceptación:

- Cálculo de 150 muestras válidas.
- Menor dispersión posicional en parada frente a instantáneo.

### Fase 4

Objetivo:

- Integrar BNO085 + módulo remoto LED/RJ45.

Aceptación:

- Telemetría IMU operativa.
- LEDs operativos en gabinete remoto.
