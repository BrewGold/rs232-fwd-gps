# Arquitectura del sistema

## 1. Resumen

El sistema desacopla adquisición GNSS, lógica de estado y reenvío serial sobre el enlace activo del UM980.

Bloques principales:

1. Receptor GNSS (UM980 / simpleRTK3B Budget)
2. Controlador ESP32-S3
3. Enlace serial principal UM980 ↔ ESP32-S3
4. IMU BNO085/BNO086 (experimental)
5. LEDs externos de estado

## 2. Diagrama lógico

```text
NTRIP caster (Internet)
        │
        ▼
ESP32-S3 (cliente NTRIP) ───── RTCM ─────► UM980 (RTK3B)
        │                                    │
        │                                    └─ NMEA (GGA/RMC) / GGA 10 Hz ──► ESP32-S3 UART1 (TX3/RX3)
        │
        ├─ I2C ──► BNO085 (experimental)
        │
        ├─ GPIO ──► LED_POWER / LED_GNSS
```

## 3. Interfaz de datos

### 3.1 GNSS ↔ ESP32-S3

- Enlace: UART1 sobre TX3/RX3
- Baudrate: 115200
- Entrada a ESP32-S3:
  - NMEA GGA
  - NMEA RMC
- Salida desde ESP32-S3:
  - RTCM (cuando hay NTRIP)
- Reenvío adicional:
  - NMEA GGA a 10 Hz por el mismo enlace serial activo según el montaje actual

### 3.3 ESP32-S3 ↔ BNO085

- Enlace: I²C (100 kHz)
- Uso actual: telemetría experimental de rumbo
- Impacto en coordenada: ninguno en v1.0

## 4. Máquina de estados

### MOVING

- Se ingiere GNSS continuo.
- Se actualiza historial para detección de parada.
- Se mantiene reenvío GGA 10 Hz.

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

### LED_POWER

- OFF: sin alimentación
- ON fijo: sistema activo

### LED_GNSS

- OFF: sin solución
- Parpadeo lento: autónomo
- 2 destellos periódicos: SBAS
- 3 destellos periódicos: Galileo HAS
- ON fijo: RTK FIX

## 8. Cableado RJ45 (módulo remoto)

- Pin 1: SDA
- Pin 2: GND
- Pin 3: SCL
- Pin 4: +3V3
- Pin 5: +3V3
- Pin 6: GND
- Pin 7: LED_POWER
- Pin 8: LED_GNSS

## 9. Fases y criterio de aceptación

### Fase 1

Objetivo:

- Validar enlace serial activo UM980 ↔ ESP32-S3 según el montaje actual.

Aceptación:

- GGA estable a 10 Hz sobre el enlace activo.
- La topología de arranque reporta UM980 en TX3/RX3 y USB1 para debug.

### Fase 2

Objetivo:

- Integrar NTRIP y flujo RTCM.

Aceptación:

- Entrada RTCM efectiva al UM980.
- Transiciones de calidad reflejadas en LED_GNSS.

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
