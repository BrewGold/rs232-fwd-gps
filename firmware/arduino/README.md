# Firmware Arduino

Sketch recomendado para esta incidencia:

- `FWMGPSV89.ino`

## Librerías necesarias

- `Adafruit BNO08x`
- dependencias transitivas de la librería (`Adafruit BusIO`, `Adafruit Unified Sensor`)

## Cableado

### UM980 → ESP32-S3
- `Serial1 RX = GPIO44` desde `TX3 / COM3` del UM980
- `Serial1 TX = GPIO43` hacia `RX3 / COM3` del UM980 (opcional)
- `115200 8N1`

### ESP32-S3 → Dynatest / MAX3232
- `Serial2 TX = GPIO17`
- `Serial2 RX = GPIO18` (opcional)
- `38400 8N1`

### BNO085
- `SDA = GPIO8`
- `SCL = GPIO9`
- dirección I2C por defecto: `0x4A`

> La dirección I2C del BNO085 queda en la constante `IMU_I2C_ADDRESS` dentro del sketch para que sea fácil ajustarla.

### LEDs
- `LED_GREEN_PIN = GPIO4` → movimiento / averaging / locked
- `LED_RED_PIN = GPIO5` → estado GNSS / PPP

## Comportamiento implementado

- Entrada GNSS no bloqueante para `$GPGGA/$GNGGA`, `$GPRMC/$GNRMC` y `#PPPNAVA`.
- Yaw del BNO085 por I2C usando `begin_I2C(..., &ImuWire)` y `enableReport(SH2_ROTATION_VECTOR, ...)`.
- `currentYaw = NAN` si no hay yaw válido; si la IMU falla, la salida sigue funcionando sin aplicar offset falso.
- Máquina de estados: `MOVING -> STOP_CONFIRM -> AVERAGING -> LOCKED`.
- En `MOVING/STOP_CONFIRM/AVERAGING` sale posición instantánea corregida.
- En `LOCKED` sale la posición promediada corregida.
- Salida `GPGGA` a `Serial2` cada `100 ms` mientras el GNSS esté fresco.
- Offset antena→pistón de `1.50 m` hacia atrás, aplicado una sola vez por ruta de salida.
- Diagnóstico PPP textual: `SIN_PPP`, `PPP_CONVERGING`, `PPP_ESTABLE`.

## Configuración manual del UM980

Configurar manualmente el receptor; el sketch **no** envía comandos al arrancar.

### COM3 hacia el ESP32-S3
```text
GPGGA COM3 1
GPRMC COM3 1
PPPNAVA COM3 1
```

### COM1 para diagnóstico
```text
GPGGA COM1 1
GPRMC COM1 1
PPPNAVA COM1 1
```

### COM2 limpio para no mezclar tráfico
```text
GPGGA COM2 0
GPRMC COM2 0
PPPNAVA COM2 0
```

## Qué debe observarse

- **COM1 / USB GPS**: comandos, NMEA nativo y `#PPPNAVA`.
- **COM3 / TX3**: `GGA`, `RMC` y `#PPPNAVA` para alimentar al ESP32-S3.
- **USB2 / COM2**: si se usa como monitor de retorno, debe quedar limpio de NMEA/PPP propios del UM980.
- **USB debug del ESP32-S3**: cambios de estado (`MOVING`, `STOP_CONFIRM`, `AVERAGING`, `LOCKED`) y diagnóstico PPP limitado.
- **Dynatest / salida RS232**: exactamente una trama `$GPGGA,...*CS` por período de `100 ms` mientras el GNSS esté fresco.

## Validación realizada en esta tarea

- Revisión estática del sketch completo.
- Revisión estática del uso de la API `Adafruit_BNO08x::begin_I2C` y `enableReport`.

No se declara validación hardware real del ESP32-S3, UM980, BNO085 ni Dynatest dentro de esta tarea.
