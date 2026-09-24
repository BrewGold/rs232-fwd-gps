# Especificación funcional v1.0

## Sistema GNSS para Dynatest FWD con Arduino UNO R4 WiFi

## Objetivo

Proporcionar al Dynatest FWD una posición GNSS mejorada mediante:

- RTK cuando exista conectividad externa adecuada.
- Galileo HAS cuando esté disponible en el UM980.
- SBAS/EGNOS como respaldo.
- Promedio temporal de coordenadas durante la parada.
- Presentación del estado GNSS mediante LEDs externos.
- Corrección geométrica antena→pistón con yaw del BNO085 cuando haya yaw válido.

## Arquitectura hardware real

### Receptor GNSS

- ArduSimple simpleRTK3B Budget
- UM980
- Salida de trabajo hacia el firmware: GGA + RMC + `#PPPNAVA`

### Controlador

- **Arduino UNO R4 WiFi**
- MCU principal: **Renesas RA4M1**
- El ESP32-S3 integrado no se usa como MCU de esta aplicación.

Funciones:

- Recepción GNSS por `Serial1`
- Detección de parada
- Promedio de coordenadas
- Generación `$GCGGA`
- Control de LEDs
- Lectura de yaw BNO085

### Sensor de orientación

- Adafruit BNO085/BNO086
- Interfaz **I²C `Wire`**
- Velocidad de bus: **100 kHz**
- Uso: yaw absoluto para aplicar offset antena→pistón

### Comunicación Dynatest

- Canal base implementado: **USB CDC (`Serial`)**
- Salida: **`$GCGGA` a 10 Hz**
- Limitación: el mismo `Serial` no debe compartirse con logs si se usa como enlace NMEA limpio.

## Comunicaciones

### UART RTK3B

- Puerto: **`Serial1`**
- Pines: **D0/D1** del UNO R4 WiFi
- Velocidad: **115200 baud**
- Mensajes aceptados: `$GPGGA`, `$GNGGA`, `$GCGGA`, `$GPRMC`, `$GNRMC`, `#PPPNAVA`

### Salida Dynatest

- Puerto implementado: **`Serial` / USB CDC**
- Velocidad práctica en host: configurar según el convertidor/monitor usado; la capa USB CDC no crea una segunda UART física en la placa.
- Formato: **`$GCGGA`**
- Tasa: **10 Hz**
- Terminación: **CRLF**

### I²C IMU

- Puerto: **`Wire`**
- Velocidad: **100 kHz**
- Pines: **SDA/SCL** del UNO R4 WiFi

## Posicionamiento

### Prioridad de soluciones

1. Posición `LOCKED` válida (parado con promedio finalizado)
2. PPP estable
3. PPP convergiendo
4. GNSS autónomo válido

### Detección de parada

Condiciones:

- velocidad `< 0.20 m/s` durante al menos `2 s` para entrar en `AVERAGING`;
- velocidad `> 0.30 m/s` para salir a `MOVING`;
- en `LOCKED`, salir a `MOVING` si la distancia supera `1 m` comparando coordenadas **raw con raw**.

### Promedio GNSS

Al detectar parada:

- ventana: `15 s`
- periodo de muestreo: `10 Hz`
- medias calculadas: latitud, longitud, altitud y yaw circular si está disponible

### Coordenada enviada

- en `MOVING`: posición instantánea con offset si hay yaw válido;
- en `LOCKED`: posición promediada con offset si hay yaw válido;
- si no hay yaw, se emite la coordenada GNSS sin corregir.

## NMEA

### Entrada

- Validación obligatoria del checksum XOR en GGA y RMC.
- El HDOP de salida debe propagarse desde el campo 8 de la GGA recibida.
- Si el HDOP de entrada falta o es inválido, se usa un fallback documentado de `1.0`.

### Salida

- Sentencia generada: **`$GCGGA`**
- Checksum: XOR NMEA
- Terminación: `\r\n`
- Timeout de frescura: si la última GGA supera el umbral configurado, la salida se silencia.

## PPP / HAS

- Solo se procesan mensajes que **empiezan por `#PPPNAVA`**.
- No se clasifican mensajes arbitrarios por contener la cadena `HAS`.

## Indicadores externos

### LED1

- OFF → sin GGA fresca
- Parpadeo → PPP convergiendo
- ON fijo → GNSS válido / PPP estable

### LED2

- OFF → `MOVING`
- Parpadeo → `AVERAGING`
- ON fijo → `LOCKED`

## Arnés remoto BNO085/LED

Etiqueta obligatoria:

- **`BNO085/LED — NO ETHERNET`**

Asignación RJ45:

1. `+5V` solo a `VIN/5V` del breakout Adafruit
2. `GND`
3. `SDA`
4. `LED1`
5. `LED2`
6. `GND`
7. `SCL`
8. `GND`

Restricciones de instalación:

- cable UTP directo pin‑a‑pin;
- nunca conectar a Ethernet ni PoE;
- mantener separación respecto al cableado de bomba/motor;
- cruces a ~90° cuando sean inevitables;
- desacoplo local de `100 nF + 10–100 µF`;
- comprobar continuidad antes de energizar.

## Aceptación Rev.1 UNO R4 WiFi

- `$GCGGA` a 10 Hz en movimiento
- `$GCGGA` a 10 Hz en parada
- HDOP variable propagado
- checksum y CRLF correctos
- silencio por timeout de frescura
- BNO085 por `Wire` a 100 kHz
- `Serial1` en D0/D1
- LEDs en pines válidos UNO R4
- RJ45 seguro y etiquetado `NO ETHERNET`
- prueba con bomba apagada y encendida
