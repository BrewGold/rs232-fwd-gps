# Firmware Arduino (versión `.ino` unificada)

Archivo principal:

- `rs232_fwd_gps_unificado.ino`

Este sketch integra en un solo archivo:

1. Lectura GNSS (UM980) por UART de entrada.
2. Lectura de heading desde magnetómetro I2C (QMC5883L).
3. Cálculo de coordenada corregida con offset.
4. Generación y envío de trama **`$GPGGA`** (GPSGGA) por UART de salida hacia **MAX3232**.
5. Logs por USB (`Serial`) y LEDs de estado.
6. Autotest inicial de 10 segundos.

---

## 1) Mapa de pines (actual)

### UART entrada GNSS (UM980)
- `GNSS_RX_PIN = 44` → ESP32 recibe desde TX3 de UM980
- `GNSS_TX_PIN = 43` → ESP32 transmite hacia RX3 de UM980 (opcional)
- `GNSS_BAUD = 115200`

### UART salida a MAX3232
- `OUT_TX_PIN = 17` → ESP32 TX hacia RX del MAX3232
- `OUT_RX_PIN = 18` → ESP32 RX desde TX del MAX3232 (opcional)
- `OUT_BAUD = 115200` (ajustar según receptor final)

### I2C magnetómetro
- `I2C_SDA_PIN = 8`
- `I2C_SCL_PIN = 9`
- `MAG_ADDR = 0x0D` (QMC5883L típico)

### LEDs (ajustables según placa)
- `LED_GNSS = 2` (fix)
- `LED_MAG = 4` (pulso por lectura de magnetómetro)
- `LED_ERR = 5` (error / sin fix)

> Si tu hardware usa otros GPIO, cambia solo las constantes al inicio del `.ino`.

---

## 2) Parámetros de navegación a ajustar

- `offsetMeters`  
  Distancia (m) para calcular el punto corregido.

- `lateralOffset`  
  - `false`: offset en dirección del heading (hacia delante).
  - `true`: offset lateral.

- `offsetToRight` (solo si `lateralOffset=true`)  
  - `true`: lateral a la derecha.
  - `false`: lateral a la izquierda.

- `declinationDeg`  
  Declinación magnética local (grados), para convertir heading magnético a verdadero.

- `headingAlpha`  
  Filtro del heading (`0..1`): menor valor = más suavizado.

---

## 3) Formato de salida NMEA

El firmware **fuerza** la salida a:
- **`$GPGGA,...*CS`** (talker `GP`)

Esto se hizo para compatibilidad con receptores que esperan específicamente **GPSGGA**.

---

## 4) Flujo de funcionamiento

1. Lee líneas NMEA del GNSS de entrada.
2. Procesa sentencias GGA (`$GPGGA` o `$GNGGA`) de entrada.
3. Toma lat/lon/fix/satélites/altitud.
4. Lee heading del magnetómetro (filtrado).
5. Aplica corrección geodésica por distancia + rumbo.
6. Construye sentencia corregida en formato `$GPGGA`.
7. Envía por `Serial2` (`OUT`) hacia MAX3232.

---

## 5) Autotest al arranque (10 s)

En `setup()` se ejecuta `runAutoTest10s()` y reporta por USB:

- estado de inicialización del magnetómetro,
- cantidad de lecturas MAG,
- cantidad de GGA de entrada detectadas,
- si hubo fix GNSS,
- cantidad de GPGGA enviadas por salida.

Resultado final:
- `PASS` → flujo básico operativo.
- `REVISAR CABLEADO/BAUD/PINES` → revisar conexiones/configuración.

---

## 6) Checklist rápido de validación

1. Ver logs por USB a `115200`.
2. Confirmar que entran GGA del GNSS (`GGA IN > 0`).
3. Confirmar que hay fix (`fixQ > 0`).
4. Confirmar lecturas de magnetómetro (`MAG lecturas > 0`).
5. Confirmar salida al MAX3232 (`GPGGA OUT > 0`).
6. Verificar que el receptor remoto acepta `$GPGGA`.

---

## 7) Notas

- Si usas HMC5883L u otro magnetómetro, habrá que adaptar init/lectura.
- Si el receptor RS232 requiere otro baud, cambia `OUT_BAUD`.
- Si no necesitas RX en UART de salida, `OUT_RX_PIN` puede quedar sin uso físico.
