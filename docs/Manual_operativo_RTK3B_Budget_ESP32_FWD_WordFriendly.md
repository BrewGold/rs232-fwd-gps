# MANUAL OPERATIVO

## simpleRTK3B Budget + Arduino UNO R4 WiFi + FWD GCGGA + BNO085 remoto

Versión: 1.2

Fecha: 24/09/2026

Preparado para: operación técnica / campo

---

## 1. Objetivo

Implementar y validar un sistema donde:

- el **simpleRTK3B Budget / UM980** entrega GNSS al **Arduino UNO R4 WiFi** por `Serial1`;
- el UNO R4 WiFi procesa posición, velocidad, offset y lógica detenido/promedio 15 s;
- el UNO R4 WiFi genera **`$GCGGA` a 10 Hz** mientras la GGA esté válida y fresca;
- el **BNO085** remoto aporta yaw por I2C para aplicar el offset antena→pistón;
- los **LED1/LED2** externos muestran estado GNSS y estado de movimiento.

### Limitación real del hardware

El UNO R4 WiFi solo dispone de una UART hardware externa principal (`Serial1`) además del USB CDC (`Serial`). Por tanto:

- `Serial1` queda dedicado al GNSS;
- `Serial`/USB CDC se usa **o bien** para diagnóstico **o bien** para salida Dynatest limpia;
- si se necesitan dos enlaces físicos simultáneos sin conmutación, hace falta hardware externo adicional.

---

## 2. Mapeo de puertos

### UNO R4 WiFi

| Interfaz | Uso | Ajuste |
|---|---|---|
| `Serial1` | GNSS UM980 | `115200`, D0/D1 |
| `Serial` | Diagnóstico USB **o** Dynatest NMEA limpio | USB CDC |
| `Wire` | BNO085 remoto | `100 kHz`, SDA/SCL |
| `D6` | LED1 | Estado GNSS/PPP |
| `D7` | LED2 | Estado MOVING/AVERAGING/LOCKED |

### UM980

| Interfaz física RTK3B | Puerto UM980 | Uso recomendado |
|---|---|---|
| USB GPS | COM1 | Configuración y observación local |
| Pixhawk + TX3/RX3 | COM3 | GGA/RMC/PPPNAVA hacia el UNO R4 |

Notas:

- Regla UART: `TX origen -> RX destino`.
- El firmware Rev.1 **no usa `Serial2`** porque no existe en el UNO R4 WiFi como segundo enlace UART externo disponible para este montaje.

---

## 3. Arnés RJ45/UTP remoto

**Etiqueta obligatoria:** `BNO085/LED — NO ETHERNET`

Asignación fija:

1. `+5V` solo a `VIN/5V` del breakout Adafruit BNO085
2. `GND`
3. `SDA`
4. `LED1`
5. `LED2`
6. `GND`
7. `SCL`
8. `GND`

### Reglas de seguridad

- Nunca conectar este RJ45 a Ethernet ni PoE.
- UTP directo pin‑a‑pin.
- Aprovechar pares trenzados cuando sea práctico.
- Mantener distancia respecto al cableado de bomba/motor.
- Si hay cruces inevitables, hacerlos a ~90°.
- Añadir desacoplo local de `100 nF + 10–100 µF` junto al breakout.
- Hacer prueba de continuidad antes de energizar.

---

## 4. Preparación del UM980

Conectarse por USB GPS (COM1), normalmente a `115200 8N1`, y habilitar como mínimo:

```text
GPGGA COM3 1
GPRMC COM3 1
PPPNAVA COM3 1
SAVECONFIG
```

Opcionalmente, para observación local por COM1:

```text
GPGGA COM1 1
GPRMC COM1 1
PPPNAVA COM1 1
SAVECONFIG
```

### Importante

- El firmware procesa GGA/RMC con checksum válido.
- El parser PPP/HAS **solo** procesa líneas que empiezan por `#PPPNAVA`.
- No se deben clasificar mensajes arbitrarios solo por contener la cadena `HAS`.

---

## 5. Comportamiento funcional esperado

### 5.1 Movimiento

- Mientras la GGA esté fresca, el firmware emite **`$GCGGA` a 10 Hz**.
- Si el BNO085 entrega yaw válido, se aplica offset de `0.55 m` con bearing `yaw + 270°`.
- Si no hay yaw válido, se emite la coordenada GNSS sin corregir.

### 5.2 Detección de parada

- Entrada a `AVERAGING` tras `2 s` por debajo de `0.20 m/s`.
- Muestreo durante `15 s`.
- Paso a `LOCKED` con posición media válida.

### 5.3 Salida de lock

- Salida a `MOVING` si la velocidad supera `0.30 m/s`.
- Salida a `MOVING` si la distancia supera `1 m` comparando coordenadas del mismo marco (`raw` con `raw`).

### 5.4 Frescura y HDOP

- El HDOP de la salida proviene del campo 8 de la GGA de entrada.
- Solo se usa fallback `1.0` si el campo no existe o es inválido.
- Si la GGA deja de estar fresca, la salida se silencia.

---

## 6. Modo USB CDC

### Modo A — Diagnóstico

Usar `Serial` para ver mensajes de diagnóstico por USB.

### Modo B — Dynatest limpio

Usar `Serial` como flujo NMEA limpio hacia el enlace Dynatest/host.

### Restricción

No usar el mismo `Serial`/USB CDC simultáneamente para logs y para NMEA limpio.

---

## 7. Checklist operativo en banco/campo

### Antes de energizar

- [ ] Antena GNSS conectada
- [ ] Continuidad del arnés RJ45 verificada
- [ ] RJ45 etiquetado `NO ETHERNET`
- [ ] Breakout BNO085 alimentado solo por `VIN/5V`
- [ ] GND común correcto
- [ ] Bomba/motor apagados para la primera validación

### Validación inicial

- [ ] `Serial1` recibe GGA/RMC desde COM3 del UM980
- [ ] BNO085 responde por `Wire` a `100 kHz`
- [ ] LED1 y LED2 responden según estado
- [ ] Se observa `$GCGGA` con checksum y `CRLF`
- [ ] Tasa de salida de `10 Hz`
- [ ] HDOP cambia con el valor real de entrada
- [ ] Sin salida cuando vence el timeout de frescura

### Validación funcional

- [ ] `$GCGGA` en movimiento
- [ ] `$GCGGA` durante parada y `LOCKED`
- [ ] Confirmación de parada a `2 s`
- [ ] Promedio correcto a `15 s`
- [ ] Reentrada a `MOVING` al superar `0.30 m/s`
- [ ] Reentrada a `MOVING` al desplazar más de `1 m`

### Interferencia electromagnética

- [ ] Repetir prueba con bomba apagada
- [ ] Repetir prueba con bomba encendida
- [ ] Confirmar que el I2C y la salida NMEA se mantienen estables

---

## 8. Sketch objetivo y referencias históricas

### Sketch objetivo real

- `firmware/arduino/rs232_fwd_gps_uno_r4_wifi_rev_1_0.ino`

### Sketches históricos conservados

- `firmware/arduino/rs232_fwd_gps_final_v_0_99.ino`
- `firmware/arduino/rs232_fwd_gps_draft_v0_9.ino`

Los sketches históricos dependen de pines/APIs de ESP32-S3 y se conservan solo como referencia; **no representan el despliegue real UNO R4 WiFi**.
