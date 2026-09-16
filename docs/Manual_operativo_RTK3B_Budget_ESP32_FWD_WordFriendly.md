MANUAL OPERATIVO  
simpleRTK3B Budget + ESP32-S3 UNO + FWD GPGGA + USB2/RS232

Versión: 1.0  
Fecha: 16/09/2026  
Preparado para: Operación técnica / campo  
Referencia base: User Guide simpleRTK3B Budget (ArduSimple, mod. 2026/04/05)

====================================================

1. OBJETIVO
-----------

Implementar y validar un sistema donde:

- simpleRTK3B Budget entrega GNSS al ESP32 por COM3 (TX3/RX3).
- ESP32 procesa offset + lógica detenido/promedio 15 s.
- ESP32 reenvía GPGGA por:
  - COM2 (RX2) del RTK3B para visualización por USB2.
  - MAX3232 para salida RS232 (opcional).
- En detenido, salida promediada (15 s) a 10 Hz.

====================================================

2. MAPEO DE PUERTOS (RTK3B BUDGET)
----------------------------------

Tabla 1. Puertos internos y externos

| Interfaz física RTK3B | Puerto UM980 | Uso recomendado |
|---|---|---|
| USB GPS | COM1 | Configuración/comandos/diagnóstico |
| XBee socket + TX2/RX2 | COM2 | Retorno FWD + monitor por USB2 |
| Pixhawk + TX3/RX3 | COM3 | Salida GNSS hacia ESP32 |

Notas clave:
- Regla UART: TX origen -> RX destino.
- No inyectar datos hacia TX2/TX3.
- Para monitor limpio en USB2, COM2 debe quedar sin NMEA propio.

====================================================

3. ARQUITECTURA DE SEÑAL
------------------------

Flujo principal:

1) RTK3B COM3 TX3 -> ESP32 GNSS RX  
2) ESP32 procesa (INST / AVG15s)  
3) ESP32 OUT TX -> RTK3B COM2 RX2 (monitor USB2)  
4) ESP32 OUT TX -> MAX3232 -> RX equipo RS232 (opcional)

====================================================

4. REQUISITOS PREVIOS
---------------------

- Antena GNSS conectada antes de energizar.
- Vista de cielo adecuada.
- Alimentación estable.
- GND común entre RTK3B, ESP32 y MAX3232.
- Driver FTDI VCP instalado si PC no detecta puertos:
  https://ftdichip.com/drivers/vcp-drivers/

====================================================

5. CONFIGURACIÓN RTK3B POR COMANDOS (SIN GUI)
----------------------------------------------

Conectarse por USB GPS (COM1), típicamente a 115200.

5.1 Configurar COM3 para alimentar ESP32

Comandos:
GPGGA COM3 1
GPVTG COM3 1
GPRMC COM3 1
SAVECONFIG

Resultado esperado:
- COM3 emite posición + velocidad útil para detectar detenido/movimiento.

5.2 Limpiar COM2 para evitar mezcla en USB2

Comandos:
GPGGA COM2 0
GPGSA COM2 0
GPGSV COM2 0
GPGST COM2 0
GPRMC COM2 0
GPVTG COM2 0
SAVECONFIG

Resultado esperado:
- COM2 sin NMEA propio del RTK3B.
- USB2 muestra principalmente lo que inyecta ESP32.

5.3 Activar Galileo HAS (PPP) en Budget

Comandos:
CONFIG PPP ENABLE E6-HAS
CONFIG PPP DATUM WGS84
CONFIG PPP CONVERGE 50 50
CONFIG SIGNALGROUP 2
SAVECONFIG

Desactivar PPP:
CONFIG PPP DISABLE
SAVECONFIG

====================================================

6. CONFIGURACIÓN ESP32 (REFERENCIA DE FIRMWARE)
------------------------------------------------

Archivo:
firmware/arduino/rs232_fwd_gps_unificado.ino

Parámetros críticos:
- GNSS_BAUD = 115200 (igual a COM3)
- OUT_BAUD = 115200 (igual a COM2/USB2 y RS232 destino)
- AVG_WINDOW_MS = 15000
- OUT_PERIOD_MS = 100 (10 Hz)
- Stop enter <= 0.20 m/s, exit >= 0.30 m/s, confirmación 2 s

====================================================

7. QUÉ VER EN CADA MONITOR
--------------------------

Tabla 2. Monitoreo esperado

| Monitor | Qué debe verse | Objetivo |
|---|---|---|
| USB1 / COM1 (RTK3B) | Respuestas OK a comandos, GNSS nativo | Verificar estado receptor y configuración |
| USB debug ESP32 | OUT[INST] en movimiento; OUT[AVG15s] en detenido; línea GPGGA | Validar lógica interna |
| USB2 / COM2 | GPGGA continuo a 10 Hz, sin mezcla | Validar retorno FWD final |
| Equipo RS232 | Mismo GPGGA lógico que USB2 | Validar salida a terceros |

====================================================

8. PROCEDIMIENTO DE VALIDACIÓN (PASO A PASO)
---------------------------------------------

1. Verificar antena y alimentación.
2. Aplicar comandos COM3 (GGA + VTG/RMC).
3. Limpiar COM2 (frecuencias en 0) y guardar.
4. (Opcional) Activar HAS y guardar.
5. Cargar firmware en ESP32.
6. Abrir monitor debug ESP32.
7. En movimiento confirmar OUT[INST].
8. Detener 2–3 s y confirmar OUT[AVG15s].
9. Abrir USB2 y confirmar GPGGA a 10 Hz.
10. (Opcional) Confirmar recepción RS232 externa.

Criterio de aceptación:
- Sistema alterna correctamente INST <-> AVG15s.
- USB2 limpio y estable a 10 Hz.
- Sin colisión de tramas en COM2.

====================================================

9. TROUBLESHOOTING
------------------

Caso A: ESP32 no recibe GNSS
- Revisar TX3->RX ESP32.
- Revisar baud COM3 vs GNSS_BAUD.
- Revisar GND común.

Caso B: USB2 no muestra retorno
- Verificar inyección a RX2 (COM2 RX).
- Revisar baud COM2 vs OUT_BAUD.
- Verificar bridge USB2<->COM2.

Caso C: Tramas mezcladas en USB2
- Desactivar NMEA de COM2 (todos a 0).
- Ejecutar SAVECONFIG.

Caso D: No entra en AVG15s
- Confirmar que llega VTG o RMC.
- Ajustar umbrales de velocidad.

Caso E: RS232 sin datos
- Revisar MAX3232 y cableado.
- Confirmar baud y GND.

====================================================

10. BLOQUES RÁPIDOS (COPIAR/PEGAR)
----------------------------------

10.1 Producción mínima (sin HAS)

GPGGA COM3 1
GPVTG COM3 1
GPRMC COM3 1
GPGGA COM2 0
GPGSA COM2 0
GPGSV COM2 0
GPGST COM2 0
GPRMC COM2 0
GPVTG COM2 0
SAVECONFIG

10.2 Producción + HAS

GPGGA COM3 1
GPVTG COM3 1
GPRMC COM3 1
GPGGA COM2 0
GPGSA COM2 0
GPGSV COM2 0
GPGST COM2 0
GPRMC COM2 0
GPVTG COM2 0
CONFIG PPP ENABLE E6-HAS
CONFIG PPP DATUM WGS84
CONFIG PPP CONVERGE 50 50
CONFIG SIGNALGROUP 2
SAVECONFIG

====================================================

11. CHECKLIST DE CAMPO (IMPRIMIBLE)
-----------------------------------

Datos de ensayo:
- Fecha: __________________
- Técnico: _______________
- Ubicación: ______________
- Firmware ESP32: _________
- Baud COM3: _____________
- Baud COM2: _____________

Pre-check:
[ ] Antena conectada antes de power  
[ ] Cielo abierto suficiente  
[ ] GND común confirmado  
[ ] COM1 accesible por USB GPS  

Configuración:
[ ] COM3 con GGA + VTG/RMC  
[ ] COM2 sin NMEA propio  
[ ] SAVECONFIG ejecutado  
[ ] HAS configurado (si aplica)  

Validación:
[ ] ESP32 muestra OUT[INST] en movimiento  
[ ] ESP32 muestra OUT[AVG15s] detenido  
[ ] USB2 muestra GPGGA a 10 Hz  
[ ] RS232 externo OK (si aplica)  

Resultado final:
[ ] APROBADO  
[ ] OBSERVADO  

Observaciones:
____________________________________________________
____________________________________________________
____________________________________________________

Firmas:
Técnico: _____________________   Fecha: ___/___/____
Supervisor: __________________   Fecha: ___/___/____
