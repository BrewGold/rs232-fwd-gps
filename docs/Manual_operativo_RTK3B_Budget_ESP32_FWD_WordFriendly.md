MANUAL OPERATIVO  
simpleRTK3B Budget + ESP32-S3 UNO + FWD GCGGA + USB2/RS232 + Galileo HAS

Versión: 1.1  
Fecha: 17/09/2026  
Preparado para: Operación técnica / campo  
Referencia base: User Guide simpleRTK3B Budget (ArduSimple, mod. 2026/04/05)

====================================================

1. OBJETIVO
-----------

Implementar y validar un sistema donde:

- simpleRTK3B Budget / UM980 entrega GNSS al ESP32 por COM3 (TX3/RX3).
- COM3 entrega GGA, VTG/RMC y, durante la prueba HAS, PPPNAVA.
- COM1 / USB GPS se utiliza para configuración, diagnóstico y observación local.
- ESP32 procesa posición, velocidad, offset y lógica detenido/promedio 15 s.
- ESP32 reenvía GCGGA por:
  - COM2 (RX2) del RTK3B para visualización por USB2.
  - MAX3232 para salida RS232 al Dynatest, opcional.
- En detenido, salida promediada (15 s) a 10 Hz.
- Galileo HAS se configura en el UM980 y se observa simultáneamente por COM1 y COM3.

Importante:
- La arquitectura de HAS está preparada, pero el estado final exacto emitido por
  PPPNAVA debe confirmarse con una captura real del UM980 instalado.
- No asumir que el estado final se llama PPP_VALID. El firmware debe conservar y
  registrar la línea PPPNAVA real hasta cerrar el parser definitivo.

====================================================

2. MAPEO DE PUERTOS (RTK3B BUDGET)
-----------------------------------

Tabla 1. Puertos internos y externos

| Interfaz física RTK3B | Puerto UM980 | Uso recomendado |
|---|---|---|
| USB GPS | COM1 | Configuración, comandos y diagnóstico HAS |
| XBee socket + TX2/RX2 | COM2 | Retorno FWD + monitor por USB2 |
| Pixhawk + TX3/RX3 | COM3 | GGA/VTG/RMC/PPPNAVA hacia ESP32 |

Notas clave:
- Regla UART: TX origen -> RX destino.
- No inyectar datos hacia TX2/TX3 salvo que el procedimiento lo requiera.
- Para monitor limpio en USB2, COM2 debe quedar sin NMEA propio.
- COM1 y COM3 deben recibir los diagnósticos HAS durante la prueba.
- El ESP32 debe ignorar PPPNAVA/BESTNAVA para la salida Dynatest; solo debe
  reenviar/generar GPGGA.

====================================================

3. ARQUITECTURA DE SEÑAL
------------------------

Flujo principal:

1) RTK3B COM3 TX3 -> ESP32 GNSS RX  
2) COM3 entrega GGA + VTG/RMC + PPPNAVA de diagnóstico  
3) ESP32 procesa posición, velocidad, estado y offset  
4) ESP32 OUT TX -> RTK3B COM2 RX2 (monitor USB2)  
5) ESP32 OUT TX -> MAX3232 -> Dynatest Compact15 (RS232)  
6) COM1 / USB GPS permite observar configuración y PPPNAVA directamente

Durante una prueba HAS:

- COM1 muestra los comandos, respuestas y diagnósticos locales.
- COM3 lleva los mismos diagnósticos necesarios hacia el ESP32.
- COM2 queda reservado para el retorno GCGGA del ESP32.
- Dynatest recibe únicamente GCGGA a 38400 baudios y 10 Hz.

====================================================

4. REQUISITOS PREVIOS
---------------------

- Antena GNSS conectada antes de energizar.
- Vista de cielo adecuada y sin obstrucciones.
- Alimentación estable.
- GND común entre RTK3B, ESP32 y MAX3232.
- Firmware UM980 compatible con E6-HAS.
- PC con monitor serie configurado a 115200, 8N1 y final de línea CR+LF.
- Driver FTDI VCP instalado si PC no detecta puertos:
  https://ftdichip.com/drivers/vcp-drivers/

Para una prueba HAS válida:

- No activar NTRIP/RTCM externo durante la comprobación.
- Mantener la antena fija y con cielo abierto.
- No apagar el receptor durante la convergencia.
- Registrar el texto completo de PPPNAVA y BESTNAVA.

====================================================

5. CONFIGURACIÓN RTK3B POR COMANDOS (SIN GUI)
----------------------------------------------

Conectarse por USB GPS (COM1), típicamente a 115200, 8N1, CR+LF.
Enviar los comandos uno por uno y comprobar la respuesta del receptor.

5.1 Configurar COM3 para alimentar ESP32

Comandos:

GPGGA COM3 1
GPVTG COM3 1
GPRMC COM3 1

Resultado esperado:
- COM3 emite posición y calidad de fix.
- COM3 emite velocidad/rumbo para detectar detenido/movimiento.

5.2 Configurar diagnóstico HAS en COM3

Para que el ESP32 pueda registrar el estado PPP/HAS que llega desde el UM980:

PPPNAVA COM3 1
BESTNAVA COM3 1

Si el firmware del UM980 no acepta la forma con puerto, probar la sintaxis
básica y verificar después con UNILOGLIST:

PPPNAVA 1
BESTNAVA 1

Resultado esperado en COM3:

$GPGGA,...
$GPVTG,...
$GPRMC,...
#PPPNAVA,...
#BESTNAVA,...

El ESP32 debe procesar GGA/VTG/RMC y conservar PPPNAVA/BESTNAVA como diagnóstico.
No debe reenviar esas líneas al Dynatest.

5.3 Configurar diagnóstico HAS en COM1 / USB GPS

Para observar el estado directamente en el monitor serie del PC:

GPGGA COM1 1
GPGSA COM1 1
GPGSV COM1 1
GPGST COM1 1
GPRMC COM1 1
PPPNAVA COM1 1
BESTNAVA COM1 1

COM1 debe mostrar las respuestas a los comandos y, a continuación, las líneas
NMEA y de diagnóstico del UM980.

5.4 Limpiar COM2 para evitar mezcla en USB2

Comandos:

GPGGA COM2 0
GPGSA COM2 0
GPGSV COM2 0
GPGST COM2 0
GPRMC COM2 0
GPVTG COM2 0
PPPNAVA COM2 0
BESTNAVA COM2 0

Resultado esperado:
- COM2 queda sin NMEA ni diagnóstico propio del RTK3B.
- USB2 muestra principalmente lo que inyecta el ESP32.

5.5 Activar Galileo HAS en Budget

Comandos:

CONFIG PPP ENABLE E6-HAS
CONFIG PPP DATUM WGS84
CONFIG PPP CONVERGE 50 50
CONFIG SIGNALGROUP 2

Guardar:

SAVECONFIG

Desactivar PPP/HAS si se necesita volver a GNSS normal:

CONFIG PPP DISABLE
SAVECONFIG

====================================================

6. SECUENCIA COMPLETA HAS (COPIAR/PEGAR)
-----------------------------------------

Conectado a COM1 / USB GPS, enviar una línea cada vez:

GPGGA COM3 1
GPVTG COM3 1
GPRMC COM3 1
PPPNAVA COM3 1
BESTNAVA COM3 1

GPGGA COM1 1
GPGSA COM1 1
GPGSV COM1 1
GPGST COM1 1
GPRMC COM1 1
PPPNAVA COM1 1
BESTNAVA COM1 1

GPGGA COM2 0
GPGSA COM2 0
GPGSV COM2 0
GPGST COM2 0
GPRMC COM2 0
GPVTG COM2 0
PPPNAVA COM2 0
BESTNAVA COM2 0

CONFIG PPP ENABLE E6-HAS
CONFIG PPP DATUM WGS84
CONFIG PPP CONVERGE 50 50
CONFIG SIGNALGROUP 2
SAVECONFIG

UNILOGLIST

UNILOGLIST se utiliza para confirmar qué mensajes están activos y en qué puerto.
La respuesta exacta puede variar según el firmware del UM980.

====================================================

7. QUÉ CAPTURAR PARA CERRAR EL PARSER HAS
------------------------------------------

La captura de campo es obligatoria antes de fijar definitivamente la clasificación
PPP en el firmware.

7.1 Captura de convergencia

En COM1 o en el registro de COM3, esperar una línea que contenga:

PPP_CONVERGING

Guardar 10-20 líneas completas consecutivas, incluyendo:

#PPPNAVA,...
#BESTNAVA,...
#PPPNAVA,...
#BESTNAVA,...

No modificar las líneas ni quitar checksums.

7.2 Captura estable

Después de dejar el receptor con cielo abierto el tiempo necesario, guardar otras
10-20 líneas completas cuando ya no aparezca PPP_CONVERGING.

No asumir previamente que el estado final se llama PPP_VALID. Puede variar según
el build del UM980. La clasificación final debe basarse en el texto real capturado.

7.3 Información adicional

Incluir una vez en la captura:

VERSIONA
UNILOGLIST

Guardar también:
- versión/build del UM980;
- fecha y hora de la prueba;
- ubicación aproximada;
- si había o no NTRIP/RTCM;
- calidad de cielo y antena utilizada.

====================================================

8. ESTADOS HAS ESPERADOS
------------------------

Estados de diagnóstico que pueden observarse:

- SIN_PPP: solución autónoma o sin solución PPP/HAS confirmada.
- PPP_CONVERGING: PPP/HAS está calculando y convergiendo.
- PPP_ESTABLE: estado PPP estable confirmado por captura real del equipo.
- PPP_DESCONOCIDO: llega PPPNAVA, pero el literal no coincide con los estados
  conocidos y debe conservarse para revisión.

Regla de ingeniería:

- PPP_CONVERGING se puede reconocer por el literal recibido.
- El estado estable no debe llamarse PPP_VALID hasta confirmarlo en campo.
- GGA fixQ se conserva inicialmente tal como lo entrega el UM980.
- No inventar un fixQ NMEA para HAS sin una política documentada y validada.

====================================================

9. CONFIGURACIÓN ESP32 (REFERENCIA DE FIRMWARE)
------------------------------------------------

Firmware operativo de referencia:

firmware/arduino/firmware_arduino_rs232_fwd_gps_unificado.ino

Draft separado para evolución:

firmware/arduino/rs232_fwd_gps_draft_v0_9.ino

El draft no reemplaza al firmware operativo y queda pendiente de datos reales
de PPPNAVA, parser definitivo, BNO085, offset y GGA final.

Parámetros críticos del enlace operativo:

- GNSS_BAUD = 115200 (igual a COM3).
- OUT_BAUD = 38400 (Dynatest/COM2 según la arquitectura final).
- AVG_WINDOW_MS = 15000.
- OUT_PERIOD_MS = 100 (10 Hz).
- Stop enter <= 0.20 m/s.
- Stop exit >= 0.30 m/s.
- Confirmación de detenido: 2 s.

====================================================

10. QUÉ VER EN CADA MONITOR
---------------------------

| Monitor | Qué debe verse | Objetivo |
|---|---|---|
| USB GPS / COM1 | OK, NMEA nativo, PPPNAVA, BESTNAVA | Configurar y verificar HAS |
| COM3 / TX3 | GGA, VTG/RMC, PPPNAVA, BESTNAVA | Alimentar ESP32 y transportar diagnóstico |
| USB debug ESP32 | GGA recibida, estado PPPNAVA bruto, INST/AVG15s | Validar lógica interna |
| USB2 / COM2 | GCGGA del ESP32 a 10 Hz, sin mezcla | Validar retorno FWD |
| Dynatest / RS232 | Solo GCGGA a 38400 y 10 Hz | Validar entrada del Compact15 |

Ejemplo de líneas que deben verse en COM1 y COM3 durante la prueba:

$GPGGA,...
$GPVTG,...
$GPRMC,...
#PPPNAVA,...PPP_CONVERGING...
#BESTNAVA,...

Cuando exista una solución estable, conservar también 10-20 líneas reales del
estado posterior para cerrar el parser definitivo.

====================================================

11. PROCEDIMIENTO DE VALIDACIÓN (PASO A PASO)
----------------------------------------------

1. Verificar antena, alimentación y cielo abierto.
2. Conectar COM1 / USB GPS a 115200, 8N1, CR+LF.
3. Ejecutar VERSIONA y UNILOGLIST; guardar la respuesta.
4. Configurar COM3 con GGA + VTG/RMC.
5. Activar PPPNAVA y BESTNAVA en COM1 y COM3.
6. Limpiar COM2 y guardar la configuración.
7. Activar HAS con los comandos de la sección 6.
8. Confirmar PPP_CONVERGING en COM1 y COM3.
9. Registrar 10-20 líneas consecutivas de PPPNAVA/BESTNAVA.
10. Esperar la solución estable y registrar otras 10-20 líneas.
11. Cargar el firmware operativo en el ESP32.
12. Confirmar que el ESP32 ignora PPPNAVA/BESTNAVA para la salida Dynatest.
13. En movimiento confirmar OUT[INST].
14. Detener 2-3 s y confirmar OUT[AVG15s].
15. Abrir USB2 y confirmar GPGGA a 10 Hz sin mezcla.
16. Confirmar recepción RS232 en Dynatest.
17. Solo después de la captura real, cerrar el parser definitivo de HAS.

Criterio de aceptación:
- COM1 y COM3 muestran PPPNAVA/BESTNAVA durante la prueba HAS.
- El estado PPP_CONVERGING queda registrado.
- El estado estable queda registrado con texto real del UM980.
- El ESP32 mantiene la salida GCGGA aunque el estado PPP sea desconocido.
- USB2 queda limpio y estable a 10 Hz.
- Dynatest recibe solo GCGGA a 38400.
- No hay doble aplicación del offset.

====================================================

12. TROUBLESHOOTING
-------------------

Caso A: ESP32 no recibe GNSS
- Revisar TX3 -> RX ESP32.
- Revisar baud COM3 vs GNSS_BAUD.
- Revisar GND común.
- Confirmar que COM3 emite GGA con un adaptador/monitor apropiado.

Caso B: No aparece PPPNAVA en COM1
- Ejecutar PPPNAVA 1.
- Comprobar la respuesta del comando.
- Ejecutar UNILOGLIST.
- Revisar versión/build con VERSIONA.

Caso C: PPPNAVA aparece en COM1 pero no en COM3
- Ejecutar PPPNAVA COM3 1.
- Si devuelve error, probar PPPNAVA 1 y revisar UNILOGLIST.
- Confirmar que el cable TX3 -> ESP32 no esté saturado por una velocidad incorrecta.

Caso D: COM3 mezcla o satura el enlace
- Mantener solo GGA, VTG/RMC y PPPNAVA durante la primera prueba.
- No activar GSV de alta frecuencia en COM3 salvo necesidad.
- El ESP32 debe ignorar mensajes no necesarios para el flujo de salida.

Caso E: USB2 muestra tramas mezcladas
- Desactivar NMEA y PPPNAVA/BESTNAVA de COM2.
- Ejecutar SAVECONFIG.
- Verificar que la inyección ESP32 entra por RX2.

Caso F: No entra en AVG15s
- Confirmar que llega VTG o RMC válido.
- Revisar umbrales de velocidad.
- Confirmar que no se esté usando un estado de velocidad obsoleto.

Caso G: Dynatest no recibe datos
- Confirmar MAX3232 y cruce TX/RX.
- Confirmar 38400, 8N1.
- Confirmar que el firmware genera GCGGA con checksum.
- Confirmar que COM2 no está conectado directamente al Dynatest por error.

====================================================

13. BLOQUES RÁPIDOS (COPIAR/PEGAR)
-----------------------------------

13.1 Producción mínima sin HAS

GPGGA COM3 1
GPVTG COM3 1
GPRMC COM3 1
GPGGA COM2 0
GPGSA COM2 0
GPGSV COM2 0
GPGST COM2 0
GPRMC COM2 0
GPVTG COM2 0
PPPNAVA COM2 0
BESTNAVA COM2 0
SAVECONFIG

13.2 Producción + HAS visible en COM1 y COM3

GPGGA COM3 1
GPVTG COM3 1
GPRMC COM3 1
PPPNAVA COM3 1
BESTNAVA COM3 1
GPGGA COM1 1
GPGSA COM1 1
GPGSV COM1 1
GPGST COM1 1
GPRMC COM1 1
PPPNAVA COM1 1
BESTNAVA COM1 1
GPGGA COM2 0
GPGSA COM2 0
GPGSV COM2 0
GPGST COM2 0
GPRMC COM2 0
GPVTG COM2 0
PPPNAVA COM2 0
BESTNAVA COM2 0
CONFIG PPP ENABLE E6-HAS
CONFIG PPP DATUM WGS84
CONFIG PPP CONVERGE 50 50
CONFIG SIGNALGROUP 2
SAVECONFIG
UNILOGLIST

13.3 Desactivar diagnósticos después de capturar

PPPNAVA COM1 0
BESTNAVA COM1 0
PPPNAVA COM3 0
BESTNAVA COM3 0
SAVECONFIG

Si la sintaxis con puerto no es aceptada, utilizar la forma que indique
UNILOGLIST y confirmar el resultado en COM1 y COM3.

====================================================

14. CHECKLIST DE CAMPO (IMPRIMIBLE)
------------------------------------

Datos de ensayo:
- Fecha: __________________
- Técnico: _______________
- Ubicación: ______________
- Firmware ESP32: _________
- Firmware/build UM980: ___
- Baud COM1: _____________
- Baud COM3: _____________
- Baud COM2: _____________

Pre-check:
[ ] Antena conectada antes de power  
[ ] Cielo abierto suficiente  
[ ] GND común confirmado  
[ ] COM1 accesible por USB GPS  
[ ] Firmware/build UM980 registrado  
[ ] Sin NTRIP/RTCM externo durante prueba HAS  

Configuración:
[ ] COM3 con GGA + VTG/RMC  
[ ] PPPNAVA activo en COM1  
[ ] BESTNAVA activo en COM1  
[ ] PPPNAVA activo en COM3  
[ ] BESTNAVA activo en COM3  
[ ] COM2 sin NMEA propio  
[ ] COM2 sin PPPNAVA/BESTNAVA  
[ ] HAS configurado  
[ ] SAVECONFIG ejecutado  
[ ] VERSIONA y UNILOGLIST guardados  

Captura HAS:
[ ] 10-20 líneas PPP_CONVERGING guardadas  
[ ] 10-20 líneas estado estable guardadas  
[ ] Checksums conservados  
[ ] Estado final no asumido sin evidencia  

Validación:
[ ] ESP32 recibe GGA desde COM3  
[ ] ESP32 registra PPPNAVA sin reenviarlo al Dynatest  
[ ] ESP32 muestra OUT[INST] en movimiento  
[ ] ESP32 muestra OUT[AVG15s] detenido  
[ ] USB2 muestra GCGGA a 10 Hz  
[ ] Dynatest recibe GCGGA a 38400  
[ ] RS232 externo OK  

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
