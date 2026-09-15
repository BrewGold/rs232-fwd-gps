# ESP32-S3 Firmware

## Build

```bash
pio run
```

## Upload

```bash
pio run -t upload
```

## Serial monitor

```bash
pio device monitor -b 115200
```

## Módulos

- `gnss_uart`: parseo NMEA de entrada (GGA/RMC)
- `ntrip_client`: gestión de caster y RTCM
- `stop_detector`: detección de parada
- `position_averager`: promedio temporal 15 s
- `gga_output`: generación/salida GGA al Dynatest
- `led_status`: patrones LED_POWER y LED_GNSS
- `imu_service`: integración BNO085 (experimental)
