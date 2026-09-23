# SimuladorCorriente

PWA para operar un instrumento de simulación y medición de señales 4–20 mA mediante Web Serial con HC-05 Bluetooth SPP cuando el navegador/sistema operativo lo expone como puerto serie.

## Funciones principales

- Salida manual 4–20 mA con resolución de 0.1 mA.
- Modo simulación sin hardware.
- Entrada 4–20 mA con mínimo, máximo, promedio y gráfica.
- Rampas lineales, triangulares, escalonadas, cíclicas y multipunto.
- Perfiles guardados en `localStorage`, importación/exportación JSON y carga al instrumento.
- Calibración y escalado independiente para salida y entrada.
- Variables: temperatura, presión, caudal, posición angular, corriente, tensión y personalizada.
- Conversión automática entre mA y unidades de ingeniería configurables.
- Exportación CSV.
- PWA instalable y soporte offline.
- Web Serial + protocolo ASCII por líneas.

## Protocolo base

- `SET:CURRENT:12.500`
- `SET:OUTPUT:ON`
- `SET:OUTPUT:OFF`
- `SET:OUTPUT:OPEN`
- `GET:STATUS`
- `PROFILE:NEW:n:cantidad`
- `PROFILE:POINT:n:indice:tiempo:ma`
- `PROFILE:SAVE:n`
- `PROFILE:RUN:n`
- Respuesta de datos: `DATA:OUT=12.500:IN=12.480`

## Uso

Para Web Serial se requiere HTTPS o localhost y un navegador compatible. En Android, emparejar primero el HC-05 con el sistema y luego abrir la PWA en Chrome compatible.

## Despliegue en Netlify

Este repositorio está preparado para despliegue continuo desde Netlify.

- Rama de producción: `main`
- Build command: ninguno
- Publish directory: `.`
- Configuración: `netlify.toml`

Netlify debe quedar conectado directamente a `MoyaPIC/SimuladorCorriente`. Cada cambio en `main` puede generar un nuevo despliegue automáticamente.
