# Firmware Simulink Controller

Firmware para Arduino UNO / ATmega328P compatible con la PWA Simulink.

## Pinout
- D6: buzzer
- D8: entrada de error de lazo 4-20 mA
- D9: RX SoftwareSerial, conectado al TX del HC-05
- D10: TX SoftwareSerial, conectado al RX del HC-05
- A4/SDA y A5/SCL: bus I2C compartido

## I2C por defecto
- MCP4725: 0x60
- ADS1115: 0x48
- 24C512: 0x50

## Funciones
- Generación 4.0-20.0 mA con MCP4725
- Medición de entrada con ADS1115
- Calibración de dos puntos de salida y entrada
- 16 perfiles guardados en 24C512
- Hasta 988 puntos por perfil
- Ejecución autónoma no bloqueante con interpolación
- CRC16 de perfiles
- Comunicación HC-05 a 9600 baud
- Monitor USB a 115200 baud
- Detección de lazo abierto/cerrado en D8
- Buzzer de alarma
- Protocolo ASCII compatible con Simulink

## Ajustes importantes
En el encabezado del sketch se puede cambiar:
- polaridad activa de D8 mediante LOOP_ERROR_ACTIVE_LEVEL
- uso de pull-up en D8 mediante LOOP_ERROR_USE_PULLUP
- buzzer activo/pasivo mediante BUZZER_PASSIVE

El valor inicial supone HIGH = lazo abierto.

## Nota sobre SET:OUTPUT:OFF
No se indicó un pin físico de habilitación/desconexión de salida. Por seguridad, OFF mantiene 4.0 mA. Si la placa incorpora un transistor, relé o enable para abrir físicamente la salida, se puede agregar ese pin al firmware.
