# PRACTICA-8
## Comunicación Serie Bidireccional entre UART0 y UART2

## CUÁL ES EL FIN DE ESTA PRÁCTICA? 

Implementar una comunicación bidireccional constante entre dos interfaces UART (UART0 y UART2) en un ESP32, facilitando el reenvío de datos entre un teclado conectado mediante terminal serie y un segundo puerto UART simulado.

### MATERIAL
El sistema utiliza una placa ESP32-S1, que se conecta al monitor serie del ordenador para la comunicación y supervisión de datos. La entrada de datos se realiza a través de un teclado conectado mediante el terminal serie, y para facilitar la transmisión se emplea un cable puente que conecta el pin TX2 con RX2 directamente en la placa.


## CÓDIGO

```cpp
#include <Arduino.h>

void setup() {
  Serial.begin(115200);   // Inicializar Serial (UART0)
  Serial2.begin(115200);  // Inicializar Serial2 (UART2)
}

void loop() {
  // Leer datos de la UART0 y enviarlos a la UART2
  if (Serial.available()) {
    char c = Serial.read();
    Serial2.write(c);
  }
  
  // Leer datos de la UART2 y enviarlos a la UART0
  if (Serial2.available()) {
    char c = Serial2.read();
    Serial.write(c);
  }
}
```
## CONCLUSIONES

El programa configura una comunicación bidireccional entre los puertos UART0 y UART2 del ESP32. Durante la ejecución de la función setup(), ambos puertos se inicializan con una velocidad de 115200 baudios. Posteriormente, en la función loop(), se verifica de forma continua si existen datos disponibles en cualquiera de los dos puertos. Cuando se detecta información en UART0, esta se lee y se transmite hacia UART2; de igual manera, si la información proviene de UART2, se reenvía a UART0. Así se establece un ciclo constante de envío y recepción de datos entre ambos puertos.

## EJERCICIO 2 (OPCIONAL)
## CÓDIGO

```cpp

#include <TinyGPS.h>
#include <HardwareSerial.h>

TinyGPS gps;
HardwareSerial SerialGPS(1); // Usamos UART1 (puedes cambiar los pines si es necesario)

void setup() {
  Serial.begin(115200); // Comunicación con PC
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17); // Comunicación con GPS, RX=16, TX=17
  Serial.println("Iniciando recepción de datos GPS...");
}

void loop() {
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // Intentamos recibir datos durante un segundo
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (SerialGPS.available()) {
      char c = SerialGPS.read();
      if (gps.encode(c)) {
        newData = true;
      }
    }
  }

  if (newData) {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.println(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }

  gps.stats(&chars, &sentences, &failed);
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
}

```
## Conclusion
Este programa establece la comunicación con un módulo GPS externo utilizando la biblioteca TinyGPS y un puerto UART adicional denominado SerialGPS, empleando para ello los pines GPIO16 (RX) y GPIO17 (TX) del ESP32.

Durante la fase de configuración en setup(), se inicializan tanto la conexión serie con el ordenador como la comunicación serial con el GPS, lo que permite recibir datos del módulo GPS mientras se supervisa la información en el monitor serie del PC.

En la función loop(), el programa permanece activo durante intervalos de un segundo, leyendo continuamente los datos que llegan por SerialGPS. Cada carácter recibido se envía al objeto gps, encargado de interpretar las sentencias NMEA. Cuando se detecta una secuencia válida, se extraen e imprimen en el monitor serie los siguientes datos:

Latitud
Longitud
Número de satélites visibles
Precisión de la señal
Además, se presentan estadísticas relevantes como:

Total de caracteres procesados
Cantidad de frases NMEA válidas
Número de errores de checksum
Este ejemplo proporciona una forma práctica de visualizar en tiempo real información geográfica, facilitando la comprensión del funcionamiento básico de los módulos GPS y la interpretación de sus mensajes.
