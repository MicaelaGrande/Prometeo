// CLASE SÁBADO 31/10

#define triggerPin 5 // Envía el pulso de sonido (Trigger)
#define echoPin 18   // Escucha el rebote del sonido (Echo)
#define led1Pin 19   // LED 1 (rango de 0 a 10 cm)
#define led2Pin 21   // LED 2 (rango de 11 a 20 cm)
#define led3Pin 22   // LED 3 (rango de 21 a 30 cm)

int delaymili = 200; // Tiempo de pausa entre lecturas (200 milisegundos)
int maximumRangeCm = 300; // Límite máximo de detección en cm
int minimumRangeCm = 0;   // Límite mínimo de detección en cm

long duration, distanceCm; // Variables del tiempo de vuelo y la distancia calculada

void setup() {
  Serial.begin(115200); // Inicia la comunicación con la PC a 115200 baudios
  
  pinMode(triggerPin, OUTPUT); // Trigger como SALIDA
  pinMode(echoPin, INPUT);     // Echo como ENTRADA
  
  pinMode(led1Pin, OUTPUT);    // LED 1 como SALIDA
  pinMode(led2Pin, OUTPUT);    // LED 2 como SALIDA
  pinMode(led3Pin, OUTPUT);    // LED 3 como SALIDA
}

void loop() {
  initTrigger(); // Llama a la función que dispara el pulso ultrasónico
  
  duration = pulseIn(echoPin, HIGH); // Mide en microsegundos cuánto tarda en volver el pulso
  distanceCm = microsecCm(duration); // Convierte ese tiempo a centímetros usando la función de abajo
  
  // Apaga todos los LEDs antes de evaluar la nueva distancia para evitar que queden trabados
  digitalWrite(led1Pin, LOW); // Apaga LED 1
  digitalWrite(led2Pin, LOW); // Apaga LED 2
  digitalWrite(led3Pin, LOW); // Apaga LED 3

  // Evalúa si la distancia es un error (0) o está fuera del alcance útil (mayor a 300 cm)
  if (distanceCm >= maximumRangeCm || distanceCm <= minimumRangeCm) {
    Serial.println("Fuera de rango"); // Informa en el monitor que no detecta nada válido
  } 
  else {
    Serial.print("Distancia en cm: "); 
    Serial.println(distanceCm);        // Escribe el número medido

    // Si el objeto está entre 1 y 10 cm, enciende solo el primer LED
    if (distanceCm > 0 && distanceCm <= 10) {
      digitalWrite(led1Pin, HIGH);
      digitalWrite(led2Pin, LOW);
      digitalWrite(led3Pin, LOW);
    }
    // Si el objeto está entre 11 y 20 cm, enciende los dos primeros LEDs
    else if (distanceCm > 10 && distanceCm <= 20) {
      digitalWrite(led1Pin, HIGH);
      digitalWrite(led2Pin, HIGH);
      digitalWrite(led3Pin, LOW);
    }
    // Si el objeto está entre 21 y 30 cm, enciende los tres LEDs juntos
    else if (distanceCm > 20 && distanceCm <= 30) {
      digitalWrite(led1Pin, HIGH);
      digitalWrite(led2Pin, HIGH);
      digitalWrite(led3Pin, HIGH);
    }
  }
  
  delay(delaymili); // Espera 200 milisegundos antes de repetir todo el ciclo de medición
}

// Función que convierte el tiempo de vuelo del sonido en centímetros
long microsecCm(long microsecond) {
  return microsecond / 58; // Se divide por 58 porque el sonido tarda aprox 58.8us en recorrer 2 cm (ida y vuelta)
}

// Función que genera la secuencia de disparo que exige el HC-SR04
void initTrigger() {
  digitalWrite(triggerPin, LOW);  // Asegura que el pin arranque apagado para tener una señal limpia
  delayMicroseconds(2);           // Espera 2 microsegundos
  digitalWrite(triggerPin, HIGH); // Enciende el pin para mandar la ráfaga de sonido
  delayMicroseconds(10);          // Lo mantiene encendido 10 microsegundos (tiempo mínimo requerido)
  digitalWrite(triggerPin, LOW);  // Apaga el pin, terminando el disparo
}