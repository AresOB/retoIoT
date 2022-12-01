#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "MQ135.h"



#define MQTT_TRANSMIT_RATE_MS 500 // Se define el periodo de publicación de mensajes de MQTT
#define DHT_PIN 2 // Se define el pin Digital al que estará conectado el sensor DHT11 (GPIO2)
#define SENSOR_READ_TIMEOUT_US 1000 // Es el tiempo máximo de espera en micros segundos para recibir un bitframe del sensor
#define SENSOR_READ_TIMEOUT_FLAG -1
#define VOLTAGE_MEASURES_DELAY_MS   20 //ms

#define RAIN_PIN 4


MQ135 mq135;
const byte analogInputPin = A0;
const float R0 = 16852.10; //calibración
const float ADC_VOLTAGE = 3.3; //  V
const int ADC_MAX_VALUE = 1023; // valor en bits

const float a_CO = 578.15;
const float b_CO = -3.916;


const float a_CO2 = 113.42;
const float b_CO2 = -2.859;


#define VOLTAGE_MEASURES            10


int pulse_time[80]; // Arreglo de 80 enteros donde se almacenará la duración en microsegundos de los diferentes pulsos recibidos desde el sensor.
bool bit_frame[40]; // Arreglo de 40 valores booleanos donde se almacenarán los 40 bits recibidos por la trama de datos.
uint8_t bytes_array[5];

// **********************************************
// Modifica los siguientes rubros con los valores adecuados
// para establecer la comunicación con el servidor MQTT.
// **********************************************
const char* SSID = "Tec-IoT"; // Se define el nombre de la red a la cual se conectará el módulo NodeMCU
const char* PASSWORD = "spotless.magnetic.bridge"; // Se define la contraseña de la red

const char* MQTT_SERVER = "10.48.64.176"; //Es la dirección IP del equipo donde corre el servidor MQTT.
const char* CLIENT_ID = "EQUIPO-10"; // Es un identificador único del cliente conectado. Utiliza el número de tu equipo para asignar este valor P.ej: "EQUIPO-1"
const int MQTT_BROKER_PORT = 1883; // Es el puerto de comunicación (puerto de red) asignado al servidor MQTT. 
 

const char* topic_temperature = "/temperature"; // Es el nombre del tópico 1 al cual se publicarán los mensajes.
const char* topic_humidity = "/humidity"; // Es el nombre del tópico 2 al cual se publicarán los mensajes.
const char* topic_rain = "/rain"; // Es el nombre del tópico 2 al cual se publicarán los mensajes.
const char* topic_CO = "/CO"; // Es el nombre del tópico 2 al cual se publicarán los mensajes.
const char* topic_CO2 = "/CO2"; // Es el nombre del tópico 2 al cual se publicarán los mensajes.


// **********************************************

WiFiClient espClient; // Permite crear una conexión a un servidor con base en su dirección IP y puerto de comunicación.
PubSubClient client(espClient); // Permite configurar un cliente para envío y recepción de mensajes MQTT.

void setup() 
{
  delay(5000);
  Serial.begin(9600);
  setup_wifi(); // Función usada para configurar la conexión WiFi
  client.setServer(MQTT_SERVER, 1883); // Función usada para configurar la conexión al servidor MQTT.
  pinMode(DHT_PIN, INPUT_PULLUP);
  pinMode(RAIN_PIN, INPUT);
  pinMode(analogInputPin, INPUT);
  mq135.begin(R0);
}

void loop() 
{
  // Si el cliente MQTT pierde la conexión con el servidor, se intenta reconectar.
  if (!client.connected()) 
  {
    reconnect(); // Función usada para reconectar el cliente MQTT.
  }
  // La función loop() se llama con regularidad para permitir la llegada de mensajes y mantener la conexión con el servidor MQTT.
  // Si se ha perdido la conexión con el servidor, se intenta establecer nuevamente la misma.
  if(!client.loop())
  {
    client.connect(CLIENT_ID);
  }

  

  bool read_success = readSensor(); // Función usada para leer y procesar los datos de entrada del sensor.
  if (read_success) // Si se logó recibir un mensaje del sensor de forma satisfactoria.
  {
    float temperature = get_temperature(bytes_array); // Se obtiene el valor de temperatura.
    char payloadTemperature[10];
    sprintf(payloadTemperature, "%.2f", temperature);
    client.publish(topic_temperature, payloadTemperature);


    float humidity = get_humidity(bytes_array); // Se obtiene el valor de humedad
    char payloadHumidity[10];
    sprintf(payloadHumidity, "%.2f", humidity);
    client.publish(topic_humidity, payloadHumidity);

    float sensor_voltage = getSensorVoltage(); 
    float CO_PPM = mq135.getPPM(a_CO, b_CO, sensor_voltage);
    float CO2_PPM = mq135.getPPM(a_CO2, b_CO2, sensor_voltage);

    char payloadCO[15];
    sprintf(payloadCO, "%.2f", CO_PPM);
    client.publish(topic_CO, payloadCO);

    char payloadCO2[15];
    sprintf(payloadCO2, "%.2f", CO2_PPM);
    client.publish(topic_CO2, payloadCO2);

    bool value = 0;
    value = digitalRead(5);  //lectura digital de pin
    value = !value;
    char payloadRain[2];
    sprintf(payloadRain, "%d", value);
    client.publish(topic_rain, payloadRain);



    Serial.print("Payload: ");
    Serial.print(payloadTemperature);
    Serial.print(", published to: ");
   // Serial.print(topic_1);
    Serial.println(" MQTT topic.");

    Serial.print("Payload2: ");
    Serial.print(payloadHumidity);
    Serial.print(", published to: ");
    //Serial.print(topic_2);
    Serial.println(" MQTT topic.");

    // Se imprime en la interfaz Serial los valroes de temperatura y humedad obtenidos. 
  }
  delay(20000);
  
  
  delay(MQTT_TRANSMIT_RATE_MS); // Se detiene el código por un lapso de tiempo definido (MQTT_TRANSMIT_RATE_MS)
}
// Función usada para obtener el valor de temperatura obtenido del sensor DHT11 a partir de los bytes recibidos.
float get_temperature(uint8_t *bytes_array)
{
  // **********************************************
  // Obten el valor de temperatura con base en el tercer y cuarto byte
  // de la trama de bits obtenida.
  // **********************************************
  float temperature = bytes_array[2] + (bytes_array[3] * 0.1);
  // **********************************************
  
  return temperature;
}

// Función usada para obtener el valor de humedad obtenido del sensor DHT11 a partir de los bytes recibidos.
float get_humidity(uint8_t *bytes_array)
{
  // **********************************************
  // Obten el valor de humedad con base en el primer y segundo byte
  // de la trama de bits obtenida.
  // **********************************************
  float humidity = bytes_array[0] + (bytes_array[1] * 0.1);
  // **********************************************

  return humidity;
}

// Función usada para configurar la conexión WiFi
// 1. Se inicia la conexión con base en el SSID y Password declarados.
// 2. Mientras la conexión no se haya establecido, se imprime un mensaje de espera ".".
// 3. Una vez establecida la conexión, se imprime en el monitor Serial la dirección IP asiganada al módulo NodeMCU.
void setup_wifi() 
{
  delay(10);
  
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(SSID);
  
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.print("WiFi connected - ESP IP address: ");
  Serial.println(WiFi.localIP());
}

// Función usada para reconectar el cliente MQTT.
// 1. Mientras la conexión no se haya establecido, se vuelve a intentar la conexión hasta tener exito.
// 2. Si la conexión es exitosa, se imprime un mensaje de confirmación en el monitor Serial "connected".
// 3. Si la conexión falla, se vuelte a intentar la reconexión pasados 5 segundos.
void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(CLIENT_ID))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("MQTT connection failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}


bool readSensor()
{
  send_dht_start_signal();
  bool dht_confirm = wait_dht_ready_signal();
  if (!dht_confirm) return false;
  
  get_dht_response_pulse_times(pulse_time);

  bool dht_response_success = get_dht_bit_frame(pulse_time, bit_frame);
  if (!dht_response_success) return false;

  get_dht_bytes_array(bit_frame, bytes_array);

  bool correct_checksum = validate_response_checksum(bytes_array);
  if (!correct_checksum) return false;

  return true;
}

// Función usada para recibir un pulso por parte del sensor DHT11.
// Se puede especificar el nivel lógico del pulso esperado con el argumento de función pulse_level.
// Este argumento booleano indicará que se espera un pulso en bajo (pulse_level = LOW) o un pulso en alto (pulse_level = HIGH)
// La función regresará un número entero correspondiente al lapso de tiempo que duró dicho pulso detectado. 
// Es decir el tiempo que pasó desde que se detectó el pulso (p.ej: LOW), hasta que el valor de entrada cambió su estado. (p.ej: LOW -> HIGH)
// En caso de que el nivel lógico del pulso se haya mantenido por el tiempo máximo de espera (SENSOR_READ_TIMEOUT_US), 
// la función regresará el valor de la bandera de timeout (SENSOR_READ_TIMEOUT_FLAG)
int get_dht_pulse_time(bool pulse_level)
{
  long initial_sensor_read_time_us = micros();
  while(digitalRead(DHT_PIN) == pulse_level)
  {
    if (micros() - initial_sensor_read_time_us >= SENSOR_READ_TIMEOUT_US) 
    {
      return SENSOR_READ_TIMEOUT_FLAG;
    }
  }
  return micros() - initial_sensor_read_time_us;
}

// Función usada para generar la señal de inicio por parte del microcontrolador para adquirir lecturas del sensor.
// Se deberá modificar la configuración y estado del pin digital asignado al sensor, así como los tiempos de duración entre cada estado
// para establecer una correcta comunicación con el sensor DHT11.
void send_dht_start_signal()
{
  // **********************************************
  // Con base al datasheet, configura el modo de operación del pin 
  // asignado al sensor. 
  // Genera los pulsos de la trama de inicio correctamente.
  // Considera los tiempos de duración (delay) de los pulsos.
  // **********************************************
  pinMode(DHT_PIN, INPUT_PULLUP);
  delay(1);

  pinMode(DHT_PIN, OUTPUT);
  digitalWrite(DHT_PIN, LOW);
  delay(20);
  
  pinMode(DHT_PIN, INPUT_PULLUP);
  delayMicroseconds(40);
  // **********************************************
}

// Función usada para esperar la recepción de las señales de confirmación por parte del sensor DHT11 para comenzar la lectura. 
// Se utiliza la función get_dht_pulse_time() para esperar un pulso lógico en bajo y posteriormente un pulso lógico en alto.
// Secuencia generada por parte del sensor para confirmar una correcta comunicación.
// La función regresará un booleano indicando si la comunicación fue exitosa o no.
bool wait_dht_ready_signal()
{
  if (get_dht_pulse_time(LOW) == SENSOR_READ_TIMEOUT_FLAG) 
  {
    Serial.println("DHT timeout waiting for start signal low pulse.");
    return false;
  }

  if (get_dht_pulse_time(HIGH) == SENSOR_READ_TIMEOUT_FLAG) 
  {
    Serial.println("DHT timeout waiting for start signal high pulse.");
    return false;
  }
  return true;
}

// Función usada para obtener la duración de los pulsos recibidos desde el sensor DHT11.
// Los tiempos obtenidos se almacenarán en el arreglo pulse_time por pares.
void get_dht_response_pulse_times(int *pulse_time)
{
  // **********************************************
  // Completa la estructura del ciclo for para almacenar
  // la duración de los pulsos bajos y altos de la señal recibida.
  // Obten los tiempos que se generan de la función get_dht_pulse_time().
  // **********************************************
  for (int i = 0; i < 80; i+= 2)
  {
    pulse_time[i] = get_dht_pulse_time(LOW); //Almacena la duración del pulso en bajo
    pulse_time[i + 1] = get_dht_pulse_time(HIGH); //Almacena la duración del pulso en alto
  }
  // **********************************************
}

// Función usada para obtener el bit frame recibido desde el sensor a partir de la duración de los pulsos detectados.
// Por cada par de pulsos almacenados en el arreglo pulse_time, se identificará el valor digital enviado por el sensor
// con base en la duración de ambos pulsos.
bool get_dht_bit_frame(int *pulse_time, bool *bit_frame)
{
  // **********************************************
  // Completa la estructura del ciclo for para obtener los tiempos
  // en alto y bajo almacenados en el arreglo pulse_time.
  // **********************************************
  for (int i; i < 40; i++)
  {
    int low_level_time_us = pulse_time[2*i];
    int high_level_time_us = pulse_time[(2*i)+1];

    if ((low_level_time_us == SENSOR_READ_TIMEOUT_FLAG) || (high_level_time_us == SENSOR_READ_TIMEOUT_FLAG))
    {
      Serial.println("DHT timeout waiting for pulse.");
      return false;
    }

    // **********************************************
    // Compara ambos tiempos para definir el valor de cada bit contenido en el 
    // bit frame enviado por el sensor.
    // **********************************************
    if (low_level_time_us < high_level_time_us)
    {
      bit_frame[i] = 1;
    }
    else
    {
      bit_frame[i] = 0;
    }
  }
  // **********************************************
  
  return true;
}

// Función usada para obtener la representación en bytes del bit frame recibido.
// Dicha representación será almacenada en el arreglo bytes_array que incluirá un total de 5 bytes.
// Observe y analice la lógica usada para generar cada uno de los bytes a partir de sus bits correspondientes dentro del bit frame.
void get_dht_bytes_array(bool *bit_frame, uint8_t *bytes_array)
{
  for (int i = 0; i < 5; i++)
  {
    uint8_t byte = 0b00000000; 
    if (bit_frame[(8 * i) + 0] == 1) byte =  byte | 0b10000000;
    if (bit_frame[(8 * i) + 1] == 1) byte =  byte | 0b01000000;
    if (bit_frame[(8 * i) + 2] == 1) byte =  byte | 0b00100000;
    if (bit_frame[(8 * i) + 3] == 1) byte =  byte | 0b00010000;
    if (bit_frame[(8 * i) + 4] == 1) byte =  byte | 0b00001000;
    if (bit_frame[(8 * i) + 5] == 1) byte =  byte | 0b00000100;
    if (bit_frame[(8 * i) + 6] == 1) byte =  byte | 0b00000010;
    if (bit_frame[(8 * i) + 7] == 1) byte =  byte | 0b00000001;
    bytes_array[i] = byte;
  }
}

// Función usada para validar la correcta recepción de datos, comparando el valor del checksum recibido con su valor calculado.
// Se calcula el valor del checksum a partir de la suma de los bytes recibidos y se aplica una máscara booleana para conservar 
// exclusivamente el byte menos significativo de la suma obtenida (los últimos 8 bits). 
// Se compara el valor del checksum calculado con el último byte de la trama de bits recibida del sensor.
// La función regresará un valor booleano que representará si el checksum recibido es correcto o no.
bool validate_response_checksum(uint8_t *bytes_array)
{
  uint8_t checksum = (bytes_array[0] + bytes_array[1] + bytes_array[2] + bytes_array[3]) & 0b11111111;
  
  // **********************************************
  // Compara el valor calculado del checksum
  // con el último byte de la trama de bits recibidos del sensor.
  // **********************************************
  if (checksum == bytes_array[4])
  {
    return true;
  }
  // **********************************************
  
  else
  {
    Serial.println("DHT checksum failure!");
    return false;
  }
} 
float getSensorVoltage()
{
  // Variable temporal flotante donde se almacenará el promedio de las estimaciones de voltaje. Se inicializa en 0.
  float mean_sensor_voltage = 0.0;

  // Ciclo FOR que permitirá realizar el número de estimaciones deseadas. (número de estimaciones = VOLTAGE_MEASURES)
  for (int i = 0; i < VOLTAGE_MEASURES; i++)
  {
    // **********************************************
    // Lee el valor de la entrada analógica y calcula el voltaje detectado
    // Suma el valor del voltaje detectado a la variable temporal que se declaro anteriormente (mean_sensor_voltage).
    // **********************************************
    float sensor_analog_read = analogRead(analogInputPin);
    float sensor_voltage = sensor_analog_read*(ADC_VOLTAGE/ADC_MAX_VALUE);
    mean_sensor_voltage += sensor_voltage;
    // ********************************************** 

    // Se realiza un delay de VOLTAGE_MEASURES_DELAY_MS milisegundos, antes de realizar la siguiente estimación.
    delay(VOLTAGE_MEASURES_DELAY_MS);
  }

  // **********************************************
  // Obten el promedio de tus estimaciones en función de las variables definidas.
  // **********************************************
  mean_sensor_voltage /= 10;
  // **********************************************

  // Se retorna el valor flotante del promedio obtenido.
  return mean_sensor_voltage;
}
