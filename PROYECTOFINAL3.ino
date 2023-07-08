  #include <ESP8266WiFi.h>
  #include <PubSubClient.h>
  #include <Wire.h>

  // Reemplaza los valores con tus credenciales de Wi-Fi
  const char* ssid = "Redmi 9";
  const char* password = "12345678";
  bool conditionMet1 = false;  // Variable para realizar un seguimiento del estado de la primera condición
  bool conditionMet2 = false;
  bool conditionMet3 = false; 
  bool conditionMet4 = false; 
  // Reemplaza los valores con las credenciales de tu servidor MQTT en AWS
  const char* mqtt_server = "54.166.174.134";
  const int mqtt_port = 1883;
  const char* mqtt_user = "usuario_mqtt";
  const char* mqtt_password = "contraseña_mqtt";
  const char* topic2 = "test";
  const int BUZZER_PIN = D4;
  const int ledPin = D7;
  // Dirección I2C de la IMU
  #define MPU 0x68
  bool datoImpreso = false;

  // Ratios de conversión
  #define A_R 16384.0 // 32768/2
  #define G_R 131.0    // 32768/250

  // Conversión de radianes a grados (180/PI)
  #define RAD_TO_DEG 57.295779

  // Valores RAW
  int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;

  // Ángulos
  float Acc[2];
  float Gy[3];
  float Angle[3];

  String valores;

  long tiempo_prev;
  float dt;

  WiFiClient espClient;
  PubSubClient client(espClient);

  void setup() {
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, HIGH);
    Wire.begin(4, 5);  // D2(GPIO4)=SDA / D1(GPIO5)=SCL
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    Serial.begin(115200);

    // Inicializar la conexión Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.println("Conectado a Wi-Fi");

    

    // Configurar el servidor MQTT
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback); // Configurar la función de devolución de llamada para recibir datos MQTT
  }

  void loop() {
    // Verificar si la conexión MQTT se mantiene
    if (!client.connected()) {
      reconnect();
    }

    // Manejar los mensajes MQTT entrantes
    client.loop();

    // Leer los valores del Acelerómetro de la IMU
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // Pedir el registro 0x3B - corresponde al AcX
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);  // A partir del 0x3B, se piden 6 registros
    AcX = Wire.read() << 8 | Wire.read();  // Cada valor ocupa 2 registros
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();

    // A partir de los valores del acelerómetro, se calculan los ángulos Y, X
    // respectivamente, con la fórmula de la tangente.
    Acc[1] = atan(-1 * (AcX / A_R) / sqrt(pow((AcY /A_R), 2) + pow((AcZ / A_R), 2))) * RAD_TO_DEG;
    Acc[0] = atan((AcY / A_R) / sqrt(pow((AcX / A_R), 2) + pow((AcZ / A_R), 2))) * RAD_TO_DEG;

    // Leer los valores del Giroscopio
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);  // A partir del 0x43, se piden 6 registros
    GyX = Wire.read() << 8 | Wire.read();  // Cada valor ocupa 2 registros
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();

    // Cálculo del ángulo del Giroscopio
    Gy[0] = GyX / G_R;
    Gy[1] = GyY / G_R;
    Gy[2] = GyZ / G_R;

    dt = (millis() - tiempo_prev) / 1000.0;
    tiempo_prev = millis();

    // Aplicar el Filtro Complementario
    Angle[0] = 0.98 * (Angle[0] + Gy[0] * dt) + 0.02 * Acc[0];
    Angle[1] = 0.98 * (Angle[1] + Gy[1] * dt) + 0.02 * Acc[1];

    // Integración respecto del tiempo para calcular el YAW
    Angle[2] = Angle[2] + Gy[2] * dt;
  if (Angle[1] > 70.0 && !conditionMet1) {
    conditionMet1 = true;
    digitalWrite(BUZZER_PIN, LOW);
    sendMessage("0");
  } else if (Angle[1] <= 70.0 && conditionMet1) {
    conditionMet1 = false;
    digitalWrite(BUZZER_PIN, HIGH);
    sendMessage("1");
  }

   if (Angle[1] < -70.0 && !conditionMet2) {
    conditionMet2 = true;
    digitalWrite(BUZZER_PIN, LOW);
    sendMessage("0");
  } else if (Angle[1] >= -70.0 && conditionMet2) {
    conditionMet2 = false;
    digitalWrite(BUZZER_PIN, HIGH);
    sendMessage("1");
  }

  if (Angle[0] > 70.0 && !conditionMet3) {
    conditionMet3 = true;
    digitalWrite(BUZZER_PIN, LOW);
    sendMessage("0");
  } else if (Angle[0] <= 70.0 && conditionMet3) {
    conditionMet3 = false;
    digitalWrite(BUZZER_PIN, HIGH);
    sendMessage("1");
  }
  
  if (Angle[0] < -70.0 && !conditionMet4) {
    conditionMet4 = true;
    digitalWrite(BUZZER_PIN, LOW);
    sendMessage("0");
  } else if (Angle[0] >= -70.0 && conditionMet4) {
    conditionMet4 = false;
    digitalWrite(BUZZER_PIN, HIGH);
    sendMessage("1");
  }
 
  delay(10);

  }

  void reconnect() {
    // Volver a conectar al servidor MQTT
    while (!client.connected()) {
      if (client.connect("ESP8266Client", mqtt_user, mqtt_password)) {
        Serial.println("Conectado al servidor MQTT");
        client.subscribe("examen4b"); // Suscribirse al tópico deseado para recibir datos MQTT
      } else {
        Serial.print("Fallo en la conexión al servidor MQTT, estado: ");
        Serial.print(client.state());
        delay(2000);
      }
    }
  }

  void sendMessage(const char* message) {
    // Publicar el mensaje en el tópico deseado
    const char* topic = "examen4b";
    client.publish(topic, message);
  }

  void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Mensaje recibido en el tema: ");
    Serial.println(topic);
    Serial.print("Contenido del mensaje: ");
    for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
    }
    Serial.println();

    // Convertir el payload a una cadena de caracteres
    String message = "";
    for (int i = 0; i < length; i++) {
      message += (char)payload[i];
    }

    // Comparar el mensaje recibido para cambiar el estado del LED
    if (message == "Operando") {
      digitalWrite(ledPin, HIGH); // Encender el LED
    } else if (message == "Durmiendo") {
      digitalWrite(ledPin, LOW); // Apagar el LED
    }
  }