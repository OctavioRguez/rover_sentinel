#include <WiFi.h>
#include <WiFiUdp.h>

// Pines del joystick
const int joyXPin = 34; // Pin analógico para el eje X
const int joyYPin = 35; // Pin analógico para el eje Y
const int joyButtonPin = 32; // Pin digital para el botón

// Valores del joystick
int joyXValue = 0;
int joyYValue = 0;
int buttonValue = 0;

// Configura tu red Wi-Fi
const char* ssid = "Freevirus";
const char* password = "freemalware";

// Dirección IP y puerto de la Jetson Nano
const char* jetson_ip = "192.168.211.104";
const int jetson_port = 5005;

WiFiUDP udp;

void setup() {
  // Inicia la comunicación serial para depuración
  Serial.begin(115200);
  pinMode(joyButtonPin, INPUT_PULLUP);

  // Conéctate a la red Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Inicializa el cliente UDP
  udp.begin(jetson_port);
}

void loop() {
  // Lee los valores del joystick
  joyXValue = analogRead(joyXPin);
  joyYValue = analogRead(joyYPin);
  buttonValue = !digitalRead(joyButtonPin);

  // Imprime los valores del joystick en el monitor serial
  Serial.print("Joystick X: ");
  Serial.print(joyXValue);
  Serial.print(", Y: ");
  Serial.print(joyYValue);
  Serial.print(", Button: ");
  Serial.println(buttonValue);

  // Construye el mensaje con los datos del joystick
  String message = String(joyXValue) + "," + String(joyYValue) + "," + String(buttonValue);

  // Envia los datos del joystick a la Jetson Nano
  udp.beginPacket(jetson_ip, jetson_port);
  udp.write((const uint8_t*)message.c_str(), message.length());
  udp.endPacket();

  // Espera un segundo antes de la próxima lectura
  delay(100);
}