/**
 * SIMULACIÓN WOKWI - Sistema IoT Detección de Deslizamientos
 * 
 * Para usar en Wokwi:
 * 1. Crear nuevo proyecto ESP32
 * 2. Copiar este código
 * 3. Configurar el diagram.json con los componentes
 * 
 * Componentes simulados:
 * - MPU6050 (real en Wokwi)
 * - Potenciómetros para simular sensores analógicos
 * - Botón para simular vibración
 * - LEDs y LCD I2C
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>

// ============= CONFIGURACIÓN DE PINES =============
// En Wokwi usaremos potenciómetros para simular sensores analógicos
#define RAIN_SENSOR_PIN 34      // Potenciómetro 1 - Simula lluvia
#define SOIL_SENSOR_PIN 35      // Potenciómetro 2 - Simula humedad
#define VIBRATION_PIN 15        // Botón - Simula vibración
#define BUZZER_PIN 12           // Buzzer
#define LED_GREEN 2             // LED Verde
#define LED_YELLOW 4            // LED Amarillo  
#define LED_RED 5               // LED Rojo

// ============= CONFIGURACIÓN DE UMBRALES =============
#define TILT_THRESHOLD 15.0     // Grados de inclinación
#define SOIL_THRESHOLD 70       // % humedad crítica
#define RAIN_THRESHOLD 60       // % intensidad lluvia
#define VIBRATION_THRESHOLD 3   // Pulsos en ventana

// ============= OBJETOS GLOBALES =============
Adafruit_MPU6050 mpu;
LiquidCrystal_I2C lcd(0x27, 20, 4);

// ============= VARIABLES GLOBALES =============
struct SensorData {
  float tiltX, tiltY;
  int rainIntensity;
  int soilMoisture;
  int vibrationCount;
  int riskLevel;
};

SensorData data;
unsigned long lastUpdate = 0;
unsigned long vibrationWindow = 0;
int vibrationPulses = 0;
bool lastVibState = false;

// Variables para simulación
float simTiltOffset = 0;
bool demoMode = false;
int demoStep = 0;
unsigned long demoTimer = 0;

// ============= SETUP =============
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=================================");
  Serial.println("  SISTEMA IoT - DESLIZAMIENTOS");
  Serial.println("    Simulación en WOKWI");
  Serial.println("=================================");
  Serial.println();
  
  // Configurar pines
  pinMode(VIBRATION_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  
  // Test inicial de LEDs
  Serial.println("Test de LEDs...");
  testLEDs();
  
  // Inicializar I2C
  Wire.begin();
  
  // Inicializar LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sistema IoT v1.0");
  lcd.setCursor(0, 1);
  lcd.print("Inicializando...");
  
  // Inicializar MPU6050
  if (!mpu.begin()) {
    Serial.println("ERROR: MPU6050 no encontrado!");
    Serial.println("Continuando en modo simulación...");
    lcd.setCursor(0, 2);
    lcd.print("Modo Simulacion");
  } else {
    Serial.println("MPU6050 detectado correctamente");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  
  delay(2000);
  lcd.clear();
  
  // Instrucciones para el usuario
  printInstructions();
}

// ============= FUNCIONES DE TEST =============
void testLEDs() {
  digitalWrite(LED_GREEN, HIGH);
  delay(300);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, HIGH);
  delay(300);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, HIGH);
  delay(300);
  digitalWrite(LED_RED, LOW);
}

void printInstructions() {
  Serial.println("\n=== INSTRUCCIONES DE USO ===");
  Serial.println("CONTROLES EN WOKWI:");
  Serial.println("- Potenciometro 1: Simula intensidad de lluvia");
  Serial.println("- Potenciometro 2: Simula humedad del suelo");
  Serial.println("- Boton: Simula pulsos de vibracion");
  Serial.println("- MPU6050: Inclinar para simular movimiento");
  Serial.println();
  Serial.println("COMANDOS SERIAL:");
  Serial.println("- 'd': Activar modo demo automatico");
  Serial.println("- 'n': Volver a modo normal");
  Serial.println("- 't': Test de componentes");
  Serial.println("- 'h': Mostrar esta ayuda");
  Serial.println("===========================\n");
}

// ============= LECTURA DE SENSORES =============
void readSensors() {
  // Leer MPU6050 (o simular si no está disponible)
  if (mpu.begin()) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    data.tiltX = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
    data.tiltY = atan2(a.acceleration.x, a.acceleration.z) * 180 / PI;
  } else {
    // Simulación de inclinación para testing
    data.tiltX = simTiltOffset + random(-2, 2);
    data.tiltY = simTiltOffset/2 + random(-2, 2);
  }
  
  // Leer potenciómetro 1 como sensor de lluvia
  int rainRaw = analogRead(RAIN_SENSOR_PIN);
  data.rainIntensity = map(rainRaw, 0, 4095, 0, 100);
  
  // Leer potenciómetro 2 como sensor de humedad
  int soilRaw = analogRead(SOIL_SENSOR_PIN);
  data.soilMoisture = map(soilRaw, 0, 4095, 0, 100);
  
  // Detectar vibración (botón como simulación)
  bool vibState = !digitalRead(VIBRATION_PIN); // Pull-up, así que invertimos
  if (vibState && !lastVibState) {
    vibrationPulses++;
  }
  lastVibState = vibState;
  
  // Contar pulsos en ventana de 1 segundo
  if (millis() - vibrationWindow > 1000) {
    data.vibrationCount = vibrationPulses;
    vibrationPulses = 0;
    vibrationWindow = millis();
  }
}

// ============= LÓGICA DE FUSIÓN =============
void calculateRisk() {
  int alertCount = 0;
  float riskScore = 0;
  
  // Evaluar inclinación
  float tiltMag = sqrt(data.tiltX * data.tiltX + data.tiltY * data.tiltY);
  if (tiltMag > TILT_THRESHOLD) {
    alertCount++;
    riskScore += 0.35;
  }
  
  // Evaluar lluvia
  if (data.rainIntensity > RAIN_THRESHOLD) {
    alertCount++;
    riskScore += 0.20;
  }
  
  // Evaluar humedad del suelo
  if (data.soilMoisture > SOIL_THRESHOLD) {
    alertCount++;
    riskScore += 0.25;
  }
  
  // Evaluar vibración
  if (data.vibrationCount > VIBRATION_THRESHOLD) {
    alertCount++;
    riskScore += 0.20;
  }
  
  // Determinar nivel de riesgo
  if (alertCount == 0) {
    data.riskLevel = 0; // Normal
  } else if (alertCount == 1) {
    data.riskLevel = 1; // Precaución
  } else if (alertCount == 2) {
    data.riskLevel = 2; // Alerta Media
  } else if (alertCount >= 3) {
    data.riskLevel = 3; // Alerta Alta
  }
  
  // Condición crítica
  if (alertCount == 4 || riskScore > 0.85) {
    data.riskLevel = 4; // Crítico
  }
}

// ============= SISTEMA DE ALERTAS =============
void updateAlerts() {
  // Actualizar LEDs
  digitalWrite(LED_GREEN, data.riskLevel == 0);
  digitalWrite(LED_YELLOW, data.riskLevel == 1 || data.riskLevel == 2);
  digitalWrite(LED_RED, data.riskLevel >= 3);
  
  // Buzzer según nivel de riesgo
  if (data.riskLevel >= 3) {
    // Patrón de alarma
    if (data.riskLevel == 3) {
      // Beep intermitente
      tone(BUZZER_PIN, 1000, 200);
      delay(300);
    } else if (data.riskLevel == 4) {
      // Alarma continua más aguda
      tone(BUZZER_PIN, 2000, 100);
      delay(100);
    }
  } else {
    noTone(BUZZER_PIN);
  }
}

// ============= ACTUALIZAR DISPLAY =============
void updateDisplay() {
  // Línea 0: Estado del sistema
  lcd.setCursor(0, 0);
  switch(data.riskLevel) {
    case 0:
      lcd.print("Estado: NORMAL      ");
      break;
    case 1:
      lcd.print("Estado: PRECAUCION  ");
      break;
    case 2:
      lcd.print("Estado: ALERTA MEDIA");
      break;
    case 3:
      lcd.print("Estado: ALERTA ALTA ");
      break;
    case 4:
      lcd.print("Estado: CRITICO!!!  ");
      break;
  }
  
  // Línea 1: Inclinación
  lcd.setCursor(0, 1);
  lcd.print("Incl: ");
  float tiltMag = sqrt(data.tiltX * data.tiltX + data.tiltY * data.tiltY);
  if (tiltMag < 10) lcd.print(" ");
  lcd.print(tiltMag, 1);
  lcd.print((char)223); // Símbolo de grado
  if (tiltMag > TILT_THRESHOLD) {
    lcd.print(" !");
  } else {
    lcd.print("  ");
  }
  
  lcd.print(" Vib:");
  lcd.print(data.vibrationCount);
  lcd.print("  ");
  
  // Línea 2: Lluvia y Humedad
  lcd.setCursor(0, 2);
  lcd.print("Lluv:");
  if (data.rainIntensity < 10) lcd.print(" ");
  lcd.print(data.rainIntensity);
  lcd.print("% ");
  
  lcd.print("Hum:");
  if (data.soilMoisture < 10) lcd.print(" ");
  lcd.print(data.soilMoisture);
  lcd.print("%  ");
  
  // Línea 3: Alertas activas o instrucciones
  lcd.setCursor(0, 3);
  if (data.riskLevel >= 2) {
    lcd.print("!ALERTA ACTIVA!     ");
  } else {
    lcd.print("Ajusta potenciometros");
  }
}

// ============= ENVIAR DATOS AL MONITOR SERIAL =============
void printSerialData() {
  Serial.print("Inclinacion: ");
  Serial.print(sqrt(data.tiltX * data.tiltX + data.tiltY * data.tiltY), 1);
  Serial.print("° | Lluvia: ");
  Serial.print(data.rainIntensity);
  Serial.print("% | Humedad: ");
  Serial.print(data.soilMoisture);
  Serial.print("% | Vibracion: ");
  Serial.print(data.vibrationCount);
  Serial.print(" | Riesgo: ");
  
  switch(data.riskLevel) {
    case 0: Serial.println("NORMAL"); break;
    case 1: Serial.println("PRECAUCION"); break;
    case 2: Serial.println("ALERTA MEDIA"); break;
    case 3: Serial.println("ALERTA ALTA"); break;
    case 4: Serial.println("CRITICO"); break;
  }
}

// ============= MODO DEMO =============
void runDemoMode() {
  if (millis() - demoTimer > 3000) {
    demoTimer = millis();
    demoStep = (demoStep + 1) % 5;
    
    Serial.println("\n=== MODO DEMO - Escenario " + String(demoStep + 1) + " ===");
    
    switch(demoStep) {
      case 0: // Normal
        simTiltOffset = 5;
        Serial.println("Simulando: Condiciones normales");
        break;
      case 1: // Lluvia moderada
        simTiltOffset = 8;
        Serial.println("Simulando: Lluvia moderada");
        break;
      case 2: // Inclinación peligrosa
        simTiltOffset = 18;
        Serial.println("Simulando: Inclinacion peligrosa");
        break;
      case 3: // Múltiples alertas
        simTiltOffset = 16;
        vibrationPulses = 5;
        Serial.println("Simulando: Multiples alertas");
        break;
      case 4: // Crítico
        simTiltOffset = 25;
        vibrationPulses = 8;
        Serial.println("Simulando: Situacion CRITICA");
        break;
    }
  }
}

// ============= LOOP PRINCIPAL =============
void loop() {
  // Actualizar cada 100ms
  if (millis() - lastUpdate > 100) {
    lastUpdate = millis();
    
    // Leer sensores
    readSensors();
    
    // Si está en modo demo, modificar valores
    if (demoMode) {
      runDemoMode();
    }
    
    // Calcular nivel de riesgo
    calculateRisk();
    
    // Actualizar alertas
    updateAlerts();
    
    // Actualizar display
    updateDisplay();
    
    // Enviar datos por serial cada 1 segundo
    static unsigned long lastSerial = 0;
    if (millis() - lastSerial > 1000) {
      lastSerial = millis();
      printSerialData();
    }
  }
  
  // Procesar comandos del serial
  if (Serial.available()) {
    char cmd = Serial.read();
    switch(cmd) {
      case 'd':
      case 'D':
        demoMode = true;
        Serial.println("\n>>> MODO DEMO ACTIVADO <<<");
        break;
      case 'n':
      case 'N':
        demoMode = false;
        simTiltOffset = 0;
        Serial.println("\n>>> MODO NORMAL <<<");
        break;
      case 't':
      case 'T':
        testComponents();
        break;
      case 'h':
      case 'H':
        printInstructions();
        break;
    }
  }
}

// ============= TEST DE COMPONENTES =============
void testComponents() {
  Serial.println("\n=== TEST DE COMPONENTES ===");
  
  // Test LEDs
  Serial.println("1. Test LEDs (Verde-Amarillo-Rojo)");
  testLEDs();
  
  // Test Buzzer
  Serial.println("2. Test Buzzer");
  tone(BUZZER_PIN, 1000, 500);
  delay(600);
  
  // Test sensores
  Serial.println("3. Valores actuales de sensores:");
  Serial.print("   - Potenciometro Lluvia: ");
  Serial.print(analogRead(RAIN_SENSOR_PIN));
  Serial.println("/4095");
  Serial.print("   - Potenciometro Humedad: ");
  Serial.print(analogRead(SOIL_SENSOR_PIN));
  Serial.println("/4095");
  Serial.print("   - Boton Vibracion: ");
  Serial.println(digitalRead(VIBRATION_PIN) ? "No presionado" : "Presionado");
  
  Serial.println("=== FIN DEL TEST ===\n");
}