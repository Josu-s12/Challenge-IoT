# 🌍 Sistema IoT para Detección de Deslizamientos

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)  
![Status](https://img.shields.io/badge/status-functional-green)

**Integrantes:**  
- Daniel Pareja Franco  
- Juan José Forero Peña  
- Josué Sarmiento  

**Asignatura:** Internet de las Cosas  
**Universidad de La Sabana**  
**Fecha:** 25 de agosto de 2025  

---

## **📌 Descripción**

Este proyecto consiste en el desarrollo de un **prototipo IoT** para la **detección temprana de deslizamientos de tierra** en zonas de riesgo, utilizando sensores de **inclinación**, **lluvia**, **vibración** y **humedad del suelo**.  

El sistema realiza una **fusión lógica de señales** para generar **alertas locales inmediatas (in situ)**, sin necesidad de redes de comunicación, ideal para zonas con poca o nula conectividad.

---

## **🎯 Características Principales**
- Medición en tiempo real de:
  - Inclinación del talud.
  - Intensidad de lluvia.
  - Humedad del suelo.
  - Vibración del terreno.  
- **Lógica de fusión:** mínimo 3 señales críticas para generar una alerta válida.  
- **Sistema de alerta local:**  
  - **Visual:** LEDs de estado (verde, amarillo, rojo).  
  - **Sonoro:** buzzer activo con patrones escalables.  
  - **Pantalla:** LCD u OLED para visualización de datos en tiempo real.  
- **Escalable:** posibilidad de incorporar comunicación inalámbrica (WiFi, LoRa, NB-IoT) en futuras versiones.  

---

## **📂 Estructura del Repositorio**

📂 Proyecto-IoT-Deslizamientos
┣ 📂 code
┃ ┣ simulacion_wokwi.ino # Versión para simulador Wokwi
┃ ┗ main.c # Versión para ESP-IDF
┣ 📂 docs
┃ ┣ diagramas/ # Diagramas de arquitectura, UML y esquemáticos
┃ ┣ fotos/ # Evidencia de hardware y pruebas
┃ ┗ pruebas/ # Resultados experimentales
┣ 📂 extras
┃ ┗ datasheets/ # Hojas técnicas de sensores
┣ README.md
┗ LICENSE


---

## **⚙️ Requisitos**

### **Hardware**
- **ESP32 DevKit V1**
- **MPU6050** (Sensor de inclinación/vibración)
- **YL-83** (Sensor de lluvia)
- **SW-420** (Sensor de vibración)
- **Sensor capacitivo de humedad de suelo v1.2**
- **LCD 16x2 I2C** u **OLED 128x64**
- **Buzzer activo 5V**
- **LEDs RGB**
- **Batería LiPo 7.4V + módulo de carga**

### **Software**
- [Arduino IDE](https://www.arduino.cc/en/software)
- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/) (para versión avanzada)
- [Wokwi](https://wokwi.com) para simulación
- [Fritzing](https://fritzing.org/) para esquemáticos
- [Draw.io](https://app.diagrams.net/) o [Lucidchart](https://www.lucidchart.com/) para diagramas UML

---

## **🚀 Instalación y Configuración**

### **🔹 Opción 1: Simulación en Wokwi**
1. Abre [Wokwi](https://wokwi.com/) y crea un proyecto para ESP32.
2. Copia el contenido del archivo:
/code/simulacion_wokwi.ino
3. Configura el archivo `diagram.json` con:
- MPU6050
- Potenciómetros para lluvia y humedad
- Botón para vibración
- LEDs y buzzer
4. Haz clic en **"Run"** y monitorea los datos en tiempo real en el puerto serial.
5. Ajusta los potenciómetros y el botón para simular diferentes escenarios.

---

### **🔹 Opción 2: Hardware Real (Arduino IDE)**
1. **Clonar el repositorio**
```bash
git clone https://github.com/usuario/Proyecto-IoT-Deslizamientos.git
cd Proyecto-IoT-Deslizamientos

### Pasos para ejecutar la simulación

#### ** Abrir el archivo de simulación**
- Abre `simulacion_wokwi.ino` en **Arduino IDE**.

#### **Instalar librerías**
- **Adafruit MPU6050**  
- **Adafruit Unified Sensor**  
- **LiquidCrystal_I2C**

#### **Configurar placa y puerto**
- Ve a **Herramientas > Placa > ESP32 Dev Module**.  
- Selecciona el **puerto COM** correcto.

#### **Cargar el programa**
- Conecta el **ESP32** y **sube el sketch**.

#### **Monitorear datos**
- Abre el **Monitor Serial** a `115200 baud` para ver datos de sensores y nivel de alerta.

#### **Prueba física**
- Inclina el prototipo.  
- Simula vibraciones.  
- Moja el sensor de humedad para comprobar alertas.

---

### **🔹 Opción 3: Versión Avanzada con ESP-IDF**

#### **Instalar ESP-IDF**
- Sigue la [guía oficial](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html).

#### **Clonar el proyecto**
```bash
git clone https://github.com/tuusuario/tu-repositorio.git
cd tu-repositorio

