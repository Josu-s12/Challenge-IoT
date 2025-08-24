/* main.c - Sistema IoT Detección de Deslizamientos (ESP-IDF, versión completa y corregida)
   - Lógica de lluvia (YL-83) corregida: se basa SOLO en el valor analógico para evitar falsos positivos
   - Sin conflictos con APIs de I2C (no redefinimos i2c_master_write_byte)
   - ADC con driver oneshot (no legacy)
   - LCD I2C (PCF8574->HD44780), MPU6050 básico, LEDs, SW-420, rotación de pantallas
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "esp_system.h"

// ================= DEFINICIONES =================
static const char *TAG = "LANDSLIDE_MONITOR";

/* Pines y configuración I2C */
#define LED_VERDE         GPIO_NUM_2
#define LED_AMARILLO      GPIO_NUM_4
#define LED_ROJO          GPIO_NUM_5

#define HW080_ADC_CHANNEL ADC_CHANNEL_6  // GPIO34 - Humedad de suelo
#define YL83_ADC_CHANNEL  ADC_CHANNEL_7  // GPIO35 - Lluvia

#define HW080_DIGITAL_PIN GPIO_NUM_25    // (no usado en lógica final)
#define YL83_DIGITAL_PIN  GPIO_NUM_32    // (no usado en lógica final)
#define SW420_DIGITAL_PIN GPIO_NUM_33    // Vibración

#define I2C_MASTER_SCL_IO    22
#define I2C_MASTER_SDA_IO    21
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   100000

#define MPU6050_ADDR         0x68
#define LCD_ADDR_PRIMARY     0x27
#define LCD_ADDR_SECONDARY   0x3F

/* LCD (PCF8574 -> HD44780 4-bit) */
#define LCD_CMD_CLEAR        0x01
#define LCD_BACKLIGHT        0x08
#define LCD_ENABLE           0x04
#define LCD_REGISTER_SELECT  0x01

/* Umbrales (en unidades ADC crudas 0..4095 @atten=0dB) */
#define INCLINATION_WARNING  15.0f
#define INCLINATION_CRITICAL 25.0f

#define SOIL_MOISTURE_DRY    3000   // ajustar con logs
#define SOIL_MOISTURE_WET    1200   // ajustar con logs

#define RAIN_THRESHOLD_LOW   1800   // lluvia ligera si <= alto y > bajo
#define RAIN_THRESHOLD_HIGH  2800   // SIN lluvia si >= alto

#define VIBRATION_COUNT_ALERT       8
#define VIBRATION_INTENSITY_THRESHOLD 3

#define NO_OF_SAMPLES        32
#define UPDATE_INTERVAL_MS   1000
#define LCD_ROTATION_MS      3000

#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<LED_VERDE) | (1ULL<<LED_AMARILLO) | (1ULL<<LED_ROJO))
#define GPIO_INPUT_PIN_SEL   ((1ULL<<HW080_DIGITAL_PIN) | (1ULL<<YL83_DIGITAL_PIN) | (1ULL<<SW420_DIGITAL_PIN))

// ================= ESTRUCTURAS =================
typedef struct {
    int16_t accel_x, accel_y, accel_z;
    float pitch, roll;
} mpu6050_data_t;

typedef struct {
    uint32_t hw080_voltage, yl83_voltage; // valores ADC (crudos o mV si calibras)
    bool hw080_digital, yl83_digital, sw420_vibration;
    bool rain_detected, soil_dry;
    int  rain_level, soil_moisture_level; // 0=seco/sin lluvia, 1=medio, 2=mojado/lluvia fuerte
    uint32_t vibration_count;
    uint32_t vibration_intensity;
} sensors_data_t;

typedef struct {
    mpu6050_data_t mpu_data;
    sensors_data_t sensor_data;
    int alert_level; // 0=OK, 1=ALERTA, 2=CRITICO
} system_status_t;

// ================= GLOBALES =================
static uint8_t lcd_addr = 0;
static system_status_t system_status;
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_chan6_handle = NULL;
static adc_cali_handle_t adc1_cali_chan7_handle = NULL;
static bool do_calibration1_chan6 = false;
static bool do_calibration1_chan7 = false;

/* Prototipos */
static esp_err_t i2c_master_init(void);
static esp_err_t i2c_scan_devices(void);
static esp_err_t lcd_init(void);
static esp_err_t lcd_write_byte(uint8_t data, bool is_data);
static esp_err_t lcd_clear(void);
static esp_err_t lcd_set_cursor(uint8_t col, uint8_t row);
static esp_err_t lcd_print(const char* str);
static void     lcd_printf(uint8_t col, uint8_t row, const char* format, ...);

static esp_err_t mpu6050_init(void);
static esp_err_t mpu6050_read_data(mpu6050_data_t *data);

static esp_err_t init_additional_sensors(void);
static esp_err_t read_additional_sensors(sensors_data_t *sensor_data);
static void analyze_system_status(system_status_t *status);
static void init_leds(void);
static void update_leds(system_status_t *status);
static void monitoring_task(void *arg);

// ================= I2C =================
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t i2c_scan_devices(void) {
    for (int addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            if (addr == LCD_ADDR_PRIMARY || addr == LCD_ADDR_SECONDARY) {
                lcd_addr = addr;
                ESP_LOGI(TAG, "LCD encontrado en 0x%02X", addr);
            }
            if (addr == MPU6050_ADDR) {
                ESP_LOGI(TAG, "MPU6050 encontrado en 0x%02X", addr);
            }
        }
    }
    return ESP_OK;
}

// ================= LCD (PCF8574 -> HD44780 4-bit) =================
static esp_err_t lcd_write_byte(uint8_t data, bool is_data) {
    if (lcd_addr == 0) return ESP_FAIL;

    uint8_t high_nibble = data & 0xF0;
    uint8_t low_nibble  = (data << 4) & 0xF0;
    uint8_t rs = is_data ? LCD_REGISTER_SELECT : 0;

    // Enviar nibble alto (pulso ENABLE)
    uint8_t byte_high = high_nibble | LCD_BACKLIGHT | rs;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (lcd_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, byte_high | LCD_ENABLE, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Bajar ENABLE
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (lcd_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, byte_high, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Enviar nibble bajo (pulso ENABLE)
    uint8_t byte_low = low_nibble | LCD_BACKLIGHT | rs;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (lcd_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, byte_low | LCD_ENABLE, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Bajar ENABLE
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (lcd_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, byte_low, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(pdMS_TO_TICKS(1));

    return ESP_OK;
}

static esp_err_t lcd_init(void) {
    if (lcd_addr == 0) return ESP_FAIL;

    ESP_LOGI(TAG, "Inicializando LCD en 0x%02X", lcd_addr);
    vTaskDelay(pdMS_TO_TICKS(200)); // espera para estabilizar

    // Reset sequence (3x 0x30), luego 0x20 (4-bit)
    for (int i = 0; i < 3; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (lcd_addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, 0x30 | LCD_BACKLIGHT, true);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 200 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // 4-bit
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (lcd_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x20 | LCD_BACKLIGHT, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 200 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Config LCD
    lcd_write_byte(0x28, false);  // 4-bit, 2 líneas
    vTaskDelay(pdMS_TO_TICKS(10));
    lcd_write_byte(0x08, false);  // Display off
    vTaskDelay(pdMS_TO_TICKS(10));
    lcd_write_byte(0x01, false);  // Clear
    vTaskDelay(pdMS_TO_TICKS(10));
    lcd_write_byte(0x06, false);  // Entry mode
    vTaskDelay(pdMS_TO_TICKS(10));
    lcd_write_byte(0x0C, false);  // Display on (cursor off)
    vTaskDelay(pdMS_TO_TICKS(10));

    // Clear final
    lcd_write_byte(0x01, false);
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGI(TAG, "LCD inicializado correctamente");
    return ESP_OK;
}

static esp_err_t lcd_clear(void) {
    esp_err_t ret = lcd_write_byte(LCD_CMD_CLEAR, false);
    vTaskDelay(pdMS_TO_TICKS(2));
    return ret;
}

static esp_err_t lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t address = (row == 0) ? (0x80 + col) : (0xC0 + col);
    return lcd_write_byte(address, false);
}

static esp_err_t lcd_print(const char* str) {
    if (str == NULL) return ESP_FAIL;
    for (int i = 0; str[i] != '\0' && i < 16; i++) {
        esp_err_t ret = lcd_write_byte((uint8_t)str[i], true);
        if (ret != ESP_OK) return ret;
    }
    return ESP_OK;
}

static void lcd_printf(uint8_t col, uint8_t row, const char* format, ...) {
    char buffer[17];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    lcd_set_cursor(col, row);
    lcd_print(buffer);
}

// ================= MPU6050 (mínimo) =================
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

static esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t mpu6050_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);

    if (len > 1) i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t mpu6050_init(void) {
    uint8_t who_am_i = 0;
    esp_err_t ret = mpu6050_read_bytes(MPU6050_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK || who_am_i != 0x68) return ESP_FAIL;
    ret = mpu6050_write_byte(MPU6050_PWR_MGMT_1, 0x00); // wake up
    vTaskDelay(pdMS_TO_TICKS(100));
    return ret;
}

static esp_err_t mpu6050_read_data(mpu6050_data_t *data) {
    uint8_t raw_data[6] = {0};
    esp_err_t ret = mpu6050_read_bytes(MPU6050_ACCEL_XOUT_H, raw_data, 6);
    if (ret == ESP_OK) {
        data->accel_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
        data->accel_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
        data->accel_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);

        float ax = (float)data->accel_x;
        float ay = (float)data->accel_y;
        float az = (float)data->accel_z;

        float denom_p = sqrtf(ay * ay + az * az);
        float denom_r = sqrtf(ax * ax + az * az);

        data->pitch = (denom_p > 0.1f) ? atan2f(ax, denom_p) * 180.0f / M_PI : 0.0f;
        data->roll  = (denom_r > 0.1f) ? atan2f(ay, denom_r) * 180.0f / M_PI : 0.0f;
    }
    return ret;
}

// ================= Sensores ADC / digitales =================
static esp_err_t init_additional_sensors(void) {
    // ADC oneshot
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_0, // thresholds están en crudo 0..4095
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, HW080_ADC_CHANNEL, &cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, YL83_ADC_CHANNEL, &cfg));

    // Calibración opcional (si hay eFuse/TP)
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_0,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_config, &adc1_cali_chan6_handle) == ESP_OK) {
        do_calibration1_chan6 = true;
    }
    if (adc_cali_create_scheme_line_fitting(&cali_config, &adc1_cali_chan7_handle) == ESP_OK) {
        do_calibration1_chan7 = true;
    }

    // Entradas digitales (SW-420 y pines digitales de módulos; los digitales de suelo/lluvia no se usan)
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    return ESP_OK;
}

static esp_err_t read_additional_sensors(sensors_data_t *sd) {
    int hw080_adc_raw = 0;
    int yl83_adc_raw = 0;

    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        int t1 = 0, t2 = 0;
        adc_oneshot_read(adc1_handle, HW080_ADC_CHANNEL, &t1);
        adc_oneshot_read(adc1_handle, YL83_ADC_CHANNEL, &t2);
        hw080_adc_raw += t1;
        yl83_adc_raw += t2;
    }
    hw080_adc_raw /= NO_OF_SAMPLES;
    yl83_adc_raw /= NO_OF_SAMPLES;

    // Guardamos crudo (o mV si calibra)
    if (do_calibration1_chan6) {
        int mv = 0; adc_cali_raw_to_voltage(adc1_cali_chan6_handle, hw080_adc_raw, &mv);
        sd->hw080_voltage = (uint32_t)mv;  // en mV
    } else {
        sd->hw080_voltage = (uint32_t)hw080_adc_raw; // crudo
    }
    if (do_calibration1_chan7) {
        int mv = 0; adc_cali_raw_to_voltage(adc1_cali_chan7_handle, yl83_adc_raw, &mv);
        sd->yl83_voltage = (uint32_t)mv;   // en mV
    } else {
        sd->yl83_voltage = (uint32_t)yl83_adc_raw; // crudo
    }

    // No usamos los pines digitales de HW080/YL83 para la decisión (evita falsos positivos)
    sd->hw080_digital = 0;
    sd->yl83_digital  = 0;

    // Vibración (contador por flanco)
    static bool sw420_prev = false;
    bool sw420_now = gpio_get_level(SW420_DIGITAL_PIN);
    if (sw420_now && !sw420_prev) {
        sd->vibration_count++;
        sd->sw420_vibration = true;
    } else {
        sd->sw420_vibration = false;
    }
    sw420_prev = sw420_now;

    // Historial simple para "intensidad"
    static uint32_t vib_hist[10] = {0};
    static int hist_idx = 0;
    static int rot = 0;
    if (sd->sw420_vibration) vib_hist[hist_idx]++;
    if (++rot >= 1) { // se asume llamada cada ~1s
        hist_idx = (hist_idx + 1) % 10;
        vib_hist[hist_idx] = 0;
        rot = 0;
    }
    sd->vibration_intensity = 0;
    for (int i = 0; i < 10; i++) sd->vibration_intensity += vib_hist[i];

    // Humedad del suelo (umbral sobre dato crudo si no hay mV)
    // Si calibraste (mV), ajusta los números acorde a tu escala medida.
    uint32_t soil_val = sd->hw080_voltage;
    bool use_raw_soil = !do_calibration1_chan6; // true si estamos en crudo
    if (use_raw_soil) {
        if (soil_val > SOIL_MOISTURE_DRY) { sd->soil_dry = true;  sd->soil_moisture_level = 0; }
        else if (soil_val < SOIL_MOISTURE_WET) { sd->soil_dry = false; sd->soil_moisture_level = 2; }
        else { sd->soil_dry = false; sd->soil_moisture_level = 1; }
    } else {
        // Ejemplo en mV (ajusta a tus medidas reales)
        if (soil_val > 2500) { sd->soil_dry = true; sd->soil_moisture_level = 0; }
        else if (soil_val < 800) { sd->soil_dry = false; sd->soil_moisture_level = 2; }
        else { sd->soil_dry = false; sd->soil_moisture_level = 1; }
    }

    // Lluvia (YL-83) — CORREGIDO: usamos SOLO el valor analógico
    uint32_t rain_val = sd->yl83_voltage;
    bool use_raw_rain = !do_calibration1_chan7;
    if (use_raw_rain) {
        if (rain_val >= RAIN_THRESHOLD_HIGH) { sd->rain_detected = false; sd->rain_level = 0; }
        else if (rain_val >= RAIN_THRESHOLD_LOW) { sd->rain_detected = true; sd->rain_level = 1; }
        else { sd->rain_detected = true; sd->rain_level = 2; }
    } else {
        // Ejemplo en mV (ajusta a tus medidas reales)
        if (rain_val >= 2500) { sd->rain_detected = false; sd->rain_level = 0; }
        else if (rain_val >= 1500) { sd->rain_detected = true; sd->rain_level = 1; }
        else { sd->rain_detected = true; sd->rain_level = 2; }
    }

    // Logs para calibración rápida
    ESP_LOGI(TAG, "YL83=%"PRIu32"%s | Rain=%s | HW080=%"PRIu32"%s | Suelo=%s",
             sd->yl83_voltage, use_raw_rain ? "raw" : "mV",
             sd->rain_detected ? "SI" : "NO",
             sd->hw080_voltage, use_raw_soil ? "raw" : "mV",
             sd->soil_dry ? "SECO" : "HUMEDO");

    return ESP_OK;
}

// ================= Lógica de análisis =================
static void analyze_system_status(system_status_t *st) {
    float max_inclination = fmaxf(fabsf(st->mpu_data.pitch), fabsf(st->mpu_data.roll));
    st->alert_level = 0;
    int individual_risks = 0;
    int critical_risks = 0;

    bool inclination_critical = (max_inclination >= INCLINATION_CRITICAL);
    bool inclination_warning  = (max_inclination >= INCLINATION_WARNING);
    if (inclination_critical) critical_risks++;
    else if (inclination_warning) individual_risks++;

    bool rain_critical = (st->sensor_data.rain_level >= 2);
    bool rain_warning  = (st->sensor_data.rain_level >= 1);
    if (rain_critical) critical_risks++;
    else if (rain_warning) individual_risks++;

    bool soil_critical = (st->sensor_data.soil_moisture_level >= 2);
    bool soil_warning  = st->sensor_data.soil_dry;
    if (soil_critical) critical_risks++;
    else if (soil_warning) individual_risks++;

    bool vibration_critical = (st->sensor_data.vibration_intensity >= VIBRATION_INTENSITY_THRESHOLD);
    bool vibration_warning  = (st->sensor_data.vibration_count >= VIBRATION_COUNT_ALERT);
    if (vibration_critical) critical_risks++;
    else if (vibration_warning) individual_risks++;

    if (critical_risks >= 2 || (critical_risks >= 1 && individual_risks >= 2)) st->alert_level = 2;
    else if (critical_risks >= 1 || individual_risks >= 2) st->alert_level = 1;
    else st->alert_level = 0;
}

// ================= LEDs =================
static void init_leds(void) {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(LED_VERDE, 0);
    gpio_set_level(LED_AMARILLO, 0);
    gpio_set_level(LED_ROJO, 0);
}

static void update_leds(system_status_t *st) {
    switch (st->alert_level) {
        case 2: gpio_set_level(LED_VERDE, 0); gpio_set_level(LED_AMARILLO, 0); gpio_set_level(LED_ROJO, 1); break;
        case 1: gpio_set_level(LED_VERDE, 0); gpio_set_level(LED_AMARILLO, 1); gpio_set_level(LED_ROJO, 0); break;
        default: gpio_set_level(LED_VERDE, 1); gpio_set_level(LED_AMARILLO, 0); gpio_set_level(LED_ROJO, 0); break;
    }
}

// ================= Tarea principal =================
static void monitoring_task(void *arg) {
    static int display_mode = 0;
    static TickType_t last_lcd_update = 0;
    static uint32_t vibration_reset_counter = 0;

    ESP_LOGI(TAG, "Iniciando monitoreo");
    while (1) {
        esp_err_t mpu_ok = mpu6050_read_data(&system_status.mpu_data);
        esp_err_t sensors_ok = read_additional_sensors(&system_status.sensor_data);

        if (mpu_ok == ESP_OK && sensors_ok == ESP_OK) {
            analyze_system_status(&system_status);
            update_leds(&system_status);

            TickType_t now = xTaskGetTickCount();
            if ((now - last_lcd_update) >= pdMS_TO_TICKS(LCD_ROTATION_MS)) {
                if (lcd_addr != 0) {
                    switch (display_mode) {
                        case 0:
                            lcd_printf(0, 0, "P:%+5.1f R:%+5.1f",
                                       system_status.mpu_data.pitch,
                                       system_status.mpu_data.roll);
                            if (system_status.alert_level == 2)
                                lcd_printf(0, 1, "CRITICO! Niv:%d ", system_status.alert_level);
                            else if (system_status.alert_level == 1)
                                lcd_printf(0, 1, "ALERTA:  Niv:%d ", system_status.alert_level);
                            else
                                lcd_printf(0, 1, "Normal:  OK    ");
                            break;
                        case 1: {
                            bool use_raw = !do_calibration1_chan6;
                            lcd_printf(0, 0, "Suelo: %s",
                                       system_status.sensor_data.soil_dry ? "SECO" : "HUMEDO");
                            if (use_raw)
                                lcd_printf(0, 1, "HW080:%"PRIu32"raw ", system_status.sensor_data.hw080_voltage);
                            else
                                lcd_printf(0, 1, "HW080:%"PRIu32"mV  ", system_status.sensor_data.hw080_voltage);
                            break;
                        }
                        case 2: {
                            bool use_raw = !do_calibration1_chan7;
                            lcd_printf(0, 0, "Lluvia: %s", system_status.sensor_data.rain_detected ? "SI" : "NO");
                            if (use_raw)
                                lcd_printf(0, 1, "YL83:%"PRIu32"raw  ", system_status.sensor_data.yl83_voltage);
                            else
                                lcd_printf(0, 1, "YL83:%"PRIu32"mV   ", system_status.sensor_data.yl83_voltage);
                            break;
                        }
                        case 3:
                            lcd_printf(0, 0, "Vib: %s", system_status.sensor_data.sw420_vibration ? "SI" : "NO");
                            lcd_printf(0, 1, "Int:%"PRIu32" Cnt:%"PRIu32" ",
                                       system_status.sensor_data.vibration_intensity,
                                       system_status.sensor_data.vibration_count);
                            break;
                    }
                    display_mode = (display_mode + 1) % 4;
                }
                last_lcd_update = now;
            }

            static int log_counter = 0;
            if (++log_counter >= 10) {
                ESP_LOGI(TAG, "Alert=%d | Pitch=%.1f | Roll=%.1f | Suelo=%s(%"PRIu32") | Lluvia=%s(%"PRIu32") | VibInt=%"PRIu32" | VibCnt=%"PRIu32,
                         system_status.alert_level,
                         system_status.mpu_data.pitch,
                         system_status.mpu_data.roll,
                         system_status.sensor_data.soil_dry ? "SECO" : "HUMEDO",
                         system_status.sensor_data.hw080_voltage,
                         system_status.sensor_data.rain_detected ? "SI" : "NO",
                         system_status.sensor_data.yl83_voltage,
                         system_status.sensor_data.vibration_intensity,
                         system_status.sensor_data.vibration_count);
                log_counter = 0;
            }

            if (++vibration_reset_counter >= 60) {
                system_status.sensor_data.vibration_count = 0;
                vibration_reset_counter = 0;
            }
        } else {
            ESP_LOGW(TAG, "Error sensores: MPU=%d, ADC=%d", mpu_ok, sensors_ok);
            if (lcd_addr != 0) {
                lcd_printf(0, 0, "Error Sensores  ");
                lcd_printf(0, 1, "Verificar conex.");
            }
            // LED rojo intermitente
            gpio_set_level(LED_ROJO, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LED_ROJO, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(UPDATE_INTERVAL_MS));
    }
}

// ================= main =================
void app_main(void)
{
    ESP_LOGI(TAG, "Sistema IoT Deteccion de Deslizamientos - Inicio");

    memset(&system_status, 0, sizeof(system_status_t));
    init_leds();

    // Prueba rápida LEDs
    gpio_set_level(LED_VERDE, 1);
    gpio_set_level(LED_AMARILLO, 1);
    gpio_set_level(LED_ROJO, 1);
    vTaskDelay(pdMS_TO_TICKS(300));
    gpio_set_level(LED_VERDE, 0);
    gpio_set_level(LED_AMARILLO, 0);
    gpio_set_level(LED_ROJO, 0);

    // I2C + Scan + periféricos
    if (i2c_master_init() == ESP_OK) {
        ESP_LOGI(TAG, "I2C inicializado");
        i2c_scan_devices();

        if (mpu6050_init() == ESP_OK) {
            ESP_LOGI(TAG, "MPU6050 inicializado");
        } else {
            ESP_LOGW(TAG, "MPU6050 no inicializado");
        }

        if (lcd_addr != 0) {
            if (lcd_init() == ESP_OK) {
                lcd_printf(0, 0, "Sistema IoT");
                lcd_printf(0, 1, "Iniciando...");
                vTaskDelay(pdMS_TO_TICKS(1000));
                lcd_clear();
            } else {
                ESP_LOGW(TAG, "Error inicializando LCD");
            }
        } else {
            ESP_LOGW(TAG, "LCD no detectado");
        }
    } else {
        ESP_LOGE(TAG, "Error inicializando I2C");
    }

    if (init_additional_sensors() == ESP_OK) {
        ESP_LOGI(TAG, "Sensores adicionales OK");
    } else {
        ESP_LOGE(TAG, "Error sensores adicionales");
    }

    if (lcd_addr != 0) {
        lcd_clear();
        lcd_printf(0, 0, "Sistema Listo");
        lcd_printf(0, 1, "Monitoreando...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        lcd_clear();
    }

    xTaskCreate(monitoring_task, "monitoring", 8192, NULL, 5, NULL);
    ESP_LOGI(TAG, "Sistema iniciado");
}