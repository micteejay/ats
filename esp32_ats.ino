#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <math.h>
#include <string.h>

// WiFi Configuration
#define SSID_MAX_LEN 32
#define PASS_MAX_LEN 64
char ssid[SSID_MAX_LEN] = "";
char password[PASS_MAX_LEN] = "";
const char* hostname = "ATS-ESP32";

// Web Server
WebServer server(80);

// LCD Configuration
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ATS Control Pins
#define NEPA_RELAY_PIN      26    // Relay for NEPA connection
#define GEN_RELAY_PIN       27    // Relay for Generator connection
#define BUZZER_PIN         25    // Alarm buzzer
#define STATUS_LED_PIN     2     // Status indicator LED

// Analog pin mapping for ESP32
#define VOLT_PIN_NEPA      36    // VP pin for NEPA voltage
#define VOLT_PIN_GEN       39    // VN pin for Generator voltage
#define CURR_PIN_LOAD      34    // Current sensor pin

// Menu Control Buttons
#define BTN_UP             32    // Menu UP button
#define BTN_DOWN           33    // Menu DOWN button
#define BTN_ENTER          35    // Menu ENTER button

// Additional control pins
#define MANUAL_MODE_PIN    4     // Manual/Auto mode switch
#define RESET_ALARM_PIN    5     // Reset alarm button

// AC Detection Constants
#define AC_DETECTION_TIMEOUT  100    // 100ms timeout for AC detection
#define SAMPLES_PER_READING   100    // Reduced for ESP32 performance
#define SAMPLE_DELAY_MS       1      // 1ms delay per sample
#define MIN_AC_THRESHOLD      5      // Higher threshold to avoid floating noise
#define AC_MIN_SPAN           10     // Minimum ADC span across samples

// Voltage thresholds (configurable via menu)
uint16_t MIN_VOLTAGE = 180;           // Minimum acceptable voltage
uint16_t MAX_VOLTAGE = 290;           // Maximum acceptable voltage
uint16_t VOLTAGE_HYSTERESIS = 10;     // Voltage switching hysteresis

// Timing parameters
uint16_t SWITCH_DELAY = 100;          // 100ms delay before switching
uint16_t SOURCE_RETURN_DELAY = 2000;  // 2 second delay before switching back

// Calibration variables
int8_t nepa_voltage_offset = 0;       // NEPA voltage calibration offset
int8_t gen_voltage_offset = 0;        // Generator voltage calibration offset
int8_t current_offset = 0;            // Current sensor calibration offset
float voltage_multiplier = 0.482;     // Voltage conversion factor
float current_multiplier = 0.039;     // Current conversion factor

// Power source enumeration
enum PowerSource {
  SOURCE_NONE,
  SOURCE_NEPA,
  SOURCE_GENERATOR
};

// System states
enum SystemState {
  STATE_INIT,
  STATE_NEPA_ACTIVE,
  STATE_GEN_ACTIVE,
  STATE_SWITCHING,
  STATE_FAULT,
  STATE_MANUAL
};

// Menu system variables
enum MenuState {
  MENU_MAIN,
  MENU_MONITOR,
  MENU_SETTINGS,
  MENU_CALIBRATION,
  MENU_WIFI,
  MENU_INFO
};

MenuState current_menu = MENU_MAIN;
uint8_t menu_index = 0;
uint8_t submenu_index = 0;
bool in_submenu = false;
bool menu_active = false;
uint32_t last_button_press = 0;
bool button_up_pressed = false;
bool button_down_pressed = false;
bool button_enter_pressed = false;

// System variables
SystemState current_state = STATE_INIT;
PowerSource active_source = SOURCE_NONE;
bool auto_mode = true;
bool settings_modified = false;
bool wifi_connected = false;

// AC reading variables
uint16_t sensorV_nepa = 0, sensorV_gen = 0;
uint16_t sensorI_load = 0;
float amp_load = 0.0;
uint16_t volt_nepa = 0, volt_gen = 0, volt_load = 0;
uint16_t watt_nepa = 0, watt_gen = 0, watt_load = 0;
float kwh_nepa = 0, kwh_gen = 0, kwh_total = 0;

// System status
struct SystemStatus {
  bool gen_running;
  bool alarm_active;
  bool system_fault;
  uint32_t state_change_time;
  uint32_t last_switch_time;
  uint8_t fault_code;
  uint16_t runtime_hours;
  bool nepa_valid;
  bool gen_valid;
  bool load_detected;
  uint16_t switch_count;
  bool nepa_relay_on;
  bool gen_relay_on;
} status;

// Timing variables
uint32_t last_reading = 0;
uint32_t last_display_update = 0;
uint8_t display_screen = 0;
uint32_t last_energy_update = 0;

// EEPROM addresses
#define EEPROM_SIZE 512
#define EEPROM_RUNTIME_ADDR      0
#define EEPROM_FAULT_LOG_ADDR    4
#define EEPROM_KWH_NEPA_ADDR     8
#define EEPROM_KWH_GEN_ADDR     12
#define EEPROM_CONFIG_ADDR      16
#define EEPROM_CAL_NEPA_V_ADDR  20
#define EEPROM_CAL_GEN_V_ADDR   24
#define EEPROM_CAL_CURRENT_ADDR 28
#define EEPROM_SETTINGS_ADDR    32
#define EEPROM_WIFI_SSID_ADDR   50
#define EEPROM_WIFI_PASS_ADDR   (EEPROM_WIFI_SSID_ADDR + SSID_MAX_LEN)

// Main menu items
const char* main_menu[] = {
  "1.Monitor",
  "2.Settings",
  "3.Calibration",
  "4.WiFi Config",
  "5.System Info",
  "6.Exit Menu"
};
const uint8_t main_menu_size = 6;

// Monitor submenu
const char* monitor_menu[] = {
  "NEPA Status",
  "Generator",
  "Load Power",
  "Energy Total",
  "Back"
};
const uint8_t monitor_menu_size = 5;

// Settings submenu
const char* settings_menu[] = {
  "Min Voltage",
  "Max Voltage",
  "Switch Delay",
  "Return Delay",
  "Auto Mode",
  "Back"
};
const uint8_t settings_menu_size = 6;

// Calibration submenu
const char* calibration_menu[] = {
  "NEPA Voltage",
  "Gen Voltage",
  "Current Cal",
  "Save & Exit",
  "Back"
};
const uint8_t calibration_menu_size = 5;

void setup() {
  Serial.begin(115200);

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // Initialize pins
  pinMode(NEPA_RELAY_PIN, OUTPUT);
  pinMode(GEN_RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);

  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_ENTER, INPUT_PULLUP);
  pinMode(MANUAL_MODE_PIN, INPUT_PULLUP);
  pinMode(RESET_ALARM_PIN, INPUT_PULLUP);

  // Initialize outputs to safe state
  digitalWrite(NEPA_RELAY_PIN, LOW);
  digitalWrite(GEN_RELAY_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // Initialize relay state tracking
  status.nepa_relay_on = false;
  status.gen_relay_on = false;

  // Initialize LCD
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Load saved data
  loadSystemData();
  loadWifiCredentials();

  // Startup sequence
  lcd.setCursor(0, 0);
  lcd.print(F("2-Way ATS v1.0"));
  lcd.setCursor(0, 1);
  lcd.print(F("ESP32 System"));
  delay(1000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Initializing..."));
  delay(1000);

  // Initialize WiFi
  WiFi.mode(WIFI_STA);
  WiFi.hostname(hostname);
  WiFi.begin(ssid, password);

  lcd.setCursor(0, 1);
  lcd.print(F("Connecting WiFi..."));

  // Wait for WiFi connection
  int wifi_timeout = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_timeout < 20) {
    delay(500);
    wifi_timeout++;
    lcd.setCursor(0, 1);
    lcd.print(F("WiFi:"));
    lcd.print(wifi_timeout * 0.5, 1);
    lcd.print(F("s"));
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("WiFi Connected!"));
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());
    delay(2000);
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("WiFi Failed"));
    lcd.setCursor(0, 1);
    lcd.print(F("Check Settings"));
    delay(2000);
  }

  // Setup web server routes
  setupWebServer();
  server.begin();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Press ENTER for"));
  lcd.setCursor(0, 1);
  lcd.print(F("Menu System"));
  delay(1000);

  Serial.println("=== 2-Way ATS ESP32 System Started ===");
  Serial.println("Firmware Version: 1.0 - ESP32");
  Serial.println("Press ENTER button to access menu");

  current_state = STATE_INIT;
  status.state_change_time = millis();
}

void loop() {
  unsigned long current_time = millis();

  // Handle web server
  server.handleClient();

  // Handle button inputs
  handleButtons();

  // Menu system
  if (menu_active) {
    handleMenu();
    // Auto-exit menu after 30 seconds of inactivity
    if (current_time - last_button_press > 30000) {
      exitMenu();
    }
    return; // Skip normal operation when in menu
  }

  // Read input switches
  bool reset_alarm = !digitalRead(RESET_ALARM_PIN);

  // Reset alarm if button pressed
  if (reset_alarm && status.alarm_active) {
    status.alarm_active = false;
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("Alarm Reset");
  }

  // Read AC power sources every second
  if (current_time - last_reading >= 1000) {
    readACPowerSources();
    last_reading = current_time;
  }

  // Update system state and control relays
  updateSystemState();
  controlRelays();

  // Update energy consumption every second
  if (current_time - last_energy_update >= 1000) {
    updateEnergyConsumption();
    last_energy_update = current_time;
  }

  // Update runtime statistics
  updateRuntime();

  // Normal display update every 3 seconds when not in menu
  if (current_time - last_display_update >= 3000) {
    updateNormalDisplay();
    last_display_update = current_time;
  }

  // Handle alarms and status indicators
  handleAlarms();

  // Save system data if modified or every hour
  static unsigned long last_save = 0;
  if ((settings_modified && current_time - last_save >= 60000) ||
      (current_time - last_save >= 3600000)) {
    saveSystemData();
    last_save = current_time;
  }

  // Small delay to prevent watchdog issues
  delay(10);
}

// Core ATS Functions
void readACPowerSources() {
  // Read NEPA voltage
  long vAdc_nepa = 0;
  if (readACVoltage(VOLT_PIN_NEPA, vAdc_nepa)) {
    volt_nepa = (vAdc_nepa * voltage_multiplier) + nepa_voltage_offset;
    if (volt_nepa < 0) volt_nepa = 0;
    if (volt_nepa > 400) volt_nepa = 0;
    status.nepa_valid = isVoltageValid(volt_nepa);
    sensorV_nepa = vAdc_nepa;
  } else {
    status.nepa_valid = false;
    volt_nepa = 0;
    sensorV_nepa = 0;
  }

  // Read Generator voltage
  long vAdc_gen = 0;
  if (readACVoltage(VOLT_PIN_GEN, vAdc_gen)) {
    volt_gen = (vAdc_gen * voltage_multiplier) + gen_voltage_offset;
    if (volt_gen < 0) volt_gen = 0;
    if (volt_gen > 400) volt_gen = 0;
    status.gen_valid = isVoltageValid(volt_gen);
    sensorV_gen = vAdc_gen;
  } else {
    status.gen_valid = false;
    volt_gen = 0;
    sensorV_gen = 0;
  }

  // Read load current from active source
  long chosenIAdc = 0;
  switch (active_source) {
    case SOURCE_NEPA:
      volt_load = volt_nepa;
      chosenIAdc = analogRead(CURR_PIN_LOAD);
      break;
    case SOURCE_GENERATOR:
      volt_load = volt_gen;
      chosenIAdc = analogRead(CURR_PIN_LOAD);
      break;
    default:
      // Use any available source for current measurement
      if (volt_nepa > 0) {
        volt_load = volt_nepa;
        chosenIAdc = analogRead(CURR_PIN_LOAD);
      } else if (volt_gen > 0) {
        volt_load = volt_gen;
        chosenIAdc = analogRead(CURR_PIN_LOAD);
      } else {
        volt_load = 0;
        chosenIAdc = 0;
      }
      break;
  }

  if (volt_load > 0 && chosenIAdc > 0) {
    amp_load = ((chosenIAdc * current_multiplier) / 3) + (current_offset * 0.01);
    if (amp_load < 0) amp_load = 0;
    if (amp_load > 100) amp_load = 0;
    watt_load = volt_load * amp_load;
    sensorI_load = chosenIAdc;
  } else {
    amp_load = 0;
    watt_load = 0;
    sensorI_load = 0;
  }

  // Attribute power to active source
  switch (active_source) {
    case SOURCE_NEPA:
      watt_nepa = watt_load;
      watt_gen = 0;
      break;
    case SOURCE_GENERATOR:
      watt_gen = watt_load;
      watt_nepa = 0;
      break;
    default:
      watt_nepa = watt_gen = 0;
      break;
  }

  status.load_detected = (watt_load > 10);

  // Serial output for debugging
  if (!menu_active) {
    Serial.println("=== 2-Way ATS Readings ===");
    Serial.printf("NEPA - Voltage: %dV, Valid: %s, Energy: %.2fkWh\n",
                  volt_nepa, status.nepa_valid ? "YES" : "NO", kwh_nepa);
    Serial.printf("GEN  - Voltage: %dV, Valid: %s, Energy: %.2fkWh\n",
                  volt_gen, status.gen_valid ? "YES" : "NO", kwh_gen);
    Serial.printf("LOAD - Voltage: %dV, Current: %.2fA, Power: %dW\n",
                  volt_load, amp_load, watt_load);
    Serial.printf("Active Source: %s\n",
                  active_source == SOURCE_NEPA ? "NEPA" :
                  active_source == SOURCE_GENERATOR ? "GENERATOR" : "NONE");
  }
}

bool readACVoltage(int analogPin, long& sensorValue) {
  unsigned long startTime = millis();

  // Wait for AC signal with timeout
  while (analogRead(analogPin) < MIN_AC_THRESHOLD) {
    if (millis() - startTime > AC_DETECTION_TIMEOUT) {
      sensorValue = 0;
      return false;
    }
    delay(1);
  }

  // Take samples
  sensorValue = 0;
  int minV = 4095, maxV = 0;
  for (int i = 0; i < SAMPLES_PER_READING; i++) {
    int reading = analogRead(analogPin);
    sensorValue += reading;
    if (reading < minV) minV = reading;
    if (reading > maxV) maxV = reading;
    delay(SAMPLE_DELAY_MS);
  }

  // Check if we have enough AC swing
  if ((maxV - minV) < AC_MIN_SPAN) {
    sensorValue = 0;
    return false;
  }

  sensorValue = sensorValue / SAMPLES_PER_READING;
  return true;
}

void updateSystemState() {
  unsigned long current_time = millis();
  unsigned long time_in_state = current_time - status.state_change_time;
  PowerSource best_source = selectBestSource();

  switch (current_state) {
    case STATE_INIT:
      // Immediately activate the best available source
      if (best_source != SOURCE_NONE) {
        if (best_source == SOURCE_NEPA) {
          current_state = STATE_NEPA_ACTIVE;
          active_source = SOURCE_NEPA;
        } else if (best_source == SOURCE_GENERATOR) {
          current_state = STATE_GEN_ACTIVE;
          active_source = SOURCE_GENERATOR;
        }
        status.state_change_time = current_time;
        Serial.printf("State: INIT -> %s_ACTIVE\n",
                      active_source == SOURCE_NEPA ? "NEPA" : "GEN");
      } else if (time_in_state > 1000) {
        // Force transition after 1 second
        if (volt_nepa > 100) {
          current_state = STATE_NEPA_ACTIVE;
          active_source = SOURCE_NEPA;
        } else if (volt_gen > 100) {
          current_state = STATE_GEN_ACTIVE;
          active_source = SOURCE_GENERATOR;
        }
        status.state_change_time = current_time;
      }
      break;

    case STATE_NEPA_ACTIVE:
      if (!status.nepa_valid || volt_nepa < MIN_VOLTAGE) {
        if (time_in_state > SWITCH_DELAY) {
          if (status.gen_valid && volt_gen > (MIN_VOLTAGE + VOLTAGE_HYSTERESIS)) {
            current_state = STATE_GEN_ACTIVE;
            active_source = SOURCE_GENERATOR;
            status.switch_count++;
            settings_modified = true;
            Serial.println("State: NEPA_ACTIVE -> GEN_ACTIVE");
          } else {
            current_state = STATE_FAULT;
            status.system_fault = true;
            status.fault_code = 1; // NEPA failure
            Serial.println("State: NEPA_ACTIVE -> FAULT");
          }
          status.state_change_time = current_time;
        }
      }
      break;

    case STATE_GEN_ACTIVE:
      // Check if NEPA has returned
      if (status.nepa_valid && volt_nepa > (MIN_VOLTAGE + VOLTAGE_HYSTERESIS)) {
        if (time_in_state > SOURCE_RETURN_DELAY) {
          current_state = STATE_NEPA_ACTIVE;
          active_source = SOURCE_NEPA;
          status.gen_running = false;
          status.state_change_time = current_time;
          status.switch_count++;
          settings_modified = true;
          Serial.println("State: GEN_ACTIVE -> NEPA_ACTIVE");
        }
      } else if (!status.gen_valid || volt_gen < MIN_VOLTAGE) {
        if (time_in_state > SWITCH_DELAY) {
          current_state = STATE_FAULT;
          status.system_fault = true;
          status.fault_code = 2; // Generator failure
          status.state_change_time = current_time;
          Serial.println("State: GEN_ACTIVE -> FAULT");
        }
      }
      break;

    case STATE_FAULT:
      // Try to recover every 30 seconds
      if (time_in_state > 30000) {
        status.system_fault = false;
        current_state = STATE_INIT;
        status.state_change_time = current_time;
        Serial.println("State: FAULT -> INIT (recovery attempt)");
      }
      break;

    default:
      current_state = STATE_INIT;
      status.state_change_time = current_time;
      break;
  }
}

PowerSource selectBestSource() {
  // Priority: NEPA > Generator
  if (status.nepa_valid && volt_nepa >= MIN_VOLTAGE) {
    return SOURCE_NEPA;
  } else if (status.gen_valid && volt_gen >= MIN_VOLTAGE) {
    return SOURCE_GENERATOR;
  }
  return SOURCE_NONE;
}

void controlRelays() {
  if (current_state == STATE_SWITCHING || current_state == STATE_FAULT) {
    return;
  }

  bool nepa_should_be_on = (active_source == SOURCE_NEPA);
  bool gen_should_be_on = (active_source == SOURCE_GENERATOR);

  // Only change relay states if they need to change
  if (nepa_should_be_on != status.nepa_relay_on) {
    if (nepa_should_be_on) {
      // Turn off generator first
      if (status.gen_relay_on) {
        digitalWrite(GEN_RELAY_PIN, LOW);
        status.gen_relay_on = false;
        delay(5); // Break-before-make
      }
      digitalWrite(NEPA_RELAY_PIN, HIGH);
      status.nepa_relay_on = true;
      Serial.println("NEPA relay activated");
    } else {
      digitalWrite(NEPA_RELAY_PIN, LOW);
      status.nepa_relay_on = false;
      Serial.println("NEPA relay deactivated");
    }
  }

  if (gen_should_be_on != status.gen_relay_on) {
    if (gen_should_be_on) {
      // Turn off NEPA first
      if (status.nepa_relay_on) {
        digitalWrite(NEPA_RELAY_PIN, LOW);
        status.nepa_relay_on = false;
        delay(5); // Break-before-make
      }
      digitalWrite(GEN_RELAY_PIN, HIGH);
      status.gen_relay_on = true;
      Serial.println("Generator relay activated");
    } else {
      digitalWrite(GEN_RELAY_PIN, LOW);
      status.gen_relay_on = false;
      Serial.println("Generator relay deactivated");
    }
  }
}

void updateEnergyConsumption() {
  if (watt_load > 0) {
    // Convert watts to kWh (1 second = 1/3600 hour)
    float energy_increment = (watt_load * 0.001f) / 3600.0f;

    switch (active_source) {
      case SOURCE_NEPA:
        kwh_nepa += energy_increment;
        break;
      case SOURCE_GENERATOR:
        kwh_gen += energy_increment;
        break;
      default:
        break;
    }

    kwh_total = kwh_nepa + kwh_gen;
    settings_modified = true;
  }
}

bool isVoltageValid(uint16_t voltage) {
  return (voltage >= MIN_VOLTAGE && voltage <= MAX_VOLTAGE);
}

void handleAlarms() {
  // Status LED patterns
  if (current_state == STATE_FAULT || status.system_fault) {
    digitalWrite(STATUS_LED_PIN, (millis() / 250) % 2); // Fast blink - fault
    if (!status.alarm_active) {
      status.alarm_active = true;
      digitalWrite(BUZZER_PIN, HIGH);
    }
  } else if (current_state == STATE_GEN_ACTIVE) {
    digitalWrite(STATUS_LED_PIN, (millis() / 1000) % 2); // Slow blink - generator
  } else {
    digitalWrite(STATUS_LED_PIN, HIGH); // Steady on - NEPA or normal
  }

  // Turn off buzzer after 30 seconds
  static unsigned long alarm_start = 0;
  if (status.alarm_active) {
    if (alarm_start == 0) alarm_start = millis();
    if (millis() - alarm_start > 30000) {
      digitalWrite(BUZZER_PIN, LOW);
      alarm_start = 0;
    }
  } else {
    alarm_start = 0;
  }
}

// Menu and Display Functions
void handleButtons() {
  static bool last_up = HIGH, last_down = HIGH, last_enter = HIGH;
  static unsigned long debounce_time = 0;

  bool current_up = digitalRead(BTN_UP);
  bool current_down = digitalRead(BTN_DOWN);
  bool current_enter = digitalRead(BTN_ENTER);

  // Debounce buttons
  if (millis() - debounce_time > 200) {
    if (current_up == LOW && last_up == HIGH) {
      button_up_pressed = true;
      last_button_press = millis();
      debounce_time = millis();
    }

    if (current_down == LOW && last_down == HIGH) {
      button_down_pressed = true;
      last_button_press = millis();
      debounce_time = millis();
    }

    if (current_enter == LOW && last_enter == HIGH) {
      button_enter_pressed = true;
      last_button_press = millis();
      debounce_time = millis();

      // Enter menu if not already active
      if (!menu_active) {
        menu_active = true;
        current_menu = MENU_MAIN;
        menu_index = 0;
        in_submenu = false;
      }
    }
  }

  last_up = current_up;
  last_down = current_down;
  last_enter = current_enter;
}

void handleMenu() {
  if (button_up_pressed) {
    button_up_pressed = false;

    if (in_submenu) {
      submenu_index--;
      if (submenu_index < 0) {
        switch (current_menu) {
          case MENU_MONITOR: submenu_index = monitor_menu_size - 1; break;
          case MENU_SETTINGS: submenu_index = settings_menu_size - 1; break;
          case MENU_CALIBRATION: submenu_index = calibration_menu_size - 1; break;
          default: submenu_index = 0; break;
        }
      }
    } else {
      menu_index--;
      if (menu_index < 0) menu_index = main_menu_size - 1;
    }
    displayMenu();
  }

  if (button_down_pressed) {
    button_down_pressed = false;

    if (in_submenu) {
      submenu_index++;
      switch (current_menu) {
        case MENU_MONITOR:
          if (submenu_index >= monitor_menu_size) submenu_index = 0;
          break;
        case MENU_SETTINGS:
          if (submenu_index >= settings_menu_size) submenu_index = 0;
          break;
        case MENU_CALIBRATION:
          if (submenu_index >= calibration_menu_size) submenu_index = 0;
          break;
        default:
          submenu_index = 0;
          break;
      }
    } else {
      menu_index++;
      if (menu_index >= main_menu_size) menu_index = 0;
    }
    displayMenu();
  }

  if (button_enter_pressed) {
    button_enter_pressed = false;
    handleMenuSelection();
  }
}

void displayMenu() {
  lcd.clear();

  if (!in_submenu) {
    // Display main menu
    lcd.setCursor(0, 0);
    lcd.print(F("MAIN MENU"));
    lcd.setCursor(0, 1);
    lcd.print(F(">"));
    lcd.print(main_menu[menu_index]);
  } else {
    // Display submenu
    lcd.setCursor(0, 0);
    switch (current_menu) {
      case MENU_MONITOR:
        lcd.print(F("MONITOR"));
        lcd.setCursor(0, 1);
        lcd.print(F(">"));
        lcd.print(monitor_menu[submenu_index]);
        break;

      case MENU_SETTINGS:
        lcd.print(F("SETTINGS"));
        lcd.setCursor(0, 1);
        lcd.print(F(">"));
        lcd.print(settings_menu[submenu_index]);
        break;

      case MENU_CALIBRATION:
        lcd.print(F("CALIBRATION"));
        lcd.setCursor(0, 1);
        lcd.print(F(">"));
        lcd.print(calibration_menu[submenu_index]);
        break;

      default:
        lcd.print(F("Menu Error"));
        break;
    }
  }
}

void handleMenuSelection() {
  if (!in_submenu) {
    // Main menu selection
    switch (menu_index) {
      case 0: // Monitor
        current_menu = MENU_MONITOR;
        in_submenu = true;
        submenu_index = 0;
        break;

      case 1: // Settings
        current_menu = MENU_SETTINGS;
        in_submenu = true;
        submenu_index = 0;
        break;

      case 2: // Calibration
        current_menu = MENU_CALIBRATION;
        in_submenu = true;
        submenu_index = 0;
        break;

      case 3: // WiFi Config
        handleWiFiConfig();
        break;

      case 4: // System Info
        displaySystemInfo();
        break;

      case 5: // Exit Menu
        exitMenu();
        break;

      default:
        break;
    }
  } else {
    // Submenu selection
    switch (current_menu) {
      case MENU_MONITOR:
        handleMonitorMenu();
        break;

      case MENU_SETTINGS:
        handleSettingsMenu();
        break;

      case MENU_CALIBRATION:
        handleEnhancedCalibrationMenu();
        break;

      default:
        break;
    }
  }

  displayMenu();
}

void handleMonitorMenu() {
  switch (submenu_index) {
    case 0: // NEPA Status
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("NEPA:"));
      lcd.print(volt_nepa);
      lcd.print(F("V"));
      lcd.setCursor(0, 1);
      lcd.print(status.nepa_valid ? F("HEALTHY") : F("FAILED"));
      lcd.print(" ");
      lcd.print(kwh_nepa, 2);
      lcd.print(F("kWh"));
      delay(2000);
      break;

    case 1: // Generator
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("GEN:"));
      lcd.print(volt_gen);
      lcd.print(F("V"));
      lcd.setCursor(0, 1);
      lcd.print(status.gen_running ? F("RUNNING ") : F("STOPPED "));
      lcd.print(kwh_gen, 2);
      lcd.print(F("kWh"));
      delay(2000);
      break;

    case 2: // Load Power
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("LOAD:"));
      lcd.print(amp_load, 1);
      lcd.print(F("A"));
      lcd.setCursor(0, 1);
      lcd.print(F("Power:"));
      lcd.print(watt_load);
      lcd.print(F("W"));
      delay(2000);
      break;

    case 3: // Energy Total
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Total Energy:"));
      lcd.setCursor(0, 1);
      lcd.print(kwh_total, 2);
      lcd.print("kWh");
      delay(2000);
      break;

    case 4: // Back
      in_submenu = false;
      current_menu = MENU_MAIN;
      break;

    default:
      break;
  }
}

void handleSettingsMenu() {
  static bool editing = false;
  static int edit_value = 0;

  if (!editing) {
    switch (submenu_index) {
      case 0: // Min Voltage
        edit_value = MIN_VOLTAGE;
        editing = true;
        break;
      case 1: // Max Voltage
        edit_value = MAX_VOLTAGE;
        editing = true;
        break;
      case 2: // Switch Delay
        edit_value = SWITCH_DELAY / 1000;
        editing = true;
        break;
      case 3: // Return Delay
        edit_value = SOURCE_RETURN_DELAY / 1000;
        editing = true;
        break;
      case 4: // Auto Mode
        auto_mode = !auto_mode;
        settings_modified = true;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Auto Mode:"));
        lcd.setCursor(0, 1);
        lcd.print(auto_mode ? F("ENABLED") : F("DISABLED"));
        delay(2000);
        break;
      case 5: // Back
        in_submenu = false;
        current_menu = MENU_MAIN;
        break;
      default:
        break;
    }
  } else {
    // Handle value editing
    if (button_up_pressed) {
      button_up_pressed = false;
      edit_value++;
      // Add bounds checking
      switch (submenu_index) {
        case 0: if (edit_value > 200) edit_value = 200; break;
        case 1: if (edit_value > 300) edit_value = 300; break;
        case 2: if (edit_value > 60) edit_value = 60; break;
        case 3: if (edit_value > 600) edit_value = 600; break;
      }
    }
    if (button_down_pressed) {
      button_down_pressed = false;
      edit_value--;
      // Add bounds checking
      switch (submenu_index) {
        case 0: if (edit_value < 100) edit_value = 100; break;
        case 1: if (edit_value < 200) edit_value = 200; break;
        case 2: if (edit_value < 1) edit_value = 1; break;
        case 3: if (edit_value < 30) edit_value = 30; break;
      }
    }

    // Display editing screen
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(settings_menu[submenu_index]);
    lcd.setCursor(0, 1);
    lcd.print(F("Value:"));
    lcd.print(edit_value);

    if (button_enter_pressed) {
      button_enter_pressed = false;
      // Save the edited value
      switch (submenu_index) {
        case 0: MIN_VOLTAGE = edit_value; break;
        case 1: MAX_VOLTAGE = edit_value; break;
        case 2: SWITCH_DELAY = edit_value * 1000; break;
        case 3: SOURCE_RETURN_DELAY = edit_value * 1000; break;
        default: break;
      }
      editing = false;
      settings_modified = true;

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Setting Saved!"));
      delay(1500);
    }
  }
}

void handleCalibrationMenu() {
  static bool calibrating = false;
  static int cal_value = 0;
  static uint8_t cal_source = 0;
  static unsigned long last_cal_display = 0;

  if (!calibrating) {
    switch (submenu_index) {
      case 0: // NEPA Voltage
        cal_value = nepa_voltage_offset;
        cal_source = 0;
        calibrating = true;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("NEPA Calibration"));
        lcd.setCursor(0, 1);
        lcd.print(F("Press ENTER"));
        delay(1500);
        break;
      case 1: // Gen Voltage
        cal_value = gen_voltage_offset;
        cal_source = 1;
        calibrating = true;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("GEN Calibration"));
        lcd.setCursor(0, 1);
        lcd.print(F("Press ENTER"));
        delay(1500);
        break;
      case 2: // Current
        cal_value = current_offset;
        cal_source = 2;
        calibrating = true;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Current Calibration"));
        lcd.setCursor(0, 1);
        lcd.print(F("Press ENTER"));
        delay(1500);
        break;
      case 3: // Save & Exit
        saveCalibrationData();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Calibration"));
        lcd.setCursor(0, 1);
        lcd.print(F("Saved!"));
        delay(2000);
        exitMenu();
        break;
      case 4: // Back
        in_submenu = false;
        current_menu = MENU_MAIN;
        break;
      default:
        break;
    }
  } else {
    // Handle calibration adjustment with live feedback
    if (button_up_pressed) {
      button_up_pressed = false;
      cal_value++;
      if (cal_value > 100) cal_value = 100;
    }

    if (button_down_pressed) {
      button_down_pressed = false;
      cal_value--;
      if (cal_value < -100) cal_value = -100;
    }

    // Update calibration display every 500ms for live feedback
    if (millis() - last_cal_display >= 500) {
      last_cal_display = millis();
      displayCalibrationScreen(cal_source, cal_value);
    }

    if (button_enter_pressed) {
      button_enter_pressed = false;
      // Save calibration value
      switch (cal_source) {
        case 0:
          nepa_voltage_offset = cal_value;
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("NEPA Cal Saved"));
          lcd.setCursor(0, 1);
          lcd.print(F("Offset:"));
          lcd.print(cal_value);
          break;
        case 1:
          gen_voltage_offset = cal_value;
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("GEN Cal Saved"));
          lcd.setCursor(0, 1);
          lcd.print(F("Offset:"));
          lcd.print(cal_value);
          break;
        case 2:
          current_offset = cal_value;
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Current Cal Saved"));
          lcd.setCursor(0, 1);
          lcd.print(F("Offset:"));
          lcd.print(cal_value);
          break;
        default:
          break;
      }

      calibrating = false;
      settings_modified = true;
      delay(2000);
    }
  }
}

void displayCalibrationScreen(uint8_t cal_source, int cal_value) {
  lcd.clear();

  switch (cal_source) {
    case 0: // NEPA Voltage
      lcd.setCursor(0, 0);
      lcd.print(F("NEPA Voltage Cal"));
      lcd.setCursor(0, 1);

      // Show live voltage reading with current calibration
      int raw_voltage = sensorV_nepa * voltage_multiplier;
      int live_calibrated = raw_voltage + cal_value;

      lcd.print(F("Raw:"));
      lcd.print(raw_voltage);
      lcd.print(F("V"));

      // Show offset value on second line
      lcd.setCursor(0, 1);
      lcd.print(F("Cal:"));
      lcd.print(live_calibrated);
      lcd.print(F("V Off:"));
      lcd.print(cal_value);
      break;

    case 1: // Generator Voltage
      lcd.setCursor(0, 0);
      lcd.print(F("GEN Voltage Cal"));
      lcd.setCursor(0, 1);

      // Show live voltage reading with current calibration
      int raw_voltage_gen = sensorV_gen * voltage_multiplier;
      int live_calibrated_gen = raw_voltage_gen + cal_value;

      lcd.print(F("Raw:"));
      lcd.print(raw_voltage_gen);
      lcd.print(F("V"));

      // Show offset value on second line
      lcd.setCursor(0, 1);
      lcd.print(F("Cal:"));
      lcd.print(live_calibrated_gen);
      lcd.print(F("V Off:"));
      lcd.print(cal_value);
      break;

    case 2: // Current
      lcd.setCursor(0, 0);
      lcd.print(F("Current Calibration"));
      lcd.setCursor(0, 1);

      // Show live current reading with current calibration
      float raw_current = (sensorI_load * current_multiplier) / 3;
      float live_calibrated_current = raw_current + (cal_value * 0.01);

      lcd.print(F("Raw:"));
      lcd.print(raw_current, 1);
      lcd.print(F("A"));

      // Show offset value on second line
      lcd.setCursor(0, 1);
      lcd.print(F("Cal:"));
      lcd.print(live_calibrated_current, 1);
      lcd.print(F("A Off:"));
      lcd.print(cal_value);
      break;

    default:
      break;
  }
}

// Enhanced calibration helper functions
void startCalibrationMode() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Calibration Mode"));
  lcd.setCursor(0, 1);
  lcd.print(F("Active"));
  delay(1000);
}

void showCalibrationInstructions(uint8_t cal_source) {
  lcd.clear();
  lcd.setCursor(0, 0);

  switch (cal_source) {
    case 0:
      lcd.print(F("NEPA Calibration"));
      lcd.setCursor(0, 1);
      lcd.print(F("Use UP/DOWN to adjust"));
      break;
    case 1:
      lcd.print(F("GEN Calibration"));
      lcd.setCursor(0, 1);
      lcd.print(F("Use UP/DOWN to adjust"));
      break;
    case 2:
      lcd.print(F("Current Calibration"));
      lcd.setCursor(0, 1);
      lcd.print(F("Use UP/DOWN to adjust"));
      break;
  }
  delay(2000);
}

// Improved calibration validation
bool validateCalibrationValue(int value, uint8_t cal_source) {
  switch (cal_source) {
    case 0: // NEPA Voltage
    case 1: // Generator Voltage
      return (value >= -100 && value <= 100);
    case 2: // Current
      return (value >= -100 && value <= 100);
    default:
      return false;
  }
}

// Auto-calibration function for voltage sensors
void autoCalibrateVoltage(uint8_t cal_source) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Auto-Calibrating..."));

  // Take multiple readings and average them
  long total_reading = 0;
  int valid_readings = 0;

  for (int i = 0; i < 10; i++) {
    int reading = 0;
    if (cal_source == 0) {
      reading = analogRead(VOLT_PIN_NEPA);
    } else if (cal_source == 1) {
      reading = analogRead(VOLT_PIN_GEN);
    }

    if (reading > 0) {
      total_reading += reading;
      valid_readings++;
    }
    delay(100);
  }

  if (valid_readings > 0) {
    long avg_reading = total_reading / valid_readings;
    float measured_voltage = avg_reading * voltage_multiplier;

    // Calculate offset to reach target voltage (assuming 230V is target)
    int calculated_offset = 230 - measured_voltage;

    // Apply bounds checking
    if (calculated_offset < -100) calculated_offset = -100;
    if (calculated_offset > 100) calculated_offset = 100;

    // Apply the calculated offset
    switch (cal_source) {
      case 0:
        nepa_voltage_offset = calculated_offset;
        break;
      case 1:
        gen_voltage_offset = calculated_offset;
        break;
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Auto-Cal Complete"));
    lcd.setCursor(0, 1);
    lcd.print(F("Offset:"));
    lcd.print(calculated_offset);
    settings_modified = true;
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Auto-Cal Failed"));
    lcd.setCursor(0, 1);
    lcd.print(F("No valid readings"));
  }

  delay(3000);
}

// Enhanced calibration menu with auto-calibration option
void handleEnhancedCalibrationMenu() {
  static bool calibrating = false;
  static int cal_value = 0;
  static uint8_t cal_source = 0;
  static unsigned long last_cal_display = 0;

  if (!calibrating) {
    switch (submenu_index) {
      case 0: // NEPA Voltage
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("NEPA Voltage"));
        lcd.setCursor(0, 1);
        lcd.print(F("1.Manual 2.Auto"));
        delay(2000);

        // Wait for user choice
        unsigned long choice_start = millis();
        while (millis() - choice_start < 5000) {
          if (button_up_pressed) {
            button_up_pressed = false;
            // Manual calibration
            cal_value = nepa_voltage_offset;
            cal_source = 0;
            calibrating = true;
            showCalibrationInstructions(cal_source);
            break;
          }
          if (button_down_pressed) {
            button_down_pressed = false;
            // Auto calibration
            autoCalibrateVoltage(0);
            return;
          }
          delay(100);
        }
        break;

      case 1: // Gen Voltage
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("GEN Voltage"));
        lcd.setCursor(0, 1);
        lcd.print(F("1.Manual 2.Auto"));
        delay(2000);

        // Wait for user choice
        choice_start = millis();
        while (millis() - choice_start < 5000) {
          if (button_up_pressed) {
            button_up_pressed = false;
            // Manual calibration
            cal_value = gen_voltage_offset;
            cal_source = 1;
            calibrating = true;
            showCalibrationInstructions(cal_source);
            break;
          }
          if (button_down_pressed) {
            button_down_pressed = false;
            // Auto calibration
            autoCalibrateVoltage(1);
            return;
          }
          delay(100);
        }
        break;

      case 2: // Current
        cal_value = current_offset;
        cal_source = 2;
        calibrating = true;
        showCalibrationInstructions(cal_source);
        break;

      case 3: // Save & Exit
        saveCalibrationData();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Calibration"));
        lcd.setCursor(0, 1);
        lcd.print(F("Saved!"));
        delay(2000);
        exitMenu();
        break;

      case 4: // Back
        in_submenu = false;
        current_menu = MENU_MAIN;
        break;

      default:
        break;
    }
  } else {
    // Handle calibration adjustment with live feedback
    if (button_up_pressed) {
      button_up_pressed = false;
      cal_value++;
      if (cal_value > 100) cal_value = 100;
    }

    if (button_down_pressed) {
      button_down_pressed = false;
      cal_value--;
      if (cal_value < -100) cal_value = -100;
    }

    // Update calibration display every 500ms for live feedback
    if (millis() - last_cal_display >= 500) {
      last_cal_display = millis();
      displayCalibrationScreen(cal_source, cal_value);
    }

    if (button_enter_pressed) {
      button_enter_pressed = false;

      // Validate calibration value
      if (validateCalibrationValue(cal_value, cal_source)) {
        // Save calibration value
        switch (cal_source) {
          case 0:
            nepa_voltage_offset = cal_value;
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("NEPA Cal Saved"));
            lcd.setCursor(0, 1);
            lcd.print(F("Offset:"));
            lcd.print(cal_value);
            break;
          case 1:
            gen_voltage_offset = cal_value;
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("GEN Cal Saved"));
            lcd.setCursor(0, 1);
            lcd.print(F("Offset:"));
            lcd.print(cal_value);
            break;
          case 2:
            current_offset = cal_value;
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("Current Cal Saved"));
            lcd.setCursor(0, 1);
            lcd.print(F("Offset:"));
            lcd.print(cal_value);
            break;
          default:
            break;
        }

        calibrating = false;
        settings_modified = true;
        delay(2000);
      } else {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Invalid Value"));
        lcd.setCursor(0, 1);
        lcd.print(F("Try Again"));
        delay(2000);
      }
    }
  }
}

void handleWiFiConfig() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("WiFi Status"));

  lcd.setCursor(0, 1);
  if (wifi_connected) {
    lcd.print(F("Connected"));
  } else {
    lcd.print(F("Not Connected"));
  }
  delay(2000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("SSID:"));
  lcd.setCursor(0, 1);
  if (strlen(ssid) > 0) {
    lcd.print(ssid);
  } else {
    lcd.print(F("Not Set"));
  }
  delay(2000);

  if (wifi_connected) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("IP Address:"));
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());
    delay(3000);
  }
}

void displaySystemInfo() {
  for (int screen = 0; screen < 3; screen++) {
    lcd.clear();

    switch (screen) {
      case 0:
        lcd.setCursor(0, 0);
        lcd.print(F("ATS System v1.0"));
        lcd.setCursor(0, 1);
        lcd.print(F("Runtime:"));
        lcd.print(status.runtime_hours);
        lcd.print(F("H"));
        break;

      case 1:
        lcd.setCursor(0, 0);
        lcd.print(F("Switches:"));
        lcd.print(status.switch_count);
        lcd.setCursor(0, 1);
        lcd.print(F("Faults:"));
        lcd.print(status.fault_code);
        break;

      case 2:
        lcd.setCursor(0, 0);
        lcd.print(F("Active:"));
        switch (active_source) {
          case SOURCE_NEPA: lcd.print(F("NEPA")); break;
          case SOURCE_GENERATOR: lcd.print(F("GEN")); break;
          default: lcd.print(F("NONE")); break;
        }
        lcd.setCursor(0, 1);
        lcd.print(F("Mode:"));
        lcd.print(auto_mode ? F("AUTO") : F("MANUAL"));
        break;

      default:
        break;
    }

    delay(2000);

    // Check for button press to exit early
    handleButtons();
    if (button_enter_pressed) {
      button_enter_pressed = false;
      break;
    }
  }
}

void updateNormalDisplay() {
  if (menu_active) return;

  lcd.clear();

  switch (display_screen) {
    case 0: // System Status
      lcd.setCursor(0, 0);
      lcd.print(F("ATS "));
      lcd.print(auto_mode ? F("AUTO") : F("MAN"));
      lcd.print(F(" "));
      switch (active_source) {
        case SOURCE_NEPA: lcd.print(F("NEPA")); break;
        case SOURCE_GENERATOR: lcd.print(F("GEN")); break;
        default: lcd.print(F("NONE")); break;
      }

      lcd.setCursor(0, 1);
      switch (current_state) {
        case STATE_NEPA_ACTIVE: lcd.print(F("NEPA ACTIVE")); break;
        case STATE_GEN_ACTIVE: lcd.print(F("GEN ACTIVE")); break;
        case STATE_SWITCHING: lcd.print(F("SWITCHING")); break;
        case STATE_FAULT:
          lcd.print(F("FAULT:"));
          lcd.print(status.fault_code);
          break;
        default: lcd.print(F("INITIALIZING")); break;
      }
      break;

    case 1: // Source Voltages
      lcd.setCursor(0, 0);
      lcd.print(F("N:"));
      lcd.print(volt_nepa);
      lcd.print(F(" G:"));
      lcd.print(volt_gen);
      lcd.setCursor(0, 1);
      lcd.print(F("Load:"));
      lcd.print(volt_load);
      lcd.print(F("V "));
      lcd.print(amp_load, 1);
      lcd.print(F("A"));
      break;

    case 2: // Power and Energy
      lcd.setCursor(0, 0);
      lcd.print(F("Power:"));
      lcd.print(watt_load);
      lcd.print(F("W"));
      lcd.setCursor(0, 1);
      lcd.print(F("Energy:"));
      lcd.print(kwh_total, 1);
      lcd.print(F("kWh"));
      break;

    default:
      display_screen = 0;
      break;
  }

  display_screen++;
  if (display_screen > 2) display_screen = 0;
}

void exitMenu() {
  menu_active = false;
  in_submenu = false;
  current_menu = MENU_MAIN;
  menu_index = 0;
  submenu_index = 0;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Exiting Menu..."));
  delay(1000);
}

// Web Server Functions
void setupWebServer() {
  // Main status page
  server.on("/", HTTP_GET, handleRoot);
  server.on("/wifi", HTTP_GET, handleWifiConfigPage);
  server.on("/savewifi", HTTP_POST, handleSaveWifiConfig);

  // API endpoints
  server.on("/api/status", HTTP_GET, handleStatusAPI);
  server.on("/api/control", HTTP_POST, handleControlAPI);
  server.on("/api/settings", HTTP_GET, handleSettingsAPI);
  server.on("/api/settings", HTTP_POST, handleSettingsUpdate);

  // Static files (if needed)
  server.onNotFound(handleNotFound);

  Serial.println("Web server routes configured");
}

void handleSaveWifiConfig() {
  String new_ssid = server.arg("ssid");
  String new_pass = server.arg("password");

  if (new_ssid.length() > 0) {
    saveWifiCredentials(new_ssid.c_str(), new_pass.c_str());

    String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>WiFi Configuration Saved</title>
    <meta http-equiv="refresh" content="5;url=/">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background-color: #f0f0f0; text-align: center; }
        .container { max-width: 800px; margin: 50px auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
    </style>
</head>
<body>
    <div class="container">
        <h1>Settings Saved!</h1>
        <p>Your new WiFi settings have been saved. The device will now restart.</p>
        <p>You will be redirected to the dashboard in 5 seconds. If not, please connect to the new WiFi network and navigate to the device's IP address.</p>
    </div>
</body>
</html>
    )";
    server.send(200, "text/html", html);

    delay(1000);
    ESP.restart();
  } else {
    server.send(400, "text/plain", "SSID cannot be empty.");
  }
}

void handleWifiConfigPage() {
  String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>WiFi Configuration</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background-color: #f0f0f0; }
        .container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
        .form-group { margin-bottom: 15px; }
        label { display: block; margin-bottom: 5px; }
        input[type="text"], input[type="password"] { width: 100%; padding: 8px; border: 1px solid #ccc; border-radius: 4px; }
        .btn { background: #007bff; color: white; padding: 10px 20px; border: none; border-radius: 5px; cursor: pointer; }
        .btn:hover { background: #0056b3; }
        .notice { background-color: #fff3cd; border: 1px solid #ffeeba; color: #856404; padding: 10px; border-radius: 5px; margin-top: 20px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>WiFi Configuration</h1>
        <p>Enter your WiFi network credentials below. The device will restart after saving.</p>
        <form action="/savewifi" method="POST">
            <div class="form-group">
                <label for="ssid">SSID</label>
                <input type="text" id="ssid" name="ssid" required>
            </div>
            <div class="form-group">
                <label for="password">Password</label>
                <input type="password" id="password" name="password">
            </div>
            <button type="submit" class="btn">Save and Restart</button>
        </form>
        <div class="notice">
            <strong>Note:</strong> The device will restart to apply the new settings. Please reconnect to the new WiFi network if the credentials change.
        </div>
        <p><a href="/">Back to Dashboard</a></p>
    </div>
</body>
</html>
  )";
  server.send(200, "text/html", html);
}

void handleRoot() {
  String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>2-Way ATS System</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background-color: #f0f0f0; }
        .container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
        .status-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 20px; margin: 20px 0; }
        .status-card { background: #f8f9fa; padding: 15px; border-radius: 8px; border-left: 4px solid #007bff; }
        .status-card.nepa { border-left-color: #28a745; }
        .status-card.gen { border-left-color: #ffc107; }
        .status-card.fault { border-left-color: #dc3545; }
        .btn { background: #007bff; color: white; padding: 10px 20px; border: none; border-radius: 5px; cursor: pointer; margin: 5px; }
        .btn:hover { background: #0056b3; }
        .btn.danger { background: #dc3545; }
        .btn.danger:hover { background: #c82333; }
        .btn.success { background: #28a745; }
        .btn.success:hover { background: #218838; }
        .refresh { text-align: center; margin: 20px 0; }
        .auto-refresh { margin: 20px 0; text-align: center; }
    </style>
</head>
<body>
    <div class="container">
        <h1>2-Way ATS System</h1>
        <div class="auto-refresh">
            <label><input type="checkbox" id="autoRefresh" checked> Auto-refresh every 5 seconds</label>
        </div>

        <div class="status-grid">
            <div class="status-card" id="nepaCard">
                <h3>NEPA Power</h3>
                <p>Voltage: <span id="nepaVoltage">--</span>V</p>
                <p>Status: <span id="nepaStatus">--</span></p>
                <p>Energy: <span id="nepaEnergy">--</span>kWh</p>
            </div>

            <div class="status-card" id="genCard">
                <h3>Generator</h3>
                <p>Voltage: <span id="genVoltage">--</span>V</p>
                <p>Status: <span id="genStatus">--</span></p>
                <p>Energy: <span id="genEnergy">--</span>kWh</p>
            </div>

            <div class="status-card" id="loadCard">
                <h3>Load</h3>
                <p>Voltage: <span id="loadVoltage">--</span>V</p>
                <p>Current: <span id="loadCurrent">--</span>A</p>
                <p>Power: <span id="loadPower">--</span>W</p>
            </div>

            <div class="status-card" id="systemCard">
                <h3>System</h3>
                <p>Active Source: <span id="activeSource">--</span></p>
                <p>State: <span id="systemState">--</span></p>
                <p>Total Energy: <span id="totalEnergy">--</span>kWh</p>
            </div>
        </div>

        <div class="refresh">
            <button class="btn" onclick="refreshStatus()">Refresh Now</button>
            <button class="btn success" onclick="setAutoMode(true)">Enable Auto</button>
            <button class="btn danger" onclick="setAutoMode(false)">Disable Auto</button>
        </div>

        <div class="refresh">
            <button class="btn success" onclick="forceSource('nepa')">Force NEPA</button>
            <button class="btn success" onclick="forceSource('gen')">Force Generator</button>
        </div>
        <div class="refresh">
            <a href="/wifi" class="btn">WiFi Settings</a>
        </div>
    </div>

    <script>
        let autoRefreshInterval;

        function startAutoRefresh() {
            if (autoRefreshInterval) clearInterval(autoRefreshInterval);
            autoRefreshInterval = setInterval(refreshStatus, 5000);
        }

        function stopAutoRefresh() {
            if (autoRefreshInterval) clearInterval(autoRefreshInterval);
        }

        document.getElementById('autoRefresh').addEventListener('change', function() {
            if (this.checked) {
                startAutoRefresh();
            } else {
                stopAutoRefresh();
            }
        });

        async function refreshStatus() {
            try {
                const response = await fetch('/api/status');
                const data = await response.json();
                updateDisplay(data);
            } catch (error) {
                console.error('Error fetching status:', error);
            }
        }

        function updateDisplay(data) {
            document.getElementById('nepaVoltage').textContent = data.nepa_voltage;
            document.getElementById('nepaStatus').textContent = data.nepa_valid ? 'HEALTHY' : 'FAILED';
            document.getElementById('nepaEnergy').textContent = data.nepa_energy.toFixed(2);

            document.getElementById('genVoltage').textContent = data.gen_voltage;
            document.getElementById('genStatus').textContent = data.gen_valid ? 'HEALTHY' : 'FAILED';
            document.getElementById('genEnergy').textContent = data.gen_energy.toFixed(2);

            document.getElementById('loadVoltage').textContent = data.load_voltage;
            document.getElementById('loadCurrent').textContent = data.load_current.toFixed(1);
            document.getElementById('loadPower').textContent = data.load_power;

            document.getElementById('activeSource').textContent = data.active_source;
            document.getElementById('systemState').textContent = data.system_state;
            document.getElementById('totalEnergy').textContent = data.total_energy.toFixed(2);

            // Update card colors based on status
            updateCardColors(data);
        }

        function updateCardColors(data) {
            const nepaCard = document.getElementById('nepaCard');
            const genCard = document.getElementById('genCard');
            const systemCard = document.getElementById('systemCard');

            nepaCard.className = 'status-card ' + (data.nepa_valid ? 'nepa' : 'fault');
            genCard.className = 'status-card ' + (data.gen_valid ? 'gen' : 'fault');
            systemCard.className = 'status-card ' + (data.system_state === 'FAULT' ? 'fault' : '');
        }

        async function setAutoMode(enabled) {
            try {
                const response = await fetch('/api/control', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({action: 'auto_mode', value: enabled})
                });
                if (response.ok) {
                    refreshStatus();
                }
            } catch (error) {
                console.error('Error setting auto mode:', error);
            }
        }

        async function forceSource(source) {
            try {
                const response = await fetch('/api/control', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({action: 'force_source', value: source})
                });
                if (response.ok) {
                    refreshStatus();
                }
            } catch (error) {
                console.error('Error forcing source:', error);
            }
        }

        // Start auto-refresh on page load
        startAutoRefresh();
        refreshStatus();
    </script>
</body>
</html>
  )";

  server.send(200, "text/html", html);
}

void handleStatusAPI() {
  DynamicJsonDocument doc(1024);

  doc["nepa_voltage"] = volt_nepa;
  doc["nepa_valid"] = status.nepa_valid;
  doc["nepa_energy"] = kwh_nepa;

  doc["gen_voltage"] = volt_gen;
  doc["gen_valid"] = status.gen_valid;
  doc["gen_energy"] = kwh_gen;

  doc["load_voltage"] = volt_load;
  doc["load_current"] = amp_load;
  doc["load_power"] = watt_load;

  doc["active_source"] = (active_source == SOURCE_NEPA) ? "NEPA" :
                         (active_source == SOURCE_GENERATOR) ? "GENERATOR" : "NONE";

  doc["system_state"] = (current_state == STATE_NEPA_ACTIVE) ? "NEPA_ACTIVE" :
                        (current_state == STATE_GEN_ACTIVE) ? "GEN_ACTIVE" :
                        (current_state == STATE_FAULT) ? "FAULT" : "INIT";

  doc["total_energy"] = kwh_total;
  doc["auto_mode"] = auto_mode;
  doc["switch_count"] = status.switch_count;

  String response;
  serializeJson(doc, response);

  server.send(200, "application/json", response);
}

void handleControlAPI() {
  if (server.hasArg("plain")) {
    DynamicJsonDocument doc(256);
    deserializeJson(doc, server.arg("plain"));

    String action = doc["action"];
    String value = doc["value"];

    if (action == "auto_mode") {
      auto_mode = (value == "true");
      settings_modified = true;
      server.send(200, "application/json", "{\"status\":\"success\"}");
    } else if (action == "force_source") {
      if (value == "nepa" && status.nepa_valid) {
        current_state = STATE_NEPA_ACTIVE;
        active_source = SOURCE_NEPA;
        status.state_change_time = millis();
        server.send(200, "application/json", "{\"status\":\"success\"}");
      } else if (value == "gen" && status.gen_valid) {
        current_state = STATE_GEN_ACTIVE;
        active_source = SOURCE_GENERATOR;
        status.state_change_time = millis();
        server.send(200, "application/json", "{\"status\":\"success\"}");
      } else {
        server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid source or source not available\"}");
      }
    } else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid action\"}");
    }
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"No data received\"}");
  }
}

void handleSettingsAPI() {
  DynamicJsonDocument doc(512);

  doc["min_voltage"] = MIN_VOLTAGE;
  doc["max_voltage"] = MAX_VOLTAGE;
  doc["switch_delay"] = SWITCH_DELAY / 1000;
  doc["return_delay"] = SOURCE_RETURN_DELAY / 1000;
  doc["auto_mode"] = auto_mode;

  String response;
  serializeJson(doc, response);

  server.send(200, "application/json", response);
}

void handleSettingsUpdate() {
  if (server.hasArg("plain")) {
    DynamicJsonDocument doc(512);
    deserializeJson(doc, server.arg("plain"));

    if (doc.containsKey("min_voltage")) MIN_VOLTAGE = doc["min_voltage"];
    if (doc.containsKey("max_voltage")) MAX_VOLTAGE = doc["max_voltage"];
    if (doc.containsKey("switch_delay")) SWITCH_DELAY = doc["switch_delay"] * 1000;
    if (doc.containsKey("return_delay")) SOURCE_RETURN_DELAY = doc["return_delay"] * 1000;
    if (doc.containsKey("auto_mode")) auto_mode = doc["auto_mode"];

    settings_modified = true;
    server.send(200, "application/json", "{\"status\":\"success\"}");
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"No data received\"}");
  }
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// EEPROM Functions
void loadWifiCredentials() {
  Serial.println("Loading WiFi credentials from EEPROM...");
  for (int i = 0; i < SSID_MAX_LEN; i++) {
    ssid[i] = EEPROM.read(EEPROM_WIFI_SSID_ADDR + i);
    if (ssid[i] == '\0') {
      break;
    }
  }
  ssid[SSID_MAX_LEN - 1] = '\0'; // Ensure null termination

  for (int i = 0; i < PASS_MAX_LEN; i++) {
    password[i] = EEPROM.read(EEPROM_WIFI_PASS_ADDR + i);
    if (password[i] == '\0') {
      break;
    }
  }
  password[PASS_MAX_LEN - 1] = '\0'; // Ensure null termination

  Serial.print("SSID loaded: ");
  Serial.println(ssid);
}

void saveWifiCredentials(const char* new_ssid, const char* new_pass) {
  Serial.println("Saving new WiFi credentials to EEPROM...");

  // Save SSID
  strncpy(ssid, new_ssid, SSID_MAX_LEN);
  ssid[SSID_MAX_LEN - 1] = '\0';
  for (int i = 0; i < SSID_MAX_LEN; i++) {
    EEPROM.write(EEPROM_WIFI_SSID_ADDR + i, ssid[i]);
    if (ssid[i] == '\0') break;
  }

  // Save Password
  strncpy(password, new_pass, PASS_MAX_LEN);
  password[PASS_MAX_LEN - 1] = '\0';
  for (int i = 0; i < PASS_MAX_LEN; i++) {
    EEPROM.write(EEPROM_WIFI_PASS_ADDR + i, password[i]);
    if (password[i] == '\0') break;
  }

  if (EEPROM.commit()) {
    Serial.println("WiFi credentials committed to EEPROM.");
  } else {
    Serial.println("EEPROM commit failed.");
  }
}

void saveSystemData() {
  EEPROM.put(EEPROM_RUNTIME_ADDR, status.runtime_hours);
  EEPROM.put(EEPROM_KWH_NEPA_ADDR, kwh_nepa);
  EEPROM.put(EEPROM_KWH_GEN_ADDR, kwh_gen);
  EEPROM.put(EEPROM_CONFIG_ADDR, status.switch_count);

  EEPROM.commit();
  Serial.println("System data saved to EEPROM");
  settings_modified = false;
}

void saveCalibrationData() {
  EEPROM.put(EEPROM_CAL_NEPA_V_ADDR, nepa_voltage_offset);
  EEPROM.put(EEPROM_CAL_GEN_V_ADDR, gen_voltage_offset);
  EEPROM.put(EEPROM_CAL_CURRENT_ADDR, current_offset);

  // Save settings
  struct Settings {
    int min_voltage;
    int max_voltage;
    unsigned long switch_delay;
    unsigned long return_delay;
    bool auto_mode_setting;
  } settings;

  settings.min_voltage = MIN_VOLTAGE;
  settings.max_voltage = MAX_VOLTAGE;
  settings.switch_delay = SWITCH_DELAY;
  settings.return_delay = SOURCE_RETURN_DELAY;
  settings.auto_mode_setting = auto_mode;

  EEPROM.put(EEPROM_SETTINGS_ADDR, settings);
  EEPROM.commit();

  Serial.println("Calibration and settings saved to EEPROM");
  settings_modified = false;
}

void loadSystemData() {
  // Load runtime hours and energy data
  EEPROM.get(EEPROM_RUNTIME_ADDR, status.runtime_hours);
  EEPROM.get(EEPROM_KWH_NEPA_ADDR, kwh_nepa);
  EEPROM.get(EEPROM_KWH_GEN_ADDR, kwh_gen);
  EEPROM.get(EEPROM_CONFIG_ADDR, status.switch_count);

  // Load calibration data
  EEPROM.get(EEPROM_CAL_NEPA_V_ADDR, nepa_voltage_offset);
  EEPROM.get(EEPROM_CAL_GEN_V_ADDR, gen_voltage_offset);
  EEPROM.get(EEPROM_CAL_CURRENT_ADDR, current_offset);

  // Load settings
  struct Settings {
    int min_voltage;
    int max_voltage;
    unsigned long switch_delay;
    unsigned long return_delay;
    bool auto_mode_setting;
  } settings;

  EEPROM.get(EEPROM_SETTINGS_ADDR, settings);

  // Validate and apply loaded data
  if (status.runtime_hours > 100000 || status.runtime_hours < 0) status.runtime_hours = 0;
  if (kwh_nepa < 0 || kwh_nepa > 1e7) kwh_nepa = 0;
  if (kwh_gen < 0 || kwh_gen > 1e7) kwh_gen = 0;
  if (status.switch_count > 65535 || status.switch_count < 0) status.switch_count = 0;

  // Validate calibration offsets
  if (nepa_voltage_offset < -100 || nepa_voltage_offset > 100) nepa_voltage_offset = 0;
  if (gen_voltage_offset < -100 || gen_voltage_offset > 100) gen_voltage_offset = 0;
  if (current_offset < -100 || current_offset > 100) current_offset = 0;

  // Validate and apply settings
  if (settings.min_voltage >= 100 && settings.min_voltage <= 200) MIN_VOLTAGE = settings.min_voltage;
  if (settings.max_voltage >= 200 && settings.max_voltage <= 300) MAX_VOLTAGE = settings.max_voltage;
  if (settings.switch_delay >= 1000 && settings.switch_delay <= 30000) SWITCH_DELAY = settings.switch_delay;
  if (settings.return_delay >= 30000 && settings.return_delay <= 300000) SOURCE_RETURN_DELAY = settings.return_delay;
  auto_mode = settings.auto_mode_setting;

  kwh_total = kwh_nepa + kwh_gen;

  Serial.println("System data loaded from EEPROM");
}

// Utility Functions
void logFault(int fault_code) {
  status.fault_code = fault_code;

  // Save fault to EEPROM
  int fault_count = EEPROM.read(EEPROM_FAULT_LOG_ADDR);
  fault_count++;
  if (fault_count > 255) fault_count = 255;
  EEPROM.write(EEPROM_FAULT_LOG_ADDR, fault_count);
  EEPROM.commit();

  settings_modified = true;

  Serial.printf("FAULT LOGGED: %d\n", fault_code);
  switch (fault_code) {
    case 1: Serial.println("NEPA Failure"); break;
    case 2: Serial.println("Generator Failure"); break;
    case 3: Serial.println("All Sources Failed"); break;
    default: Serial.println("Unknown Fault"); break;
  }
}

// Runtime update function
void updateRuntime() {
  static unsigned long last_runtime_update = 0;
  unsigned long current_time = millis();

  if (current_time - last_runtime_update >= 3600000) { // Every hour
    status.runtime_hours++;
    settings_modified = true;
    last_runtime_update = current_time;
  }
}
