#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <math.h>

// Debug configuration - set to false to save memory
#define DEBUG_MODE false

// Debug macros - only compile debug output when DEBUG_MODE is true
#if DEBUG_MODE
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(x, y) Serial.print(x, y)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(x, y)
#endif

// Define enums early so Arduino's auto-prototypes see them
enum PowerSource {
  SOURCE_NONE,
  SOURCE_NEPA,
  SOURCE_GENERATOR,
  SOURCE_INVERTER
};

// Forward declaration to ensure type is known before any auto-prototypes
PowerSource selectBestSource();

// Ensure enums are defined before Arduino adds auto-prototypes
// moved to top to avoid prototype order issues

// Initialize LCD with I2C address 0x27, 16 columns and 2 rows
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ATS Control Pins
#define NEPA_RELAY_PIN      7    // Relay for NEPA connection
#define GEN_RELAY_PIN       8    // Relay for Generator connection
#define INVERTER_RELAY_PIN  9    // Relay for Solar Inverter connection
#define BUZZER_PIN         11    // Alarm buzzer
#define STATUS_LED_PIN     13    // Status indicator LED

// Analog pin mapping (adjust here if your wiring differs)
#define VOLT_PIN_NEPA      A0
#define VOLT_PIN_GEN       A1
#define VOLT_PIN_INV       A2
#define CURR_PIN_LOAD      A3

// Menu Control Buttons
#define BTN_UP             2     // Menu UP button
#define BTN_ENTER          3     // Menu ENTER button
#define BTN_DOWN           4     // Menu DOWN button

// Manual mode selection buttons (pull to ground)
#define MANUAL_NEPA_PIN    5     // Manual NEPA selection button
#define MANUAL_GEN_PIN     6     // Manual Generator selection button
#define MANUAL_INV_PIN     10    // Manual Inverter selection button (Moved to 10 to resolve conflict)

// Additional control pins
#define RESET_ALARM_PIN    12    // Reset alarm button (Moved to 12 to resolve conflict)

// AC Detection Constants
#define AC_DETECTION_TIMEOUT  100    // 100ms timeout for AC detection
#define SAMPLES_PER_READING   301    // Match template sampling density
#define SAMPLE_DELAY_MS       1      // Match template 1ms delay per sample
#define MIN_AC_THRESHOLD      5      // Higher threshold to avoid floating noise
#define AC_MIN_SPAN           10     // Minimum ADC span across samples to qualify as AC

// Voltage thresholds (configurable via menu) - optimized types
uint16_t MIN_VOLTAGE = 100;           // Minimum acceptable voltage
uint16_t MAX_VOLTAGE = 290;           // Maximum acceptable voltage
uint16_t VOLTAGE_HYSTERESIS = 10;     // Voltage switching hysteresis

// Timing parameters (configurable via menu) - optimized for fast switching
uint16_t SWITCH_DELAY = 100;          // 100ms delay before switching (fast)
uint16_t SOURCE_RETURN_DELAY = 2000;  // 2 second delay before switching back

// Calibration variables - optimized types
int8_t nepa_voltage_offset = 0;       // NEPA voltage calibration offset
int8_t gen_voltage_offset = 0;        // Generator voltage calibration offset
int8_t inv_voltage_offset = 0;        // Inverter voltage calibration offset
int8_t current_offset = 0;            // Current sensor calibration offset
float voltage_multiplier = 0.482;     // Voltage conversion factor
float current_multiplier = 0.039;     // Current conversion factor

// AC reading variables - optimized types
uint16_t sensorV_nepa = 0, sensorV_gen = 0, sensorV_inv = 0;
uint16_t sensorI_load = 0;
float amp_load = 0.0;
uint16_t volt_nepa = 0, volt_gen = 0, volt_inv = 0, volt_load = 0;
uint16_t watt_nepa = 0, watt_gen = 0, watt_inv = 0, watt_load = 0;
uint16_t kw_nepa = 0, kw_gen = 0, kw_inv = 0, kw_total = 0;
float kwh_nepa = 0, kwh_gen = 0, kwh_inv = 0, kwh_total = 0;
uint8_t tm = 0, tm1 = 0;

// Voltage filtering for spike rejection
uint16_t volt_nepa_filtered = 0, volt_gen_filtered = 0, volt_inv_filtered = 0;
uint16_t volt_nepa_history[5] = {0}, volt_gen_history[5] = {0}, volt_inv_history[5] = {0};
uint8_t volt_history_index = 0;

// Menu system variables - optimized types
enum MenuState {
  MENU_MAIN,
  MENU_MONITOR,
  MENU_SETTINGS,
  MENU_CALIBRATION,
  MENU_MAINTENANCE,
  MENU_INFO,
  MENU_CAL_VOLTAGE,
  MENU_CAL_CURRENT,
  MENU_CAL_TIMING,
  MENU_CAL_THRESHOLDS,
  MENU_RESET_DATA,
  MENU_FACTORY_RESET
};

// Manual mode selection variables
bool manual_mode_active = false;
PowerSource manual_selected_source = SOURCE_NONE;
uint32_t manual_selection_time = 0;

MenuState current_menu = MENU_MAIN;
uint8_t menu_index = 0;
uint8_t submenu_index = 0;
bool in_submenu = false;
bool menu_active = false;
uint32_t last_button_press = 0;
bool button_up_pressed = false;
bool button_down_pressed = false;
bool button_enter_pressed = false;

// Calibration state variables - optimized types
bool calibrating_voltage = false;
int8_t current_cal_value = 0;
uint8_t current_cal_source = 0; // 0=NEPA, 1=GEN, 2=INV, 3=Current

// Main menu items stored in PROGMEM to save SRAM
const char menu_main_0[] PROGMEM = "1.Monitor";
const char menu_main_1[] PROGMEM = "2.Settings";
const char menu_main_2[] PROGMEM = "3.Calibration";
const char menu_main_3[] PROGMEM = "4.Maintenance";
const char menu_main_4[] PROGMEM = "5.System Info";
const char menu_main_5[] PROGMEM = "6.Exit Menu";
const char* const main_menu[] PROGMEM = {
  menu_main_0,
  menu_main_1,
  menu_main_2,
  menu_main_3,
  menu_main_4,
  menu_main_5
};
const uint8_t main_menu_size = 6;

// Monitor submenu stored in PROGMEM
const char menu_monitor_0[] PROGMEM = "NEPA Status";
const char menu_monitor_1[] PROGMEM = "Generator";
const char menu_monitor_2[] PROGMEM = "Inverter";
const char menu_monitor_3[] PROGMEM = "Load Power";
const char menu_monitor_4[] PROGMEM = "Energy Total";
const char menu_monitor_5[] PROGMEM = "Back";
const char* const monitor_menu[] PROGMEM = {
  menu_monitor_0,
  menu_monitor_1,
  menu_monitor_2,
  menu_monitor_3,
  menu_monitor_4,
  menu_monitor_5
};
const uint8_t monitor_menu_size = 6;

// Settings submenu stored in PROGMEM
const char menu_settings_0[] PROGMEM = "Min Voltage";
const char menu_settings_1[] PROGMEM = "Max Voltage";
const char menu_settings_2[] PROGMEM = "Switch Delay";
const char menu_settings_3[] PROGMEM = "Return Delay";
const char menu_settings_4[] PROGMEM = "Auto Mode";
const char menu_settings_5[] PROGMEM = "Back";
const char* const settings_menu[] PROGMEM = {
  menu_settings_0,
  menu_settings_1,
  menu_settings_2,
  menu_settings_3,
  menu_settings_4,
  menu_settings_5
};
const uint8_t settings_menu_size = 6;

// Calibration submenu stored in PROGMEM
const char menu_cal_0[] PROGMEM = "NEPA Voltage";
const char menu_cal_1[] PROGMEM = "Gen Voltage";
const char menu_cal_2[] PROGMEM = "Inv Voltage";
const char menu_cal_3[] PROGMEM = "Current Cal";
const char menu_cal_4[] PROGMEM = "Save & Exit";
const char menu_cal_5[] PROGMEM = "Back";
const char* const calibration_menu[] PROGMEM = {
  menu_cal_0,
  menu_cal_1,
  menu_cal_2,
  menu_cal_3,
  menu_cal_4,
  menu_cal_5
};
const uint8_t calibration_menu_size = 6;

// Helper to print strings stored in PROGMEM menu tables
void lcdPrintMenuItem(const char* const* table, uint8_t index) {
  const char* ptr = (const char*)pgm_read_word(&table[index]);
  lcd.print((const __FlashStringHelper*)ptr);
}

// Sanitize floats loaded from EEPROM (guard against NaN/Inf/garbage)
static inline float sanitizeFloat(float value) {
  if (isnan(value) || isinf(value) || value < 0.0f || value > 1.0e7f) {
    return 0.0f;
  }
  return value;
}

// Voltage filtering function to reject spikes and noise
uint16_t filterVoltage(uint16_t new_voltage, uint16_t* history, uint8_t& history_index) {
  // Add new reading to history
  history[history_index] = new_voltage;
  history_index = (history_index + 1) % 5;

  // Calculate median (simple approach - sort and take middle)
  uint16_t temp[5];
  memcpy(temp, history, sizeof(temp));

  // Simple bubble sort for small array
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4 - i; j++) {
      if (temp[j] > temp[j + 1]) {
        uint16_t swap = temp[j];
        temp[j] = temp[j + 1];
        temp[j + 1] = swap;
      }
    }
  }

  // Return median value (middle of sorted array)
  return temp[2];
}

// System states - simplified for fast switching
enum SystemState {
  STATE_INIT,
  STATE_NEPA_ACTIVE,
  STATE_GEN_ACTIVE,
  STATE_INV_ACTIVE,
  STATE_SWITCHING,
  STATE_FAULT,
  STATE_MANUAL
};

// PowerSource enum is defined at the top of the file

// System variables
SystemState current_state = STATE_INIT;
PowerSource active_source = SOURCE_NONE;
bool auto_mode = true;
bool settings_modified = false; // Flag to track when settings need to be saved to EEPROM

// System status - optimized types
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
  bool inv_valid;
  bool load_detected;
  uint16_t switch_count;
  // Relay state tracking to prevent flickering
  bool nepa_relay_on;
  bool gen_relay_on;
  bool inv_relay_on;
} status;

// Timing variables - optimized types
uint32_t last_reading = 0;
uint32_t last_display_update = 0;
uint8_t display_screen = 0;

// EEPROM addresses
#define EEPROM_RUNTIME_ADDR      0
#define EEPROM_FAULT_LOG_ADDR    4
#define EEPROM_KWH_NEPA_ADDR     8
#define EEPROM_KWH_GEN_ADDR     12
#define EEPROM_KWH_INV_ADDR     16
#define EEPROM_CONFIG_ADDR      20
#define EEPROM_CAL_NEPA_V_ADDR  24
#define EEPROM_CAL_GEN_V_ADDR   28
#define EEPROM_CAL_INV_V_ADDR   32
#define EEPROM_CAL_CURRENT_ADDR 36
#define EEPROM_SETTINGS_ADDR    40

// Timer interrupt for energy calculation - optimized
ISR(TIMER1_COMPA_vect) {
  TCNT1 = 0;  // Reset timer for next interrupt

  tm++;

  if (tm >= 3125) {
    tm = 0;
    tm1++;

    // Update energy consumption based on active source
    if (watt_load > 0) {
      switch (active_source) {
        case SOURCE_NEPA:
          kwh_nepa += (watt_load * 0.001f);
          settings_modified = true;
          break;
        case SOURCE_GENERATOR:
          kwh_gen += (watt_load * 0.001f);
          settings_modified = true;
          break;
        case SOURCE_INVERTER:
          kwh_inv += (watt_load * 0.001f);
          settings_modified = true;
          break;
        default:
          break;
      }
    }

    kwh_total = kwh_nepa + kwh_gen + kwh_inv;
  }

  if (tm1 >= 36) {
    tm1 = 0;
    switch (active_source) {
      case SOURCE_NEPA:
        kw_nepa = (uint16_t)(kwh_nepa / 36.0f);
        break;
      case SOURCE_GENERATOR:
        kw_gen = (uint16_t)(kwh_gen / 36.0f);
        break;
      case SOURCE_INVERTER:
        kw_inv = (uint16_t)(kwh_inv / 36.0f);
        break;
      default:
        break;
    }
    kw_total = (uint16_t)(kwh_total / 36.0f);
  }

  // Update runtime every hour
  static uint16_t timer_counter = 0;
  timer_counter++;
  if (timer_counter >= 3600) {
    timer_counter = 0;
    status.runtime_hours++;
    settings_modified = true;
  }
}

// Test function declaration (implementation is below)
void testRelays();

// System health monitoring function declaration
void checkSystemHealth();

// Failsafe mode function declaration
void enableFailsafeMode();

// Menu validation function declaration
void validateMenuState();

// Settings display function declaration
void displaySettingsValue(uint8_t setting_index);

// Simple settings preview function declaration
void showSettingPreview(uint8_t setting_index);

// Settings help function declaration
void showSettingsHelp();

// Menu status display function declaration
void showMenuStatus();

// Helper function to display menu header and timeout
void displayMenuHeaderAndTimeout();

void setup() {
  // Initialize watchdog timer
  wdt_enable(WDTO_8S);

  cli();  // Stop interrupts during setup

  // Reset timer control registers
  TCCR1A = 0;
  TCCR1B = 0;

  // Set prescaler to 256
  TCCR1B |= (1 << CS12);

  // Enable compare match mode on register A
  TIMSK1 |= (1 << OCIE1A);

  // Set compare register A value (corrected)
  OCR1A = 62;  // For 1ms interrupt at 16MHz with 256 prescaler

  sei();

  // Initialize pins
  pinMode(NEPA_RELAY_PIN, OUTPUT);
  pinMode(GEN_RELAY_PIN, OUTPUT);
  pinMode(INVERTER_RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);

  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_ENTER, INPUT_PULLUP);
  pinMode(RESET_ALARM_PIN, INPUT_PULLUP);

  // Set up manual source selection pins
  pinMode(MANUAL_NEPA_PIN, INPUT_PULLUP);
  pinMode(MANUAL_GEN_PIN, INPUT_PULLUP);
  pinMode(MANUAL_INV_PIN, INPUT_PULLUP);

  // Initialize outputs to safe state
  digitalWrite(NEPA_RELAY_PIN, LOW);
  digitalWrite(GEN_RELAY_PIN, LOW);
  digitalWrite(INVERTER_RELAY_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // Initialize relay state tracking
  status.nepa_relay_on = false;
  status.gen_relay_on = false;
  status.inv_relay_on = false;

  Serial.begin(9600);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Load saved data
  loadSystemData();

  // Startup sequence
  lcd.setCursor(0, 0);
  lcd.print(F("3-Source ATS v3"));
  lcd.setCursor(0, 1);
  lcd.print(F("Initializing..."));
  delay(1000);
  wdt_reset(); // Reset watchdog
  delay(1000);

  // Reset watchdog

  DEBUG_PRINTLN(F("=== 3-Source ATS System Started ==="));
  DEBUG_PRINTLN(F("Firmware Version: 3.0 - Optimized"));
  DEBUG_PRINTLN(F("Press ENTER button to access menu"));

  // Test relays to verify hardware
  DEBUG_PRINTLN(F("Starting relay test sequence"));
  testRelays();

  current_state = STATE_INIT;
  status.state_change_time = millis();

  // Reset watchdog
  wdt_reset();
}

// Serial command handler for manual control
// Available commands:
// 1-6: Manual relay control (NEPA, GEN, INV on/off)
// t: Test all relays
// s: Show relay status
// f: Force source activation
// r: Reset to INIT state
// i: Immediate inverter activation
// d: Debug system state
// a: Activate failsafe mode
// m: Test menu system
// h: Show settings help
// u: Show menu status
// x: Test settings menu
// y: Force settings editing mode
// Note: Menu auto-exits after 30 seconds of inactivity
void checkSerialCommands() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    switch(cmd) {
      case '1': // NEPA relay on
        digitalWrite(NEPA_RELAY_PIN, HIGH);
        status.nepa_relay_on = true;
        DEBUG_PRINTLN(F("MANUAL: NEPA relay ON"));
        break;
      case '2': // NEPA relay off
        digitalWrite(NEPA_RELAY_PIN, LOW);
        status.nepa_relay_on = false;
        DEBUG_PRINTLN(F("MANUAL: NEPA relay OFF"));
        break;
      case '3': // GEN relay on
        digitalWrite(GEN_RELAY_PIN, HIGH);
        status.gen_relay_on = true;
        DEBUG_PRINTLN(F("MANUAL: GEN relay ON"));
        break;
      case '4': // GEN relay off
        digitalWrite(GEN_RELAY_PIN, LOW);
        status.gen_relay_on = false;
        DEBUG_PRINTLN(F("MANUAL: GEN relay OFF"));
        break;
      case '5': // INV relay on
        digitalWrite(INVERTER_RELAY_PIN, HIGH);
        status.inv_relay_on = true;
        DEBUG_PRINTLN(F("MANUAL: INV relay ON"));
        break;
      case '6': // INV relay off
        digitalWrite(INVERTER_RELAY_PIN, LOW);
        status.inv_relay_on = false;
        DEBUG_PRINTLN(F("MANUAL: INV relay OFF"));
        break;
      case 't': // Test all relays
        testRelays();
        break;
      case 's': // Show status
        DEBUG_PRINTLN(F("=== RELAY STATUS ==="));
        DEBUG_PRINT(F("NEPA relay: "));
        DEBUG_PRINTLN(digitalRead(NEPA_RELAY_PIN) ? F("ON") : F("OFF"));
        DEBUG_PRINT(F("GEN relay: "));
        DEBUG_PRINTLN(digitalRead(GEN_RELAY_PIN) ? F("ON") : F("OFF"));
        DEBUG_PRINT(F("INV relay: "));
        DEBUG_PRINTLN(digitalRead(INVERTER_RELAY_PIN) ? F("ON") : F("OFF"));
        break;
      case 'f': // Force source activation - NEW COMMAND
        forceSourceActivation();
        break;
      case 'r': // Reset to INIT state - NEW COMMAND
        DEBUG_PRINTLN(F("MANUAL: Resetting to INIT state"));
        current_state = STATE_INIT;
        active_source = SOURCE_NONE;
        status.state_change_time = millis();
        DEBUG_PRINTLN(F("System reset to INITIALIZING"));
        break;
      case 'i': // Immediate inverter activation - NEW COMMAND
        DEBUG_PRINTLN(F("MANUAL: Immediately activating INVERTER"));
        current_state = STATE_INV_ACTIVE;
        active_source = SOURCE_INVERTER;
        status.state_change_time = millis();
        DEBUG_PRINTLN(F("FORCED: Inverter activated"));
        break;
      case 'd': // Debug current state - NEW COMMAND
        debugSystemState();
        break;
      case 'a': // Activate failsafe mode - NEW COMMAND
        enableFailsafeMode();
        break;
      case 'm': // Test menu system - NEW COMMAND
        DEBUG_PRINTLN(F("MANUAL: Testing menu system"));
        menu_active = true;
        current_menu = MENU_MAIN;
        menu_index = 0;
        in_submenu = false;
        submenu_index = 0;
        displayMenu();
        break;
      case 'h': // Show settings help - NEW COMMAND
        DEBUG_PRINTLN(F("MANUAL: Showing settings help"));
        showSettingsHelp();
        break;
      case 'u': // Show menu status - NEW COMMAND
        DEBUG_PRINTLN(F("MANUAL: Showing menu status"));
        showMenuStatus();
        break;
      case 'x': // Test settings menu - NEW COMMAND
        DEBUG_PRINTLN(F("MANUAL: Testing settings menu"));
        menu_active = true;
        current_menu = MENU_SETTINGS;
        menu_index = 1; // Settings is index 1
        in_submenu = true;
        submenu_index = 0; // Start with Min Voltage
        displayMenu();
        break;
      case 'y': // Force settings editing mode - NEW COMMAND
        DEBUG_PRINTLN(F("MANUAL: Forcing settings editing mode"));
        menu_active = true;
        current_menu = MENU_SETTINGS;
        menu_index = 1; // Settings is index 1
        in_submenu = true;
        submenu_index = 0; // Start with Min Voltage
        // Force editing mode
        static bool force_editing = true;
        if (force_editing) {
          // This will be handled in the settings menu
          DEBUG_PRINTLN(F("MANUAL: Ready to test editing mode"));
        }
        displayMenu();
        break;
    }
  }
}

// Implementation of selectBestSource function
PowerSource selectBestSource() {
  // Debug output for source selection
  DEBUG_PRINTLN(F("DEBUG - === SOURCE SELECTION START ==="));
  DEBUG_PRINT(F("NEPA: Valid="));
  DEBUG_PRINT(status.nepa_valid ? F("YES") : F("NO"));
  DEBUG_PRINT(F(", Voltage="));
  DEBUG_PRINTF(volt_nepa, DEC);
  DEBUG_PRINT(F("V"));

  DEBUG_PRINT(F(" | INV: Valid="));
  DEBUG_PRINT(status.inv_valid ? F("YES") : F("NO"));
  DEBUG_PRINT(F(", Voltage="));
  DEBUG_PRINTF(volt_inv, DEC);
  DEBUG_PRINT(F("V"));

  DEBUG_PRINT(F(" | GEN: Valid="));
  DEBUG_PRINT(status.gen_valid ? F("YES") : F("NO"));
  DEBUG_PRINT(F(", Voltage="));
  DEBUG_PRINTF(volt_gen, DEC);
  DEBUG_PRINTLN(F("V"));

  // During initialization, use more lenient criteria
  bool is_initializing = (current_state == STATE_INIT);
  int threshold = is_initializing ? MIN_VOLTAGE : (MIN_VOLTAGE + VOLTAGE_HYSTERESIS);

  DEBUG_PRINT(F("DEBUG - Using threshold: "));
  DEBUG_PRINTF(threshold, DEC);
  DEBUG_PRINT(F("V (initializing: "));
  DEBUG_PRINT(is_initializing ? F("YES") : F("NO"));
  DEBUG_PRINTLN(F(")"));

  PowerSource selected = SOURCE_NONE;

  // FAST SWITCHING: Priority-based source selection: NEPA > Inverter > Generator
  // Inverter is prioritized over Generator for instant switching
  if (status.nepa_valid && volt_nepa >= threshold) {
    DEBUG_PRINTLN(F("DEBUG - Selected: NEPA (meets threshold)"));
    selected = SOURCE_NEPA;
  } else if (status.inv_valid && volt_inv >= threshold) {
    DEBUG_PRINTLN(F("DEBUG - Selected: INVERTER (meets threshold) - FAST SWITCH"));
    selected = SOURCE_INVERTER;
  } else if (status.gen_valid && volt_gen >= threshold) {
    DEBUG_PRINTLN(F("DEBUG - Selected: GENERATOR (meets threshold) - BACKUP"));
    selected = SOURCE_GENERATOR;
  } else {
    DEBUG_PRINTLN(F("DEBUG - No source meets threshold, checking basic validity"));

    // If no source meets the threshold criteria, check if any source is valid at all
    // This helps during initialization when readings might be unstable
    if (is_initializing) {
      if (status.nepa_valid && volt_nepa > 100) {  // Any reasonable voltage
        DEBUG_PRINTLN(F("DEBUG - Selected: NEPA (basic validity)"));
        selected = SOURCE_NEPA;
      } else if (status.inv_valid && volt_inv > 100) {
        DEBUG_PRINTLN(F("DEBUG - Selected: INVERTER (basic validity)"));
        selected = SOURCE_INVERTER;
      } else if (status.gen_valid && volt_gen > 100) {
        DEBUG_PRINTLN(F("DEBUG - Selected: GENERATOR (basic validity)"));
        selected = SOURCE_GENERATOR;
      } else {
        DEBUG_PRINTLN(F("DEBUG - No source meets basic validity"));
      }
    } else {
      DEBUG_PRINTLN(F("DEBUG - Not initializing, strict criteria only"));
    }
  }

  DEBUG_PRINT(F("DEBUG - === SOURCE SELECTION RESULT: "));
  switch(selected) {
    case SOURCE_NONE: DEBUG_PRINTLN(F("NONE ===")); break;
    case SOURCE_NEPA: DEBUG_PRINTLN(F("NEPA ===")); break;
    case SOURCE_GENERATOR: DEBUG_PRINTLN(F("GENERATOR ===")); break;
    case SOURCE_INVERTER: DEBUG_PRINTLN(F("INVERTER ===")); break;
    default: DEBUG_PRINTLN(F("UNKNOWN ===")); break;
  }

  return selected;
}

void debugSystemState() {
  DEBUG_PRINTLN(F("=== SYSTEM DEBUG INFO ==="));
  DEBUG_PRINT(F("Current State: "));
  DEBUG_PRINTF(current_state, DEC);
  DEBUG_PRINT(F(" ("));
  switch(current_state) {
    case STATE_INIT: DEBUG_PRINTLN(F("INITIALIZING)")); break;
    case STATE_NEPA_ACTIVE: DEBUG_PRINTLN(F("NEPA_ACTIVE)")); break;
    case STATE_INV_ACTIVE: DEBUG_PRINTLN(F("INV_ACTIVE)")); break;
    case STATE_GEN_ACTIVE: DEBUG_PRINTLN(F("GEN_ACTIVE)")); break;
    case STATE_SWITCHING: DEBUG_PRINTLN(F("SWITCHING)")); break;
    case STATE_FAULT: DEBUG_PRINTLN(F("FAULT)")); break;
    default: DEBUG_PRINTLN(F("UNKNOWN)")); break;
  }

  DEBUG_PRINT(F("Active Source: "));
  switch(active_source) {
    case SOURCE_NONE: DEBUG_PRINTLN(F("NONE")); break;
    case SOURCE_NEPA: DEBUG_PRINTLN(F("NEPA")); break;
    case SOURCE_GENERATOR: DEBUG_PRINTLN(F("GENERATOR")); break;
    case SOURCE_INVERTER: DEBUG_PRINTLN(F("INVERTER")); break;
    default: DEBUG_PRINTLN(F("UNKNOWN")); break;
  }

  DEBUG_PRINT(F("Auto Mode: "));
  DEBUG_PRINTLN(auto_mode ? F("YES") : F("NO"));

  DEBUG_PRINT(F("Time in current state: "));
  DEBUG_PRINTF(millis() - status.state_change_time, DEC);
  DEBUG_PRINTLN(F("ms"));

  DEBUG_PRINT(F("System fault: "));
  DEBUG_PRINTLN(status.system_fault ? F("YES") : F("NO"));

  DEBUG_PRINTLN(F("=== VOLTAGE READINGS ==="));
  DEBUG_PRINT(F("NEPA: "));
  DEBUG_PRINTF(volt_nepa, DEC);
  DEBUG_PRINT(F("V (Valid: "));
  DEBUG_PRINT(status.nepa_valid ? F("YES") : F("NO"));
  DEBUG_PRINTLN(F(")"));

  DEBUG_PRINT(F("Generator: "));
  DEBUG_PRINTF(volt_gen, DEC);
  DEBUG_PRINT(F("V (Valid: "));
  DEBUG_PRINT(status.gen_valid ? F("YES") : F("NO"));
  DEBUG_PRINTLN(F(")"));

  DEBUG_PRINT(F("Inverter: "));
  DEBUG_PRINTF(volt_inv, DEC);
  DEBUG_PRINT(F("V (Valid: "));
  DEBUG_PRINT(status.inv_valid ? F("YES") : F("NO"));
  DEBUG_PRINTLN(F(")"));

  DEBUG_PRINTLN(F("=== RELAY STATUS ==="));
  DEBUG_PRINT(F("NEPA relay: "));
  DEBUG_PRINTLN(digitalRead(NEPA_RELAY_PIN) ? F("ON") : F("OFF"));
  DEBUG_PRINT(F("GEN relay: "));
  DEBUG_PRINTLN(digitalRead(GEN_RELAY_PIN) ? F("ON") : F("OFF"));
  DEBUG_PRINT(F("INV relay: "));
  DEBUG_PRINTLN(digitalRead(INVERTER_RELAY_PIN) ? F("ON") : F("OFF"));

  DEBUG_PRINTLN(F("========================"));
}

void loop() {
  unsigned long current_time = millis();

  // Reset watchdog timer
  wdt_reset();

  // Check for serial commands
  checkSerialCommands();

  // Handle button inputs
  handleButtons();
  handleManualInput();

  // Menu system
  if (menu_active) {
    // Validate menu state before handling
    validateMenuState();

    // Auto-exit menu after 30 seconds of inactivity
    if (current_time - last_button_press > 30000) { // 30 seconds timeout
      DEBUG_PRINTLN(F("MENU: Auto-exit due to inactivity"));
      exitMenu();
      return;
    }

    // Show warning when 25 seconds have passed
    if (current_time - last_button_press > 25000 && current_time - last_button_press <= 25100) {
      // Show warning for 1 second
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Menu will exit"));
      lcd.setCursor(0, 1);
      lcd.print(F("in 5 seconds"));
      delay(1000);
      wdt_reset();
      displayMenu(); // Return to normal menu display
    }

    handleMenu();

    // Menu stays active until manually exited or timeout
    return; // Skip normal operation when in menu
  }

  // Read input switches
  bool reset_alarm = !digitalRead(RESET_ALARM_PIN);

  // Reset alarm if button pressed
  if (reset_alarm && status.alarm_active) {
    status.alarm_active = false;
    digitalWrite(BUZZER_PIN, LOW);
    DEBUG_PRINTLN(F("Alarm Reset"));
  }

  // Read AC power sources with timeout protection
  if (current_time - last_reading >= 1000) { // Read every 1 second instead of continuously
    readACPowerSources();
    last_reading = current_time;
  }

  // CRITICAL FIX: Always run the state machine logic, regardless of auto_mode
  // The auto_mode flag should only affect whether we respond to manual switches
  updateSystemState();
  controlRelays();

  // Add emergency state check to prevent getting stuck
  emergencyStateCheck();

  // Normal display update every 3 seconds when not in menu
  if (current_time - last_display_update >= 3000) {
    updateNormalDisplay();
    last_display_update = current_time;
  }

  // Handle alarms and status indicators
  handleAlarms();

  // Only save system data if settings have been modified
  // or every hour for runtime statistics
  static unsigned long last_save = 0;
  if ((settings_modified && current_time - last_save >= 60000) || // Save modified settings after 1 minute
      (current_time - last_save >= 3600000)) { // Save runtime stats every hour
    saveSystemData();
    last_save = current_time;
  }
}

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
        DEBUG_PRINTLN(F("ENTERING MENU - Button pressed"));
        menu_active = true;
        current_menu = MENU_MAIN;
        menu_index = 0;
        in_submenu = false;
        submenu_index = 0;

        // Display initial menu
        displayMenu();
      }
    }
  }

  last_up = current_up;
  last_down = current_down;
  last_enter = current_enter;
}

void handleManualInput() {
  // Do nothing if in menu or already in manual mode to prevent constant re-triggering
  if (menu_active || !auto_mode) return;

  PowerSource manually_selected = SOURCE_NONE;

  // Check for manual source selection (buttons are active LOW)
  if (digitalRead(MANUAL_NEPA_PIN) == LOW) {
    manually_selected = SOURCE_NEPA;
  } else if (digitalRead(MANUAL_GEN_PIN) == LOW) {
    manually_selected = SOURCE_GENERATOR;
  } else if (digitalRead(MANUAL_INV_PIN) == LOW) {
    manually_selected = SOURCE_INVERTER;
  }

  if (manually_selected != SOURCE_NONE) {
    auto_mode = false; // Switch to manual mode
    settings_modified = true; // Ensure this change can be saved
    active_source = manually_selected;
    current_state = STATE_MANUAL; // A dedicated state for manual mode
    status.state_change_time = millis();

    lcd.clear();
    lcd.print(F("Manual Mode On"));
    lcd.setCursor(0, 1);
    lcd.print(F("Source: "));

    switch(manually_selected) {
      case SOURCE_NEPA:
        DEBUG_PRINTLN(F("MANUAL OVERRIDE: NEPA selected"));
        lcd.print(F("NEPA"));
        break;
      case SOURCE_GENERATOR:
        DEBUG_PRINTLN(F("MANUAL OVERRIDE: GEN selected"));
        lcd.print(F("GEN"));
        break;
      case SOURCE_INVERTER:
        DEBUG_PRINTLN(F("MANUAL OVERRIDE: INV selected"));
        lcd.print(F("INV"));
        break;
      default: break;
    }
    delay(2000);
    wdt_reset();
  }
}

void handleMenu() {
  // Check if we're in calibration mode first
  static bool last_calibrating_state = false;
  if (calibrating_voltage != last_calibrating_state) {
    DEBUG_PRINT(F("CALIBRATION STATE CHANGED: "));
    DEBUG_PRINTLN(calibrating_voltage ? F("ACTIVE") : F("INACTIVE"));
    last_calibrating_state = calibrating_voltage;
  }

  if (calibrating_voltage) {
    // Handle calibration adjustment
    if (button_up_pressed) {
      button_up_pressed = false;
      current_cal_value++;
      if (current_cal_value > 100) current_cal_value = 100;
      DEBUG_PRINT(F("CALIBRATION UP - Value: "));
      DEBUG_PRINTLN(current_cal_value);
      displayMenu(); // Update display immediately
    }

    if (button_down_pressed) {
      button_down_pressed = false;
      current_cal_value--;
      if (current_cal_value < -100) current_cal_value = -100;
      DEBUG_PRINT(F("CALIBRATION DOWN - Value: "));
      DEBUG_PRINTLN(current_cal_value);
      displayMenu(); // Update display immediately
    }

    if (button_enter_pressed) {
      button_enter_pressed = false;
      // Save calibration value
      switch (current_cal_source) {
        case 0: nepa_voltage_offset = current_cal_value; break;
        case 1: gen_voltage_offset = current_cal_value; break;
        case 2: inv_voltage_offset = current_cal_value; break;
        case 3: current_offset = current_cal_value; break;
        default: break;
      }
      calibrating_voltage = false;
      settings_modified = true; // Mark settings as modified

      // Debug output
      DEBUG_PRINT(F("CALIBRATION SAVED - Source: "));
      switch (current_cal_source) {
        case 0: DEBUG_PRINT(F("NEPA")); break;
        case 1: DEBUG_PRINT(F("GEN")); break;
        case 2: DEBUG_PRINT(F("INV")); break;
        case 3: DEBUG_PRINT(F("CURRENT")); break;
        default: DEBUG_PRINT(F("UNKNOWN")); break;
      }
      DEBUG_PRINT(F(", Value: "));
      DEBUG_PRINTLN(current_cal_value);

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Cal Value Saved"));
      lcd.setCursor(0, 1);
      lcd.print(F("Offset:"));
      lcd.print(current_cal_value);
      delay(1000);
      wdt_reset(); // Reset watchdog
      delay(1000);
      displayMenu(); // Update display after saving
    }

    // Add a way to cancel calibration by pressing DOWN for 2 seconds
    static unsigned long down_press_start = 0;
    if (button_down_pressed) {
      if (down_press_start == 0) {
        down_press_start = millis();
      } else if (millis() - down_press_start > 2000) {
        // Cancel calibration
        calibrating_voltage = false;
        down_press_start = 0;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Calibration"));
        lcd.setCursor(0, 1);
        lcd.print(F("Cancelled"));
        delay(1000);
        wdt_reset(); // Reset watchdog
        delay(1000);
        displayMenu(); // Update display after cancelling
      }
    } else {
      down_press_start = 0;
    }

    return; // Exit early, don't process normal menu navigation
  }

  // Normal menu navigation (when not calibrating)
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
    handleMenuSelection();
  }

  // Debug menu state
  DEBUG_PRINT(F("Menu State - Active: "));
  DEBUG_PRINT(menu_active ? F("YES") : F("NO"));
  DEBUG_PRINT(F(", Menu: "));
  DEBUG_PRINTF(current_menu, DEC);
  DEBUG_PRINT(F(", Index: "));
  DEBUG_PRINTF(menu_index, DEC);
  DEBUG_PRINT(F(", Submenu: "));
  DEBUG_PRINT(in_submenu ? F("YES") : F("NO"));
  DEBUG_PRINT(F(", SubIndex: "));
  DEBUG_PRINTLN(submenu_index);
}

void displayMenuHeaderAndTimeout() {
    // Show menu status indicator ("M" for Menu) in the top-right corner
    lcd.setCursor(15, 0);
    lcd.print(F("M"));

    // Calculate and display the inactivity countdown timer in the bottom-right corner
    unsigned long time_left = 30 - ((millis() - last_button_press) / 1000);
    lcd.setCursor(14, 1); // Move cursor to position 14 to handle 1 or 2 digits
    if (time_left > 0 && time_left <= 30) {
        if (time_left < 10) {
            lcd.print(F(" ")); // Add a space to clear the tens digit
        }
        lcd.print(time_left);
    }
}

void displayMenu() {
  lcd.clear();

  // Debug display function call
  DEBUG_PRINT(F("DISPLAY MENU - Submenu: "));
  DEBUG_PRINT(in_submenu ? F("YES") : F("NO"));
  DEBUG_PRINT(F(", Menu: "));
  DEBUG_PRINTF(current_menu, DEC);
  DEBUG_PRINT(F(", Index: "));
  DEBUG_PRINTF(menu_index, DEC);
  DEBUG_PRINT(F(", SubIndex: "));
  DEBUG_PRINTLN(submenu_index);

  if (!in_submenu) {
    // Display main menu
    lcd.setCursor(0, 0);
    lcd.print(F("MAIN MENU"));
    lcd.setCursor(0, 1);
    lcd.print(F(">"));
    lcdPrintMenuItem(main_menu, menu_index);
  } else {
    // Display submenu
    lcd.setCursor(0, 0);
    switch (current_menu) {
      case MENU_MONITOR:
        lcd.print(F("MONITOR"));
        lcd.setCursor(0, 1);
        lcd.print(F(">"));
        lcdPrintMenuItem(monitor_menu, submenu_index);
        break;

      case MENU_SETTINGS:
        lcd.print(F("SETTINGS"));
        lcd.setCursor(0, 1);
        lcd.print(F(">"));
        lcdPrintMenuItem(settings_menu, submenu_index);
        break;

      case MENU_CALIBRATION:
        if (calibrating_voltage) {
          // Show calibration editing interface with live values
          lcd.clear();
          lcd.setCursor(0, 0);
          lcdPrintMenuItem(calibration_menu, current_cal_source);

          // Show live voltage reading with current calibration
          int raw_voltage = 0;
          int live_calibrated = 0;

          switch (current_cal_source) {
            case 0: // NEPA
              raw_voltage = sensorV_nepa * voltage_multiplier;
              live_calibrated = raw_voltage + current_cal_value;
              break;
            case 1: // Generator
              raw_voltage = sensorV_gen * voltage_multiplier;
              live_calibrated = raw_voltage + current_cal_value;
              break;
            case 2: // Inverter
              raw_voltage = sensorV_inv * voltage_multiplier;
              live_calibrated = raw_voltage + current_cal_value;
              break;
            case 3: // Current
              float raw_current = (sensorI_load * current_multiplier) / 3;
              float live_current = raw_current + (current_cal_value * 0.01);
              lcd.setCursor(0, 1);
              lcd.print(F("Raw:"));
              lcd.print(raw_current, 1);
              lcd.print(F("A Cal:"));
              lcd.print(live_current, 1);
              lcd.print(F("A"));

              // Show current offset value
              lcd.setCursor(15, 0);
              lcd.print(current_cal_value);
              break;
            default:
              break;
          }

          if (current_cal_source < 3) { // Voltage calibration
            lcd.setCursor(0, 1);
            lcd.print(F("Raw:"));
            lcd.print(raw_voltage);
            lcd.print(F("V Cal:"));
            lcd.print(live_calibrated);
            lcd.print(F("V"));

            // Show current offset value
            lcd.setCursor(15, 0);
            lcd.print(current_cal_value);
          }
        } else {
          // Show normal calibration menu
          lcd.print(F("CALIBRATION"));
          lcd.setCursor(0, 1);
          lcd.print(F(">"));
          lcdPrintMenuItem(calibration_menu, submenu_index);
        }
        break;

      case MENU_CAL_VOLTAGE:
        displayVoltageCalibration();
        break;

      case MENU_CAL_CURRENT:
        displayCurrentCalibration();
        break;

      case MENU_CAL_TIMING:
        displayTimingSettings();
        break;

      case MENU_CAL_THRESHOLDS:
        displayThresholdSettings();
        break;

      default:
        lcd.print(F("Menu Error"));
        break;
    }
  }

  // If not in the middle of a calibration, show the header and timeout.
  if (!calibrating_voltage) {
      displayMenuHeaderAndTimeout();
  }
}

void handleMenuSelection() {
  DEBUG_PRINT(F("MENU SELECTION - Menu: "));
  DEBUG_PRINTF(menu_index, DEC);
  DEBUG_PRINT(F(", Submenu: "));
  DEBUG_PRINT(in_submenu ? F("YES") : F("NO"));
  DEBUG_PRINT(F(", Current: "));
  DEBUG_PRINTF(current_menu, DEC);
  DEBUG_PRINTLN(F(""));

  if (!in_submenu) {
    // Main menu selection
    switch (menu_index) {
      case 0: // Monitor
        DEBUG_PRINTLN(F("Entering MONITOR submenu"));
        current_menu = MENU_MONITOR;
        in_submenu = true;
        submenu_index = 0;
        break;

      case 1: // Settings
        DEBUG_PRINTLN(F("Entering SETTINGS submenu"));
        current_menu = MENU_SETTINGS;
        in_submenu = true;
        submenu_index = 0;
        break;

      case 2: // Calibration
        DEBUG_PRINTLN(F("Entering CALIBRATION submenu"));
        current_menu = MENU_CALIBRATION;
        in_submenu = true;
        submenu_index = 0;
        break;

      case 3: // Maintenance
        DEBUG_PRINTLN(F("Executing MAINTENANCE"));
        handleMaintenance();
        break;

      case 4: // System Info
        DEBUG_PRINTLN(F("Displaying SYSTEM INFO"));
        displaySystemInfo();
        break;

      case 5: // Exit Menu
        if (button_enter_pressed) {
          button_enter_pressed = false;
          DEBUG_PRINTLN(F("Exiting menu"));

          // Show exit confirmation
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Exit Menu?"));
          lcd.setCursor(0, 1);
          lcd.print(F("ENTER: Yes"));

          // Wait for confirmation
          unsigned long exit_start = millis();
          while (millis() - exit_start < 5000) { // 5 second timeout
            wdt_reset();

            if (button_enter_pressed) {
              button_enter_pressed = false;
        exitMenu();
              return; // Don't display menu after exit
            }

            if (button_up_pressed || button_down_pressed) {
              button_up_pressed = false;
              button_down_pressed = false;
              displayMenu(); // Return to menu
              return;
            }
          }

          // Timeout - return to menu
          displayMenu();
          return;
        }
        break;

      default:
        DEBUG_PRINTLN(F("Unknown menu selection"));
        break;
    }
  } else {
    // Submenu selection
    switch (current_menu) {
      case MENU_MONITOR:
        DEBUG_PRINTLN(F("Handling MONITOR submenu"));
        handleMonitorMenu();
        break;

      case MENU_SETTINGS:
        DEBUG_PRINTLN(F("Handling SETTINGS submenu"));
        handleSettingsMenu();
        break;

      case MENU_CALIBRATION:
        DEBUG_PRINTLN(F("Handling CALIBRATION submenu"));
        handleCalibrationMenu();
        break;

      default:
        DEBUG_PRINTLN(F("Unknown submenu"));
        break;
    }
  }

  // Only display menu if we didn't exit
  if (menu_active) {
  displayMenu();
  }
}

void handleMonitorMenu() {
  // Consume the enter button press to prevent re-triggering
  button_enter_pressed = false;

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
      lcd.print(kw_nepa);
      lcd.print(F("Wh"));
      delay(750);
      wdt_reset(); // Reset watchdog
      delay(750);
      break;

    case 1: // Generator
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("GEN:"));
      lcd.print(volt_gen);
      lcd.print(F("V"));
      lcd.setCursor(0, 1);
      lcd.print(status.gen_running ? F("RUNNING ") : F("STOPPED "));
      lcd.print(kw_gen);
      lcd.print(F("Wh"));
      delay(1500);
      wdt_reset(); // Reset watchdog
      delay(1500);
      break;

    case 2: // Inverter
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("INV:"));
      lcd.print(volt_inv);
      lcd.print(F("V"));
      lcd.setCursor(0, 1);
      lcd.print(status.inv_valid ? F("HEALTHY") : F("FAILED"));
      lcd.print(" ");
      lcd.print(kw_inv);
      lcd.print(F("Wh"));
      delay(1500);
      wdt_reset(); // Reset watchdog
      delay(1500);
      break;

    case 3: // Load Power
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("LOAD:"));
      lcd.print(amp_load, 1);
      lcd.print(F("A"));
      lcd.setCursor(0, 1);
      lcd.print(F("Power:"));
      lcd.print(watt_load);
      lcd.print(F("W"));
      delay(1500);
      wdt_reset(); // Reset watchdog
      delay(1500);
      break;

    case 4: // Energy Total
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Total Energy:"));
      lcd.setCursor(0, 1);
      lcd.print(kw_total);
      lcd.print("Wh");
      delay(1500);
      wdt_reset(); // Reset watchdog
      delay(1500);
      break;

    case 5: // Back
      DEBUG_PRINTLN(F("MONITOR: Back to main menu"));
      in_submenu = false;
      current_menu = MENU_MAIN;
      submenu_index = 0;
      break;

    default:
      break;
  }
}

void handleSettingsMenu() {
  DEBUG_PRINTLN(F("=== SETTINGS MENU ENTERED ==="));

  static bool editing = false;
  static int edit_value = 0;
  static uint8_t last_submenu_index = 255; // Track submenu changes

  // Debug output for settings menu
  DEBUG_PRINT(F("SETTINGS MENU - Submenu: "));
  DEBUG_PRINTF(submenu_index, DEC);
  DEBUG_PRINT(F(", Editing: "));
  DEBUG_PRINTLN(editing ? F("YES") : F("NO"));

  // Reset editing state if submenu changed
  if (last_submenu_index != submenu_index) {
    editing = false;
    last_submenu_index = submenu_index;
    DEBUG_PRINT(F("SETTINGS: Submenu changed to "));
    DEBUG_PRINTLN(submenu_index);
  }

  // Handle editing mode
  if (editing) {
    DEBUG_PRINTLN(F("SETTINGS: In editing mode"));
    // Handle value editing
    if (button_up_pressed) {
      button_up_pressed = false;
      edit_value++;
      // Add bounds checking
      switch (submenu_index) {
        case 0: if (edit_value > 200) edit_value = 200; break; // Min voltage max
        case 1: if (edit_value > 300) edit_value = 300; break; // Max voltage max
        case 2: if (edit_value > 60) edit_value = 60; break;   // Switch delay max
        case 3: if (edit_value > 600) edit_value = 600; break; // Return delay max
      }
      DEBUG_PRINT(F("SETTINGS: Value increased to "));
      DEBUG_PRINTLN(edit_value);

      // Update display immediately
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("EDITING:"));
      lcdPrintMenuItem(settings_menu, submenu_index);
      lcd.setCursor(0, 1);
      lcd.print(F("Value:"));
      lcd.print(edit_value);

      // Show units
      switch (submenu_index) {
        case 0: lcd.print(F("V")); break;
        case 1: lcd.print(F("V")); break;
        case 2: lcd.print(F("s")); break;
        case 3: lcd.print(F("s")); break;
      }
    }

    if (button_down_pressed) {
      button_down_pressed = false;
      edit_value--;
      // Add bounds checking
      switch (submenu_index) {
        case 0: if (edit_value < 100) edit_value = 100; break; // Min voltage min
        case 1: if (edit_value < 200) edit_value = 200; break; // Max voltage min
        case 2: if (edit_value < 1) edit_value = 1; break;     // Switch delay min
        case 3: if (edit_value < 30) edit_value = 30; break;   // Return delay min
      }
      DEBUG_PRINT(F("SETTINGS: Value decreased to "));
      DEBUG_PRINTLN(edit_value);

      // Update display immediately
    lcd.clear();
    lcd.setCursor(0, 0);
      lcd.print(F("EDITING:"));
    lcdPrintMenuItem(settings_menu, submenu_index);
    lcd.setCursor(0, 1);
    lcd.print(F("Value:"));
    lcd.print(edit_value);

      // Show units
      switch (submenu_index) {
        case 0: lcd.print(F("V")); break;
        case 1: lcd.print(F("V")); break;
        case 2: lcd.print(F("s")); break;
        case 3: lcd.print(F("s")); break;
      }
    }

    if (button_enter_pressed) {
      button_enter_pressed = false;
      // Save the edited value
      switch (submenu_index) {
        case 0:
          MIN_VOLTAGE = edit_value;
          DEBUG_PRINT(F("SETTINGS: Min Voltage saved as "));
          DEBUG_PRINTLN(edit_value);
          break;
        case 1:
          MAX_VOLTAGE = edit_value;
          DEBUG_PRINT(F("SETTINGS: Max Voltage saved as "));
          DEBUG_PRINTLN(edit_value);
          break;
        case 2:
          SWITCH_DELAY = edit_value * 1000;
          DEBUG_PRINT(F("SETTINGS: Switch Delay saved as "));
          DEBUG_PRINTLN(edit_value);
          break;
        case 3:
          SOURCE_RETURN_DELAY = edit_value * 1000;
          DEBUG_PRINT(F("SETTINGS: Return Delay saved as "));
          DEBUG_PRINTLN(edit_value);
          break;
        default: break;
      }
      editing = false;
      settings_modified = true; // Mark settings as modified

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Setting Saved!"));
      lcd.setCursor(0, 1);
      lcd.print(F("Value:"));
      lcd.print(edit_value);
      delay(1500);
      wdt_reset(); // Reset watchdog
      delay(1500);

      // Return to settings display
      displayMenu();
      return;
    }

    // Cancel editing with DOWN button long press
    static unsigned long down_press_start = 0;
    if (button_down_pressed) {
      if (down_press_start == 0) {
        down_press_start = millis();
      } else if (millis() - down_press_start > 2000) {
        // Cancel editing
        editing = false;
        down_press_start = 0;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Editing"));
        lcd.setCursor(0, 1);
        lcd.print(F("Cancelled"));
        delay(1000);
        wdt_reset();
        displayMenu(); // Return to menu
        return;
      }
    } else {
      down_press_start = 0;
    }

    return; // Stay in editing mode
  }

  // Not editing - handle menu navigation
  DEBUG_PRINT(F("SETTINGS: Button states - UP: "));
  DEBUG_PRINT(button_up_pressed ? F("YES") : F("NO"));
  DEBUG_PRINT(F(", DOWN: "));
  DEBUG_PRINT(button_down_pressed ? F("YES") : F("NO"));
  DEBUG_PRINT(F(", ENTER: "));
  DEBUG_PRINTLN(button_enter_pressed ? F("YES") : F("NO"));

  // Also show raw button readings
  DEBUG_PRINT(F("SETTINGS: Raw buttons - UP: "));
  DEBUG_PRINT(digitalRead(BTN_UP) ? F("HIGH") : F("LOW"));
  DEBUG_PRINT(F(", DOWN: "));
  DEBUG_PRINT(digitalRead(BTN_DOWN) ? F("HIGH") : F("LOW"));
  DEBUG_PRINT(F(", ENTER: "));
  DEBUG_PRINTLN(digitalRead(BTN_ENTER) ? F("HIGH") : F("LOW"));

  switch (submenu_index) {
    case 0: // Min Voltage
      DEBUG_PRINTLN(F("SETTINGS: Handling Min Voltage option"));
      if (button_enter_pressed) {
        button_enter_pressed = false;
        edit_value = MIN_VOLTAGE;
        editing = true;
        DEBUG_PRINTLN(F("SETTINGS: Editing Min Voltage"));

        // Show editing screen
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("EDITING:"));
        lcdPrintMenuItem(settings_menu, submenu_index);
        lcd.setCursor(0, 1);
        lcd.print(F("Value:"));
        lcd.print(edit_value);
        lcd.print(F("V"));
      } else if (button_up_pressed || button_down_pressed) {
        // Show preview when navigating
        DEBUG_PRINTLN(F("SETTINGS: Showing Min Voltage preview"));
        showSettingPreview(submenu_index);
        displayMenu(); // Return to menu
      }
      break;

      case 1: // Max Voltage
        if (button_enter_pressed) {
          button_enter_pressed = false;
          edit_value = MAX_VOLTAGE;
          editing = true;
          DEBUG_PRINTLN(F("SETTINGS: Editing Max Voltage"));

          // Show editing screen
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("EDITING:"));
          lcdPrintMenuItem(settings_menu, submenu_index);
          lcd.setCursor(0, 1);
          lcd.print(F("Value:"));
          lcd.print(edit_value);
          lcd.print(F("V"));
        } else if (button_up_pressed || button_down_pressed) {
          // Show preview when navigating
          showSettingPreview(submenu_index);
          displayMenu(); // Return to menu
        }
        break;

    case 2: // Switch Delay
      if (button_enter_pressed) {
        button_enter_pressed = false;
        edit_value = SWITCH_DELAY / 1000;
        editing = true;
        DEBUG_PRINTLN(F("SETTINGS: Editing Switch Delay"));

        // Show editing screen
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("EDITING:"));
        lcdPrintMenuItem(settings_menu, submenu_index);
        lcd.setCursor(0, 1);
        lcd.print(F("Value:"));
        lcd.print(edit_value);
        lcd.print(F("s"));
      } else if (button_up_pressed || button_down_pressed) {
        // Show preview when navigating
        showSettingPreview(submenu_index);
        displayMenu(); // Return to menu
      }
      break;

    case 3: // Return Delay
      if (button_enter_pressed) {
        button_enter_pressed = false;
        edit_value = SOURCE_RETURN_DELAY / 1000;
        editing = true;
        DEBUG_PRINTLN(F("SETTINGS: Editing Return Delay"));

        // Show editing screen
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("EDITING:"));
        lcdPrintMenuItem(settings_menu, submenu_index);
        lcd.setCursor(0, 1);
        lcd.print(F("Value:"));
        lcd.print(edit_value);
        lcd.print(F("s"));
      } else if (button_up_pressed || button_down_pressed) {
        // Show preview when navigating
        showSettingPreview(submenu_index);
        displayMenu(); // Return to menu
        return;
      }
      break;

    case 4: // Auto Mode
      if (button_enter_pressed) {
        button_enter_pressed = false;
        auto_mode = !auto_mode;
        settings_modified = true; // Mark settings as modified
        DEBUG_PRINT(F("SETTINGS: Auto Mode changed to "));
        DEBUG_PRINTLN(auto_mode ? F("ENABLED") : F("DISABLED"));

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Auto Mode:"));
        lcd.setCursor(0, 1);
        lcd.print(auto_mode ? F("ENABLED") : F("DISABLED"));
        delay(1000);
        wdt_reset(); // Reset watchdog
        delay(1000);

        // Return to settings display
        displayMenu();
      } else if (button_up_pressed || button_down_pressed) {
        // Show preview when navigating
        showSettingPreview(submenu_index);
        displayMenu(); // Return to menu
        return;
      }
      break;

    case 5: // Back
      if (button_enter_pressed) {
        button_enter_pressed = false;
        DEBUG_PRINTLN(F("SETTINGS: Back to main menu"));
        in_submenu = false;
        current_menu = MENU_MAIN;
        submenu_index = 0;
        editing = false; // Reset editing state
      }
      break;

    default:
      break;
  }
}

void handleCalibrationMenu() {
  if (!calibrating_voltage) {
    // This is the entry point, consume the flag
    button_enter_pressed = false;

    switch (submenu_index) {
      case 0: // NEPA Voltage
        current_cal_value = nepa_voltage_offset;
        current_cal_source = 0;
        calibrating_voltage = true;
        DEBUG_PRINTLN(F("ENTERING NEPA VOLTAGE CALIBRATION"));
        DEBUG_PRINT(F("Initial value: "));
        DEBUG_PRINTLN(current_cal_value);
        break;
      case 1: // Gen Voltage
        current_cal_value = gen_voltage_offset;
        current_cal_source = 1;
        calibrating_voltage = true;
        DEBUG_PRINTLN(F("ENTERING GEN VOLTAGE CALIBRATION"));
        DEBUG_PRINT(F("Initial value: "));
        DEBUG_PRINTLN(current_cal_value);
        break;
      case 2: // Inv Voltage
        current_cal_value = inv_voltage_offset;
        current_cal_source = 2;
        calibrating_voltage = true;
        DEBUG_PRINTLN(F("ENTERING INV VOLTAGE CALIBRATION"));
        DEBUG_PRINT(F("Initial value: "));
        DEBUG_PRINTLN(current_cal_value);
        break;
      case 3: // Current
        current_cal_value = current_offset;
        current_cal_source = 3;
        calibrating_voltage = true;
        DEBUG_PRINTLN(F("ENTERING CURRENT CALIBRATION"));
        DEBUG_PRINT(F("Initial value: "));
        DEBUG_PRINTLN(current_cal_value);
        break;
      case 4: // Save & Exit
        saveCalibrationData();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Calibration"));
        lcd.setCursor(0, 1);
        lcd.print(F("Saved!"));
        delay(1000);
        wdt_reset(); // Reset watchdog
        delay(1000);
        exitMenu();
        break;
      case 5: // Back
        DEBUG_PRINTLN(F("CALIBRATION: Back to main menu"));
        in_submenu = false;
        current_menu = MENU_MAIN;
        submenu_index = 0;
        break;
      default:
        break;
    }
  }
}

void displayVoltageCalibration() {
  int raw_voltage = 0;
  int calibrated_voltage = 0;
  int offset = 0;

  switch (submenu_index) {
    case 0: // NEPA
      raw_voltage = sensorV_nepa * voltage_multiplier;
      calibrated_voltage = raw_voltage + nepa_voltage_offset;
      offset = nepa_voltage_offset;
      break;
    case 1: // Generator
      raw_voltage = sensorV_gen * voltage_multiplier;
      calibrated_voltage = raw_voltage + gen_voltage_offset;
      offset = gen_voltage_offset;
      break;
    case 2: // Inverter
      raw_voltage = sensorV_inv * voltage_multiplier;
      calibrated_voltage = raw_voltage + inv_voltage_offset;
      offset = inv_voltage_offset;
      break;
    default:
      break;
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Raw:"));
  lcd.print(raw_voltage);
  lcd.print(F("V Cal:"));
  lcd.print(calibrated_voltage);
  lcd.setCursor(0, 1);
  lcd.print(F("Offset:"));
  lcd.print(offset);
  lcd.print(F(" +/-"));
}

void displayCurrentCalibration() {
  float raw_current = (sensorI_load * current_multiplier) / 3;
  float calibrated_current = raw_current + (current_offset * 0.01);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Raw:"));
  lcd.print(raw_current, 1);
  lcd.print(F("A"));
  lcd.setCursor(0, 1);
  lcd.print(F("Cal:"));
  lcd.print(calibrated_current, 2);
  lcd.print(F("A Off:"));
  lcd.print(current_offset);
}

void displayTimingSettings() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Switch:"));
  lcd.print(SWITCH_DELAY/1000);
  lcd.print(F("s"));
  lcd.setCursor(0, 1);
  lcd.print(F("Return:"));
  lcd.print(SOURCE_RETURN_DELAY/1000);
  lcd.print(F("s"));
}

void displayThresholdSettings() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Min:"));
  lcd.print(MIN_VOLTAGE);
  lcd.print(F("V Max:"));
  lcd.print(MAX_VOLTAGE);
  lcd.print(F("V"));
  lcd.setCursor(0, 1);
  lcd.print(F("Hyst:"));
  lcd.print(VOLTAGE_HYSTERESIS);
  lcd.print(F("V"));
}

// New function to display current settings values
void displaySettingsValue(uint8_t setting_index) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcdPrintMenuItem(settings_menu, setting_index);

  lcd.setCursor(0, 1);
  switch (setting_index) {
    case 0: // Min Voltage
      lcd.print(F("Current: "));
      lcd.print(MIN_VOLTAGE);
      lcd.print(F("V"));
      break;
    case 1: // Max Voltage
      lcd.print(F("Current: "));
      lcd.print(MAX_VOLTAGE);
      lcd.print(F("V"));
      break;
    case 2: // Switch Delay
      lcd.print(F("Current: "));
      lcd.print(SWITCH_DELAY / 1000);
      lcd.print(F("s"));
      break;
    case 3: // Return Delay
      lcd.print(F("Current: "));
      lcd.print(SOURCE_RETURN_DELAY / 1000);
      lcd.print(F("s"));
      break;
    case 4: // Auto Mode
      lcd.print(F("Current: "));
      lcd.print(auto_mode ? F("ENABLED") : F("DISABLED"));
      break;
    case 5: // Back
      lcd.print(F("Return to main menu"));
      break;
    default:
      lcd.print(F("Unknown setting"));
      break;
  }
}

void handleMaintenance() {
  // Consume the enter button press to prevent re-triggering
  button_enter_pressed = false;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("MAINTENANCE"));
  lcd.setCursor(0, 1);
  lcd.print(F("1.Reset 2.Test"));

  unsigned long menu_start = millis();
  // Wait for user choice with timeout
  while (millis() - menu_start < 10000) { // 10 second timeout
    handleButtons();

    if (button_up_pressed) {
      button_up_pressed = false;
      // Reset energy data
      kwh_nepa = kwh_gen = kwh_inv = kwh_total = 0;
      kw_nepa = kw_gen = kw_inv = kw_total = 0;
      status.switch_count = 0;
      settings_modified = true; // Mark settings as modified
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Energy Data"));
      lcd.setCursor(0, 1);
      lcd.print(F("Reset!"));
      delay(2000);
      break;
    }

    if (button_down_pressed) {
      button_down_pressed = false;
      // Relay test sequence
      testRelays();
      break;
    }

    if (button_enter_pressed) {
      button_enter_pressed = false;
      break;
    }

    wdt_reset(); // Reset watchdog during menu
  }
}

void testRelays() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("RELAY TEST"));

  // Reset watchdog before starting tests
  wdt_reset();

  // Test NEPA relay
  lcd.setCursor(0, 1);
  lcd.print(F("Testing NEPA..."));
  digitalWrite(NEPA_RELAY_PIN, HIGH);
  status.nepa_relay_on = true;
  delay(1000); // Reduced delay time
  wdt_reset(); // Reset watchdog
  delay(1000); // Split delay to reset watchdog in between
  digitalWrite(NEPA_RELAY_PIN, LOW);
  status.nepa_relay_on = false;

  // Reset watchdog
  wdt_reset();

  // Test Generator relay
  lcd.setCursor(0, 1);
  lcd.print(F("Testing GEN... "));
  digitalWrite(GEN_RELAY_PIN, HIGH);
  status.gen_relay_on = true;
  delay(1000); // Reduced delay time
  wdt_reset(); // Reset watchdog
  delay(1000); // Split delay to reset watchdog in between
  digitalWrite(GEN_RELAY_PIN, LOW);
  status.gen_relay_on = false;

  // Reset watchdog
  wdt_reset();

  // Test Inverter relay
  lcd.setCursor(0, 1);
  lcd.print(F("Testing INV... "));
  digitalWrite(INVERTER_RELAY_PIN, HIGH);
  delay(1000); // Reduced delay time
  wdt_reset(); // Reset watchdog
  delay(1000); // Split delay to reset watchdog in between
  digitalWrite(INVERTER_RELAY_PIN, LOW);

  // Reset watchdog
  wdt_reset();

  // Test buzzer
  lcd.setCursor(0, 1);
  lcd.print(F("Testing Buzzer.."));
  digitalWrite(BUZZER_PIN, HIGH);
  delay(500); // Reduced delay time
  wdt_reset(); // Reset watchdog
  delay(500); // Split delay to reset watchdog in between
  digitalWrite(BUZZER_PIN, LOW);

  // Reset watchdog
  wdt_reset();

  lcd.setCursor(0, 1);
  lcd.print(F("Test Complete! "));
  delay(1000); // Reduced delay time
  wdt_reset(); // Reset watchdog
  delay(1000); // Split delay to reset watchdog in between

  // Final watchdog reset before returning to setup
  wdt_reset();
}

void displaySystemInfo() {
  // Consume the enter button press to prevent re-triggering
  button_enter_pressed = false;

  for (int screen = 0; screen < 3; screen++) {
    lcd.clear();

    switch (screen) {
      case 0:
        lcd.setCursor(0, 0);
        lcd.print(F("ATS System v3.0"));
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
        lcd.print(EEPROM.read(EEPROM_FAULT_LOG_ADDR));
        break;

      case 2:
        lcd.setCursor(0, 0);
        lcd.print(F("Active:"));
        switch (active_source) {
          case SOURCE_NEPA: lcd.print(F("NEPA")); break;
          case SOURCE_GENERATOR: lcd.print(F("GEN")); break;
          case SOURCE_INVERTER: lcd.print(F("INV")); break;
          default: lcd.print(F("NONE")); break;
        }
        lcd.setCursor(0, 1);
        lcd.print(F("Mode:"));
        lcd.print(auto_mode ? F("AUTO") : F("MANUAL"));
        break;

      default:
        break;
    }

    delay(1500);
    wdt_reset(); // Reset watchdog
    delay(1500);

    // Check for button press to exit early
    handleButtons();
    if (button_enter_pressed) {
      button_enter_pressed = false;
      break;
    }

    wdt_reset(); // Reset watchdog during info display
  }
}

void exitMenu() {
  DEBUG_PRINTLN(F("EXIT MENU - Resetting menu state"));

  menu_active = false;
  in_submenu = false;
  current_menu = MENU_MAIN;
  menu_index = 0;
  submenu_index = 0;

  // Reset calibration state
  calibrating_voltage = false;
  current_cal_value = 0;
  current_cal_source = 0;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Exiting Menu..."));
  delay(1000);
  wdt_reset(); // Reset watchdog

  DEBUG_PRINTLN(F("Menu state reset complete"));
}

// Improved AC reading with timeout protection
bool readACVoltage(int analogPin, long& sensorValue, unsigned long timeoutMs = AC_DETECTION_TIMEOUT) {
  unsigned long startTime = millis();

  // Wait for AC signal with timeout
  while (analogRead(analogPin) < MIN_AC_THRESHOLD) {
    if (millis() - startTime > timeoutMs) {
      sensorValue = 0;
      return false; // No AC detected
    }
    wdt_reset(); // Reset watchdog while waiting
  }

  // Take samples
  sensorValue = 0;
  for (int i = 0; i < SAMPLES_PER_READING; i++) {
    sensorValue += analogRead(analogPin);
    delay(SAMPLE_DELAY_MS);

    // Reset watchdog periodically during sampling
    if (i % 10 == 0) {
      wdt_reset();
    }
  }

  sensorValue = sensorValue / SAMPLES_PER_READING;
  return true; // AC detected and read successfully
}

// Synchronized voltage & current sampling (template-aligned)
bool readSyncedVI(int voltagePin, int currentPin, long& avgVoltageAdc, long& avgCurrentAdc, unsigned long timeoutMs = AC_DETECTION_TIMEOUT) {
  unsigned long startTime = millis();
  // Gate on AC presence for the chosen voltage pin
  while (analogRead(voltagePin) < MIN_AC_THRESHOLD) {
    if (millis() - startTime > timeoutMs) {
      avgVoltageAdc = 0;
      avgCurrentAdc = 0;
      return false;
    }
    wdt_reset();
  }

  long sumV = 0;
  long sumI = 0;
  int minV = 1023, maxV = 0;
  for (int i = 0; i < SAMPLES_PER_READING; i++) {
    int v = analogRead(voltagePin);
    sumV += v;
    if (v < minV) minV = v;
    if (v > maxV) maxV = v;
    sumI += analogRead(currentPin);
    if ((i & 0x0F) == 0) wdt_reset();
    delay(SAMPLE_DELAY_MS);
  }

  if ((maxV - minV) < AC_MIN_SPAN) {
    // Not enough swing to be real AC
    avgVoltageAdc = 0;
    avgCurrentAdc = 0;
    return false;
  }

  // Match template scaling: divide by 30 (empirically tuned with 301 samples)
  avgVoltageAdc = sumV / 30;
  avgCurrentAdc = sumI / 30;
  return true;
}

void readACPowerSources() {
  // Synchronized VI sampling per source
  long vAdc_nepa = 0, iAdc_nepa = 0;
  long vAdc_gen  = 0, iAdc_gen  = 0;
  long vAdc_inv  = 0, iAdc_inv  = 0;

  // NEPA (A0)
  if (readSyncedVI(VOLT_PIN_NEPA, CURR_PIN_LOAD, vAdc_nepa, iAdc_nepa)) {
    DEBUG_PRINT(F("DEBUG - NEPA Raw ADC: "));
    DEBUG_PRINTLN(vAdc_nepa);

    volt_nepa = (vAdc_nepa * voltage_multiplier) + nepa_voltage_offset;
    if (volt_nepa < 0) volt_nepa = 0;
    if (volt_nepa > 400) volt_nepa = 0;

    DEBUG_PRINT(F("DEBUG - NEPA Calculated Voltage: "));
    DEBUG_PRINTF(volt_nepa, DEC);
    DEBUG_PRINTLN(F("V"));

    status.nepa_valid = isVoltageValid(volt_nepa);
    sensorV_nepa = vAdc_nepa;  // Store raw sensor value for display/calibration
  } else {
    DEBUG_PRINTLN(F("DEBUG - NEPA reading failed or below threshold"));
    status.nepa_valid = false;
    volt_nepa = 0;
    vAdc_nepa = 0;
    iAdc_nepa = 0;
    sensorV_nepa = 0;
  }

  // Generator (A1)
  if (readSyncedVI(VOLT_PIN_GEN, CURR_PIN_LOAD, vAdc_gen, iAdc_gen)) {
    volt_gen = (vAdc_gen * voltage_multiplier) + gen_voltage_offset;
    if (volt_gen < 0) volt_gen = 0;
    if (volt_gen > 400) volt_gen = 0;
    status.gen_valid = isVoltageValid(volt_gen);
    sensorV_gen = vAdc_gen;  // Store raw sensor value
  } else {
    status.gen_valid = false;
    volt_gen = 0;
    vAdc_gen = 0;
    iAdc_gen = 0;
    sensorV_gen = 0;
  }

  // Inverter (A2)
  if (readSyncedVI(VOLT_PIN_INV, CURR_PIN_LOAD, vAdc_inv, iAdc_inv)) {
    volt_inv = (vAdc_inv * voltage_multiplier) + inv_voltage_offset;
    if (volt_inv < 0) volt_inv = 0;
    if (volt_inv > 400) volt_inv = 0;
    status.inv_valid = isVoltageValid(volt_inv);
    sensorV_inv = vAdc_inv;  // Store raw sensor value
  } else {
    status.inv_valid = false;
    volt_inv = 0;
    vAdc_inv = 0;
    iAdc_inv = 0;
    sensorV_inv = 0;
  }

  // Compute load current and power from synchronized samples of the active source
  long chosenIAdc = 0;
  switch (active_source) {
    case SOURCE_NEPA:
      volt_load = volt_nepa;
      chosenIAdc = iAdc_nepa;
      break;
    case SOURCE_GENERATOR:
      volt_load = volt_gen;
      chosenIAdc = iAdc_gen;
      break;
    case SOURCE_INVERTER:
      volt_load = volt_inv;
      chosenIAdc = iAdc_inv;
      break;
    default:
      // FIX: Even with no active source, we should still measure current from available source
      // This helps with initialization when we haven't selected a source yet
      if (volt_nepa > 0) {
        volt_load = volt_nepa;
        chosenIAdc = iAdc_nepa;
      } else if (volt_gen > 0) {
        volt_load = volt_gen;
        chosenIAdc = iAdc_gen;
      } else if (volt_inv > 0) {
        volt_load = volt_inv;
        chosenIAdc = iAdc_inv;
      } else {
        volt_load = 0;
        chosenIAdc = 0;
      }
      break;
  }

  if (volt_load > 0 && chosenIAdc > 0) {
    amp_load = ((chosenIAdc * current_multiplier) / 3) + (current_offset * 0.01);
    if (amp_load < 0) amp_load = 0;
    if (amp_load > 100) amp_load = 100;
    watt_load = volt_load * amp_load;
    sensorI_load = chosenIAdc;  // Store raw current sensor value
  } else {
    amp_load = 0;
    watt_load = 0;
    sensorI_load = 0;
  }

  // Attribute instantaneous power to the active source
  switch (active_source) {
    case SOURCE_NEPA: watt_nepa = watt_load; watt_gen = 0; watt_inv = 0; break;
    case SOURCE_GENERATOR: watt_gen = watt_load; watt_nepa = 0; watt_inv = 0; break;
    case SOURCE_INVERTER: watt_inv = watt_load; watt_nepa = 0; watt_gen = 0; break;
    default: watt_nepa = watt_gen = watt_inv = 0; break;
  }

  status.load_detected = (watt_load > 10);

  // Serial output (only when not in menu to avoid clutter)
  if (!menu_active) {
    DEBUG_PRINTLN(F("=== 3-Source ATS Readings ==="));
    DEBUG_PRINT(F("NEPA - Voltage: "));
    DEBUG_PRINTF(volt_nepa, DEC);
    DEBUG_PRINT(F("V, Valid: "));
    DEBUG_PRINT(status.nepa_valid ? F("YES") : F("NO"));
    DEBUG_PRINT(F(", Energy: "));
    DEBUG_PRINTF(kwh_nepa, 2);
    DEBUG_PRINTLN(F("kWh"));

    DEBUG_PRINT(F("GEN  - Voltage: "));
    DEBUG_PRINTF(volt_gen, DEC);
    DEBUG_PRINT(F("V, Valid: "));
    DEBUG_PRINT(status.gen_valid ? F("YES") : F("NO"));
    DEBUG_PRINT(F(", Running: "));
    DEBUG_PRINT(status.gen_running ? F("YES") : F("NO"));
    DEBUG_PRINT(F(", Energy: "));
    DEBUG_PRINTF(kwh_gen, 2);
    DEBUG_PRINTLN(F("kWh"));

    DEBUG_PRINT(F("INV  - Voltage: "));
    DEBUG_PRINTF(volt_inv, DEC);
    DEBUG_PRINT(F("V, Valid: "));
    DEBUG_PRINT(status.inv_valid ? F("YES") : F("NO"));
    DEBUG_PRINT(F(", Energy: "));
    DEBUG_PRINTF(kwh_inv, 2);
    DEBUG_PRINTLN(F("kWh"));

    DEBUG_PRINT(F("LOAD - Voltage: "));
    DEBUG_PRINTF(volt_load, DEC);
    DEBUG_PRINT(F("V, Current: "));
    DEBUG_PRINTF(amp_load, 2);
    DEBUG_PRINT(F("A, Power: "));
    DEBUG_PRINTF(watt_load, DEC);
    DEBUG_PRINTLN(F("W"));

    DEBUG_PRINT(F("Active Source: "));
    switch (active_source) {
      case SOURCE_NEPA: DEBUG_PRINTLN(F("NEPA")); break;
      case SOURCE_GENERATOR: DEBUG_PRINTLN(F("GENERATOR")); break;
      case SOURCE_INVERTER: DEBUG_PRINTLN(F("INVERTER")); break;
      default: DEBUG_PRINTLN(F("NONE")); break;
    }

    DEBUG_PRINT(F("Total Energy: "));
    DEBUG_PRINTF(kwh_total, 2);
    DEBUG_PRINTLN(F("kWh"));
    DEBUG_PRINT(F("System State: "));
    switch (current_state) {
      case STATE_INIT: DEBUG_PRINTLN(F("INITIALIZING")); break;
      case STATE_NEPA_ACTIVE: DEBUG_PRINTLN(F("NEPA ACTIVE")); break;
      case STATE_GEN_ACTIVE: DEBUG_PRINTLN(F("GENERATOR ACTIVE")); break;
      case STATE_INV_ACTIVE: DEBUG_PRINTLN(F("INVERTER ACTIVE")); break;
      case STATE_SWITCHING: DEBUG_PRINTLN(F("SWITCHING")); break;
      case STATE_FAULT: DEBUG_PRINTLN(F("FAULT")); break;
      case STATE_MANUAL: DEBUG_PRINTLN(F("MANUAL MODE")); break;
      default: DEBUG_PRINTLN(F("UNKNOWN")); break;
    }
    DEBUG_PRINTLN();
  }
}
//////////////////////////////////////////////////
// Fixed updateSystemState() function - replace the existing one

// Replace the existing updateSystemState() function with this complete version
void updateSystemState() {
  // If in manual mode, do not perform any automatic state changes.
  if (!auto_mode) {
    // The active_source is controlled by handleManualInput() or the menu.
    // The state should be STATE_MANUAL.
    // controlRelays() will handle the relay states based on active_source.
    return;
  }
  unsigned long current_time = millis();
  unsigned long time_in_state = current_time - status.state_change_time;
  PowerSource best_source = selectBestSource();

  // Debug output to help diagnose state transitions
  DEBUG_PRINT(F("DEBUG - Current State: "));
  DEBUG_PRINTF(current_state, DEC);
  DEBUG_PRINT(F(" ("));
  switch(current_state) {
    case STATE_INIT: DEBUG_PRINT(F("INIT")); break;
    case STATE_NEPA_ACTIVE: DEBUG_PRINT(F("NEPA_ACTIVE")); break;
    case STATE_INV_ACTIVE: DEBUG_PRINT(F("INV_ACTIVE")); break;
    case STATE_GEN_ACTIVE: DEBUG_PRINT(F("GEN_ACTIVE")); break;
    case STATE_SWITCHING: DEBUG_PRINT(F("SWITCHING")); break;
    case STATE_FAULT: DEBUG_PRINT(F("FAULT")); break;
    default: DEBUG_PRINT(F("UNKNOWN")); break;
  }
  DEBUG_PRINT(F("), Time in state: "));
  DEBUG_PRINTF(time_in_state, DEC);
  DEBUG_PRINT(F("ms, Best source: "));
  switch(best_source) {
    case SOURCE_NONE: DEBUG_PRINT(F("NONE")); break;
    case SOURCE_NEPA: DEBUG_PRINT(F("NEPA")); break;
    case SOURCE_GENERATOR: DEBUG_PRINT(F("GENERATOR")); break;
    case SOURCE_INVERTER: DEBUG_PRINT(F("INVERTER")); break;
    default: DEBUG_PRINT(F("UNKNOWN")); break;
  }
  DEBUG_PRINT(F(", Active source: "));
  switch(active_source) {
    case SOURCE_NONE: DEBUG_PRINTLN(F("NONE")); break;
    case SOURCE_NEPA: DEBUG_PRINTLN(F("NEPA")); break;
    case SOURCE_GENERATOR: DEBUG_PRINTLN(F("GENERATOR")); break;
    case SOURCE_INVERTER: DEBUG_PRINTLN(F("INVERTER")); break;
    default: DEBUG_PRINTLN(F("UNKNOWN")); break;
  }

  switch (current_state) {
    case STATE_INIT:
      DEBUG_PRINTLN(F("DEBUG - In INIT state"));
      DEBUG_PRINT(F("DEBUG - INIT conditions: time_in_state="));
      DEBUG_PRINTF(time_in_state, DEC);
      DEBUG_PRINT(F(", best_source="));
      DEBUG_PRINTLN(best_source);

              // FAST SWITCHING: Immediately activate the best available source
        if (best_source != SOURCE_NONE) {
          DEBUG_PRINTLN(F("DEBUG - Best source available, immediate activation"));

          if (best_source == SOURCE_NEPA) {
            DEBUG_PRINTLN(F("DEBUG - Activating NEPA immediately"));
            current_state = STATE_NEPA_ACTIVE;
            active_source = SOURCE_NEPA;
            status.state_change_time = current_time;
            DEBUG_PRINTLN(F("State: INIT -> NEPA_ACTIVE (immediate)"));

          } else if (best_source == SOURCE_INVERTER) {
            DEBUG_PRINTLN(F("DEBUG - Activating INVERTER immediately"));
            current_state = STATE_INV_ACTIVE;
            active_source = SOURCE_INVERTER;
            status.state_change_time = current_time;
            DEBUG_PRINTLN(F("State: INIT -> INV_ACTIVE (immediate)"));

          } else if (best_source == SOURCE_GENERATOR) {
            DEBUG_PRINTLN(F("DEBUG - Activating GENERATOR immediately"));
            current_state = STATE_GEN_ACTIVE;
            active_source = SOURCE_GENERATOR;
            status.state_change_time = current_time;
            DEBUG_PRINTLN(F("State: INIT -> GEN_ACTIVE (immediate)"));
          }

        } else if (time_in_state > 500) {  // Reduced timeout to 500ms for fast response
          DEBUG_PRINTLN(F("DEBUG - INIT timeout reached, forcing transition"));

          // Force transition even without "best" source - use any available voltage
          if (volt_nepa > 100) {  // Any reasonable voltage reading
            DEBUG_PRINTLN(F("DEBUG - Forcing NEPA activation (has voltage)"));
            current_state = STATE_NEPA_ACTIVE;
            active_source = SOURCE_NEPA;
            status.state_change_time = current_time;
            DEBUG_PRINTLN(F("State: INIT -> NEPA_ACTIVE (forced voltage)"));

          } else if (volt_inv > 100) {
            DEBUG_PRINTLN(F("DEBUG - Forcing INVERTER activation (has voltage)"));
            current_state = STATE_INV_ACTIVE;
            active_source = SOURCE_INVERTER;
            status.state_change_time = current_time;
            DEBUG_PRINTLN(F("State: INIT -> INV_ACTIVE (forced voltage)"));

          } else if (volt_gen > 100) {
            DEBUG_PRINTLN(F("DEBUG - Forcing GENERATOR activation (has voltage)"));
            current_state = STATE_GEN_ACTIVE;
            active_source = SOURCE_GENERATOR;
            status.state_change_time = current_time;
            DEBUG_PRINTLN(F("State: INIT -> GEN_ACTIVE (forced voltage)"));

          } else {
            // Last resort - default to NEPA to get system running
            DEBUG_PRINTLN(F("DEBUG - Last resort: defaulting to NEPA"));
            current_state = STATE_NEPA_ACTIVE;
            active_source = SOURCE_NEPA;
            status.state_change_time = current_time;
            DEBUG_PRINTLN(F("State: INIT -> NEPA_ACTIVE (last resort)"));
          }
        } else {
          DEBUG_PRINTLN(F("DEBUG - Still in INIT, waiting for valid source or timeout"));
        }
      break;

          case STATE_NEPA_ACTIVE:
        DEBUG_PRINTLN(F("DEBUG - In NEPA_ACTIVE state"));
        if (!status.nepa_valid || volt_nepa < MIN_VOLTAGE) {
          DEBUG_PRINTLN(F("DEBUG - NEPA failed, looking for alternatives"));
        if (time_in_state > SWITCH_DELAY) {
          // FAST SWITCHING: Check for best available source
          if (status.inv_valid && volt_inv > (MIN_VOLTAGE + VOLTAGE_HYSTERESIS)) {
            // Inverter is higher priority (instant start)
            current_state = STATE_INV_ACTIVE;
            active_source = SOURCE_INVERTER;
            status.switch_count++;
            settings_modified = true;
            DEBUG_PRINTLN(F("State: NEPA_ACTIVE -> INV_ACTIVE (fast switch)"));
          } else if (status.gen_valid && volt_gen > (MIN_VOLTAGE + VOLTAGE_HYSTERESIS)) {
            // Generator as backup
            current_state = STATE_GEN_ACTIVE;
            active_source = SOURCE_GENERATOR;
            status.switch_count++;
            settings_modified = true;
            DEBUG_PRINTLN(F("State: NEPA_ACTIVE -> GEN_ACTIVE (fast switch)"));
          } else {
            current_state = STATE_FAULT;
            status.system_fault = true;
            status.fault_code = 4; // All sources failed
            DEBUG_PRINTLN(F("State: NEPA_ACTIVE -> FAULT (all sources failed)"));
          }
          status.state_change_time = current_time;
          logFault(1); // NEPA failure
        }
      }
      break;

          case STATE_INV_ACTIVE:
        DEBUG_PRINTLN(F("DEBUG - In INV_ACTIVE state"));
      // Check if higher priority source available
      if (status.nepa_valid && volt_nepa > (MIN_VOLTAGE + VOLTAGE_HYSTERESIS)) {
        if (time_in_state > SOURCE_RETURN_DELAY) {
          current_state = STATE_NEPA_ACTIVE;
          active_source = SOURCE_NEPA;
          status.state_change_time = current_time;
          status.switch_count++;
          settings_modified = true;
          DEBUG_PRINTLN(F("State: INV_ACTIVE -> NEPA_ACTIVE (return)"));
        }
      } else if (!status.inv_valid || volt_inv < MIN_VOLTAGE) {
        if (time_in_state > SWITCH_DELAY) {
          current_state = STATE_GEN_ACTIVE;
          active_source = SOURCE_GENERATOR;
          status.state_change_time = current_time;
          status.switch_count++;
          settings_modified = true;
          logFault(2); // Inverter failure
          DEBUG_PRINTLN(F("State: INV_ACTIVE -> GEN_ACTIVE (fast switch)"));
        }
      }
      break;



          case STATE_GEN_ACTIVE:
        DEBUG_PRINTLN(F("DEBUG - In GEN_ACTIVE state"));
      // Check if higher priority sources have returned
      if (status.nepa_valid && volt_nepa > (MIN_VOLTAGE + VOLTAGE_HYSTERESIS)) {
        if (time_in_state > SOURCE_RETURN_DELAY) {
          current_state = STATE_NEPA_ACTIVE;
          active_source = SOURCE_NEPA;
          status.gen_running = false;
          status.state_change_time = current_time;
          status.switch_count++;
          settings_modified = true;
          DEBUG_PRINTLN(F("State: GEN_ACTIVE -> NEPA_ACTIVE (return)"));
        }
      } else if (!status.gen_valid || volt_gen < MIN_VOLTAGE) {
        if (time_in_state > SWITCH_DELAY) {
          // If Generator fails, check if Inverter is available
          if (status.inv_valid && volt_inv > (MIN_VOLTAGE + VOLTAGE_HYSTERESIS)) {
            current_state = STATE_INV_ACTIVE;
            active_source = SOURCE_INVERTER;
            status.gen_running = false;
            status.state_change_time = current_time;
            status.switch_count++;
            settings_modified = true;
            DEBUG_PRINTLN(F("State: GEN_ACTIVE -> INV_ACTIVE (generator failed)"));
          } else {
            current_state = STATE_FAULT;
            status.system_fault = true;
            status.fault_code = 4; // All sources failed
            status.state_change_time = current_time;
            logFault(4);
            DEBUG_PRINTLN(F("State: GEN_ACTIVE -> FAULT (all sources failed)"));
          }
        }
      }
      break;

          case STATE_FAULT:
        DEBUG_PRINTLN(F("DEBUG - In FAULT state"));
      // Try to recover every 30 seconds
      if (time_in_state > 30000) {
        status.system_fault = false;
        current_state = STATE_INIT;
        status.state_change_time = current_time;
        DEBUG_PRINTLN(F("State: FAULT -> INIT (recovery attempt)"));
      }
      break;

          case STATE_MANUAL:
        DEBUG_PRINTLN(F("DEBUG - In MANUAL state"));
      // Manual mode - no automatic switching
      break;

    default:
      DEBUG_PRINTLN(F("DEBUG - In UNKNOWN state"));
      current_state = STATE_INIT;
      status.state_change_time = current_time;
      DEBUG_PRINTLN(F("State: UNKNOWN -> INIT"));
      break;
  }
}

// selectBestSource function is now implemented above - duplicate removed

// Add this debugging function to force manual testing
void forceSourceActivation() {
  DEBUG_PRINTLN(F("=== MANUAL SOURCE ACTIVATION TEST ==="));

  // Read current voltages first
  DEBUG_PRINT(F("Current readings - NEPA: "));
  DEBUG_PRINTF(volt_nepa, DEC);
  DEBUG_PRINT(F("V, GEN: "));
  DEBUG_PRINTF(volt_gen, DEC);
  DEBUG_PRINT(F("V, INV: "));
  DEBUG_PRINTF(volt_inv, DEC);
  DEBUG_PRINTLN(F("V"));

  if (volt_inv > 100 && status.inv_valid) {
    DEBUG_PRINTLN(F("Manually activating INVERTER"));
    current_state = STATE_INV_ACTIVE;
    active_source = SOURCE_INVERTER;
    status.state_change_time = millis();
    DEBUG_PRINTLN(F("FORCED: State -> INV_ACTIVE"));

    // Also immediately control relays
    digitalWrite(NEPA_RELAY_PIN, LOW);
    digitalWrite(GEN_RELAY_PIN, LOW);
    digitalWrite(INVERTER_RELAY_PIN, HIGH);
    status.nepa_relay_on = false;
    status.gen_relay_on = false;
    status.inv_relay_on = true;
    DEBUG_PRINTLN(F("FORCED: Inverter relay activated"));

  } else if (volt_nepa > 100 && status.nepa_valid) {
    DEBUG_PRINTLN(F("Manually activating NEPA"));
    current_state = STATE_NEPA_ACTIVE;
    active_source = SOURCE_NEPA;
    status.state_change_time = millis();
    DEBUG_PRINTLN(F("FORCED: State -> NEPA_ACTIVE"));

    // Also immediately control relays
    digitalWrite(NEPA_RELAY_PIN, HIGH);
    digitalWrite(GEN_RELAY_PIN, LOW);
    digitalWrite(INVERTER_RELAY_PIN, LOW);
    status.nepa_relay_on = true;
    status.gen_relay_on = false;
    status.inv_relay_on = false;
    DEBUG_PRINTLN(F("FORCED: NEPA relay activated"));

  } else if (volt_gen > 100 && status.gen_valid) {
    DEBUG_PRINTLN(F("Manually activating GENERATOR"));
    current_state = STATE_GEN_ACTIVE;
    active_source = SOURCE_GENERATOR;
    status.state_change_time = millis();
    DEBUG_PRINTLN(F("FORCED: State -> GEN_ACTIVE"));

    // Also immediately control relays
    digitalWrite(NEPA_RELAY_PIN, LOW);
    digitalWrite(GEN_RELAY_PIN, HIGH);
    digitalWrite(INVERTER_RELAY_PIN, LOW);
    status.nepa_relay_on = false;
    status.gen_relay_on = true;
    status.inv_relay_on = false;
    DEBUG_PRINTLN(F("FORCED: Generator relay activated"));

  } else {
    DEBUG_PRINTLN(F("No source available for manual activation"));
    DEBUG_PRINTLN(F("Voltage readings:"));
    DEBUG_PRINT(F("  NEPA: "));
    DEBUG_PRINTF(volt_nepa, DEC);
    DEBUG_PRINT(F("V (valid: "));
    DEBUG_PRINT(status.nepa_valid ? F("YES") : F("NO"));
    DEBUG_PRINTLN(F(")"));
    DEBUG_PRINT(F("  Inverter: "));
    DEBUG_PRINTF(volt_inv, DEC);
    DEBUG_PRINT(F("V (valid: "));
    DEBUG_PRINT(status.inv_valid ? F("YES") : F("NO"));
    DEBUG_PRINTLN(F(")"));
    DEBUG_PRINT(F("  Generator: "));
    DEBUG_PRINTF(volt_gen, DEC);
    DEBUG_PRINT(F("V (valid: "));
    DEBUG_PRINT(status.gen_valid ? F("YES") : F("NO"));
    DEBUG_PRINTLN(F(")"));
  }
}

void emergencyStateCheck() {
  static unsigned long last_emergency_check = 0;
  unsigned long current_time = millis();

  // Emergency check every 5 seconds
  if (current_time - last_emergency_check > 5000) {
    last_emergency_check = current_time;

    // FAST SWITCHING: If stuck in INIT for more than 5 seconds, force activation of best available source
    if (current_state == STATE_INIT &&
        (current_time - status.state_change_time) > 5000) {

      if (status.inv_valid && volt_inv > MIN_VOLTAGE) {
        DEBUG_PRINTLN(F("EMERGENCY: Stuck in INIT, forcing INV activation"));
        current_state = STATE_INV_ACTIVE;
        active_source = SOURCE_INVERTER;
        status.state_change_time = current_time;

        // Force relay activation
        digitalWrite(NEPA_RELAY_PIN, LOW);
        digitalWrite(GEN_RELAY_PIN, LOW);
        digitalWrite(INVERTER_RELAY_PIN, HIGH);
        status.nepa_relay_on = false;
        status.gen_relay_on = false;
        status.inv_relay_on = true;

        DEBUG_PRINTLN(F("EMERGENCY: Inverter activated"));
      } else if (status.gen_valid && volt_gen > MIN_VOLTAGE) {
        DEBUG_PRINTLN(F("EMERGENCY: Stuck in INIT, forcing GEN activation"));
        current_state = STATE_GEN_ACTIVE;
        active_source = SOURCE_GENERATOR;
        status.state_change_time = current_time;

        // Force relay activation
        digitalWrite(NEPA_RELAY_PIN, LOW);
        digitalWrite(GEN_RELAY_PIN, HIGH);
        digitalWrite(INVERTER_RELAY_PIN, LOW);
        status.nepa_relay_on = false;
        status.gen_relay_on = true;
        status.inv_relay_on = false;

        DEBUG_PRINTLN(F("EMERGENCY: Generator activated"));
      }
    }

    // System health monitoring
    checkSystemHealth();
  }
}

// New function to monitor system health
void checkSystemHealth() {
  static uint16_t consecutive_failures = 0;
  bool system_healthy = true;

  // Check if we have any valid power source
  if (!status.nepa_valid && !status.gen_valid && !status.inv_valid) {
    consecutive_failures++;
    system_healthy = false;
    DEBUG_PRINTLN(F("HEALTH CHECK: No valid power sources detected"));
  } else {
    consecutive_failures = 0; // Reset counter if we have valid sources
  }

  // Check for relay state mismatches
  if (status.nepa_relay_on != (active_source == SOURCE_NEPA)) {
    DEBUG_PRINTLN(F("HEALTH CHECK: NEPA relay state mismatch detected"));
    system_healthy = false;
  }
  if (status.gen_relay_on != (active_source == SOURCE_GENERATOR)) {
    DEBUG_PRINTLN(F("HEALTH CHECK: Generator relay state mismatch detected"));
    system_healthy = false;
  }
  if (status.inv_relay_on != (active_source == SOURCE_INVERTER)) {
    DEBUG_PRINTLN(F("HEALTH CHECK: Inverter relay state mismatch detected"));
    system_healthy = false;
  }

  // If we have multiple consecutive failures, log a fault
  if (consecutive_failures >= 3) {
    DEBUG_PRINTLN(F("HEALTH CHECK: Multiple consecutive failures, logging fault"));
    logFault(8); // System health failure
    consecutive_failures = 0; // Reset counter
  }

  // Update system fault status
  if (!system_healthy && !status.system_fault) {
    DEBUG_PRINTLN(F("HEALTH CHECK: System health degraded"));
  } else if (system_healthy && status.system_fault) {
    DEBUG_PRINTLN(F("HEALTH CHECK: System health restored"));
    status.system_fault = false;
  }
}
//////////////////////////////////////////////////////////////////////////////**************************************


/////////////////////////////////////////////////////////////////////////////
void controlRelays() {
  // Only control relays if we're not in switching or fault state
  if (current_state == STATE_SWITCHING || current_state == STATE_FAULT) {
    DEBUG_PRINTLN(F("DEBUG - Not controlling relays due to system state"));
    return;
  }

  // Determine desired relay states
  bool nepa_should_be_on = (active_source == SOURCE_NEPA);
  bool gen_should_be_on = (active_source == SOURCE_GENERATOR);
  bool inv_should_be_on = (active_source == SOURCE_INVERTER);

  // Only change relay states if they need to change (prevents flickering)
  if (nepa_should_be_on != status.nepa_relay_on) {
    if (nepa_should_be_on) {
      // Turning NEPA ON - first turn off other relays with proper timing
      if (status.gen_relay_on) {
        digitalWrite(GEN_RELAY_PIN, LOW);
        status.gen_relay_on = false;
        delay(50); // Increased delay for better relay safety
        wdt_reset(); // Reset watchdog during delay
      }
      if (status.inv_relay_on) {
        digitalWrite(INVERTER_RELAY_PIN, LOW);
        status.inv_relay_on = false;
        delay(50); // Increased delay for better relay safety
        wdt_reset(); // Reset watchdog during delay
      }
      digitalWrite(NEPA_RELAY_PIN, HIGH);
      status.nepa_relay_on = true;
      DEBUG_PRINTLN(F("DEBUG - NEPA relay activated"));

      // Verify relay activation
      delay(5);
      if (digitalRead(NEPA_RELAY_PIN) != HIGH) {
        DEBUG_PRINTLN(F("WARNING: NEPA relay may not have activated properly"));
        logFault(5); // Relay failure
      }
    } else {
      digitalWrite(NEPA_RELAY_PIN, LOW);
      status.nepa_relay_on = false;
      DEBUG_PRINTLN(F("DEBUG - NEPA relay deactivated"));
    }
  }

  if (gen_should_be_on != status.gen_relay_on) {
    if (gen_should_be_on) {
      // Turning Generator ON - first turn off other relays
      if (status.nepa_relay_on) {
        digitalWrite(NEPA_RELAY_PIN, LOW);
        status.nepa_relay_on = false;
        delay(50); // Increased delay for better relay safety
        wdt_reset(); // Reset watchdog during delay
      }
      if (status.inv_relay_on) {
        digitalWrite(INVERTER_RELAY_PIN, LOW);
        status.inv_relay_on = false;
        delay(50); // Increased delay for better relay safety
        wdt_reset(); // Reset watchdog during delay
      }
      digitalWrite(GEN_RELAY_PIN, HIGH);
      status.gen_relay_on = true;
      DEBUG_PRINTLN(F("DEBUG - Generator relay activated"));

      // Verify relay activation
      delay(5);
      if (digitalRead(GEN_RELAY_PIN) != HIGH) {
        DEBUG_PRINTLN(F("WARNING: Generator relay may not have activated properly"));
        logFault(6); // Relay failure
      }
    } else {
      digitalWrite(GEN_RELAY_PIN, LOW);
      status.gen_relay_on = false;
      DEBUG_PRINTLN(F("DEBUG - Generator relay deactivated"));
    }
  }

  if (inv_should_be_on != status.inv_relay_on) {
    if (inv_should_be_on) {
      // Turning Inverter ON - first turn off other relays
      if (status.nepa_relay_on) {
        digitalWrite(NEPA_RELAY_PIN, LOW);
        status.nepa_relay_on = false;
        delay(50); // Increased delay for better relay safety
        wdt_reset(); // Reset watchdog during delay
      }
      if (status.gen_relay_on) {
        digitalWrite(GEN_RELAY_PIN, LOW);
        status.gen_relay_on = false;
        delay(50); // Increased delay for better relay safety
        wdt_reset(); // Reset watchdog during delay
      }
      digitalWrite(INVERTER_RELAY_PIN, HIGH);
      status.inv_relay_on = true;
      DEBUG_PRINTLN(F("DEBUG - Inverter relay activated"));

      // Verify relay activation
      delay(5);
      if (digitalRead(INVERTER_RELAY_PIN) != HIGH) {
        DEBUG_PRINTLN(F("WARNING: Inverter relay may not have activated properly"));
        logFault(7); // Relay failure
      }
    } else {
      digitalWrite(INVERTER_RELAY_PIN, LOW);
      status.inv_relay_on = false;
      DEBUG_PRINTLN(F("DEBUG - Inverter relay deactivated"));
    }
  }
}

void updateNormalDisplay() {
  if (menu_active) return; // Don't update if menu is active

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
        case SOURCE_INVERTER: lcd.print(F("INV")); break;
        default: lcd.print(F("NONE")); break;
      }

      lcd.setCursor(0, 1);
      switch (current_state) {
        case STATE_NEPA_ACTIVE: lcd.print(F("NEPA ACTIVE")); break;
        case STATE_INV_ACTIVE: lcd.print(F("INV ACTIVE")); break;
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
      lcd.print(F(" I:"));
      lcd.print(volt_inv);
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
  } else if (current_state == STATE_INV_ACTIVE) {
    // Double blink pattern for inverter
    int pattern = (millis() / 200) % 10;
    digitalWrite(STATUS_LED_PIN, (pattern < 2 || (pattern > 4 && pattern < 7)) ? HIGH : LOW);
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

bool isVoltageValid(uint16_t voltage) {
  bool is_valid = (voltage >= MIN_VOLTAGE && voltage <= MAX_VOLTAGE);

  // Debug output for voltage validation
  DEBUG_PRINT(F("DEBUG - Validating voltage: "));
  DEBUG_PRINTF(voltage, DEC);
  DEBUG_PRINT(F("V, MIN: "));
  DEBUG_PRINTF(MIN_VOLTAGE, DEC);
  DEBUG_PRINT(F(", MAX: "));
  DEBUG_PRINTF(MAX_VOLTAGE, DEC);
  DEBUG_PRINT(F(", Result: "));
  DEBUG_PRINTLN(is_valid ? F("VALID") : F("INVALID"));

  return is_valid;
}

void logFault(int fault_code) {
  status.fault_code = fault_code;

  // Save fault to EEPROM
  int fault_count = EEPROM.read(EEPROM_FAULT_LOG_ADDR);
  fault_count++;
  if (fault_count > 255) fault_count = 255; // Prevent overflow
  EEPROM.write(EEPROM_FAULT_LOG_ADDR, fault_count);

  // Mark settings as modified to ensure data is saved
  settings_modified = true;

  DEBUG_PRINT(F("FAULT LOGGED: "));
  switch (fault_code) {
    case 1: DEBUG_PRINTLN(F("NEPA Failure")); break;
    case 2: DEBUG_PRINTLN(F("Inverter Failure")); break;
    case 3: DEBUG_PRINTLN(F("Generator Start Failure")); break;
    case 4: DEBUG_PRINTLN(F("All Sources Failed")); break;
    case 5: DEBUG_PRINTLN(F("NEPA Relay Failure")); break;
    case 6: DEBUG_PRINTLN(F("Generator Relay Failure")); break;
    case 7: DEBUG_PRINTLN(F("Inverter Relay Failure")); break;
    case 8: DEBUG_PRINTLN(F("System Health Failure")); break;
    case 9: DEBUG_PRINTLN(F("Failsafe Activation Failure")); break;
    default: DEBUG_PRINTLN(F("Unknown Fault")); break;
  }
}

void saveSystemData() {
  // Save runtime hours and energy data to EEPROM
  EEPROM.put(EEPROM_RUNTIME_ADDR, status.runtime_hours);
  EEPROM.put(EEPROM_KWH_NEPA_ADDR, kwh_nepa);
  EEPROM.put(EEPROM_KWH_GEN_ADDR, kwh_gen);
  EEPROM.put(EEPROM_KWH_INV_ADDR, kwh_inv);

  // Save switch count
  EEPROM.put(EEPROM_CONFIG_ADDR, status.switch_count);

  DEBUG_PRINTLN(F("System data saved to EEPROM"));

  // Reset the settings modified flag
  settings_modified = false;
}

void saveCalibrationData() {
  // Only save if settings have been modified
  if (!settings_modified) {
    DEBUG_PRINTLN(F("No changes to save"));
    return;
  }

  // Save all calibration data to EEPROM
  EEPROM.put(EEPROM_CAL_NEPA_V_ADDR, nepa_voltage_offset);
  EEPROM.put(EEPROM_CAL_GEN_V_ADDR, gen_voltage_offset);
  EEPROM.put(EEPROM_CAL_INV_V_ADDR, inv_voltage_offset);
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

  DEBUG_PRINTLN(F("Calibration and settings saved to EEPROM"));

  // Reset the settings modified flag
  settings_modified = false;
}

void loadSystemData() {
  // Reset watchdog before EEPROM operations
  wdt_reset();

  // Load runtime hours and energy data from EEPROM
  EEPROM.get(EEPROM_RUNTIME_ADDR, status.runtime_hours);
  EEPROM.get(EEPROM_KWH_NEPA_ADDR, kwh_nepa);
  EEPROM.get(EEPROM_KWH_GEN_ADDR, kwh_gen);
  EEPROM.get(EEPROM_KWH_INV_ADDR, kwh_inv);
  EEPROM.get(EEPROM_CONFIG_ADDR, status.switch_count);

  // Reset watchdog during EEPROM operations
  wdt_reset();

  // Load calibration data
  EEPROM.get(EEPROM_CAL_NEPA_V_ADDR, nepa_voltage_offset);
  EEPROM.get(EEPROM_CAL_GEN_V_ADDR, gen_voltage_offset);
  EEPROM.get(EEPROM_CAL_INV_V_ADDR, inv_voltage_offset);
  EEPROM.get(EEPROM_CAL_CURRENT_ADDR, current_offset);

  // Reset watchdog during EEPROM operations
  wdt_reset();

  // Load settings
  struct Settings {
    int min_voltage;
    int max_voltage;
    unsigned long switch_delay;
    unsigned long return_delay;
    bool auto_mode_setting;
  } settings;

  EEPROM.get(EEPROM_SETTINGS_ADDR, settings);

  // Reset watchdog after EEPROM operations
  wdt_reset();

  // Validate and apply loaded data with bounds checking
  if (status.runtime_hours > 100000 || status.runtime_hours < 0) status.runtime_hours = 0;
  kwh_nepa = sanitizeFloat(kwh_nepa);
  kwh_gen  = sanitizeFloat(kwh_gen);
  kwh_inv  = sanitizeFloat(kwh_inv);
  if (status.switch_count > 65535 || status.switch_count < 0) status.switch_count = 0;

  // Reset watchdog during validation
  wdt_reset();

  // Validate calibration offsets
  if (nepa_voltage_offset < -100 || nepa_voltage_offset > 100) nepa_voltage_offset = 0;
  if (gen_voltage_offset < -100 || gen_voltage_offset > 100) gen_voltage_offset = 0;
  if (inv_voltage_offset < -100 || inv_voltage_offset > 100) inv_voltage_offset = 0;
  if (current_offset < -100 || current_offset > 100) current_offset = 0;

  // Validate and apply settings
  if (settings.min_voltage >= 100 && settings.min_voltage <= 200) MIN_VOLTAGE = settings.min_voltage;
  if (settings.max_voltage >= 200 && settings.max_voltage <= 300) MAX_VOLTAGE = settings.max_voltage;
  if (settings.switch_delay >= 1000 && settings.switch_delay <= 30000) SWITCH_DELAY = settings.switch_delay;
  if (settings.return_delay >= 30000 && settings.return_delay <= 300000) SOURCE_RETURN_DELAY = settings.return_delay;
  auto_mode = settings.auto_mode_setting;

  kwh_total = kwh_nepa + kwh_gen + kwh_inv;

  // Reset watchdog after completing all operations
  wdt_reset();

  DEBUG_PRINTLN(F("System data loaded from EEPROM"));
}

// Failsafe mode for critical applications
void enableFailsafeMode() {
  DEBUG_PRINTLN(F("=== FAILSAFE MODE ACTIVATED ==="));

  // Disable automatic switching
  auto_mode = false;

  // Force NEPA if available (most reliable source)
  if (status.nepa_valid && volt_nepa > MIN_VOLTAGE) {
    DEBUG_PRINTLN(F("FAILSAFE: Activating NEPA (most reliable source)"));
    current_state = STATE_NEPA_ACTIVE;
    active_source = SOURCE_NEPA;
    status.state_change_time = millis();

    // Force relay activation
    digitalWrite(NEPA_RELAY_PIN, HIGH);
    digitalWrite(GEN_RELAY_PIN, LOW);
    digitalWrite(INVERTER_RELAY_PIN, LOW);
    status.nepa_relay_on = true;
    status.gen_relay_on = false;
    status.inv_relay_on = false;

  } else if (status.inv_valid && volt_inv > MIN_VOLTAGE) {
    DEBUG_PRINTLN(F("FAILSAFE: Activating Inverter (backup source)"));
    current_state = STATE_INV_ACTIVE;
    active_source = SOURCE_INVERTER;
    status.state_change_time = millis();

    // Force relay activation
    digitalWrite(NEPA_RELAY_PIN, LOW);
    digitalWrite(GEN_RELAY_PIN, LOW);
    digitalWrite(INVERTER_RELAY_PIN, HIGH);
    status.nepa_relay_on = false;
    status.gen_relay_on = false;
    status.inv_relay_on = true;

  } else {
    DEBUG_PRINTLN(F("FAILSAFE: No reliable source available"));
    current_state = STATE_FAULT;
    status.system_fault = true;
    logFault(9); // Failsafe activation failure
  }

  // Mark settings as modified
  settings_modified = true;

  DEBUG_PRINTLN(F("=== FAILSAFE MODE COMPLETE ==="));
}

// Menu state validation and recovery
void validateMenuState() {
  DEBUG_PRINTLN(F("=== MENU STATE VALIDATION ==="));

  // Check for invalid menu states
  if (menu_active) {
    DEBUG_PRINT(F("Menu Active: YES, Current: "));
    DEBUG_PRINTF(current_menu, DEC);
    DEBUG_PRINT(F(", Index: "));
    DEBUG_PRINTF(menu_index, DEC);
    DEBUG_PRINT(F(", Submenu: "));
    DEBUG_PRINT(in_submenu ? F("YES") : F("NO"));
    DEBUG_PRINT(F(", SubIndex: "));
    DEBUG_PRINTLN(submenu_index);

    // Validate menu indices
    if (menu_index >= main_menu_size) {
      DEBUG_PRINTLN(F("FIXING: Invalid main menu index"));
      menu_index = 0;
    }

    if (in_submenu) {
      switch (current_menu) {
        case MENU_MONITOR:
          if (submenu_index >= monitor_menu_size) {
            DEBUG_PRINTLN(F("FIXING: Invalid monitor submenu index"));
            submenu_index = 0;
          }
          break;
        case MENU_SETTINGS:
          if (submenu_index >= settings_menu_size) {
            DEBUG_PRINTLN(F("FIXING: Invalid settings submenu index"));
            submenu_index = 0;
          }
          break;
        case MENU_CALIBRATION:
          if (submenu_index >= calibration_menu_size) {
            DEBUG_PRINTLN(F("FIXING: Invalid calibration submenu index"));
            submenu_index = 0;
          }
          break;
        default:
          DEBUG_PRINTLN(F("FIXING: Unknown submenu type, resetting"));
          in_submenu = false;
          submenu_index = 0;
          break;
      }
    }

    // If we're in a submenu but current_menu is invalid, fix it
    if (in_submenu && (current_menu < MENU_MONITOR || current_menu > MENU_CALIBRATION)) {
      DEBUG_PRINTLN(F("FIXING: Invalid submenu type, resetting to main"));
      in_submenu = false;
      current_menu = MENU_MAIN;
      submenu_index = 0;
    }

  } else {
    DEBUG_PRINTLN(F("Menu Active: NO"));
    // Reset all menu state if menu is not active
    in_submenu = false;
    current_menu = MENU_MAIN;
    menu_index = 0;
    submenu_index = 0;
  }

  DEBUG_PRINTLN(F("=== MENU VALIDATION COMPLETE ==="));
}

// Menu status display function
void showMenuStatus() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("MENU STATUS"));
  lcd.setCursor(0, 1);

  if (menu_active) {
    lcd.print(F("ACTIVE - "));
    switch (current_menu) {
      case MENU_MAIN:
        lcd.print(F("Main Menu"));
        break;
      case MENU_MONITOR:
        lcd.print(F("Monitor"));
        break;
      case MENU_SETTINGS:
        lcd.print(F("Settings"));
        break;
      case MENU_CALIBRATION:
        lcd.print(F("Calibration"));
        break;
      default:
        lcd.print(F("Unknown"));
        break;
    }
  } else {
    lcd.print(F("INACTIVE"));
  }

  delay(3000);
  wdt_reset();

  if (menu_active) {
    displayMenu(); // Return to menu
  }
}

// Settings help function
void showSettingsHelp() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("SETTINGS HELP"));
  lcd.setCursor(0, 1);
  lcd.print(F("ENTER: Preview"));
  delay(2000);
  wdt_reset();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("ENTER: Edit"));
  lcd.setCursor(0, 1);
  lcd.print(F("UP/DOWN: Change"));
  delay(2000);
  wdt_reset();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("ENTER: Save"));
  lcd.setCursor(0, 1);
  lcd.print(F("BACK: Exit"));
  delay(2000);
  wdt_reset();
}

// Simple settings preview function
void showSettingPreview(uint8_t setting_index) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Current:"));
  lcdPrintMenuItem(settings_menu, setting_index);

  lcd.setCursor(0, 1);
  switch (setting_index) {
    case 0: // Min Voltage
      lcd.print(F("Value: "));
      lcd.print(MIN_VOLTAGE);
      lcd.print(F("V"));
      break;
    case 1: // Max Voltage
      lcd.print(F("Value: "));
      lcd.print(MAX_VOLTAGE);
      lcd.print(F("V"));
      break;
    case 2: // Switch Delay
      lcd.print(F("Value: "));
      lcd.print(SWITCH_DELAY / 1000);
      lcd.print(F("s"));
      break;
    case 3: // Return Delay
      lcd.print(F("Value: "));
      lcd.print(SOURCE_RETURN_DELAY / 1000);
      lcd.print(F("s"));
      break;
    case 4: // Auto Mode
      lcd.print(F("Value: "));
      lcd.print(auto_mode ? F("ENABLED") : F("DISABLED"));
      break;
    case 5: // Back
      lcd.print(F("Return to main"));
      break;
    default:
      lcd.print(F("Unknown"));
      break;
  }

  delay(2000);
  wdt_reset();
}
