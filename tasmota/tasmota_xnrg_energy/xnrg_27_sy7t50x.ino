/*
  xnrg_27_sy7t50x.ino - Silergy SY7T501/SY7T502 energy sensor and relay support for Tasmota

  SPDX-License-Identifier: GPL-3.0-only
*/

#ifdef USE_ENERGY_SENSOR
#ifdef USE_SY7T50X
/*********************************************************************************************\
 * Silergy SY7T501/SY7T502 single-phase energy measurement IC with relay control
 *
 * Used in Ogemray SW40 25A (Powered by Shelly, ESP-Shelly-C38F) where the IC, not a GPIO, drives
 * the relay coil. The driver therefore also claims POWER1 when no GPIO relay is configured.
 *
 * The IC has no non-volatile storage: every setting is lost on reset, so the driver re-applies
 * them, including the wanted relay state, whenever PAD_CNTL shows the coil output is no longer set.
 *
 * Board values come from the Ogemray stock firmware 2.1.1 (shelly_sy7t502_driver.cpp): register
 * writes, scale constants and the sign of WATT_X. Note that the IC only runs with its power pin
 * low, so GPIO7 is Output Lo in the template, and that RLY_STATUS bit 1 is set with the contact
 * open, unlike the datasheet text.
 *
 * The board NTC is on GPIO_ADC_TEMP and the status LED on GPIO_SY7T50X_LED, a single WS2812 pixel
 * the driver writes directly (see Sy7UpdateLed) so the relay stays POWER1 rather than becoming a
 * second power device. Overtemp cut-off uses Energy->use_overtemp; the analog driver now samples the
 * NTC every second so that cut-off acts on a fresh reading. Tasmota's cut-off is a single threshold
 * (SetOption42) that latches off, unlike stock's 105C-trip / 95C-clear auto-reclose hysteresis.
 *
 * Template (Ogemray SW40 25A, ESP32-C3):
 *   {"NAME":"Ogemray 25A","GPIO":[0,4736,0,0,32,12608,160,3872,0,0,12576,0,0,0,0,0,0,0,12640,0,0,0],"FLAG":0,"BASE":1}
 *
 *   GPIO1  ADC Temp (NTC)  4736    GPIO6  Switch1          160
 *   GPIO4  Button1           32    GPIO7  Output Lo        3872   (IC power, active low)
 *   GPIO5  SY7T50X Rx     12608    GPIO10 SY7T50X Tx      12576
 *   GPIO18 SY7T50X Led    12640    (status LED: red no-link, green relay closed, off relay open)
 *
 *   After applying: Module 0, then AdcParam1 2,10000,10000,3450 for the NTC.
 *
 * Commands:
 *   VoltageSet / CurrentSet / PowerSet  Calibrate against a known load
 *   EnergyConfig                        Show link, register and relay contact state
 *   EnergyConfig Read <addr>            Read register, result is logged
 *   EnergyConfig Write <addr> <value>   Write register
 *   EnergyConfig Pad <value>            Override PAD_CNTL   (lost on restart, use a rule to persist)
 *   EnergyConfig Relay <0..3>           Override RLY_TYP    (lost on restart, use a rule to persist)
 *   EnergyConfig Ce <value>             Override CE_CONFIG  (lost on restart, use a rule to persist)
\*********************************************************************************************/

#define XNRG_27                 27

#include <TasmotaSerial.h>

#ifndef SY7T50X_VOLTAGE_CAL
#define SY7T50X_VOLTAGE_CAL     231800       // VRMS_X counts per V
#endif
#ifndef SY7T50X_CURRENT_CAL
#define SY7T50X_CURRENT_CAL     581586       // IRMS_X counts per A, with SY7T50X_I0GAIN applied
#endif
#ifndef SY7T50X_POWER_CAL
#define SY7T50X_POWER_CAL       100000       // WATT_X counts per 100W, with SY7T50X_I0GAIN applied
#endif
#ifndef SY7T50X_I0GAIN
#define SY7T50X_I0GAIN          0x3E58       // Current channel gain 1.974
#endif
#ifndef SY7T50X_CE_CONFIG
#define SY7T50X_CE_CONFIG       0x00000000   // Pre-amp gain 1
#endif
#ifndef SY7T50X_PAD_CNTL
#define SY7T50X_PAD_CNTL        0x001AFF47   // DO0 push-pull, active high relay control, with pull-down
#endif
#ifndef SY7T50X_RLY_TYPE
#define SY7T50X_RLY_TYPE        0            // 0 = Single coil non-latching, 1 = Single coil latching, 2 = Dual coil latching
#endif
#ifndef SY7T50X_ACCUMCYC
#define SY7T50X_ACCUMCYC        50           // Line cycles per frame: one second at 50Hz
#endif
#ifndef SY7T50X_MIN_POWER
#define SY7T50X_MIN_POWER       1.0f         // [W] Below this the load is reported as zero
#endif

#define SY7_BAUDRATE            4800
#define SY7_PREAMBLE            0xEA
#define SY7_SYNC                0x55         // Auto-baud synchronization character
#define SY7_CMD_WRITE           0x80
#define SY7_REPLY_ERROR         0x80

#define SY7_REG_LF_PERIOD       0x14
#define SY7_REG_ZC_RLY_CNTL     0x18
#define SY7_REG_PAD_CNTL        0x1C
#define SY7_REG_RLY_STATUS      0x40
#define SY7_MEM_ACCUMCYC        0x83
#define SY7_MEM_I0GAIN          0x87
#define SY7_MEM_FRAME_X         0x90         // Block read start: FRAME_X, FREQ_LP, VRMS_X, IRMS_X, WATT_X
#define SY7_MEM_CE_CONFIG       0x98

#define SY7_ZC_RLY_DEFAULT      0x0F010F00   // ZC_BLANK_TIME, ZC_DEGLITCH and T_PW at their documented defaults
#define SY7_RLY_ON              0x01
#define SY7_RLY_OFF             0x02
#define SY7_STATUS_OPEN         0x02
#define SY7_PAD_DO0_MASK        0x07

#define SY7_BLOCK_WORDS         5
#define SY7_BLOCK_BYTES         (SY7_BLOCK_WORDS * 4)
#define SY7_RX_SIZE             (SY7_BLOCK_BYTES + 4)
#define SY7_REPLY_TIMEOUT       400          // [ms] As stock, longest reply takes 50ms at 4800 baud
#define SY7_SYNC_WAIT           100          // [ms] Time for the auto-baud block to lock
#define SY7_MAX_RETRIES         5
#define SY7_CONTACT_SETTLE      2            // [s] Time allowed for the contact to follow a command
#define SY7_CONTACT_LOG_PAUSE   30           // [s] Between repeated contact errors
#define SY7_MAX_ENERGY_GAP      5000         // [ms] Longest gap between readings to integrate over

enum Sy7Jobs {
  SY7_JOB_NONE, SY7_JOB_CE_CONFIG, SY7_JOB_ACCUMCYC, SY7_JOB_I0GAIN, SY7_JOB_ZC_RLY, SY7_JOB_PAD,
  SY7_JOB_RELAY, SY7_JOB_USER, SY7_JOB_BLOCK, SY7_JOB_PERIOD, SY7_JOB_STATUS, SY7_JOB_PAD_CHECK };

TasmotaSerial *Sy7Serial = nullptr;

struct SY7 {
  uint32_t reply_timeout;                    // millis() at which a pending job failed
  uint32_t sync_done;                        // millis() at which the sync character has settled
  uint32_t ce_config;
  uint32_t pad_cntl;
  uint32_t frame;
  uint32_t user_value;
  uint32_t last_reading;                     // millis() of the previous reading
  uint32_t raw_voltage;
  uint32_t raw_current;
  int32_t raw_power;
  uint8_t rx[SY7_RX_SIZE];
  uint8_t rx_count;
  uint8_t rx_expected;
  uint8_t retries;
  uint8_t contact_timer;
  uint8_t contact;                           // Last RLY_STATUS
  uint8_t req_addr;                          // Address of the job waiting for a reply
  uint8_t user_addr;
  uint8_t relay_type;
  uint8_t job;                                // Job waiting for a reply
  uint8_t init_step;                          // Next init job, NONE once initialised
  bool synced;
  bool relay_owner;                          // Relay is driven by the IC rather than a GPIO
  bool relay_want;
  bool relay_pending;
  bool user_pending;
  bool user_write;
  bool poll_block;
  bool poll_period;
  bool poll_status;
  bool poll_pad;
  bool reading_valid;                        // last_reading is usable
  bool online;
  uint8_t led_r, led_g, led_b;               // Last colour written to the board status LED
  bool led_valid;                            // led_r/g/b have been written at least once
} Sy7;

/********************************************************************************************/

uint8_t Sy7Checksum(const uint8_t *data, uint32_t length) {
  uint8_t sum = 0;
  for (uint32_t i = 0; i < length; i++) { sum += data[i]; }
  return sum;
}

uint32_t Sy7Word(const uint8_t *data) {
  return data[0] | (data[1] << 8) | (data[2] << 16) | ((uint32_t)data[3] << 24);
}

void Sy7Start(uint32_t job, uint32_t expected) {
  Sy7.job = job;
  Sy7.rx_count = 0;
  Sy7.rx_expected = expected;
  Sy7.reply_timeout = millis() + SY7_REPLY_TIMEOUT;
}

void Sy7Write(uint32_t job, uint8_t address, uint32_t value) {
  uint8_t frame[8] = { SY7_PREAMBLE, SY7_CMD_WRITE | 4, address,
    (uint8_t)value, (uint8_t)(value >> 8), (uint8_t)(value >> 16), (uint8_t)(value >> 24), 0 };
  frame[7] = Sy7Checksum(frame, 7);
  Sy7Serial->write(frame, sizeof(frame));
  Sy7.req_addr = address;
  Sy7Start(job, 3);                          // Preamble, ack & count, checksum
}

void Sy7Read(uint32_t job, uint8_t address, uint32_t bytes) {
  uint8_t frame[4] = { SY7_PREAMBLE, (uint8_t)bytes, address, 0 };
  frame[3] = Sy7Checksum(frame, 3);
  Sy7Serial->write(frame, sizeof(frame));
  Sy7.req_addr = address;
  Sy7Start(job, bytes + 4);                  // Preamble, ack & count, address, data, checksum
}

void Sy7Restart(void) {
  while (Sy7Serial->available()) { Sy7Serial->read(); }
  uint8_t sync = SY7_SYNC;
  Sy7Serial->write(&sync, 1);
  Sy7.sync_done = millis() + SY7_SYNC_WAIT;
  Sy7.synced = false;
  Sy7.job = SY7_JOB_NONE;
  Sy7.init_step = SY7_JOB_CE_CONFIG;
  Sy7.retries = 0;
  Sy7.reading_valid = false;
}

void Sy7Failed(void) {
  uint32_t job = Sy7.job;
  Sy7.job = SY7_JOB_NONE;
  Sy7.retries++;
  if (SY7_JOB_RELAY == job) {
    Sy7.relay_pending = true;                // Not acknowledged, send it again
  }
  if (Sy7.retries > SY7_MAX_RETRIES) {
    if (SY7_JOB_USER == job) {               // A bad address must not keep the link down
      AddLog(LOG_LEVEL_ERROR, PSTR("SY7: No valid reply for register %02X"), Sy7.user_addr);
      Sy7.user_pending = false;
      Sy7.retries = 0;
      return;
    }
    if (Sy7.online) {
      AddLog(LOG_LEVEL_ERROR, PSTR("SY7: No valid reply to job %d, resynchronizing"), job);
      Sy7.online = false;
    }
    Sy7Restart();
  }
}

void Sy7ProcessBlock(void) {
  const uint8_t *data = &Sy7.rx[3];

  Sy7.frame = Sy7Word(data);
  Sy7.raw_voltage = Sy7Word(data + 8);
  Sy7.raw_current = Sy7Word(data + 12);
  Sy7.raw_power = -(int32_t)Sy7Word(data + 16);  // The shunt is wired so a load reads negative

  float voltage = (float)Sy7.raw_voltage / EnergyGetCalibration(ENERGY_VOLTAGE_CALIBRATION);
  float current = (float)Sy7.raw_current / EnergyGetCalibration(ENERGY_CURRENT_CALIBRATION);
  float power = (float)Sy7.raw_power * 100.0f / EnergyGetCalibration(ENERGY_POWER_CALIBRATION);
  Energy->voltage[0] = (voltage < 1.0f) ? 0.0f : voltage;
  Energy->current[0] = (current < 0.01f) ? 0.0f : current;                       // Zero below 10mA, each
  Energy->active_power[0] = (fabsf(power) < SY7T50X_MIN_POWER) ? 0.0f : power;    // and within 1W, independently as stock
  Energy->data_valid[0] = 0;

  uint32_t now = millis();
  if (Sy7.reading_valid) {
    uint32_t elapsed = now - Sy7.last_reading;
    if (elapsed <= SY7_MAX_ENERGY_GAP) {
      Energy->kWhtoday_delta[0] += Energy->active_power[0] * elapsed / 36;  // In 10^-5 Wh, signed: the core books export separately (SetOption162)
    }
    EnergyUpdateToday();
  }
  Sy7.last_reading = now;
  Sy7.reading_valid = true;
}

void Sy7Reply(void) {
  uint32_t job = Sy7.job;
  Sy7.job = SY7_JOB_NONE;
  Sy7.retries = 0;
  if (!Sy7.online) {
    AddLog(LOG_LEVEL_INFO, PSTR("SY7: Link up"));
    Sy7.online = true;
  }

  switch (job) {
    case SY7_JOB_CE_CONFIG:
    case SY7_JOB_ACCUMCYC:
    case SY7_JOB_I0GAIN:
    case SY7_JOB_ZC_RLY:
      Sy7.init_step = job +1;                // Init jobs are in enum order
      break;
    case SY7_JOB_PAD:
      Sy7.init_step = SY7_JOB_NONE;
      Sy7.relay_pending = Sy7.relay_owner;   // Coil output is live now, apply the wanted state
      break;
    case SY7_JOB_RELAY:
      Sy7.contact_timer = SY7_CONTACT_SETTLE;
      break;
    case SY7_JOB_USER:
      Sy7.user_pending = false;
      if (Sy7.user_write) {
        AddLog(LOG_LEVEL_INFO, PSTR("SY7: Wrote %02X = %08X"), Sy7.user_addr, Sy7.user_value);
      } else {
        AddLog(LOG_LEVEL_INFO, PSTR("SY7: Read %02X = %08X"), Sy7.user_addr, Sy7Word(&Sy7.rx[3]));
      }
      break;
    case SY7_JOB_BLOCK:
      Sy7.poll_block = false;
      Sy7ProcessBlock();
      break;
    case SY7_JOB_PERIOD: {
      Sy7.poll_period = false;
      uint32_t period = Sy7Word(&Sy7.rx[3]) & 0xFFFF;          // In 1.6us counts
      Energy->frequency[0] = (period) ? 625000.0f / period : NAN;
      break;
    }
    case SY7_JOB_STATUS:
      Sy7.poll_status = false;
      Sy7.contact = Sy7.rx[3];
      if (Sy7.relay_owner && !Sy7.relay_pending && !Sy7.contact_timer && Energy->voltage[0] > 0) {
        bool closed = !(Sy7.contact & SY7_STATUS_OPEN);
        if (closed != Sy7.relay_want) {
          AddLog(LOG_LEVEL_ERROR, PSTR("SY7: Relay commanded %s but contact reads %s (status %02X)"),
            GetStateText(Sy7.relay_want), GetStateText(closed), Sy7.contact);
          Sy7.contact_timer = SY7_CONTACT_LOG_PAUSE;
        }
      }
      break;
    case SY7_JOB_PAD_CHECK:
      Sy7.poll_pad = false;
      if ((Sy7Word(&Sy7.rx[3]) ^ Sy7.pad_cntl) & SY7_PAD_DO0_MASK) {  // Reset default has DO0 disabled
        AddLog(LOG_LEVEL_INFO, PSTR("SY7: PAD_CNTL %08X, restoring configuration"), Sy7Word(&Sy7.rx[3]));
        Sy7Restart();
      }
      break;
    default:
      break;
  }
}

void Sy7ReadReply(void) {
  while (Sy7Serial->available()) {
    uint8_t byte = Sy7Serial->read();
    if (SY7_JOB_NONE == Sy7.job) { continue; }                  // Nobody asked
    if ((0 == Sy7.rx_count) && (byte != SY7_PREAMBLE)) { continue; }
    Sy7.rx[Sy7.rx_count++] = byte;

    if ((3 == Sy7.rx_count) && (Sy7.rx[1] & SY7_REPLY_ERROR)) {  // Error replies carry no data
      AddLog(LOG_LEVEL_DEBUG, PSTR("SY7: Job %d refused"), Sy7.job);
      Sy7Failed();
      return;
    }
    if (Sy7.rx_count >= Sy7.rx_expected) {
      if (Sy7Checksum(Sy7.rx, Sy7.rx_count -1) != Sy7.rx[Sy7.rx_count -1]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("SY7: Job %d bad checksum %*_H"), Sy7.job, Sy7.rx_count, Sy7.rx);
        Sy7Failed();
      } else if ((Sy7.rx_expected > 3) && (Sy7.rx[2] != Sy7.req_addr)) {  // Read reply echoes the requested address
        AddLog(LOG_LEVEL_DEBUG, PSTR("SY7: Job %d echoed address %02X, wanted %02X"), Sy7.job, Sy7.rx[2], Sy7.req_addr);
        Sy7Failed();
      } else {
        Sy7Reply();
      }
      return;
    }
  }
}

void Sy7NextJob(void) {
  if (!Sy7.synced) {
    if (!TimeReached(Sy7.sync_done)) { return; }
    Sy7.synced = true;
  }

  switch (Sy7.init_step) {
    case SY7_JOB_CE_CONFIG:
      Sy7Write(SY7_JOB_CE_CONFIG, SY7_MEM_CE_CONFIG, Sy7.ce_config);
      return;
    case SY7_JOB_ACCUMCYC:
      Sy7Write(SY7_JOB_ACCUMCYC, SY7_MEM_ACCUMCYC, SY7T50X_ACCUMCYC);
      return;
    case SY7_JOB_I0GAIN:
      Sy7Write(SY7_JOB_I0GAIN, SY7_MEM_I0GAIN, SY7T50X_I0GAIN);
      return;
    case SY7_JOB_ZC_RLY:
      Sy7Write(SY7_JOB_ZC_RLY, SY7_REG_ZC_RLY_CNTL, SY7_ZC_RLY_DEFAULT | (Sy7.relay_type << 4));
      return;
    case SY7_JOB_PAD:
      Sy7Write(SY7_JOB_PAD, SY7_REG_PAD_CNTL, Sy7.pad_cntl);
      return;
    default:
      break;
  }

  if (Sy7.relay_pending) {
    Sy7.relay_pending = false;               // Set again by a newer command or a failure
    Sy7Write(SY7_JOB_RELAY, SY7_REG_ZC_RLY_CNTL, SY7_ZC_RLY_DEFAULT | (Sy7.relay_type << 4) |
      ((Sy7.relay_want) ? SY7_RLY_ON : SY7_RLY_OFF));
  }
  else if (Sy7.user_pending) {
    if (Sy7.user_write) {
      Sy7Write(SY7_JOB_USER, Sy7.user_addr, Sy7.user_value);
    } else {
      Sy7Read(SY7_JOB_USER, Sy7.user_addr, 4);
    }
  }
  else if (Sy7.poll_block) {
    Sy7Read(SY7_JOB_BLOCK, SY7_MEM_FRAME_X, SY7_BLOCK_BYTES);
  }
  else if (Sy7.poll_period) {
    Sy7Read(SY7_JOB_PERIOD, SY7_REG_LF_PERIOD, 4);
  }
  else if (Sy7.poll_status) {
    Sy7Read(SY7_JOB_STATUS, SY7_REG_RLY_STATUS, 4);
  }
  else if (Sy7.poll_pad) {
    Sy7Read(SY7_JOB_PAD_CHECK, SY7_REG_PAD_CNTL, 4);
  }
}

void Sy7Loop(void) {
  if (!Sy7Serial) { return; }

  Sy7ReadReply();
  if (Sy7.job != SY7_JOB_NONE) {
    if (TimeReached(Sy7.reply_timeout)) {
      AddLog(LOG_LEVEL_DEBUG, PSTR("SY7: Job %d timeout after %d bytes"), Sy7.job, Sy7.rx_count);
      Sy7Failed();
    }
    return;
  }
  Sy7NextJob();
}

void Sy7UpdateLed(void) {
  // The board's status LED is a single WS2812 pixel on its own pin, not a relay, so the driver
  // drives it directly and the relay stays POWER1. Red = no IC link, green = relay closed, off =
  // relay open. To use it as a plain connectivity indicator instead, drop GPIO_SY7T50X_LED from the
  // template and assign the pin as WS2812, matching stock firmware's status-light behaviour.
  if (!PinUsed(GPIO_SY7T50X_LED)) { return; }
  uint8_t r, g, b;
  if (!Sy7.online)         { r = 24; g = 0;  b = 0; }   // No reply from the IC
  else if (Sy7.relay_want) { r = 0;  g = 24; b = 0; }   // Relay commanded closed
  else                     { r = 0;  g = 0;  b = 0; }   // Relay commanded open
  if (Sy7.led_valid && (r == Sy7.led_r) && (g == Sy7.led_g) && (b == Sy7.led_b)) { return; }
  Sy7.led_r = r; Sy7.led_g = g; Sy7.led_b = b; Sy7.led_valid = true;
#ifdef ESP32
  rgbLedWrite(Pin(GPIO_SY7T50X_LED), r, g, b);
#endif
}

void Sy7EverySecond(void) {
  if (!Sy7Serial) { return; }
  Sy7UpdateLed();

  if (Energy->data_valid[0] > ENERGY_WATCHDOG) {
    Energy->frequency[0] = NAN;              // Core watchdog zeroes the other readings
  }
  if (Sy7.contact_timer) { Sy7.contact_timer--; }

  Sy7.poll_block = true;
  Sy7.poll_period = true;
  Sy7.poll_status = Sy7.relay_owner;
  Sy7.poll_pad = true;
}

bool Sy7SetPower(void) {
  if (!Sy7.relay_owner) { return false; }

  Sy7.relay_want = XdrvMailbox.index & 1;
  Sy7.relay_pending = true;                  // Serviced by Sy7Loop, also when the IC is not ready yet
  Sy7UpdateLed();                            // Reflect the commanded state at once
  return true;
}

void Sy7ResponseState(void) {
  Response_P(PSTR("{\"SY7T50X\":{\"Online\":%d,\"Frame\":%u,\"Vrms\":%u,\"Irms\":%u,\"Watt\":%d,"
                  "\"Pad\":\"%08X\",\"Ce\":\"%08X\",\"Relay\":%d,\"Contact\":\"%02X\"}}"),
    Sy7.online, Sy7.frame, Sy7.raw_voltage, Sy7.raw_current, Sy7.raw_power,
    Sy7.pad_cntl, Sy7.ce_config, Sy7.relay_type, Sy7.contact);
}

void Sy7Config(void) {
  char *tokens = XdrvMailbox.data;
  char *save = nullptr;
  char *keyword = (XdrvMailbox.data_len) ? strtok_r(tokens, " ,", &save) : nullptr;
  if (!keyword) {
    Sy7ResponseState();
    return;
  }
  char *arg1 = strtok_r(nullptr, " ,", &save);
  char *arg2 = strtok_r(nullptr, " ,", &save);
  uint32_t value1 = (arg1) ? strtoul(arg1, nullptr, 0) : 0;

  bool is_read = !strcasecmp_P(keyword, PSTR("Read"));
  bool is_write = !strcasecmp_P(keyword, PSTR("Write"));
  if ((is_read || is_write) && Sy7.user_pending) {
    ResponseCmndError();                       // A diagnostic read or write is already in flight
    return;
  }

  if (is_read && arg1 && (value1 < 0x100)) {
    Sy7.user_addr = value1;
    Sy7.user_write = false;
    Sy7.user_pending = true;
  }
  else if (is_write && arg2 && (value1 < 0x100)) {
    Sy7.user_addr = value1;
    Sy7.user_value = strtoul(arg2, nullptr, 0);
    Sy7.user_write = true;
    Sy7.user_pending = true;
  }
  else if (!strcasecmp_P(keyword, PSTR("Pad")) && arg1) {
    Sy7.pad_cntl = value1;
    Sy7Restart();
  }
  else if (!strcasecmp_P(keyword, PSTR("Ce")) && arg1) {
    Sy7.ce_config = value1;
    Sy7Restart();
  }
  else if (!strcasecmp_P(keyword, PSTR("Relay")) && arg1 && (value1 < 4)) {
    Sy7.relay_type = value1;
    Sy7Restart();
  }
  else {
    ResponseCmndError();
    return;
  }
  Sy7ResponseState();
}

bool Sy7Command(void) {
  bool serviced = true;
  float value = CharToFloat(XdrvMailbox.data);

  if ((CMND_POWERCAL == Energy->command_code) ||
      (CMND_VOLTAGECAL == Energy->command_code) ||
      (CMND_CURRENTCAL == Energy->command_code)) {
    // Service in xdrv_03_energy.ino
  }
  else if (CMND_POWERSET == Energy->command_code) {        // xxx.xx W
    // Without a reading the payload must not reach EnergyCommandCalSetResponse as a calibration
    XdrvMailbox.payload = (XdrvMailbox.data_len && Sy7.raw_power && (value > 0)) ?
      abs(Sy7.raw_power) * 100.0f / value : 0;
  }
  else if (CMND_VOLTAGESET == Energy->command_code) {      // xxx.xx V
    XdrvMailbox.payload = (XdrvMailbox.data_len && Sy7.raw_voltage && (value > 0)) ?
      Sy7.raw_voltage / value : 0;
  }
  else if (CMND_CURRENTSET == Energy->command_code) {      // xxx.xx mA
    XdrvMailbox.payload = (XdrvMailbox.data_len && Sy7.raw_current && (value > 0)) ?
      Sy7.raw_current * 1000.0f / value : 0;
  }
  else if (CMND_ENERGYCONFIG == Energy->command_code) {
    Sy7Config();
  }
  else serviced = false;  // Unknown command

  return serviced;
}

void Sy7Init(void) {
  // Software serial init needs to be done here as earlier (serial) interrupts may lead to Exceptions
  Sy7Serial = new TasmotaSerial(Pin(GPIO_SY7T50X_RX), Pin(GPIO_SY7T50X_TX), 1);
  if (Sy7Serial->begin(SY7_BAUDRATE)) {
    if (Sy7Serial->hardwareSerial()) {
      ClaimSerial();
    }
#ifdef ESP32
    AddLog(LOG_LEVEL_DEBUG, PSTR("SY7: Serial UART%d"), Sy7Serial->getUart());
#endif
    if (HLW_UREF_PULSE == EnergyGetCalibration(ENERGY_VOLTAGE_CALIBRATION)) {
      EnergySetCalibration(ENERGY_POWER_CALIBRATION, SY7T50X_POWER_CAL);
      EnergySetCalibration(ENERGY_VOLTAGE_CALIBRATION, SY7T50X_VOLTAGE_CAL);
      EnergySetCalibration(ENERGY_CURRENT_CALIBRATION, SY7T50X_CURRENT_CAL);
    }
    if (!TasmotaGlobal.power_on_delay) {     // SetOption47 applies the state later
      Sy7.relay_want = TasmotaGlobal.power & 1;  // Restored without SetDevicePower when SetOption0 is off
    }
    Sy7Restart();
  } else {
    TasmotaGlobal.energy_driver = ENERGY_NONE;
  }
}

void Sy7PreInit(void) {
  if (PinUsed(GPIO_SY7T50X_RX) && PinUsed(GPIO_SY7T50X_TX)) {
    Sy7.ce_config = SY7T50X_CE_CONFIG;
    Sy7.pad_cntl = SY7T50X_PAD_CNTL;
    Sy7.relay_type = SY7T50X_RLY_TYPE;
    Sy7.relay_owner = (0 == TasmotaGlobal.devices_present);
    if (Sy7.relay_owner) {
      UpdateDevicesPresent(1);
    }
    Energy->use_overtemp = true;             // Use global temperature for overtemp detection
    TasmotaGlobal.energy_driver = XNRG_27;
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xnrg27(uint32_t function) {
  bool result = false;

  switch (function) {
    case FUNC_LOOP:
      Sy7Loop();
      break;
    case FUNC_EVERY_SECOND:
      Sy7EverySecond();
      break;
    case FUNC_SET_DEVICE_POWER:
      result = Sy7SetPower();
      break;
    case FUNC_COMMAND:
      result = Sy7Command();
      break;
    case FUNC_INIT:
      Sy7Init();
      break;
    case FUNC_PRE_INIT:
      Sy7PreInit();
      break;
  }
  return result;
}

#endif  // USE_SY7T50X
#endif  // USE_ENERGY_SENSOR
