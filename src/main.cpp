#include <Arduino.h>
#include <EEPROM.h>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <ctime>

// See README.md for architecture, wiring, boot flow, and the full BLE command reference.

namespace
{

  constexpr const char *kFirmwareVersion = "1.0.0";

  constexpr bool kDebugLogEnabled = true;

  template <typename T>
  inline void dbgPrint(const T &v)
  {
    if (kDebugLogEnabled)
    {
      Serial.print(v);
    }
  }

  template <typename T>
  inline void dbgPrintln(const T &v)
  {
    if (kDebugLogEnabled)
    {
      Serial.println(v);
    }
  }

  inline void dbgPrintln()
  {
    if (kDebugLogEnabled)
    {
      Serial.println();
    }
  }

  HardwareSerial &notecardUart = Serial1;
  constexpr uint8_t kNotecardAttnPin = D5;

  constexpr uint8_t kBleUartRxPin = A0;
  constexpr uint8_t kBleUartTxPin = A3;
  HardwareSerial bleUart(kBleUartRxPin, kBleUartTxPin);

  constexpr uint8_t kAdcPin = PA1; // water level sensor

  constexpr const char *kInboundNotefile = "data.qi";       // note.get
  constexpr const char *kOutboundNotefile = "retrofit.qo"; // note.add

  // Must match this device's id in the app's categories.ts.
  constexpr const char *kDeviceCategoryId = "dewater-pump-float";

  // Set true to wipe EEPROM on every boot (testing/reflashing only).
  constexpr bool kClearEepromOnBoot = 0;

  constexpr uint16_t kEepromFlagAddr = 0;
  constexpr uint16_t kEepromProductUidAddr = 1;
  constexpr uint16_t kMaxProductUidLength = 127;
  constexpr uint8_t kConfiguredFlag = 0xAA;

  constexpr uint16_t kEepromSerialNumFlagAddr = 129;
  constexpr uint16_t kEepromSerialNumAddr = 130;
  constexpr uint16_t kMaxSerialNumLength = 127;
  constexpr uint8_t kSerialNumConfiguredFlag = 0xBB;

  constexpr uint8_t kPumpCount = 6;
  constexpr uint16_t kEepromRetrofitFlagAddr = 258;
  constexpr uint8_t kRetrofitConfiguredFlag = 0xCC;
  constexpr uint16_t kEepromDataIntervalSecAddr = 259; // 4 bytes
  constexpr uint16_t kEepromSensorInitSecAddr = 263;   // 2 bytes
  constexpr uint16_t kEepromEmaSampleAddr = 265;       // 2 bytes
  constexpr uint16_t kEepromBleModeAddr = 267;         // 1 byte
  constexpr uint16_t kEepromPumpThresholdsAddr = 268;  // 24 bytes: 6 * (high, low) * 2

  char gProductUid[kMaxProductUidLength + 1] = {0};
  char gSerialNumber[kMaxSerialNumLength + 1] = {0};

  // gSamplePeriodMs is mutable at runtime via set_sample_period; not persisted,
  // so it resets to the default on power-cycle.
  constexpr unsigned long kDefaultSamplePeriodMs = 60000;  // 1 minute
  constexpr unsigned long kMinSamplePeriodMs = 1000;       // floor: avoid flooding the link
  constexpr unsigned long kMaxSamplePeriodMs = 86400000UL; // 24h ceiling
  unsigned long gSamplePeriodMs = kDefaultSamplePeriodMs;
  unsigned long lastSampleMs = 0;

  // Retrofit config below is persisted to EEPROM and mutable via the flat
  // `{"set_...":"..."}` commands in tryHandleRetrofitCommand.
  constexpr unsigned long kDefaultDataIntervalSec = kDefaultSamplePeriodMs / 1000;
  constexpr unsigned long kMinDataIntervalSec = 1;
  constexpr unsigned long kMaxDataIntervalSec = kMaxSamplePeriodMs / 1000;
  constexpr uint16_t kDefaultSensorInitSec = 20;
  constexpr uint16_t kMaxSensorInitSec = 3600;
  constexpr uint16_t kDefaultEmaSampleN = 200;
  constexpr uint16_t kMinEmaSampleN = 1;
  constexpr uint16_t kMaxEmaSampleN = 5000;
  constexpr uint16_t kAdcMaxValue = 4095;
  // Raw 0-100 pump setting as received over BLE; see effectivePumpHighThreshold()/
  // effectivePumpLowThreshold() for how this maps to the real trigger point.
  constexpr uint16_t kPumpThresholdMaxPercent = 100;

  unsigned long gDataIntervalSec = kDefaultDataIntervalSec;
  uint16_t gSensorInitSec = kDefaultSensorInitSec;
  uint16_t gEmaSampleN = kDefaultEmaSampleN;
  bool gBleSleepMode = false; // false = normal (continuous sync), true = sleep (on-demand)
  uint16_t gPumpHighThr[kPumpCount] = {0};
  uint16_t gPumpLowThr[kPumpCount] = {0};

  // Computed by updatePumpControl(); not persisted (resets to OFF on boot).
  // No GPIO/relay is driven -- reported to the PWA over BLE only.
  bool gPumpState[kPumpCount] = {false};

  // UTC epoch of each pump's most recent ON/OFF transition, for get_pump_states
  // read-back. 0 = no transition since boot (or time unknown at that moment).
  unsigned long gPumpLastChangeEpoch[kPumpCount] = {0};

  // Refreshed continuously by updateAdcFilter(), independent of gSamplePeriodMs
  // (which only controls how often a sample is reported/sent).
  int gRawAdcValue = 0;
  float gFilteredAdcValue = 0.0f;
  bool gAdcFilterInitialized = false;
  unsigned long lastEmaSampleMs = 0;
  constexpr unsigned long kEmaSamplePeriodMs = 250;

  void armNotecardAttn()
  {
    char attnCmd[200];
    snprintf(attnCmd, sizeof(attnCmd),
             "{\"req\":\"card.attn\",\"mode\":\"arm,files\",\"files\":[\"%s\"]}",
             kInboundNotefile);
    notecardUart.println(attnCmd);
    notecardUart.readStringUntil('\n'); // clear response
  }

  bool loadProductUidFromEeprom()
  {
    uint8_t flag = EEPROM.read(kEepromFlagAddr);
    if (flag != kConfiguredFlag)
    {
      dbgPrintln("-- No stored ProductUID (first-time setup) --");
      return false;
    }

    uint16_t addr = kEepromProductUidAddr;
    uint16_t len = 0;
    while (len < kMaxProductUidLength)
    {
      char c = EEPROM.read(addr + len);
      gProductUid[len] = c;
      if (c == '\0')
      {
        dbgPrint("Loaded ProductUID from EEPROM: ");
        dbgPrintln(gProductUid);
        return true;
      }
      len++;
    }

    dbgPrintln("ERROR: ProductUID in EEPROM is not null-terminated");
    return false;
  }

  void saveProductUidToEeprom(const char *uid)
  {
    if (uid == nullptr || uid[0] == '\0')
    {
      dbgPrintln("ERROR: Cannot save empty ProductUID");
      return;
    }

    size_t len = strlen(uid);
    if (len > kMaxProductUidLength)
    {
      dbgPrint("ERROR: ProductUID too long (max ");
      dbgPrint(kMaxProductUidLength);
      dbgPrintln(" chars)");
      return;
    }

    // Write string + null, then clear the tail (avoid stale chars if the new value is shorter).
    for (size_t i = 0; i <= len; i++)
    {
      EEPROM.write(kEepromProductUidAddr + i, uid[i]);
    }
    for (size_t i = len + 1; i <= kMaxProductUidLength; i++)
    {
      EEPROM.write(kEepromProductUidAddr + i, 0);
    }

    EEPROM.write(kEepromFlagAddr, kConfiguredFlag);

    strcpy(gProductUid, uid);
    dbgPrint("Saved ProductUID to EEPROM: ");
    dbgPrintln(gProductUid);
  }

  void clearProductUidEeprom()
  {
    EEPROM.write(kEepromFlagAddr, 0x00);
    memset(gProductUid, 0, sizeof(gProductUid));
    dbgPrintln("Cleared ProductUID from EEPROM");
  }

  bool loadSerialNumberFromEeprom()
  {
    uint8_t flag = EEPROM.read(kEepromSerialNumFlagAddr);
    if (flag != kSerialNumConfiguredFlag)
    {
      dbgPrintln("-- No stored SerialNumber --");
      return false;
    }

    uint16_t addr = kEepromSerialNumAddr;
    uint16_t len = 0;
    while (len < kMaxSerialNumLength)
    {
      char c = EEPROM.read(addr + len);
      gSerialNumber[len] = c;
      if (c == '\0')
      {
        dbgPrint("Loaded SerialNumber from EEPROM: ");
        dbgPrintln(gSerialNumber);
        return true;
      }
      len++;
    }

    dbgPrintln("ERROR: SerialNumber in EEPROM is not null-terminated");
    return false;
  }

  void saveSerialNumberToEeprom(const char *sn)
  {
    if (sn == nullptr || sn[0] == '\0')
    {
      dbgPrintln("ERROR: Cannot save empty SerialNumber");
      return;
    }

    size_t len = strlen(sn);
    if (len > kMaxSerialNumLength)
    {
      dbgPrint("ERROR: SerialNumber too long (max ");
      dbgPrint(kMaxSerialNumLength);
      dbgPrintln(" chars)");
      return;
    }

    // Write string + null, then clear the tail (avoid stale chars if the new value is shorter).
    for (size_t i = 0; i <= len; i++)
    {
      EEPROM.write(kEepromSerialNumAddr + i, sn[i]);
    }
    for (size_t i = len + 1; i <= kMaxSerialNumLength; i++)
    {
      EEPROM.write(kEepromSerialNumAddr + i, 0);
    }

    EEPROM.write(kEepromSerialNumFlagAddr, kSerialNumConfiguredFlag);

    strcpy(gSerialNumber, sn);
    dbgPrint("Saved SerialNumber to EEPROM: ");
    dbgPrintln(gSerialNumber);
  }

  void clearSerialNumberEeprom()
  {
    EEPROM.write(kEepromSerialNumFlagAddr, 0x00);
    memset(gSerialNumber, 0, sizeof(gSerialNumber));
    dbgPrintln("Cleared SerialNumber from EEPROM");
  }

  void clearAllConfigEeprom()
  {
    clearProductUidEeprom();
    clearSerialNumberEeprom();
    EEPROM.write(kEepromRetrofitFlagAddr, 0x00); // re-initializes with defaults on next boot
    dbgPrintln("Cleared ALL config from EEPROM");
  }

  // AT+NAME limit per the HM-10 datasheet (1-12 bytes).
  constexpr uint8_t kBleNameMaxLen = 12;

  // Builds a short per-device name from category + serial, e.g. "dewater-pump-float"
  // + "0001234567" -> "DPF-01234567" (serial truncated to its trailing chars if needed).
  void buildBleAdvertisedName(char *out, size_t outSize)
  {
    char prefix[4] = {0};
    uint8_t prefixLen = 0;
    bool atSegmentStart = true;
    for (const char *p = kDeviceCategoryId; *p != '\0' && prefixLen < 3; p++)
    {
      if (*p == '-')
      {
        atSegmentStart = true;
        continue;
      }
      if (atSegmentStart)
      {
        prefix[prefixLen++] = static_cast<char>(toupper(*p));
        atSegmentStart = false;
      }
    }
    prefix[prefixLen] = '\0';

    const size_t serialLen = strlen(gSerialNumber);
    const size_t maxSerialChars = outSize - 1 /* dash */ - prefixLen - 1 /* null */;
    const char *serialStart = (serialLen > maxSerialChars)
                                   ? (gSerialNumber + serialLen - maxSerialChars)
                                   : gSerialNumber;

    snprintf(out, outSize, "%s-%s", prefix, serialStart);
  }

  // Best-effort: no-op if the serial number isn't known yet. Must run before
  // any BLE central connects -- see README.md for why.
  void applyBleAdvertisedName()
  {
    if (gSerialNumber[0] == '\0')
    {
      return;
    }

    char name[kBleNameMaxLen + 1];
    buildBleAdvertisedName(name, sizeof(name));

    dbgPrint(">> Setting BLE advertised name to: ");
    dbgPrintln(name);

    bleUart.print("AT+NAME");
    bleUart.print(name);
    delay(200);
    dbgPrint(">> AT+NAME response: ");
    while (bleUart.available())
    {
      dbgPrint(static_cast<char>(bleUart.read()));
    }
    dbgPrintln();
  }

  void savePumpThresholdToEeprom(uint8_t pumpIdx, bool isHigh, uint16_t value)
  {
    const uint16_t addr = kEepromPumpThresholdsAddr + (pumpIdx * 4) + (isHigh ? 0 : 2);
    EEPROM.put(addr, value);
  }

  void saveRetrofitConfigToEeprom()
  {
    EEPROM.put(kEepromDataIntervalSecAddr, gDataIntervalSec);
    EEPROM.put(kEepromSensorInitSecAddr, gSensorInitSec);
    EEPROM.put(kEepromEmaSampleAddr, gEmaSampleN);
    EEPROM.write(kEepromBleModeAddr, gBleSleepMode ? 1 : 0);
    for (uint8_t i = 0; i < kPumpCount; i++)
    {
      savePumpThresholdToEeprom(i, true, gPumpHighThr[i]);
      savePumpThresholdToEeprom(i, false, gPumpLowThr[i]);
    }
    EEPROM.write(kEepromRetrofitFlagAddr, kRetrofitConfiguredFlag);
  }

  // Writes+loads defaults on first boot (flag byte not yet set).
  void loadRetrofitConfigFromEeprom()
  {
    if (EEPROM.read(kEepromRetrofitFlagAddr) != kRetrofitConfiguredFlag)
    {
      dbgPrintln("-- No stored retrofit config, writing defaults --");
      saveRetrofitConfigToEeprom();
      return;
    }

    EEPROM.get(kEepromDataIntervalSecAddr, gDataIntervalSec);
    EEPROM.get(kEepromSensorInitSecAddr, gSensorInitSec);
    EEPROM.get(kEepromEmaSampleAddr, gEmaSampleN);
    gBleSleepMode = (EEPROM.read(kEepromBleModeAddr) == 1);
    for (uint8_t i = 0; i < kPumpCount; i++)
    {
      const uint16_t addr = kEepromPumpThresholdsAddr + (i * 4);
      EEPROM.get(addr, gPumpHighThr[i]);
      EEPROM.get(addr + 2, gPumpLowThr[i]);
    }
    dbgPrintln("Loaded retrofit config from EEPROM");
  }

  void sendGetConfigJsonTo(Print &port)
  {
    port.print("{\"status\":\"ok\",\"category\":\"");
    port.print(kDeviceCategoryId);
    port.print("\",\"product_uid\":\"");
    port.print(gProductUid);
    port.print("\",\"serial_number\":\"");
    port.print(gSerialNumber);
    port.print("\",\"sample_period_ms\":");
    port.print(gSamplePeriodMs);
    port.println("}");
  }

  // tryHandle*Command functions share a contract: return true (and reply on
  // `reply`) if `line` matched, else false so the next handler can try.

  bool tryHandleGetConfigLine(const String &line, Print &replyPort)
  {
    if (line.indexOf("\"cmd\":\"get_config\"") == -1)
    {
      return false;
    }
    sendGetConfigJsonTo(replyPort);
    return true;
  }

  bool extractJsonStringValue(const String &line, const char *keyWithQuotes, String &out);
  bool extractJsonNumberValue(const String &line, const char *keyWithColon, unsigned long &out);
  bool extractJsonRawValue(const String &line, const char *keyWithColon, String &out);

  void sendStatusJsonTo(Print &port)
  {
    // Integer millivolt arithmetic to avoid float.
    const int adc = analogRead(kAdcPin);
    const int adcMv = ((long)adc * 3300) / 4095;
    char adcVoltStr[12];
    snprintf(adcVoltStr, sizeof(adcVoltStr), "%d.%03d", adcMv / 1000, adcMv % 1000);

    notecardUart.println("{\"req\":\"card.voltage\"}");
    const String voltResp = notecardUart.readStringUntil('\n');
    String supplyV;
    if (!extractJsonRawValue(voltResp, "\"value\":", supplyV))
    {
      supplyV = "null";
    }

    notecardUart.println("{\"req\":\"card.location\"}");
    const String locResp = notecardUart.readStringUntil('\n');
    String lat, lon;
    const bool hasLoc = extractJsonRawValue(locResp, "\"lat\":", lat) &&
                        extractJsonRawValue(locResp, "\"lon\":", lon);

    port.print("{\"status\":\"ok\",\"version\":\"");
    port.print(kFirmwareVersion);
    port.print("\",\"category\":\"");
    port.print(kDeviceCategoryId);
    port.print("\",\"product_uid\":\"");
    port.print(gProductUid);
    port.print("\",\"serial_number\":\"");
    port.print(gSerialNumber);
    port.print("\",\"sample_period_ms\":");
    port.print(gSamplePeriodMs);
    port.print(",\"adc\":");
    port.print(adc);
    port.print(",\"adc_voltage_v\":");
    port.print(adcVoltStr);
    port.print(",\"supply_voltage_v\":");
    port.print(supplyV);
    port.print(",\"lat\":");
    port.print(hasLoc ? lat : String("null"));
    port.print(",\"lon\":");
    port.print(hasLoc ? lon : String("null"));
    port.println("}");
  }

  bool tryHandleGetStatusCommand(const String &line, Print &reply)
  {
    if (line.indexOf("\"cmd\":\"get_status\"") == -1)
    {
      return false;
    }
    sendStatusJsonTo(reply);
    return true;
  }

  bool tryHandleGetPumpStatesCommand(const String &line, Print &reply);

  /** Parses `"key":"value"` — value must not contain escaped quotes. */
  bool extractJsonStringValue(const String &line, const char *keyWithQuotes, String &out)
  {
    out = "";
    const int p = line.indexOf(keyWithQuotes);
    if (p < 0)
    {
      return false;
    }
    const int start = p + static_cast<int>(strlen(keyWithQuotes));
    const int end = line.indexOf('"', start);
    if (end <= start)
    {
      return false;
    }
    out = line.substring(start, end);
    return true;
  }

  /** Parses `"key":123` (unquoted). Pass the key with trailing colon, e.g. `"period_ms":`. */
  bool extractJsonNumberValue(const String &line, const char *keyWithColon, unsigned long &out)
  {
    int start = line.indexOf(keyWithColon);
    if (start < 0)
    {
      return false;
    }
    start += static_cast<int>(strlen(keyWithColon));
    while (start < line.length() && line[start] == ' ')
    {
      start++;
    }
    int end = start;
    while (end < line.length() && isDigit(line[end]))
    {
      end++;
    }
    if (end == start)
    {
      return false;
    }
    out = strtoul(line.substring(start, end).c_str(), nullptr, 10);
    return true;
  }

  /** Parses `"key":3.14` or `"key":-70.87`, keeping sign/decimal point as raw text. */
  bool extractJsonRawValue(const String &line, const char *keyWithColon, String &out)
  {
    out = "";
    int start = line.indexOf(keyWithColon);
    if (start < 0)
    {
      return false;
    }
    start += static_cast<int>(strlen(keyWithColon));
    while (start < static_cast<int>(line.length()) && line[start] == ' ')
    {
      start++;
    }
    int end = start;
    if (end < static_cast<int>(line.length()) && line[end] == '-')
    {
      end++;
    }
    while (end < static_cast<int>(line.length()) && (isDigit(line[end]) || line[end] == '.'))
    {
      end++;
    }
    if (end == start)
    {
      return false;
    }
    out = line.substring(start, end);
    return true;
  }

  void pushNotecardHubIdentityAndSync()
  {
    notecardUart.println((String("{\"req\":\"hub.set\",\"product\":\"") + gProductUid + "\",\"sn\":\"" + gSerialNumber + "\"}").c_str());
    delay(1000);
    notecardUart.println("{\"req\":\"hub.set\",\"mode\":\"continuous\",\"sync\":true}");
    delay(2000);
    dbgPrintln("Performing hub.sync after identity update...");
    notecardUart.println("{\"req\":\"hub.sync\"}");
    notecardUart.readStringUntil('\n');
    delay(1500);
  }

  // Shared tail for a successful UID/serial update: reply, push identity to the
  // Notecard, sync, then reset so the new identity takes effect from boot.
  void finishIdentityUpdateAndReset(Print &reply, const char *dbgMsg)
  {
    reply.println(
        "{\"status\":\"ok\",\"msg\":\"Identity saved. Syncing Notehub and restarting...\"}");
    dbgPrintln(dbgMsg);
    pushNotecardHubIdentityAndSync();
    delay(500);
    NVIC_SystemReset();
  }

  // Recognizes setup_device (lenient match) or set_config; pushes identity to
  // the Notecard and resets the MCU. Ignored during setupPhaseReceiveDeviceConfig.
  bool tryHandleRuntimeIdentityCommand(const String &line, Print &reply)
  {
    const bool cmdGetCfg = line.indexOf("\"cmd\":\"get_config\"") >= 0;
    if (cmdGetCfg)
    {
      return false;
    }

    const bool wantsSetup =
        line.indexOf("setup_device") >= 0 || line.indexOf("\"cmd\":\"setup_device\"") >= 0;
    const bool wantsSetCfg = line.indexOf("\"cmd\":\"set_config\"") >= 0;
    if (!wantsSetup && !wantsSetCfg)
    {
      return false;
    }

    String uid;
    String sn;
    const bool hasUid = extractJsonStringValue(line, "\"product_uid\":\"", uid);
    const bool hasSn = extractJsonStringValue(line, "\"serial_number\":\"", sn);

    if (wantsSetup)
    {
      if (!hasUid || !hasSn || uid.length() == 0 || sn.length() == 0)
      {
        reply.println("{\"status\":\"error\",\"msg\":\"setup_device requires product_uid and serial_number\"}");
        return true;
      }
      if (uid.length() > kMaxProductUidLength || sn.length() > kMaxSerialNumLength)
      {
        reply.println("{\"status\":\"error\",\"msg\":\"UID or serial too long\"}");
        return true;
      }
      saveProductUidToEeprom(uid.c_str());
      saveSerialNumberToEeprom(sn.c_str());
    }
    else
    {
      // set_config allows a partial update (uid only, sn only, or both).
      bool any = false;
      if (hasUid && uid.length() > 0 && uid.length() <= kMaxProductUidLength)
      {
        saveProductUidToEeprom(uid.c_str());
        any = true;
      }
      if (hasSn && sn.length() > 0 && sn.length() <= kMaxSerialNumLength)
      {
        saveSerialNumberToEeprom(sn.c_str());
        any = true;
      }
      if (!any)
      {
        reply.println("{\"status\":\"error\",\"msg\":\"No valid product_uid or serial_number in set_config\"}");
        return true;
      }
    }

    finishIdentityUpdateAndReset(reply, ">> Runtime identity update: pushing hub.set + sync, then reset");
    return true;
  }

  // {"cmd":"set_sample_period","period_ms":N} — applies immediately, not persisted to EEPROM.
  bool tryHandleSetSamplePeriodCommand(const String &line, Print &reply)
  {
    if (line.indexOf("\"cmd\":\"set_sample_period\"") == -1)
    {
      return false;
    }

    unsigned long periodMs = 0;
    if (!extractJsonNumberValue(line, "\"period_ms\":", periodMs) ||
        periodMs < kMinSamplePeriodMs || periodMs > kMaxSamplePeriodMs)
    {
      char err[160];
      snprintf(err, sizeof(err),
               "{\"status\":\"error\",\"msg\":\"period_ms must be between %lu and %lu\"}",
               kMinSamplePeriodMs, kMaxSamplePeriodMs);
      reply.println(err);
      return true;
    }

    gSamplePeriodMs = periodMs;
    dbgPrint(">> Sample period updated to (ms): ");
    dbgPrintln(gSamplePeriodMs);

    char ok[96];
    snprintf(ok, sizeof(ok),
             "{\"status\":\"ok\",\"msg\":\"Sample period updated\",\"period_ms\":%lu}",
             gSamplePeriodMs);
    reply.println(ok);
    return true;
  }

  // {"cmd":"echo","payload":...} or bare "echo <text>" — round-trip test for the BLE link.
  bool tryHandleEchoCommand(const String &line, Print &reply)
  {
    const bool isJsonEcho = line.indexOf("\"cmd\":\"echo\"") >= 0;
    const bool isBareEcho = line.startsWith("echo ");
    if (!isJsonEcho && !isBareEcho)
    {
      return false;
    }

    if (isBareEcho)
    {
      String payload = line.substring(5);
      reply.println(String("{\"status\":\"ok\",\"echo\":\"") + payload + "\"}");
      return true;
    }

    String strPayload;
    if (extractJsonStringValue(line, "\"payload\":\"", strPayload))
    {
      reply.println(String("{\"status\":\"ok\",\"echo\":\"") + strPayload + "\"}");
      return true;
    }

    String rawPayload;
    if (extractJsonRawValue(line, "\"payload\":", rawPayload))
    {
      reply.println(String("{\"status\":\"ok\",\"echo\":") + rawPayload + "}");
      return true;
    }

    reply.println("{\"status\":\"ok\",\"msg\":\"echo received\"}");
    return true;
  }

  // Below: flat `{"<name>":"<value>"}` retrofit commands (values always quoted),
  // distinct from the `{"cmd":"..."}` commands above.

  // normal = continuous Notecard sync, sleep = on-demand ("minimum") sync.
  void applyBleModeToNotecard()
  {
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "{\"req\":\"hub.set\",\"mode\":\"%s\"}", gBleSleepMode ? "minimum" : "continuous");
    notecardUart.println(cmd);
    notecardUart.readStringUntil('\n');
  }

  // Non-blocking: refreshes at most once per kEmaSamplePeriodMs.
  void updateAdcFilter()
  {
    const unsigned long nowMs = millis();
    if (gAdcFilterInitialized && (nowMs - lastEmaSampleMs < kEmaSamplePeriodMs))
    {
      return;
    }
    lastEmaSampleMs = nowMs;

    gRawAdcValue = analogRead(kAdcPin);
    if (!gAdcFilterInitialized)
    {
      gFilteredAdcValue = static_cast<float>(gRawAdcValue);
      gAdcFilterInitialized = true;
      return;
    }
    // Standard EMA: alpha = 2 / (N + 1), N = gEmaSampleN (configured smoothing level).
    const float alpha = 2.0f / (static_cast<float>(gEmaSampleN) + 1.0f);
    gFilteredAdcValue += alpha * (static_cast<float>(gRawAdcValue) - gFilteredAdcValue);
  }

  // Uses the EMA-filtered reading, not raw ADC, to avoid relay chatter from sensor noise.
  uint8_t computeCurrentWaterLevelPercent()
  {
    float v = gFilteredAdcValue;
    if (v < 0.0f)
    {
      v = 0.0f;
    }
    else if (v > static_cast<float>(kAdcMaxValue))
    {
      v = static_cast<float>(kAdcMaxValue);
    }
    return static_cast<uint8_t>((v * 100.0f / static_cast<float>(kAdcMaxValue)) + 0.5f);
  }

  // Maps the raw 0-100 setting to its real current_water_level trigger point
  // (High -> upper half 50-100%, Low -> lower half 0-50%) -- see README.md
  // "Pump ON/OFF control" for why.
  uint8_t effectivePumpHighThreshold(uint8_t pumpIdx)
  {
    return static_cast<uint8_t>(50 + gPumpHighThr[pumpIdx] / 2);
  }

  uint8_t effectivePumpLowThreshold(uint8_t pumpIdx)
  {
    return static_cast<uint8_t>(gPumpLowThr[pumpIdx] / 2);
  }

  // Queries the Notecard's real-time clock (set from cell tower / GPS on the
  // first successful sync). Returns UTC seconds since the Unix epoch, or 0 if
  // the Notecard has not yet obtained the time -- before that its response is
  // {"err":"time is not yet set ..."} with no "time" field.
  unsigned long fetchNotecardEpochTime()
  {
    notecardUart.println("{\"req\":\"card.time\"}");
    const String resp = notecardUart.readStringUntil('\n');
    unsigned long epoch = 0;
    if (!extractJsonNumberValue(resp, "\"time\":", epoch))
    {
      return 0;
    }
    return epoch;
  }

  // Writes epoch as a JSON numeric token, or "null" when it is 0 (unknown).
  void epochToJsonToken(unsigned long epoch, char *out, size_t outSize)
  {
    if (epoch != 0)
    {
      snprintf(out, outSize, "%lu", epoch);
    }
    else
    {
      strncpy(out, "null", outSize);
      out[outSize - 1] = '\0';
    }
  }

  // Appends " (UTC YYYY-MM-DD HH:MM:SS)" to the serial log for a non-zero epoch;
  // a no-op when the Notecard has no time yet, so the log line just omits it.
  void dbgPrintUtcSuffix(unsigned long epoch)
  {
    if (epoch == 0)
    {
      return;
    }
    const time_t t = static_cast<time_t>(epoch);
    struct tm tmUtc;
    gmtime_r(&t, &tmUtc);
    char human[24];
    strftime(human, sizeof(human), "%Y-%m-%d %H:%M:%S", &tmUtc);
    dbgPrint(" (UTC ");
    dbgPrint(human);
    dbgPrint(")");
  }

  // Pushes a pump's new ON/OFF state to the PWA over BLE and logs it. Caller
  // has already updated gPumpState[pumpIdx] and fetched the UTC epoch once for
  // the cycle (0 = time not yet available from the Notecard, emitted as null).
  void sendPumpStateChange(uint8_t pumpIdx, uint8_t pct, unsigned long epoch, Print &blePort)
  {
    gPumpLastChangeEpoch[pumpIdx] = epoch;

    char timeTok[16];
    epochToJsonToken(epoch, timeTok, sizeof(timeTok));

    char msg[96];
    snprintf(msg, sizeof(msg),
             "{\"pump_%u_state\":\"%s\",\"current_water_level\":%u,\"time\":%s}",
             pumpIdx + 1, gPumpState[pumpIdx] ? "on" : "off", pct, timeTok);
    blePort.println(msg);

    dbgPrint(">> Pump state changed: ");
    dbgPrint(msg);
    dbgPrintUtcSuffix(epoch);
    dbgPrintln();
  }

  // Hysteresis: ON above the high threshold, OFF below the low threshold.
  // Edge-triggered -- a BLE push only fires on an actual state change.
  void updatePumpControl(Print &blePort)
  {
    const uint8_t pct = computeCurrentWaterLevelPercent();
    // Fetched lazily on the first change this cycle so an idle sample costs no
    // Notecard round-trip; reused for every pump that flips at the same level.
    unsigned long epoch = 0;
    bool epochFetched = false;
    for (uint8_t i = 0; i < kPumpCount; i++)
    {
      const uint8_t realHigh = effectivePumpHighThreshold(i);
      const uint8_t realLow = effectivePumpLowThreshold(i);

      bool changed = false;
      if (!gPumpState[i] && pct > realHigh)
      {
        gPumpState[i] = true;
        changed = true;
      }
      else if (gPumpState[i] && pct < realLow)
      {
        gPumpState[i] = false;
        changed = true;
      }

      if (changed)
      {
        if (!epochFetched)
        {
          epoch = fetchNotecardEpochTime();
          epochFetched = true;
        }
        sendPumpStateChange(i, pct, epoch, blePort);
      }
    }
  }

  // On-demand read-back of all 6 pumps' current ON/OFF state, for a PWA that
  // just (re)connected and would otherwise have to wait for the next
  // hysteresis transition -- see README.md "Pump ON/OFF control".
  bool tryHandleGetPumpStatesCommand(const String &line, Print &reply)
  {
    if (line.indexOf("\"cmd\":\"get_pump_states\"") == -1)
    {
      return false;
    }

    reply.print("{\"status\":\"ok\"");
    for (uint8_t i = 0; i < kPumpCount; i++)
    {
      reply.print(",\"pump_");
      reply.print(i + 1);
      reply.print("_state\":\"");
      reply.print(gPumpState[i] ? "on" : "off");
      reply.print("\"");

      // UTC epoch of this pump's last transition (null = none since boot).
      char sinceTok[16];
      epochToJsonToken(gPumpLastChangeEpoch[i], sinceTok, sizeof(sinceTok));
      reply.print(",\"pump_");
      reply.print(i + 1);
      reply.print("_since\":");
      reply.print(sinceTok);
    }
    reply.print(",\"current_water_level\":");
    reply.print(computeCurrentWaterLevelPercent());
    reply.println("}");
    return true;
  }

  // Tolerates stray spaces in flat command keys, e.g. `{" set_ble_mode ":"sleep"}`.
  String stripSpaces(const String &s)
  {
    String out;
    out.reserve(s.length());
    for (size_t i = 0; i < s.length(); i++)
    {
      if (s[i] != ' ')
      {
        out += s[i];
      }
    }
    return out;
  }

  bool flatKeyPresent(const String &compactLine, const String &key)
  {
    return compactLine.indexOf("\"" + key + "\":") >= 0;
  }

  bool extractFlatStringValue(const String &compactLine, const String &key, String &out)
  {
    return extractJsonStringValue(compactLine, ("\"" + key + "\":\"").c_str(), out);
  }

  bool extractFlatULongValue(const String &compactLine, const String &key, unsigned long &out)
  {
    String s;
    if (!extractFlatStringValue(compactLine, key, s) || s.length() == 0)
    {
      return false;
    }
    out = strtoul(s.c_str(), nullptr, 10);
    return true;
  }

  // Handles {"echo":"<field>"}; `field` has already been extracted from the command.
  bool tryHandleRetrofitEcho(const String &field, Print &reply)
  {
    char buf[160];

    if (field == "embedded_software_ver")
    {
      snprintf(buf, sizeof(buf), "{\"status\":\"ok\",\"embedded_software_ver\":\"%s\"}", kFirmwareVersion);
      reply.println(buf);
      return true;
    }
    if (field == "notecard_ver" || field == "uid")
    {
      // card.version's "version" is the Notecard firmware version;
      // its "device" field is the Notehub-assigned DeviceUID.
      notecardUart.println("{\"req\":\"card.version\"}");
      const String resp = notecardUart.readStringUntil('\n');
      String value;
      const bool ok = (field == "notecard_ver")
                          ? extractJsonStringValue(resp, "\"version\":\"", value)
                          : extractJsonStringValue(resp, "\"device\":\"", value);
      if (ok)
      {
        snprintf(buf, sizeof(buf), "{\"status\":\"ok\",\"%s\":\"%s\"}", field.c_str(), value.c_str());
      }
      else
      {
        snprintf(buf, sizeof(buf), "{\"status\":\"error\",\"msg\":\"No response from Notecard\"}");
      }
      reply.println(buf);
      return true;
    }
    if (field == "set_data_e_t_sec")
    {
      snprintf(buf, sizeof(buf), "{\"status\":\"ok\",\"set_data_e_t_sec\":%lu}", gDataIntervalSec);
      reply.println(buf);
      return true;
    }
    if (field == "set_sensor_init_t_sec")
    {
      snprintf(buf, sizeof(buf), "{\"status\":\"ok\",\"set_sensor_init_t_sec\":%u}", gSensorInitSec);
      reply.println(buf);
      return true;
    }
    if (field == "sample_rate")
    {
      snprintf(buf, sizeof(buf), "{\"status\":\"ok\",\"sample_rate\":%u}", gEmaSampleN);
      reply.println(buf);
      return true;
    }
    if (field == "ble_state")
    {
      snprintf(buf, sizeof(buf), "{\"status\":\"ok\",\"ble_state\":\"%s\"}", gBleSleepMode ? "sleep" : "normal");
      reply.println(buf);
      return true;
    }
    if (field == "sensor_adc_value")
    {
      updateAdcFilter();
      // Manual 2-decimal formatting (values always >= 0) avoids relying on %f support.
      char filteredStr[16];
      const int filteredWhole = static_cast<int>(gFilteredAdcValue);
      const int filteredFrac = static_cast<int>(gFilteredAdcValue * 100.0f) % 100;
      snprintf(filteredStr, sizeof(filteredStr), "%d.%02d", filteredWhole, filteredFrac);
      snprintf(buf, sizeof(buf), "{\"status\":\"ok\",\"raw\":%d,\"filtered\":%s}", gRawAdcValue, filteredStr);
      reply.println(buf);
      return true;
    }
    for (uint8_t i = 0; i < kPumpCount; i++)
    {
      char highKey[24];
      char lowKey[24];
      snprintf(highKey, sizeof(highKey), "pump_%u_high_thr", i + 1);
      snprintf(lowKey, sizeof(lowKey), "pump_%u_low_thr", i + 1);
      if (field == highKey)
      {
        snprintf(buf, sizeof(buf), "{\"status\":\"ok\",\"%s\":%u}", highKey, gPumpHighThr[i]);
        reply.println(buf);
        return true;
      }
      if (field == lowKey)
      {
        snprintf(buf, sizeof(buf), "{\"status\":\"ok\",\"%s\":%u}", lowKey, gPumpLowThr[i]);
        reply.println(buf);
        return true;
      }
    }

    return false;
  }

  // Handles flat retrofit config/echo commands, e.g. {"set_data_e_t_sec":"900"} or {"echo":"sample_rate"}.
  bool tryHandleRetrofitCommand(const String &line, Print &reply)
  {
    const String compact = stripSpaces(line);

    String echoField;
    if (extractFlatStringValue(compact, "echo", echoField))
    {
      if (!tryHandleRetrofitEcho(echoField, reply))
      {
        reply.println("{\"status\":\"error\",\"msg\":\"Unknown echo field\"}");
      }
      return true;
    }

    unsigned long val;
    char buf[128];

    if (extractFlatULongValue(compact, "set_data_e_t_sec", val))
    {
      if (val < kMinDataIntervalSec || val > kMaxDataIntervalSec)
      {
        snprintf(buf, sizeof(buf), "{\"status\":\"error\",\"msg\":\"set_data_e_t_sec must be between %lu and %lu\"}",
                 kMinDataIntervalSec, kMaxDataIntervalSec);
      }
      else
      {
        gDataIntervalSec = val;
        gSamplePeriodMs = gDataIntervalSec * 1000UL;
        EEPROM.put(kEepromDataIntervalSecAddr, gDataIntervalSec);
        snprintf(buf, sizeof(buf), "{\"status\":\"ok\",\"set_data_e_t_sec\":%lu}", gDataIntervalSec);
      }
      reply.println(buf);
      return true;
    }

    if (extractFlatULongValue(compact, "set_sensor_init_t_sec", val))
    {
      if (val > kMaxSensorInitSec)
      {
        snprintf(buf, sizeof(buf), "{\"status\":\"error\",\"msg\":\"set_sensor_init_t_sec must be 0-%u\"}", kMaxSensorInitSec);
      }
      else
      {
        gSensorInitSec = static_cast<uint16_t>(val);
        EEPROM.put(kEepromSensorInitSecAddr, gSensorInitSec);
        snprintf(buf, sizeof(buf), "{\"status\":\"ok\",\"set_sensor_init_t_sec\":%u}", gSensorInitSec);
      }
      reply.println(buf);
      return true;
    }

    if (extractFlatULongValue(compact, "set_sample", val))
    {
      if (val < kMinEmaSampleN || val > kMaxEmaSampleN)
      {
        snprintf(buf, sizeof(buf), "{\"status\":\"error\",\"msg\":\"set_sample must be between %u and %u\"}",
                 kMinEmaSampleN, kMaxEmaSampleN);
      }
      else
      {
        gEmaSampleN = static_cast<uint16_t>(val);
        EEPROM.put(kEepromEmaSampleAddr, gEmaSampleN);
        snprintf(buf, sizeof(buf), "{\"status\":\"ok\",\"set_sample\":%u}", gEmaSampleN);
      }
      reply.println(buf);
      return true;
    }

    String bleModeVal;
    if (extractFlatStringValue(compact, "set_ble_mode", bleModeVal))
    {
      if (bleModeVal == "normal" || bleModeVal == "sleep")
      {
        gBleSleepMode = (bleModeVal == "sleep");
        EEPROM.write(kEepromBleModeAddr, gBleSleepMode ? 1 : 0);
        applyBleModeToNotecard();
        snprintf(buf, sizeof(buf), "{\"status\":\"ok\",\"set_ble_mode\":\"%s\"}", bleModeVal.c_str());
      }
      else
      {
        snprintf(buf, sizeof(buf), "{\"status\":\"error\",\"msg\":\"set_ble_mode must be normal or sleep\"}");
      }
      reply.println(buf);
      return true;
    }

    if (flatKeyPresent(compact, "reset_ble"))
    {
      dbgPrintln(">> Received reset_ble command");
      // No hardware reset pin wired to the HM-10; AT+RESET may not apply while a BLE session is live.
      bleUart.print("AT+RESET");
      delay(500);
      bleUart.end();
      delay(100);
      bleUart.begin(9600);
      reply.println("{\"status\":\"ok\",\"msg\":\"BLE module reset\"}");
      return true;
    }

    for (uint8_t i = 0; i < kPumpCount; i++)
    {
      char highKey[24];
      char lowKey[24];
      snprintf(highKey, sizeof(highKey), "pump_%u_set_high", i + 1);
      snprintf(lowKey, sizeof(lowKey), "pump_%u_set_low", i + 1);

      if (extractFlatULongValue(compact, highKey, val))
      {
        if (val > kPumpThresholdMaxPercent)
        {
          snprintf(buf, sizeof(buf), "{\"status\":\"error\",\"msg\":\"%s must be 0-%u\"}", highKey, kPumpThresholdMaxPercent);
        }
        else
        {
          gPumpHighThr[i] = static_cast<uint16_t>(val);
          savePumpThresholdToEeprom(i, true, gPumpHighThr[i]);
          snprintf(buf, sizeof(buf), "{\"status\":\"ok\",\"%s\":%u}", highKey, gPumpHighThr[i]);

          // Operator "stop now": if the new start (high) level is now above the
          // current water level, a running pump shouldn't be running yet -- stop
          // it. This is deliberately edge-triggered on the command, not folded
          // into updatePumpControl(): a continuous "OFF while below high" rule
          // would erase the deadband between high/low and make the pump chatter.
          // Once stopped this way it restarts normally, only when the level next
          // climbs above this new (higher) start level.
          const uint8_t pct = computeCurrentWaterLevelPercent();
          if (gPumpState[i] && pct < effectivePumpHighThreshold(i))
          {
            gPumpState[i] = false;
            sendPumpStateChange(i, pct, fetchNotecardEpochTime(), reply);
          }
        }
        reply.println(buf);
        return true;
      }
      if (extractFlatULongValue(compact, lowKey, val))
      {
        if (val > kPumpThresholdMaxPercent)
        {
          snprintf(buf, sizeof(buf), "{\"status\":\"error\",\"msg\":\"%s must be 0-%u\"}", lowKey, kPumpThresholdMaxPercent);
        }
        else
        {
          gPumpLowThr[i] = static_cast<uint16_t>(val);
          savePumpThresholdToEeprom(i, false, gPumpLowThr[i]);
          snprintf(buf, sizeof(buf), "{\"status\":\"ok\",\"%s\":%u}", lowKey, gPumpLowThr[i]);
        }
        reply.println(buf);
        return true;
      }
    }

    return false;
  }

  // Flat identity update: {"set_product_uid":"..."} and/or {"set_serial_number":"..."}.
  // Mirrors set_config -- saves to EEPROM, pushes identity to the Notecard, then resets.
  bool tryHandleFlatIdentityCommand(const String &line, Print &reply)
  {
    const String compact = stripSpaces(line);

    String uid;
    String sn;
    const bool hasUid = extractFlatStringValue(compact, "set_product_uid", uid);
    const bool hasSn = extractFlatStringValue(compact, "set_serial_number", sn);
    if (!hasUid && !hasSn)
    {
      return false;
    }

    bool any = false;
    if (hasUid && uid.length() > 0 && uid.length() <= kMaxProductUidLength)
    {
      saveProductUidToEeprom(uid.c_str());
      any = true;
    }
    if (hasSn && sn.length() > 0 && sn.length() <= kMaxSerialNumLength)
    {
      saveSerialNumberToEeprom(sn.c_str());
      any = true;
    }
    if (!any)
    {
      reply.println("{\"status\":\"error\",\"msg\":\"No valid set_product_uid or set_serial_number\"}");
      return true;
    }

    finishIdentityUpdateAndReset(reply, ">> Flat identity update: pushing hub.set + sync, then reset");
    return true;
  }

  // Blocks until product UID + serial are stored and confirmed over BLE
  // (setup_device then confirm_setup). No timeout -- main loop / ADC don't
  // start until this returns.
  void setupPhaseReceiveDeviceConfig(bool uidAlreadyStored, bool snAlreadyStored)
  {
    bool uidReceived = uidAlreadyStored;
    bool snReceived = snAlreadyStored;
    bool confirmationReceived = false;

    dbgPrintln();
    dbgPrintln("========================================");
    dbgPrintln("  SETUP PHASE: Device Configuration");
    dbgPrintln("========================================");
    dbgPrintln();
    dbgPrintln("STEP 1: Send setup command from the PWA over BLE:");
    dbgPrintln(R"({"cmd":"setup_device","product_uid":"YOUR_UID","serial_number":"YOUR_SN"})");
    dbgPrintln();
    dbgPrintln("STEP 2: After receiving values, send confirmation:");
    dbgPrintln(R"({"cmd":"confirm_setup"})");
    dbgPrintln();
    dbgPrintln("Optional: Send {\"cmd\":\"get_config\"} to read current values.");
    dbgPrintln();

    while (!confirmationReceived)
    {
      if (bleUart.available())
      {
        String line = bleUart.readStringUntil('\n');
        line.trim();
        if (line.length() > 0)
        {
          dbgPrint(">> BLE: ");
          dbgPrintln(line);

          if (tryHandleGetConfigLine(line, bleUart))
          {
            // handled
          }
          else if (line.indexOf("\"cmd\":\"confirm_setup\"") != -1)
          {
            if (uidReceived && snReceived)
            {
              confirmationReceived = true;
              bleUart.println("{\"status\":\"ok\",\"msg\":\"Setup confirmed. Device booting...\"}");
              dbgPrintln("Setup confirmed by PWA!");
            }
            else
            {
              bleUart.println("{\"status\":\"error\",\"msg\":\"ProductUID and SerialNumber must be set first\"}");
              dbgPrintln("Cannot confirm: missing ProductUID or SerialNumber");
            }
          }
          else
          {
            String uid;
            String sn;
            if (!uidReceived && extractJsonStringValue(line, "\"product_uid\":\"", uid))
            {
              if (uid.length() > 0 && uid.length() <= kMaxProductUidLength)
              {
                saveProductUidToEeprom(uid.c_str());
                uidReceived = true;
                dbgPrintln("ProductUID received and stored");
              }
            }
            if (!snReceived && extractJsonStringValue(line, "\"serial_number\":\"", sn))
            {
              if (sn.length() > 0 && sn.length() <= kMaxSerialNumLength)
              {
                saveSerialNumberToEeprom(sn.c_str());
                snReceived = true;
                dbgPrintln("SerialNumber received and stored");
              }
            }
          }
        }
      }

      delay(50);
    }
  }

} // namespace

// ===================================================================
//  SETUP
// ===================================================================
void setup()
{
  Serial.begin(9600);       // debug output only, not a command channel
  notecardUart.begin(9600);
  bleUart.begin(9600);      // HM-10 default baud

  notecardUart.setTimeout(5000);
  bleUart.setTimeout(1000);

  pinMode(kAdcPin, INPUT_ANALOG);
  analogReadResolution(12);
  pinMode(kNotecardAttnPin, INPUT);

  delay(3000);

  if (kClearEepromOnBoot)
  {
    dbgPrintln("WARNING: Clearing EEPROM (kClearEepromOnBoot = 1)");
    clearAllConfigEeprom();
  }

  bool uidLoaded = loadProductUidFromEeprom();
  bool snLoaded = loadSerialNumberFromEeprom();

  if (uidLoaded && snLoaded)
  {
    // Already-configured device rebooting: apply the name now, before the PWA
    // (or anything else) has a chance to connect.
    applyBleAdvertisedName();
  }

  if (!uidLoaded || !snLoaded)
  {
    setupPhaseReceiveDeviceConfig(uidLoaded, snLoaded);
    // The PWA's setup session may still be connected here, so this can be a
    // no-op -- it will reliably apply on the device's next boot regardless.
    applyBleAdvertisedName();
  }

  loadRetrofitConfigFromEeprom();
  gSamplePeriodMs = gDataIntervalSec * 1000UL;

  notecardUart.println((String("{\"req\":\"hub.set\",\"product\":\"") + gProductUid + "\",\"sn\":\"" + gSerialNumber + "\"}").c_str());
  delay(1000);
  {
    char modeCmd[80];
    snprintf(modeCmd, sizeof(modeCmd), "{\"req\":\"hub.set\",\"mode\":\"%s\",\"sync\":true}",
             gBleSleepMode ? "minimum" : "continuous");
    notecardUart.println(modeCmd);
  }
  delay(2000);

  // Sync immediately to register the device on the Notehub dashboard.
  dbgPrintln("Performing initial hub.sync to register device...");
  notecardUart.println("{\"req\":\"hub.sync\"}");
  notecardUart.readStringUntil('\n');
  delay(3000);

  dbgPrint("Notecard configured - ProductUID: ");
  dbgPrint(gProductUid);
  dbgPrint(" | SerialNumber: ");
  dbgPrintln(gSerialNumber);
  delay(5000);

  armNotecardAttn();

  dbgPrintln("===Starting main loop===");
  delay(3000);
}

// ===================================================================
//  MAIN LOOP
// ===================================================================
void loop()
{
  const unsigned long nowMs = millis();

  // 0) Refresh ADC filter + pump hysteresis; pushes a BLE update on any state change.
  updateAdcFilter();
  updatePumpControl(bleUart);

  // 1) Inbound Notecard note (ATTN pin HIGH): sync, fetch + delete one note, re-arm.
  if (digitalRead(kNotecardAttnPin) == HIGH)
  {
    dbgPrintln();
    dbgPrintln("-- Notecard ATTN HIGH: polling inbound note --");

    notecardUart.println("{\"req\":\"hub.sync\"}");
    notecardUart.readStringUntil('\n'); // clear immediate {}
    delay(5000);

    while (notecardUart.available())
    {
      notecardUart.read();
    }

    char getNoteCmd[300];
    snprintf(getNoteCmd, sizeof(getNoteCmd),
             "{\"req\":\"note.get\",\"file\":\"%s\",\"delete\":true}",
             kInboundNotefile);
    notecardUart.println(getNoteCmd);

    const String noteContent = notecardUart.readStringUntil('\n');
    dbgPrint(">> Note Received: ");
    dbgPrintln(noteContent);

    dbgPrintln("Re-arming Notecard ATTN...");
    armNotecardAttn();
    delay(3000);
  }

  // 2) Inbound BLE commands from the PWA.
  if (bleUart.available())
  {
    String line = bleUart.readStringUntil('\n');
    line.trim();
    if (line.length() > 0)
    {
      dbgPrint(">> BLE RX: ");
      dbgPrintln(line);

      if (tryHandleGetConfigLine(line, bleUart))
      {
        // handled
      }
      else if (tryHandleGetStatusCommand(line, bleUart))
      {
        // handled
      }
      else if (tryHandleGetPumpStatesCommand(line, bleUart))
      {
        // handled
      }
      else if (tryHandleRuntimeIdentityCommand(line, bleUart))
      {
        // handled (may reset MCU)
      }
      else if (tryHandleFlatIdentityCommand(line, bleUart))
      {
        // handled (may reset MCU)
      }
      else if (tryHandleSetSamplePeriodCommand(line, bleUart))
      {
        // handled
      }
      else if (tryHandleEchoCommand(line, bleUart))
      {
        // handled
      }
      else if (tryHandleRetrofitCommand(line, bleUart))
      {
        // handled
      }
      else if (line.indexOf("\"cmd\":\"reset_config\"") != -1)
      {
        dbgPrintln(">> Received reset_config command");
        clearProductUidEeprom();
        clearSerialNumberEeprom();
        bleUart.println("{\"status\":\"ok\",\"msg\":\"Config cleared. Restarting for reconfiguration...\"}");
        delay(1000);
        NVIC_SystemReset();
      }
      else
      {
        dbgPrintln(">> Unrecognized command");
      }
    }
  }

  // 3) Periodic water-level report -> BLE (to the PWA) + Notecard (cloud).
  if (nowMs - lastSampleMs >= gSamplePeriodMs)
  {
    lastSampleMs = nowMs;

    const uint8_t waterLevelPct = computeCurrentWaterLevelPercent();
    const unsigned long epoch = fetchNotecardEpochTime();

    dbgPrint("Current water level (%): ");
    dbgPrint(waterLevelPct);
    dbgPrintUtcSuffix(epoch);
    dbgPrintln();

    char timeTok[16];
    epochToJsonToken(epoch, timeTok, sizeof(timeTok));
    char levelMsg[64];
    snprintf(levelMsg, sizeof(levelMsg),
             "{\"current_water_level\":%u,\"time\":%s}", waterLevelPct, timeTok);
    bleUart.println(levelMsg);

    char addNoteCmd[220];
    snprintf(addNoteCmd, sizeof(addNoteCmd),
             "{\"req\":\"note.add\",\"file\":\"%s\",\"sync\":true,\"body\":{\"current_water_level\":%u}}",
             kOutboundNotefile, waterLevelPct);
    notecardUart.println(addNoteCmd);
    const String addResp = notecardUart.readStringUntil('\n');
    if (addResp.indexOf("\"err\"") >= 0)
    {
      dbgPrint("Warning: Note add response: ");
      dbgPrintln(addResp);
    }
  }
}
