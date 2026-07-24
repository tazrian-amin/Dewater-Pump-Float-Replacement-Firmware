# Dewater Pump Float Replacement Firmware

Firmware for a [Blues Wireless Swan R5](https://blues.com/products/swan/) (STM32,
Arduino framework) that replaces a mechanical float switch with an ADC-based
water level sensor, for controlling a dewater pump. It reports data both
locally to a companion Progressive Web App (PWA) over BLE, and to the cloud via
a Blues Notecard.

## Architecture

Three communication channels, each with a distinct role:

| Channel | Hardware | Role |
|---|---|---|
| `Serial` | USB | Debug log output only — never a command channel |
| `bleUart` | HM-10 module (A0/A3) | **Sole control channel** for the PWA — all commands below |
| `notecardUart` | `Serial1` (F_TX/F_RX, ATTN on D5) | Blues Notecard JSON API — relays data to Notehub |

**Wiring:**
- `notecardUart` (Serial1): `F_TX`→`N_RX`, `F_RX`→`N_TX`, `F_D5`→`N_ATTN`
- `bleUart` (HM-10): `A0`←`TXD`, `A3`→`RXD`, `3V3`→`VCC`, `GND`→`GND`
- Don't wire a USB-serial adapter to A0/A3 while the HM-10 is connected — they share pins.

**Notefiles:** `data.qi` = inbound (`note.get`), `retrofit.qo` = outbound (`note.add`).

## Boot flow

1. On every boot, the ADC pin, Notecard ATTN pin, and both UARTs are initialized.
2. If `kClearEepromOnBoot` is `1` (currently set for testing/reflashing — see
   `src/main.cpp`), all EEPROM config is wiped.
3. If product UID and serial number are already stored in EEPROM, the device
   applies its BLE advertised name and proceeds directly to the main loop.
4. **First-time setup** (no UID/serial stored): the device blocks in a setup
   phase until the PWA sends `setup_device` followed by `confirm_setup` over
   BLE (see [Commands](#commands) below). `get_config` is answered during this
   wait so the app can poll current values. There is no timeout — the main
   loop and ADC sampling do not start until setup is confirmed.
5. Retrofit config (data interval, sensor warm-up, EMA smoothing, BLE mode,
   pump thresholds) is loaded from EEPROM (defaults are written on first run).
6. Identity is pushed to the Notecard (`hub.set`) and an initial `hub.sync` is
   performed to register the device on Notehub.

## BLE advertised name

On boot (once the serial number is known), the device sets a short per-unit
BLE advertised name via `AT+NAME`, e.g. `DPF-01234567` (category prefix +
trailing serial digits), so the PWA's scan list can tell devices apart
instead of every unit showing the module default.

- `kDeviceCategoryId` (`dewater-pump-float`) must match this device's id in
  the app's `categories.ts`.
- Many "HM-10" clones silently ignore `AT+NAME` and keep advertising as
  `HMSoft` regardless — the app instead confirms category via
  `get_config`/`get_status`, so this is a harmless no-op on those units.
- `AT+NAME` must be sent before any BLE central connects — once connected,
  the HM-10 treats UART bytes as pass-through data rather than AT commands.

## Main loop

Each iteration of `loop()`:

0. **ADC filter refresh + pump hysteresis** — refreshes the EMA-filtered ADC
   reading (every 250 ms) and re-evaluates each pump's ON/OFF state against
   its high/low thresholds, pushing a BLE update immediately on any change
   (see [Pump ON/OFF control](#pump-onoff-control) below).
1. **Notecard ATTN check** — if the ATTN pin is HIGH, syncs, fetches + deletes
   one inbound note from `data.qi`, then re-arms ATTN.
2. **BLE command processing** — reads one line from the PWA and dispatches it
   through the command handlers below (first match wins).
3. **Periodic water-level report** — every `sample_period_ms`, computes
   `current_water_level` (0–100%, from the filtered ADC — see below) and
   sends it both over BLE (`{"current_water_level":<pct>}`) and to the
   Notecard (`note.add` on `retrofit.qo`, same field).

The ADC itself is continuously filtered via an EMA (every 250 ms), independent
of the reporting interval, so pump hysteresis and diagnostic reads
(`echo`/`sensor_adc_value`) always see a fresh, smoothed reading rather than a
noisy instantaneous one.

## Water level percentage

`current_water_level` is the EMA-filtered ADC reading (not a fresh raw
`analogRead`, to avoid pump chatter from sensor noise) scaled from the 12-bit
ADC range (0–4095) to a rounded percentage (0–100). It's the value sent in
the periodic BLE/Notecard report and compared against each pump's
thresholds. Raw ADC counts are still available on request via `get_status`
(`adc`) and the flat `echo`/`sensor_adc_value` field, for calibration/debug.

## Pump ON/OFF control

Each of the 6 pumps has a high and low setting (0–100, default 0 until
configured via `pump_N_set_high`/`pump_N_set_low`) and a logical ON/OFF
state, using hysteresis.

**The raw 0–100 setting is not the real trigger point.** The PWA's slider UI
gives the High control only the upper half of the water-level range and the
Low control only the lower half, so each setting is mapped to a real
`current_water_level` trigger before use:

| Setting | Real trigger |
|---|---|
| `real_high = 50 + (high_setting / 2)` | e.g. High **50%** → triggers at **75%** real water level |
| `real_low = low_setting / 2` | e.g. Low **50%** → triggers at **25%** real water level |

Hysteresis then runs against these real values:

- A pump turns **ON** once `current_water_level` rises **above** `real_high`.
- It stays ON until `current_water_level` drops **below** `real_low`, at
  which point it turns **OFF**.

**Manual stop via the High setting.** Because normal hysteresis only consults
`real_low` while a pump is running, raising `pump_N_set_high` alone would
otherwise have no effect on an already-ON pump. As an operator override, when a
`pump_N_set_high` command arrives and the new `real_high` is now **above** the
current water level, a running pump is stopped immediately (an OFF push is
emitted). This is edge-triggered on the command only — it is intentionally not
part of the per-loop hysteresis check, since a continuous "OFF while below
high" rule would erase the deadband and cause on/off chatter. After such a
stop, the pump restarts normally, only once the level climbs above the new
(higher) `real_high`.

`pump_N_set_high`/`pump_N_set_low` and the `pump_N_high_thr`/`pump_N_low_thr`
echo fields all read/write the *raw* 0–100 setting (matching what the PWA's
slider shows) — the real trigger values are internal to the hysteresis logic
and aren't directly exposed over BLE.

This is evaluated every loop iteration. Whenever a pump's state actually
flips, the firmware immediately pushes an unsolicited BLE message:

```json
{"pump_1_state":"on","current_water_level":74}
```

Notes:
- Pump state is **not persisted** — all 6 pumps reset to OFF on every boot.
- This is a **logical state only**: no GPIO/relay pin is driven. Physical
  pump control is not wired up yet.
- A PWA that (re)connects mid-session can read current states on demand via
  `{"cmd":"get_pump_states"}` (see [Commands](#commands)) instead of waiting
  for the next transition.
- Thresholds are validated independently (0–100 each); nothing enforces
  `high > low` per pump in firmware, so a misconfigured pair could oscillate.

## EEPROM layout

| Address | Contents |
|---|---|
| `0` | `0xAA` if product UID configured, else `0x00` |
| `1–128` | Product UID string (null-terminated, max 127 chars) |
| `129` | `0xBB` if serial number configured, else `0x00` |
| `130–257` | Serial number string (null-terminated, max 127 chars) |
| `258` | `0xCC` if retrofit config below is configured, else `0x00` |
| `259–262` | Data send interval, seconds (`unsigned long`) |
| `263–264` | Sensor warm-up time, seconds (`uint16_t`) |
| `265–266` | EMA filter smoothing level, N (`uint16_t`) |
| `267` | BLE/Notecard sync mode: `0` = normal (continuous), `1` = sleep (on-demand) |
| `268–291` | 6 pumps × (high threshold, low threshold) × 2 bytes each (`uint16_t`) |

## Commands

All commands are sent over BLE as single JSON lines. The firmware recognizes
**two distinct styles**, checked in this order (first match wins — see
`loop()` in `src/main.cpp`):

### A. `{"cmd":"..."}` style

| Command | Request | Reply / effect |
|---|---|---|
| Get config | `{"cmd":"get_config"}` | `{status, category, product_uid, serial_number, sample_period_ms}` |
| Get status | `{"cmd":"get_status"}` | Full status: version, category, product_uid, serial_number, sample_period_ms, adc, adc_voltage_v, supply_voltage_v (from Notecard `card.voltage`), lat/lon (from `card.location`) |
| Get pump states | `{"cmd":"get_pump_states"}` | On-demand read-back: `{status, pump_1_state...pump_6_state ("on"/"off"), current_water_level}` — for a PWA that just (re)connected, instead of waiting for the next hysteresis transition |
| Setup device | `{"cmd":"setup_device","product_uid":"...","serial_number":"..."}` | **First-boot only.** Stores both fields; must be followed by `confirm_setup` |
| Confirm setup | `{"cmd":"confirm_setup"}` | **First-boot only.** Unblocks setup phase once UID+SN are stored |
| Set config | `{"cmd":"set_config","product_uid":"...","serial_number":"..."}` | Runtime identity change (partial update ok); saves to EEPROM, pushes identity to Notecard, **resets MCU** |
| Set sample period | `{"cmd":"set_sample_period","period_ms":N}` | Sets sample period in memory only (1000–86400000 ms) — **not persisted**, resets to default on power-cycle |
| Echo | `{"cmd":"echo","payload":...}` or bare `echo <text>` | Round-trip link test — echoes the payload back |
| Reset config | `{"cmd":"reset_config"}` | Clears product UID + serial number from EEPROM, **resets MCU** back into setup phase |

### B. Flat `{"<name>":"<value>"}` style (retrofit config — values are always quoted strings, even numbers)

| Command | Effect | Persisted? |
|---|---|---|
| `set_product_uid` | Sets product UID | Yes → resets MCU |
| `set_serial_number` | Sets serial number (can combine with `set_product_uid` in one message) | Yes → resets MCU |
| `set_data_e_t_sec` | Data send interval, seconds (1–86400); also updates the live sample period | Yes |
| `set_sensor_init_t_sec` | Sensor warm-up time, seconds (0–3600) | Yes |
| `set_sample` | EMA filter smoothing level, N (1–5000) | Yes |
| `set_ble_mode` | `"normal"` (continuous Notecard sync) or `"sleep"` (on-demand sync) | Yes, also pushed to Notecard |
| `reset_ble` | Sends `AT+RESET` to the HM-10 module and reinitializes the BLE UART (no hardware reset pin is wired; may not take effect while a BLE session is live) | — |
| `pump_N_set_high` / `pump_N_set_low` (N = 1–6) | Pump high/low setting (0–100) — see [Pump ON/OFF control](#pump-onoff-control) for how this maps to the real trigger | Yes |
| `echo` | Diagnostic read-back — see fields below | — |

**Flat `echo` fields** — `{"echo":"<field>"}`:

| Field | Returns |
|---|---|
| `embedded_software_ver` | Firmware version string |
| `notecard_ver` | Notecard firmware version (via `card.version`) |
| `uid` | Notehub-assigned DeviceUID (via `card.version`) |
| `set_data_e_t_sec` | Current data send interval, seconds |
| `set_sensor_init_t_sec` | Current sensor warm-up time, seconds |
| `sample_rate` | Current EMA smoothing level, N |
| `ble_state` | Current BLE mode (`"normal"` / `"sleep"`) |
| `sensor_adc_value` | Current raw + EMA-filtered ADC reading |
| `pump_N_high_thr` / `pump_N_low_thr` (N = 1–6) | Current pump threshold values |

> Note: there are two separate "echo" mechanisms — `{"cmd":"echo","payload":...}`
> (generic link round-trip test) vs. flat `{"echo":"<field_name>"}` (reads back
> a specific config value). They're easy to conflate but functionally distinct.

## Configuration reference

| Setting | Default | Valid range |
|---|---|---|
| Data send interval | 60 s | 1–86400 s |
| Sensor warm-up time | 20 s | 0–3600 s |
| EMA smoothing level (N) | 200 | 1–5000 |
| BLE/Notecard sync mode | normal (continuous) | normal / sleep |
| Pump high/low settings (×6) | 0 | 0–100 (raw setting, not the real trigger — see [Pump ON/OFF control](#pump-onoff-control)) |
