# CLAUDE.md

Headless USB-Host RTL-SDR receiver on the Waveshare ESP32-P4-WIFI6-DEV-KIT: 1090 MHz ADS-B/Mode-S
decoded on-chip, a console REPL on the serial port and over SSH (the radar TUI is the `tui` command
inside it, the system monitor `top`), decoded frames fed out over Ethernet/WiFi as AVR raw (:30001), Beast (:30005) and a
JSON aircraft snapshot (:8888). No display. Raw IQ (~4 MB/s) never leaves the chip — that is the
point of decoding locally.

This file holds only what is not derivable from the code: schematic facts, build gotchas and the
cross-file invariants. Where the rest lives:

- `docs/console.md` — console REPL + SSH console: ownership and the screen/sink model, the bugs
  found on hardware, the command reference. **Read before touching `shell.c`, `screen.c` or
  `net_ssh.c`.**
- `../c6-notes.md` — single source of truth for the ESP32-C6 co-processor (pinout, host
  `sdkconfig` block, SDIO reflash runbook, P2 brick recovery). Read before anything C6-related.
- `../wsl-readsb-notes.md` — the readsb/tar1090 host in WSL and the link it reaches the board
  over. Read before touching the feed path; readsb is the cross-check for a suspect decode.

## Hardware

**ESP32-P4NRW32X, silicon rev 3.x** — no embedded flash, 32 MB PSRAM in-package, 16 MB external
flash (`partitions.csv` at 0x10000). ESP-IDF ≥5.5 defaults to the rev3 linker scripts; an older
toolchain silently builds a rev1 ELF, which is the wrong image for this board.

**PSRAM** is on: `CONFIG_SPIRAM=y`, HEX mode, 200 MHz (250 MHz exists on rev 3.x, untried).
`SPIRAM_MALLOC_ALWAYSINTERNAL` (16K) and `SPIRAM_MALLOC_RESERVE_INTERNAL` (32K) stay at their
defaults so small/DMA/ISR allocations remain internal. Static buffers move to PSRAM with
`EXT_RAM_BSS_ATTR` (`CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY=y`) under one rule, written above
`s_aircraft[]` in `class_driver.c`: **task-context only and CPU-only** — PSRAM is cache-backed, so
nothing an ISR touches and nothing a DMA engine reads or writes may go there. `s_mag` is the only
tagged buffer on the demod hot path; if `adsb_rx`'s core1 share moves off ~21-24%, it is the
suspect and the rollback is deleting one attribute. Deliberately left internal: `s_log` (any path
may log, including IRAM-safe ones) and `mode-s.c`'s `maglut` (2M random lookups/s — a candidate,
but wants its own A/B on `adsb_rx` %).

**WiFi/BT**: the P4 has no radio. An **ESP32-C6FH8 (U12) on 4-bit SDIO slot 1** provides WiFi6 +
BLE5 through `espressif/esp_hosted` (+ `esp_wifi_remote`). Pins D0=14 D1=15 D2=16 D3=17 CLK=18
CMD=19 are esp_hosted's stock P4 slot-1 defaults — no overrides needed. C6 `CHIP_PU` → **P4
GPIO54** (`ESP_HOSTED_SDIO_GPIO_RESET_SLAVE`, default already 54); C6 GPIO2 → P4 GPIO6 (R77).
Header **P2** (4 bare unpopulated pads: `C6_U0TXD`/`C6_U0RXD`/GND/`C6_IO9`) is the C6's serial
escape hatch for a bricked co-processor only — the over-SDIO OTA path works. Don't confuse it with
**P6**, the populated 40-pin P4 GPIO header. The C6 runs ESP-Hosted slave **2.12.6**, the host
pins the same version, and both ends are in **packet mode** and must stay in agreement (streaming
host vs packet slave drops reads silently at `sdio_drv.c:959`; the reverse asserts at
`transport_drv.c:876`). `CONFIG_ESP_HOSTED_MEMPOOL_PREFER_SPIRAM` does **not** exist in 2.12.6
(landed in 2.12.8); the only PSRAM knob there is `ESP_HOSTED_DFLT_TASK_FROM_SPIRAM`, left off
because a task stack in PSRAM is unreachable while the cache is disabled during a flash write.

**Ethernet**: RJ45 behind an **IP101GRI** PHY (U2) on RMII, driven by the P4's own EMAC — no
esp_hosted, no PSRAM. The pin map is `ETH_ESP32_EMAC_DEFAULT_CONFIG()`'s ESP32-P4 branch unchanged
(MDC=31, MDIO=52, REF_CLK in=50, TX_EN=49, TXD0=34, TXD1=35, CRS_DV=28, RXD0=29, RXD1=30),
verified pin-for-pin against U2's schematic nets; `net_eth.c` overrides only PHY reset
(**GPIO51**). REF_CLK is an input (`EMAC_CLK_EXT_IN`) because the PHY's 50M_CLKO is strapped back
to its own 50M_CLKI and to GPIO50. `phy_addr` stays `ESP_ETH_PHY_ADDR_AUTO` — the strap resistors
were never traced. Keep any future pin remap off GPIO52/53 (RMII IO_MUX candidates; 53 is the
audio PA-enable). IDF v6.0 removed the vendor PHY drivers, so `espressif/ip101` comes from the
component registry.

**Audio**: ES8311 on I²C SCL=8/SDA=7, I²S MCLK=13/BCK=12/WS=10/DOUT=9/DIN=11, PA-enable GPIO53 —
identical on the older Nano, so audio code is board-agnostic. Driven through
`espressif/esp_codec_dev` (`espressif/es8311` is deprecated upstream and uses the EOL
`driver/i2c.h`). Three non-obvious things: the I²S channel is initialised but **not** enabled in
`audio_init()` because `esp_codec_dev_open()` enables it itself (enabling twice fails); tones go
out through `i2s_channel_write()` because `esp_codec_dev_write()` blocks up to 1 s on a full DMA
queue; `audio_codec_i2c_cfg_t.addr` wants the **8-bit** address (`0x30`).

**USB host power**: hardwired-always-on load switch; host/device mode is jumper **H3** (must be
HOST), not firmware. The Nano's GPIO46 VBUS code is gone.

## Build

ESP-IDF **v6.0+** (`main/idf_component.yml` declares `idf: '>=6.0'`).

```bash
source <esp-idf>/export.sh   # per shell, before every idf.py call
idf.py set-target esp32p4    # once per fresh build dir
idf.py build
idf.py -p PORT flash monitor
```

- Managed components (`espressif/usb`, `esp_codec_dev`, `ip101`, `esp_hosted`,
  `david-cermak/libssh` …) are fetched by `idf.py build`. **`idf.py add-dependency --dry-run` is
  not dry** — it mutates the manifest.
- After moving the tree to another host run `idf.py fullclean` once: `build/` caches absolute
  paths.
- `david-cermak/libssh` 0.12.2 needs `CONFIG_VFS_SUPPORT_TERMIOS=y` (defaults to `n` since v6.0;
  its `getpass.c` uses `struct termios` unconditionally) **and** a local patch,
  `patches/libssh-0.12.2-mbedtls-4.0.0.patch`, applied from the top-level `CMakeLists.txt` at
  configure time (`patches/apply.sh`, idempotent — it has to re-apply because
  `managed_components/` is gitignored and re-fetched whenever the manifest or lock changes). The
  patch fixes the three sites where the component targets an mbedTLS 4.x snapshot instead of the
  4.0.0 that IDF v6.0.1 ships (`pk->psa_type`; `mbedtls_pk_set_pubkey_from_prv` →
  `mbedtls_pk_rsa_set_pubkey_from_prv`). `david-cermak/mbedtls_v3_shim` is not an escape route —
  it writes `pk->psa_type` too.
- `partitions.csv`: `ota_0`/`ota_1` at 2 MB each + 8 KB `otadata`; `nvs` and the 11.9 MB
  `storage` kept their old offsets so the switch did not force an NVS erase. Image is ~1.5 MB.

## Working in this repo

Remote `https://github.com/bwinhwang/esp32p4-rtl-sdr-v4.git` (personal fork; upstream
`SAMS0N1TE/esp32p4-rtl-sdr-v4`). **Commit identity**: the global `user.email` is the JoyNext work
address; this repo carries a local override to `bwinhwang@gmail.com` — check `git config
user.email` on every fresh clone / new host and set it locally, never touch the global. Unpushed
commits with the wrong author:
`git rebase -f <upstream> --exec 'git commit --amend --no-edit --author="Binhong Wang <bwinhwang@gmail.com>"'`.

## Architecture

```
usb_host_lib_task   core0 prio 2   lib-level events only, idle in steady state
class_driver_task   core0 prio 3   enumeration state machine AND the context the transfer-
                                   completion callbacks run in: stream_xfer_cb -> stream_push()'s
                                   4 MB/s memcpy into the ring lands HERE
rtlsdr_setup_task   core0 prio 4   transient, spawned on NEW_DEV
rtl_pump            core1 prio 12  only re-submits the 8x16KB transfers (~244/s); nearly idle
adsb_rx_task        core1 prio 5   the real load: ring -> demodulate() -> mode_s_detect() -> on_msg()
                                   (ICAO/CRC/callsign/alt/vel/heading/vrate + CPR lat/lon)
screen              core0 prio 2   the draw task (screen.c): paints `tui` / `top` only while a viewer is on it
audio_task          core1 prio 6   ES8311 tones — ABOVE adsb_rx_task on the same core
usb_recover_task    core0 prio 4
wifi_mgr            core0 prio 3   C6 bring-up, then STA join attempts with backoff
shell               core0 prio 3   UART0's ONLY reader; prompt from boot, with or without a dongle
ssh_srv             core0 prio 3   accept loop; runs the line editor and the commands itself, so
                                   `tasks` over SSH is measuring this task
```

- **Core placement is not what the priorities suggest.** Demod is on core1; enumeration, the screens
  and the USB data copy are on core0 — transfer callbacks run in whoever called
  `usb_host_client_handle_events()`, which is only ever `class_driver_task`. That copy is ~1% of
  core0.
- **Core0 load is console bytes/second and nothing else** (~8 µs CPU per byte out of UART0, a
  repaint ~13 KB). `TUI_REFRESH_MS` is the throttle: 150 → 63% of core0, 500 → 16%. `-O2`,
  `setvbuf` and bypassing the stdio lock were all measured and do not help. Never route the frame
  through `printf`: `uart_vfs`'s `write()` calls `uart_write_bytes(&c, 1)` **per character** for
  the CRLF translation, one mutex per byte. `fb_flush()` writes `uart_write_bytes()` directly.
- Measure with `top` (`top [seconds]`; per-core busy %, internal heap / PSRAM, per-task CPU%, stack
  headroom, TIME+, and the `display` line = the console-bytes cost of whatever screens are up),
  or `tasks` / `usb` when numbers have to be copied. The TUI's `CPU0`/`CPU1` gauges and TASKS
  panel read the same sampler (`top.c`) until the TUI rewrite drops them. Needs
  `CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS`; the counter is `..._COUNTER_TYPE_U64` so TIME+ does
  not wrap every 71 min.
- **`heap internal` in `top` (and `HEAP` on the TUI status row) is internal RAM only**
  (`heap_caps_get_free_size(MALLOC_CAP_INTERNAL)`) — `esp_get_free_heap_size()` folds in 32 MB of
  PSRAM and hides the number that actually runs out. PSRAM is the separate line/field. A min-ever
  below the 32 KB `SPIRAM_MALLOC_RESERVE_INTERNAL` line means task stacks/DMA callers are eating
  the reserve.
  Runtime heap (task stacks, USB host, lwIP, esp_hosted, libssh) is the bigger consumer than
  static data; `SPIRAM_TRY_ALLOCATE_WIFI_LWIP` (see `sdkconfig.defaults`) is the next lever.
- **Priority vs the demod loop.** lwIP's tcpip task (prio 18) is pinned by
  `CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0`. esp_hosted's `sdio_*`/`rpc_*` tasks are **prio 23 with
  no affinity**, hardcoded in `port_esp_hosted_host_os.h` with plain `xTaskCreate`; no Kconfig or
  runtime API can pin them (they show `-` in the TASKS panel's CORE column). Measured under a
  10 Mbit/s flood they land mostly on core0 and cost no IQ samples — scheduler behaviour, not a
  guarantee. If `USB drop` ever climbs, the fix is vendoring the component to pin them to core0.
  **Do not raise `adsb_rx_task` above 23** — it would starve the SDIO link it depends on. And
  **measure this board from a wired peer**: a laptop's own WiFi NIC produces phantom latency spikes.
- **USB streaming ring** (`esp_libusb.c`): **1 MB in PSRAM** (8 × 16 KB slots, 256 ms of slack at
  2 MSPS), explicit `heap_caps_malloc(MALLOC_CAP_SPIRAM)`, size and cap both `#if CONFIG_SPIRAM`.
  Safe in PSRAM because only the CPU touches it (the USB stack DMAs into its own internal buffer
  and `stream_push()` memcpys from there) and the P4 guarantees coherence for pointer access across
  cores — no `esp_cache_msync()` needed. The rule that follows: nothing touching it may run with
  the cache disabled. `stream_push()` has `IRAM_ATTR` but is **not** an ISR; anything that turns
  that path into a real ISR breaks the rule. Bulk reads use a 4-slot pre-submitted pipeline with
  escalating recovery: endpoint clear → teardown+reinit → `adsb_request_recover()` →
  `usb_recover_task` (`class_driver.c`) → `rtlsdr_reset_interface()`.
- **Naming trap**: two unrelated structs are both named `class_driver_t` — the small one in
  `esp_libusb.h`/`rtl-sdr.h` (carried as `rtlsdr_dev_t.driver_obj`) and the large one local to
  `class_driver.c` (enumeration bookkeeping). Never in the same TU, so it compiles. `cmd_usb()`
  declares the `esp_libusb_stream_*` prototypes locally for this reason.
- `mode-s.c` is dump1090-derived. `mode_s_detect()` fills `timestamp_12mhz`/`signal_level` itself
  from a caller-supplied buffer timestamp (2 MSPS ⇒ 6 ticks/sample at Beast's 12 MHz). CPR
  lat/lon is decoded downstream in `class_driver.c`'s `cpr_decode()` into `aircraft_t`, not in
  `mode_s_msg`.
- **API polarity traps**: `rtlsdr_get_tuner_pll_locked()` returns **1 for locked**, 0 unlocked,
  -1 unknown — not librtlsdr's 0-is-success convention. The dongle has no gain-*mode* getter and
  keeps returning the last manual value under AGC, so `cmd_sdr()` mirrors the mode in a static.

### Aircraft classification and synthetic contacts

`plane_cat.c` maps ICAO + callsign onto MIL/COM/GA/UNK for colouring only. Two ordering rules are
the whole reason the file needs care: **commercial is tested before GA** (an airline flight
number — three letters then a digit — also satisfies the GA registration rule), and **a military
prefix must be followed by a digit** (or `CG` swallows every Canadian `C-Gxxx` and `GAF` every
British `G-AFxx`).

**Do not filter "placeholder" contacts.** ICAO `AAAAAA`/callsign `1000` and `FFFFFE`/`3456` with
thousands of messages are real airborne transponders — readsb on the same Beast stream shows DF17,
CRC clean, ADS-B v2, consecutive squawks 3631/3632, coherent moving fixes NE over the Zhoushan sea
— most likely UAVs or a test flight with unset identity. All-digit callsigns exist in the wild;
`plane_cat.c` leaves them `UNK`.

The **`t` key** (`inject_fake_aircraft()`) pushes one synthetic contact per press, cycling the
four categories, positioned relative to `CONFIG_ADSB_RX_LAT/LON`. `s_aircraft[]` has no lock and
is safe only because `on_msg()` runs in `adsb_rx_task` — so the key handler (core0) sets
`s_inject_req` and the demod loop does the write; with no dongle there is no writer to race and it
runs inline. Same rule for anything else added to a key handler that touches the aircraft table:
defer it.

**TUI column budget**: header and rows are 88 visible chars against `LEFT_W` = 111. Each row also
hand-computes its `vis` width to pad the coloured line — a new column means updating that too, or
the right panel shifts.

## Console REPL and SSH — invariants only

Full design in `docs/console.md`. What breaks if dropped:

- `shell_init()` installs the UART driver at the top of `app_main`; the console task is UART0's
  only reader for the life of the board. Only `esp_console_run()` (the Eval half of esp_console)
  is used — the Read half is `shell.c`'s own line editor, shared by both transports.
- Console ownership (`OWNER_NONE/UART/SSH`) and "which screen is in front here" (`SCREEN_NONE/
  TUI/TOP`, one per transport) are independent axes. SSH **preempts** the serial shell. Handover
  order is fixed: **claim → swap stdout → print banner**, and on exit **restore stdout → close**.
- Screens are a service (`screen.c`: `screen_attach/screen_detach`, one draw task); neither
  `class_driver.c` nor `top.c` knows a transport. Each sink names its screen, so the radar on
  serial and `top` over SSH coexist; within a screen, state and width are shared across viewers.
  `screen_attach()` clears the terminal itself before publishing the slot — never clear after.
- `screen.c`'s paint lock interlocks the frame and the prompt; anything that writes UART0 in
  response to a keystroke needs `screen_hold()`.
- `net_ssh.c` swaps the **global** `stdout` *and* `tls_stdout`/`tls_stderr` (the console component
  force-includes a header that redirects `printf` to the thread-local one). The `funopen()` stream
  does its own `\n → \r\n`. The write callback never touches libssh — it feeds a ring the SSH task
  drains.
- `ssh_init()` before any `ssh_pki_*`; never `ssh_disconnect()` from an auth callback; never
  create/destroy a libssh session while another is live (`reject_extra()` uses raw `accept()`).
- **Test SSH with a real `ssh` client on a real terminal** — paramiko hides every
  terminal-semantics bug.
- Log facilities: `sys_log()` (board — never in the LOG panel), `air_log()` (sky), `ui_log()`
  (the display answering a key, for whoever pressed it). `shell_async_print()` redraws the prompt
  only when no command is running, the display does not own stdout, and the console lock is free
  (taken with **no wait**; skipped from the console task's own thread).

## Networking (`net_wifi.c`, `net_eth.c`, `feed_*.c`, `web_config.c`)

Two deployments: fixed at home feeding readsb over Ethernet, or in the car with a phone on the
SoftAP. Only decoded messages go out (hundreds of B/s).

- **`WIFI_MODE_APSTA` permanently.** `net_wifi.c` never calls `esp_wifi_deinit()` and never
  destroys a netif — a second `esp_netif_create_default_wifi_ap()` asserts in `netif_add` and
  re-init wedges SDIO. Within that rule either half can be switched off at runtime
  (`net_wifi_set_sta_enabled/ap_enabled`, persisted in NVS `netcfg`), and the two are **not
  symmetric**: STA off only stops the join loop (free, keeps credentials); AP off is a real
  `esp_wifi_set_mode(APSTA → STA)` — reasoned safe from the component source, **not verified on
  hardware**; fallback is hiding the SSID. Switching the AP off can strand a headless board, so
  both entry points refuse unless Ethernet has a lease or the STA is joined (console needs
  `force`).
- `esp_wifi_set_config(WIFI_IF_AP, …)` is **rejected in a mode without an AP interface** —
  `wifi_bringup()` configures in APSTA unconditionally, then narrows to STA before
  `esp_wifi_start()`.
- `NET_WIFI_AP` means only "not joined upstream"; whether anything beacons is
  `net_wifi_ap_enabled()`. `net`, the config page and the TUI header consult both.
- STA config: `pmf_cfg.capable = true` (an AP requiring PMF otherwise refuses and the board just
  sits at "AP only"). `esp_wifi_set_protocol()` adds `WIFI_PROTOCOL_11AX` on both interfaces (the
  default bitmap is B/G/N; the C6 supports HE). `esp_wifi_set_ps(WIFI_PS_NONE)` right after
  `esp_wifi_start()` — the default `WIFI_PS_MIN_MODEM` naps between the *upstream* AP's DTIMs and
  on a single-radio C6 every nap drops SoftAP clients within ~30 s. All three non-fatal if the
  slave rejects them.
- STA route priority is demoted to **40** (esp_netif's default 100 beats Ethernet's 50 and steals
  the default route from the cable). This decides the *default* route only.
- **The STA address is unreachable from the LAN while Ethernet is up** and both share a subnet:
  lwIP's `ip4_route()` returns the first matching netif (Ethernet is added first), so replies to
  the STA address egress on the cable and die. Recorded, not fixed — neither deployment needs it;
  fixing means `CONFIG_LWIP_IP4_ROUTE_HOOK`.
- **The STA join sweep drops SoftAP clients**: an upstream SSID configured but out of range makes
  the C6 sweep 2.4 GHz on every retry, and the SoftAP goes with it. `retry_delay_ms()` backs off
  30 s → 480 s and holds a 5-minute floor while any station is associated;
  `esp_wifi_set_scan_parameters()` sets 60 ms per channel against 150 ms home (default 120/30).
  The wait loop sleeps in 5 s slices because the delay depends on the client count. `wifi sta off`
  removes the sweep entirely. The SoftAP is also dragged to the upstream AP's channel on join —
  costs throughput (~10 → ~1.7 Mbit/s co-channel with a busy home AP), does **not** drop
  associated stations.
- `on_wifi_event()` logs the disconnect reason once per *reason* (201 no AP, 202 auth, 15 bad
  key), cleared by a successful join. A phone rejoining with a different MAC each time is
  Android's MAC randomisation, not a bug.
- SoftAP gateway is **192.168.8.1** (`wifi_bringup()` overrides esp_netif's 192.168.4.0/24).
- Credentials (upstream WiFi, SSH user/pass/host key) live in NVS namespace `netcfg`, never in
  Kconfig — `sdkconfig` is tracked and the remote is public.
- **Feeds** bind `INADDR_ANY`, so every interface serves them with no per-interface code. AVR raw
  :30001 and Beast :30005 share the ring-buffer/non-blocking-broadcast design; Beast's 12 MHz
  timestamp is receiver-local monotonic (`mode_s_msg.timestamp_12mhz`), **not** PPS-disciplined —
  feeder compatibility, not MLAT; `signal_level` is the bit-slicing delta, relative only. JSON
  :8888 broadcasts one full `aircraft_t` snapshot per tick (~1.3 Hz, NDJSON, `aircraft.json`-like)
  so a slow client misses a tick instead of needing backpressure.
- `web_config.c` (port `CONFIG_ADSB_WEB_PORT`, default 80) pins httpd to **core0 prio 3** —
  `httpd_config_t` defaults to `tskNO_AFFINITY` at prio 5, exactly `adsb_rx_task`'s.
  Connect-per-request HTTP/1.0 clients RST-storm it (`CONFIG_LWIP_MAX_ACTIVE_TCP` 16 + TIME_WAIT
  PCBs); keep-alive is fine and nothing real churns connections — raise the limit only if some
  client does. `CONFIG_LWIP_MAX_SOCKETS` is 16 (three feeds + page + SSH).
- **Android client**: `WifiNetworkSpecifier` (local-only, keeps cellular for tiles), bind only the
  board socket (a dedicated OkHttpClient — the pool is global), hardcode 192.168.8.1 over mDNS.
  BLE provisioning was dropped (SoftAP+BLE coexistence unstable; esp_hosted BLE is version-pinned
  to the C6 image).

## OTA (`ota.c`)

Developer push on a trusted LAN/SoftAP — deliberately **no auth, no signed images, no HTTPS**
(a second TLS stack next to esp_hosted's and libssh's). If the board ever leaves a hobbyist's own
network, add a token check before anything else.

```
curl --data-binary @build/usb_host_lib_example.bin http://<board-ip>/ota
```

- `POST /ota` is registered on the config page's httpd instance (`ota_register_http()`), raw
  binary body, no multipart.
- `esp_ota_begin()` uses `OTA_SIZE_UNKNOWN` — erases the whole slot up front but is immune to an
  undercounting `Content-Length`, which with the exact-size path truncates silently instead of
  failing validation. A short `vTaskDelay()` before `esp_restart()` lets the response leave.
- `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`: `ota_init()` (from `app_main`, before USB/audio/net
  so nothing failing ahead of it skips it) confirms the image after **20 s** — past enumeration and
  network bring-up. Any panic/watchdog/reset before that reverts on the next boot with no operator.
  A direct `idf.py flash` never enters `PENDING_VERIFY`, so `ota_init()` is a no-op there. `ota
  rollback` reverts on demand either way.
