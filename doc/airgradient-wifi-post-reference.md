# AirGradient Wi-Fi Telemetry — Payload & Communication Reference

Reference notes on how the AirGradient firmware monorepo posts measurement data
to its backend over Wi-Fi. Written as a model to follow for adding cloud posting
to this project.

**Source analysed:** `/Users/sirawatwongmonta/Documents/fw/oaf-fw-core`
(clone of `github.com/Open-Air-Foundation/firmware-core`), product `go`.
All file references below are relative to that repo root.

---

## 1. Stack

```
CloudService (product task)      products/go/main/go_cloud.cpp
  └─ AgClient                    components/airgradient-client/services/ag_client.cpp
       ├─ payload_serializer     components/airgradient-client/services/payload_serializer.cpp
       └─ HttpClient (interface) components/airgradient-client/clients/http_client.h
            └─ WifiHttpClient    components/airgradient-client/backends/wifi_http_client.cpp
                 └─ esp_http_client + mbedTLS
```

Responsibility split:

| Layer | Owns |
|---|---|
| `CloudService` | Cadence, snapshot lifetime, arming/gating, event dispatch |
| `AgClient` | URL construction, JSON serialization, HTTP-status → result mapping |
| `HttpClient` | Abstract seam — mocked in host tests (`tests/mock_http_client.h`) |
| `WifiHttpClient` | `esp_http_client` lifecycle, TLS cert, response capture |

`WifiHttpClient` is firmware-only and not host-testable; everything above it is
pure C++ and runs in the host test suite.

---

## 2. Endpoints and identity

Built in `ag_client.cpp:268-286`.

| Operation | Method | URL |
|---|---|---|
| Post measures | `POST` | `https://hw.airgradient.com/sensors/airgradient:<serial>/measures` |
| Fetch config | `GET` | `https://hw.airgradient.com/sensors/airgradient:<serial>/go/config` |

- `<serial>` is a 12-character hex string (the device MAC), set via
  `AgClient::begin(serial, NetworkType::Wifi)`.
- The config route varies by build-time device model
  (`CONFIG_AG_DEVICE_MODEL_GO` → `go/config`,
  `CONFIG_AG_DEVICE_MODEL_ONE_OPEN_AIR` / `_MAX` → `one/config`).
  The measures route is model-independent.
- The domain is overridable at runtime via `set_http_domain()` /
  `reset_http_domain()`.

### Authentication

**There is none at the HTTP layer** — no API key, no bearer token, no signed
body. Identity is the serial number in the URL path; the backend rejects an
unregistered serial with HTTP 400.

Transport security is **TLS with a pinned root CA**: AirGradient's own
self-signed root, hardcoded as `AG_SERVER_ROOT_CA` in
`components/airgradient-client/services/ag_server_cert.h` and passed as
`config.cert_pem` on every request (`wifi_http_client.cpp:73`, `:116`).

> If you copy this design, note the trade-off: pinning a private root means the
> device only ever trusts your server, but a CA rotation requires a firmware
> update. The lack of a per-device secret means anyone who knows a serial number
> can post data for it.

---

## 3. Payload

A flat, single-level JSON object built with cJSON and printed into a
**1024-byte stack buffer** (`POST_BODY_BUFFER_SIZE`, `ag_client.h:75`), sent with
`Content-Type: application/json`.

### Always present

| Key | Type | Source |
|---|---|---|
| `wifi` | int | STA RSSI in dBm |
| `boot` | uint32 | Device uptime in whole minutes |

Everything else is a sensor field and is **omitted entirely when invalid**.
A POST issued before the first sensor read is therefore legal and minimal:

```json
{"wifi":-58,"boot":0}
```

A populated one:

```json
{"wifi":-58,"boot":42,"rco2":812,"atmp":23.51,"rhum":48.32,
 "pm01":3.4,"pm02":7.1,"pm10":8.9,"tvocIndex":102,"noxIndex":1,"volt":4.05}
```

### Full key set

Names and precision are fixed by the server contract
(`payload_serializer.cpp:18-52`).

| Group | Keys | Precision |
|---|---|---|
| Metadata | `wifi`, `boot` | integer |
| CO2 | `rco2` | integer |
| Temperature / humidity | `atmp`, `rhum` | 2 decimals |
| PM atmospheric mass (µg/m³) | `pm01`, `pm02`, `pm10` | 1 decimal |
| PM standard mass (CF=1) | `pm01Standard`, `pm02Standard`, `pm10Standard` | 1 decimal |
| PM particle counts | `pm003Count`, `pm005Count`, `pm01Count`, `pm02Count`, `pm50Count`, `pm10Count` | integer |
| TVOC / NOx | `tvocIndex`, `tvocRaw`, `noxIndex`, `noxRaw` | integer |
| Power | `volt` (battery), `light` (charging / panel) | 2 decimals |
| O3 / NO2 electrodes | `measure0` … `measure4` | 3 decimals |

Note `pm02` is PM2.5 (not PM0.2) and `pm50Count` is the 5.0 µm count — the
naming is inherited from the PMS5003 convention. Particle counts are expressed
as particles per 0.1 L; drivers with different native units convert before the
serializer sees them.

### Two patterns worth copying

**Sentinel-driven omission.** Every measure struct default-initializes each
field to a "MeasuresInvalid" sentinel, and each has an `is_*_valid()` predicate.
The serializer emits a key only when the predicate passes. The consequence is
that "sensor not fitted", "sensor not warmed up", and "sensor faulted" all
produce the same wire result — an absent key — instead of a zero or a null that
the backend has to disambiguate. Value-initialising a `Measures{}` and posting it
is always safe.

**Dual-channel reduction** (`emit_dual_float`, `payload_serializer.cpp:82-95`).
Boards carrying two sensors of the same type collapse to one key:

- both channels valid → arithmetic mean
- one valid → that channel's value
- neither valid → key omitted

Rounding happens *after* the average, so precision is not lost twice.

The serializer's input is `MeasuresInput` (`types/client_types.h`), a bundle of
non-owning pointers with null meaning "this board has no such channel". Three
`Measures` variants (`Measures`, `MeasuresBasic`, `MeasuresAGo`) each get an
overload that fills the bundle, so no conversion happens at the call site.

---

## 4. Result mapping

`ag_client.cpp:292-319`. Transport outcome and HTTP status collapse into one
enum (`AgClientResult`) that the caller reacts to.

| Condition | POST measures | GET config |
|---|---|---|
| `esp_http_client_perform` failed | `TransportError` | `TransportError` |
| Response body truncated | — | `BufferTooSmall` *(beats status)* |
| 200 | `Ok` | `Ok` |
| 201 | `Ok` | `ServerError` |
| 400 | `ServerError` | `NotRegistered` |
| 429 | `Ok` — rate-limited but stored | `ServerError` |
| anything else | `ServerError` | `ServerError` |

Treating 429 as success on the POST path is deliberate: the backend accepted and
stored the sample, it just doesn't want another one yet. Retrying would waste
radio time.

Truncation outranking status on the GET path is also deliberate — a 200 with a
partial JSON body is unusable, so it must not look like a success.

---

## 5. Communication sequence

### Bring-up

1. `WifiManager` connects. Auto-connect resolves the SSID: 0 saved networks →
   fail; 1 saved → direct connect; >1 saved → scan, intersect visible APs with
   the saved list, rank by RSSI, single attempt per candidate with failover.
2. On `IP_EVENT_STA_GOT_IP` the orchestrator calls `CloudService::start()`.
   **Heap is claimed here, not in the constructor** — 8 KB task stack + 2 KB
   fetch buffer + semaphore. Offline modes never pay for it.
3. `arm()` enables periodic ticks.

### Steady state — one task iteration

Default cadence is 60 s for both POST and FETCH.

1. Check the shutdown latch; sample gates `_armed`, `_disable_cloud`,
   `_config_fetch_enabled`.
2. **POST is checked before FETCH.** Copy the cached `MeasuresAGo` snapshot
   under a mutex (µs-scale hold), read RSSI and uptime, serialize, POST.
3. **Re-sample the gates.** A disarm or disable arriving during the in-flight
   POST gates the FETCH out of this iteration.
4. FETCH if enabled and due: GET into the 2 KB buffer, parse once, queue a
   value-only event to the orchestrator, which merges it into settings, persists,
   and applies runtime changes.
5. Sleep until the earlier of the two deadlines.

**Deadlines are start-anchored**: `next_due = started_at + interval`, not
completion time. A 15 s POST leaves 45 s of idle rather than dragging the
cadence out to 75 s.

### Sensor data path

Sensor readings never reach the network task directly. The producer task pushes
snapshots via `update_measures_snapshot()` (mutex-guarded, unconditional, in all
modes); the cloud task samples whatever is latest when its own deadline fires.
The two cadences are fully decoupled, and a slow POST cannot back-pressure
sampling.

### Gating

| Gate | Effect |
|---|---|
| `disable_cloud` | Global transport gate — suppresses both POST and FETCH |
| `config_fetch_enabled` | FETCH only — lets "local configuration control" stop cloud config changes while telemetry keeps flowing |
| `armed` / `disarmed` | Cadence on/off; in-flight requests still drain and post their result |

Wi-Fi disconnect → `disarm()`. Reconnect → `arm(false)`, resuming at the next
interval rather than firing immediately. Mode change → `disarm()` + `stop()`,
performed **before** Wi-Fi shutdown so in-flight HTTP drains while the socket is
still alive. OTA pauses cloud with `disarm()` only, never `stop()` — a committed
download reboots anyway, and a stop/start round-trip would only fragment the heap
on the failure path.

### Failure handling

Deliberately minimal:

- Each request is a fresh `esp_http_client_init` → `perform` → `cleanup`.
  **No connection reuse, no keep-alive.**
- 15 s timeout (`DEFAULT_TIMEOUT_MS`, `wifi_http_client.h:26`).
- **No retry inside the client** and **no store-and-forward queue.** A failed
  POST is logged; the next 60 s tick is the retry. Data from a failed interval is
  simply lost.
- `airgradient-payload-cache` exists in the tree but backs the on-device chart
  history in RTC memory — it is not an upload buffer.

Result events to the orchestrator are best-effort: a full queue drops the
zero-wait event and the next periodic cycle is the next opportunity.

---

## 6. Config fetch response

The GET returns a JSON object that the device parses into a settings update. It
is the backend's channel for changing device behaviour without a firmware
update. Selected fields (`products/go/docs/cloud_service.md`):

| Field | Values | Meaning |
|---|---|---|
| `pmStandard` | `"ugm3"`, `"us-aqi"` | Display unit for PM |
| `temperatureUnit` | `"c"`, `"f"` | Display unit for temperature |
| `measurementInterval` | 1 … 3600 | Sampling interval, seconds |
| `abcDays` | `-1`, 1 … 200 | CO2 automatic baseline calibration period; `-1` disables |
| `tvocLearningOffset`, `noxLearningOffset` | 1 … 1000 | Gas-index algorithm tuning |
| `corrections.pm02` | `none`, `epa_2021`, `custom_via_pm25_raw` | PM2.5 correction model |
| `corrections.atmp`, `corrections.rhum` | `none`, `custom` | Temp / humidity correction |
| `co2CalibrationRequested` | bool | One-shot action, not persisted |

Parsing rules worth reproducing:

- Each valid field sets its own bit in an update mask. A malformed field does not
  prevent valid siblings from being applied.
- A missing field means "keep the current setting", never "reset to default".
- One-shot action flags are edge-detection-free by contract: the backend returns
  `true` exactly once per request, then `false`.
- Truncated bodies, malformed roots, and trailing non-whitespace data all yield
  an empty update mask — a partial config is never half-applied.

---

## 7. Resource budget (ESP32-C5)

The TLS handshake against a 4096-bit RSA server certificate temporarily consumes
most of the largest contiguous free block on a ~183 KB heap. The Go product's
`sdkconfig` is tuned for it:

| Setting | IDF default | Go value | Saving |
|---|---|---|---|
| `CONFIG_MBEDTLS_SSL_IN_CONTENT_LEN` | 16384 | 4096 | 12 KB per TLS connection |
| `CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM` | 32 | 16 | steady-state headroom |
| `CONFIG_ESP_WIFI_DYNAMIC_TX_BUFFER_NUM` | 32 | 16 | steady-state headroom |

The 4 KB TLS input buffer is sufficient because payloads are small (POST response
< 256 B, FETCH config < 1 KB) and the workload is two requests per minute. Heap
probes bracket `start()`, `stop()`, POST, and FETCH to catch regressions.

Buffer sizes to size against:

| Buffer | Size | Location |
|---|---|---|
| POST body | 1024 B (stack) | `ag_client.h:75` |
| URL | 128 B (stack) | `ag_client.h:76` |
| Config fetch response | 2048 B (heap, only while running) | `go_cloud.cpp` `FETCH_BUFFER_BYTES` |
| Cloud task stack | 8192 B (heap, only while running) | `go_cloud.cpp` `CLOUD_TASK_STACK_SIZE` |

Cloud task priority is 4 — below the sensor producer (5), above GPS (3).

---

## 8. Checklist for porting to this project

1. Decide the identity + auth model. The AirGradient scheme (serial in URL, no
   token, pinned private root CA) only works if you also control the backend and
   accept that serials are guessable.
2. Define the JSON key set and precision **once**, in a table, and keep the
   serializer the only place that knows those strings.
3. Give every measure field an invalid sentinel and an `is_valid()` predicate;
   omit rather than emit zeros.
4. Put the HTTP call behind an abstract interface so the serializer and
   status-mapping logic can be host-tested without hardware.
5. Collapse transport + status into a single result enum, and decide explicitly
   which non-200 codes count as success (429) and which failure outranks status
   (truncation).
6. Keep sensor sampling and network cadence decoupled through a mutex-guarded
   snapshot.
7. Anchor deadlines to request start, not completion.
8. Allocate network heap on connect and free it on disconnect if the product has
   offline modes.
9. Tear the network task down before the Wi-Fi stack so in-flight requests drain.
10. Budget the TLS handshake explicitly — it is usually the largest transient
    allocation in the whole firmware.

---

## 9. As implemented in this project

Ported in `components/wifi/` and `components/cloud/`.

| Reference | Here |
|---|---|
| `WifiManager` + `EspWifiHal` | `components/wifi/wifi_sta.c` — STA connect, exponential-backoff reconnect, RSSI, MAC-derived serial |
| `CloudService` | `components/cloud/cloud.c` — task, start-anchored cadence, enable gate |
| `payload_serializer.cpp` | `components/cloud/cloud_payload.c` — same property names and precision |
| `WifiHttpClient` | `components/cloud/cloud_client.c` — `esp_http_client`, URL build, status mapping |
| `AG_SERVER_ROOT_CA` | `components/cloud/server_ca.pem`, embedded via `EMBED_TXTFILES` |
| Product `sdkconfig` TLS tuning | `sdkconfig.defaults` |

Differences worth knowing:

- **No separate snapshot.** `air_monitor_get()` is already a mutex-guarded copy,
  so the cloud task samples it directly instead of having the producer push into
  a second cached snapshot.
- **Raw gas ticks only.** The SGP41 here reports raw ticks, not the 0..500 gas
  index, so `tvocRaw` / `noxRaw` are emitted and `tvocIndex` / `noxIndex` never
  appear. Conditioning counts as "no reading".
- **No dual-channel reduction.** This board has one of each sensor, so the
  averaging path has nothing to average.
- **Extra properties.** `pm04`, `typicalSize`, `pressure` and `altitude` are
  readings this board produces that the AirGradient schema has no key for.
  `CONFIG_CLOUD_PAYLOAD_EXTRA_FIELDS=n` drops them for a strictly
  contract-compatible body.
- **Config fetch is not implemented.** Only the POST half is ported; settings
  still come from the on-device menu.
