# evse-cloud-agent

The device half of the **EVSE cloud relay protocol** — the part that runs on a
charger. It publishes consolidated, versioned JSON documents over the charger's
existing MQTT connection and answers acknowledged commands.

The protocol is specified in `PROTOCOL.md` in
[evse-cloud-client](https://github.com/Overwatt/evse-cloud-client), section
*Device agent (charger ↔ cloud, MQTT)*. That repository holds the spec and the
phone-side client; this one holds the charger-side implementation.

Platform free by construction: `<stdint.h>`, `<stddef.h>`, ArduinoJson,
`<string.h>`, `<math.h>` and nothing else. No Arduino, no ESP-IDF, no vendor
SDK, no `#ifdef` per target. It compiles and is fully tested on a laptop.

## The wire format, in one table

All topics sit under the charger's MQTT base topic. Payloads are JSON objects
carrying `"v": 1`; receivers ignore unknown fields.

| Topic (suffix)   | Direction    | Retained | When                                                        |
|------------------|--------------|----------|-------------------------------------------------------------|
| `agent/status`   | device→cloud | yes      | on connect, every `interval_s`, ~1 s after a state change    |
| `agent/presence` | device→cloud | yes      | on connect                                                   |
| `agent/session`  | device→cloud | no       | once per charging run, when it ends                          |
| `agent/cmd`      | cloud→device | –        | subscribed at connect                                        |
| `agent/ack`      | device→cloud | no       | one per command received                                     |

One document per ingest removes the race a one-topic-per-key surface has, where
state, vehicle presence and session energy arrive as independent messages and a
server has to guess which snapshot it is looking at. Retention means a
reconnecting consumer reads current truth straight from the broker with no
baseline seeding.

## Integrating it

Implement `EvseCloudAgentHost` — nine methods, no inheritance beyond it:

```cpp
class MyHost : public EvseCloudAgentHost
{
  bool publish(const char *suffix, const char *payload, bool retain);
  uint64_t monotonicMs();
  uint32_t epochSeconds();          // 0 when the clock has never been set
  void readState(EvseCloudAgentState &state);
  const char *firmwareVersion();
  const char *ipAddress();
  EvseCloudAgentResult setOverride(bool active, uint32_t current, bool has_current);
  EvseCloudAgentResult clearOverride();
};
```

Then drive the core from wherever your firmware's scheduler lives:

```cpp
EvseCloudAgentCore core(host);

core.setInterval(30);              // seconds; 0 disables the agent entirely
core.onConnected();                // transport up: presence + status go out
core.onDisconnected();
core.onStateChanged();             // EVSE state or vehicle presence moved
core.onCommand(payload, length);   // a payload arrived on agent/cmd
unsigned long wait_ms = core.loop();  // call again in wait_ms
```

`loop()` returns the milliseconds until it next wants to run, so it maps cleanly
onto a cooperative scheduler and costs nothing between publishes. Nothing polls.

Three rules the core keeps so hosts do not have to:

* **Never publish a timestamp it does not have.** `epochSeconds()` returning 0
  means `ts` is omitted rather than sent as 1970.
* **Never execute a command twice.** The last 8 ids are remembered; a QoS 1
  redelivery is re-acknowledged with the original outcome.
* **Never lose a charging run to a dropped link.** A session that ends while
  the transport is down is held and published on the next connect.

## Footprint

Fixed, by design. No `DynamicJsonDocument`, no dynamic containers, nothing that
grows. On an ESP32 (`xtensa-esp32-elf-size`, `-Os`): **940 bytes of `.bss`,
13.9 KB of flash**, of which the command ring is 352 B and the serialisation
buffer 384 B. Per publish the core puts one `StaticJsonDocument` on the stack —
the largest, the status document, is about 336 bytes — and allocates nothing.

This matters: the firmware this was written for runs near its heap floor with
TLS active, and an agent that pushes it over is worse than no agent.

## Tests

```
pio test -e native
```

20 doctest cases, 123 assertions, covering payload shapes against the spec's own
examples, the publish cadence and debouncing, session boundaries and every
`reason` value, command de-duplication, expiry, malformed input and unsupported
ops. A fake host stands in for the charger, so the whole protocol is exercised
without hardware.

## Reference integration

The OpenEVSE ESP32 firmware, where `src/evse_cloud_agent/` pairs these two files
with a `MicroTasks` task that reads `EvseManager`, publishes through the
firmware's existing Mongoose MQTT client — never a second socket — and maps
`override.set` onto the same `ManualOverride` claim path the firmware's own MQTT
override topic uses.

## Versioning

`EVSE_CLOUD_AGENT_VERSION` is this implementation, reported in `agent/presence`
as `agent`. `EVSE_CLOUD_AGENT_PROTOCOL_VERSION` is the wire format, reported as
both `v` on every payload and `proto` in presence. The protocol version only
moves on a breaking change; new fields are additive and receivers ignore what
they do not know.

## Known deviations from PROTOCOL.md

* **QoS is the host's choice.** The core asks for publishes through
  `publish(suffix, payload, retain)` and does not name a QoS. The spec asks for
  QoS 1; the OpenEVSE integration uses QoS 0 to match the rest of its publishes.
  Retained status and presence self-heal on the next publish either way.
* **The presence LWT is the host's problem.** The spec suggests
  `agent/presence {online:false}` as the connection's last will. A host with one
  MQTT connection whose LWT is already spoken for cannot do that, and the core
  does not pretend otherwise — it publishes birth, not death.
* **`config.get` is unimplemented** and acked `unsupported`; the spec lists the
  verb without specifying its payload.
* **A command carrying `exp_ts` while the clock is unset** is refused with
  `code: "no_clock"`. Freshness cannot be checked without a clock, and refusing
  beats toggling a charger on a stale replay. The spec leaves the code set open.

## License

MIT.
