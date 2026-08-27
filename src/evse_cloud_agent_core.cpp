// -------------------------------------------------------------------
// EVSE cloud relay protocol - portable device agent core
//
// See evse_cloud_agent_core.h. No firmware headers here, on purpose.
// -------------------------------------------------------------------

#include "evse_cloud_agent_core.h"

#include <ArduinoJson.h>
#include <string.h>
#include <math.h>

// Document capacities. Written with the ArduinoJson macros so they stay
// correct on both a 32 bit target and a 64 bit host test build.
#define EVSE_CLOUD_AGENT_STATUS_DOC   (JSON_OBJECT_SIZE(14) + JSON_ARRAY_SIZE(3) + 64)
#define EVSE_CLOUD_AGENT_PRESENCE_DOC (JSON_OBJECT_SIZE(7) + 32)
#define EVSE_CLOUD_AGENT_SESSION_DOC  (JSON_OBJECT_SIZE(5) + 32)
#define EVSE_CLOUD_AGENT_ACK_DOC      (JSON_OBJECT_SIZE(5) + JSON_OBJECT_SIZE(1) + 64)
#define EVSE_CLOUD_AGENT_CMD_DOC      (JSON_OBJECT_SIZE(5) + JSON_OBJECT_SIZE(2) + 192)

// Error codes used in agent/ack. Static strings, referenced not copied.
static const char *s_code_expired     = "expired";
static const char *s_code_unsupported = "unsupported";
static const char *s_code_bad_args    = "bad_args";
static const char *s_code_failed      = "failed";
static const char *s_code_no_clock    = "no_clock";

static const char *s_state_active   = "active";
static const char *s_state_disabled = "disabled";

// One decimal place keeps the payload small; the protocol's own examples
// carry no more precision than this.
static double round1(double value)
{
  return round(value * 10.0) / 10.0;
}

const char *evse_cloud_agent_reason_to_string(uint8_t reason)
{
  switch(reason)
  {
    case EvseCloudAgentReason_Unplugged: return "unplugged";
    case EvseCloudAgentReason_Fault:     return "fault";
    case EvseCloudAgentReason_Sleep:     return "sleep";
    case EvseCloudAgentReason_Command:   return "command";
    case EvseCloudAgentReason_Vehicle:
    default:                             return "vehicle";
  }
}

EvseCloudAgentCore::EvseCloudAgentCore(EvseCloudAgentHost &host) :
  _host(host),
  _enabled(false),
  _connected(false),
  _interval_ms(EVSE_CLOUD_AGENT_DEFAULT_INTERVAL_S * 1000),
  _has_published(false),
  _published_state(0),
  _published_vehicle(false),
  _last_status_ms(0),
  _change_pending(false),
  _change_at_ms(0),
  _charging(false),
  _session_start_ts(0),
  _session_start_ms(0),
  _session_wh(0),
  _cmd_next(0),
  _had_cmd(false),
  _last_cmd_ms(0)
{
  _pending_session.start_ts = 0;
  _pending_session.end_ts = 0;
  _pending_session.wh = 0;
  _pending_session.reason = EvseCloudAgentReason_Vehicle;
  _pending_session.valid = false;

  memset(_cmds, 0, sizeof(_cmds));
  _buf[0] = '\0';

  _enabled = _interval_ms > 0;
}

void EvseCloudAgentCore::setInterval(uint32_t interval_s)
{
  _interval_ms = interval_s * 1000;
  _enabled = interval_s > 0;
}

void EvseCloudAgentCore::onConnected()
{
  _connected = true;
  if(!_enabled) {
    return;
  }

  publishPresence();

  // A run that finished while the link was down is reported now
  if(_pending_session.valid) {
    publishSession(_pending_session);
    _pending_session.valid = false;
  }

  EvseCloudAgentState state;
  _host.readState(state);
  uint64_t now = _host.monotonicMs();
  trackSession(state, now);
  publishStatus(now, state);
}

void EvseCloudAgentCore::onDisconnected()
{
  _connected = false;
}

void EvseCloudAgentCore::onStateChanged()
{
  if(!_enabled) {
    return;
  }

  EvseCloudAgentState state;
  _host.readState(state);
  uint64_t now = _host.monotonicMs();

  trackSession(state, now);

  if(!_has_published ||
     state.state != _published_state ||
     state.vehicle != _published_vehicle)
  {
    if(!_change_pending) {
      _change_pending = true;
      _change_at_ms = now;
    }
  }
}

uint32_t EvseCloudAgentCore::loop()
{
  if(!_enabled || !_connected) {
    return EVSE_CLOUD_AGENT_IDLE_MS;
  }

  uint64_t now = _host.monotonicMs();

  bool due_change = _change_pending &&
                    (now - _change_at_ms) >= EVSE_CLOUD_AGENT_DEBOUNCE_MS;
  bool due_interval = (now - _last_status_ms) >= _interval_ms;

  if(due_change || due_interval)
  {
    EvseCloudAgentState state;
    _host.readState(state);
    trackSession(state, now);
    publishStatus(now, state);
    now = _host.monotonicMs();
  }

  uint64_t next = _last_status_ms + _interval_ms;
  if(_change_pending)
  {
    uint64_t debounce_due = _change_at_ms + EVSE_CLOUD_AGENT_DEBOUNCE_MS;
    if(debounce_due < next) {
      next = debounce_due;
    }
  }

  return next > now ? (uint32_t)(next - now) : 1;
}

// -------------------------------------------------------------------
// Publishing
// -------------------------------------------------------------------

void EvseCloudAgentCore::publishStatus(uint64_t now, const EvseCloudAgentState &state)
{
  StaticJsonDocument<EVSE_CLOUD_AGENT_STATUS_DOC> doc;

  doc["v"] = EVSE_CLOUD_AGENT_PROTOCOL_VERSION;

  uint32_t ts = _host.epochSeconds();
  if(0 != ts) {
    doc["ts"] = ts;
  }

  doc["uptime_s"] = (uint32_t)(now / 1000);
  doc["state"] = state.state;
  doc["vehicle"] = state.vehicle ? 1 : 0;
  doc["session_wh"] = round1(state.session_wh);

  if(_charging && 0 != _session_start_ts) {
    doc["session_start_ts"] = _session_start_ts;
  }

  if(state.amp_valid) {
    doc["amp"] = round1(state.amp);
  }
  if(state.volt_valid) {
    doc["volt"] = round1(state.volt);
  }
  if(state.pilot_valid) {
    doc["pilot_a"] = state.pilot_a;
  }
  if(state.temp_valid) {
    doc["temp_c"] = round1(state.temp_c);
  }
  if(state.rssi_valid) {
    doc["wifi_rssi"] = state.wifi_rssi;
  }
  if(state.heap_valid) {
    doc["free_heap"] = state.free_heap;
  }

  if(0 != state.flags)
  {
    JsonArray flags = doc.createNestedArray("flags");
    if(state.flags & EVSE_CLOUD_AGENT_FLAG_MANUAL_OVERRIDE) {
      flags.add("manual_override");
    }
    if(state.flags & EVSE_CLOUD_AGENT_FLAG_DIVERT_ACTIVE) {
      flags.add("divert_active");
    }
    if(state.flags & EVSE_CLOUD_AGENT_FLAG_LIMIT_ACTIVE) {
      flags.add("limit_active");
    }
  }

  size_t length = serializeJson(doc, _buf, sizeof(_buf));
  if(doc.overflowed() || 0 == length || length >= sizeof(_buf) - 1) {
    return;
  }

  if(_host.publish(EVSE_CLOUD_AGENT_TOPIC_STATUS, _buf, true))
  {
    _has_published = true;
    _published_state = state.state;
    _published_vehicle = state.vehicle;
    _last_status_ms = now;
    _change_pending = false;
  }
  else
  {
    // The transport refused it. Restart both timers so a persistent
    // failure retries on the usual cadence instead of spinning.
    _last_status_ms = now;
    _change_at_ms = now;
  }
}

void EvseCloudAgentCore::publishPresence()
{
  StaticJsonDocument<EVSE_CLOUD_AGENT_PRESENCE_DOC> doc;

  doc["v"] = EVSE_CLOUD_AGENT_PROTOCOL_VERSION;
  doc["online"] = true;

  uint32_t ts = _host.epochSeconds();
  if(0 != ts) {
    doc["ts"] = ts;
  }

  const char *fw = _host.firmwareVersion();
  if(NULL != fw && '\0' != fw[0]) {
    doc["fw"] = fw;
  }
  doc["agent"] = EVSE_CLOUD_AGENT_VERSION;
  doc["proto"] = EVSE_CLOUD_AGENT_PROTOCOL_VERSION;

  const char *ip = _host.ipAddress();
  if(NULL != ip && '\0' != ip[0]) {
    doc["ip"] = ip;
  }

  size_t length = serializeJson(doc, _buf, sizeof(_buf));
  if(doc.overflowed() || 0 == length || length >= sizeof(_buf) - 1) {
    return;
  }

  _host.publish(EVSE_CLOUD_AGENT_TOPIC_PRESENCE, _buf, true);
}

void EvseCloudAgentCore::publishSession(const SessionRecord &record)
{
  StaticJsonDocument<EVSE_CLOUD_AGENT_SESSION_DOC> doc;

  doc["v"] = EVSE_CLOUD_AGENT_PROTOCOL_VERSION;
  doc["start_ts"] = record.start_ts;
  doc["end_ts"] = record.end_ts;
  doc["wh"] = round1(record.wh);
  doc["reason"] = evse_cloud_agent_reason_to_string(record.reason);

  size_t length = serializeJson(doc, _buf, sizeof(_buf));
  if(doc.overflowed() || 0 == length || length >= sizeof(_buf) - 1) {
    return;
  }

  _host.publish(EVSE_CLOUD_AGENT_TOPIC_SESSION, _buf, false);
}

// -------------------------------------------------------------------
// Session boundaries
// -------------------------------------------------------------------

void EvseCloudAgentCore::trackSession(const EvseCloudAgentState &state, uint64_t now)
{
  bool charging = EVSE_CLOUD_AGENT_STATE_CHARGING == state.state;

  if(charging)
  {
    if(!_charging)
    {
      _charging = true;
      _session_start_ms = now;
      _session_start_ts = _host.epochSeconds();
    }
    _session_wh = state.session_wh;
    return;
  }

  if(!_charging) {
    return;
  }

  _charging = false;

  // The meter usually still holds the run's energy at this point; fall
  // back to the last value seen while charging if it has been cleared.
  double wh = state.session_wh > 0 ? state.session_wh : _session_wh;

  uint32_t end_ts = _host.epochSeconds();
  uint32_t start_ts = _session_start_ts;
  if(0 == start_ts && 0 != end_ts)
  {
    // The clock was set part way through the run, recover the start from
    // the monotonic elapsed time rather than dropping the record.
    uint32_t elapsed = (uint32_t)((now - _session_start_ms) / 1000);
    start_ts = end_ts > elapsed ? end_ts - elapsed : 0;
  }

  _session_start_ts = 0;
  _session_wh = 0;

  if(0 == end_ts || 0 == start_ts)
  {
    // No usable clock: publishing zeroed timestamps would collide with
    // every other unstamped session in a server that dedupes on start_ts.
    return;
  }

  uint8_t reason;
  if(_had_cmd && (now - _last_cmd_ms) <= EVSE_CLOUD_AGENT_CMD_REASON_WINDOW_MS) {
    reason = EvseCloudAgentReason_Command;
  } else if(state.state >= EVSE_CLOUD_AGENT_STATE_FAULT_FIRST &&
            state.state <= EVSE_CLOUD_AGENT_STATE_FAULT_LAST) {
    reason = EvseCloudAgentReason_Fault;
  } else if(EVSE_CLOUD_AGENT_STATE_SLEEPING == state.state ||
            EVSE_CLOUD_AGENT_STATE_DISABLED == state.state) {
    reason = EvseCloudAgentReason_Sleep;
  } else if(!state.vehicle || EVSE_CLOUD_AGENT_STATE_NOT_CONNECTED == state.state) {
    reason = EvseCloudAgentReason_Unplugged;
  } else {
    reason = EvseCloudAgentReason_Vehicle;
  }

  SessionRecord record;
  record.start_ts = start_ts;
  record.end_ts = end_ts;
  record.wh = wh;
  record.reason = reason;
  record.valid = true;

  if(_connected) {
    publishSession(record);
  } else {
    // Hold it for the next connect rather than losing the run
    _pending_session = record;
  }
}

// -------------------------------------------------------------------
// Commands
// -------------------------------------------------------------------

EvseCloudAgentCore::CmdRecord *EvseCloudAgentCore::findCmd(const char *id)
{
  for(uint8_t i = 0; i < EVSE_CLOUD_AGENT_CMD_HISTORY; i++)
  {
    if('\0' != _cmds[i].id[0] && 0 == strcmp(_cmds[i].id, id)) {
      return &_cmds[i];
    }
  }

  return NULL;
}

void EvseCloudAgentCore::rememberCmd(const char *id, bool ok, const char *code,
                                     const char *result_state)
{
  CmdRecord &record = _cmds[_cmd_next];
  strncpy(record.id, id, EVSE_CLOUD_AGENT_CMD_ID_LEN - 1);
  record.id[EVSE_CLOUD_AGENT_CMD_ID_LEN - 1] = '\0';
  record.ok = ok;
  record.code = code;
  record.result_state = result_state;

  _cmd_next = (uint8_t)((_cmd_next + 1) % EVSE_CLOUD_AGENT_CMD_HISTORY);
}

void EvseCloudAgentCore::ack(const char *id, bool ok, const char *code,
                             const char *result_state)
{
  StaticJsonDocument<EVSE_CLOUD_AGENT_ACK_DOC> doc;

  doc["v"] = EVSE_CLOUD_AGENT_PROTOCOL_VERSION;
  doc["id"] = id;
  doc["ok"] = ok;

  uint32_t ts = _host.epochSeconds();
  if(0 != ts) {
    doc["ts"] = ts;
  }

  if(!ok && NULL != code) {
    doc["code"] = code;
  }
  if(ok && NULL != result_state) {
    doc.createNestedObject("result")["state"] = result_state;
  }

  size_t length = serializeJson(doc, _buf, sizeof(_buf));
  if(doc.overflowed() || 0 == length || length >= sizeof(_buf) - 1) {
    return;
  }

  _host.publish(EVSE_CLOUD_AGENT_TOPIC_ACK, _buf, false);
}

void EvseCloudAgentCore::onCommand(const char *payload, size_t length)
{
  if(!_enabled || NULL == payload || 0 == length) {
    return;
  }

  StaticJsonDocument<EVSE_CLOUD_AGENT_CMD_DOC> doc;
  if(DeserializationError::Ok != deserializeJson(doc, payload, length)) {
    return;
  }

  const char *id = doc["id"];
  if(NULL == id || '\0' == id[0]) {
    // Without an id there is nothing to acknowledge and no way to
    // de-duplicate a redelivery, so the command is not safe to run.
    return;
  }

  // A redelivery is acknowledged again, never executed again
  CmdRecord *previous = findCmd(id);
  if(NULL != previous) {
    ack(previous->id, previous->ok, previous->code, previous->result_state);
    return;
  }

  uint32_t now_ts = _host.epochSeconds();
  if(doc.containsKey("exp_ts"))
  {
    uint32_t exp_ts = doc["exp_ts"];
    if(0 == now_ts)
    {
      // The clock is unset, so the command's freshness cannot be
      // checked. Refusing beats toggling a charger on a stale replay.
      ack(id, false, s_code_no_clock, NULL);
      rememberCmd(id, false, s_code_no_clock, NULL);
      return;
    }
    if(exp_ts < now_ts)
    {
      ack(id, false, s_code_expired, NULL);
      rememberCmd(id, false, s_code_expired, NULL);
      return;
    }
  }

  const char *op = doc["op"];
  if(NULL == op) {
    ack(id, false, s_code_bad_args, NULL);
    rememberCmd(id, false, s_code_bad_args, NULL);
    return;
  }

  bool ok = false;
  const char *code = NULL;
  const char *result_state = NULL;

  if(0 == strcmp(op, "ping"))
  {
    ok = true;
  }
  else if(0 == strcmp(op, "override.set"))
  {
    const char *state = doc["args"]["state"];
    bool active = false;

    if(NULL != state && 0 == strcmp(state, s_state_active)) {
      active = true;
    } else if(NULL != state && 0 == strcmp(state, s_state_disabled)) {
      active = false;
    } else {
      state = NULL;
    }

    if(NULL == state)
    {
      ok = false;
      code = s_code_bad_args;
    }
    else
    {
      bool has_current = doc["args"].containsKey("charge_current");
      uint32_t current = has_current ? (uint32_t)doc["args"]["charge_current"] : 0;
      if(has_current && 0 == current)
      {
        ok = false;
        code = s_code_bad_args;
      }
      else
      {
        EvseCloudAgentResult result = _host.setOverride(active, current, has_current);
        ok = EvseCloudAgentResult_Ok == result;
        code = EvseCloudAgentResult_BadArgs == result ? s_code_bad_args : s_code_failed;
        result_state = active ? s_state_active : s_state_disabled;
        _had_cmd = true;
        _last_cmd_ms = _host.monotonicMs();
      }
    }
  }
  else if(0 == strcmp(op, "override.clear"))
  {
    EvseCloudAgentResult result = _host.clearOverride();
    ok = EvseCloudAgentResult_Ok == result;
    code = s_code_failed;
    _had_cmd = true;
    _last_cmd_ms = _host.monotonicMs();
  }
  else
  {
    ok = false;
    code = s_code_unsupported;
  }

  if(!ok) {
    result_state = NULL;
  }

  ack(id, ok, code, result_state);
  rememberCmd(id, ok, code, result_state);

  if(ok && (0 == strcmp(op, "override.set") || 0 == strcmp(op, "override.clear")))
  {
    // The claim change moves the EVSE, report the new truth promptly
    onStateChanged();
  }
}
