#ifndef _EVSE_CLOUD_AGENT_CORE_H
#define _EVSE_CLOUD_AGENT_CORE_H

// -------------------------------------------------------------------
// EVSE cloud relay protocol - portable device agent core
//
// This file (and its .cpp) implement the charger side of the "Device
// agent" section of PROTOCOL.md and NOTHING else. It deliberately has
// no OpenEVSE, Arduino or ESP-IDF dependency so it can be lifted
// file-for-file into another firmware (or compiled on the host for
// tests). Everything platform specific arrives through
// EvseCloudAgentHost.
//
// Only <stdint.h>/<stddef.h> here; the .cpp additionally uses
// ArduinoJson (itself platform independent) and <string.h>.
// -------------------------------------------------------------------

#include <stdint.h>
#include <stddef.h>

// Protocol version carried in every payload as "v"
#define EVSE_CLOUD_AGENT_PROTOCOL_VERSION   1

// Version of this agent implementation, reported in agent/presence
#define EVSE_CLOUD_AGENT_VERSION            "0.1.0"

// Topic suffixes, appended to the charger's MQTT base topic
#define EVSE_CLOUD_AGENT_TOPIC_STATUS       "agent/status"
#define EVSE_CLOUD_AGENT_TOPIC_PRESENCE     "agent/presence"
#define EVSE_CLOUD_AGENT_TOPIC_SESSION      "agent/session"
#define EVSE_CLOUD_AGENT_TOPIC_CMD          "agent/cmd"
#define EVSE_CLOUD_AGENT_TOPIC_ACK          "agent/ack"

// EVSE state codes (OpenEVSE style RAPI codes, see PROTOCOL.md). Repeated
// here rather than included so the core stays free of firmware headers.
#define EVSE_CLOUD_AGENT_STATE_NOT_CONNECTED  1
#define EVSE_CLOUD_AGENT_STATE_CONNECTED      2
#define EVSE_CLOUD_AGENT_STATE_CHARGING       3
#define EVSE_CLOUD_AGENT_STATE_FAULT_FIRST    4
#define EVSE_CLOUD_AGENT_STATE_FAULT_LAST    11
#define EVSE_CLOUD_AGENT_STATE_SLEEPING     254
#define EVSE_CLOUD_AGENT_STATE_DISABLED     255

// Optional status flags, an open set in the protocol
#define EVSE_CLOUD_AGENT_FLAG_MANUAL_OVERRIDE (1u << 0)
#define EVSE_CLOUD_AGENT_FLAG_DIVERT_ACTIVE   (1u << 1)
#define EVSE_CLOUD_AGENT_FLAG_LIMIT_ACTIVE    (1u << 2)

#ifndef EVSE_CLOUD_AGENT_DEFAULT_INTERVAL_S
#define EVSE_CLOUD_AGENT_DEFAULT_INTERVAL_S 30
#endif

// A state/vehicle change publishes a status document this long after the
// first change is seen, so a burst of transitions costs one publish.
#ifndef EVSE_CLOUD_AGENT_DEBOUNCE_MS
#define EVSE_CLOUD_AGENT_DEBOUNCE_MS 1000
#endif

// How long after a command a session ending is still blamed on it
#ifndef EVSE_CLOUD_AGENT_CMD_REASON_WINDOW_MS
#define EVSE_CLOUD_AGENT_CMD_REASON_WINDOW_MS 15000
#endif

// Idle poll when there is nothing to do (disconnected/disabled)
#ifndef EVSE_CLOUD_AGENT_IDLE_MS
#define EVSE_CLOUD_AGENT_IDLE_MS 5000
#endif

// Command ids we remember for de-duplication of QoS 1 redeliveries
#ifndef EVSE_CLOUD_AGENT_CMD_HISTORY
#define EVSE_CLOUD_AGENT_CMD_HISTORY 8
#endif

// Longest command id we keep (longer ids are still executed, but only the
// first EVSE_CLOUD_AGENT_CMD_ID_LEN-1 characters take part in de-duplication)
#ifndef EVSE_CLOUD_AGENT_CMD_ID_LEN
#define EVSE_CLOUD_AGENT_CMD_ID_LEN 32
#endif

// Serialisation scratch buffer, sized for the largest document (status)
#ifndef EVSE_CLOUD_AGENT_PAYLOAD_BUF
#define EVSE_CLOUD_AGENT_PAYLOAD_BUF 384
#endif

// -------------------------------------------------------------------
// One snapshot of everything the status document can carry. The host
// fills this in on demand; every optional value has a validity flag so
// unknown values are omitted rather than published as zero.
// -------------------------------------------------------------------
struct EvseCloudAgentState
{
  uint8_t  state;             // EVSE state code
  bool     vehicle;           // vehicle connected

  double   session_wh;        // energy of the session in progress, Wh

  double   amp;               // charge current, A
  bool     amp_valid;
  double   volt;              // supply voltage, V
  bool     volt_valid;
  int32_t  pilot_a;           // pilot/charge current setting, A
  bool     pilot_valid;
  double   temp_c;            // monitored temperature, degrees C
  bool     temp_valid;
  int32_t  wifi_rssi;         // dBm
  bool     rssi_valid;
  uint32_t free_heap;         // bytes
  bool     heap_valid;

  uint32_t flags;             // EVSE_CLOUD_AGENT_FLAG_*

  EvseCloudAgentState() :
    state(0), vehicle(false), session_wh(0),
    amp(0), amp_valid(false), volt(0), volt_valid(false),
    pilot_a(0), pilot_valid(false), temp_c(0), temp_valid(false),
    wifi_rssi(0), rssi_valid(false), free_heap(0), heap_valid(false),
    flags(0)
  {
  }
};

// Why a charging run ended, published as agent/session "reason"
enum EvseCloudAgentReason
{
  EvseCloudAgentReason_Vehicle = 0,   // the EV stopped, still plugged in
  EvseCloudAgentReason_Unplugged,
  EvseCloudAgentReason_Fault,
  EvseCloudAgentReason_Sleep,
  EvseCloudAgentReason_Command        // a recent agent command caused it
};

// Outcome of executing a command, reported back in the ack
enum EvseCloudAgentResult
{
  EvseCloudAgentResult_Ok = 0,
  EvseCloudAgentResult_BadArgs,       // -> code "bad_args"
  EvseCloudAgentResult_Failed         // -> code "failed"
};

// -------------------------------------------------------------------
// Everything the core needs from the world around it.
// -------------------------------------------------------------------
class EvseCloudAgentHost
{
  public:
    virtual ~EvseCloudAgentHost() { }

    // Publish payload on <base topic>/<topic_suffix>. Returns false if the
    // transport could not accept it (eg not connected).
    virtual bool publish(const char *topic_suffix, const char *payload, bool retain) = 0;

    // Milliseconds since boot, must not wrap for the life of the device
    virtual uint64_t monotonicMs() = 0;

    // Epoch seconds, or 0 when the clock has never been synchronised
    virtual uint32_t epochSeconds() = 0;

    // Fill in the current state of the charger
    virtual void readState(EvseCloudAgentState &state) = 0;

    // Firmware version string for agent/presence
    virtual const char *firmwareVersion() = 0;

    // Current IP address for agent/presence, may be an empty string
    virtual const char *ipAddress() = 0;

    // "override.set": force the EVSE active or disabled, optionally at a
    // given charge current (has_charge_current false = leave as is)
    virtual EvseCloudAgentResult setOverride(bool active, uint32_t charge_current,
                                             bool has_charge_current) = 0;

    // "override.clear": drop any override claim. Idempotent.
    virtual EvseCloudAgentResult clearOverride() = 0;
};

// -------------------------------------------------------------------
// The agent itself
// -------------------------------------------------------------------
class EvseCloudAgentCore
{
  private:
    struct CmdRecord
    {
      char id[EVSE_CLOUD_AGENT_CMD_ID_LEN];
      bool ok;
      const char *code;           // static string, NULL when ok
      const char *result_state;   // static string, NULL when no result
    };

    struct SessionRecord
    {
      uint32_t start_ts;
      uint32_t end_ts;
      double   wh;
      uint8_t  reason;
      bool     valid;
    };

    EvseCloudAgentHost &_host;

    bool     _enabled;
    bool     _connected;
    uint32_t _interval_ms;

    // status publishing
    bool     _has_published;
    uint8_t  _published_state;
    bool     _published_vehicle;
    uint64_t _last_status_ms;
    bool     _change_pending;
    uint64_t _change_at_ms;

    // session tracking
    bool     _charging;
    uint32_t _session_start_ts;
    uint64_t _session_start_ms;
    double   _session_wh;
    SessionRecord _pending_session;

    // command handling
    CmdRecord _cmds[EVSE_CLOUD_AGENT_CMD_HISTORY];
    uint8_t   _cmd_next;
    bool      _had_cmd;
    uint64_t  _last_cmd_ms;

    char _buf[EVSE_CLOUD_AGENT_PAYLOAD_BUF];

    void publishStatus(uint64_t now, const EvseCloudAgentState &state);
    void publishPresence();
    void publishSession(const SessionRecord &record);
    void trackSession(const EvseCloudAgentState &state, uint64_t now);
    void ack(const char *id, bool ok, const char *code, const char *result_state);

    CmdRecord *findCmd(const char *id);
    void rememberCmd(const char *id, bool ok, const char *code, const char *result_state);

  public:
    EvseCloudAgentCore(EvseCloudAgentHost &host);

    // interval_s of 0 disables the agent entirely (no publishes, no
    // command handling); any other value is the status publish period.
    void setInterval(uint32_t interval_s);
    bool isEnabled() const { return _enabled; }
    uint32_t getInterval() const { return _interval_ms / 1000; }

    // The transport connected: publishes retained presence and status,
    // and flushes a session record held over a disconnection.
    void onConnected();
    void onDisconnected();

    // The EVSE state or vehicle presence may have changed. Cheap; the
    // resulting status publish is debounced.
    void onStateChanged();

    // Call from the host's task loop, returns the milliseconds to wait
    // before the next call.
    uint32_t loop();

    // A payload arrived on <base topic>/agent/cmd
    void onCommand(const char *payload, size_t length);

    // True while a charging run is being tracked
    bool isSessionActive() const { return _charging; }
};

// Text for a session end reason, exposed for tests and logging
const char *evse_cloud_agent_reason_to_string(uint8_t reason);

#endif // _EVSE_CLOUD_AGENT_CORE_H
