#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include <ArduinoJson.h>
#include <string>
#include <vector>
#include <string.h>
#include <stdio.h>

#include "evse_cloud_agent_core.h"

// The core is deliberately platform free, so the whole of it can be
// exercised on the host with a fake host implementation.

struct Publication
{
  std::string topic;
  std::string payload;
  bool retain;
};

class FakeHost : public EvseCloudAgentHost
{
  public:
    std::vector<Publication> published;

    bool connected = true;
    uint64_t now_ms = 0;
    uint32_t epoch = 1787700000;

    EvseCloudAgentState state;

    int override_calls = 0;
    int clear_calls = 0;
    bool last_override_active = false;
    uint32_t last_override_current = 0;
    bool last_override_has_current = false;
    EvseCloudAgentResult override_result = EvseCloudAgentResult_Ok;

    FakeHost()
    {
      state.state = EVSE_CLOUD_AGENT_STATE_NOT_CONNECTED;
    }

    bool publish(const char *topic_suffix, const char *payload, bool retain)
    {
      if(!connected) {
        return false;
      }
      Publication entry;
      entry.topic = topic_suffix;
      entry.payload = payload;
      entry.retain = retain;
      published.push_back(entry);
      return true;
    }

    uint64_t monotonicMs() { return now_ms; }
    uint32_t epochSeconds() { return epoch; }
    void readState(EvseCloudAgentState &out) { out = state; }
    const char *firmwareVersion() { return "5.1.2"; }
    const char *ipAddress() { return "10.75.1.157"; }

    EvseCloudAgentResult setOverride(bool active, uint32_t charge_current, bool has_charge_current)
    {
      override_calls++;
      last_override_active = active;
      last_override_current = charge_current;
      last_override_has_current = has_charge_current;
      return override_result;
    }

    EvseCloudAgentResult clearOverride()
    {
      clear_calls++;
      return EvseCloudAgentResult_Ok;
    }

    // Test helpers

    const Publication *last(const char *topic) const
    {
      for(size_t i = published.size(); i > 0; i--) {
        if(published[i - 1].topic == topic) {
          return &published[i - 1];
        }
      }
      return NULL;
    }

    size_t count(const char *topic) const
    {
      size_t found = 0;
      for(size_t i = 0; i < published.size(); i++) {
        if(published[i].topic == topic) {
          found++;
        }
      }
      return found;
    }
};

static void parse(const Publication *publication, DynamicJsonDocument &doc)
{
  REQUIRE(publication != NULL);
  REQUIRE(DeserializationError::Ok == deserializeJson(doc, publication->payload));
}

TEST_CASE("connecting publishes retained presence and status")
{
  FakeHost host;
  EvseCloudAgentCore core(host);

  host.state.state = EVSE_CLOUD_AGENT_STATE_CONNECTED;
  host.state.vehicle = true;
  host.state.session_wh = 1234.5;
  host.state.amp = 24.14;
  host.state.amp_valid = true;
  host.state.volt = 242.0;
  host.state.volt_valid = true;
  host.state.free_heap = 98304;
  host.state.heap_valid = true;
  host.state.flags = EVSE_CLOUD_AGENT_FLAG_MANUAL_OVERRIDE;
  host.now_ms = 86400000;

  core.onConnected();

  DynamicJsonDocument presence(512);
  const Publication *presence_pub = host.last(EVSE_CLOUD_AGENT_TOPIC_PRESENCE);
  parse(presence_pub, presence);
  CHECK(presence_pub->retain);
  CHECK(presence["v"].as<int>() == 1);
  CHECK(presence["online"].as<bool>() == true);
  CHECK(presence["proto"].as<int>() == 1);
  CHECK(std::string(presence["agent"].as<const char *>()) == EVSE_CLOUD_AGENT_VERSION);
  CHECK(std::string(presence["fw"].as<const char *>()) == "5.1.2");
  CHECK(std::string(presence["ip"].as<const char *>()) == "10.75.1.157");

  DynamicJsonDocument status(1024);
  const Publication *status_pub = host.last(EVSE_CLOUD_AGENT_TOPIC_STATUS);
  parse(status_pub, status);
  CHECK(status_pub->retain);
  CHECK(status["v"].as<int>() == 1);
  CHECK(status["ts"].as<uint32_t>() == 1787700000);
  CHECK(status["uptime_s"].as<uint32_t>() == 86400);
  CHECK(status["state"].as<int>() == 2);
  CHECK(status["vehicle"].as<int>() == 1);
  CHECK(status["session_wh"].as<double>() == doctest::Approx(1234.5));
  CHECK(status["amp"].as<double>() == doctest::Approx(24.1));
  CHECK(status["volt"].as<double>() == doctest::Approx(242.0));
  CHECK(status["free_heap"].as<uint32_t>() == 98304);
  CHECK_FALSE(status.containsKey("pilot_a"));
  CHECK_FALSE(status.containsKey("temp_c"));
  CHECK_FALSE(status.containsKey("session_start_ts"));
  CHECK(std::string(status["flags"][0].as<const char *>()) == "manual_override");
}

TEST_CASE("ts is omitted while the clock is unsynchronised")
{
  FakeHost host;
  EvseCloudAgentCore core(host);
  host.epoch = 0;

  core.onConnected();

  DynamicJsonDocument status(1024);
  parse(host.last(EVSE_CLOUD_AGENT_TOPIC_STATUS), status);
  CHECK_FALSE(status.containsKey("ts"));

  DynamicJsonDocument presence(512);
  parse(host.last(EVSE_CLOUD_AGENT_TOPIC_PRESENCE), presence);
  CHECK_FALSE(presence.containsKey("ts"));
}

TEST_CASE("a state change publishes once, after the debounce")
{
  FakeHost host;
  EvseCloudAgentCore core(host);

  core.onConnected();
  size_t baseline = host.count(EVSE_CLOUD_AGENT_TOPIC_STATUS);

  host.now_ms = 1000;
  host.state.state = EVSE_CLOUD_AGENT_STATE_CONNECTED;
  host.state.vehicle = true;
  core.onStateChanged();

  // A second transition inside the window must not cost a second publish
  host.now_ms = 1200;
  host.state.state = EVSE_CLOUD_AGENT_STATE_CHARGING;
  core.onStateChanged();

  host.now_ms = 1500;
  core.loop();
  CHECK(host.count(EVSE_CLOUD_AGENT_TOPIC_STATUS) == baseline);

  host.now_ms = 2100;
  core.loop();
  CHECK(host.count(EVSE_CLOUD_AGENT_TOPIC_STATUS) == baseline + 1);

  DynamicJsonDocument status(1024);
  parse(host.last(EVSE_CLOUD_AGENT_TOPIC_STATUS), status);
  CHECK(status["state"].as<int>() == 3);
  CHECK(status["session_start_ts"].as<uint32_t>() == 1787700000);

  // ... and nothing more until the interval comes round
  host.now_ms = 5000;
  core.loop();
  CHECK(host.count(EVSE_CLOUD_AGENT_TOPIC_STATUS) == baseline + 1);

  host.now_ms = 40000;
  core.loop();
  CHECK(host.count(EVSE_CLOUD_AGENT_TOPIC_STATUS) == baseline + 2);
}

TEST_CASE("a refused publish backs off rather than spinning")
{
  FakeHost host;
  EvseCloudAgentCore core(host);
  core.onConnected();

  // The transport starts refusing without the core being told
  host.connected = false;
  host.now_ms = 40000;
  CHECK(core.loop() > 1000);

  host.now_ms = 41000;
  CHECK(core.loop() > 1000);
}

TEST_CASE("interval of 0 disables the agent")
{
  FakeHost host;
  EvseCloudAgentCore core(host);
  core.setInterval(0);

  core.onConnected();
  core.onStateChanged();
  core.loop();

  CHECK(host.published.size() == 0);
  CHECK_FALSE(core.isEnabled());
}

TEST_CASE("leaving the charging state publishes a session record")
{
  FakeHost host;
  EvseCloudAgentCore core(host);

  core.onConnected();

  host.now_ms = 10000;
  host.state.state = EVSE_CLOUD_AGENT_STATE_CHARGING;
  host.state.vehicle = true;
  host.state.session_wh = 0;
  core.onStateChanged();
  CHECK(core.isSessionActive());

  host.now_ms = 7210000;
  host.epoch = 1787707200;
  host.state.session_wh = 8500;
  host.state.state = EVSE_CLOUD_AGENT_STATE_CONNECTED;
  core.onStateChanged();
  CHECK_FALSE(core.isSessionActive());

  DynamicJsonDocument session(512);
  const Publication *session_pub = host.last(EVSE_CLOUD_AGENT_TOPIC_SESSION);
  parse(session_pub, session);
  CHECK_FALSE(session_pub->retain);
  CHECK(session["v"].as<int>() == 1);
  CHECK(session["start_ts"].as<uint32_t>() == 1787700000);
  CHECK(session["end_ts"].as<uint32_t>() == 1787707200);
  CHECK(session["wh"].as<double>() == doctest::Approx(8500));
  CHECK(std::string(session["reason"].as<const char *>()) == "vehicle");
}

TEST_CASE("session end reason follows the state that ended it")
{
  struct Case
  {
    uint8_t state;
    bool vehicle;
    const char *reason;
  } cases[] = {
    { EVSE_CLOUD_AGENT_STATE_CONNECTED,     true,  "vehicle" },
    { EVSE_CLOUD_AGENT_STATE_NOT_CONNECTED, false, "unplugged" },
    { EVSE_CLOUD_AGENT_STATE_FAULT_FIRST,     true,  "fault" },
    { EVSE_CLOUD_AGENT_STATE_SLEEPING,      true,  "sleep" },
    { EVSE_CLOUD_AGENT_STATE_DISABLED,      true,  "sleep" }
  };

  for(size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
  {
    FakeHost host;
    EvseCloudAgentCore core(host);
    core.onConnected();

    host.now_ms = 1000;
    host.state.state = EVSE_CLOUD_AGENT_STATE_CHARGING;
    host.state.vehicle = true;
    core.onStateChanged();

    host.now_ms = 60000;
    host.state.state = cases[i].state;
    host.state.vehicle = cases[i].vehicle;
    core.onStateChanged();

    DynamicJsonDocument session(512);
    parse(host.last(EVSE_CLOUD_AGENT_TOPIC_SESSION), session);
    CHECK(std::string(session["reason"].as<const char *>()) == cases[i].reason);
  }
}

TEST_CASE("a session that ends offline is held until the next connect")
{
  FakeHost host;
  EvseCloudAgentCore core(host);

  core.onConnected();

  host.now_ms = 1000;
  host.state.state = EVSE_CLOUD_AGENT_STATE_CHARGING;
  host.state.vehicle = true;
  core.onStateChanged();

  core.onDisconnected();
  host.connected = false;
  host.now_ms = 60000;
  host.state.state = EVSE_CLOUD_AGENT_STATE_NOT_CONNECTED;
  host.state.vehicle = false;
  core.onStateChanged();
  CHECK(host.count(EVSE_CLOUD_AGENT_TOPIC_SESSION) == 0);

  host.connected = true;
  core.onConnected();
  CHECK(host.count(EVSE_CLOUD_AGENT_TOPIC_SESSION) == 1);
}

TEST_CASE("ping is acknowledged")
{
  FakeHost host;
  EvseCloudAgentCore core(host);
  core.onConnected();

  const char *cmd = "{\"v\":1,\"id\":\"01J8QZ3M9PXW\",\"op\":\"ping\"}";
  core.onCommand(cmd, strlen(cmd));

  DynamicJsonDocument ack(512);
  const Publication *ack_pub = host.last(EVSE_CLOUD_AGENT_TOPIC_ACK);
  parse(ack_pub, ack);
  CHECK_FALSE(ack_pub->retain);
  CHECK(ack["v"].as<int>() == 1);
  CHECK(std::string(ack["id"].as<const char *>()) == "01J8QZ3M9PXW");
  CHECK(ack["ok"].as<bool>() == true);
  CHECK(ack["ts"].as<uint32_t>() == 1787700000);
}

TEST_CASE("override.set drives the host and reports the new state")
{
  FakeHost host;
  EvseCloudAgentCore core(host);
  core.onConnected();

  const char *cmd = "{\"v\":1,\"id\":\"A1\",\"op\":\"override.set\","
                    "\"args\":{\"state\":\"disabled\",\"charge_current\":16}}";
  core.onCommand(cmd, strlen(cmd));

  CHECK(host.override_calls == 1);
  CHECK(host.last_override_active == false);
  CHECK(host.last_override_has_current == true);
  CHECK(host.last_override_current == 16);

  DynamicJsonDocument ack(512);
  parse(host.last(EVSE_CLOUD_AGENT_TOPIC_ACK), ack);
  CHECK(ack["ok"].as<bool>() == true);
  CHECK(std::string(ack["result"]["state"].as<const char *>()) == "disabled");
}

TEST_CASE("override.set without a state is refused")
{
  FakeHost host;
  EvseCloudAgentCore core(host);
  core.onConnected();

  const char *cmd = "{\"v\":1,\"id\":\"A2\",\"op\":\"override.set\",\"args\":{\"charge_current\":16}}";
  core.onCommand(cmd, strlen(cmd));

  CHECK(host.override_calls == 0);

  DynamicJsonDocument ack(512);
  parse(host.last(EVSE_CLOUD_AGENT_TOPIC_ACK), ack);
  CHECK(ack["ok"].as<bool>() == false);
  CHECK(std::string(ack["code"].as<const char *>()) == "bad_args");
}

TEST_CASE("override.clear releases the claim")
{
  FakeHost host;
  EvseCloudAgentCore core(host);
  core.onConnected();

  const char *cmd = "{\"v\":1,\"id\":\"A3\",\"op\":\"override.clear\"}";
  core.onCommand(cmd, strlen(cmd));

  CHECK(host.clear_calls == 1);

  DynamicJsonDocument ack(512);
  parse(host.last(EVSE_CLOUD_AGENT_TOPIC_ACK), ack);
  CHECK(ack["ok"].as<bool>() == true);
}

TEST_CASE("a redelivered command is re-acked, never re-executed")
{
  FakeHost host;
  EvseCloudAgentCore core(host);
  core.onConnected();

  const char *cmd = "{\"v\":1,\"id\":\"DUP1\",\"op\":\"override.set\","
                    "\"args\":{\"state\":\"active\"}}";
  core.onCommand(cmd, strlen(cmd));
  core.onCommand(cmd, strlen(cmd));
  core.onCommand(cmd, strlen(cmd));

  CHECK(host.override_calls == 1);
  CHECK(host.count(EVSE_CLOUD_AGENT_TOPIC_ACK) == 3);

  DynamicJsonDocument ack(512);
  parse(host.last(EVSE_CLOUD_AGENT_TOPIC_ACK), ack);
  CHECK(ack["ok"].as<bool>() == true);
  CHECK(std::string(ack["result"]["state"].as<const char *>()) == "active");
}

TEST_CASE("the de-dupe ring only remembers the most recent ids")
{
  FakeHost host;
  EvseCloudAgentCore core(host);
  core.onConnected();

  char cmd[96];
  for(int i = 0; i < EVSE_CLOUD_AGENT_CMD_HISTORY + 1; i++) {
    snprintf(cmd, sizeof(cmd), "{\"v\":1,\"id\":\"ID%d\",\"op\":\"ping\"}", i);
    core.onCommand(cmd, strlen(cmd));
  }

  // ID0 has been pushed out of the ring, so it executes again rather than
  // being treated as a redelivery. Only ids seen recently are protected.
  const char *first = "{\"v\":1,\"id\":\"ID0\",\"op\":\"override.set\",\"args\":{\"state\":\"active\"}}";
  core.onCommand(first, strlen(first));
  CHECK(host.override_calls == 1);

  const char *recent = "{\"v\":1,\"id\":\"ID8\",\"op\":\"override.set\",\"args\":{\"state\":\"active\"}}";
  core.onCommand(recent, strlen(recent));
  CHECK(host.override_calls == 1);
}

TEST_CASE("an expired command is refused")
{
  FakeHost host;
  EvseCloudAgentCore core(host);
  core.onConnected();

  const char *cmd = "{\"v\":1,\"id\":\"E1\",\"op\":\"override.set\","
                    "\"args\":{\"state\":\"active\"},\"exp_ts\":1787699999}";
  core.onCommand(cmd, strlen(cmd));

  CHECK(host.override_calls == 0);

  DynamicJsonDocument ack(512);
  parse(host.last(EVSE_CLOUD_AGENT_TOPIC_ACK), ack);
  CHECK(ack["ok"].as<bool>() == false);
  CHECK(std::string(ack["code"].as<const char *>()) == "expired");

  // The refusal is remembered, so a redelivery is answered the same way
  core.onCommand(cmd, strlen(cmd));
  CHECK(host.count(EVSE_CLOUD_AGENT_TOPIC_ACK) == 2);
  CHECK(host.override_calls == 0);
}

TEST_CASE("an unexpired command still runs")
{
  FakeHost host;
  EvseCloudAgentCore core(host);
  core.onConnected();

  const char *cmd = "{\"v\":1,\"id\":\"E2\",\"op\":\"override.set\","
                    "\"args\":{\"state\":\"active\"},\"exp_ts\":1787700120}";
  core.onCommand(cmd, strlen(cmd));

  CHECK(host.override_calls == 1);
}

TEST_CASE("an expiring command is refused when the clock is unset")
{
  FakeHost host;
  EvseCloudAgentCore core(host);
  host.epoch = 0;
  core.onConnected();

  const char *cmd = "{\"v\":1,\"id\":\"E3\",\"op\":\"override.set\","
                    "\"args\":{\"state\":\"active\"},\"exp_ts\":1787700120}";
  core.onCommand(cmd, strlen(cmd));

  CHECK(host.override_calls == 0);

  DynamicJsonDocument ack(512);
  parse(host.last(EVSE_CLOUD_AGENT_TOPIC_ACK), ack);
  CHECK(ack["ok"].as<bool>() == false);
  CHECK(std::string(ack["code"].as<const char *>()) == "no_clock");
}

TEST_CASE("an unknown op is acked as unsupported")
{
  FakeHost host;
  EvseCloudAgentCore core(host);
  core.onConnected();

  const char *cmd = "{\"v\":1,\"id\":\"U1\",\"op\":\"config.get\"}";
  core.onCommand(cmd, strlen(cmd));

  DynamicJsonDocument ack(512);
  parse(host.last(EVSE_CLOUD_AGENT_TOPIC_ACK), ack);
  CHECK(ack["ok"].as<bool>() == false);
  CHECK(std::string(ack["code"].as<const char *>()) == "unsupported");
}

TEST_CASE("a command with no id is dropped silently")
{
  FakeHost host;
  EvseCloudAgentCore core(host);
  core.onConnected();
  size_t baseline = host.published.size();

  const char *cmd = "{\"v\":1,\"op\":\"ping\"}";
  core.onCommand(cmd, strlen(cmd));
  const char *garbage = "not json";
  core.onCommand(garbage, strlen(garbage));

  CHECK(host.published.size() == baseline);
}

TEST_CASE("a session ended by a recent command is blamed on the command")
{
  FakeHost host;
  EvseCloudAgentCore core(host);
  core.onConnected();

  host.now_ms = 1000;
  host.state.state = EVSE_CLOUD_AGENT_STATE_CHARGING;
  host.state.vehicle = true;
  core.onStateChanged();

  host.now_ms = 60000;
  const char *cmd = "{\"v\":1,\"id\":\"C1\",\"op\":\"override.set\","
                    "\"args\":{\"state\":\"disabled\"}}";
  core.onCommand(cmd, strlen(cmd));

  host.now_ms = 61000;
  host.state.state = EVSE_CLOUD_AGENT_STATE_SLEEPING;
  core.onStateChanged();

  DynamicJsonDocument session(512);
  parse(host.last(EVSE_CLOUD_AGENT_TOPIC_SESSION), session);
  CHECK(std::string(session["reason"].as<const char *>()) == "command");
}
