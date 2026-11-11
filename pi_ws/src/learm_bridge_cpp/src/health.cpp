#include "learm_bridge_cpp/health.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace learm_bridge_cpp {
namespace {
std::uint16_t u16(const std::uint8_t* p) { return p[0] | (std::uint16_t(p[1]) << 8); }
std::uint32_t u32(const std::uint8_t* p) {
  return std::uint32_t(p[0]) | (std::uint32_t(p[1]) << 8) |
         (std::uint32_t(p[2]) << 16) | (std::uint32_t(p[3]) << 24);
}
std::string number(double value) {
  if (!std::isfinite(value)) return "unknown";
  std::ostringstream s; s << std::fixed << std::setprecision(6) << value; return s.str();
}
std::string hex(std::uint16_t value) {
  std::ostringstream s; s << "0x" << std::uppercase << std::hex << std::setw(4)
                          << std::setfill('0') << value; return s.str();
}
std::string faults(std::uint16_t value) {
  static const std::pair<std::uint16_t,const char*> bits[]{{1,"COMMAND_TIMEOUT"},{2,"SERIAL_OVERFLOW"},{4,"PROTOCOL"},{8,"ESTOP"},{16,"MOTOR"},{32,"ARM"},{64,"HEALTH"},{32768,"INTERNAL"}};
  std::string result;
  for(const auto& bit:bits) if(value & bit.first) { if(!result.empty()) result+='|';result+=bit.second; }
  if(value & 0x7f80U) {if(!result.empty())result+='|';result+="UNKNOWN_BITS";}
  if(result.empty())result="NONE";
  return result+" ("+hex(value)+")";
}
std::string age(std::uint16_t value) { return value == kAgeUnknown ? "unknown" : std::to_string(value); }
const char* safety_name(std::uint8_t value) {
  static const char* names[]{"STARTUP","NORMAL","STOPPING","SAFE_IDLE","ESTOP_LATCHED","ESTOP_READY","BOOT_PENDING","FAULT"};
  return value < 8 ? names[value] : "UNKNOWN";
}
const char* event_name(std::uint16_t value) {
  static const char* names[]{"NONE","INIT","STARTUP_READY","MOTION_ACCEPTED","STOP_REQUESTED","STOP_CONFIRMED","ESTOP_ACTIVITY","ESTOP_NEUTRAL_READY","ESTOP_MODE_RECOVERED","BOOT_AUTHORIZED","BOOT_CANCELLED","FAULT_LATCHED"};
  return value < 12 ? names[value] : "UNKNOWN";
}
void set(HealthDiagnostic& d, HealthLevel level, const std::string& message) {
  if (static_cast<int>(level) >= static_cast<int>(d.level)) { d.level = level; d.message = message; }
}
void add(HealthDiagnostic& d, const std::string& key, const std::string& value) { d.values.emplace_back(key, value); }
std::string csv(const std::array<std::uint16_t, 5>& values, std::uint8_t mask, bool ages) {
  std::string out;
  for (std::size_t i=0; i<values.size(); ++i) {
    if (i) out += ',';
    out += ages && !(mask & (1U << i)) ? "unknown" : (ages ? age(values[i]) : std::to_string(values[i]));
  }
  return out;
}
}  // namespace

HealthState decode_health_state(const Frame& frame) {
  if (frame.message_type != kMsgHealthState || (frame.payload.size()!=44 && frame.payload.size()!=128))
    throw std::invalid_argument("HEALTH_STATE must contain 44 or 128 bytes");
  const auto* p=frame.payload.data(); HealthState s;
  s.timestamp_ms=u32(p); s.minimum_heap_bytes=u32(p+4);
  for (std::size_t i=0;i<5;++i) s.stack_free_words[i]=u16(p+8+2*i);
  for (std::size_t i=0;i<11;++i) s.counters[i]=u16(p+18+2*i);
  s.latched_fault_flags=u16(p+40); s.safety_state=p[42];
  if (frame.payload.size()==44) return s;
  if (p[44]!=1) throw std::invalid_argument("unsupported HEALTH_STATE schema");
  s.extended=true; s.schema=p[44]; s.heartbeat_valid_mask=p[45];
  s.active_faults=u16(p+46); s.historical_faults=u16(p+48); s.last_fault_bits=u16(p+50);
  s.last_fault_timestamp_ms=u32(p+52);
  for (std::size_t i=0;i<5;++i) s.heartbeat_age_ms[i]=u16(p+56+2*i);
  s.chassis_age_ms=u16(p+66); s.i2c_consecutive_failures=u16(p+68); s.command_age_ms=u16(p+70);
  s.command_validity=p[72]; s.arm_status=p[73]; s.arm_query=p[74]; s.owner=p[75];
  s.last_event=u16(p+76); s.last_event_timestamp_ms=u32(p+80); s.uptime_ms=u32(p+84); s.tx_drops=u32(p+88);
  for (std::size_t i=92;i<108 && p[i]!=0;++i) {
    const char c=static_cast<char>(p[i]); s.firmware_id += p[i]>=32 && p[i]<=126 ? c : '?';
  }
  return s;
}
void HealthThresholds::validate() const {
  if (!std::isfinite(health_stale_after_s) || health_stale_after_s<=0 ||
      !std::isfinite(offline_after_s) || offline_after_s<=health_stale_after_s)
    throw std::invalid_argument("health thresholds require 0 < health_stale_after_s < health_offline_after_s");
  if (!minimum_heap_bytes || !chassis_max_age_ms || chassis_max_age_ms>=kAgeSaturated)
    throw std::invalid_argument("minimum_heap_bytes and chassis_max_age_ms must be positive; age < 65534");
  for (auto v:task_deadlines_ms) if (!v || v>=kAgeSaturated) throw std::invalid_argument("task_deadlines_ms must be in 1..65533");
  for (auto v:stack_warning_words) if (!v) throw std::invalid_argument("stack_warning_words must be positive");
}
void HealthTracker::connected() {
  updated_this_connection_=false; received_at_s_.reset(); progressed_at_s_.reset();
}
void HealthTracker::update(const HealthState& state, double now) {
  if (!std::isfinite(now)) throw std::invalid_argument("health host timestamp must be finite");
  // Modulo subtraction accepts normal uint32 rollover. A backwards jump is a
  // new device epoch; all freshness baselines restart, with no prior fault latch.
  const auto delta=state_ ? std::uint32_t(state.timestamp_ms-state_->timestamp_ms) : 0U;
  if (updated_this_connection_ && state_ && delta >= 0x80000000U) {
    ++boot_count_;
    progressed_at_s_.reset();
    received_at_s_.reset();
  }
  if (!updated_this_connection_ || !state_ || delta!=0) progressed_at_s_=now;
  received_at_s_=now; updated_this_connection_=true; state_=state;
}
HealthSnapshot HealthTracker::snapshot(double now) const {
  const auto elapsed=[now](const std::optional<double>& time) {
    return time ? std::max(0.0,now-*time) : std::numeric_limits<double>::infinity();
  };
  return HealthSnapshot{state_,updated_this_connection_,elapsed(progressed_at_s_),elapsed(received_at_s_),boot_count_};
}

std::vector<HealthDiagnostic> assess_health(const HealthSnapshot& snap, const HealthThresholds& t, bool serial_open) {
  std::vector<HealthDiagnostic> ds;
  for (const char* suffix:{"link","chassis","arm","communication","safety","resources","tasks"})
    ds.push_back({std::string("learm/health/")+suffix,"ONLINE",HealthLevel::ok,{}});
  auto& link=ds[0];
  const bool fresh=serial_open && snap.updated_this_connection && snap.state && snap.health_age_s<=t.health_stale_after_s;
  const auto* s=snap.state ? &*snap.state : nullptr;
  add(link,"health_age_s",number(snap.health_age_s));
  add(link,"receive_age_s",number(snap.receive_age_s));
  add(link,"device_epoch",std::to_string(snap.boot_count));
  add(link,"updated_this_connection",snap.updated_this_connection ? "true":"false");
  add(link,"firmware_id",s && s->extended ? s->firmware_id : "UNKNOWN");
  add(link,"uptime_ms",s && s->extended ? std::to_string(s->uptime_ms):"unknown");
  add(link,"active_faults",s && s->extended ? faults(s->active_faults):"unknown");
  add(link,"historical_faults",s ? faults(s->extended?s->historical_faults:s->latched_fault_flags):"unknown");
  add(link,"last_fault",s && s->extended ? faults(s->last_fault_bits):"unknown");
  std::string fault_age="unknown";
  if(s && s->extended && s->last_fault_bits) {
    const auto delta=std::uint32_t(s->uptime_ms-s->last_fault_timestamp_ms);
    if(delta<0x80000000U) fault_age=number(delta*.001+snap.health_age_s);
  }
  add(link,"last_fault_age_s",fault_age);
  add(link,"last_event",s && s->extended ? event_name(s->last_event):"UNKNOWN");
  add(link,"last_event_code",s && s->extended ? std::to_string(s->last_event):"unknown");
  add(link,"owner",s && s->extended ? (s->owner==1?"RASPBERRY_PI":"UNKNOWN"): "UNKNOWN");
  add(link,"safety_state",s?safety_name(s->safety_state):"UNKNOWN");
  if (s) {
    add(link,"health_timestamp_ms",std::to_string(s->timestamp_ms));
    add(link,"schema",s->extended?"1":"legacy");
    if(s->extended && (s->active_faults & ~std::uint16_t(8))) set(link,HealthLevel::error,"active controller faults");
    else if(s->extended && (s->active_faults & 8)) set(link,HealthLevel::warn,"controlled emergency stop");
    else if(!s->extended) set(link,HealthLevel::warn,"legacy health: current faults unknown");
    add(ds[1],"chassis_age_ms",s->extended?age(s->chassis_age_ms):"unknown");
    if(!s->extended || s->chassis_age_ms==kAgeUnknown) set(ds[1],HealthLevel::warn,"chassis freshness unknown");
    else if(s->chassis_age_ms>t.chassis_max_age_ms) set(ds[1],HealthLevel::error,"chassis telemetry overdue");
    if(s->extended && (s->active_faults & 16)) set(ds[1],HealthLevel::error,"motor fault active");
    add(ds[2],"arm_status",s->extended?(s->arm_status==1?"IDLE":s->arm_status==2?"BUSY":s->arm_status==3?"FAULT":"UNKNOWN"):"UNKNOWN");
    add(ds[2],"arm_query",s->extended?std::to_string(s->arm_query):"unknown");
    if(!s->extended || s->arm_status==0 || s->arm_status>3) set(ds[2],HealthLevel::warn,"arm status unknown");
    else if(s->arm_status==3 || (s->active_faults & 32)) set(ds[2],HealthLevel::error,"arm fault active");
    else if(s->arm_status==2 && s->arm_query==2) set(ds[2],HealthLevel::warn,"busy arm query failed");
    add(ds[3],"i2c_consecutive_failures",s->extended?std::to_string(s->i2c_consecutive_failures):"unknown");
    add(ds[3],"command_age_ms",s->extended?age(s->command_age_ms):"unknown");
    add(ds[3],"command_validity",s->extended?std::to_string(s->command_validity):"unknown");
    add(ds[3],"tx_drops",s->extended?std::to_string(s->tx_drops):"unknown");
    static const char* counter_names[]{"uart_framing","uart_crc","uart_overflow","uart_restart","i2c_timeout","i2c_nack","command_timeout","arm_rejection","ps2_estop","bootloader_reject","task_heartbeat_miss"};
    for(std::size_t i=0;i<s->counters.size();++i) add(ds[3],counter_names[i],std::to_string(s->counters[i]));
    if(!s->extended) set(ds[3],HealthLevel::warn,"legacy counters only; current communication unknown");
    else if(s->i2c_consecutive_failures || (s->active_faults & 6)) set(ds[3],HealthLevel::error,"communication fault active");
    else if(s->command_validity==2 || (s->active_faults & 1)) set(ds[3],HealthLevel::warn,"command expired; motion controlled");
    else if(s->command_validity>2) set(ds[3],HealthLevel::warn,"command validity unknown");
    add(ds[4],"safety_state",safety_name(s->safety_state));
    add(ds[4],"latched_fault_flags",hex(s->latched_fault_flags));
    if(s->safety_state==7 || (s->extended && (s->active_faults & (64|32768)))) set(ds[4],HealthLevel::error,"controller safety fault");
    else if(s->safety_state!=1 && s->safety_state!=3) set(ds[4],HealthLevel::warn,"controlled safety transition");
    if(!s->extended) set(ds[4],HealthLevel::warn,"legacy safety: active faults unknown");
    add(ds[5],"minimum_heap_bytes",std::to_string(s->minimum_heap_bytes));
    add(ds[5],"stack_free_words",csv(s->stack_free_words,31,false));
    if(s->minimum_heap_bytes<t.minimum_heap_bytes) set(ds[5],HealthLevel::warn,"minimum heap below threshold");
    for(std::size_t i=0;i<5;++i) if(s->stack_free_words[i]<t.stack_warning_words[i]) set(ds[5],HealthLevel::warn,"task stack below threshold");
    add(ds[6],"task_order","DEFAULT,CHASSIS,SERIAL,ARM,HEALTH");
    add(ds[6],"heartbeat_age_ms",s->extended?csv(s->heartbeat_age_ms,s->heartbeat_valid_mask,true):"unknown,unknown,unknown,unknown,unknown");
    add(ds[6],"stack_free_words",csv(s->stack_free_words,31,false));
    for(std::size_t i=0;i<5;++i) {
      if(!s->extended || !(s->heartbeat_valid_mask & (1U<<i)) || s->heartbeat_age_ms[i]==kAgeUnknown) set(ds[6],HealthLevel::warn,"task heartbeat unknown");
      else if(s->heartbeat_age_ms[i]>t.task_deadlines_ms[i]) set(ds[6],HealthLevel::error,"task heartbeat deadline missed");
    }
  }
  for(std::size_t i=0;i<ds.size();++i) {
    std::string detail=ds[i].message=="ONLINE"?"no current issue detected":ds[i].message;
    if(s && i==1) detail += "; sample age="+(s->extended?age(s->chassis_age_ms):"unknown")+
        " ms, I2C consecutive fail="+(s->extended?std::to_string(s->i2c_consecutive_failures):"unknown");
    if(s && i==2) detail += std::string("; state=")+(s->extended?(s->arm_status==1?"IDLE":s->arm_status==2?"BUSY":s->arm_status==3?"FAULT":"UNKNOWN"):"UNKNOWN")+
        ", last query="+(s->extended?(s->arm_query==1?"OK":s->arm_query==2?"FAILED":"UNKNOWN"):"UNKNOWN");
    if(s && i==3) detail += "; CRC="+std::to_string(s->counters[1])+", RX overflow="+std::to_string(s->counters[2])+
        ", TX drop="+(s->extended?std::to_string(s->tx_drops):"unknown")+
        "; command="+(s->extended?(s->command_validity==1?"VALID":s->command_validity==2?"EXPIRED":"UNKNOWN"):"UNKNOWN")+
        ", age="+(s->extended?age(s->command_age_ms):"unknown")+" ms";
    if(s && i==5) detail += "; minimum heap="+std::to_string(s->minimum_heap_bytes)+" bytes";
    if(s && i==4) detail += std::string("; ")+safety_name(s->safety_state);
    add(ds[i],"detail",detail);
  }
  if(fresh) link.message="ONLINE";
  if(!fresh) {
    const bool offline=s && (!serial_open || (snap.updated_this_connection && snap.health_age_s>t.offline_after_s));
    for(auto& d:ds) { d.level=HealthLevel::stale; d.message=offline?"OFFLINE":"STALE"; }
  }
  return ds;
}
}  // namespace learm_bridge_cpp
