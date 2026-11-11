#include "learm_bridge_cpp/health.hpp"
#include <iostream>
#include <stdexcept>
namespace b=learm_bridge_cpp;
void check(bool ok,const char* why) { if(!ok) throw std::runtime_error(why); }
void u16(std::vector<std::uint8_t>& p,int i,std::uint16_t v) {p[i]=v&255;p[i+1]=v>>8;}
b::Frame fixture(bool ext) {
 b::Frame f; f.message_type=b::kMsgHealthState;f.payload.resize(ext?128:44);
 f.payload[0]=1;u16(f.payload,4,4096);for(int i=0;i<5;++i)u16(f.payload,8+2*i,128);f.payload[42]=1;
 if(ext){f.payload[44]=1;f.payload[45]=31;for(int i=0;i<5;++i)u16(f.payload,56+2*i,1);u16(f.payload,66,1);f.payload[72]=1;f.payload[73]=1;f.payload[75]=1;f.payload[92]='F';}
 return f;
}
int main(){try{
 auto f=fixture(true);auto s=b::decode_health_state(f);
 check(s.extended&&s.timestamp_ms==1&&s.minimum_heap_bytes==4096&&s.firmware_id=="F","offsets");
 auto legacy=b::decode_health_state(fixture(false));check(!legacy.extended,"legacy");
 for(int n:{0,43,45,127,129}){auto bad=f;bad.payload.resize(n);try{b::decode_health_state(bad);throw std::runtime_error("bad length");}catch(const std::invalid_argument&){}}
 f.payload[44]=2;try{b::decode_health_state(f);throw std::runtime_error("bad schema");}catch(const std::invalid_argument&){}
 b::HealthThresholds t;t.validate();b::HealthTracker tracker;tracker.connected();
 check(b::assess_health(tracker.snapshot(10),t,true)[0].level==b::HealthLevel::stale,"no health");
 tracker.update(s,10);auto ds=b::assess_health(tracker.snapshot(10.1),t,true);check(ds.size()==7,"seven statuses");
 for(const auto& d:ds)check(d.level==b::HealthLevel::ok,"healthy (idle arm query unknown)");
 tracker.update(s,10.4);check(b::assess_health(tracker.snapshot(10.6),t,true)[0].message=="STALE","frozen snapshot");
 check(b::assess_health(tracker.snapshot(12.1),t,true)[0].message=="OFFLINE","offline");
 tracker.connected();check(tracker.snapshot(12.2).state.has_value()&&!tracker.snapshot(12.2).updated_this_connection,"reconnect retains stale data");
 s.timestamp_ms=0xfffffff0U;tracker.update(s,13);s.timestamp_ms=10;tracker.update(s,13.1);check(tracker.snapshot(13.2).health_age_s<.2,"rollover");
 s.timestamp_ms=1;tracker.update(s,13.3);check(tracker.snapshot(13.3).health_age_s==0,"restart");
 ++s.timestamp_ms;s.active_faults=16;tracker.update(s,14);check(b::assess_health(tracker.snapshot(14),t,true)[1].level==b::HealthLevel::error,"motor");
 ++s.timestamp_ms;s.active_faults=0;s.historical_faults=16;tracker.update(s,14.1);ds=b::assess_health(tracker.snapshot(14.1),t,true);check(ds[0].level==b::HealthLevel::ok&&ds[1].level==b::HealthLevel::ok,"recovery");
 ++s.timestamp_ms;s.active_faults=8;s.safety_state=4;tracker.update(s,14.2);check(b::assess_health(tracker.snapshot(14.2),t,true)[4].level==b::HealthLevel::warn,"controlled estop");
 ++s.timestamp_ms;s.active_faults=0;s.safety_state=1;s.heartbeat_age_ms[1]=51;tracker.update(s,15);check(b::assess_health(tracker.snapshot(15),t,true)[6].level==b::HealthLevel::error,"deadline");
 ++s.timestamp_ms;s.heartbeat_age_ms[1]=b::kAgeUnknown;tracker.update(s,15.1);check(b::assess_health(tracker.snapshot(15.1),t,true)[6].level==b::HealthLevel::warn,"unknown heartbeat");
 legacy.latched_fault_flags=16;tracker.update(legacy,16);check(b::assess_health(tracker.snapshot(16),t,true)[0].level==b::HealthLevel::warn,"legacy unknown active");
 t.offline_after_s=.1;try{t.validate();throw std::runtime_error("threshold validation");}catch(const std::invalid_argument&){}
 std::cout<<"PASS: health parsing, freshness, recovery and thresholds\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
