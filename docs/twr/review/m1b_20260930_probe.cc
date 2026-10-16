// Review reproduction only: prints buggy baseline observations, NOT acceptance tests.
// Baseline core SHA256: 1a4814d1d5bbc6f41bf698bb1989a1e624e0cee77b7b874c1c40495cf3a87345
// After API changes, add proper reject/success assertions to the maintained QA.
#include "twr_m1b_test_support.h"
#include <iostream>
#include <stdexcept>
using namespace gr::uwb::twr;
using namespace gr::uwb::twr::m1b_test;
CoreConfig cfg(Role r=Role::Responder, Protocol p=Protocol::Ss) {
 CoreConfig c; c.endpoint_id="test"; c.role=r; c.protocol=p;
 c.pan_id=0x1234;c.local_address=r==Role::Initiator?0x101:0x202;
 c.peer_address=r==Role::Initiator?0x202:0x101;c.session_id=7;c.session_generation=1;
 c.sequence_modulus=4;c.local_domain=make_domain("dev",1e9);
 c.peer_binding=make_binding(c.local_domain);ClockRatio::unity_same_clock(c.local_domain,c.ratio);
 c.frame_profile=make_profile(1e9);c.local_calibration=make_calibration("cal",1,0,1000000000);
 c.exchange_timeout_ticks=1000;c.evidence_wait_ticks=1000;c.result_queue_capacity=1;c.max_in_flight=1;return c;
}
CoreAction action(CoreActionBatch b,CoreActionKind k) {for(size_t i=0;i<b.count;i++)if(b.actions[i].kind==k)return b.actions[i];return {};}
bool has(CoreActionBatch b,CoreActionKind k){for(size_t i=0;i<b.count;i++)if(b.actions[i].kind==k)return true;return false;}
void setup(EndpointCore& e,CoreConfig c){std::string why;if(!e.configure(c,why))throw std::runtime_error(why);}
CoreEvent rx(CoreConfig c,FrameType t,int seq=0) {
 CoreEvent e;e.kind=CoreEventKind::RxFrame;e.event_id=100+seq;e.decode_ok=true;e.fcs_passed=true;
 e.frame.version=1;e.frame.function_code=t;e.frame.session_id=c.session_id;e.frame.seq=seq;
 e.frame.pan_id=c.pan_id;e.frame.src_addr=c.peer_address;e.frame.dst_addr=c.local_address;e.frame.flags=make_flags(true,false);
 if(t==FrameType::Response){e.frame.timestamps[0]=120;e.frame.timestamps[1]=220;}
 e.rx_time=make_rx_ts(300,c.local_domain,"cal");e.rx_first_path=FirstPathQuality::passed(20,6,.9);return e;
}
CoreEvent plan(CoreAction a,CoreConfig c,int ticks=400,TxOutcome outcome=TxOutcome::Unknown){return make_tx_planned(a.token,a.tx_intent,ticks,c.local_domain,"cal",outcome);}
CoreEvent done(CoreAction a,TxOutcome o=TxOutcome::Completed){CoreEvent e;e.kind=CoreEventKind::TxOutcomeResolved;e.token=a.token;e.tx_outcome=o;return e;}
int main(){
 {auto c=cfg();EndpointCore e;setup(e,c);auto ev=rx(c,FrameType::Poll);ev.rx_first_path=FirstPathQuality::not_recorded();auto a=action(e.post(ev),CoreActionKind::PrepareTx);auto pe=plan(a,c);pe.planned_tx_time.domain=make_domain("wrong-device",1e9,77);e.post(pe);auto b=e.post(done(a));auto r=action(b,CoreActionKind::TerminalResult);std::cout<<"missing_rx_quality_wrong_tx_domain: terminal="<<has(b,CoreActionKind::TerminalResult)<<" completion="<<protocol_completion_status_to_string(r.result.completion)<<"\n";}
 {auto c=cfg();EndpointCore e;setup(e,c);auto a=action(e.post(rx(c,FrameType::Poll)),CoreActionKind::PrepareTx);e.post(plan(a,c));e.post(done(a));ProtocolTofEstimate r;e.pop_result(r);c.session_generation=2;c.peer_binding.session_generation=2;c.peer_binding.binding_generation=2;std::string why;e.reset(c,2,why);auto b=e.post(rx(c,FrameType::Poll));std::cout<<"same_wire_after_reset: prepare="<<has(b,CoreActionKind::PrepareTx)<<" accepted="<<e.counters().accepted_exchanges<<"\n";}
 {auto c=cfg();EndpointCore e;setup(e,c);auto a=action(e.post(rx(c,FrameType::Poll)),CoreActionKind::PrepareTx);auto b=e.post(plan(a,c,400,TxOutcome::Completed));std::cout<<"planned_completed: submit="<<has(b,CoreActionKind::SubmitTx)<<" terminal="<<has(b,CoreActionKind::TerminalResult)<<" outcome_events="<<e.counters().tx_outcome_completed<<"\n";}
 {auto c=cfg(Role::Initiator,Protocol::Ds);EndpointCore e;setup(e,c);auto poll=action(e.post(make_begin(1,0)),CoreActionKind::PrepareTx);e.post(plan(poll,c,100));auto fin=action(e.post(rx(c,FrameType::Response)),CoreActionKind::PrepareTx);e.post(plan(fin,c));auto b=e.post(done(fin));std::cout<<"ds_poll_unknown: terminal="<<has(b,CoreActionKind::TerminalResult)<<" completed="<<e.counters().tx_outcome_completed<<" submitted="<<e.counters().tx_submitted<<"\n";e.post(done(poll,TxOutcome::Underflow));std::cout<<"late_poll_underflow: failed="<<e.counters().tx_outcome_failed<<" mismatch="<<e.counters().tx_token_mismatch<<"\n";}
 {auto c=cfg();EndpointCore e;setup(e,c);for(int i=0;i<2;i++){auto a=action(e.post(rx(c,FrameType::Poll,i)),CoreActionKind::PrepareTx);e.post(plan(a,c));e.post(done(a));}std::cout<<"responder_queue: accepted="<<e.counters().accepted_exchanges<<" terminal="<<e.counters().terminal_results<<" dropped="<<e.counters().results_dropped<<"\n";}
 {auto c=cfg();EndpointCore e;setup(e,c);CoreEvent bad;bad.kind=CoreEventKind::TxOutcomeResolved;bad.tx_outcome=static_cast<TxOutcome>(99);e.post(bad);std::cout<<"idle_invalid: accepted="<<e.counters().accepted_exchanges<<" terminal="<<e.counters().terminal_results<<" inflight="<<e.in_flight()<<"\n";}
 {auto c=cfg(Role::Initiator);EndpointCore e;setup(e,c);auto old=action(e.post(make_begin(1,0)),CoreActionKind::PrepareTx);e.post(make_cancel());ProtocolTofEstimate r;e.pop_result(r);c.session_generation=2;c.peer_binding.session_generation=2;c.peer_binding.binding_generation=2;std::string why;bool ok=e.reset(c,2,why);auto fresh=action(e.post(make_begin(2,0)),CoreActionKind::PrepareTx);auto b=e.post(plan(old,c,100));std::cout<<"reset_stale_plan: reset="<<ok<<" oldtoken="<<old.token.value<<" newtoken="<<fresh.token.value<<" stale_submitted="<<has(b,CoreActionKind::SubmitTx)<<"\n";}
 {auto c=cfg(Role::Initiator);EndpointCore e;setup(e,c);auto a=action(e.post(make_begin(1,0)),CoreActionKind::PrepareTx);e.post(plan(a,c,100));auto b=e.post(make_cancel());std::cout<<"cancel_submitted: abort="<<has(b,CoreActionKind::AbortPending)<<" terminal="<<has(b,CoreActionKind::TerminalResult)<<"\n";}
 {auto c=cfg(Role::Initiator);EndpointCore e;setup(e,c);auto a=action(e.post(make_begin(1,0)),CoreActionKind::PrepareTx);e.post(plan(a,c,100));e.post(rx(c,FrameType::Response));auto b=e.post(make_deadline(1000));std::cout<<"absolute_deadline: terminal_at_1000="<<has(b,CoreActionKind::TerminalResult)<<" inflight="<<e.in_flight()<<"\n";}
}
