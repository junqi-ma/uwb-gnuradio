import sys,time,json
sys.path.insert(0,'/usr/local/lib/python3.10/site-packages')
import uhd
import numpy as np
from pathlib import Path
p=Path(__file__).parent
u=uhd.usrp.MultiUSRP('addr=192.168.10.2,recv_frame_size=8000,num_recv_frames=4096')
ch=3
u.set_rx_rate(737.28e6,ch); u.set_rx_freq(6489.6e6,ch); u.set_rx_gain(60,ch); u.set_rx_antenna('RX1',ch)
rate=u.get_rx_rate(ch); n=round(rate*.100)
a=uhd.usrp.StreamArgs('sc16','sc16'); a.channels=[ch]
s=u.get_rx_stream(a); x=np.empty((1,n),np.int32)
md=uhd.types.RXMetadata(); errors=[]; chunks=[]; got=0
cmd=uhd.types.StreamCMD(uhd.types.StreamMode.num_done); cmd.num_samps=n; cmd.stream_now=False; cmd.time_spec=u.get_time_now()+uhd.types.TimeSpec(.2)
s.issue_stream_cmd(cmd); start=time.monotonic()
while got<n:
    k=s.recv(x[:,got:min(n,got+1048576)],md,1.0)
    if md.error_code != uhd.types.RXMetadataErrorCode.none: errors.append({'at':got,'error':str(md.error_code),'text':md.strerror()})
    if k: chunks.append({'at':got,'n':k,'time':md.time_spec.get_real_secs()}); got+=k
    if time.monotonic()-start>8 or (not k and errors): break
s.issue_stream_cmd(uhd.types.StreamCMD(uhd.types.StreamMode.stop_cont))
x[0,:got].tofile(p/'rx_ch3_rx1_100ms.sc16')
m={'rate':rate,'freq':u.get_rx_freq(ch),'gain':u.get_rx_gain(ch),'channel':ch,'antenna':u.get_rx_antenna(ch),'requested':n,'got':got,'duration_ms':got/rate*1000,'errors':errors,'chunks':chunks}
(p/'capture.json').write_text(json.dumps(m,indent=2)); print(json.dumps({k:v for k,v in m.items() if k!='chunks'},indent=2))
