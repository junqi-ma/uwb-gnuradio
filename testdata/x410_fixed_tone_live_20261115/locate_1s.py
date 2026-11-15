from pathlib import Path
import numpy as np,json
p=Path(__file__).parent;fs=737280000
x=np.memmap(p/'clean_1s.sc16',dtype=np.int16).reshape(-1,2)
D=750;W=6000;step=100;ts=[];ms=[]
def sums(v):
 s=np.r_[0,np.cumsum(v)];return s[W:]-s[:-W]
for a in range(0,len(x)-W-D,1000000):
 b=min(len(x),a+1000000+W+D);v=x[a:b].astype(np.float32);z=(v[:,0]+1j*v[:,1])/32768
 u=z[:-D];v=z[D:];q=sums(u*v.conj());e=sums(abs(u)**2)*sums(abs(v)**2);m=abs(q)**2/np.maximum(e,1e-20)
 ids=np.arange(0,min(1000000,len(m)),step);ts.extend((a+ids).tolist());ms.extend(m[ids].tolist())
t=np.array(ts);m=np.array(ms);ids=np.flatnonzero(m>.15);gs=np.split(ids,np.flatnonzero(np.diff(t[ids])>30000)+1) if len(ids) else []
rows=[dict(start=int(t[g[0]]),end=int(t[g[-1]]+W+D),peak=float(m[g].max())) for g in gs]
(p/'starts_1s.txt').write_text(''.join(str(r['start'])+'\n' for r in rows));(p/'arrival_1s.json').write_text(json.dumps(rows,indent=2));keep=[r for r in rows if r['peak']>.5 and r['end']-r['start']>36864];(p/'packet_starts_1s.txt').write_text(''.join(str(r['start'])+'\n' for r in keep)); print('candidates',len(rows),'qualified_packets',len(keep))
