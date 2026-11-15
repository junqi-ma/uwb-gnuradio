import json
from pathlib import Path
import numpy as np
from scipy.signal import fftconvolve,find_peaks
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path(__file__).parent
meta=json.loads((p/'capture.json').read_text()); fs=meta['rate']
r=np.memmap(p/'rx_ch3_rx1_40ms.sc16',dtype='<i2',mode='r').reshape(-1,2)
N=len(r); B=100; D=round(1016*fs/998.4e6); W=8*D
idx=np.arange(0,N-W-D,B); ac=np.empty(len(idx)); energy=np.empty(N//B)
def cx(a,b):
 t=np.asarray(r[a:b],np.float32)/32768.; return t[:,0]+1j*t[:,1]
def sums(v,w):
 c=np.empty(len(v)+1,dtype=np.complex128 if np.iscomplexobj(v) else np.float64); c[0]=0; np.cumsum(v,dtype=c.dtype,out=c[1:]); return c[w:]-c[:-w]
# Exact deployed energy metric: first 16 samples per 100, trailing 32-block average.
for a in range(0,N//B,10000):
 b=min(N//B,a+10000); t=np.asarray(r[a*B:b*B],np.float64).reshape(-1,B,2)[:,:16]; energy[a:b]=(t*t).sum(axis=(1,2))/32768**2
energy=sums(energy,32)/32; et=(np.arange(len(energy))+32)*B/fs
# Eight repeated symbols, lag nearest native symbol period. No resampling, no period prior.
for a in range(0,len(idx),10000):
 ii=idx[a:a+10000]; start=int(ii[0]); end=int(ii[-1]+W+D)
 x=cx(start,end); z=x[:-D]*x[D:].conj(); pp=sums(z,W); e1=sums(abs(x[:-D])**2,W); e2=sums(abs(x[D:])**2,W); j=ii-start
 ac[a:a+len(ii)]=abs(pp[j])**2/np.maximum(e1[j]*e2[j],1e-30)
at=(idx+W+D)/fs
# Independent template cross-correlation reference, evaluated over ALL samples.
refs={}
for code in [9,10]:
 t=np.fromfile(f'testdata/reference_preamble_code{code}_737p28.cf32',np.complex64); L=len(t); out=np.zeros(N//B)
 for a in range(0,N-L+1,1000000):
  end=min(N-L+1,a+1000000); x=cx(a,end+L-1); dot=fftconvolve(x,t[::-1].conj(),mode='valid'); e=sums(abs(x)**2,L); m=abs(dot)**2/np.maximum(e*np.vdot(t,t).real,1e-30)
  jj=np.arange(a,end)//B; np.maximum.at(out,jj,m)
 refs[code]=out
 print('reference',code,'max',out.max(),flush=True)
np.savez_compressed(p/'metrics.npz',energy=energy,energy_time=et,autocorr=ac,autocorr_time=at,ref9=refs[9],ref10=refs[10])
def groups(t,m,thr,gap=20e-6):
 ids=np.flatnonzero(m>=thr)
 if not len(ids):return []
 splits=np.flatnonzero(np.diff(t[ids])>gap)+1
 return [{'start_ms':float(t[g[0]]*1000),'end_ms':float(t[g[-1]]*1000),'peak':float(m[g].max())} for g in np.split(ids,splits)]
result={'lag_samples':D,'true_symbol_samples':1016*fs/998.4e6,'window_samples':W,'energy_quantiles':np.quantile(energy,[0,.5,.9,.99,.999,1]).tolist(),'autocorr_quantiles':np.quantile(ac,[0,.5,.9,.99,.999,1]).tolist(),'energy':{},'autocorr':{},'reference':{}}
for thr in [.02,.004,.001,.0001]:result['energy'][str(thr)]=groups(et,energy,thr)
for thr in [.01,.02,.05,.1,.2,.3]:result['autocorr'][str(thr)]=groups(at,ac,thr)
for code in refs:
 for thr in [.05,.1,.2]:result['reference'][f'{code}:{thr}']=groups((np.arange(len(refs[code]))+.5)*B/fs,refs[code],thr)
(p/'analysis.json').write_text(json.dumps(result,indent=2))
fig,axs=plt.subplots(3,1,figsize=(14,9),sharex=True)
axs[0].plot(et*1e3,energy,lw=.5);axs[0].set_yscale('log');axs[0].axhline(.02,c='r',ls='--',label='auto default 0.02');axs[0].axhline(.004,c='orange',ls='--',label='detector default 0.004');axs[0].set_ylabel('Energy gate');axs[0].legend()
axs[1].plot(at*1e3,ac,lw=.5);axs[1].axhline(.05,c='r',ls='--');axs[1].set_ylabel('Normalized autocorrelation')
for code in refs:axs[2].plot((np.arange(len(refs[code]))+.5)*B/fs*1e3,refs[code],lw=.5,label=f'code {code}')
axs[2].legend();axs[2].set_ylabel('Template correlation squared');axs[2].set_xlabel('Time (ms)')
for ax in axs:ax.grid(alpha=.3)
fig.tight_layout();fig.savefig(p/'comparison.png',dpi=160)
print(json.dumps(result,indent=2))
