"""Offline prototype: train first 0.5 ms of every 20 ms, predict remainder.
No packet arrival metadata is used for fitting or cancelling.
"""
from pathlib import Path
import json
import numpy as np
from scipy.optimize import minimize_scalar
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path(__file__).parent
fs=737.28e6
r=np.memmap(p/'rx_ch3_rx1_40ms.sc16',dtype='<i2',mode='r').reshape(-1,2)
N=len(r); period=round(.020*fs); training=round(.0005*fs)
D=750;W=6000;B=100
idx=np.arange(0,N-W-D,B);times=(idx+W+D)/fs
out=np.memmap('/tmp/uwb_periodic_tone_cancel_20261115.cf32',dtype=np.complex64,mode='w+',shape=(N,))
def cx(a,b):
 x=r[a:b].astype(np.float64)/32768.; return x[:,0]+1j*x[:,1]
def sums(v,w):
 c=np.empty(len(v)+1,dtype=np.complex128 if np.iscomplexobj(v) else np.float64);c[0]=0;np.cumsum(v,dtype=c.dtype,out=c[1:]);return c[w:]-c[:-w]
models=[]
for start in range(0,N,period):
 end=min(N,start+period);tr=min(training,end-start);x=cx(start,start+tr)
 # FFT initialisation is confined to this cycle's first 0.5 ms.
 sp=abs(np.fft.fft(x*np.hanning(tr)))**2;k=int(np.argmax(sp)); yy=np.log(np.maximum(sp[np.array([k-1,k,k+1])%tr],1e-100));delta=.5*(yy[0]-yy[2])/(yy[0]-2*yy[1]+yy[2]);f0=(k+delta)/tr*fs
 if f0>fs/2:f0-=fs
 nn=np.arange(tr,dtype=np.float64)-(tr-1)/2
 # Frequency is optimized in Hz as an offset, avoiding a relative tolerance
 # tied to the absolute 233 MHz carrier. Complex coefficient is analytic LS.
 base=x*np.exp(-2j*np.pi*(f0/fs)*nn)
 def coeff(df):return np.mean(base*np.exp(-2j*np.pi*(df/fs)*nn))
 opt=minimize_scalar(lambda df:-abs(coeff(df))**2,bounds=(-fs/tr,fs/tr),method='bounded',options={'xatol':1e-5})
 f=f0+opt.x;c=coeff(opt.x)
 # c is referenced to the midpoint of the training window. Its complex phase
 # must be preserved; each 20 ms model predicts from this same time origin.
 center=(tr-1)/2
 for a in range(start,end,1000000):
  b=min(end,a+1000000);phase=np.remainder((f/fs)*(np.arange(a-start,b-start,dtype=np.float64)-center),1)
  pred=c*np.exp(2j*np.pi*phase);out[a:b]=cx(a,b)-pred
 models.append({'cycle_start_ms':start/fs*1000,'training_ms':tr/fs*1000,'fft_frequency_hz':f0,'frequency_hz':float(f),'amplitude':float(abs(c)),'phase_at_training_midpoint_rad':float(np.angle(c)),'optimizer_success':bool(opt.success)})
 print('model',models[-1],flush=True)
out.flush();ac=np.empty(len(idx))
for a in range(0,len(idx),10000):
 ii=idx[a:a+10000];start=int(ii[0]);end=int(ii[-1]+W+D);x=out[start:end];j=ii-start
 pp=sums(x[:-D]*x[D:].conj(),W);e1=sums(abs(x[:-D])**2,W);e2=sums(abs(x[D:])**2,W)
 ac[a:a+len(ii)]=abs(pp[j])**2/np.maximum(e1[j]*e2[j],1e-30)
np.save(p/'autocorr_periodic_cancel.npy',ac)
def groups(thr):
 ids=np.flatnonzero(ac>=thr)
 if not len(ids):return []
 return [{'start_ms':float(times[g[0]]*1000),'end_ms':float(times[g[-1]]*1000),'peak':float(ac[g].max())} for g in np.split(ids,np.flatnonzero(np.diff(times[ids])>20e-6)+1)]
a=np.load(p/'metrics.npz');notch=np.load(p/'autocorr_notched.npy')
# The following reference is for offline scoring only, never used in models.
ref=np.array(json.loads((p/'summary.json').read_text())['reference']['arrival_ms'])/1000
bg=np.ones(len(times),bool)
for rr in ref:bg &= ~((times>rr-.0004)&(times<rr+.0007))
results={'period_ms':20,'training_ms':.5,'models':models,'raw_background_ac_median':float(np.median(a['autocorr'][bg])),'clean_background_ac_median':float(np.median(ac[bg])),'thresholds':{str(thr):groups(thr) for thr in [.1,.15,.2,.3,.4,.5,.6]},'projected_tone_checks':[],'template_peak_checks':[],'boundary':{}}
# Test coherent residual of the target tone on disjoint background blocks.
for lo,hi in [(1,2),(5,6),(10,11),(15,16),(18,19),(21,22),(25,26),(30,31),(35,36),(38,39)]:
 aa=round(lo*.001*fs);bb=round(hi*.001*fs);x=cx(aa,bb);z=np.asarray(out[aa:bb],np.complex128);f=models[aa//period]['frequency_hz'];osc=np.exp(-2j*np.pi*(f/fs)*np.arange(len(x)))
 before=np.mean(x*osc);after=np.mean(z*osc)
 results['projected_tone_checks'].append({'interval_ms':[lo,hi],'suppression_db':float(20*np.log10(abs(before)/max(abs(after),1e-30)))})
# Per-burst matched-filter peak before/after, diagnostic, not EVM or FCS.
from scipy.signal import fftconvolve
s=np.fromfile('testdata/reference_preamble_code10_737p28.cf32',np.complex64);L=len(s);Et=np.vdot(s,s).real
for rr in ref:
 aa=max(0,round((rr-5e-6)*fs));bb=min(N,round((rr+40e-6)*fs));peaks=[]
 for x in [cx(aa,bb),np.asarray(out[aa:bb],np.complex128)]:
  dot=fftconvolve(x,s[::-1].conj(),mode='valid');en=sums(abs(x)**2,L);peaks.append(float(np.max(abs(dot)**2/np.maximum(en*Et,1e-30))))
 results['template_peak_checks'].append({'arrival_ms':rr*1000,'before':peaks[0],'after':peaks[1]})
b=period
for label,x in [('raw',cx(b-1,b+1)),('clean',np.asarray(out[b-1:b+1],np.complex128))]:results['boundary'][label+'_adjacent_step_abs']=float(abs(x[1]-x[0]))
# Model-switch discontinuity at the same sample, distinct from raw signal step.
c0,c1=models;nn0=(period-(training-1)/2);nn1=-(training-1)/2
v0=c0['amplitude']*np.exp(1j*(c0['phase_at_training_midpoint_rad']+2*np.pi*np.remainder(c0['frequency_hz']/fs*nn0,1)))
v1=c1['amplitude']*np.exp(1j*(c1['phase_at_training_midpoint_rad']+2*np.pi*np.remainder(c1['frequency_hz']/fs*nn1,1)))
results['boundary']['prediction_switch_abs']=float(abs(v1-v0));results['boundary']['prediction_switch_relative_to_tone']=float(abs(v1-v0)/abs(v1))
(p/'periodic_cancel.json').write_text(json.dumps(results,indent=2))
fig,axs=plt.subplots(3,1,figsize=(13,9),sharex=True)
for ax,v,title,color in zip(axs,[a['autocorr'],notch,ac],['Raw delayed autocorrelation','After IIR notch (previous experiment)','After 0.5 ms estimation / 20 ms periodic tone subtraction'],['#777777','#009988','#0077bb']):
 ax.plot(times*1000,v,lw=.6,color=color);ax.set_title(title);ax.set_ylim(0,.9);ax.set_ylabel('Normalized autocorrelation');ax.grid(alpha=.3)
 for rr in ref:ax.axvline(rr*1000,c='#cc3311',lw=.5,alpha=.4)
 for st in [0,20]:ax.axvspan(st,st+.5,color='#e6ab02',alpha=.3)
 ax.axvline(20,c='black',lw=.7,ls='--')
axs[-1].set_xlabel('Time (ms); shaded regions: model estimation');axs[-1].set_xlim(0,40)
fig.tight_layout();fig.savefig(p/'periodic_cancel_comparison.png',dpi=170);fig.savefig(p/'periodic_cancel_comparison.pdf')
print(json.dumps({k:v for k,v in results.items() if k!='thresholds'},indent=2))
for th,g in results['thresholds'].items():print('threshold',th,'groups',len(g),'times',[round(v['start_ms'],6) for v in g])
