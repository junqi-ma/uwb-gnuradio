from pathlib import Path
import json
import numpy as np
p=Path(__file__).parent;fs=737.28e6
r=np.memmap(p/'rx_ch3_rx1_40ms.sc16',dtype='<i2').reshape(-1,2)
f0=json.loads((p/'notch.json').read_text())['notch_freq_hz'];B=32768
n=np.arange(B,dtype=np.float64);base=np.exp(-2j*np.pi*(f0/fs)*n); c=[]; t=[]
for a in range(0,len(r)-B+1,B):
 x=r[a:a+B].astype(np.float64)/32768.;x=x[:,0]+1j*x[:,1]
 # Coefficient in the f0 frame, origin at each block's center.
 cc=np.mean(x*base)*np.exp(-2j*np.pi*np.remainder(f0*a/fs,1))
 c.append(cc);t.append((a+B/2)/fs)
c=np.array(c);t=np.array(t)
ref=np.array(json.loads((p/'summary.json').read_text())['reference']['arrival_ms'])/1000
bg=np.ones(len(t),bool)
for a in ref:bg &= ~((t>a-.0004)&(t<a+.0007))
phase=np.unwrap(np.angle(c));train=bg&(t<.0015)&(t>.0001)
fit=np.polyfit(t[train],phase[train],1);df=fit[0]/(2*np.pi)
coef=np.mean(c[train]*np.exp(-1j*fit[0]*t[train]));pred=coef*np.exp(1j*fit[0]*t)
# This is diagnostic residual of blockwise tone projections, not complete IQ or UWB EVM.
res={'block_samples':B,'block_us':B/fs*1e6,'f0_fft_hz':f0,'training_interval_ms':[.1,1.5],'training_blocks':int(train.sum()),'fit_frequency_hz':float(f0+df),'training_amplitude':float(abs(coef)),'background_block_amplitude_p5_p50_p95':np.quantile(abs(c[bg]),[.05,.5,.95]).tolist(),'prediction_intervals':[]}
for lo,hi in [(0.1,1.5),(5,10),(15,20),(30,40)]:
 mask=bg&(t>=lo/1000)&(t<hi/1000)
 residual=c[mask]-pred[mask]
 res['prediction_intervals'].append({'interval_ms':[lo,hi],'projected_tone_suppression_db':float(10*np.log10(np.mean(abs(c[mask])**2)/np.mean(abs(residual)**2))),'phase_error_std_rad':float(np.std(phase[mask]-fit[0]*t[mask]-np.angle(coef)))})
# Retrospective fit over all background, marks stationary model's upper potential, not causal.
fitall=np.polyfit(t[bg],phase[bg],1);ca=np.mean(c[bg]*np.exp(-1j*fitall[0]*t[bg]));pa=ca*np.exp(1j*fitall[0]*t[bg])
res['retrospective_all_background_frequency_hz']=float(f0+fitall[0]/(2*np.pi));res['retrospective_projected_tone_suppression_db']=float(10*np.log10(np.mean(abs(c[bg])**2)/np.mean(abs(c[bg]-pa)**2)))
np.savez(p/'tone_model.npz',time=t,coefficient=c,prediction=pred,background=bg)
(p/'tone_model.json').write_text(json.dumps(res,indent=2));print(json.dumps(res,indent=2))
