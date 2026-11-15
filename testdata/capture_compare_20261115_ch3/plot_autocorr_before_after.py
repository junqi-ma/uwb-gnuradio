from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path(__file__).parent
a=np.load(p/'metrics.npz'); t=a['autocorr_time']*1000
before=a['autocorr']; after=np.load(p/'autocorr_notched.npy')
s=json.loads((p/'summary.json').read_text()); arrivals=s['reference']['arrival_ms']
fig,axs=plt.subplots(3,1,figsize=(13,9),gridspec_kw={'height_ratios':[1.25,1,1]})
axs[0].plot(t,before,color='#777777',lw=.65,label='Before notch')
axs[0].plot(t,after,color='#0077bb',lw=.65,label='After notch (-232.96 MHz)')
for x in arrivals:axs[0].axvline(x,color='#cc3311',alpha=.4,lw=.65)
axs[0].set_xlim(0,40);axs[0].set_title('40 ms capture: normalized delayed autocorrelation (same IQ, same detector)');axs[0].set_xlabel('Time from capture start (ms)');axs[0].legend(loc='upper right')
for ax,start,end,title in [(axs[1],arrivals[0]-.04,arrivals[0]+.32,'First preamble'),(axs[2],arrivals[5]-.30,arrivals[5]+.32,'Sixth preamble: interference before arrival')]:
 sel=(t>=start)&(t<=end);ref=arrivals[0] if ax is axs[1] else arrivals[5]
 ax.plot((t[sel]-ref)*1000,before[sel],color='#777777',lw=1,label='Before notch')
 ax.plot((t[sel]-ref)*1000,after[sel],color='#0077bb',lw=1,label='After notch')
 ax.axvline(0,color='#cc3311',ls='--',lw=1,label='Code-10 template reference')
 ax.axhline(.15,color='#009988',ls=':',lw=1,label='Exploratory threshold 0.15')
 ax.set_title(title);ax.set_xlabel('Time relative to template reference (us)');ax.set_xlim((start-ref)*1000,(end-ref)*1000)
axs[1].legend(loc='upper right',fontsize=8)
for ax in axs:ax.set_ylim(0,.9);ax.set_ylabel(r'$|P|^2/(E_1 E_2)$');ax.grid(alpha=.25)
fig.text(.5,.012,'737.28 MS/s | lag 750 samples | integration 6000 samples | step 100 samples | causal window-end timestamps',ha='center',fontsize=9)
fig.tight_layout(rect=(0,.035,1,1));fig.savefig(p/'autocorr_before_after.png',dpi=180);fig.savefig(p/'autocorr_before_after.pdf')
