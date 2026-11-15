from pathlib import Path
exec((Path(__file__).parent/'analyze.py').read_text().split('# Exact deployed energy')[0])
from scipy.signal import lfilter
# Estimate strongest background spectral line, diagnostic only, not packet arrival truth.
x=cx(round(.001*fs),round(.001*fs)+262144);sp=abs(np.fft.fft(x*np.hanning(len(x))))**2;k=int(np.argmax(sp)); y=np.log(sp[(np.array([k-1,k,k+1]))%len(x)]);delta=.5*(y[0]-y[2])/(y[0]-2*y[1]+y[2]);f=(k+delta)/len(x)*fs
if f>fs/2:f-=fs
z=np.exp(2j*np.pi*f/fs);b=np.array([1,-z]);den=np.array([1,-.995*z]);zi=np.zeros(1,complex)
filt=np.memmap(p/'notched.cf32',dtype=np.complex64,mode='w+',shape=(N,))
for start in range(0,N,1000000):
 end=min(N,start+1000000);yy,zi=lfilter(b,den,cx(start,end),zi=zi);filt[start:end]=yy
filt.flush();ac=np.empty(len(idx))
for aa in range(0,len(idx),10000):
 ii=idx[aa:aa+10000];start=int(ii[0]);end=int(ii[-1]+W+D);x=filt[start:end];j=ii-start
 pp=sums(x[:-D]*x[D:].conj(),W);e1=sums(abs(x[:-D])**2,W);e2=sums(abs(x[D:])**2,W)
 ac[aa:aa+len(ii)]=abs(pp[j])**2/np.maximum(e1[j]*e2[j],1e-30)
at=(idx+W+D)/fs;np.save(p/'autocorr_notched.npy',ac)
def groups(t,m,thr,gap=20e-6):
 ids=np.flatnonzero(m>=thr)
 if not len(ids):return []
 return [{'start_ms':float(t[g[0]]*1000),'end_ms':float(t[g[-1]]*1000),'peak':float(m[g].max())} for g in np.split(ids,np.flatnonzero(np.diff(t[ids])>gap)+1)]
res={'notch_freq_hz':f,'pole_radius':.995,'quantiles':np.quantile(ac,[0,.5,.9,.99,.999,1]).tolist(),'thresholds':{}}
for thr in [.02,.05,.1,.15,.2,.25,.3,.4,.5,.6]:res['thresholds'][str(thr)]=groups(at,ac,thr)
(p/'notch.json').write_text(json.dumps(res,indent=2));print('notch',f,'quantiles',res['quantiles'])
for t,g in res['thresholds'].items():print(t,len(g),[round(v['start_ms'],6) for v in g][:30])
a=np.load(p/'metrics.npz');fig,axs=plt.subplots(3,1,figsize=(14,8),sharex=True)
axs[0].plot(a['energy_time']*1000,a['energy'],lw=.5);axs[0].axhline(.004,c='r',ls='--');axs[0].set_ylabel('Energy (raw)')
axs[1].plot(at*1000,ac,lw=.5);axs[1].axhline(.2,c='r',ls='--');axs[1].set_ylabel('Autocorrelation (notched)')
rt=(np.arange(len(a['ref10']))+.5)*B/fs;axs[2].plot(rt*1000,a['ref10'],lw=.5);axs[2].set_ylabel('Code10 reference');axs[2].set_xlabel('Time (ms)')
for ax in axs:ax.grid(alpha=.3)
fig.tight_layout();fig.savefig(p/'comparison_notched.png',dpi=160)
