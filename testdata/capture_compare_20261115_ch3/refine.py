from pathlib import Path
exec((Path(__file__).parent/'analyze.py').read_text().split('# Exact deployed energy')[0])
a=np.load(p/'metrics.npz'); raw=a['autocorr']; corr=np.empty(len(idx)); means=[]
for aa in range(0,len(idx),10000):
 ii=idx[aa:aa+10000]; start=int(ii[0]); end=int(ii[-1]+W+D); x=cx(start,end); j=ii-start
 u=x[:-D];v=x[D:]; su=sums(u,W);sv=sums(v,W); pp=sums(u*v.conj(),W)-su*sv.conj()/W
 e1=np.maximum(sums(abs(u)**2,W)-abs(su)**2/W,0);e2=np.maximum(sums(abs(v)**2,W)-abs(sv)**2/W,0)
 corr[aa:aa+len(ii)]=abs(pp[j])**2/np.maximum(e1[j]*e2[j],1e-30)
 means.append([float(x.real.mean()),float(x.imag.mean())])
at=(idx+W+D)/fs
np.save(p/'autocorr_dc_removed.npy',corr)
def groups(t,m,thr,gap=20e-6):
 ids=np.flatnonzero(m>=thr)
 if not len(ids):return []
 return [{'start_ms':float(t[g[0]]*1000),'end_ms':float(t[g[-1]]*1000),'peak':float(m[g].max())} for g in np.split(ids,np.flatnonzero(np.diff(t[ids])>gap)+1)]
res={'mean_iq_chunks':np.mean(means,axis=0).tolist(),'quantiles':np.quantile(corr,[0,.5,.9,.99,.999,1]).tolist(),'thresholds':{}}
for thr in [.005,.01,.02,.05,.1,.2,.3]:res['thresholds'][str(thr)]=groups(at,corr,thr)
(p/'refine.json').write_text(json.dumps(res,indent=2))
print('means',res['mean_iq_chunks'],'quantiles',res['quantiles'])
for t,g in res['thresholds'].items(): print(t,len(g),[round(v['start_ms'],6) for v in g][:30])
fig,axs=plt.subplots(4,1,figsize=(14,11),sharex=True)
axs[0].plot(a['energy_time']*1000,a['energy'],lw=.5);axs[0].axhline(.004,c='orange',ls='--',label='0.004');axs[0].axhline(.02,c='r',ls='--',label='auto 0.02');axs[0].set_ylabel('Energy gate');axs[0].legend()
axs[1].plot(a['autocorr_time']*1000,raw,lw=.5);axs[1].set_ylabel('Raw autocorrelation')
axs[2].plot(at*1000,corr,lw=.5);axs[2].axhline(.05,c='r',ls='--');axs[2].set_ylabel('DC-removed autocorr.')
rt=(np.arange(len(a['ref10']))+.5)*B/fs
axs[3].plot(rt*1000,a['ref10'],lw=.5,label='code 10');axs[3].plot(rt*1000,a['ref9'],lw=.5,label='code 9');axs[3].legend();axs[3].set_ylabel('Template score squared');axs[3].set_xlabel('Time (ms)')
for ax in axs:ax.grid(alpha=.3)
fig.tight_layout();fig.savefig(p/'comparison_dc_removed.png',dpi=160)
fig,axs=plt.subplots(3,1,figsize=(12,8),sharex=True)
for k in range(8):
 t0=2.183634+k*4.9998644
 mask=abs(a['energy_time']*1000-t0)<.12;axs[0].plot((a['energy_time'][mask]*1000-t0)*1000,a['energy'][mask],label=str(k))
 mask=abs(at*1000-t0)<.12;axs[1].plot((at[mask]*1000-t0)*1000,corr[mask])
 mask=abs(rt*1000-t0)<.12;axs[2].plot((rt[mask]*1000-t0)*1000,a['ref10'][mask])
axs[0].axhline(.004,c='r',ls='--');axs[1].axhline(.05,c='r',ls='--')
for ax,label in zip(axs,['Energy','DC-removed autocorr.','Code10 reference']):ax.set_ylabel(label);ax.grid(alpha=.3)
axs[2].set_xlabel('Time relative to reference arrival (us)');fig.tight_layout();fig.savefig(p/'packet_overlay.png',dpi=160)
