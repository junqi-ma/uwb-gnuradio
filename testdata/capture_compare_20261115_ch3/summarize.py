import json,numpy as np
from pathlib import Path
p=Path(__file__).parent
m=json.loads((p/'analysis.json').read_text());q=json.loads((p/'notch.json').read_text()); ref=m['reference']['10:0.1'];rt=np.array([x['start_ms'] for x in ref]); results={}
for name,g in [('energy_default_0.02',m['energy']['0.02']),('energy_0.004',m['energy']['0.004']),('raw_autocorr_0.1',m['autocorr']['0.1']),('notched_autocorr_0.15',q['thresholds']['0.15']),('notched_autocorr_0.2',q['thresholds']['0.2'])]:
 t=np.array([x['start_ms'] for x in g]);hits=[];assigned=set()
 for i,r in enumerate(rt):
  ids=np.flatnonzero(abs(t-r)<.5)
  if len(ids):j=ids[np.argmin(abs(t[ids]-r))];hits.append((i,j));assigned.add(int(j))
 err=np.array([(t[j]-rt[i])*1000 for i,j in hits]);arr=np.array([t[j] for i,j in hits]);k=np.array([i for i,j in hits]);fit=np.polyfit(k,arr,1) if len(k)>1 else [float('nan')]*2;resid=(arr-np.polyval(fit,k))*1000
 results[name]={'candidate_groups':len(g),'matched_reference_bursts':len(hits),'unmatched_groups':len(g)-len(assigned),'arrival_ms':arr.tolist(),'offset_from_reference_us':err.tolist(),'offset_std_us':float(err.std()) if len(err) else None,'max_abs_offset_us':float(abs(err).max()) if len(err) else None,'fitted_period_ms':float(fit[0]),'fitted_residual_std_us':float(resid.std()) if len(err) else None,'adjacent_interval_ms':np.diff(arr).tolist()}
results['reference']={'arrival_ms':rt.tolist(),'fitted_period_ms':float(np.polyfit(np.arange(8),rt,1)[0]),'note':'code10 normalized template correlation squared >=0.1, grouped gaps <=20us; bin width 100 native samples=0.1356us; not injected truth or FCS verification'}
(p/'summary.json').write_text(json.dumps(results,indent=2,allow_nan=True));print(json.dumps(results,indent=2))
