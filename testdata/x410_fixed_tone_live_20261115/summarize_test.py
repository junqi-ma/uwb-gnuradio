from pathlib import Path
import json,collections,hashlib,subprocess
import numpy as np
p=Path(__file__).parent
out={'commit':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'capture':{},'decode':{}}
for n in ['capture.json','capture_1s.json']:
 a=json.loads((p/n).read_text());ch=a['chunks'];err=[(v['at'],(v['time']-ch[0]['time'])*a['rate']-v['at']) for v in ch];out['capture'][n]={k:v for k,v in a.items() if k!='chunks'};out['capture'][n]['max_timestamp_error_samples']=max(abs(e[1]) for e in err)
def crc(x):
 c=0
 for b in x:
  for _ in range(8):
   t=(c^b)&1;c>>=1
   if t:c^=0x8408
   b>>=1
 return c
for name in ['clean_tail','raw_tail','clean_stock','raw_stock']:
 r=[json.loads(l) for l in (p/f'{name}_1s.jsonl').read_text().splitlines()];ok=[x for x in r if x['fcs']];ind=0
 for x in ok:
  b=bytes.fromhex(x['hex']);ind+=crc(b[:-2])==int.from_bytes(b[-2:],'little')
 out['decode'][name]={'windows':len(r),'status_histogram':dict(collections.Counter(x['status'] for x in r)),'timing_ok':sum(x['timing_ok'] for x in r),'sfd_ok':sum(x['sfd_ok'] for x in r),'phr_ok':sum(x['phr_ok'] for x in r),'fcs_pass':len(ok),'independent_crc_pass':ind,'lengths':dict(collections.Counter(x['length'] for x in ok)),'diagnostic_service_us_quantiles':dict(zip(['min','p50','p95','max'],np.quantile([x['us'] for x in r],[0,.5,.95,1]).tolist()))}
r=[json.loads(l) for l in (p/'clean_tail_1s.jsonl').read_text().splitlines()];t=np.array([x['seed']-14746+x['detected']*48/65 for x in r]);fit=np.polyfit(np.arange(len(t)),t,1);res=t-np.polyval(fit,np.arange(len(t)));out['arrival']={'qualified_count':len(t),'candidate_count':len(json.loads((p/'arrival_1s.json').read_text())),'fit_period_ms':fit[0]/737280,'fit_residual_std_us':float(np.std(res)/737.28),'fit_residual_max_abs_us':float(max(abs(res))/737.28),'raw_template_metric_median':float(np.median([x['timing_metric'] for x in [json.loads(l) for l in (p/'raw_tail_1s.jsonl').read_text().splitlines()]])),'clean_template_metric_median':float(np.median([x['timing_metric'] for x in r]))}
h=hashlib.sha256()
with (p/'rx_ch3_rx1_1s.sc16').open('rb') as f:
 for block in iter(lambda:f.read(8*1024*1024),b''):h.update(block)
out['capture']['raw_1s_sha256']=h.hexdigest()
(p/'summary.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
