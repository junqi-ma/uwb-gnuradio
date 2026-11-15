from pathlib import Path
import subprocess,os,fcntl,json,statistics,hashlib
root=Path(__file__).resolve().parents[3]
out=Path(__file__).resolve().parent
new=root/'gr-uwb/build-cfo-tail-fix/apps/uwb_dw3000_live'
old=Path('/tmp/uwb_dw3000_live_before_lock')
iq=root/'testdata/x410_fixed_tone_live_20261115/rx_ch3_rx1_1s.sc16'
variants={'old':(old,[]),'new_acquire':(new,['--sync','acquire']),
          'new_locked':(new,['--sync','locked']),
          'new_locked_fused':(new,['--sync','locked','--fused','on'])}
env=os.environ.copy();env.pop('LD_LIBRARY_PATH',None)
allruns=[]
def fields(line):return dict(p.split('=',1) for p in line.split()[1:] if '=' in p)
def stat(v):
 v=sorted(v)
 def pct(q):
  x=(len(v)-1)*q;i=int(x);return v[i]+(v[min(i+1,len(v)-1)]-v[i])*(x-i)
 return dict(n=len(v),mean=statistics.mean(v),p50=pct(.5),p95=pct(.95),p99=pct(.99),maximum=v[-1]) if v else None
with open('/tmp/uwb-sic-benchmark.lock','a') as lock:
 fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 for rep in range(3):
  order=list(variants) if rep%2==0 else list(reversed(variants))
  for name in order:
   exe,extra=variants[name];cmd=[str(exe),'--replay',str(iq),'--workers','2','--summary','1000',*extra]
   path=out/f'{name}_r{rep+1}.log';assert not path.exists()
   with path.open('w') as f:r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,env=env,timeout=30)
   assert r.returncode==0,path
   lines=path.read_text().splitlines();packets=[fields(l[l.index('PACKET '):]) for l in lines if 'PACKET ' in l]
   final=next(fields(l) for l in lines if l.startswith('FINAL '));assert len(packets)==int(final['completed'])
   measured=[p for p in packets if float(p['time_ms'])>=100 and 'compute_us' in p]
   metrics={k:stat([float(p[k]) for p in measured if k in p]) for k in ['compute_us','demod_us','resample_us','retry_us','queue_us','latency_ms','st_timing','st_cir','st_payload','st_fallback']}
   diag={k:sum(int(p.get(k,0)) for p in measured) for k in ['legacy_scans','bounded_ok','bounded_fail','backtrack','ondemand_fb','st_fb_count','full_retry']}
   row=dict(variant=name,rep=rep+1,command=cmd,final=final,metrics=metrics,diagnostics=diag)
   allruns.append(row);print(name,rep+1,final,'compute',round(metrics['compute_us']['mean'],1),flush=True)
   (out/'runs.json').write_text(json.dumps(allruns,indent=2)+'\n')
summary={}
for name in variants:
 runs=[r for r in allruns if r['variant']==name]
 summary[name]=dict(finals=[r['final'] for r in runs],
  median_run_mean={k:statistics.median(r['metrics'][k]['mean'] for r in runs) for k in runs[0]['metrics'] if runs[0]['metrics'][k]},
  median_run_p95={k:statistics.median(r['metrics'][k]['p95'] for r in runs) for k in runs[0]['metrics'] if runs[0]['metrics'][k]},
  worst_run_max={k:max(r['metrics'][k]['maximum'] for r in runs) for k in runs[0]['metrics'] if runs[0]['metrics'][k]},
  diagnostics=[r['diagnostics'] for r in runs])
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
(out/'metadata.json').write_text(json.dumps(dict(base_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),dirty=True,
 iq_sha256=hashlib.sha256(iq.read_bytes()).hexdigest(),binaries={name:hashlib.sha256(exe.read_bytes()).hexdigest() for name,(exe,_) in variants.items()},workers=2,warmup_excluded_signal_time_ms=100),indent=2)+'\n')
print(json.dumps(summary,indent=2),flush=True)
