from pathlib import Path
import json,statistics
p=Path(__file__).parent
out={}
for filename in ['results.txt','results_repeat_cpu7.txt']:
 data={}
 for line in (p/filename).read_text().splitlines():
  v=line.split(',')
  if len(v)!=8 or not v[0].isdigit():continue
  chunk,session,group=map(int,v[:3]);a1,b1,b2,a2,ratio=map(float,v[3:])
  data.setdefault(chunk,[]).append((session,group,(a1+a2)/2,(b1+b2)/2,ratio))
 summary={}
 for chunk,rows in data.items():
  summary[str(chunk)]={'paired_groups':len(rows),'median_baseline_us':statistics.median(x[2] for x in rows),'median_fused_us':statistics.median(x[3] for x in rows),'median_paired_change_pct':100*(statistics.median(x[4] for x in rows)-1),'session_median_paired_change_pct':[100*(statistics.median(x[4] for x in rows if x[0]==s)-1) for s in range(3)]}
 out[filename]=summary
(p/'summary.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
