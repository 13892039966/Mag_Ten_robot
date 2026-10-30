"""Sequential, bounded experiments; retain physics failures and all raw records."""
import sys,subprocess,json,time,datetime
from pathlib import Path
R=Path(__file__).resolve().parents[1]
deadline=datetime.datetime.fromisoformat(json.loads((R/'tuning/task.json').read_text())['deadline_utc'].replace('Z','+00:00')).timestamp()
for name in sys.argv[1:]:
 remaining=deadline-time.time()
 if remaining<10:
  print('9-hour task deadline reached; no new experiment started',flush=True);break
 config=R/'tuning/configs'/f'{name}.json'
 out=R/'outputs'/('tune_'+name)
 if out.exists():raise FileExistsError(out)
 print('START',name,time.strftime('%Y-%m-%d %H:%M:%S'),flush=True)
 command=[sys.executable,str(R/'run.py'),'--dt','.02','--cpu','4','--until-terminal','--wall-seconds',str(min(1200.,remaining-5.)),'--no-gif','--controller-config',str(config),'--output',str(out)]
 with (R/'tuning'/f'{name}.log').open('w') as log:result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
 report=dict(name=name,parameters=json.loads(config.read_text()),exit_code=result.returncode)
 if (out/'summary.json').exists():
  report['summary']=json.loads((out/'summary.json').read_text())
  with (out/'analysis.log').open('w') as log:analysis=subprocess.run([sys.executable,str(R/'analyze.py'),str(out)],stdout=log,stderr=subprocess.STDOUT)
  report['analysis_exit_code']=analysis.returncode
  if (out/'independent_verification.json').exists():report['geometry']=json.loads((out/'independent_verification.json').read_text())
 with (R/'tuning/results.jsonl').open('a') as f:f.write(json.dumps(report)+'\n')
 print('END',json.dumps(report),flush=True)
 if report.get('summary',{}).get('station_reached'):break
