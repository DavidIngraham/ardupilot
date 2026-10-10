"""Sequential native experiments; each case records its full configuration."""
import os,json,subprocess,sys
from pathlib import Path
root=Path(__file__).resolve().parents[3]
out=Path('/home/david/repos/pg-longitudinal-design');out.mkdir(exist_ok=True)
cases=json.loads(Path(sys.argv[1]).read_text())
for name,config in cases.items():
 folder=out/name;folder.mkdir(exist_ok=True);p=folder/'config.json';p.write_text(json.dumps(config,indent=2))
 env=os.environ|{'BUILDLOGS':str(folder),'PG_STUDY_CONFIG':str(p),'PG_PROFILE':config.get('profile','mission')}
 with (out/(name+'.log')).open('w') as log:
  result=subprocess.run([sys.executable,'-u',str(root/'Tools/simulation/paraglider/longitudinal_autotest.py'),'--no-check-parameter-leaks','--speedup','10','test.Plane.ParagliderLongitudinalStudy'],cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT)
 print(name,result.returncode,flush=True)
 if result.returncode:sys.exit(result.returncode)
