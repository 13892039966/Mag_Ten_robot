import ast,json,re
from pathlib import Path
root=Path(__file__).resolve().parents[1];issues=[];imports=set()
for p in root.rglob('*'):
 if not p.is_file():continue
 if p.suffix.lower() in {'.dll','.exe','.pyd','.pyc'}:issues.append(str(p.relative_to(root)))
 if p.suffix=='.py':
  s=p.read_text(encoding='utf-8-sig');tree=ast.parse(s)
  for n in ast.walk(tree):
   if isinstance(n,ast.Import):imports.update(x.name.split('.')[0] for x in n.names)
   elif isinstance(n,ast.ImportFrom) and n.module and not n.level:imports.add(n.module.split('.')[0])
  if re.search(r'(?i)[a-z]:[\\/]',s):issues.append('absolute drive path '+str(p.relative_to(root)))
print(json.dumps(dict(import_roots=sorted(imports),issues=issues),indent=2));raise SystemExit(bool(issues))
