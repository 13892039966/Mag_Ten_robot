"""Linux-only native preflight. Does not animate physics."""
import os,sys,json,platform,re
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
def main():
    if sys.platform!='linux' or platform.machine()!='x86_64':raise RuntimeError('Ubuntu/Linux x86_64 target required')
    if sys.version_info[:2]!=(3,12):raise RuntimeError('Selected SofaPython3 ABI requires CPython3.12')
    prefix=Path(os.environ['CONDA_PREFIX']).resolve()
    versions=list(prefix.rglob('Sofa.ConfigConfigVersion.cmake'))
    if not versions or not any(re.search(r'PACKAGE_VERSION\s+"?25\.0?6(?:\.0+)?["\s)]',p.read_text(errors='replace')) for p in versions):raise RuntimeError('Cannot verify SOFA25.06 SDK')
    from tendon_sim.bootstrap import load_sofa
    Sofa,SofaRuntime=load_sofa();SofaRuntime.importPlugin('MultiThreading')
    for module in [Sofa,SofaRuntime]:
        if prefix not in Path(module.__file__).resolve().parents:raise RuntimeError('Foreign SOFA module '+str(module.__file__))
    from tendon_sim.scheduler import seed_scheduler,actual_thread_count
    seed_scheduler(4);assert actual_thread_count()==4
    if len(os.sched_getaffinity(0))<4:raise RuntimeError('Fewer than4 allowed logical CPUs')
    node=Sofa.Core.Node('PreflightNoPhysics')
    node.addObject('GenericConstraintSolver');node.addObject('ParallelBVHNarrowPhase',nbThreads=4)
    node.addObject('LocalMinDistance');node.addChild('OriginalColon')
    from tendon_sim.libigl_wall import replace_wall_pre_init,DLL,get_hybrid_stats
    replace_wall_pre_init(node)
    result=dict(status='NATIVE_PREFLIGHT_ONLY',physics_steps=0,python=sys.version,platform=platform.platform(),
        sofa_prefix=str(prefix),libigl_adapter=str(DLL),actual_pool=actual_thread_count(),query_stats=get_hybrid_stats(),
        logical_cpus=os.cpu_count(),allowed_cpus=sorted(os.sched_getaffinity(0)))
    print(json.dumps(result,indent=2));(ROOT/'.install/doctor.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
if __name__=='__main__':main()
