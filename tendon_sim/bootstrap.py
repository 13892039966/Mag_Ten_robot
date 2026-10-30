PLUGIN_NAMES = ['BeamAdapter','SoftRobots','Sofa.Component.AnimationLoop',
             'Sofa.Component.Collision.Detection.Algorithm','Sofa.Component.Collision.Detection.Intersection',
             'Sofa.Component.Collision.Geometry','Sofa.Component.Collision.Response.Contact',
             'Sofa.Component.Constraint.Lagrangian.Solver','Sofa.Component.Constraint.Lagrangian.Correction',
             'Sofa.Component.Constraint.Projective','Sofa.Component.IO.Mesh',
             'Sofa.Component.StateContainer','Sofa.Component.Topology.Container.Constant',
             'Sofa.Component.Topology.Container.Dynamic','Sofa.Component.Mapping.NonLinear',
             'Sofa.Component.Mapping.Linear','Sofa.Component.Mass','Sofa.Component.ODESolver.Backward',
             'Sofa.Component.LinearSolver.Direct','Sofa.Component.SolidMechanics.Spring',
             'Sofa.Component.MechanicalLoad','Sofa.Component.Visual']

def load_sofa():
    import Sofa
    import SofaRuntime
    import Sofa.Simulation
    for name in PLUGIN_NAMES:SofaRuntime.importPlugin(name)
    return Sofa,SofaRuntime
