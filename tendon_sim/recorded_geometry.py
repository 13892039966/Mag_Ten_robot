"""This portable package supports exactly its hash-checked phantom asset."""
from .geometry_adapter import load_phantom
def load_recorded_geometry(metadata):
    geometry,route=load_phantom()
    asset=metadata.get('geometry_asset',{})
    if asset.get('mesh_sha256')!=geometry['verified_sha256']['wall_world_m.stl']:
        raise ValueError('Recorded wall does not match package geometry')
    return geometry,route
