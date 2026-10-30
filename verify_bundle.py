"""Offline content verification, no SOFA import."""
import hashlib,json
from pathlib import Path
def main():
    root=Path(__file__).resolve().parent;manifest=json.loads((root/'BUNDLE_SHA256.json').read_text(encoding='utf-8'))
    for name,expected in manifest.items():
        p=(root/name).resolve()
        if root not in p.parents:raise ValueError('Invalid manifest path')
        actual=hashlib.sha256(p.read_bytes()).hexdigest()
        if actual!=expected:raise RuntimeError('SHA mismatch: '+name)
    print('VERIFIED',len(manifest),'files. This does not compile or run Linux physics.')
if __name__=='__main__':main()
