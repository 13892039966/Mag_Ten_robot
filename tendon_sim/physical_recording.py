"""Lossless, bounded-buffer records of every physical step (no state repair)."""
import hashlib
import json
import os
from pathlib import Path
import tempfile

import numpy as np


def _sha(path):
    h = hashlib.sha256()
    with open(str(path), 'rb') as source:
        for block in iter(lambda: source.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def _atomic(path, writer):
    fd, temporary = tempfile.mkstemp(prefix=path.name + '.', dir=str(path.parent))
    try:
        with os.fdopen(fd, 'wb') as target:
            writer(target)
            target.flush()
            os.fsync(target.fileno())
        os.replace(temporary, str(path))
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


class ChunkRecorder:
    def __init__(self, output_dir, dt, metadata, node_count, chunk_frames=200):
        self.path = Path(output_dir)
        self.path.mkdir(parents=True, exist_ok=True)
        if (self.path / 'manifest.json').exists():
            raise FileExistsError('manifest already exists')
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be positive and finite')
        if not isinstance(node_count, int) or node_count <= 0:
            raise ValueError('node_count must be a positive integer')
        if not isinstance(chunk_frames, int) or not 1 <= chunk_frames <= 200:
            raise ValueError('chunk_frames must be between 1 and 200')
        self.limit = chunk_frames
        self.buffer = []
        self.closed = False
        self.manifest = dict(dt=float(dt), node_count=node_count, total_frames=0,
                             complete=False, reason='recording', metadata=json.loads(json.dumps(metadata)),
                             chunks=[], initial_state_sha256=None)
        self._manifest()

    def _manifest(self):
        data = json.dumps(self.manifest, indent=2, allow_nan=False).encode('utf-8')
        _atomic(self.path / 'manifest.json', lambda f: f.write(data))

    def append(self, t, poses, velocities, commands):
        if self.closed:
            raise RuntimeError('recorder is closed')
        expected = self.manifest['total_frames'] * self.manifest['dt']
        if not np.isfinite(t) or abs(float(t) - expected) > 1e-9:
            raise ValueError('time must start at zero and advance by dt without gaps')
        arrays = []
        for value, shape in zip((poses, velocities, commands),
                                ((self.manifest['node_count'], 7), (self.manifest['node_count'], 6), (5,))):
            a = np.array(value, dtype=np.float64, copy=True)
            if a.shape != shape or not np.isfinite(a).all():
                raise ValueError('invalid state shape or nonfinite values')
            arrays.append(a)
        if self.manifest['total_frames'] == 0:
            h = hashlib.sha256()
            for a in arrays:
                h.update(a.astype('<f8', copy=False).tobytes(order='C'))
            self.manifest['initial_state_sha256'] = h.hexdigest()
        self.buffer.append((float(t),) + tuple(arrays))
        self.manifest['total_frames'] += 1
        if len(self.buffer) >= self.limit:
            self._flush()

    def _flush(self):
        if not self.buffer:
            return
        count = len(self.buffer)
        filename = 'chunk_{:06d}.npz'.format(len(self.manifest['chunks']))
        target = self.path / filename
        values = dict(zip(('t', 'poses', 'velocities', 'commands'),
                          [np.stack(items) for items in zip(*self.buffer)]))
        _atomic(target, lambda f: np.savez_compressed(f, **values))
        self.manifest['chunks'].append(dict(file=filename,
            start_index=self.manifest['total_frames'] - count, count=count, sha256=_sha(target)))
        self.buffer.clear()
        self._manifest()

    def close(self, complete=False, reason='stopped'):
        if not self.closed:
            self._flush()
            self.manifest.update(complete=bool(complete), reason=str(reason))
            self._manifest()
            self.closed = True
        return json.loads(json.dumps(self.manifest))


class RecordingReader:
    def __init__(self, path):
        path = Path(path)
        manifest_path = path / 'manifest.json' if path.is_dir() else path
        self.path = manifest_path.parent
        self.manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        self.metadata = self.manifest['metadata']
        self.frame_count = self.manifest['total_frames']
        self.dt = self.manifest['dt']
        self.complete = self.manifest['complete']
        self._cached_index = None
        self._cached = None
        start = 0
        for chunk in self.manifest['chunks']:
            if chunk['start_index'] != start or not 1 <= chunk['count'] <= 200:
                raise ValueError('invalid chunk index or count')
            start += chunk['count']
        if start != self.frame_count or not np.isfinite(self.dt) or self.dt <= 0:
            raise ValueError('manifest frame count or dt invalid')

    def _load(self, index):
        if self._cached_index == index:
            return self._cached
        chunk = self.manifest['chunks'][index]
        filename = chunk['file']
        if Path(filename).name != filename:
            raise ValueError('chunk filename must be local')
        path = self.path / filename
        if _sha(path) != chunk['sha256']:
            raise ValueError('chunk sha256 mismatch')
        with np.load(str(path), allow_pickle=False) as archive:
            data = {k: archive[k] for k in ('t', 'poses', 'velocities', 'commands')}
        count, nodes = chunk['count'], self.manifest['node_count']
        for key, shape in [('t', (count,)), ('poses', (count, nodes, 7)),
                           ('velocities', (count, nodes, 6)), ('commands', (count, 5))]:
            if data[key].shape != shape or not np.isfinite(data[key]).all():
                raise ValueError('chunk shape or finite-value check failed')
        expected = np.arange(chunk['start_index'], chunk['start_index'] + count) * self.dt
        if not np.all(np.abs(data['t'] - expected) <= 1e-9):
            raise ValueError('discontinuous physical time')
        self._cached_index, self._cached = index, data
        return data

    def frame(self, i):
        if not isinstance(i, (int, np.integer)):
            raise TypeError('frame index must be an integer')
        if not 0 <= i < self.frame_count:
            raise IndexError('frame index out of range')
        for index, chunk in enumerate(self.manifest['chunks']):
            if chunk['start_index'] <= i < chunk['start_index'] + chunk['count']:
                data = self._load(index)
                local = i - chunk['start_index']
                return {k: float(v[local]) if k == 't' else v[local].copy() for k, v in data.items()}
        raise ValueError('missing chunk')

    def verify(self):
        # Reload even the cached chunk so on-disk tampering is checked again.
        self._cached_index = self._cached = None
        for index in range(len(self.manifest['chunks'])):
            self._load(index)
        return True
