import ctypes
from pathlib import Path
_lib=None
def library():
    global _lib
    if _lib is None:
        path=Path(__file__).resolve().parents[1]/'native/build/libTendonScheduler25.so'
        _lib=ctypes.CDLL(str(path))
        _lib.campaign_configure_default_scheduler.argtypes=[ctypes.c_uint]
        _lib.campaign_configure_default_scheduler.restype=ctypes.c_int
        _lib.campaign_last_scheduler_thread_count.argtypes=[]
        _lib.campaign_last_scheduler_thread_count.restype=ctypes.c_int
    return _lib
def seed_scheduler(n):
    if n not in (1,4):raise ValueError('Supported pool counts: 1 or 4')
    actual=int(library().campaign_configure_default_scheduler(n))
    if actual!=n:raise RuntimeError(f'Scheduler requested {n}, actual {actual}')
    return actual
def actual_thread_count():return int(library().campaign_last_scheduler_thread_count())
