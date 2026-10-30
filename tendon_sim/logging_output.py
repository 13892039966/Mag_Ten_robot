import os,threading,time,queue
from pathlib import Path
class AsyncKeyboardOutput:
    """Neither slow disk nor a selected Windows console may stop input/GUI."""
    def __init__(self,path,console_writer=None,file_writer=None,capacity=1024):
        if capacity<1:raise ValueError('Expected positive log queue capacity')
        self.path=Path(path);self.file_queue=queue.Queue(capacity);self.console_queue=queue.Queue(capacity)
        self.dropped_file=0;self.dropped_console=0;self.errors_file=0;self.errors_console=0;self.stopping=threading.Event()
        self.threads=[]
        for name,q,writer in [('file',self.file_queue,file_writer or self._file),('console',self.console_queue,console_writer or self._console)]:
            thread=threading.Thread(target=self._drain,args=(name,q,writer),name='colon-keyboard-'+name+'-log',daemon=True);thread.start();self.threads.append(thread)
    def _file(self,text):
        with self.path.open('a',encoding='utf-8') as f:f.write(text)
    def _console(self,text):os.write(1,text.encode('utf-8'))
    def _drain(self,name,q,writer):
        while not self.stopping.is_set() or not q.empty():
            try:text=q.get(timeout=.05)
            except queue.Empty:continue
            try:writer(text)
            except Exception:setattr(self,'errors_'+name,getattr(self,'errors_'+name)+1)
            finally:q.task_done()
    def __call__(self,message):
        text=time.strftime('%H:%M:%S')+' '+message+'\n'
        for name,q in [('file',self.file_queue),('console',self.console_queue)]:
            try:q.put_nowait(text)
            except queue.Full:setattr(self,'dropped_'+name,getattr(self,'dropped_'+name)+1)
    def close(self):
        self.stopping.set()
        for thread in self.threads:thread.join(timeout=.15)

