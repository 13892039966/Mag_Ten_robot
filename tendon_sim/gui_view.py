"""Qt window rendering the actual SOFA scene through Sofa.SofaGL (no physics copy)."""
import time
import numpy as np
from scipy.spatial.transform import Rotation
from PySide6.QtCore import Qt
from PySide6.QtGui import QSurfaceFormat
from PySide6.QtOpenGL import QOpenGLFunctions_2_1
from PySide6.QtOpenGLWidgets import QOpenGLWidget
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton

from .physics import Sofa, SofaRuntime
import Sofa.SofaGL


def tube_mesh(poses, radius):
    angles=np.arange(12)*2*np.pi/12
    ring=np.column_stack([np.zeros(12),radius*np.cos(angles),radius*np.sin(angles)])
    rotation=Rotation.from_quat(poses[:,3:]).as_matrix()
    points=(np.einsum('nij,kj->nki',rotation,ring)+poses[:,:3,None].transpose(0,2,1)).reshape(-1,3)
    triangles=[]
    for i in range(len(poses)-1):
        for k in range(12):
            a=i*12+k;b=i*12+(k+1)%12;c=b+12;d=a+12
            triangles.extend([[a,b,c],[a,c,d]])
    return points,triangles


class SofaViewport(QOpenGLWidget):
    def __init__(self, root, parent):
        super().__init__(parent)
        self.root=root;self.center=np.zeros(3);self.distance=1.;self.yaw=-55.;self.pitch=65.;self.last=None
        self.ready=False;self.failure=None

    def initializeGL(self):
        try:
            self.gl=QOpenGLFunctions_2_1();self.gl.initializeOpenGLFunctions()
            Sofa.SofaGL.glewInit()
            Sofa.Simulation.initVisual(self.root)
            self.ready=True
        except Exception as error:
            self.failure=error

    def paintGL(self):
        if not self.ready:return
        g=self.gl;g.glViewport(0,0,int(self.width()*self.devicePixelRatioF()),int(self.height()*self.devicePixelRatioF()))
        g.glClearColor(.96,.97,.99,1.);g.glClear(0x4000|0x0100)
        g.glEnable(0x0B71);g.glDisable(0x0B44)
        g.glMatrixMode(0x1701);g.glLoadIdentity()
        aspect=self.width()/max(1,self.height());near=.001;top=near*.4142
        g.glFrustum(-top*aspect,top*aspect,-top,top,near,20.)
        g.glMatrixMode(0x1700);g.glLoadIdentity()
        g.glEnable(0x0B50);g.glEnable(0x4000);g.glEnable(0x0BA1)
        g.glLightfv(0x4000,0x1203,[0.,0.,1.,0.])
        g.glLightfv(0x4000,0x1200,[.35,.35,.35,1.])
        g.glLightfv(0x4000,0x1201,[.85,.85,.85,1.])
        g.glLightModeli(0x0B52,1)
        g.glEnable(0x0BE2);g.glBlendFunc(0x0302,0x0303)
        g.glTranslated(0,0,-self.distance)
        g.glRotated(self.pitch,1,0,0);g.glRotated(self.yaw,0,0,1);g.glTranslated(*(-self.center))
        Sofa.SofaGL.draw(self.root)

    def mousePressEvent(self,event):self.last=event.position()
    def mouseMoveEvent(self,event):
        if self.last is not None:
            delta=event.position()-self.last;self.yaw+=delta.x()*.4;self.pitch+=delta.y()*.4;self.last=event.position();self.update()
    def mouseReleaseEvent(self,event):self.last=None
    def wheelEvent(self,event):self.distance=np.clip(self.distance*np.exp(-event.angleDelta().y()/1200),.03,10.);self.update()


class SofaWindow:
    def __init__(self, h, reference, original_mesh):
        self.h=h;self.reference=reference;self.paused=False;self.closed=False;self.last_render=0.;self.last_capture=time.monotonic()
        self.last_goal=reference.target(0)[0]
        fmt=QSurfaceFormat();fmt.setVersion(2,1);fmt.setProfile(QSurfaceFormat.CompatibilityProfile);fmt.setDepthBufferSize(24)
        QSurfaceFormat.setDefaultFormat(fmt)
        self.app=QApplication.instance() or QApplication(['Tendon SOFA'])
        for plugin in ['Sofa.GL.Component.Rendering3D','Sofa.GL.Component.Shader']:
            SofaRuntime.importPlugin(plugin)
        root=h['root'];visual=root.addChild('LiveVisuals')
        root.addObject('VisualStyle',displayFlags='showVisualModels hideBehaviorModels hideCollisionModels hideForceFields hideInteractionForceFields')
        wall=visual.addChild('RealColonWall');wall.addObject('MeshSTLLoader',name='mesh',filename=str(original_mesh),flipNormals=False)
        wall.addObject('OglModel',src='@mesh',color=[.85,.48,.42,.2])
        self.models=[]
        poses=np.asarray(h['state'].position.value);boundary=h['definition']['active_index']
        self.slices=[slice(0,boundary+1),slice(boundary,None)]
        for name,part,color in zip(['Passive','Active'],self.slices,[[.15,.42,.8,1.],[1.,.55,.08,1.]]):
            points,triangles=tube_mesh(poses[part],h['config'].radius_m)
            self.models.append(visual.addChild(name).addObject('OglModel',position=points.tolist(),triangles=triangles,color=color))
        self.goal_shape=np.array([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]])*.0015
        faces=[[0,2,4],[2,1,4],[1,3,4],[3,0,4],[2,0,5],[1,2,5],[3,1,5],[0,3,5]]
        self.goal=visual.addChild('CurrentTarget').addObject('OglModel',position=(self.goal_shape+self.last_goal).tolist(),triangles=faces,color=[.9,.05,.05,1.])
        self.window=QWidget();self.window.setWindowTitle('SOFA 25.06 | %g ms physics | waypoint arrival %g mm'%(h['config'].dt*1000,reference.arrival_radius_m*1000))
        self.window.resize(1250,850)
        layout=QVBoxLayout(self.window);self.status=QLabel('Initializing actual SOFA scene…');layout.addWidget(self.status)
        bar=QHBoxLayout();layout.addLayout(bar)
        for text,callback in [('Pause / Resume',self.toggle_pause),('Colon view',lambda:self.camera(False)),('Whole body',lambda:self.camera(True)),('Stop and save',self.stop)]:
            button=QPushButton(text);button.clicked.connect(callback);bar.addWidget(button)
        self.viewport=SofaViewport(root,self.window);layout.addWidget(self.viewport)
        self.window.closeEvent=lambda event:self.stop()
        self.camera(False)

    def camera(self,whole):
        points=np.asarray(self.h['geometry']['vertices_m'])
        if whole:points=np.vstack([points,np.asarray(self.h['state'].position.value)[:,:3]])
        self.viewport.center=(points.min(0)+points.max(0))/2
        self.viewport.distance=float(np.linalg.norm(np.ptp(points,axis=0))*(1.5 if whole else 1.25))
        self.viewport.update()

    def toggle_pause(self):self.paused=not self.paused
    def stop(self):self.closed=True
    def show(self):
        self.window.show();self.app.processEvents()
        if self.viewport.failure:raise RuntimeError('SOFA OpenGL initialization failed') from self.viewport.failure
        if not self.viewport.ready:raise RuntimeError('SOFA OpenGL viewport did not initialize')

    def process(self, trial, force=False):
        self.app.processEvents()
        now=time.monotonic()
        if not force and now-self.last_render<1/25:return
        self.last_render=now
        poses=np.asarray(self.h['state'].position.value)
        for model,part in zip(self.models,self.slices):
            points,_=tube_mesh(poses[part],self.h['config'].radius_m);model.position.value=points.tolist()
        self.last_goal=self.reference.target(trial.ticks*self.h['config'].dt)[0]
        self.goal.position.value=(self.goal_shape+self.last_goal).tolist()
        Sofa.Simulation.updateVisual(self.h['root'])
        error=np.linalg.norm(poses[-1,:3]-self.last_goal)*1000
        reason=trial.summary['reason'] if trial.done else ('PAUSED' if self.paused else 'RUNNING')
        self.status.setText('%s | simulated %.2f s | dt %.0f ms | target error %.2f mm | arrival 3 mm | feed %.1f mm | drag to rotate / wheel to zoom'%(reason,trial.ticks*self.h['config'].dt,self.h['config'].dt*1000,error,self.h['feed_m']*1000))
        self.viewport.update();self.app.processEvents()
        if now-self.last_capture>=2.:
            latest=trial.output/'gui_latest.png';temporary=trial.output/'gui_latest.tmp.png'
            self.screenshot(temporary);temporary.replace(latest);self.last_capture=now

    def screenshot(self,path):self.viewport.grabFramebuffer().save(str(path))
    def close(self):self.window.hide();self.window.deleteLater();self.app.processEvents()
