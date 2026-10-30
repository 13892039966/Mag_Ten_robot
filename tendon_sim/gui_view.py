"""Qt window rendering the actual SOFA scene through Sofa.SofaGL (no physics copy)."""
import time,json
import numpy as np
from scipy.spatial.transform import Rotation
from PySide6.QtCore import Qt
from PySide6.QtGui import QSurfaceFormat, QPainter
from PySide6.QtOpenGL import QOpenGLFunctions_2_1
from PySide6.QtOpenGLWidgets import QOpenGLWidget
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QSizePolicy

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
        self.ready=False;self.failure=None;self.meshes=[]

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
        # Draw the current physical-pose meshes directly: no stale Ogl VBO or
        # deferred mechanical mapping can leave a previous frame on screen.
        g.glPushAttrib(0x000FFFFF)
        g.glDisable(0x0B50);g.glDisable(0x0B57)
        for points,triangles,color in self.meshes:
            faces=points[triangles]
            normals=np.cross(faces[:,1]-faces[:,0],faces[:,2]-faces[:,0])
            normals/=np.maximum(np.linalg.norm(normals,axis=1)[:,None],1e-30)
            g.glColor4f(*color);g.glBegin(0x0004)
            for face,normal in zip(faces,normals):
                g.glNormal3d(*normal)
                for point in face:g.glVertex3d(*point)
            g.glEnd()
        g.glPopAttrib()

    def mousePressEvent(self,event):
        if event.button() in (Qt.LeftButton,Qt.RightButton):
            self.last=event.position();event.accept()

    def mouseMoveEvent(self,event):
        if self.last is None:return
        position=event.position();delta=position-self.last;self.last=position
        if event.buttons() & Qt.RightButton:
            # Pan in the screen plane, scaled by camera distance and viewport height.
            scale=2.*self.distance*.4142/max(1,self.height())
            rotation=Rotation.from_euler('x',self.pitch,degrees=True).as_matrix() @ Rotation.from_euler('z',self.yaw,degrees=True).as_matrix()
            self.center+=rotation.T@np.array([-delta.x()*scale,delta.y()*scale,0.])
        elif event.buttons() & Qt.LeftButton:
            self.yaw+=delta.x()*.4;self.pitch+=delta.y()*.4
        else:return
        self.update();event.accept()

    def mouseReleaseEvent(self,event):
        self.last=event.position() if event.buttons() & (Qt.LeftButton|Qt.RightButton) else None
        event.accept()

    def wheelEvent(self,event):self.distance=np.clip(self.distance*np.exp(-event.angleDelta().y()/1200),.03,10.);self.update()


class SofaWindow:
    def __init__(self, h, reference, original_mesh):
        self.h=h;self.reference=reference;self.paused=False;self.closed=False;self.last_render=0.;self.last_capture=time.monotonic()
        self.visual_checks=0;self.max_visual_error_m=0.
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
        poses=np.asarray(h['state'].position.value);boundary=h['definition']['active_index']
        self.slices=[slice(0,boundary+1),slice(boundary,None)]
        self.triangles=[np.asarray(tube_mesh(poses[part],h['config'].radius_m)[1],dtype=int) for part in self.slices]
        self.colors=[[.15,.42,.8,1.],[1.,.55,.08,1.]]
        self.goal_shape=np.array([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]])*.0015
        self.goal_faces=np.array([[0,2,4],[2,1,4],[1,3,4],[3,0,4],[2,0,5],[1,2,5],[3,1,5],[0,3,5]])
        self.window=QWidget();self.window.setWindowTitle('SOFA 25.06 | %g ms physics | waypoint arrival %g mm'%(h['config'].dt*1000,reference.arrival_radius_m*1000))
        self.window.resize(1250,850)
        layout=QVBoxLayout(self.window);layout.setContentsMargins(6,6,6,6);layout.setSpacing(4)
        bar=QHBoxLayout();layout.addLayout(bar)
        for text,callback in [('Pause / Resume',self.toggle_pause),('Colon view',lambda:self.camera(False)),('Whole body',lambda:self.camera(True)),('Stop and save',self.stop)]:
            button=QPushButton(text);button.clicked.connect(callback);bar.addWidget(button)
        self.viewport=SofaViewport(root,self.window)
        self.viewport.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Expanding)
        layout.addWidget(self.viewport,1)
        self.status=QLabel('Initializing actual SOFA scene…')
        self.status.setFixedHeight(26);self.status.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Fixed)
        layout.addWidget(self.status,0)
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
        # Minimized Qt/OpenGL composition can block ~1s per frame on this
        # desktop. Keep pumping restore/close events, but never enqueue another
        # repaint or framebuffer readback while no window is visible.
        handle=self.window.windowHandle()
        if not force and (self.window.isMinimized() or not self.window.isVisible() or (handle is not None and not handle.isExposed())):
            return
        now=time.monotonic()
        if not force and now-self.last_render<1/25:return
        self.last_render=now
        poses=np.asarray(self.h['state'].position.value)
        meshes=[]
        for part,triangles,color in zip(self.slices,self.triangles,self.colors):
            points,_=tube_mesh(poses[part],self.h['config'].radius_m)
            center_error=float(np.max(np.abs(points.reshape(-1,12,3).mean(axis=1)-poses[part,:3])))
            if center_error>1e-9:raise RuntimeError('Displayed mesh centers do not match actual physical nodes')
            self.max_visual_error_m=max(self.max_visual_error_m,center_error)
            meshes.append((points,triangles,color))
        self.last_goal=self.reference.target(trial.ticks*self.h['config'].dt)[0]
        meshes.append((self.goal_shape+self.last_goal,self.goal_faces,[.9,.05,.05,1.]))
        self.viewport.meshes=meshes
        Sofa.Simulation.updateVisual(self.h['root'])
        self.visual_checks+=1
        if trial.done:
            (trial.output/'gui_visual_verification.json').write_text(json.dumps(dict(verified=True,samples=self.visual_checks,max_center_error_m=self.max_visual_error_m,backend='SOFA wall renderer + immediate pose-derived meshes',scope='Every displayed robot mesh rebuilt from current physical poses; display refresh does not step physics'),indent=2))
        error=np.linalg.norm(poses[-1,:3]-self.last_goal)*1000
        reason=trial.summary['reason'] if trial.done else ('PAUSED' if self.paused else 'RUNNING')
        self.status.setText('%s | simulated %.2f s | dt %.0f ms | target error %.2f mm | arrival %g mm | feed %.1f mm | left drag: rotate / right drag: pan / wheel: zoom'%(reason,trial.ticks*self.h['config'].dt,self.h['config'].dt*1000,error,self.reference.arrival_radius_m*1000,self.h['feed_m']*1000))
        self.viewport.update();self.app.processEvents()
        if now-self.last_capture>=2.:
            latest=trial.output/'gui_latest.png';temporary=trial.output/'gui_latest.tmp.png'
            self.screenshot(temporary);temporary.replace(latest);self.last_capture=now

    def screenshot(self,path):self.viewport.grabFramebuffer().save(str(path))
    def screenshot_window(self,path):
        frame=self.window.grab();painter=QPainter(frame)
        painter.drawImage(self.viewport.pos(),self.viewport.grabFramebuffer());painter.end()
        frame.save(str(path))
    def close(self):self.window.hide();self.window.deleteLater();self.app.processEvents()
