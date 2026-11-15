"""Fixed visible-fixture reference for two genuinely open hinges of the full rule.

The terminal leaf must fold onto the previous leaf, then the resulting pair is
regrasped and folded onto the packet. No simulator poses/angles drive this plan.
All waypoints derive solely from the documented reset dimensions.
"""
import math
import numpy as np


def make_reference(t):
    from yam_arena.demo import Waypoint
    O,C=t.FINGER_OPEN,t.FINGER_CLOSED
    right_yaw=math.pi/2
    left=[];right=[]
    def add(arm,duration,p,finger,label,yaw=0.):
        (left if arm=='left' else right).append(Waypoint(duration,tuple(p),finger,label,yaw))
    # Relative to BASE, the8-leaf packet occupies y[-.20,0]. The two
    # open leaves extend to y+.40, within the actual1.3m tabletop.
    hold=t.BASE+t.R0@np.array([.18,0,t.TIP+.001])
    pivot9=t.BASE+t.R0@np.array([-t.PITCH,0,9*t.LAYER])
    start9=pivot9+np.array([0,.10,t.TIP-.012])
    for duration,dz,fg,label in [(2.,.04,O,'above terminal leaf'),(2.,0.,O,'descend to real leaf'),(1.,0.,C,'close on terminal leaf')]:
        add('left',duration,start9+[0,0,dz],fg,label,-math.pi/2)
    for duration,dz,fg,label in [(2.,.04,O,'above8-leaf packet'),(2.,0.,O,'descend around packet'),(1.,0.,C,'hold real packet')]:
        add('right',duration,hold+[0,0,dz],fg,label,right_yaw)
    for k in range(1,37):
        a=math.pi*k/36
        p=pivot9+np.array([-.10*math.sin(a),.10*math.cos(a),t.TIP-.012])
        add('left',.25,p,C,'fold terminal native hinge',-math.pi/2+a)
        add('right',.25,hold,C,'support packet',right_yaw)
    end9=pivot9+np.array([0,-.10,t.TIP-.012])
    # Regrasp actual overlapping leaves8/9 with the parallel jaw reversed;
    # their physical joint stays closed by the passive spring/detent, not a weld.
    phases=[(1.,0.,O,math.pi/2,'release terminal leaf'),
            (1.5,.045,O,math.pi/2,'clear both leaves'),
            (1.5,.045,O,-math.pi/2,'turn open parallel jaw'),
            (1.5,0.,O,-math.pi/2,'descend around folded pair'),
            (1.,0.,C,-math.pi/2,'grasp actual two-leaf bundle')]
    for duration,dz,fg,yaw,label in phases:
        add('left',duration,end9+[0,0,dz],fg,label,yaw)
        add('right',duration,hold,C,'retain original packet',right_yaw)
    pivot8=t.BASE+np.array([0,0,8*t.LAYER])
    # Same grasped material point sits one layer above leaf8 center.
    for k in range(1,37):
        a=math.pi*k/36
        p=pivot8+np.array([-.10*math.sin(a),.10*math.cos(a),t.LAYER+t.TIP-.012])
        add('left',.25,p,C,'fold penultimate hinge with retained terminal leaf',-math.pi/2+a)
        add('right',.25,hold,C,'support full-size original packet',right_yaw)
    end8=pivot8+np.array([0,-.10,t.LAYER+t.TIP-.012])
    add('left',1.,end8,O,'release newly compact full tool',math.pi/2)
    add('right',1.,hold,C,'hold whole ten-leaf tool',right_yaw)
    add('left',2.,end8+[0,.10,.05],O,'clear supporting hand')
    add('right',2.,hold,C,'retain packet while other hand leaves',right_yaw)
    # At the starting+90deg jaw yaw the rule center is+80mmY from
    # the jaw. Rotating the jaw to0 sends that offset to+80mmX, so
    # the final grasp point must be80mm before the pocket center.
    target=np.array([t.POCKET[0]-.08,t.POCKET[1],hold[2]+t.FLOOR-t.TOP])
    drop=target+[0,0,.055]
    for duration,p,fg,yaw,label in [
        (2.,hold+[0,0,.065],C,right_yaw,'lift whole folded rule'),
        (3.,drop+[0,0,.010],C,0.,'carry above recess'),
        (2.,drop,C,0.,'lower to rim-clear release height'),
        (1.,drop,O,0.,'release above pocket'),
        (2.,drop+[0,-.10,.020],O,0.,'clear all real hand surfaces'),
        (3.,np.array([.18,-.36,t.TOP+.16]),O,0.,'observe actual released support')]:
        add('right',duration,p,fg,label,yaw)
    duration=sum(w.duration for w in right)-sum(w.duration for w in left)
    add('left',duration,[.18,.36,t.TOP+.16],O,'observe released support')
    return {'left':left,'right':right}
