import cv2, time
import numpy as np

class Camera():
    def __init__(self, fps=30, frame_shape=[320,240]):
        self.vc = cv2.VideoCapture(0)

        self.fps = fps
        self.frame_shape = frame_shape
        assert self.frame_shape in [[320,240],[640,480]]

        self.vc.set(cv2.CAP_PROP_FPS, int(self.fps))
        self.vc.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.frame_shape[1]))
        self.vc.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.frame_shape[0]))
    def run(self, dur, name):
        vw = cv2.VideoWriter(name+'.avi', cv2.VideoWriter_fourcc(*'MJPG'), self.fps, self.frame_shape)
        tsfile = open(name+'.timestamps', 'a')

        t0 = time.clock()
        while time.clock()-t0 < dur:
            val,fr = self.vc.read()
            ts = time.clock()
            if not val:
                continue
            fr = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY).astype(np.uint8)
            vw.write(fr)
            tsfile.write('{:0.10f}\n'.format(ts))

        vw.release()
        tsfile.close()
