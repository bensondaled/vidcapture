import numpy as np
import os, time
    
############ Constants ##############

# camera sensor parameters
CLEYE_AUTO_GAIN = 0 #[false, true]
CLEYE_GAIN = 1 #[0, 79]
CLEYE_AUTO_EXPOSURE = 2 #[false, true]
CLEYE_EXPOSURE = 3#[0, 511]
CLEYE_AUTO_WHITEBALANCE = 4 #[false, true]
CLEYE_WHITEBALANCE_RED = 5#[0, 255]
CLEYE_WHITEBALANCE_GREEN = 6#[0, 255]
CLEYE_WHITEBALANCE_BLUE = 7#[0, 255]
# camera linear transform parameters
CLEYE_HFLIP = 8#[false, true]
CLEYE_VFLIP = 9#[false, true]
CLEYE_HKEYSTONE = 0#[-500, 500]
CLEYE_VKEYSTONE = 0#[-500, 500]
CLEYE_XOFFSET = 0#[-500, 500]
CLEYE_YOFFSET = 0#[-500, 500]
CLEYE_ROTATION = 0#[-500, 500]
CLEYE_ZOOM = 0#[-500, 500]
# camera non-linear transform parameters
CLEYE_LENSCORRECTION1 = 16#[-500, 500]
CLEYE_LENSCORRECTION2 = 17#[-500, 500]
CLEYE_LENSCORRECTION3 = 18#[-500, 500]
CLEYE_LENSBRIGHTNESS = 19#[-500, 500]
#CLEyeCameraColorMode
CLEYE_GREYSCALE = 0
CLEYE_COLOR = 1
#CLEyeCameraResolution
CLEYE_QVGA = 0
CLEYE_VGA = 1

############# Special API Structures ################

class GUID(Structure):
    _fields_ = [("Data1", c_uint32),
             ("Data2", c_uint16),
             ("Data3", c_uint16),
             ("Data4", c_uint8 * 8)]
    def __str__(self):
        return "%X-%X-%X-%s" % (self.Data1, self.Data2, self.Data3, ''.join('%02X'%x for x in self.Data4))

def CLEyeCameraGetFrameDimensions(dll, cam):
    width = c_int()
    height = c_int()
    dll.CLEyeCameraGetFrameDimensions(cam, byref(width), byref(height))
    return width.value, height.value

def mp2np(a):
    return np.frombuffer(a.get_obj(), dtype=np.uint8)

class PSEye():

    RES_SMALL = CLEYE_QVGA
    RES_LARGE = CLEYE_VGA   
    available_framerates = dict(RES_LARGE=[15,30,40,50,60,75], RES_SMALL=[15,30,60,75,100,125])
    DIMS_SMALL = (320,240)
    DIMS_LARGE = (640,480)
    COLOR = CLEYE_COLOR
    GREYSCALE = CLEYE_GREYSCALE

    def __init__(self, idx, resolution, frame_rate, color_mode, vflip=False, hflip=False, gain=60, exposure=24, wbal_red=50, wbal_blue=50, wbal_green=50, auto_gain=0, auto_exposure=0, auto_wbal=0, zoom=0, save_name=None, ts_buffer=1000, sync_flag=None, frame_buffer=None, kill_flag=None, saving_flag=None):

        # Parameters
        self.idx = idx
        self.resolution_mode = resolution
        self.resolution = [self.DIMS_SMALL,self.DIMS_LARGE][[self.RES_SMALL,self.RES_LARGE].index(self.resolution_mode)]
        self.frame_rate = frame_rate
        self.color_mode = color_mode
        self.vflip = vflip
        self.hflip = hflip
        self.gain = gain
        self.exposure = exposure
        self.auto_gain = auto_gain
        self.auto_exposure = auto_exposure
        self.auto_wbal = auto_wbal
        self.zoom = zoom
        self.wbal_red,self.wbal_blue,self.wbal_green = wbal_red,wbal_blue,wbal_green
        self.read_dims = self.resolution[::-1]
        if self.color_mode == self.COLOR:
            self.read_dims.append(4)

        self.lib = "CLEyeMulticam.dll"
        
        self.settings = [   (CLEYE_AUTO_GAIN, self.auto_gain),
                            (CLEYE_AUTO_EXPOSURE, self.auto_exposure),
                            (CLEYE_AUTO_WHITEBALANCE,self.auto_wbal),
                            (CLEYE_GAIN, self.gain),
                            (CLEYE_EXPOSURE, self.exposure),
                            (CLEYE_WHITEBALANCE_RED,self.wbal_red),
                            (CLEYE_WHITEBALANCE_BLUE,self.wbal_blue),
                            (CLEYE_WHITEBALANCE_GREEN,self.wbal_green),
                            (CLEYE_VFLIP, self.vflip),
                            (CLEYE_HFLIP, self.hflip),
                            (CLEYE_ZOOM, self.zoom),
                 ]

        self.save_name = save_name
        self.ts_buffer = ts_buffer
        
        if self.color_mode == CLEYE_GREYSCALE:
            self.bytes_per_pixel = 1
        elif self.color_mode == CLEYE_COLOR:
            self.bytes_per_pixel = 4

    def get_frame(self, timeout=20000):
        if not self._cam:
            return [None,None,None]
            
        ts = time.clock()
        got = self.dll.CLEyeCameraGetFrame(self._cam, self._buf, timeout)
        return (got,ts)
    
    def configure(self, settings):
        if not self._cam:
            return
        for param, value in settings:
            self.dll.CLEyeSetCameraParameter(self._cam, param, value)

    def run(self):
        self._init_cam()
       
        # Saving
        #if self.save_name != None:
            #self.vidts_name = self.save_name+'.tstmp'
            #self.vidts_file_temp = open(self.vidts_name, 'a')

            # MVSC, DIB\, MJPG
            #fourcc = 1
            #fourcc = cv2.VideoWriter_fourcc(*'DIB ')
            #self.vws = [cv2.VideoWriter(self.vid_name(0),fourcc,self.frame_rate,frameSize=self.resolution,isColor=False)]   
            #if not self.vws[0].isOpened():
            #    warnings.warn('Video writer failed to open')

            
        n_frames_read, self.vw_idx = 0,0
        ts_str = ''

        # Main loop
        while True:
            if self.kill_flag.value:
                break
            # Read in frame
            #DEBUG = now()
            changed_file, got = False, False
            got,ts = self.get_frame()
            #print now()-DEBUG

            if got:
                # Next videowriter
                #n_frames_read += 1
                #if n_frames_read == 50000:
                #    changed_file = True
                #    n_frames_read = 0
                #    self.vws[-1].release()
                #    self.vws.append(cv2.VideoWriter(self.vid_name(self.vw_idx+1),0,self.frame_rate,frameSize=self.resolution,isColor=False))
                #    self.vw_idx += 1
                fr = np.frombuffer(self._buf, dtype=np.uint8)
                if self.saving_flag.value:
                    self.frame_buffer.put([[ts,ts2],fr])
                    
                    #if changed_file:
                    #    ts_str += '\n'
                    #ts_str += '{:0.20f}/{:0.20f},'.format(ts,ts2)
                    #if len(ts_str) > self.ts_buffer:
                    #    self.vidts_file_temp.write(ts_str)
                    #    ts_str = ''
        #if self.save_name:
        #    self.vws[self.vw_idx].release()
        #    self.vidts_file_temp.write(ts_str)
        #    self.vidts_file_temp.close()
        
        try:
            self.dll.CLEyeCameraStop(self._cam)
            self.dll.CLEyeDestroyCamera(self._cam)
        except:
            pass
        self.thread_complete.value = 1

    def _init_cam(self):
        # Load dynamic library
        self.dll = ctypes.cdll.LoadLibrary(self.lib)
        self.dll.CLEyeGetCameraUUID.restype = GUID
        self.dll.CLEyeCameraGetFrame.argtypes = [c_void_p, c_char_p, c_int]
        self.dll.CLEyeCreateCamera.argtypes = [GUID, c_int, c_int, c_float]
    
        #print self.dll.CLEyeGetCameraCount()
    
        self._cam = self.dll.CLEyeCreateCamera(self.dll.CLEyeGetCameraUUID(self.idx), self.color_mode, self.resolution_mode, self.frame_rate)
        if not self._cam:
            warnings.warn('Camera failed to initialize.')
            return
            
        self.x, self.y = CLEyeCameraGetFrameDimensions(self.dll, self._cam)
        self._buf = ctypes.create_string_buffer(self.x * self.y * self.bytes_per_pixel) 
        
        self.configure(self.settings)
        self.dll.CLEyeCameraStart(self._cam)
        time.sleep(0.01)
       
    def get(self):
        if now()-self.last_query < 1./self.query_rate:
            return None
        self.last_query = now()
        self.saver.query_flag.value = True
        fr = mp2np(self.saver.query_queue)
        return fr.reshape([self.y,self.x])

##################################################################################################

default_cam_params = dict(  idx=0, 
                            resolution=_PSEye.RES_SMALL, 
                            query_rate = 10,
                            frame_rate=60, 
                            color_mode=_PSEye.GREYSCALE,
                            vflip = False,
                            hflip = False,
        )

if __name__ == '__main__':

    cam_params = default_cam_params.copy()
    cam_params.update(save_name=r'C:\\')
    cam = PSEye(**cam_params)
    cam.begin_saving()

    while True:
        fr = cam.get()
        if fr is not None:
            cv2.imshow('Camera View', fr)
        q = cv2.waitKey(1)
        if q == ord('q'):
            break
    cam.end()
    cv2.destroyAllWindows()
    
