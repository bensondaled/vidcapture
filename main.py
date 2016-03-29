from .cameras import Camera

frame_rate = 30  #15,30,40,50,60,75,100,125 -- some are specific to one resolution
frame_shape = [320,240]

dur = raw_input('Enter video duration: ')
dur = float(dur)

name = ''
while len(name) == 0:
    name = raw_input('Enter video name: ')

cam = Camera(fps=frame_rate, frame_shape=frame_shape)
cam.run(dur=dur, name=name)

print('Acquisition complete.')
