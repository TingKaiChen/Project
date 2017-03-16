from pyproj import Proj
import numpy as np

myProj = Proj('+proj=utm +zone=51R, +north +ellps=WGS84 +datum=WGS84 +units=m +nodefs')


lt_x, lt_y = myProj(121.0457539,24.7756600)
lb_x, lb_y = myProj(121.0452239,24.7749817)
rb_x, rb_y = myProj(121.0463000,24.7741421)
rt_x, rt_y = myProj(121.0468272,24.7748169)
lt = np.array((lt_x, lt_y))
lb = np.array((lb_x, lb_y))
rb = np.array((rb_x, rb_y))
rt = np.array((rt_x, rt_y))

print 'left: 	', np.linalg.norm(lt-lb)
print 'bottom:	', np.linalg.norm(lb-rb)
print 'right:	', np.linalg.norm(rb-rt)
print 'top:		', np.linalg.norm(rt-lt)