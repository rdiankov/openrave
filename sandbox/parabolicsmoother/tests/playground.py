import mpmath
from mpmath import mp
import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append('../')

import ramp

ramp1 = ramp.Ramp(0, 2, 1, 0)
ramp2 = ramp.Ramp(ramp1.v1, -ramp1.a, 0.8, ramp1.d)
curve = ramp.ParabolicCurve([ramp1, ramp2])

assert(ramp.Abs(ramp.sub(curve.duration - mp.mpf(1.8))) < ramp.epsilon)
