import math
import numpy
import matplotlib.pyplot as plt
from matplotlib.path import Path
from matplotlib.path import Path
import matplotlib.patches as patches

# CONTOUR COORDINATES
    #(10, 50),
    #(60, 60.1),
    #(75, 18),
    #(30, 50),
""" test 2
    (14,42),
    (40,65.1),
    (81,63.4),
    (82, 35.3),
    (67, 12.1),
    (30, 19.2),
    (14,42)
"""
    # Hexagon
# verts = [(19.685, 19.854),
#     (11.811, 23.864),
#     (3.937, 27.874),
#     ]

verts_m = [(0.0254*19.685, 0.0254*19.854),
    (0.0254*11.811, 0.0254*20.864),
    (0.0254*3.937, 0.0254*21.874),
    (0.0254*3.947, 0.0254*27.864),
    (0.0254*11.811, 0.0254*26.854),
    (0.0254*19.685, 0.0254*25.844)
    #0.0254*81.83, 0.0254*44),
    #(0.0254*72.10, 0.0254*23.16),
    #(0.0254*49.2, 0.0254*21.16),
    #(0.0254*36, 0.0254*40)
    ]

codes = [Path.MOVETO,
         Path.LINETO,
         Path.LINETO,
         Path.LINETO,
         Path.LINETO,
         Path.LINETO,
         #Path.LINETO
         ]

# CREATING PATH OF CONTOUR
path = Path(verts_m, codes)

# path2 = Path(verts[0:2], codes[0:2])  #for temporary plotting change path2 into path in plotcode

