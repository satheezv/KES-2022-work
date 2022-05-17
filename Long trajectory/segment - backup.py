import math
import numpy
import matplotlib.pyplot as plt
from matplotlib.path import Path
from matplotlib.path import Path
import matplotlib.patches as patches

# CONTOUR COORDINATES
verts = [
    #(10, 50),
    #(60, 60.1),
    #(75, 18),
    #(30, 50),
    (19,42),
    (45,65.1),
    (83,60.2),
    (87, 35.3),
    (72, 12.1),
    (32, 23.2),
    (19,42) 
    ]

codes = [Path.MOVETO,
         Path.LINETO,
         Path.LINETO,
         Path.LINETO,
         Path.LINETO,
         Path.LINETO,
         Path.LINETO
         ]

# CREATING PATH OF CONTOUR
path = Path(verts, codes)

path2 = Path(verts[0:2], codes[0:2])  #for temporary plotting change path2 into path in plotcode

segment=[]
for i in range(len(verts)-1):
    segment.append([verts[i],verts[i+1]])
    
# finding slope of all the segments
def slope(x1,y1,x2,y2):
        if (y2>y1):
            return (((y2-y1)/(x2-x1)))
        elif (y1>y2):
            return (((y1-y2)/(x1-x2)))


# vertex head function

def headq14a6090(xV,yV,d,alpha,phi):
    xF=xV+(d*math.cos(math.radians(phi-(alpha/2))))
    yF=yV-(d*math.sin(math.radians(phi-(alpha/2))))
    xL=xF+(4*math.cos(math.radians(phi-(alpha/2)-30)))
    yL=yF+(4*math.sin(math.radians(phi-(alpha/2)-30)))
    xR=xF+(4*math.cos(math.radians(phi-(alpha/2)+30)))
    yR=yF+(4*math.sin(math.radians(phi-(alpha/2)+30)))
    head_coords.append([xF,yF,xL,yL,xR,yR])

def headq23a6090(xV,yV,d,alpha,phi):
    xF=xV-(d*math.cos(math.radians(phi-(alpha/2))))
    yF=yV-(d*math.sin(math.radians(phi-(alpha/2))))
    xL=xF-(4*math.cos(math.radians(phi+90)))
    yL=yF-(4*math.sin(math.radians(phi+90)))
    xR=xF-(4*math.cos(math.radians(phi-180)))
    yR=yF-(4*math.sin(math.radians(phi-180)))
    head_coords.append([xF,yF,xL,yL,xR,yR])

def headq14a90120(xV,yV,d,alpha,phi):
    xL=xV-(4.33*math.cos(math.radians(phi-(alpha)+15)))
    yL=yV-(4.33*math.sin(math.radians(phi-(alpha)+15)))
    xF=xL-(4*math.cos(math.radians(phi-(alpha/3)+90)))
    yF=yL-(4*math.sin(math.radians(phi-(alpha/3)+90)))
    xR=xL-(4*math.cos(math.radians(phi-(alpha/3)+30)))
    yR=yL-(4*math.sin(math.radians(phi-(alpha/3)+30)))
    head_coords.append([xF,yF,xL,yL,xR,yR])

def headq23a90120(xV,yV,d,alpha,phi):
    xL=xV-(d*math.cos(math.radians(phi-(alpha/4))))
    yL=yV-(d*math.sin(math.radians(phi-(alpha/4))))
    xF=xL-(4*math.cos(math.radians(phi-(alpha/4)+90)))
    yF=yL-(4*math.sin(math.radians(phi-(alpha/4)+90)))
    xR=xL-(4*math.cos(math.radians(phi-(alpha/4)+30)))
    yR=yL-(4*math.sin(math.radians(phi-(alpha/4)+30)))
    head_coords.append([xF,yF,xL,yL,xR,yR])

def headq14a120160(xV,yV,d,alpha,phi):
    xF=xV+(d*math.cos(math.radians(phi-(alpha/2))))
    yF=yV+(d*math.sin(math.radians(phi-(alpha/2))))
    xL=xF+(4*math.cos(math.radians(phi-(alpha/2)-30)))
    yL=yF+(4*math.sin(math.radians(phi-(alpha/2)-30)))
    xR=xF+(4*math.cos(math.radians(phi-(alpha/2)+30)))
    yR=yF+(4*math.sin(math.radians(phi-(alpha/2)+30)))
    head_coords.append([xF,yF,xL,yL,xR,yR])

def headq23a120160(xV,yV,d,alpha,phi):
    xF=xV-(d*math.cos(math.radians(phi-(alpha/2))))
    yF=yV-(d*math.sin(math.radians(phi-(alpha/2))))
    xL=xF-(4*math.cos(math.radians(phi+90)))
    yL=yF-(4*math.sin(math.radians(phi+90)))
    xR=xF-(4*math.cos(math.radians(phi-180)))
    yR=yF-(4*math.sin(math.radians(phi-180)))
    head_coords.append([xF,yF,xL,yL,xR,yR])

# M,PHI1, PERPANGLE1, SEGMENT LENGTH
m=[]                                    # SLOPE
phi1=[]                                 # SEGMENT ANGLE
perpangle1=[]                           # PERPENDICULAR ANGLE TO SEGMENT
segmentlength=[]                        # SEGMENT LENGTH
quadrant=[]
dxx=[]
dyy=[]
for i in range(len(verts)-1):
    x1 = verts[i][0]
    y1 = verts[i][1]
    x2 = verts[i+1][0]
    y2 = verts[i+1][1]
    m.append(slope(x1,y1,x2,y2))
    dx=x2-x1
    dxx.append(dx)
    dy=y2-y1
    dyy.append(dy)
    segmentlength.append((math.sqrt(math.pow(dy,2)+math.pow(dx,2))))
    phi1.append(math.degrees(math.atan(m[i])))
    perpangle1.append(phi1[i]-90)
    
# IDENTIFYING QUADRANT OF THE SEGMENT
    if dx>0 and dy>0:
        quad=1
    elif dx<0 and dy>0:
        quad=2
    elif dx<0 and dy<0:
        quad=3
    elif dx>0 and dy<0:
        quad=4
    quadrant.append(quad)
quadrant_previous=quadrant.copy()    
quadrant_previous.insert(0,quadrant_previous.pop())
# FINDING TYPER OF THE VERTEX ANGLE OF THE CONTOUR
unit_vector=[]
segment_angle=[]
vertex_type=[]
head_coords=[]
path_head_vertex=[]
xC_vh=[]
yC_vh=[]
left_vertexhead=[]
right_vertexhead=[]
front_vertexhead=[]
centrd_vertexhead=[]
for i in range(len(verts)-1):
    if (i==0):
        (x1,y1) = (verts[len(verts)-2][0],verts[len(verts)-2][1])
        (x2,y2) = (verts[i+1][0],verts[i+1][1])
        (x3,y3) = (verts[i][0],verts[i][1])
        (dx1,dy1)=(x3-x1,y3-y1)
        (dx2,dy2)=(x3-x2,y3-y2)
        segmentlength1 = ((math.sqrt(math.pow(dy1,2)+math.pow(dx1,2))))
        segmentlength2 = ((math.sqrt(math.pow(dy2,2)+math.pow(dx2,2))))
        (a1,b1)=(dx1/segmentlength1,dy1/segmentlength1)
        (a2,b2)=(dx2/segmentlength2,dy2/segmentlength2)
        segment_angle.append(math.degrees(math.acos(a1*a2+b1*b2)))
    elif (i>0):
        (x1,y1) = (verts[i-1][0],verts[i-1][1])
        (x2,y2) = verts[i+1][0],verts[i+1][1]
        (x3,y3) = verts[i][0],verts[i][1]
        (dx1,dy1)=(x3-x1,y3-y1)
        (dx2,dy2)=(x3-x2,y3-y2)
        segmentlength1 = ((math.sqrt(math.pow(dy1,2)+math.pow(dx1,2))))
        segmentlength2 = ((math.sqrt(math.pow(dy2,2)+math.pow(dx2,2))))
        (a1,b1)=(dx1/segmentlength1,dy1/segmentlength1)
        (a2,b2)=(dx2/segmentlength2,dy2/segmentlength2)
        segment_angle.append(math.degrees(math.acos(a1*a2+b1*b2)))
        
for j in range(len(segment_angle)):
    if segment_angle[j]<60:
        vertex_type.append(0)
        headq14a6090(verts[j][0],verts[j][1],0.787402,segment_angle[j],phi1[j])
    elif 60<segment_angle[j]<90:
        vertex_type.append(1)
        headq14a6090(verts[j][0],verts[j][1],0.787402,segment_angle[j],phi1[j])
    elif 90<segment_angle[j]<120:
        vertex_type.append(2)
        headq14a90120(verts[j][0],verts[j][1],0.787402,segment_angle[j],phi1[j])
    elif segment_angle[j]>120:
        vertex_type.append(3)
        headq23a90120(verts[j][0],verts[j][1],0.787402,segment_angle[j],phi1[j])

    xC_vh=((1/3)*(head_coords[j][0]+head_coords[j][2]+head_coords[j][4]))
    yC_vh=((1/3)*(head_coords[j][1]+head_coords[j][3]+head_coords[j][5]))
    
    left_vertexhead.append((head_coords[j][2],head_coords[j][3]))
    right_vertexhead.append((head_coords[j][4],head_coords[j][5]))
    front_vertexhead.append((head_coords[j][0],head_coords[j][1])) 
    centrd_vertexhead.append((xC_vh,yC_vh))

  
