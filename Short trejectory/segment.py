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
verts = [(36, 40),
    (45.73, 60.84),
    (68.63, 62.84),
    (81.83, 44),
    (72.10, 23.16),
    (49.2, 21.16),
    (36, 40)
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
    yF=yV+(d*math.sin(math.radians(phi-(alpha/2))))
    xL=xF+(4*math.cos(math.radians(phi-(alpha/2)-30)))
    yL=yF+(4*math.sin(math.radians(phi-(alpha/2)-30)))
    xR=xF+(4*math.cos(math.radians(phi-(alpha/2)+30)))
    yR=yF+(4*math.sin(math.radians(phi-(alpha/2)+30)))
    head_coords.append([xF,yF,xL,yL,xR,yR])

def headq23a6090(xV,yV,d,alpha,phi):
    xF=xV-(d*math.cos(math.radians(phi-(alpha/2))))
    yF=yV-(d*math.sin(math.radians(phi-(alpha/2))))
    xL=xF-(5*math.cos(math.radians(phi+90)))
    yL=yF-(5*math.sin(math.radians(phi+90)))
    xR=xF-(5*math.cos(math.radians(phi-180)))
    yR=yF-(5*math.sin(math.radians(phi-180)))
    head_coords.append([xF,yF,xL,yL,xR,yR])

def headq23a90120(xV,yV,d,alpha,phi):
    xL=xV-(4.33*math.cos(math.radians(phi-(alpha)+15)))
    yL=yV-(4.33*math.sin(math.radians(phi-(alpha)+15)))
    xF=xL-(4*math.cos(math.radians(phi-(alpha/3)+90)))
    yF=yL-(4*math.sin(math.radians(phi-(alpha/3)+90)))
    xR=xL-(4*math.cos(math.radians(phi-(alpha/3)+30)))
    yR=yL-(4*math.sin(math.radians(phi-(alpha/3)+30)))
    head_coords.append([xF,yF,xL,yL,xR,yR])

def headq14a90120(xV,yV,d,alpha,phi):
    xL=xV+(4.33*math.cos(math.radians(phi-(alpha)+15)))
    yL=yV+(4.33*math.sin(math.radians(phi-(alpha)+15)))
    xF=xL+(4*math.cos(math.radians(phi-(alpha/3)+90)))
    yF=yL+(4*math.sin(math.radians(phi-(alpha/3)+90)))
    xR=xL+(4*math.cos(math.radians(phi-(alpha/3)+30)))
    yR=yL+(4*math.sin(math.radians(phi-(alpha/3)+30)))
    head_coords.append([xF,yF,xL,yL,xR,yR])

def headq23a120160(xV,yV,d,alpha,phi):
    xF=xV-(d*math.cos(math.radians(phi-(alpha/4))))
    yF=yV-(d*math.sin(math.radians(phi-(alpha/4))))
    xL=xF-(4*math.cos(math.radians(phi-(alpha/4)-30)))
    yL=yF-(4*math.sin(math.radians(phi-(alpha/4)-30)))
    xR=xF-(4*math.cos(math.radians(phi-(alpha/4)+30)))
    yR=yF-(4*math.sin(math.radians(phi-(alpha/4)+30)))

    xF2=xV-(d*math.cos(math.radians(phi-(alpha*3/4))))
    yF2=yV-(d*math.sin(math.radians(phi-(alpha*3/4))))
    xL2=xF2-(4*math.cos(math.radians(phi-(alpha*3/4)-30)))
    yL2=yF2-(4*math.sin(math.radians(phi-(alpha*3/4)-30)))
    xR2=xF2-(4*math.cos(math.radians(phi-(alpha*3/4)+30)))
    yR2=yF2-(4*math.sin(math.radians(phi-(alpha*3/4)+30)))
    head_coords.append([xF,yF,xL,yL,xR,yR,xF2,yF2,xL2,yL2,xR2,yR2])
    
def headq14a120160(xV,yV,d,alpha,phi):
    xF=xV+(d*math.cos(math.radians(phi-(alpha/4))))
    yF=yV+(d*math.sin(math.radians(phi-(alpha/4))))
    xL=xF+(4*math.cos(math.radians(phi-(alpha/4)-30)))
    yL=yF+(4*math.sin(math.radians(phi-(alpha/4)-30)))
    xR=xF+(4*math.cos(math.radians(phi-(alpha/4)+30)))
    yR=yF+(4*math.sin(math.radians(phi-(alpha/4)+30)))

    xF2=xV+(d*math.cos(math.radians(phi-(alpha*3/4))))
    yF2=yV+(d*math.sin(math.radians(phi-(alpha*3/4))))
    xL2=xF2+(4*math.cos(math.radians(phi-(alpha*3/4)-30)))
    yL2=yF2+(4*math.sin(math.radians(phi-(alpha*3/4)-30)))
    xR2=xF2+(4*math.cos(math.radians(phi-(alpha*3/4)+30)))
    yR2=yF2+(4*math.sin(math.radians(phi-(alpha*3/4)+30)))
    head_coords.append([xF,yF,xL,yL,xR,yR,xF2,yF2,xL2,yL2,xR2,yR2])
    
def rotate(l,n):
    return l[n:]+l[:n]
    # M,PHI1, PERPANGLE1, SEGMENT LENGTH
m=[]                                    # SLOPE
phi1=[]                                 # SEGMENT ANGLE
perpangle1=[]                           # PERPENDICULAR ANGLE TO SEGMENT
segmentlength=[]                        # SEGMENT LENGTH
quadrant=[]
for i in range(len(verts)-1):
    x1 = verts[i][0]
    y1 = verts[i][1]
    x2 = verts[i+1][0]
    y2 = verts[i+1][1]
    m.append(slope(x1,y1,x2,y2))
    dx=x2-x1
    dy=y2-y1
    segmentlength.append((math.sqrt(math.pow(dy,2)+math.pow(dx,2))))
    phi1.append(math.degrees(math.atan(m[i])))
    perpangle1.append(phi1[i]-math.degrees(90))
    
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
vertshead_cornor=[]
pathhead_cornor=[]
start_x=[]
start_y=[]
end_x=[]
end_y=[]
end_temp_x=[]
end_temp_y=[]

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

vert_reference=[]        
for j in range(len(segment_angle)):
    if segment_angle[j]<60:
        vertex_type.append(0)
        headq14a6090(verts[j][0],verts[j][1],0.787402,segment_angle[j],phi1[j])

        xC_vh=((1/3)*(head_coords[j][0]+head_coords[j][2]+head_coords[j][4]))
        yC_vh=((1/3)*(head_coords[j][1]+head_coords[j][3]+head_coords[j][5]))
    
        left_vertexhead.append((head_coords[j][2],head_coords[j][3]))
        right_vertexhead.append((head_coords[j][4],head_coords[j][5]))
        front_vertexhead.append((head_coords[j][0],head_coords[j][1])) 
        centrd_vertexhead.append((xC_vh,yC_vh))
        """
        vertshead_cornor.append ([
                front_vertexhead[j], # front
                left_vertexhead[j], # left
                right_vertexhead[j], # right
                front_vertexhead[j] # ignored
                ])


        codeshead_cornor=[Path.MOVETO,
                     Path.LINETO,
                     Path.LINETO,
                     Path.CLOSEPOLY,
                     ]

        pathhead_cornor.append(Path(vertshead_cornor[j], codeshead_cornor))
        """


        
    elif 60<segment_angle[j]<90:
        vertex_type.append(1)
        headq14a6090(verts[j][0],verts[j][1],0.787402,segment_angle[j],phi1[j])

        xC_vh=((1/3)*(head_coords[j][0]+head_coords[j][2]+head_coords[j][4]))
        yC_vh=((1/3)*(head_coords[j][1]+head_coords[j][3]+head_coords[j][5]))
    
        left_vertexhead.append((head_coords[j][2],head_coords[j][3]))
        right_vertexhead.append((head_coords[j][4],head_coords[j][5]))
        front_vertexhead.append((head_coords[j][0],head_coords[j][1])) 
        centrd_vertexhead.append((xC_vh,yC_vh))
        vert_reference.append(j)
        startx=head_coords[j][4]
        starty=head_coords[j][5]
        endx=head_coords[j][2]
        endy=head_coords[j][3]
        """
        vertshead_cornor.append ([
                front_vertexhead[j], # front
                left_vertexhead[j], # left
                right_vertexhead[j], # right
                front_vertexhead[j] # ignored
                ])


        codeshead_cornor=[Path.MOVETO,
                     Path.LINETO,
                     Path.LINETO,
                     Path.CLOSEPOLY,
                     ]

        pathhead_cornor.append(Path(vertshead_cornor[j], codeshead_cornor))
        """        
    elif 90<segment_angle[j]<119:
        vertex_type.append(2)
        if quadrant[j]==1 or quadrant[j]==4:
            headq14a90120(verts[j][0],verts[j][1],0.787402,segment_angle[j],phi1[j])
        elif quadrant[j]==2 or quadrant[j]==3:
            headq23a90120(verts[j][0],verts[j][1],0.787402,segment_angle[j],phi1[j])
        xC_vh=((1/3)*(head_coords[j][0]+head_coords[j][2]+head_coords[j][4]))
        yC_vh=((1/3)*(head_coords[j][1]+head_coords[j][3]+head_coords[j][5]))
    
        left_vertexhead.append((head_coords[j][2],head_coords[j][3]))
        right_vertexhead.append((head_coords[j][4],head_coords[j][5]))
        front_vertexhead.append((head_coords[j][0],head_coords[j][1])) 
        centrd_vertexhead.append((xC_vh,yC_vh))
        vert_reference.append(j)
        startx=head_coords[j][4]
        starty=head_coords[j][5]
        endx=head_coords[j][2]
        endy=head_coords[j][3]
        
        """
        vertshead_cornor.append ([
                front_vertexhead[j], # front
                left_vertexhead[j], # left
                right_vertexhead[j], # right
                front_vertexhead[j] # ignored
                ])


        codeshead_cornor=[Path.MOVETO,
                     Path.LINETO,
                     Path.LINETO,
                     Path.CLOSEPOLY,
                     ]

        pathhead_cornor.append(Path(vertshead_cornor[j], codeshead_cornor))
        """     
    elif 119<segment_angle[j]<160:
        vertex_type.append(3)
        if quadrant[j]==1 or quadrant[j]==4:
            headq14a120160(verts[j][0],verts[j][1],0.787402,segment_angle[j],phi1[j])
        elif quadrant[j]==2 or quadrant[j]==3:
            headq23a120160(verts[j][0],verts[j][1],0.787402,segment_angle[j],phi1[j])
        
        xC_vh2=((1/3)*(head_coords[j][6]+head_coords[j][8]+head_coords[j][10]))
        yC_vh2=((1/3)*(head_coords[j][7]+head_coords[j][9]+head_coords[j][11]))

        
        front_vertexhead2=((head_coords[j][6],head_coords[j][7]))
        right_vertexhead2=((head_coords[j][10],head_coords[j][11]))
        left_vertexhead2=((head_coords[j][8],head_coords[j][9]))
        
        front_vertexhead.append((head_coords[j][6],head_coords[j][7]))
        right_vertexhead.append((head_coords[j][10],head_coords[j][11]))
        left_vertexhead.append((head_coords[j][8],head_coords[j][9]))
        centrd_vertexhead.append((xC_vh2,yC_vh2))
        vert_reference.append(j)


        """

        vertshead_cornor2=([
                front_vertexhead2, # front
                left_vertexhead2, # left
                right_vertexhead2, # right
                front_vertexhead2 # ignored
                ])

        codeshead_cornor=[Path.MOVETO,
                     Path.LINETO,
                     Path.LINETO,
                     Path.CLOSEPOLY,
                     ]

        
        pathhead_cornor.append(Path(vertshead_cornor2, codeshead_cornor))
   
        """
        
        xC_vh=((1/3)*(head_coords[j][0]+head_coords[j][2]+head_coords[j][4]))
        yC_vh=((1/3)*(head_coords[j][1]+head_coords[j][3]+head_coords[j][5]))

        
    
        left_vertexhead.append((head_coords[j][2],head_coords[j][3]))
        
        right_vertexhead.append((head_coords[j][4],head_coords[j][5]))
        
        front_vertexhead.append((head_coords[j][0],head_coords[j][1]))
        
        centrd_vertexhead.append((xC_vh,yC_vh))

        vert_reference.append(j)
        """
        vertshead_cornor.append ([
                front_vertexhead[j], # front
                left_vertexhead[j], # left
                right_vertexhead[j], # right
                front_vertexhead[j] # ignored
                ])


        codeshead_cornor=[Path.MOVETO,
                     Path.LINETO,
                     Path.LINETO,
                     Path.CLOSEPOLY,
                     ]

        pathhead_cornor.append(Path(vertshead_cornor[j], codeshead_cornor))
        """
        
        
        startx=head_coords[j][4]  
        starty=head_coords[j][5]
        endx=head_coords[j][8]
        endy=head_coords[j][9]
        
        
        
    start_x.append(startx)
    start_y.append(starty)
    end_temp_x.append(endx)
    end_temp_y.append(endy)
    end_x=rotate(end_temp_x,1)
    end_y=rotate(end_temp_y,1)
    
