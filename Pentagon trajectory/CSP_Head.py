import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
from segment_ch import *
from constraint import *
from shapely.geometry import Polygon

# finding slope of all the segments
def slope(x1,y1,x2,y2):
    if (y2>y1):
        return (((y2-y1)/(x2-x1)))
    elif (y1>y2):
        return (((y1-y2)/(x1-x2)))

# M,PHI1, PERPANGLE1, SEGMENT LENGTH
m=[]                                    # SLOPE
phi1=[]                                 # ANGLE of each segment from x axis
perpangle1=[]                           # PERPENDICULAR ANGLE TO SEGMENT
segmentlength1=[]                        # SEGMENT LENGTH
for q in range(len(verts)-1):
    x1 = verts[q][0]
    y1 = verts[q][1]
    x2 = verts[q+1][0]
    y2 = verts[q+1][1]
    m.append(slope(x1,y1,x2,y2))
    dx=x2-x1
    dy=y2-y1
    phi1.append(math.degrees(math.atan(m[q])))
    perpangle1.append(phi1[q]-math.degrees(90))
# for q in range(len(verts)-1):
#     #if (q<(len(left_vertexhead)-1)):
#     xx1 = start_x[q]
#     yy1 = start_y[q]
#     xx2 = end_x[q]    
#     yy2 = end_y[q]     
#     #if (q==(len(left_vertexhead)-1)):
#     #xx1 = right_vertexhead[q][0]
#     #yy1 = right_vertexhead[q][1]
#     #xx2 = left_vertexhead[0][0]
#     #yy2 = left_vertexhead[0][1]
#     dxx=xx1-xx2
#     dyy=yy1-yy2
#     segmentlength1.append((math.sqrt(math.pow(dyy,2)+math.pow(dxx,2))))
    
    
# IDENTIFYING QUADRANT OF THE SEGMENT
    if dx>0 and dy>0:
        quad=1
    elif dx<0 and dy>0:
        quad=2
    elif dx<0 and dy<0:
        quad=3
    elif dx>0 and dy<0:
        quad=4

# INCHES - MM CONVERSION 
        
def in2mm(x):
    return (x*25.4)

def mm2in(x):
    return (x/25.4)

#CSP
#VARIABLES

Num_Heads=[]
HH_dist=[]
CH_dist=[]
perpLineSpacing=[]
phi=[]
headsOnSegment=[]
parllx=[]
parlly=[]
perpangle=[]
left=[]
right=[]
front=[]
segment_angle=[]
vertex_type=[]
s=[]
ss=[]
centrd=[]
segmentlengthT=[]
for kf in range(len(segment)):
    segmentlength_mm=in2mm(segmentlength1[kf])
    segmentlengthT.append(in2mm((segmentlength1[kf])))
for k in range(len(segmentlengthT)):
    segmentlengthtemp=segmentlengthT[k]
    problem = Problem(BacktrackingSolver())
    problem.addVariable("g", range(5,20))#between two heads
    problem.addVariable("h", range(1,100)) #number of heads
    problem.addVariable("I", range(2,20+1)) # between head and contour
    #problem.addConstraint(lambda g, h, I:((103+g)*h)<(segmentlength_mm-(2*math.tan(perpangle1[k])*I)),("g", "h","I"))
    problem.addConstraint(lambda g, h:(0.99*segmentlengthtemp)<((103+g)*h)<=segmentlengthtemp,("g", "h"))
    #problem.addConstraint(lambda g, h:g*h<=(20*(h-2)),("g", "h"))
    s.append(problem.getSolutions())
    ss.append(problem.getSolutions())
    
head=[]
dhh=[]
dch=[]

for k in range(len(ss)):
    head=[]
    for i in range(len(ss[k])):
        head.append(ss[k][i]["h"])
        dhh.append(ss[k][i]["g"])
        #dhh.append(18)
        dch.append(ss[k][i]["I"])
        #dch.append(20)
        
    soln=0
    

    Num_Heads.append(head[soln])

    HH_dist.append(mm2in(dhh[soln]))

    CH_dist.append(mm2in(dch[soln]))

    #finding quad
    x1 = verts[k][0]
    y1 = verts[k][1]
    x2 = verts[k+1][0]
    y2 = verts[k+1][1]
    m.append(slope(x1,y1,x2,y2))
    dx=x2-x1
    dy=y2-y1
    segmentlength.append((math.sqrt(math.pow(dy,2)+math.pow(dx,2))))
    if dx>0 and dy>0:
        quad=1
    elif dx<0 and dy>0:
        quad=2
    elif dx<0 and dy<0:
        quad=3
    elif dx>0 and dy<0:
        quad=4

    perpLineSpacing.append((4+HH_dist[k]))
    phi.append(math.degrees(math.atan(m[k])))
    perpangle.append(phi[k]-math.degrees(90))
    headsOnSegment.append(Num_Heads[k])

    # IDENTIFYING POSITION OF HEADS BASED ON QUADRANTS
    if (quad==1): 
        parllx.append(verts[k][0]+(CH_dist[k]*math.cos(math.radians(perpangle[k]))))
        parlly.append(verts[k][1]+(CH_dist[k]*math.sin(math.radians(perpangle[k]))))
    elif (quad==2): 
        parllx.append(verts[k][0]-(CH_dist[k]*math.cos(math.radians(perpangle[k]))))
        parlly.append(verts[k][1]-(CH_dist[k]*math.sin(math.radians(perpangle[k]))))
    elif (quad==3): 
        parllx.append(verts[k][0]-(CH_dist[k]*math.cos(math.radians(perpangle[k]))))
        parlly.append(verts[k][1]-(CH_dist[k]*math.sin(math.radians(perpangle[k]))))
    elif (quad==4): 
        parllx.append(verts[k][0]+(CH_dist[k]*math.cos(math.radians(perpangle[k]))))
        parlly.append(verts[k][1]+(CH_dist[k]*math.sin(math.radians(perpangle[k]))))
        
    ox=(parllx[k])
    oy=(parlly[k])
    path_head=[]
    u=[]
    v=[]
    offset=5+HH_dist[k]

    #PLACING HEAD VERTICES AND CENTROID
    for n in range(headsOnSegment[k]):
        if (quad==1):
            ox=ox+(offset*math.cos(math.radians(phi[k])))
            oy=oy+(offset*math.sin(math.radians(phi[k])))
            (xL,yL)=(ox,oy)
            (xR,yR)=((xL+(4*math.cos(math.radians(phi[k])))),(yL+(4*math.sin(math.radians(phi[k])))))
            (xF,yF)=((xL+(4*math.cos(math.radians(phi[k]-(60))))),(yL+(4*math.sin(math.radians(phi[k]-(60)))))) 
            xC=((1/3)*(xF+xL+xR))
            yC=((1/3)*(yF+yL+yR))
            offset=4+HH_dist[k]
            
        elif (quad==4):
            ox=ox+(offset*math.cos(math.radians(phi[k])))
            oy=oy+(offset*math.sin(math.radians(phi[k])))
            (xL,yL)=(ox,oy)
            (xR,yR)=((xL+(4*math.cos(math.radians(phi[k])))),(yL+(4*math.sin(math.radians(phi[k])))))
            (xF,yF)=((xL+(4*math.cos(math.radians(phi[k]-(60))))),(yL+(4*math.sin(math.radians(phi[k]-(60)))))) 
            xC=((1/3)*(xF+xL+xR))
            yC=((1/3)*(yF+yL+yR))
            offset=4+HH_dist[k]
            
        elif (quad==2):
            ox=ox-(offset*math.cos(math.radians(phi[k])))
            oy=oy-(offset*math.sin(math.radians(phi[k]))) 
            (xL,yL)=(ox,oy)
            (xR,yR)=((xL-(4*math.cos(math.radians(phi[k])))),(yL-(4*math.sin(math.radians(phi[k])))))
            (xF,yF)=((xL-(4*math.cos(math.radians(phi[k]-(60))))),(yL-(4*math.sin(math.radians(phi[k]-(60)))))) 
            xC=((1/3)*(xF+xL+xR))
            yC=((1/3)*(yF+yL+yR))
            offset=4+HH_dist[k]
            
        elif (quad==3):
            ox=ox-(offset*math.cos(math.radians(phi[k])))
            oy=oy-(offset*math.sin(math.radians(phi[k])))
            (xL,yL)=(ox,oy)
            (xR,yR)=((xL-(4*math.cos(math.radians(phi[k])))),(yL-(4*math.sin(math.radians(phi[k])))))
            (xF,yF)=((xL-(4*math.cos(math.radians(phi[k]-(60))))),(yL-(4*math.sin(math.radians(phi[k]-(60)))))) 
            xC=((1/3)*(xF+xL+xR))
            yC=((1/3)*(yF+yL+yR))
            offset=4+HH_dist[k]

        left.append((xL,yL))
        right.append((xR,yR))
        front.append((xF,yF))
        centrd.append((xC,yC))
        intrs=[]

        # CHECKING POLYGON INTERSECTION
for i in range(len(left)-1):
            p1=Polygon([left[i],right[i],front[i]])
            p2=Polygon([left[i+1],right[i+1],front[i+1]])

            if p1.intersects(p2):
                intrs.append(1)
                                
            else:
                intrs.append(0)

   

total_headsleft=left
total_headsright=right
total_headsfront=front
total_headscentrd=centrd
for i in range(len(vert_reference)):
    if i==0:
        total_headsleft.insert(i,left_vertexhead[vert_reference[i]])
        total_headsright.insert(i,right_vertexhead[vert_reference[i]])
        total_headsfront.insert(i,front_vertexhead[vert_reference[i]])
        total_headscentrd.insert(i,centrd_vertexhead[vert_reference[i]])
             
    elif i>0:
        total_headsleft.insert((sum(Num_Heads[0:vert_reference[i]]))+sum(vert_reference[0:(i+1)]),left_vertexhead[vert_reference[i]])
        total_headsright.insert((sum(Num_Heads[0:vert_reference[i]]))+sum(vert_reference[0:(i+1)]),right_vertexhead[vert_reference[i]])
        total_headsfront.insert((sum(Num_Heads[0:vert_reference[i]]))+sum(vert_reference[0:(i+1)]),front_vertexhead[vert_reference[i]])
        total_headscentrd.insert((sum(Num_Heads[0:vert_reference[i]]))+sum(vert_reference[0:(i+1)]),centrd_vertexhead[vert_reference[i]])

    
path_head1=[]
verts_head=[]
for k in range(len(total_headsleft)):
    verts_head.append ([
                front[k], # front
                left[k], # left
                right[k], # right
                front[k] # ignored
                ])


    codes_head1=[Path.MOVETO,
                     Path.LINETO,
                     Path.LINETO,
                     Path.CLOSEPOLY,
                     ]

    path_head.append(Path(verts_head[k], codes_head1))
   


