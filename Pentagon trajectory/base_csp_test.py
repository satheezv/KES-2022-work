import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
from coordinates import *
from segment_ch import *
#from CSP_Head import *
from head_support_location import *
from constraint import *
from shapely.geometry import Polygon


"""
To change:  allocate proper centroid for empty centroid head pairs
"""
# VARIABLE DECLARARION
    
centrds_Head1=[]
centrds_Head2=[]
arm_length=0.4572 #m
base_radius=11.5 #11.5 inches to cm
    #outer base variables
om_head1=[]
ocoord_1=[]
ocoord_2=[]
ocoord_3=[]
ocoord_4=[]
rect=[]
ophi_head1=[]
oanglecheck=[]
otypequad=[]

    #inner base variables
im_head1=[]
icoord_1=[]
icoord_2=[]
icoord_3=[]
icoord_4=[]
irect=[]
iphi_head1=[]
ianglecheck=[]
itypequad=[]


#SLOPE BETWEEN HEADS
def slope_head(x1,y1,x2,y2):
    xx=(x2-x1)+0.1
    yy=(y2-y1)+0.1
    slop=yy/xx
    return slop


   
# HEADS CLASSIFICATION for multi heads
# for i in range(len(centroidhead_mdp)):
#     if i % 2==0:
#         centrds_Head1.append(centroidhead_mdp[i])
#     elif i % 2 !=0:
#         centrds_Head2.append(centroidhead_mdp[i])

centrds_Head1=head_c

# QUADRANT DEFINITION FOR BASE

def quad(c_1,c_2,arm_length,perpangle_head1):
    c_3x=(c_2[0]+(arm_length*math.cos(math.radians(perpangle_head1-7))))
    c_3y=(c_2[1]+(arm_length*math.sin(math.radians(perpangle_head1-7))))
    c_4x=(c_1[0]+(arm_length*math.cos(math.radians(perpangle_head1+7))))
    c_4y=(c_1[1]+(arm_length*math.sin(math.radians(perpangle_head1+7))))
    rect.append([c_3x,c_3y,c_4x,c_4y])

## CENTROID: COORDINATES INTO POSITION
def co2po(co_x,co_y):
    po_c=0
    if 12 < co_x < 13:
        po_c=0
    elif 19 < co_x < 12:
        po_c=1
    elif 26 < co_x < 27:
        po_c=2
    elif 32 < co_x < 33:
        po_c=3
    elif 39 < co_x < 40:
        po_c=4
    elif 46 < co_x < 47:
        po_c=5
    elif 52 < co_x < 54:
        po_c=6
    elif 59 < co_x < 60:
        po_c=7
    elif 66 < co_x < 67:
        po_c=8
    elif 72 < co_x < 74:
        po_c=9
    elif 79 < co_x < 80:
        po_c=10
    elif 86 < co_x < 87:
        po_c=11
    elif 93 < co_x < 94:
        po_c=12
    if 11 < co_y < 16:
        po_r=5
    elif 23 < co_y < 28:
        po_r=4
    elif 34 < co_y < 39:
        po_r=3
    elif 46 < co_y < 51:
        po_r=2
    elif 57 < co_y < 63:
        po_r=1
    elif 69 < co_y < 74:
        po_r=0
    return (po_r,po_c)
        

# RECTANGLE CREATION

    #OUTER BASE HEAD1
rect=[]
for i in range(len(centrds_Head1)):
    e1=((centrds_Head1[i][0])-(arm_length)),((centrds_Head1[i][1])-(arm_length))
    e2=((centrds_Head1[i][0])+(arm_length)),((centrds_Head1[i][1])-(arm_length))
    e3=((centrds_Head1[i][0])+(arm_length)),((centrds_Head1[i][1])+(arm_length))
    e4=((centrds_Head1[i][0])-(arm_length)),((centrds_Head1[i][1])+(arm_length))
    
    # if (i<(len(centrds_Head1))-1):
    #     oc_1=(centrds_Head1[i])
    #     oc_2=(centrds_Head1[i+1])
    # elif (i==(len(centrds_Head1))-1):
    #     oc_1=(centrds_Head1[i])
    #     oc_2=(centrds_Head1[i-1])
    # om_head1.append(slope_head(oc_1[0],oc_1[1],oc_2[0],oc_2[1]))
    # ophi_head1.append(math.degrees(math.atan(om_head1[i])))
    # dxx=oc_2[0]-oc_1[0]
    # dyy=oc_2[1]-oc_1[1]
    
    # if dxx>0 and dyy>0:   #QUADRANT CHECK
    #     operpangle_head1=(ophi_head1[i]+(60))
    #     otypequad.append(1)
    #     quad(oc_1,oc_2,arm_length,operpangle_head1)
        
    # elif dxx<0 and dyy>0:
    #     operpangle_head1=(ophi_head1[i]-(60))
    #     otypequad.append(2)
    #     quad(oc_1,oc_2,arm_length,operpangle_head1)
        
    # elif dxx<0 and dyy<0:
    #     operpangle_head1=(ophi_head1[i]-(60))
    #     otypequad.append(3)
    #     quad(oc_1,oc_2,arm_length,operpangle_head1)

    # elif dxx>0 and dyy<0:
    #     operpangle_head1=(ophi_head1[i]+(60))
    #     otypequad.append(4)
    #     quad(oc_1,oc_2,arm_length,operpangle_head1)

    # oanglecheck.append(operpangle_head1)
    # oc_3=(rect[i][0],rect[i][1])
    # oc_4=(rect[i][2],rect[i][3])
    
    ocoord_1.append(e1)
    ocoord_2.append(e2)
    ocoord_3.append(e3)
    ocoord_4.append(e4)    


#     #INNER BASE HEAD2
# rect=[]
# for j in range(len(centrds_Head2)):
#     if (j<(len(centrds_Head2))-1):
#         ic_1=(centrds_Head2[j])
#         ic_2=(centrds_Head2[j+1])
#     elif (j==(len(centrds_Head2))-1):
#         ic_1=(centrds_Head2[j])
#         ic_2=(centrds_Head2[0])
#     im_head1.append(slope_head(ic_1[0],ic_1[1],ic_2[0],ic_2[1]))
#     iphi_head1.append(math.degrees(math.atan(im_head1[j])))
#     dxx=ic_2[0]-ic_1[0]
#     dyy=ic_2[1]-ic_1[1]
    
#     if dxx>0 and dyy>0:   #QUADRANT CHECK
#         iperpangle_head1=(iphi_head1[j]-(90))
#         itypequad.append(1)
#         quad(ic_1,ic_2,arm_length,iperpangle_head1)
        
#     elif dxx<0 and dyy>0:
#         iperpangle_head1=(iphi_head1[j]+(90))
#         itypequad.append(2)
#         quad(ic_1,ic_2,arm_length,iperpangle_head1)
        
#     elif dxx<0 and dyy<0:
#         iperpangle_head1=(iphi_head1[j]+(90))
#         itypequad.append(3)
#         quad(ic_1,ic_2,arm_length,iperpangle_head1)

#     elif dxx>0 and dyy<0:
#         iperpangle_head1=(iphi_head1[j]-(90))
#         itypequad.append(4)
#         quad(ic_1,ic_2,arm_length,iperpangle_head1)

#     ianglecheck.append(iperpangle_head1)
#     ic_3=(rect[j][0],rect[j][1])
#     ic_4=(rect[j][2],rect[j][3])
    
#     icoord_1.append(ic_1)
#     icoord_2.append(ic_2)
#     icoord_3.append(ic_3)
#     icoord_4.append(ic_4)   

#CENTROID CHECK
    #AREA OF TRIANGLE
yy=[]
def area_tri(p1,p2,p3):
    return abs(0.5*((p1[0]*(p2[1]-p3[1]))+(p2[0]*(p3[1]-p1[1]))+(p3[0]*(p1[1]-p2[1]))))
    #AREA OF RECTANGLE
def area_rect(r1,r2,r3,r4):
    rect_breadth=(math.sqrt(((r1[0]-r2[0])**2)+((r1[1]-r2[1])**2)))
    rect_length=(math.sqrt(((r2[0]-r3[0])**2)+((r2[1]-r3[1])**2)))
    rect_area=rect_breadth*rect_length
    rect_area1=0.5*abs(((r1[1]-r3[1])*(r4[0]-r2[0]))+((r1[0]-r3[0])*(r2[1]-r4[1])))
    return rect_area1

    #SUMMATION OF AREAS OF TRIANGLES INSIDE THE RECTANGLE
def sum_area_tri(r1,r2,r3,r4,cent):
    ta1=area_tri(r1,r2,cent)
    ta2=area_tri(r2,r3,cent)
    ta3=area_tri(r3,r4,cent)
    ta4=area_tri(r4,r1,cent)
    tri_comb_area=(ta1+ta2+ta3+ta4)
    return tri_comb_area

    # MEASURE THE DISTANCE BETWEEN THE TWO COORDINATES
def dist_measure(x1,y1,x2,y2):
    return abs(math.sqrt(((y2-y1)**2)+((x2-x1)**2)))


#OUTER BASE CSP
ocentroid_opt=[]
for m in range(len(ocoord_1)):
    ocent=[]
    for k in range(len(A)):
        ocent.append((A[k]*0.0254,B[k]*0.0254))  
    head_cent=centrds_Head1[m]              
    oc1=ocoord_1[m]
    oc2=ocoord_2[m]
    oc3=ocoord_3[m]
    oc4=ocoord_4[m]
    problem = Problem(BacktrackingSolver())
    problem.addVariable("d", (ocent))
    problem.addConstraint(lambda d:((area_rect(oc1,oc2,oc3,oc4))>=(sum_area_tri(oc1,oc2,oc3,oc4,d))),("d"))
    problem.addConstraint(lambda d:(dist_measure(d[0],d[1],head_cent[0],head_cent[1])<=arm_length),("d"))
    #problem.addConstraint(lambda d:(dist_measure(d[0],d[1],oc2[0],oc2[1])>=10),("d"))
    ocentroid_opt.append(problem.getSolutions())

# SELECTING NEAREST CENTROID AMONGST THE POSSIBLE CENTROIDS
ocentroid_headpair=[]
ocurrent_location=[(0,0)]
ocurrent_position=[]
odist_final=[]
odist_index=[]
ocentroid_final=[]
for n in range(len(ocentroid_opt)):
    odist=[]
    for q in range(len(ocentroid_opt[n])):
        if (len(ocentroid_opt[n])!=0):
            ocentroid_headpair.append(ocentroid_opt[n][q])
            odist.append([dist_measure(ocurrent_location[n][0],ocurrent_location[n][1],ocentroid_headpair[q]["d"][0],ocentroid_headpair[q]["d"][1])])

        else:
            ocentroid_headpair.append([])
            odist.append([])
    # IF CONDITION FOR FINDING LEAST DISTANCE CENTROID AND ITS INDEX
    if (len(odist)>=1):
        odist_final.append(min(odist))
        odist_index.append([odist.index(odist_final[n])])      
    else:
        odist_final.append(odist)
        odist_index.append([])
    # IDENTIFYING THE COORDINATES OF THE SELECTED CENTROID    
    if (len(odist_index[n])>=1):
        h=odist_index[n][0]
        ocentroid_final.append(ocentroid_opt[n][h]["d"])
    else:
        ocentroid_final.append(ocentroid_final[n-1])
    occc=ocentroid_final[n]
    p_r,p_c=co2po(occc[0]/0.0254,occc[1]/0.0254)
    ocurrent_location.append((occc[0],occc[1]))
    ocurrent_position.append((p_r,p_c))


# #INNER BASE CSP
# icentroid_opt=[]
# for m in range(len(icoord_1)):
#     icent=[]
#     for k in range(len(A)):
#         icent.append((A[k],B[k]))                
#     ic1=icoord_1[m]
#     ic2=icoord_2[m]
#     ic3=icoord_3[m]
#     ic4=icoord_4[m]
#     #oc_final=ocentroid_final[m]
#     problem = Problem(BacktrackingSolver())
#     problem.addVariable("c", (icent))
#     problem.addConstraint(lambda c:((area_rect(ic1,ic2,ic3,ic4))>=(sum_area_tri(ic1,ic2,ic3,ic4,c))),("c"))
#     #problem.addConstraint(lambda c:(dist_measure(ic[0],ic[1],oc_final[0],oc_final[1])>23),("c"))
#     problem.addConstraint(lambda c:(dist_measure(ic1[0],ic1[1],c[0],c[1])>=11.5),("c"))
#     problem.addConstraint(lambda c:(dist_measure(ic2[0],ic2[1],c[0],c[1])>=11.5),("c"))
#     icentroid_opt.append(problem.getSolutions())

# # SELECTING NEAREST CENTROID AMONGST THE POSSIBLE CENTROIDS
# icentroid_headpair=[]
# icurrent_location=[(0,0)]
# icurrent_position=[]
# idist_final=[]
# idist_index=[]
# icentroid_final=[]
# for n in range(len(icentroid_opt)):
#     idist=[]
#     for q in range(len(icentroid_opt[n])):
#         if (len(icentroid_opt[n])!=0):
#             icentroid_headpair.append(icentroid_opt[n][q])
#             idist.append([dist_measure(icurrent_location[n][0],icurrent_location[n][1],icentroid_headpair[q]["c"][0],icentroid_headpair[q]["c"][1])])

#         else:
#             icentroid_headpair.append([])
#             idist.append([])
#     # IF CONDITION FOR FINDING LEAST DISTANCE CENTROID AND ITS INDEX
#     if (len(idist)>=1):
#         idist_final.append(min(idist))
#         idist_index.append([idist.index(idist_final[n])])      
#     else:
#         idist_final.append(idist)
#         idist_index.append([])
#     # IDENTIFYING THE COORDINATES OF THE SELECTED CENTROID    
#     if (len(idist_index[n])>=1):
#         g=idist_index[n][0]
#         icentroid_final.append(icentroid_opt[n][g]["c"])
#     else:
#         icentroid_final.append(icentroid_final[n-1])
#     iccc=icentroid_final[n]
#     p_r1,p_c1=co2po(iccc[0],iccc[1])
#     icurrent_location.append((iccc[0],iccc[1]))
#     icurrent_position.append((p_r1,p_c1))
