import matplotlib.pyplot as plt

# Pins
class Coordinate(object):
    def __init__(self,x,y):
        self.x = x
        self.y = y


# COORDINATES OF PINS
P1= Coordinate(12.75,7.93)
P2= Coordinate(26.15,7.93)
P3= Coordinate(39.55,7.93)
P4= Coordinate(52.95,7.93)
P5= Coordinate(66.35,7.93)
P6= Coordinate(79.75,7.93)
P7= Coordinate(93.15,7.93)
P8= Coordinate(6.05,19.54)
P9= Coordinate(19.45,19.54)
P10= Coordinate(32.85,19.54)
P11= Coordinate(46.25,19.54)
P12= Coordinate(59.5,19.54)
P13= Coordinate(73.05,19.54)
P14= Coordinate(86.45,19.54)
P15= Coordinate(99.85,19.54)
P16= Coordinate(12.75,31.14)
P17= Coordinate(26.15,31.14)
P18= Coordinate(39.55,31.14)
P19= Coordinate(52.95,31.14)
P20= Coordinate(66.35,31.14)
P21= Coordinate(79.75,31.14)
P22= Coordinate(93.15,31.14)
P23= Coordinate(6.05,42.75)
P24= Coordinate(19.45,42.75)
P25= Coordinate(32.85,42.75)
P26= Coordinate(46.25,42.75)
P27= Coordinate(59.65,42.75)
P28= Coordinate(73.05,42.75)
P29= Coordinate(86.45,42.75)
P30= Coordinate(99.85,42.75)
P31= Coordinate(12.75,54.35)
P32= Coordinate(26.15,54.35)
P33= Coordinate(39.55,54.35)
P34= Coordinate(52.95,54.35)
P35= Coordinate(66.35,54.35)
P36= Coordinate(79.75,54.35)
P37= Coordinate(93.15,54.35)
P38= Coordinate(6.05,65.95)
P39= Coordinate(19.45,65.95)
P40= Coordinate(32.85,65.95)
P41= Coordinate(46.25,65.95)
P42= Coordinate(59.65,65.95)
P43= Coordinate(73.05,65.95)
P44= Coordinate(86.45,65.95)
P45= Coordinate(99.85,65.95)
P46= Coordinate(12.75,77.56)
P47= Coordinate(26.15,77.56)
P48= Coordinate(39.55,77.56)
P49= Coordinate(52.95,77.56)
P50= Coordinate(66.35,77.56)
P51= Coordinate(79.75,77.56)
P52= Coordinate(93.15,77.56)

# CENTROID OF THE PIN COMBINATION

def avg(l,m,n):
    return ((l+m+n)/3)


# X-AXIS AVG VALUE FOR CENTROID
x_c = [avg(P1.x,P8.x,P9.x),avg(P1.x,P2.x,P9.x),avg(P2.x,P9.x,P10.x), avg(P2.x,P3.x,P10.x),\
       avg(P3.x,P10.x,P11.x),avg(P3.x,P4.x,P11.x),avg(P4.x,P11.x,P12.x), avg(P4.x,P5.x,P12.x),\
       avg(P5.x,P12.x,P13.x),avg(P5.x,P6.x,P13.x),avg(P6.x,P13.x,P14.x),avg(P6.x,P7.x,P14.x),avg(P7.x,P14.x,P15.x),\
    avg(P8.x,P9.x,P16.x),avg(P9.x,P16.x,P17.x),avg(P9.x,P10.x,P17.x),avg(P10.x,P17.x,P18.x),\
    avg(P10.x,P11.x,P18.x),avg(P11.x,P18.x,P19.x),avg(P11.x,P12.x,P19.x),avg(P12.x,P19.x,P20.x),\
    avg(P12.x,P13.x,P20.x),avg(P13.x,P20.x,P21.x),avg(P13.x,P14.x,P21.x),avg(P14.x,P21.x,P22.x),avg(P14.x,P15.x,P22.x),\
    avg(P16.x,P23.x,P24.x),avg(P16.x,P17.x,P24.x),avg(P17.x,P24.x,P25.x),avg(P17.x,P18.x,P25.x),\
    avg(P18.x,P25.x,P26.x),avg(P18.x,P19.x,P26.x),avg(P19.x,P26.x,P27.x),avg(P19.x,P20.x,P27.x),\
    avg(P20.x,P27.x,P28.x),avg(P20.x,P21.x,P28.x),avg(P21.x,P28.x,P29.x),avg(P21.x,P22.x,P29.x),avg(P22.x,P29.x,P30.x),\
    avg(P23.x,P24.x,P31.x),avg(P24.x,P31.x,P32.x),avg(P24.x,P25.x,P32.x),avg(P25.x,P32.x,P33.x),\
    avg(P25.x,P26.x,P33.x),avg(P26.x,P33.x,P34.x),avg(P26.x,P27.x,P34.x),avg(P27.x,P34.x,P35.x),\
    avg(P27.x,P28.x,P35.x),avg(P28.x,P35.x,P36.x),avg(P28.x,P29.x,P36.x),avg(P29.x,P36.x,P37.x),avg(P29.x,P30.x,P37.x),\
    avg(P31.x,P38.x,P39.x),avg(P31.x,P32.x,P39.x),avg(P32.x,P39.x,P40.x),avg(P32.x,P33.x,P40.x),\
    avg(P33.x,P40.x,P41.x),avg(P33.x,P34.x,P41.x),avg(P34.x,P41.x,P42.x),avg(P34.x,P35.x,P42.x),\
    avg(P35.x,P42.x,P43.x),avg(P35.x,P36.x,P43.x),avg(P36.x,P43.x,P44.x),avg(P36.x,P37.x,P44.x),avg(P37.x,P44.x,P45.x),\
    avg(P38.x,P39.x,P46.x),avg(P39.x,P46.x,P47.x),avg(P39.x,P40.x,P47.x),avg(P40.x,P47.x,P48.x),\
    avg(P40.x,P41.x,P48.x),avg(P41.x,P48.x,P49.x),avg(P41.x,P42.x,P49.x),avg(P42.x,P49.x,P50.x),\
    avg(P42.x,P43.x,P50.x),avg(P43.x,P50.x,P51.x),avg(P43.x,P44.x,P51.x),avg(P44.x,P51.x,P52.x),avg(P44.x,P45.x,P52.x)]

# Y-AXIS AVG VALUE FOR CENTROID
y_c = [avg(P1.y,P8.y,P9.y),avg(P1.y,P2.y,P9.y),avg(P2.y,P9.y,P10.y), avg(P2.y,P3.y,P10.y),\
       avg(P3.y,P10.y,P11.y),avg(P3.y,P4.y,P11.y),avg(P4.y,P11.y,P12.y), avg(P4.y,P5.y,P12.y),\
       avg(P5.y,P12.y,P13.y),avg(P5.y,P6.y,P13.y),avg(P6.y,P13.y,P14.y),avg(P6.y,P7.y,P14.y),avg(P7.y,P14.y,P15.y),\
    avg(P8.y,P9.y,P16.y),avg(P9.y,P16.y,P17.y),avg(P9.y,P10.y,P17.y),avg(P10.y,P17.y,P18.y),\
    avg(P10.y,P11.y,P18.y), avg(P11.y,P18.y,P19.y),avg(P11.y,P12.y,P19.y),avg(P12.y,P19.y,P20.y),\
    avg(P12.y,P13.y,P20.y),avg(P13.y,P20.y,P21.y),avg(P13.y,P14.y,P21.y),avg(P14.y,P21.y,P22.y),avg(P14.y,P15.y,P22.y),\
    avg(P16.y,P23.y,P24.y),avg(P16.y,P17.y,P24.y),avg(P17.y,P24.y,P25.y),avg(P17.y,P18.y,P25.y),\
    avg(P18.y,P25.y,P26.y),avg(P18.y,P19.y,P26.y),avg(P19.y,P26.y,P27.y),avg(P19.y,P20.y,P27.y),\
    avg(P20.y,P27.y,P28.y),avg(P20.y,P21.y,P28.y),avg(P21.y,P28.y,P29.y),avg(P21.y,P22.y,P29.y),avg(P22.y,P29.y,P30.y),\
    avg(P23.y,P24.y,P31.y),avg(P24.y,P31.y,P32.y),avg(P24.y,P25.y,P32.y),avg(P25.y,P32.y,P33.y),\
    avg(P25.y,P26.y,P33.y),avg(P26.y,P33.y,P34.y),avg(P26.y,P27.y,P34.y),avg(P27.y,P34.y,P35.y),\
    avg(P27.y,P28.y,P35.y),avg(P28.y,P35.y,P36.y),avg(P28.y,P29.y,P36.y),avg(P29.y,P36.y,P37.y),avg(P29.y,P30.y,P37.y),\
    avg(P31.y,P38.y,P39.y),avg(P31.y,P32.y,P39.y),avg(P32.y,P39.y,P40.y),avg(P32.y,P33.y,P40.y),\
    avg(P33.y,P40.y,P41.y),avg(P33.y,P34.y,P41.y),avg(P34.y,P41.y,P42.y),avg(P34.y,P35.y,P42.y),\
    avg(P35.y,P42.y,P43.y),avg(P35.y,P36.y,P43.y),avg(P36.y,P43.y,P44.y),avg(P36.y,P37.y,P44.y),avg(P37.y,P44.y,P45.y),\
    avg(P38.y,P39.y,P46.y),avg(P39.y,P46.y,P47.y),avg(P39.y,P40.y,P47.y),avg(P40.y,P47.y,P48.y),\
    avg(P40.y,P41.y,P48.y),avg(P41.y,P48.y,P49.y),avg(P41.y,P42.y,P49.y),avg(P42.y,P49.y,P50.y),\
    avg(P42.y,P43.y,P50.y),avg(P43.y,P50.y,P51.y),avg(P43.y,P44.y,P51.y),avg(P44.y,P51.y,P52.y),avg(P44.y,P45.y,P52.y)]


A=[]
B=[]
for x in range(len(x_c)):
    A.append(round(x_c[x],3))
    B.append(round(y_c[x],3))

