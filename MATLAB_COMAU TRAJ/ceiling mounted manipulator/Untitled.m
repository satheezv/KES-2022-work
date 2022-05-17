syms L1 L2 L3 L4 L5 L6 Q1 Q2 Q3 Q4 Q5 Q6

dhparams = [0   	0       -.165   0;
            0       pi/2   -0.125   0;
            -.270   0       0       0;
            -.070   pi/2	0       0;
            0       pi/2	0.302   0;
            0      -pi/2    0.072   0];

alphaa=[0,90,0,90,90,-90]; % this is the alpha value for all  the link
a=[0,0,-270, -70, 0, 0]; % Length of the Link
d=[-165,-125,0,0,374,0]; %Offset
%Q=[Q1,Q2,Q3,Q4,Q5,Q6]; % joint angle variation
Q=[0,0,1.5708,0,0,0];
%%Transformation Matrices
for i=1:6
    switch i
        case 1
            T01= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
        case 2
            T12= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
        case 3
            T23= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
        case 4
            T34= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
        case 5
            T45= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
        case 6
            T56= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
    end
end
T01 % first link with respect to base
T02= T01*T12
%you are looking for T05 then
T06= T01*T12*T23*T34*T45*T56
