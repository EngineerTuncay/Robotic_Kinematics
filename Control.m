function[sonuc]=Control(q1,q2,d3)
global h1 l2 d2
t01=[cosd(q1) -sind(q1) 0 0;sind(q1) cosd(q1) 0 0;0 0 1 h1;0 0 0 1];
t12=[cosd(q2) -sind(q2) 0 0;0 0 -1 -d2;sind(q2) cosd(q2) 0 0;0 0 0 1];
t23=[1 0 0 0;0 0 1 (l2+d3);0 -1 0 0;0 0 0 1];
t03=t01*t12*t23;
sonuc(1)=t03(1,4);
sonuc(2)=t03(2,4);
sonuc(3)=t03(3,4);
end