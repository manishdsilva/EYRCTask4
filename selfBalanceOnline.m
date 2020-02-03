####%robot parameters
##mb = 0.987; %mass of robot
##mw = 0.025; %mass of wheels
##jb = 0.00383; %moment of inertia about the centre of mass
##r = 0.04; %radius of wheels
##jw = 4E-05; %moment of inertia for the wheels
##l = 0.102; %distance from wheel axle to CoM
##ke = 0.855;
##km = 0.316;
##R = 7.2; %motor resistance
##g = 9.81; %gravity
##b = 0.002; %Viscous friction constant
pkg load control;
pkg load signal
mb = 1.0;               #Robot Mass Kg 
mw = 45.5/1000;         #Wheel Mass Kg
h = 0.20       ;         #height m
w = 0.07        ;        #width m #not distance between wheels
jb = mb/12*(h*h + w*w) #Moment of Inertia about center Kgm^2
r = 0.065              ; #Radius m
jw = 0.5*(mw)*r*r     ;#Moment of Inertia of wheels Kgm^2
l = 0.05              ; ### Distance of wheel to center of mass
ke = 0.855             ;### EMF Constant Vs/rad
km = 8.5/100/2.4       ;## Torque Constant Nm/A
R = 7.2               ;### Motor Resistance ohm
g = 9.81               ; #m/s^2     
b = 0.002               ;### Viscous Friction constant Nms/rad

alp = (2*(R*b - ke*km)*(mb*l*l + mb*r*l +jb))/ R*(2*(jb*jw + jw*l*l*mb +
jb*mw*r*r + l*l*mb*mw*r*r)+jb*mb*r*r);
bet = (-l*l*mb*mb*g*r*r)/(jb*(2*jw + mb*r*r + 2*mw*r*r) + 2*jw*l*l*mb +
2*l*l*mb*mw*r*r);
gam = (-2*(R*b -ke*km)*(2*jw + mb*r*r + 2*mw*r*r + l*mb*r))/(R*r*(2*(jb*jw
+ jw*l*l*mb + jb*mw*r*r + l*l*mb*mw*r*r)+jb*mb*r*r));
delt = (l*mb*g*(2*jw + mb*r*r + 2*mw*r*r))/(2*jb*jw + 2*jw*l*l*mb +
jb*mb*r*r +2*jb*mw*r*r + 2*l*l*mb*mw*r*r);
chi = (km*r)/(R*b - ke*km);
A = [0 1 0 0;
 0 alp bet -r*alp;
 0 0 0 1;
 0 gam delt -r*alp];
B = [0; alp*chi; 0; gam*chi];
C = [1 0 0 0;
 0 1 0 0;
 0 0 1 0;
 0 0 0 1];
D = [0;0;0;0];
Q=C'*C;
Q =   [1 0 0 0 ; 
       0 0 0 0 ; 
       0 0 1 0 ; 
       0 0 0 0];
##[n,d]=ss2tf(A,B,C,D);
##n  %num
##d  %den
sys = ss(A,B,C,D);
sys = c2d(sys,0.01)
R = 1;
[K,S,e] = dlqr(sys.a,sys.b,Q,R);
e
K