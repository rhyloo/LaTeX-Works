a = 0.00544; b = 3.456e-5;c = 0.002386;d=0.01207;
Avelocidad = [0 1;-d/b -c/b]; Bvelocidad = [0;a/b]; Dvelocidad_discreto = 0; Cvelocidad_discreto = [1 0]; T = 0.05;
sys_discreto = c2d(ss(Avelocidad,Bvelocidad,Cvelocidad_discreto,Dvelocidad_discreto),T,'zoh')
[Gvelocidad_discreto,Hvelocidad_discreto,Cvelocidad_discreto,Dvelocidad_discreto] = ssdata(sys_discreto)
G = [Gvelocidad_discreto [0 0]';-Cvelocidad_discreto*Gvelocidad_discreto 1]
H = [Hvelocidad_discreto;-Cvelocidad_discreto*Hvelocidad_discreto];
P_discreto = [exp(-10*T) exp(-10*T) exp(-40*T)]
k_discreto = acker (G,H,P_discreto);
kd = [k_discreto(1) k_discreto(2)]; kid =-k_discreto(3);
GG = [Gvelocidad_discreto-Hvelocidad_discreto*kd Hvelocidad_discreto*kid;-Cvelocidad_discreto*Gvelocidad_discreto+Cvelocidad_discreto*Hvelocidad_discreto*kd 1-Cvelocidad_discreto*Hvelocidad_discreto*kid]
HH = [0 0 1]';
CC = [Cvelocidad_discreto 0];
servo = ss(GG,HH,CC,0,T)
zpk(servo)
step(servo)