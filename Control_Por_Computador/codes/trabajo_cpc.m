%%% Init
clc
clear all
clc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Trabajo de control por computador
%%%%% Integrantes %%%%%
%%% Ana Rubio Bustos
%%% Gonzalo Guillamón Martín
%%% Jorge Benavides Macías
% Definimos las variables DNI Gonzalo 
ki = 0.0544;
kb = 0.0444;
jep = 0.0064;
beq = 0.0344;
ra = 0.3438;
la = 0.0054;
kr = 3.8197;
kw = 1;
n = 0.1;
T = 0.05;
ka = 1;
%%%%% Actividad 2 %%%%%
% Función de transferencia en tiempo continuo posición y velocidad
Gposicion = tf([n*ki],[la*jep (la*beq)+(ra*jep) (ra*beq)+(n*ki*kb) 0]); %tipo 1
Gvelocidad =tf([n*ki],[la*jep (la*beq)+(ra*jep) (ra*beq)+(n*ki*kb)]); %tipo 0
%%%%% Actividad 3 %%%%%
% Función de transferencia en tiempo discreto para con tiempo de muestreo 
% T = 0.05s.
Gposicionz = c2d(Gposicion,T,'zoh');
% Representación funciones de transferencia Continua - Discreta
figure
step(Gposicion)
hold on
step(Gposicionz)
xlim([0 1])
title('Función de transferencia continua vs discreta.')
legend('Función continua','Función discreta')
grid on
%%%%% Actividad 4 %%%%%
% Genera el modelo de espacios A,B,C,D para tiempo continuo
[A,B,C,D] = ssdata(ss(Gposicion));
% Genera el modelo de espacios A,B,Cz,Dz para tiempo discreto
[G,H,Cz,Dz,T]=ssdata(ss(Gposicionz)); 
%%%%% Actividad 5 %%%%%
Gposicion_discreto = c2d(Gposicion*kr,0.05,'zoh')
Gvelocidad_discreto = c2d(Gvelocidad*kw,0.05,'zoh')
%%%%% Actividad 6 %%%%%
% Represetación en bucle cerrado para calcular las especificaciones de
% respuesta transitoria
transient_ans = kr*feedback(Gposicionz,kr)
%ltiview('step',transient_ans)
%%%%% Actividad 7 %%%%%
%rltool(Gposicionz);
k_ganancia_critica = margin(Gposicionz)    
%%%%% Actividad 8 %%%%%
%rltool(Gposicionz);
%%%%% Actividad 9 %%%%%
%rltool(Gposicionz);
%Obtenemos el siguiente controlador:
PD_9 = zpk([0.7641],[0.009156],16.404,T)
ltiview('step',kr*feedback(PD_9*Gposicionz,kr))
%%%%% Actividad 10 %%%%%
PID_10 = zpk([0.9945 0.7613],[1 -0.5366],41.625,T)
%step
%ltiview('step',kr*feedback(PDI_10*Gposicionz,kr))
%rampa
t=0:0.05:25;
alpha=1;
rampa=alpha*t;    
%ltiview('lsim',kr*feedback(PDI_10*Gposicionz,kr))
figure
stem(t,rampa);
hold on
[y,t]=lsim(kr*feedback(PID_10*Gposicionz,kr),rampa,t);
stem(t,y)
grid on
title('Funcionamiento del controlador PID')
legend('Función continua','Función discreta')
%%%%% Actividad 11 %%%%
%Controlador PD - Actividad 9
PD_11 = pid(PD_9)
%Controlador PDI - Actividad 10
PID_11 = pid(PID_10)
%%%%% Actividad 12 %%%%
%%%%% Actividad 13 %%%%
[Kc,Pm,Wg]=margin(kr*Gposicion);
Tc=2*pi/Wg;
Kp=0.75*Kc;
Ti=Tc/1.6;
Ki=Kp/Ti;
Td=Tc/10;
Kd=Td*Kp;
tf=0.01;
pidZN=pid(Kp,Ki,Kd,tf);
rltool(Gposicion,pidZN);
pidZNz=c2d(pidZN,T,'trapezoidal');
rltool(Gposicionz,pidZNz);
%ya tenemos el pidZN que queremos con el criterio del 25%
%%%%% Actividad 14 %%%%
%no se puede hacer el ziegler nichols en cerrado pero vamos a probar en
%abierto
[Kc,Pm,Wcg] = margin(Gvelocidad)
Gvelocidadz = c2d(Gvelocidad,0.05,'zoh')
step(Gvelocidadz)%Medimos t0.632 y T0.284 que corresponderan con T1 y T2 respectivamente
T1 = 0.2;
T2 = 0.1;
d = (3*(T1-T2))/2;
tau = T1-d;
d/tau
%No podemos hacer en bucle abierto, ya que d/tau no esta en el intervalo
%0.15<d/tau<0.6
%%%%% Actividad 15 %%%%
%%% Sintesis directa
step(Gvelocidad)
pole(Gvelocidad)
tau1 = 1/63.5465
tau2 = 1/5.4951
lamda = 6
k = 0.451;kp=(tau1+tau2)/(k*lamda);
ti = tau1+tau2;
td = (tau1*tau2)/(tau1+tau2);
PID = pidstd(kp,ti,td)
C = c2d(PID,0.05,'matched')
PID_ANA = pid(0.0731,0.37,0.00106,45)
%%% Controlador Rivera Morari
k=0.451
t632=0.119;t283=0.0773;
d=3*(t632-t283)/2;tau=t632-d;
0.2*tau
lamda=1;
kp=tau/(k*lamda);ti=tau;
PI=pidstd(kp,ti)
C=c2d(PI,0.05,'matched')
%%%%% Actividad 16 %%%%
% Insertar las matrices obtenidas a mano
a = 0.00544; b = 3.456e-5;c = 0.002386;d=0.01207;
Avelocidad = [0 1;-d/b -c/b]; Bvelocidad = [0;a/b]; Dvelocidad_discreto = 0; Cvelocidad_discreto = [1 0];
A_hat = [Avelocidad zeros(2,1);-Cvelocidad_discreto 0];
B_hat = [Bvelocidad;0];
C_hat = [Cvelocidad_discreto 0];
P = [Avelocidad Bvelocidad;-Cvelocidad_discreto 0] 
rank(P)
polos = [-10 -10 -40]
k_hat = acker(A_hat,B_hat,polos)
A_buclecerrado = A_hat-B_hat*k_hat;
B_buclecerrado = [0 0 1]';
Gvelocidad_controlado = ss(A_buclecerrado,B_buclecerrado,C_hat,0)
eig(A_buclecerrado)
step(Gvelocidad_controlado)
%%%%% Actividad 17 %%%%
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