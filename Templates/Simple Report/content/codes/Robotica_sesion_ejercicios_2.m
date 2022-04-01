
punto = [100,70,150];
[ATB, alpha, beta] = calculate_transformation(punto);
%% 
% 

ATB
alpha
beta
%% 
% 

view(57,6);
grid on
Base =[1 0 0 0;0 1 0 0; 0 0 1 0; 0 0 0 1];
createFRAME(Base,'b','Base',200);
createFRAME(ATB,'r','ATB',250);
%% 
% *Problema 2*

Identidad = eye(4);
% Roto respecto x
% Roto respecto z
% Roto respecto y
syms alpha beta gamma real
BTH = rotY(gamma)*rotX(alpha)*Identidad*rotZ(beta)
alpha = deg2rad(45);
beta  = deg2rad(30);
gamma = deg2rad(60);
BTH = rotY(gamma)*rotX(alpha)*Identidad*rotZ(beta)

[O,A,T] = tr2OAT(BTH) 
[O,A,T] = tr2OAT(BTH,-1) 

BTH = rotZ(deg2rad(-90))*Identidad*rotX(deg2rad(-90));
BTH_2 = rotY(gamma)*rotX(alpha)*BTH*rotZ(beta)
[O,A,T] = tr2OAT(BTH_2) 
[O,A,T] = tr2OAT(BTH_2,-1) 
%% 
% *Volver a calcular la matriz de orientación detallada en el	 enunciado si	 
% la situación inicial entre el sistema	{B} y {H} se define	de la siguiente manera:	
% El eje* $Z_H$ *coincidecon* $X_B$*,* $X_H$ *con* $-Y_B$ *e* $Y_H$ *con* $-Z_B$*. 
% A partir	de esta situación se realizarán las transformaciones dadas inicialmente.*
% 
% Otra forma de resolver el ejercicio es calcular directamente la matriz de 
% rotación porque los giros son canónicos.
% 
% [0 0 1 0
% 
% -1 0 0 0
% 
% 0  -1 0 0 
% 
% 0 0 0 1]

Base_OAT = [0 1 0 0;0 0 -1 0; -1 0 0 0; 0 0 0 1];
syms O A T real
BTH_OAT = rotZ(O)*Base_OAT*rotY(A)*rotZ(T)
A = deg2rad(-90);
BTH_OAT_degenerada = rotZ(O)*Base_OAT*rotY(A)*rotZ(T)
A = deg2rad(270);
BTH_OAT_degenerada = rotZ(O)*Base_OAT*rotY(A)*rotZ(T)

%%
function [O,A,T] = tr2OAT(matrix,m) 
if nargin==1, m=1; end
M=sign(m);

% BTH_OAT(1,3)^2+BTH_OAT(2,3)^2 == r13^2+ r23^2
% r13^2+ r23^2 == (cos(A))^2
% r33 == -sin(A)

cos_a = M*sqrt(matrix(1,3)^2+matrix(2,3)^2);
A = atan2(-matrix(3,3),cos_a);

    if  abs(cos_a) > 1e-3  
       T = atan2(matrix(3,2)/cos_a,-matrix(3,1)/cos_a);
       O = atan2(matrix(1,3)/cos_a,-matrix(2,3)/cos_a);
    else
       warning("Configuración degenerada")
    end
end

%% 
% 

function [matrix,alpha,beta] = calculate_transformation(vector)
x = vector(1);
y = vector(2);
z = vector(3);

c = sqrt(x^2+y^2);

sin_alpha = y/c;
cos_alpha = x/c;

L = sqrt(c^2+z^2);

sin_beta = z/L;
cos_beta = c/L;

alpha = atan2(sin_alpha,cos_alpha);
beta = atan2(sin_beta,cos_beta);

Identidad = eye(4);

matrix = Identidad*rot('y',-90)*rot('x',rad2deg(alpha))*rot('y',-(rad2deg(beta)))*move(0,0,-sqrt(c^2+z^2));
end