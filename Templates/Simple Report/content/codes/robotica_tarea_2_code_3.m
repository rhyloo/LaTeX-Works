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
       A = 0;
       warning('Configuracion degenerada')
       end
    end