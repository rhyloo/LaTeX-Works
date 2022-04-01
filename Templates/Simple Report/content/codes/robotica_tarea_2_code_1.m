    function [matrix, alpha, beta] = calculate_transformation(vector)
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
    I = eye(4);
    matrix = I*rotY(-pi/2)*rotX(alpha)*rotY(-(beta))*move(0,0,-sqrt(c^2+z^2));
    end