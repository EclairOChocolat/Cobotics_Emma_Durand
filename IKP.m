 function [theta1, theta2, rho] = IKP(xi, yi, zi)
     l = 20;
    d = 50;
    h = 45 ;
    %Calcul de theta1 :
    psi = atan2(yi, xi);
    r = sqrt(xi^2 + yi^2);
    a = asin(l / r);
    theta1 = psi - a;
    
    %Calcul de theta2:
    theta2 = -atan2((zi - h), (r*cos(a)));


    %Calcul de rho :
    r2 = sqrt(xi^2 + yi^2+zi^2);
    %rho = 
    rho = sqrt((r2 - l)^2 + (zi - h)^2) - d;

end


