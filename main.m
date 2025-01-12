clear all;
clc;

% Generate and understand a first configuration of the robot within these limits
%parameters
theta1 = 0;
theta2 = 0;
rho1 = 0;

figure(2);
subplot(1, 2, 1);
RRP3D(theta1,theta2,rho1);
hold on;
printing(30,60,0);
title('3D View of RRP Robot');
grid on;

subplot(1, 2, 2);
RRP3D(theta1,theta2,rho1);
hold on;
printing(30,60,0);
view(0, 90);  % Vue de dessus   
grid on;
hold off;



syms q1 q2 rho p;

alpha = [0 ; -p/2; 0]; % Here p is for pi because even if I use simplify(), the simplification is not ok
d = [0;0 ; 50+rho];
r = [45; 20; 0];
theta = [q1; q2; 0];

T0Tn = DenaHart(alpha, d, theta, r);
disp(T0Tn);
D = T0Tn(1:3,4);
disp(D); 
DKP = double(subs(D,{q1 q2 rho p},{theta1 theta2 rho1 pi}))

X = 20*sin(q1) + cos(q1)*cos(q2)*(rho + 50);
Y = cos(q2)*sin(q1)*(rho + 50) + 20*cos(q1);
Z = sin(q2)*(rho + 50) + 45;

% pos is here equal to D but it's the symplify version because I enter by
% hand X,Y,Z.
pos = [X; Y; Z];
J = jacobian(pos, [q1, q2, rho]);
disp('Jacobian Matrix (Symbolic):');
disp(simplify(J));


% Simulation DKP

tf = 15;
f = 3 %Hz
step = round(tf * f);%45
t = linspace(0, tf, step);

syms q1 q2 rho p;

alpha = [0 ; -p/2; 0];
r = [45; 20; 0];
theta = [q1; q2; 0];
qi = [0; 0; 0]
qf = [pi/3; -pi/6; 15];

rp = 10.*((t./tf).^3) - 15.*((t./tf).^4) + 6.*((t./tf).^5);
for i = 1:step

    th1(i) = qi(1) + (qf(1) - qi(1)) * rp(i);
    th2(i) = qi(2) + (qf(2) - qi(2)) * rp(i); 
    rhoo(i) = qi(3) + (qf(3) - qi(3)) * rp(i); 

    T0Tn = DenaHart(alpha,[0;0 ; 50+rhoo(i)], [th1(i); th2(i); 0], r);
    D = T0Tn(1:3,4); 
    DKP = double(subs(D,{q1 q2 rho p},{th1(i) th2(i) rhoo(i) pi}));
    
    %3D
    figure(3);
    subplot(1, 2, 1);
    RRP3D(th1(i), th2(i), rhoo(i));
    hold on;
    printing(30,60,0);
    title('3D View of RRP Robot');
    grid on;
    hold off;

    subplot(1, 2, 2);
    RRP3D(th1(i), th2(i), rhoo(i));
    hold on;
    printing(30,60,0);
    title('Top View of SCARA Robot');
    view(0, 90);  % Vue de dessus   
    label = sprintf('X=%.1f\nY=%.1f\nZ=%.1f\nTheta1=%.1f\nTheta2=%.1f\nRho=%.1f', DKP(1), DKP(2), DKP(3),th1(i),th2(i),rhoo(i) );
    text(75, -50, label, 'FontSize', 10, 'Color', 'k', ...
        'BackgroundColor', 'w', 'EdgeColor', 'k', 'Margin', 5, ...
        'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
    grid on;
    pause(0.05);  %Add pause for animation effect
    hold off;
    hold off;
end




% IKP
clear all;
clc;
tf = 15;
f = 3 %Hz
step = round(tf * f);%45
t = linspace(0, tf, step);

syms q1 q2 rho p;

alpha = [0 ; -p/2; 0];
r = [45; 20; 0];
theta = [q1; q2; 0];

X_i = [50, 35.25 , 35.25 , 32.25 ,30, 30 ];
Y_i = [20,  60   ,  60   , 60    , 62.25,65.25 ];
Z_i = [45,  45   ,   0   ,  0    ,0,0];
rp = 10.*((t./tf).^3) - 15.*((t./tf).^4) + 6.*((t./tf).^5);

% Trajectoire rectiligne 1
for j = 1:(length(X_i)-1)
    for i = 1:step
        x_i= X_i(j)+(X_i(j+1)-X_i(j))*rp(i);
        y_i= Y_i(j)+(Y_i(j+1)-Y_i(j))*rp(i);
        z_i = Z_i(j)+(Z_i(j+1)-Z_i(j))*rp(i);

        [theta1, theta2, rhoo] = IKP(x_i, y_i, z_i)
        q(:,i) =[theta1;theta2;rhoo];

        T0Tn = DenaHart(alpha,[0;0 ; 50+rhoo], [theta1; theta2; 0], r);    
        %3D
        figure(1);
        subplot(1, 2, 1);
        RRP3D(theta1, theta2, rhoo);
        hold on;
        printing(30,60,0);
        title('3D View of RRP Robot');
        grid on;
        hold off;

        subplot(1, 2, 2);
        RRP3D(theta1, theta2, rhoo);
        hold on;
        printing(30,60,0);
        title('Top View of SCARA Robot');
        view(0, 90);  % Vue de dessus   
        label = sprintf('X=%.1f\nY=%.1f\nZ=%.1f\nTheta1=%.1f\nTheta2=%.1f\nRho=%.1f', x_i, y_i, z_i,theta1,theta2,rhoo );
        text(75, -50, label, 'FontSize', 10, 'Color', 'k', ...
            'BackgroundColor', 'w', 'EdgeColor', 'k', 'Margin', 5, ...
            'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
        grid on;
        pause(0.05);  %Add pause for animation effect
        hold off;
        hold off;
    end
end

 C = [30;67.5;0];
rayon = 2.25;

% Trajectoire circulaire 1
    for i = 1:step
        angle = -pi/2 + pi * (i / (step - 1));  % -pi/2 à pi/2
        %angle = pi * (i / step);  % 0 à pi

        x_i = C(1) + rayon * cos(angle);  
        y_i = C(2) + rayon * sin(angle);  
        z_i = C(3);  

        [theta1, theta2, rhoo] = IKP(x_i, y_i, z_i)
        q(:,i) =[theta1;theta2;rhoo];

        T0Tn = DenaHart(alpha,[0;0 ; 50+rhoo], [theta1; theta2; 0], r);    
        %3D
        figure(1);
        subplot(1, 2, 1);
        RRP3D(theta1, theta2, rhoo);
        hold on;
        printing(30,60,0);
        title('3D View of RRP Robot');
        grid on;
        hold off;

        subplot(1, 2, 2);
        RRP3D(theta1, theta2, rhoo);
        hold on;
        printing(30,60,0);
        title('Top View of SCARA Robot');
        view(0, 90);  % Vue de dessus   
        label = sprintf('X=%.1f\nY=%.1f\nZ=%.1f\nTheta1=%.1f\nTheta2=%.1f\nRho=%.1f', x_i, y_i, z_i,theta1,theta2,rhoo );
        text(75, -50, label, 'FontSize', 10, 'Color', 'k', ...
            'BackgroundColor', 'w', 'EdgeColor', 'k', 'Margin', 5, ...
            'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
        grid on;
        pause(0.05);  %Add pause for animation effect
        hold off;
        hold off;
    end


% Trajectoire rectiligne 2
X_i = [30     , 30  , 32.25 , 35.25];
Y_i = [69.75  ,72.75,  75   ,  75 ];
Z_i = [0      ,   0 ,   0   ,0 ];

for j = 1:(length(X_i)-1)
    for i = 1:step
        x_i= X_i(j)+(X_i(j+1)-X_i(j))*rp(i);
        y_i= Y_i(j)+(Y_i(j+1)-Y_i(j))*rp(i);
        z_i = Z_i(j)+(Z_i(j+1)-Z_i(j))*rp(i);

        [theta1, theta2, rhoo] = IKP(x_i, y_i, z_i)
        q(:,i) =[theta1;theta2;rhoo];

        T0Tn = DenaHart(alpha,[0;0 ; 50+rhoo], [theta1; theta2; 0], r);    
        %3D
        figure(1);
        subplot(1, 2, 1);
        RRP3D(theta1, theta2, rhoo);
        hold on;
        printing(30,60,0);
        title('3D View of RRP Robot');
        grid on;
        hold off;

        subplot(1, 2, 2);
        RRP3D(theta1, theta2, rhoo);
        hold on;
        printing(30,60,0);
        title('Top View of SCARA Robot');
        view(0, 90);  % Vue de dessus   
        label = sprintf('X=%.1f\nY=%.1f\nZ=%.1f\nTheta1=%.1f\nTheta2=%.1f\nRho=%.1f', x_i, y_i, z_i,theta1,theta2,rhoo );
        text(75, -50, label, 'FontSize', 10, 'Color', 'k', ...
            'BackgroundColor', 'w', 'EdgeColor', 'k', 'Margin', 5, ...
            'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
        grid on;
        pause(0.05);  %Add pause for animation effect
        hold off;
        hold off;
    end
end


C = [37.5;75;0];

% Trajectoire circulaire 2
    for i = 1:step
        %angle = -pi/2 + pi * (i / (step - 1));  % -pi/2 à pi/2
        angle = -pi + pi * (i / (step - 1));  % -pi à 0
        %angle = pi * (i / step);  % 0 à pi

        x_i = C(1) + rayon * cos(angle);  
        y_i = C(2) + rayon * sin(angle);  
        z_i = C(3);  

        [theta1, theta2, rhoo] = IKP(x_i, y_i, z_i)
        q(:,i) =[theta1;theta2;rhoo];

        T0Tn = DenaHart(alpha,[0;0 ; 50+rhoo], [theta1; theta2; 0], r);    
        %3D
        figure(1);
        subplot(1, 2, 1);
        RRP3D(theta1, theta2, rhoo);
        hold on;
        printing(30,60,0);
        title('3D View of RRP Robot');
        grid on;
        hold off;

        subplot(1, 2, 2);
        RRP3D(theta1, theta2, rhoo);
        hold on;
        printing(30,60,0);
        title('Top View of SCARA Robot');
        view(0, 90);  % Vue de dessus   
        label = sprintf('X=%.1f\nY=%.1f\nZ=%.1f\nTheta1=%.1f\nTheta2=%.1f\nRho=%.1f', x_i, y_i, z_i,theta1,theta2,rhoo );
        text(75, -50, label, 'FontSize', 10, 'Color', 'k', ...
            'BackgroundColor', 'w', 'EdgeColor', 'k', 'Margin', 5, ...
            'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
        grid on;
        pause(0.05);  %Add pause for animation effect
        hold off;
        hold off;
    end

% Trajectoire rectiligne 3
X_i = [39.75     , 42.75 , 45 , 45];
Y_i = [75  ,75,  72.75  ,  69.75 ];
Z_i = [0      ,   0 ,   0   ,0 ];

for j = 1:(length(X_i)-1)
    for i = 1:step
        x_i= X_i(j)+(X_i(j+1)-X_i(j))*rp(i);
        y_i= Y_i(j)+(Y_i(j+1)-Y_i(j))*rp(i);
        z_i = Z_i(j)+(Z_i(j+1)-Z_i(j))*rp(i);

        [theta1, theta2, rhoo] = IKP(x_i, y_i, z_i)
        q(:,i) =[theta1;theta2;rhoo];

        T0Tn = DenaHart(alpha,[0;0 ; 50+rhoo], [theta1; theta2; 0], r);    
        %3D
        figure(1);
        subplot(1, 2, 1);
        RRP3D(theta1, theta2, rhoo);
        hold on;
        printing(30,60,0);
        title('3D View of RRP Robot');
        grid on;
        hold off;

        subplot(1, 2, 2);
        RRP3D(theta1, theta2, rhoo);
        hold on;
        printing(30,60,0);
        title('Top View of SCARA Robot');
        view(0, 90);  % Vue de dessus   
        label = sprintf('X=%.1f\nY=%.1f\nZ=%.1f\nTheta1=%.1f\nTheta2=%.1f\nRho=%.1f', x_i, y_i, z_i,theta1,theta2,rhoo );
        text(75, -50, label, 'FontSize', 10, 'Color', 'k', ...
            'BackgroundColor', 'w', 'EdgeColor', 'k', 'Margin', 5, ...
            'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
        grid on;
        pause(0.05);  %Add pause for animation effect
        hold off;
        hold off;
    end
end

C = [45;67.5;0];

% Trajectoire circulaire 1
    for i = 1:step

        angle = pi/2 + pi * (i / (step - 1));  % pi/2 à -pi/2

        x_i = C(1) + rayon * cos(angle);  
        y_i = C(2) + rayon * sin(angle);  
        z_i = C(3);  

        [theta1, theta2, rhoo] = IKP(x_i, y_i, z_i)
        q(:,i) =[theta1;theta2;rhoo];

        T0Tn = DenaHart(alpha,[0;0 ; 50+rhoo], [theta1; theta2; 0], r);    
        %3D
        figure(1);
        subplot(1, 2, 1);
        RRP3D(theta1, theta2, rhoo);
        hold on;
        printing(30,60,0);
        title('3D View of RRP Robot');
        grid on;
        hold off;

        subplot(1, 2, 2);
        RRP3D(theta1, theta2, rhoo);
        hold on;
        printing(30,60,0);
        title('Top View of SCARA Robot');
        view(0, 90);  % Vue de dessus   
        label = sprintf('X=%.1f\nY=%.1f\nZ=%.1f\nTheta1=%.1f\nTheta2=%.1f\nRho=%.1f', x_i, y_i, z_i,theta1,theta2,rhoo );
        text(75, -50, label, 'FontSize', 10, 'Color', 'k', ...
            'BackgroundColor', 'w', 'EdgeColor', 'k', 'Margin', 5, ...
            'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
        grid on;
        pause(0.05);  %Add pause for animation effect
        hold off;
        hold off;
    end

% Trajectoire rectiligne 3
X_i = [45    , 45, 42.75 , 39.75];
Y_i = [65.25  ,62.25 ,  60  ,  60];
Z_i = [0      ,   0 ,   0   ,0 ];

for j = 1:(length(X_i)-1)
    for i = 1:step
        x_i= X_i(j)+(X_i(j+1)-X_i(j))*rp(i);
        y_i= Y_i(j)+(Y_i(j+1)-Y_i(j))*rp(i);
        z_i = Z_i(j)+(Z_i(j+1)-Z_i(j))*rp(i);

        [theta1, theta2, rhoo] = IKP(x_i, y_i, z_i)
        q(:,i) =[theta1;theta2;rhoo];

        T0Tn = DenaHart(alpha,[0;0 ; 50+rhoo], [theta1; theta2; 0], r);    
        %3D
        figure(1);
        subplot(1, 2, 1);
        RRP3D(theta1, theta2, rhoo);
        hold on;
        printing(30,60,0);
        title('3D View of RRP Robot');
        grid on;
        hold off;

        subplot(1, 2, 2);
        RRP3D(theta1, theta2, rhoo);
        hold on;
        printing(30,60,0);
        title('Top View of SCARA Robot');
        view(0, 90);  % Vue de dessus   
        label = sprintf('X=%.1f\nY=%.1f\nZ=%.1f\nTheta1=%.1f\nTheta2=%.1f\nRho=%.1f', x_i, y_i, z_i,theta1,theta2,rhoo );
        text(75, -50, label, 'FontSize', 10, 'Color', 'k', ...
            'BackgroundColor', 'w', 'EdgeColor', 'k', 'Margin', 5, ...
            'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
        grid on;
        pause(0.05);  %Add pause for animation effect
        hold off;
        hold off;
    end
end

C = [37.5;60;0];

% Trajectoire circulaire 4
    for i = 1:step

        angle = pi * (i / (step - 1));  % pi/2 à -pi/2

        x_i = C(1) + rayon * cos(angle);  
        y_i = C(2) + rayon * sin(angle);  
        z_i = C(3);  

        [theta1, theta2, rhoo] = IKP(x_i, y_i, z_i)
        q(:,i) =[theta1;theta2;rhoo];

        T0Tn = DenaHart(alpha,[0;0 ; 50+rhoo], [theta1; theta2; 0], r);    
        %3D
        figure(1);
        subplot(1, 2, 1);
        RRP3D(theta1, theta2, rhoo);
        hold on;
        printing(30,60,0);
        title('3D View of RRP Robot');
        grid on;
        hold off;

        subplot(1, 2, 2);
        RRP3D(theta1, theta2, rhoo);
        hold on;
        printing(30,60,0);
        title('Top View of SCARA Robot');
        view(0, 90);  % Vue de dessus   
        label = sprintf('X=%.1f\nY=%.1f\nZ=%.1f\nTheta1=%.1f\nTheta2=%.1f\nRho=%.1f', x_i, y_i, z_i,theta1,theta2,rhoo );
        text(75, -50, label, 'FontSize', 10, 'Color', 'k', ...
            'BackgroundColor', 'w', 'EdgeColor', 'k', 'Margin', 5, ...
            'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
        grid on;
        pause(0.05);  %Add pause for animation effect
        hold off;
        hold off;
    end

 % Monte end effector
X_i = [35.25   , 35.25];
Y_i = [60  ,60];
Z_i = [0      ,   20];

for j = 1:(length(X_i)-1)
    for i = 1:step
        x_i= X_i(j)+(X_i(j+1)-X_i(j))*rp(i);
        y_i= Y_i(j)+(Y_i(j+1)-Y_i(j))*rp(i);
        z_i = Z_i(j)+(Z_i(j+1)-Z_i(j))*rp(i);

        [theta1, theta2, rhoo] = IKP(x_i, y_i, z_i)
        q(:,i) =[theta1;theta2;rhoo];

        T0Tn = DenaHart(alpha,[0;0 ; 50+rhoo], [theta1; theta2; 0], r);    
        %3D
        figure(1);
        subplot(1, 2, 1);
        RRP3D(theta1, theta2, rhoo);
        hold on;
        printing(30,60,0);
        title('3D View of RRP Robot');
        grid on;
        hold off;

        subplot(1, 2, 2);
        RRP3D(theta1, theta2, rhoo);
        hold on;
        printing(30,60,0);
        title('Top View of SCARA Robot');
        view(0, 90);  % Vue de dessus   
        label = sprintf('X=%.1f\nY=%.1f\nZ=%.1f\nTheta1=%.1f\nTheta2=%.1f\nRho=%.1f', x_i, y_i, z_i,theta1,theta2,rhoo );
        text(75, -50, label, 'FontSize', 10, 'Color', 'k', ...
            'BackgroundColor', 'w', 'EdgeColor', 'k', 'Margin', 5, ...
            'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
        grid on;
        pause(0.05);  %Add pause for animation effect
        hold off;
        hold off;
    end
end
