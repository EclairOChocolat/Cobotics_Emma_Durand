function A= printing(Px,Py,Pz)
%% A function named printing used to generate the trajectory of the profile to be printed in robot workspace %%
% INPUT(S):  Px, Py, Pz: The cartesian coordinates of the end-effector of the robot/starting point of the profile to be printed
% OUTPUT(S): A: Vector which saves the coordinates of the end-effector for performing the inverse kinematics using the robot
%%
    % x-y coordinates of the profile%
    % A scaling is applied to make it bigger than actual size %
    % 1.5 scaling is strongly recommended %

    scale=1.5;
    x= scale*[1.5,3.5,6.5,8.5,10,10,10,10,8.5,6.5,3.5,1.5,0,0,0,0,1.5];
    y= scale*[0,0,0,0,1.5,3.5,6.5,8.5,10,10,10,10,8.5,6.5,3.5,1.5,0];
    
    % Updating the x-y-z with respect to input end-effector coordinates %
    x= Px+x;
    y= Py+y;
    z= Pz;

    % Scaling for the radius %

    r=scale*1.5;
    
    % Initialization of A vector to save the coordinates of profile %

    for i = 1 : length(x)
        A(i,1)= x(i);
        A(i,2)= y(i);
        A(i,3)= z;
    end

    % Plot to generate the semi-circle (bottom) %

    C= [x(2)+r,y(2)];
    thh= 0:0.01:pi;
    xx= C(1) + r.*cos(thh);
    yy= C(2) + r.*sin(thh);
    zz= z*ones(1,315);
 
    % Plot sequence for bottom part %

    plot3([x(1),x(2)],[y(1),y(2)],[z,z],'b');
    hold on;
    plot3(xx,yy,zz,'b')
    plot3([x(3),x(4)],[y(3),y(4)],[z,z],'b');
    plot3([x(5),x(4)],[y(5),y(4)],[z,z],'b');
    plot3([x(5),x(6)],[y(5),y(6)],[z,z],'b');
 
    % Plot to generate the second semi-circle (right) %

    C=[x(6),y(6)+r];
    thh= pi/2:0.01:3*pi/2;
    xx= C(1) + r.*cos(thh);
    yy= C(2) + r.*sin(thh);
    plot3(xx,yy,zz,'b')

    % Plot sequence for right part %
    
    plot3([x(7),x(8)],[y(7),y(8)],[z,z],'b');
    plot3([x(9),x(8)],[y(9),y(8)],[z,z],'b');
    plot3([x(9),x(10)],[y(9),y(10)],[z,z],'b');

    % Plot to generate the third semi-circle (top) %

    C=[x(10)-r,y(10)];
    thh= pi:0.01:2*pi;
    xx= C(1) + r.*cos(thh);
    yy= C(2) + r.*sin(thh);
    plot3(xx,yy,zz,'b')

    % Plot sequence for top part %

    plot3([x(11),x(12)],[y(11),y(12)],[z,z],'b');
    plot3([x(13),x(12)],[y(13),y(12)],[z,z],'b');
    plot3([x(13),x(14)],[y(13),y(14)],[z,z],'b');

    % Plot to generate the fourth semi-circle (left) %

    C=[x(14),y(14)-r];
    thh= 3*pi/2:0.01:5*pi/2;
    xx= C(1) + r.*cos(thh);
    yy= C(2) + r.*sin(thh);
    
    % Plot sequence for left part %

    plot3(xx,yy,zz,'b')
    plot3([x(15),x(16)],[y(15),y(16)],[z,z],'b');
    plot3([x(17),x(16)],[y(17),y(16)],[z,z],'b');
    
    hold off;

    % HINT: The circular trajectory start points are at position 2,6,10 & 14 of the vector A
end
    

