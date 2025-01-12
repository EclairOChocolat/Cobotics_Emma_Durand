function RRP3D(th1,th2,rho2)
%% A function named RRP3D used to generate the DH model of the RRP robot in 3D space %%
% INPUT(S):  th1, th2: The three revolute angles
%            rho: The prismatic link 
% OUTPUT(S): NIL- But outputs can be added if required by user
%%
        
        % Initialization of coordinates %%
        O= [0;0;0];
        A = O+ [0;0;45];
        E = O+ [0;0;15];

%% Link-1 (blue) and first revolute joint (blue) %%
    
    % Construction of link from origin to base of first revolute joint %

    plot3([O(1),A(1)],[O(2),A(2)],[O(3),A(3)],'color','b','LineWidth',2)
    hold on;
    scatter3(0,0,0,'MarkerEdgeColor','k','MarkerFaceColor','k','Linewidth',1)
    scatter3(A(1),A(2),A(3),'MarkerEdgeColor','k','MarkerFaceColor','b','Linewidth',1)

    % Plot parameters %
    
    xlim([-80,120]);
    ylim([-80,120]);
    zlim([0,120]);
    xlabel('x')
    ylabel('y')
    zlabel('z')
    grid on;
    grid minor;
    set(gcf,'color','white')
    set(gca,'FontSize',20,'FontName','Times New Roman','FontWeight','Bold');
    x0 = 25; y0 = 45; % Origin for the plot screen
    largeur =950; % Length of plot screen from origin
    hauteur =550; % Width of plot screen from origin
    set(gcf,'units','points','position',[ x0, y0, largeur, hauteur])
    
    % Construction of first revolute joint (blue) using function 'cylinder' %

    theta= 0:0.1:2*pi;
    r= 2;
    x1= r.*cos(theta);y1= r.*sin(theta);
    z1= zeros(1, length(x1));
    z1= z1+10;
    z2= z1+15;
    fill3(x1,y1,z1,'b');
    fill3(x1,y1,z2,'b');

    for j= 1:length(x1)
        plot3([x1(1,j),x1(1,j)],[y1(1,j),y1(1,j)],[z1(1,j),z2(1,j)],'b','LineWidth',2);
    end

%% Link twist and second revolute joint %%

    [rx1,~,~]= rotations(-pi/2);
    [~,~,rz1]= rotations(th1);
    
    Aux1= [0;0;5];
    Aux2= [0;0;20];
    Aux1= rz1*rx1*Aux1;
    Aux2= rz1*rx1*Aux2;
    
    B= A+Aux1;
    C= A+Aux2;

    % Plot of link A-B%

    plot3([B(1),A(1)],[B(2),A(2)],[B(3),A(3)],'color','b','LineWidth',2)

    % Generation of second revolute joint (red) using function 'cylinder' %

    cylinderplot (theta,B,C,r,rz1*rx1,'r'); % Calling sub-function cylinderplot to create cylinder %

%% Link twist and creation of prismatic joint (magenta) %%

    [~,ry1,~]= rotations(-pi/2);
    [~,~,rz2]= rotations(th2);
    D= rz1*rx1*rz2*ry1*O;
    E= rz1*rx1*rz2*ry1*E;
    D= C+D;
    E= C+E;
    E= E+rz1*rx1*rz2*ry1*[0;0;rho2];
    
    % Plot of link E-D %

    plot3([E(1),D(1)],[E(2),D(2)],[E(3),D(3)],'color','r','LineWidth',2)
    F= E+rz1*rx1*rz2*ry1*[0;0;20];
    
    % Generation of prismatic joint (magenta) %

    rects(E,F,rz1*rx1*rz2*ry1,'m') % Calling sub-function 'rects' to generate the prismatic joint %
    G= E+rz1*rx1*rz2*ry1*[0;0;35];
    
    % Plot of link F-G & the end-effector %

    plot3([F(1),G(1)],[F(2),G(2)],[F(3),G(3)],'color','m','LineWidth',2)
    scatter3(G(1),G(2),G(3),'MarkerEdgeColor','k','MarkerFaceColor','m','Linewidth',1)

    hold off;

%% SUB-FUNCTION(S)%%

%%
    function [rx,ry,rz]= rotations(th)
    %% A SUB-FUNCTION "rotations" TO APPLY ROTATIONS/LINK TWIST TO ROBOT %% 
    % INPUT(S)  : th: Rotation angle
    % OUTPUT(S) : rx,ry,rz: The x,y,z rotations matrices
    %%
        rx= [1 0 0; 0 cos(th) -sin(th);0 sin(th) cos(th)];
        ry= [cos(th) 0 -sin(th);0 1 0;-sin(th) 0 cos(th)];
        rz= [cos(th) -sin(th) 0;sin(th) cos(th) 0; 0 0 1];
    end
%%  

%%  
    function cylinderplot (theta,P1,P2,r,Rot,couleur)
    %% A SUB-FUNCTION "cylinderplot" TO CREATE REVOLUTE JOINT(S) %% 
    % INPUT(S)  : theta: For creation of top & bottom circle of revolute
    %             P1,P2: Coordinates of top & bottom of revolute
    %             r: Radius of the cylinder/revolute
    %             Rot: Rotation angles to the joint from robot architecture
    %             couleur: A string which defines the color of the joint
    % OUTPUT(S) : NIL
    %%
        x2= r.*cos(theta);y2= r.*sin(theta);
        z2= zeros(1, length(x2));
        BE(1,:)= x2;BE(2,:)= y2;BE(3,:)= z2;
        BE= Rot*BE;
        Bott(1,:) = BE(1,:) + P1(1);
        Bott(2,:) = BE(2,:) + P1(2);
        Bott(3,:) = BE(3,:) + P1(3);
        Top(1,:) = BE(1,:) + P2(1);
        Top(2,:) = BE(2,:) + P2(2);
        Top(3,:) = BE(3,:) + P2(3);
    
        for j= 1:length(Bott)
            plot3([Bott(1,j),Top(1,j)],[Bott(2,j),Top(2,j)],[Bott(3,j),Top(3,j)],couleur,'LineWidth',2);
        end
    
        fill3(Bott(1,:),Bott(2,:),Bott(3,:),couleur);
        fill3(Top(1,:),Top(2,:),Top(3,:),couleur);
    
    end
    %% 
    function rects(P1,P2,Rot,couleur)
 %% A SUB-FUNCTION "rects" TO CREATE PRISMATIC JOINT(S) %% 
    % INPUT(S)  : P1,P2: Coordinates of top & bottom of prismatic
    %             Rot: Rotation angles to the joint from robot architecture
    %             couleur: A string which defines the color of the joint
    % OUTPUT(S) : NIL
    %%
            Coord1= [P1(1);P1(2);P1(3)]+Rot*[-2;-2;0];
            Coord2= [P1(1);P1(2);P1(3)]+Rot*[2;-2;0];
            Coord3= [P1(1);P1(2);P1(3)]+Rot*[2;2;0];
            Coord4= [P1(1);P1(2);P1(3)]+Rot*[-2;2;0];

            a=[Coord1';Coord2';Coord3';Coord4'];
            Rect1x= [a(1,1),a(2,1),a(3,1),a(4,1)];
            Rect1y= [a(1,2),a(2,2),a(3,2),a(4,2)];
            Rect1z= [a(1,3),a(2,3),a(3,3),a(4,3)];
            fill3(Rect1x,Rect1y,Rect1z,couleur);


            Coord11= [P2(1);P2(2);P2(3)]+Rot*[-2;-2;0];
            Coord21= [P2(1);P2(2);P2(3)]+Rot*[2;-2;0];
            Coord31= [P2(1);P2(2);P2(3)]+Rot*[2;2;0];
            Coord41= [P2(1);P2(2);P2(3)]+Rot*[-2;2;0];

            b=[Coord11';Coord21';Coord31';Coord41'];
            Rect1x= [b(1,1),b(2,1),b(3,1),b(4,1)];
            Rect1y= [b(1,2),b(2,2),b(3,2),b(4,2)];
            Rect1z= [b(1,3),b(2,3),b(3,3),b(4,3)];
            fill3(Rect1x,Rect1y,Rect1z,couleur);

            c=a;d=b;
        
            Side1x= [c(1,1),c(4,1),d(4,1),d(1,1)];
            Side1y= [c(1,2),c(4,2),d(4,2),d(1,2)];
            Side1z= [c(1,3),c(4,3),d(4,3),d(1,3)];
        
            Side2x= [c(2,1),c(3,1),d(3,1),d(2,1)];
            Side2y= [c(2,2),c(3,2),d(3,2),d(2,2)];
            Side2z= [c(2,3),c(3,3),d(3,3),d(2,3)];
        
            fill3(Side1x,Side1y,Side1z,couleur);
            fill3(Side2x,Side2y,Side2z,couleur);
        
            Side3x= [c(1,1),c(2,1),d(2,1),d(1,1)];
            Side3y= [c(1,2),c(2,2),d(2,2),d(1,2)];
            Side3z= [c(1,3),c(2,3),d(2,3),d(1,3)];
        
            Side4x= [c(3,1),c(4,1),d(4,1),d(3,1)];
            Side4y= [c(3,2),c(4,2),d(4,2),d(3,2)];
            Side4z= [c(3,3),c(4,3),d(4,3),d(3,3)];
        
            fill3(Side3x,Side3y,Side3z,couleur);
            fill3(Side4x,Side4y,Side4z,couleur);
    end

end

