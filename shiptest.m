%-------------------------------------------------------
%  initialize the virtual model %
%-------------------------------------------------------
% create a virtual world associated 
% with sea_scene.wrl file
world = vrworld('ocean.wrl'); 
% open the virtual world
open(world);
% draw the virtual world
fig = view(world, '-internal'); 
% update the virtual scene
vrdrawnow;

%--------------------------------------------------------
% define the size of the Elevation Grid of the wave
% (dimensions taken from the virtual scene sea_scene.wrl)
%--------------------------------------------------------
xSpacing = 20;
zSpacing = 20;
xDimension = 70;
zDimension = 74;
%--------------------------------------------------------
% Sizes of the ship (pixels)
%--------------------------------------------------------
W = 121;
L = 697;
S = sqrt(L^2+W^2);
%-------------------------------------------------------
%construct a 2D mesh for the Elevation Grid of the wave
%-------------------------------------------------------
X = 0:xSpacing:(xDimension-1)*xSpacing;
Z = 0:zSpacing:(zDimension-1)*zSpacing;
Mesh_zx = meshgrid(Z,X);

%-------------------------------------------------------
% Define the joystick 
%-------------------------------------------------------
% 1 is the default number  for the joystick.
% You have to change it in case you have more 
% than 1 joystick
joy = vrjoystick(1);

%------------------------------------------------------
% In this simulation we are using a PS3 controller, 
% each button has an identity number:
% 1 for X button
% 2 for circle
% 3 for square
% 4 for triangle
% 5 for L1
% 6 for R1
% 7 for select
% 8 for start
% 9 for L3
% 10 for R3


%-------------------------------------------------------
%  Initial conditions 
%-------------------------------------------------------
% Initial position of the ship
Position_Ship_i = [0;0;0];
% Speed of the sea flow
V_enviroment = [0;0;1];
n1 = norm(V_enviroment);
% Initial velocity of the ship
Velocity_Ship = V_enviroment;
Vel = 0;
% Initial orientation of the ship
betta_i = 0;
% Initial steering angle
theta_i = 0;
% Initial time
t = 0;
% Time step
dt = 0.1;

%------------------------------------------------------
% Create a figure to check joystick and place it to 
% the right side so it doesn't covers our simulation
%------------------------------------------------------
f = figure; 
movegui(f,'east');
%------------------------------------------------------
% Create vectors to paint the axis in our Joystick Test
x = [-1 1];
y = [0 0];

%------------------------------------------------------
% Create vectors where data will be storaged
%------------------------------------------------------
Velocity = [];
Time     = [];
Position = [];
V_norm   = [];

%-------------------------------------------------------
%  while loop to run the script  
%-------------------------------------------------------

while 1       
    % Vectors with storaged data will be used later 
    % to plot and study the simulation
    Velocity = [Velocity, Velocity_Ship];
    V_norm   = [V_norm, norm(Velocity_Ship)];
    Time     = [Time, t];
    Position = [Position, Position_Ship_i]; 
  
    
    % Define the inputs of the acceleration
    circulo = button(joy, 2);
    equis   = button(joy, 1); 
    
    % Evaluate the aceleration of the ship
    ac = 5*dt*(circulo - (equis/2));
    Vel = Vel + ac;
    
    if Vel>12
        Vel=12;
    elseif Vel<-7
        Vel=-7;
    end
    
    Velocity_Ship = [ sin(betta_i)  0 cos(betta_i); 0 1 0;
        cos(betta_i) 0 -sin(betta_i)]*[Vel;0;0];
    
    % Calculate the angle between Ship and water flow speed 
    n2 = norm(Velocity_Ship);
    if n1~=0
        if n2~=0
            ang = dot(V_enviroment, Velocity_Ship)/(n1*n2);
            alpha = acos(ang);
        else 
            ang = dot(V_enviroment, [sin(betta_i), 0, cos(betta_i)]) / n1;
            alpha = acos(ang);    
        end
    end
    
    alpha1 = abs(alpha - betta_i);    
    F_current = (1/S)*abs([W*sin(alpha1) + L*cos(alpha1)]);
    
    Velocity_Ship = Velocity_Ship + V_enviroment*(F_current)';
    
    % Comppute the inertial position of the ship
    Position_Ship = Position_Ship_i + Velocity_Ship;
    
    % Change the translation property of transform_ship
    % to translate the ship
    world.Ship.translation=(Position_Ship)';
    
    %---------------------------------------------------
    a = axis(joy, [1 2]);
    a(2) = -a(2);
    a(1) = -a(1);
    
    % Define the joystick axis: 
    %    a(1) represents the values from
    %    the left and right direction buttons
    %    where
    %    a(1)~ -1 for left direction 
    %    a(1)~ 1 for right direction
    %    a(2) represents the values from
    %    the up and and down direction buttons
    %    where 
    %    a(2)~1 for up  
    %    a(2)~-1 for down
        
    %---------------------------------------------------
    % Compute the steering angle theta by
    % adding the previous angle to an incerement of 
    % +/- 0.01 based on the pressed left/right 
    % joystick button
    theta = theta_i-0.1*a(1);
    
    % Limit the value of the steering angle between 
    % -pi/4 and pi/4
    if theta>pi/4
        theta=pi/4;
    elseif theta<-pi/4
        theta=-pi/4;
    end
    
    %---------------------------------------------------
    % Joystick Test
    %---------------------------------------------------
    
    plot(a(1), a(2), 'b*');
    line(x,y,'Color','black');
    line(y,x,'Color','black');
    xlim([-1 1])
    ylim([-1 1])
    legend(['\theta = ' num2str(theta)])
    drawnow
    
    
    
    % Compute the orientation of the Ship
    betta = betta_i+0.05*theta;        
        
    % Change the rotation property of transform_ship
    % to rotate the ship
    world.Ship.rotation=[0 1 0 betta];
    
    % Update the states according to the current 
    % computed values
    Position_Ship_i = Position_Ship;
    Velocity_Ship_i = Velocity_Ship;
    betta_i = betta;
    theta_i = theta;
    t = t+dt;
           
    %compute the sinusoidal time varying height of waves
    height = 2.*sin(3.*t+Mesh_zx);
    
    % change the height vector of dimensions 
    % (xDimen-sion, zDimension)to a  vector of 
    % dimensions (xDimension*zDimension,1) as VRML  
    % expects the height property of Elevation Grid to
    % be column vec-tor.
    height_reshape = reshape(height,xDimension*zDimension,1);
    
    % change the height property of wave_height
    world.wave_height.height = height_reshape;
    
    % update the virtual scene
    vrdrawnow;
    
    % set the time of the virtual world to be t
    set(world,'Time', t);
    
    % delay the execution of the next time step for 
    % 0.05 seconds to make the execution time closer 
    % to real time
    pause(0.05);
    
    % Create a way to exit from the infinite loop
    L1 = button(joy, 5);
    R1 = button(joy, 6);
    if L1==1
        break
    end
    if R1==1
        break
    end
end

figure
plot(Time, V_norm)
title('Ship speed over time')
xlabel('t')
ylabel('speed')


tr = figure;
plot(Position(3,:),Position(1,:), 'wo','MarkerSize', 12)
title('Ship trajectory')
grid on
xlim([-287 1071])
ylim([-447 978])
set(gca,'ytick',[]);
set(gca,'xtick',[]);
set(gca,'Color','c')
%---------------------------------------------------
% If we choose to exit the loop with R1 all windows
% will be closed, this is used to test the code in
% the process of development
if R1==1
    close all
    close(fig)
end