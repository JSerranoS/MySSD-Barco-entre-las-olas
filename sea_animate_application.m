%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  initialize the virtual model %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% create a virtual world associated with sea_scene2.wrl file
world = vrworld('sea_scene2.wrl'); 
% open the virtual world
open(world);
% draw the virtual world
fig = view(world, '-internal'); 
%update the virtual scene
vrdrawnow;
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%define the size of the Elevation Grid of the wave(dimensions taken from the virtual scene sea_scene.wrl)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
xSpacing=20;
zSpacing=20;
xDimension=70;
zDimension=50;
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%construct a 2D mesh for the Elevation Grid of the wave
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
X=0:xSpacing:(xDimension-1).*xSpacing;
Z=0:zSpacing:(zDimension-1).*zSpacing;
Mesh_zx=meshgrid(Z,X);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%load the fuzzy controller
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load Fuzzy_heading_controller.mat

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%gains for the fuzzy controller 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ke=0.5;
Kedot=2;
Kalpha=2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%initial heading angle psi of the ship
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
psi=0; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%initial time derivative of the heading angle psi of the ship
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
psi_dot=0; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%desired heading angle of the ship
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
psi_d=pi/3; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%initial error for the controller
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
e_previous=0; 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%counter index
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
index=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% psi and alpha vectors will be stored for plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
PSI=[];
ALPHA=[];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% for loop to run the script  %%%
%  across the entire desired time%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for t=0:0.1:40
    %heading error
    e_current=psi_d-psi;
    %time derivative of heading error
    e_dot=(e_current-e_previous)/0.1;
    
    %compute the fuzzy logic controller output
    alpha=Kalpha*evalfis([e_current*Ke   e_dot*Kedot],Fuzzy_Heading);
    
    %bound the angle of the rudder alpha between [-pi/5,pi/5]
    if alpha>pi/5
        alpha=pi/5;
    elseif alpha<-pi/5
        alpha=-pi/5;
    end
    
    %integrate equation (7.3) to get the time derivative of psi
    psi_dot=psi_dot+(sin(alpha)+0.5*sin(3*t))/10*0.1;
    
    %integrate psi_dot to get psi
    psi=psi+psi_dot*0.1;
    
    %change the rotation property of Ship 
    world.Patrol_boat.rotation=[0 1 0 psi];  
    
    %compute the sinusoidal time varying height of waves
    height=2.*sin(3.*t+Mesh_zx);
    
    %change the height vector of dimensions (xDimen-sion,zDimension)
    %to a  vector of dimensions (xDimension*zDimension,1) as VRML 
    % expects the height property of Elevation Grid to be column vec-tor.
    height_reshape=reshape(height,xDimension*zDimension,1);
    
    %change the height property of wave_height
    world.wave_height.height=height_reshape;
    
    %update the virtual scene
    vrdrawnow;
    
    %set the time of the virtual world to be t
    set(world,'Time', t);
    
    %delay the execution of the next time step for 0.05 seconds to 
    %make the execution time closer to real time
    pause(0.05);
    e_previous=e_current;
    index=index+1;
    PSI(index)=psi;
    ALPHA(index)=alpha;
end
figure
plot(0:0.1:40,PSI)
hold on
plot(0:0.1:40,psi_d.*ones(1,length(PSI)),'r')
plot(0:0.1:40,ALPHA,'g')
legend('\psi','\psi target','\alpha')
ylim([-0.4 1.5])