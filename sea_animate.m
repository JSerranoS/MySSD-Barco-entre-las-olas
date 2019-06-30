%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  initialize the virtual model %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% create a virtual world associated with sea_scene.wrl file
world = vrworld('sea_scene.wrl'); 
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
xDimension=31;
zDimension=20;
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%construct a 2D mesh for the Elevation Grid of the wave
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
X=0:xSpacing:(xDimension-1).*xSpacing;
Z=0:zSpacing:(zDimension-1).*zSpacing;
Mesh_zx=meshgrid(Z,X);
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% for loop to run the script  %%%
%  across the entire desired time%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for t=0:0.1:40
    %change the translation property of Ship 
    world.Ship.translation=[-100+t*10 0 0];  
    
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
end
