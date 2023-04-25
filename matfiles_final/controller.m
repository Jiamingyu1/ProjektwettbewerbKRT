function [U] = controller(X)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% function [U] = controller(X)
%
% controller for the single-track model
%
% inputs: x (x position), y (y position), v (velocity), beta
% (side slip angle), psi (yaw angle), omega (yaw rate), x_dot (longitudinal
% velocity), y_dot (lateral velocity), psi_dot (yaw rate (redundant)), 
% varphi_dot (wheel rotary frequency)
%
% external inputs (from 'racetrack.mat'): t_r_x (x coordinate of right 
% racetrack boundary), t_r_y (y coordinate of right racetrack boundary),
% t_l_x (x coordinate of left racetrack boundary), t_l_y (y coordinate of
% left racetrack boundary)
%
% outputs: delta (steering angle ), G (gear 1 ... 5), F_b (braking
% force), zeta (braking force distribution), phi (gas pedal position)
%
% files requested: racetrack.mat
%
% This file is for use within the "Project Competition" of the "Concepts of
% Automatic Control" course at the University of Stuttgart, held by F.
% Allgoewer.
%
% prepared by J. M. Montenbruck, Dec. 2013 
% mailto:jan-maximilian.montenbruck@ist.uni-stuttgart.de
%
% written by *STUDENT*, *DATE*
% mailto:*MAILADDRESS*


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% INITIALIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
alpha = 8;
a_breaking = 12.2;
k_psi = 1.8;
v_c_1 = 6;
v_c_2 = 12.5;
v_c_3 = 5.3;
v_c_4 = 6.5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% state vector
x=X(1); % x position
y=X(2); % y position
v=X(3); % velocity (strictly positive)
beta=X(4); % side slip angle
psi=X(5); % yaw angle
omega=X(6); % yaw rate
x_dot=X(7); % longitudinal velocity
y_dot=X(8); % lateral velocity
psi_dot=X(9); % yaw rate (redundant)
varphi_dot=X(10); % wheel rotary frequency (strictly positive)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% racetrack
load('racetrack.mat','t_r'); % load right  boundary from *.mat file
load('racetrack.mat','t_l'); % load left boundary from *.mat file
t_r_x=t_r(:,1); % x coordinate of right racetrack boundary
t_r_y=t_r(:,2); % y coordinate of right racetrack boundary
t_l_x=t_l(:,1); % x coordinate of left racetrack boundary
t_l_y=t_l(:,2); % y coordinate of left racetrack boundary
% solution
t_x = (t_r_x+t_l_x)/2; % x coordinate of trajectory
t_y = (t_r_y+t_l_y)/2; % y coordinate of trajectory

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% STATE FEEDBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dist = (t_x-x).^2+(t_y-y).^2 ;
pos_car = find(dist == min(dist),1);
if t_x(pos_car) == -2.5 && 0 <= t_y(pos_car,1) && t_y(pos_car,1) <= 250 % 1st straight section
    %%
    pos_ref = pos_car+20; 
    r_1 = [cos(psi),sin(psi),0];
    r_2 = [t_x(pos_ref,1)-x,t_y(pos_ref,1)-y,0];
    d_psi = asin(cross(r_1,r_2)/norm(r_1)/norm(r_2));
    d_psi = d_psi(1,3);
    delta=d_psi*k_psi; % steering angle
    G=1+floor(v/alpha); %gear 1 ... 5
    if G>5
       G = 5;
    end

    zeta=0.5; %braking force distribution
    Fb=0; % braking force
    phi=0.7; % gas pedal position

    y_breaking = (v^2-v_c_1^2)/2/a_breaking; %breaking point

    if y>250-y_breaking && y<=260 && v>v_c_1 
        Fb = 15000;
        phi = 0;
    end
elseif t_x(pos_car,1) <= -2.5 && t_x(pos_car,1)>-32.5 && 240 <= t_y(pos_car,1) && t_y(pos_car,1) <= 260 % 1st curve
    %%
    pos_ref = pos_car+20; 
    r_1 = [cos(psi),sin(psi),0];
    r_2 = [t_x(pos_ref,1)-x,t_y(pos_ref,1)-y,0];
    d_psi = asin(cross(r_1,r_2)/norm(r_1)/norm(r_2));
    d_psi = d_psi(1,3);
    delta=d_psi*k_psi; % steering angle
  
    G=1+floor(v/alpha); %gear 1 ... 5
    if G>5
        G = 5;
    end

    zeta=0.5; %braking force distribution
    if  x<=-2.5 
        if v<v_c_1
            Fb = 0;
            phi = 0.2;
        else
            Fb = 1000;
            phi = 0;
        end
    end
elseif t_x(pos_car,1) == -32.5 && 250 <= t_y(pos_car,1) && t_y(pos_car,1) <= 400 % 2nd straight section
    %%
    pos_ref = pos_car+20; 
    r_1 = [cos(psi),sin(psi),0];
    r_2 = [t_x(pos_ref,1)-x,t_y(pos_ref,1)-y,0];
    d_psi = asin(cross(r_1,r_2)/norm(r_1)/norm(r_2));
    d_psi = d_psi(1,3);
    delta=d_psi*k_psi; % steering angle
  
    G=1+floor(v/alpha); %gear 1 ... 5
    if G>5
        G = 5;
    end

    zeta=0.5; %braking force distribution
    Fb=0; % braking force
    phi=0.7; % gas pedal position

    y_breaking = (v^2-v_c_2^2)/2/a_breaking; %breaking point
    
    if y>400-y_breaking && y<=410 && v>v_c_2 
        Fb = 15000;
        phi =0;
    end
elseif 400 <= t_y(pos_car,1)  % 2st curve
    %%
    pos_ref = pos_car+20; 
    r_1 = [cos(psi),sin(psi),0];
    r_2 = [t_x(pos_ref,1)-x,t_y(pos_ref,1)-y,0];
    d_psi = asin(cross(r_1,r_2)/norm(r_1)/norm(r_2));
    d_psi = d_psi(1,3);
    delta=d_psi*k_psi; % steering angle
  
    G=1+floor(v/alpha); %gear 1 ... 5
    if G>5
        G = 5;
    end

    zeta=0.5; %braking force distribution
    if  true 
        if v<v_c_2
            Fb = 0;
            phi = 0.2;
        else
            Fb = 1000;
            phi = 0;
        end
    end
elseif t_x(pos_car,1) == 22.5 && 300 <= t_y(pos_car,1) && t_y(pos_car,1) <= 400 % 3nd straight section
    %%
    pos_ref = pos_car+20; 
    r_1 = [cos(psi),sin(psi),0];
    r_2 = [t_x(pos_ref,1)-x,t_y(pos_ref,1)-y,0];
    d_psi = asin(cross(r_1,r_2)/norm(r_1)/norm(r_2));
    d_psi = d_psi(1,3);
    delta=d_psi*k_psi; % steering angle
  
    G=1+floor(v/alpha); %gear 1 ... 5
    if G>5
        G = 5;
    end

    zeta=0.5; %braking force distribution
    Fb=0; % braking force
    phi=0.7; % gas pedal position

    y_breaking = (v^2-v_c_3^2)/2/a_breaking; %breaking point
    
    if y<=305+y_breaking && y>=295 && v>v_c_3 
        Fb = 15000;
        phi = 0;
    end
elseif t_x(pos_car,1) <= 32.5 && t_x(pos_car,1)>22.5 && 290 <= t_y(pos_car,1) && t_y(pos_car,1) <= 305 % 3st curve
    %%
    pos_ref = pos_car+20; 
    r_1 = [cos(psi),sin(psi),0];
    r_2 = [t_x(pos_ref,1)-x,t_y(pos_ref,1)-y,0];
    d_psi = asin(cross(r_1,r_2)/norm(r_1)/norm(r_2));
    d_psi = d_psi(1,3);
    delta=d_psi*k_psi; % steering angle
  
    G=1+floor(v/alpha); %gear 1 ... 5
    if G>5
        G = 5;
    end

    zeta=0.5; %braking force distribution
    if  true
        if v<v_c_3
            Fb = 0;
            phi = 0.2;
        else
            Fb = 1000;
            phi = 0;
        end
    end
elseif t_x(pos_car,1) == 32.5 && 45 <= t_y(pos_car,1) && t_y(pos_car,1) <= 290 % 4nd straight section
    %%
    pos_ref = pos_car+20; 
    r_1 = [cos(psi),sin(psi),0];
    r_2 = [t_x(pos_ref,1)-x,t_y(pos_ref,1)-y,0];
    d_psi = asin(cross(r_1,r_2)/norm(r_1)/norm(r_2));
    d_psi = d_psi(1,3);
    delta=d_psi*k_psi; % steering angle
  
    G=1+floor(v/alpha); %gear 1 ... 5
    if G>5
        G = 5;
    end

    zeta=0.5; %braking force distribution
    Fb=0; % braking force
    phi=0.7; % gas pedal position

    y_breaking = (v^2-v_c_4^2)/2/a_breaking; %breaking point
    
    if y<=45+y_breaking && y>=35 && v>v_c_4 
        Fb = 15000;
        phi = 0;
    end    
else 
     %%
    pos_ref = pos_car+20; 
    if pos_ref >1950
        pos_ref = pos_ref - 1950;
    end
    r_1 = [cos(psi),sin(psi),0];
    r_2 = [t_x(pos_ref,1)-x,t_y(pos_ref,1)-y,0];
    d_psi = asin(cross(r_1,r_2)/norm(r_1)/norm(r_2));
    d_psi = d_psi(1,3);
    delta=d_psi*k_psi; % steering angle
  
    G=1+floor(v/alpha); %gear 1 ... 5
    if G>5
        G = 5;
    end

    zeta=0.5; %braking force distribution
    if  true 
        if v<v_c_4
            Fb = 0;
            phi = 0.2;
        else
            Fb = 1000;
            phi = 0;
        end
    end
end



%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% OUTPUT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
U=[delta G Fb zeta phi]; % input vector
end

