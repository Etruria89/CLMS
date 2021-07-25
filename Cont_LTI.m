% Continuous LTI analyser
% updated:  24/07/2021
% Author: Federico Danzi
clc; clear; close all;

%--------------------------------------------------------------------------
% Input
%--------------------------------------------------------------------------

% A, B, C and D matrices of the LTI (D is null)
A = [-2 1; 0 -1];
B = [0 ; 1];
C = [1 0];
D = [0];

% Trigger for the check of the asimptotical stability of the system if not
% controllable using a u = Kx lineaer controller
CL_asymp_stable_check = true;

% Closed-loop pole assignement with system fully observable
% ---------------------------------------------------------

CLP_full = true;
% Define a number of symbolic variables ki and the matrix K
% with the same lenght of the state variable vector x 
% Note: Size of BK = Size of A
syms k1 k2;
K = [k1 k2];
% Specify the numerical values for the position of the closed-loop poles
CL_K_Poles = [ -1 + 1i , -1 - 1i]; % Note: 1i is the immaginary number


% Closed-loop pole assignement with system partially observable
% -------------------------------------------------------------

% Define a number of symbolic variables gi and the matrix G 
% with the same lenght of the input vector u%  
% Note: Size of GC = Size of A
CLP_obs = true;
syms g1 g2;
G = [g1; g2];
% Specify the numerical values for the position of the closed-loop poles
% for a partially accessible system
CL_G_Poles = [ -3; -3];

%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% Core
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------

% Create system matrices
sys = ss(A,B,C, D);

% Check open loop system stability
eig_A = eig(A);
disp("Eigenvalues of the system:")
disp(eig_A)
if all(real(eig_A) < 0)
    disp("The system is asymptotically stable!");
elseif max(real(eig_A) > 0)
    disp("The system is NOT stable!");
else
    disp("Check eigenvalues of A!");    
end

% Check the controllability matix
% Reminder:
% Controllability matrix = [B, AB, A^2B, ... , A^(n-1)B]
disp("Controllability Matrix:");
Co = ctrb(A,B);
disp(Co);
ctrb_rank = rank(Co);
if ctrb_rank == size(A,1)
    disp("The system is completely controllable");
    CTRB = true;
elseif ctrb_rank < size(A,1)
    disp("The system is NOT completely controllable!");
    CTRB = false;
    if CL_asymp_stable_check
        asymp_stability_func(sys, K);
    end
end

% Check observability matix
% Reminder
% Observability matrix = [C; CA, CA^1, ... , CA^(n-1)]
disp("Observability Matrix:");
Obs = obsv(A,C);
disp(Obs);
obsv_rank = rank(Obs);
if obsv_rank == size(A,1)
    disp("The system is completely observable");
    OBSV = true;
elseif obsv_rank < size(A,1)
    disp("The system is NOT completely observable!");
    OBSV = false;
end

%--------------------------------------------------------------------------
% Closed loop pole assignment for system fully accessible
%--------------------------------------------------------------------------

% System:          _______   
%            u    |       |   x
%         --------|  Sys. |------------>
%            |    |_______|       |
%            |                    |
%            |       ___          |
%            |______| K |_________|
%                   |___|               



if CTRB && CLP_full
    
    K_vect = pole_assignment_func(sys, K, CL_K_Poles, 'ctrb');
    
end    
    
%--------------------------------------------------------------------------
% Closed loop pole assignment with identity observer
%--------------------------------------------------------------------------

% System:           __________               _______
%             u    |          |     x       |       |     y
%    --------------| dx=Ax+Bu |------------>|   C   |------------>
%  /|\        |    |_y =Cx+Du |             |_______|   |  
%   |         |    |__________|                         |
%   |         |_______________________________          |
%   |                                        |          |
%   |                           ____________\|/____     |
%   |           z             |                   |<----'
%   K ------------------------| dz=(A-GC)z+Gy+Bu  |
%                             |___________________|
%
%                   
   
if  OBSV && CLP_full
    
    G_vect = pole_assignment_func(sys, G, CL_G_Poles, 'obsv');
    
end
    


