% Continuous LTI analyser
clc; clear; close all;

%--------------------------------------------------------------------------
% Input
%--------------------------------------------------------------------------

% A, B, C and D matrices of the LTI (D is null)
A = [1 2; 0 5];
B = [1 ; 1 ];
C = [1 7];
D = [ 0 ];


% Closed-loop pole assignement with system fully observable
% ---------------------------------------------------------

CLP_full = true;
% Define a number of symbolic variables ki and the matrix K
% with the same lenght of the state variable vector x 
% Note: Size of BK = Size of A
syms k1 k2;
K = [k1 k2];
% Specify the numerical values for the position of the closed-loop poles
CL_K_Poles = [ -6, -6];


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

% Check matrix stability
eig_A = eig(A);
disp("Eigenvalues of the system:")
disp(eig_A)
if all(eig_A < 0)
    disp("A is stable!");
elseif max(eig_A > 0)
    disp("A is NOT stable!");
else
    disp("Check eigenvalues of A!");    
end

% Check the controllability matix
% Reminder:
% Controllability matrix = [B, AB, A^2B, ... , A^(n-1)B]
disp("Controllability Matrix:");
Co = ctrb(A,B)
ctrb_rank = rank(Co);
if ctrb_rank == size(A,1)
    disp("The system is fully controllable");
    CTRB = true;
elseif ctrb_rank < size(A,1)
    disp("The system is NOT fully controllable!");
    CTRB = false;
end


% Check observability matix
% Reminder
% Observability matrix = [C; CA, CA^1, ... , CA^(n-1)]
disp("Observability Matrix:");
Obs = obsv(A,C)
obsv_rank = rank(Obs);
if obsv_rank == size(A,1)
    disp("The system is fully observable");
    OBSV = true;
elseif obsv_rank < size(A,1)
    disp("The system is NOT fully observable!");
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
% Closed loop pole assignment with observer
%--------------------------------------------------------------------------

% System:          _______   
%            u    |       |   x
%         --------|  Sys. |------------>
%            |    |_______|       |
%            |                    |
%            |       ___          |
%            |______| K |_________|
%                   |___|               
   
if  OBSV && CLP_full
    
    G_vect = pole_assignment_func(sys, G, CL_G_Poles, 'obsv');
    
end
    


