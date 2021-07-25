% Discrete LTI analyser
% updated:  25/07/2021
% Author: Federico Danzi
clc; clear; close all;

%--------------------------------------------------------------------------
% Input
%--------------------------------------------------------------------------

% F, G, C and D matrices of the LTI (D is null)
F = [0 1; -1 0];
G = [1 ; 0 ];
C = [1 1];
D = [0];


% Analysis of discrete LTIsystem stability, controllability and 
% observability

% Check open loop poles
eig_F = eig(F);
disp("Eigenvalues of the system:");
disp(eig_F)
if all(abs(eig_F) < 1)
    disp("The system is asymptotically stable!");
elseif max(abs(eig_F) > 1)
    disp("The system is NOT stable!");
elseif sum(abs(eig_F) == 1) == 1 
    disp("The system is simply stable");
else 
    disp("Check eigenvalues of F!");
end

% Check the controllability matix
% Reminder:
% Controllability matrix = [G, FG, F^2G, ... , F^(n-1)G]
disp("Controllability Matrix:");
P = ctrb(F,G);
disp(P);
ctrb_rank = rank(P);
if ctrb_rank == size(F,1)
    disp("The system is completely controllable");
elseif ctrb_rank < size(F,1)
    disp("The system is NOT completely controllable!");
end

% Check observability matix
% Reminder
% Observability matrix = [C; C, CA^1, ... , CA^(n-1)]
disp("Observability Matrix:");
Q = obsv(F,C);
disp(Q);
obsv_rank = rank(Q);
if obsv_rank == size(F,1)
    disp("The system is completely observable");
elseif obsv_rank < size(F,1)
    disp("The system is NOT completely observable!");
end