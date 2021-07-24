clc; clear; close all


% Sampling frequency
T = 5;

% Define the A matrix of your continuous LTI system
A = [-1 1; 0 0];
B = [0; 1]

% Create symbolic varaible s
syms s t

% Create a s*I square matrix with the same size of A
sI = s * eye(size(A));

% Evaluale the sI-A matrix
sI_A = sI - A;

% Invert sI-A
inv_sI_A = inv(sI_A);

% Evaluate the inverse laplace transorm of (sI - A)^-1 or rather
% e^(At)
eAt = ilaplace(inv_sI_A)

% Multiply e^(At) with B
eAt_B = eAt * B;

% Integrate eAt_B wrt t over time between 0 and T
G = int(eAt_B, t, 0, T)

% Force the value of the sampling frequency to T and evaluate the matrix
% F and G Integratinge e^(At)*B over time between 0 and T
t = T;
F = subs(eAt)


