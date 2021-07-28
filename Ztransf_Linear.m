% Discrete LTI system analyzer from Continuous LTI
% updated:  24/07/2021
% Author: Federico Danzi
clc; clear; close all

%--------------------------------------------------------------------------
% Input
%--------------------------------------------------------------------------

% Sampling frequency
T = 1;

% Define the A and B matrix of your continuous LTI system
A = [-1 1; 0 0];
B = [0; 1];
% Define your B and C matrix of your continuous LTI
C = [1 0];
D = [0]; 

% System Analysis
% Note: If this function is true stability, controllability and 
% observability of the discrete LTI are checked
Analysis = true;

% Output calculation
% Note: This calculation is performed if "Output" is true only
Output = true;
% Initial condition: satate varaibles at k=0
% Note: x_zero must have the same number of rows as the columns of A 
x_zero = [0; 0];
% Size of the output vector
output_size = 1; % It is in most of the cases 1 
% Input vector
% Note: The input vector U can have any lenght, three extra zeros at the
% end are added for fun to see how the system evolves when the input is
% removed
U = [1 0 0 0 0 0 0 0 0 0];

%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% Core
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------

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


% Output of the discrete LTI after an input signal
if Output
    % Initialize x_k to x_zero
    x_k = x_zero;
    % Initialize the output vector
    y = zeros(length(U), output_size);
    % Create the time vector
    time = linspace(0, (length(U)-1)*T, length(U));
    
    %Loop over the input and estimate the output
    for i = 1 : length(U)
        % x(k+1) = F * x(k) + G * u(k)
        % y(k) = C * x(k) + D * u(k)
        x_kp1 = F * x_k + G * U(i);
        y(i) = C * x_k + D * U(i);
        % Update the state vector for the following step
        x_k = x_kp1;        
    end
    
    % Evaluate the impulse response of the continuus LTI system
    con_sys = ss(A, B, C, D);
    T_fin = (length(U)-1) * T;
    T_cont = linspace(0, T_fin, length(U));
    [y_cont,t] = lsim(con_sys, U, T_cont);
    
    figure(1)
    stairs(time, y, 'LineWidth',1.5)
    hold on
    plot(t, y_cont, 'LineWidth',1.5)
    axis([0 length(U)*T, min(y) max(y)*1.1])
    legend(["Discrete LTI", "Continuous LTI"], 'Location', "best")
    xlabel("Time [s]"); ylabel("Y [-]")
    
end