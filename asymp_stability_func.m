function [Out_vals] = asymp_stability_func(system, symbols)
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% Asymptotical stability function:
% updated:  25/07/2021
% Author: Federico Danzi
%
% The asymptotical stability checker:
% Given a NOT controllable system checks if it is possible to get a
% asymptotically stable stable via a linear closed-loop controller as
%                   u = Kx
% Input:  
%   system:     LTI system structure
%   symbols:    simbolic array containing the unknown varaibles
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------

    % Decompose the LTI system
    A = system.A;
    B = system.B;
    C = system.C;
    D = system.D;

    % define a lambda varaible for the estimation of the closed loop
    % eigenvalues
    syms s    
    
    % Closed loop system dynamic 
    % dx = (A + BK)x if the system is fully accessible
    % whose egeivalues are the roots of
    % det(sI - (A + BK)) 
    
    K = symbols; 
    ABK = A + B*K;
    M = ABK;
    
    % Calculate sI - (A + BK)
    sI = s * eye(size(A,1));
    sI_M = (sI - M);
    
    % Evaluate its determinant and put it in a fancy way
    car_eq_tmp = det(sI_M);    
    car_eq = collect(car_eq_tmp, s);
    disp("Controlled problem characteristic equation:")
    disp(car_eq);
   
    % Compute the closed loop controller parameters as functions of K
    CL_sol = solve([car_eq == 0], s);
    
    % Solve the inequality to ensure that all the eigenvalues of the system
    % dynamic are negative
    CL_solution = solve([CL_sol < zeros(length(K),1)], K,'ReturnConditions',true);
    
    % Results with matlab mapping between varaibles  
    disp("Controller variables:"); disp(K);
    disp("Matlab variables Mapping"); disp(CL_solution.parameters);
    disp("Conditions"); disp(CL_solution.conditions);   

end
