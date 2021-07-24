function [Out_vals] = pole_assignment_func(system, symbols, target_poles, type)
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% Pole assignment function:
% updated:  24/07/2021
% Author: Federico Danzi
%
% The pole assignment function:
% Determines the numerical values to assign for a closed loop controller
% The function may be used for both a fully accessible and a partially
% observed system via the type varaibles.
% Input:  
%   system:     LTI system structure
%   symbols:    simbolic array containing the unknown varaibles
%   target_val: desired target poles
%   type:       type string to specify if the system is fully accessible 
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
    % dz = (A - GC)z + Gy + Bu  if the system is partially accessible
    % whose egeivalues are the roots of
    % det(sI - (A + BK)) 
    % det(sI - (A - GC)) 
    
    if type == 'ctrb'
        
        % Extract the K matrix
        K = symbols;
        % Estimate A + BK matrix
        ABK = A + B*K;
        % Give ABK a common name M
        M = ABK;
    elseif type == 'obsv'
        % Extract the G matrix
        G = symbols;
        AGC = A - G*C
        M = AGC;
    end
    
    % Calculate sI - (A + BK)
    sI = s * eye(size(A,1));
    sI_M = (sI - M)
    
    % Evaluate its determinant and put it in a fancy way
    car_eq_tmp = det(sI_M);
    car_eq = collect(car_eq_tmp, s)
    % Extract coefficients of the charcateristic equation
    car_coeff = coeffs(car_eq, s)
    
    % Compute the characteristic equation for the desired positions
    % of the control loop poles    
    desired_char_eq = 1;
    for i = 1 : length(target_poles)
        desired_char_eq = desired_char_eq * (s - target_poles(i));        
    end
    desired_coeff = coeffs(desired_char_eq, s)
    
    % Compute the closed loop controller parameters
    CL_sol = solve([car_coeff(1:end-1) == desired_coeff(1:end-1)], symbols);
    CL_sol_vals = structfun(@mean,CL_sol);
    
    % Print solution
    for i = 1 : length(CL_sol_vals)
        sym_str = sym2str(symbols(i));
        sym_result = strcat(sym_str, " = ", sym2str(CL_sol_vals(i)));   
        disp(sym_result)
    end

    Out_vals = CL_sol_vals;


end
