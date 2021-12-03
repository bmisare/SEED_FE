% Name: Brett Shearer
% Course: EENG350
%
%% Title: Outer Loop Control - Rotational Velocity
%
%% Description and
%% Parameters of the system/simulation
% This script determines the error between estimated and desired phi
% values and then chooses a desired velocity associated with that
% phi-value. The motor transfer function parameters are again included with
% a I-controller system on the inner loop. An integrator and additional
% control system is added on the outer loop.

Kphi = 0.0075; % Gain value 
sigphi = 20; 

%% Simulink Block Diagram
% The simulink block shown linked below is opened for the entire closed
% outer loop function. The outer loop control system is a PD controller
% tuned through simulink's tuning function.

open_system('Demo1_Part5_OuterLoop_rotVel');
out = sim('Demo1_Part5_OuterLoop_rotVel');

%% Plot of the Results
% The output plot shows actual rho-dot values following a step input.

figure(1)
plot(out.voltage);
title('Forward Velocity - Voltage')

figure(2)
plot(out.position);
title('position');

%% Interpretation of Results
% The outer loop transfer function tracks desired and actual responses. The
% I-controller system on the inner loop accurately integrates/responds to 
% the error response within the closed loop. The outer closed loop incorporates
% a PD controller. We experimented with both P, D, and PD controller
% systems to arrive at a tuned PD controller to generate the most accurate
% reponse to a step input. 