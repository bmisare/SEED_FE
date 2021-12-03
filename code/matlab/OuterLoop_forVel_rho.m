% Name: Brett Shearer
% Course: EENG350
%
%% Title: Outer Loop Control - Forward Velocity
%

%% Description and
%% Parameters of the system/simulation
% This script determines the error between estimated and desired rho
% values and then chooses a forward velocity associated with that
% rho-value. The motor transfer function parameters are again included with
% a I-controller system on the inner loop. An integrator and additional
% control system is added on the outer loop.

Krho = 1/200; % Gain value 
sigrho = 20; 

%% Simulink Block Diagram
% The simulink block shown linked below is opened for the entire closed
% outer loop function. The outer loop control system is a PD controller
% tuned through simulink's tuning function.

open_system('Demo1_Part5_OuterLoop_forVel');
out = sim('Demo1_Part5_OuterLoop_forVel');

%% Plot of the Results
% The output plot shows actual rho-dot values following a step input.

figure(1)
plot(out.voltage);
title('Forward Velocity - Voltage')

figure(2)
plot(out.position);
title('ForwardVelocity - Position');

%% Interpretation of Results
% The outer loop transfer function tracks desired and actual responses. The
% I-controller system on the inner loop accurately integrates/responds to 
% the error response within the closed loop. The outer closed loop incorporates
% a PD controller. We experimented with both P, D, and PD controller
% systems to arrive at a tuned PD controller to generate the most accurate
% reponse to a step input. 
