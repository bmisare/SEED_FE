%%
% Name: Brett Shearer
% Course: EENG350
%
%% Title:Inner Loop Controller Implementation 
%

%% Description:
% Demo 1 Inner Loop Controller implementation - Forward Velocity
%
% This script defines uses motor parameters obtained in a prior simulation 
% in order to develop control systems for the inner closed loop.  

%% Parameters of the system/simulation
% The motor block diagram variables are shown below. The motor transfer
% function is then tied to a Integrator control (I) in simulink to simulate
% output. The output of the simulink system was tracked against a step
% input.


Krho = 1/200; % Gain value 
sigrho = 20; 

%% Simulink Block Diagram
% This portion of the script runs the simulink model for block diagram that
% compares step response of the motor with a tuned I-controller function. 
% The simulation outputs  are captured and stored as an 'out' variable.

open_system('Demo1_InnerLoop_forVel');
out = sim('Demo1_InnerLoop_forVel');

%% Plot of the Results
% The output plot shows actual rho-dot values following a step input.

plot(out.actual);
title('Actual');

%% Interpretation of Results
% The inner loop transfer function tracks desired and actual responses. The
% I-controller system accurately integrates the error response within the
% closed loop. We had initally tried a P or PI controller, neither of which
% successfully tracked the step input. 