%%
% Name: Brett Shearer
% Course: EENG350
%% Title: 
% Identification Experiments, Rotational Velocity
%
%% Description
% Demo 1, Part 3 - Identification Experiments
% In this portion of the Demo 1 Steering Control portion of the assignment
% document, we conducted a series of step experiments with the Arduino and
% MatLab to determine estimates for phi-dot and rho-dot. This code relates 
% phi-dot and rotational velocity.

%% Parameters of the system/simulation
% This portion of the experiment input input voltages related to rotational
% velocity. The values were estimated from a series of step experiments
% conducted with the given hardware. The gain (Krho) and sigma values
% (sigrho) are listed below as means of estimating the motor transfer
% functions.

clear all

Vin = 200; % input voltage variable
rotationalVelocity = 1.50; 

% Transfer function values tuned using techniques in Lecture 15 EENG307
Kphi = rotationalVelocity/Vin;
sigphi = 10;

%% Simulink Block Diagram
% This portion of the script runs the simulink model for block diagram that
% compares step response of the motor with a tuned transfer function also 
% representing the motor. The simulation outputs are captured and stored as 
% an 'out' variable. The first figure is the velocity comparison and the 
% second figure is the final position comparison.

open_system('Demo1_Identification_rotationalVel');
out = sim('Demo1_Identification_rotationalVel');

%% Plot of the Results
% Plots show both velocity and position outputs from the block diagram.
% The first figure plots the Velocity output from the simulink model, and the
% second figure plots the position output. Both graphs are very
% close to the inputs. This indicates the transfer function is sufficiently
% close/accurate to represent the open loop transfer function of the motor.
%plots the Velocity output

figure(1)
plot(out.Velocity);
title('Velocity');
legend('motor', 'transferFunc');

% plots the Position output from the simulink model
figure(2)
plot(out.Position);
title('Position');

legend('motor', 'transferFunc');

%% Interpretation of Results
% The transfer function generated from techniques listed and shown in
% EENG307 Lecture 15 generates very similar plots to the step response
% of the DC motor system's behavior. The open loop transfer function is a
% sufficiently accurate representation of the open-loop step response of
% the DC motor system. 