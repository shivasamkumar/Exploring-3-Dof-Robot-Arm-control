% Initializing symbolic toolbox
syms a1 a2 l1 l2 l3 theta1 theta2 theta3 kr1 kr2 kr3 real
syms ml1 ml2 ml3 mm1 mm2 mm3 Il1 Il2 Il3 Im1 Im2 Im3 real

% Cosine and sine functions for joint angles
c1 = cos(theta1);
c12 = cos(theta1 + theta2);
c123 = cos(theta1 + theta2 + theta3);

s1 = sin(theta1);
s12 = sin(theta1 + theta2);
s123 = sin(theta1 + theta2 + theta3);

% Define the link and motor points
P1 = [l1*c1 ; l2*s1 ; 0];
P2 = [a1*c1 + l2*c12 ; a1*s1 + l2*s12 ; 0];
P3 = [a1*c1 + a2*c12 + l3*c123 ; a1*s1 + a2*s12 + l3*s123 ; 0];

% Preallocate Jacobians for link points
JPl = sym(zeros(3,3,3));
J0l = sym(zeros(3,3,3));

JPl(:,:,1) = [-l1*s1, 0, 0; l1*c1, 0, 0; 0, 0, 0];
JPl(:,:,2) = [(-a1*s1 - l2*s12), (-l2*s12), 0; (a1*c1 + l2*c12), (l2*c12), 0; 0, 0, 0];
JPl(:,:,3) = [(-a1*s1 - a2*s12 - l3*s123), (-a2*s12 - l3*s123), (-l3*s123); (a1*c1 + a2*c12 + l3*c123), (a2*c12 + l3*c123), (+l3*c123); 0, 0, 0];

J0l(:,:,1) = [0 0 0; 0 0 0; 1 0 0];
J0l(:,:,2) = [0 0 0; 0 0 0; 1 1 0];
J0l(:,:,3) = [0 0 0; 0 0 0; 1 1 1];

% Preallocate Jacobians for Motor
JPM = sym(zeros(3,3,3));
J0M = sym(zeros(3,3,3));

JPM(:,:,1) = [0 0 0; 0 0 0; 0 0 0];
JPM(:,:,2) = [-a1*s1, 0, 0; a1*c1, 0, 0; 0 0 0];
JPM(:,:,3) = [(-a1*s1 - a2*s12), (-a1*s1), 0; (a1*c1 + a2*c12), (a1*c1), 0; 0, 0, 0];

J0M(:,:,1) = [0 0 0; 0 0 0; kr1 0 0];
J0M(:,:,2) = [0 0 0; 0 0 0; 1 kr2 0];
J0M(:,:,3) = [0 0 0; 0 0 0; 1 1 kr3];

% Given rotation matrices for link and motor
R1 = [0; 0; 1];
R2 = [0; 0; 1];
R3 = [0; 0; 1];
Rm1 = [0; 0; 1];
Rm2 = [0; 0; 1];
Rm3 = [0; 0; 1];
Rs = {R1, R2, R3, Rm1, Rm2, Rm3};  % Consolidated rotation matrices

% Given masses and moments of inertia
mli = [ml1, ml2, ml3];
mmi = [mm1, mm2, mm3];
Il = [Il1, Il2, Il3];
Im = [Im1, Im2, Im3];

% Calculate the inertia matrix B
B = sym(zeros(3,3));
for i = 1:3
    B = B + mli(i) * (JPl(:,:,i)' * JPl(:,:,i)) ...
        + J0l(:,:,i)' * (Rs{i} * Il(i) * Rs{i}') * J0l(:,:,i) ...
        + mmi(i) * (JPM(:,:,i)' * JPM(:,:,i)) ...
        + J0M(:,:,i)' * (Rs{i+3} * Im(i) * Rs{i+3}') * J0M(:,:,i);
end

% Simplify each element of the inertia matrix B
B_simplified = simplify(B, 'Steps', 100);
disp('Simplified B:');
disp(B_simplified);

% Other calculations omitted for brevity...

%% Observer-Based Compensator Section
scaling_factor = 5;
desired_poles_obs = desired_poles_best * scaling_factor;

% Calculate observer gain matrix L
L = place(A_hat_numeric', C_hat_numeric', desired_poles_obs)';

% Define augmented system matrices for observer-based compensator
A_aug = [A_hat_numeric - B_hat_numeric * K_best, -B_hat_numeric * K_best;
         L * C_hat_numeric, A_hat_numeric - L * C_hat_numeric];
B_aug = [B_hat_numeric; zeros(size(B_hat_numeric))];
C_aug = [C_hat_numeric, zeros(size(C_hat_numeric))];
D_aug = zeros(size(C_hat_numeric, 1), size(B_hat_numeric, 2));

% Define the observer-based compensator system
sys_compensator_temp = ss(A_aug, B_aug, C_aug, D_aug);
dc_gain = dcgain(sys_compensator_temp);
input_scaling = inv(dc_gain);
B_aug_scaled = B_aug * input_scaling;

% Create the final compensator system
sys_compensator = ss(A_aug, B_aug_scaled, C_aug, D_aug);

% Debugging check: Ensure 'sys_compensator' is defined
if exist('sys_compensator', 'var') == 1
    disp('sys_compensator is defined successfully.');
else
    error('sys_compensator is not defined.');
end

% Time vector for simulation
t = 0:0.01:10; % Time vector
num_inputs = size(B_hat_numeric, 2);
num_outputs = size(C_hat_numeric, 1);

% Loop through each input-output pair and plot responses
figure;
for i = 1:num_inputs
    for j = 1:num_outputs
        % Linear index for subplot
        subplot_idx = (j - 1) * num_inputs + i;
        subplot(num_outputs, num_inputs, subplot_idx);
        
        % Debugging check for each step response
        try
            % Responses
            [response_sys, ~] = step(system(j, i), t);
            [response_sf, ~] = step(ssys_cl_best(j, i), t);
            [response_obs, ~] = step(sys_compensator(j, i), t);

            % Plotting
            plot(t, response_sf, 'b-', t, response_obs, 'r--', 'LineWidth', 2);
            xlabel('Time (seconds)');
            ylabel('Response');
            title(sprintf('IO Pair %d-%d', i, j));
            grid on;
        catch ME
            disp(['Error in step response for input-output pair (', num2str(i), '-', num2str(j), '): ', ME.message]);
        end
    end
end

legend('State Feedback', 'Observer-Based Compensator');
sgtitle('Comparison of Controller Responses');

% Settling Time and Overshoot
fprintf('Settling Time of SF vs OBS\n');
for i = 1:num_inputs
    for j = 1:num_outputs
        try
            [y, t] = step(ssys_cl_best(j,i));
            info_sf = stepinfo(y, t);
            [y, t] = step(sys_compensator(j,i));
            info_obs = stepinfo(y, t);
            fprintf('I/P %d - O/P %d: OverShoot = %f SF= %.2fs,  OBC= %.2fs\n', i, j, info_sf.Overshoot, info_sf.SettlingTime, info_obs.SettlingTime);
        catch ME
            disp(['Error in settling time calculation for input-output pair (', num2str(i), '-', num2str(j), '): ', ME.message]);
        end
    end
end
% Initializing symbolic toolbox
syms a1 a2 l1 l2 l3 theta1 theta2 theta3 kr1 kr2 kr3 real
syms ml1 ml2 ml3 mm1 mm2 mm3 Il1 Il2 Il3 Im1 Im2 Im3 real

% Cosine and sine functions for joint angles
c1 = cos(theta1);
c12 = cos(theta1 + theta2);
c123 = cos(theta1 + theta2 + theta3);

s1 = sin(theta1);
s12 = sin(theta1 + theta2);
s123 = sin(theta1 + theta2 + theta3);

% Define the link and motor points
P1 = [l1*c1 ; l2*s1 ; 0];
P2 = [a1*c1 + l2*c12 ; a1*s1 + l2*s12 ; 0];
P3 = [a1*c1 + a2*c12 + l3*c123 ; a1*s1 + a2*s12 + l3*s123 ; 0];

% Preallocate Jacobians for link points
JPl = sym(zeros(3,3,3));
J0l = sym(zeros(3,3,3));

JPl(:,:,1) = [-l1*s1, 0, 0; l1*c1, 0, 0; 0, 0, 0];
JPl(:,:,2) = [(-a1*s1 - l2*s12), (-l2*s12), 0; (a1*c1 + l2*c12), (l2*c12), 0; 0, 0, 0];
JPl(:,:,3) = [(-a1*s1 - a2*s12 - l3*s123), (-a2*s12 - l3*s123), (-l3*s123); (a1*c1 + a2*c12 + l3*c123), (a2*c12 + l3*c123), (+l3*c123); 0, 0, 0];

J0l(:,:,1) = [0 0 0; 0 0 0; 1 0 0];
J0l(:,:,2) = [0 0 0; 0 0 0; 1 1 0];
J0l(:,:,3) = [0 0 0; 0 0 0; 1 1 1];

% Preallocate Jacobians for Motor
JPM = sym(zeros(3,3,3));
J0M = sym(zeros(3,3,3));

JPM(:,:,1) = [0 0 0; 0 0 0; 0 0 0];
JPM(:,:,2) = [-a1*s1, 0, 0; a1*c1, 0, 0; 0 0 0];
JPM(:,:,3) = [(-a1*s1 - a2*s12), (-a1*s1), 0; (a1*c1 + a2*c12), (a1*c1), 0; 0, 0, 0];

J0M(:,:,1) = [0 0 0; 0 0 0; kr1 0 0];
J0M(:,:,2) = [0 0 0; 0 0 0; 1 kr2 0];
J0M(:,:,3) = [0 0 0; 0 0 0; 1 1 kr3];

% Given rotation matrices for link and motor
R1 = [0; 0; 1];
R2 = [0; 0; 1];
R3 = [0; 0; 1];
Rm1 = [0; 0; 1];
Rm2 = [0; 0; 1];
Rm3 = [0; 0; 1];
Rs = {R1, R2, R3, Rm1, Rm2, Rm3};  % Consolidated rotation matrices

% Given masses and moments of inertia
mli = [ml1, ml2, ml3];
mmi = [mm1, mm2, mm3];
Il = [Il1, Il2, Il3];
Im = [Im1, Im2, Im3];

% Calculate the inertia matrix B
B = sym(zeros(3,3));
for i = 1:3
    B = B + mli(i) * (JPl(:,:,i)' * JPl(:,:,i)) ...
        + J0l(:,:,i)' * (Rs{i} * Il(i) * Rs{i}') * J0l(:,:,i) ...
        + mmi(i) * (JPM(:,:,i)' * JPM(:,:,i)) ...
        + J0M(:,:,i)' * (Rs{i+3} * Im(i) * Rs{i+3}') * J0M(:,:,i);
end

% Simplify each element of the inertia matrix B
B_simplified = simplify(B, 'Steps', 100);
disp('Simplified B:');
disp(B_simplified);

% Other calculations omitted for brevity...

%% Observer-Based Compensator Section
scaling_factor = 5;
desired_poles_obs = desired_poles_best * scaling_factor;

% Calculate observer gain matrix L
L = place(A_hat_numeric', C_hat_numeric', desired_poles_obs)';

% Define augmented system matrices for observer-based compensator
A_aug = [A_hat_numeric - B_hat_numeric * K_best, -B_hat_numeric * K_best;
         L * C_hat_numeric, A_hat_numeric - L * C_hat_numeric];
B_aug = [B_hat_numeric; zeros(size(B_hat_numeric))];
C_aug = [C_hat_numeric, zeros(size(C_hat_numeric))];
D_aug = zeros(size(C_hat_numeric, 1), size(B_hat_numeric, 2));

% Define the observer-based compensator system
sys_compensator_temp = ss(A_aug, B_aug, C_aug, D_aug);
dc_gain = dcgain(sys_compensator_temp);
input_scaling = inv(dc_gain);
B_aug_scaled = B_aug * input_scaling;

% Create the final compensator system
sys_compensator = ss(A_aug, B_aug_scaled, C_aug, D_aug);

% Debugging check: Ensure 'sys_compensator' is defined
if exist('sys_compensator', 'var') == 1
    disp('sys_compensator is defined successfully.');
else
    error('sys_compensator is not defined.');
end

% Time vector for simulation
t = 0:0.01:10; % Time vector
num_inputs = size(B_hat_numeric, 2);
num_outputs = size(C_hat_numeric, 1);

% Loop through each input-output pair and plot responses
figure;
for i = 1:num_inputs
    for j = 1:num_outputs
        % Linear index for subplot
        subplot_idx = (j - 1) * num_inputs + i;
        subplot(num_outputs, num_inputs, subplot_idx);
        
        % Debugging check for each step response
        try
            % Responses
            [response_sys, ~] = step(system(j, i), t);
            [response_sf, ~] = step(ssys_cl_best(j, i), t);
            [response_obs, ~] = step(sys_compensator(j, i), t);

            % Plotting
            plot(t, response_sf, 'b-', t, response_obs, 'r--', 'LineWidth', 2);
            xlabel('Time (seconds)');
            ylabel('Response');
            title(sprintf('IO Pair %d-%d', i, j));
            grid on;
        catch ME
            disp(['Error in step response for input-output pair (', num2str(i), '-', num2str(j), '): ', ME.message]);
        end
    end
end

legend('State Feedback', 'Observer-Based Compensator');
sgtitle('Comparison of Controller Responses');

% Settling Time and Overshoot
fprintf('Settling Time of SF vs OBS\n');
for i = 1:num_inputs
    for j = 1:num_outputs
        try
            [y, t] = step(ssys_cl_best(j,i));
            info_sf = stepinfo(y, t);
            [y, t] = step(sys_compensator(j,i));
            info_obs = stepinfo(y, t);
            fprintf('I/P %d - O/P %d: OverShoot = %f SF= %.2fs,  OBC= %.2fs\n', i, j, info_sf.Overshoot, info_sf.SettlingTime, info_obs.SettlingTime);
        catch ME
            disp(['Error in settling time calculation for input-output pair (', num2str(i), '-', num2str(j), '): ', ME.message]);
        end
    end
end
