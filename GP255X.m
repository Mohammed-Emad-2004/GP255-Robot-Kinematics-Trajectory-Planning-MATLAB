t_init = 0:0.01:2;
q_init = zeros(length(t_init), 1);
q1 = [t_init', q_init];
q2 = [t_init', q_init];
q3 = [t_init', q_init];
q4 = [t_init', q_init];
q5 = [t_init', q_init];
q6 = [t_init', q_init];

robot = importrobot('GP255');
show(robot);
disp(robot);

config = homeConfiguration(robot);
tform = getTransform(robot, config, 'Body6', 'Base');
pos = tform(1:3, 4);
disp('End-effector position at home configuration (meters):');
disp(pos);

config(1).JointPosition = deg2rad(30);
config(2).JointPosition = deg2rad(-45);
config(3).JointPosition = deg2rad(60);
tform2 = getTransform(robot, config, 'Body6', 'Base');
pos2 = tform2(1:3, 4);
disp('End-effector position at [30, -45, 60, 0, 0, 0] deg (meters):');
disp(pos2);

ik = inverseKinematics('RigidBodyTree', robot);
weights = [1 1 1 1 1 1];
initialGuess = homeConfiguration(robot);
targetPos = [0.5, 0.3, 1.2];
targetTform = trvec2tform(targetPos);
[configSol, solInfo] = ik('Body6', targetTform, weights, initialGuess);

disp('Solution joint angles (degrees):');
for i = 1:6
    disp(['Joint ', num2str(i), ': ', num2str(rad2deg(configSol(i).JointPosition)), ' deg']);
end

tformCheck = getTransform(robot, configSol, 'Body6', 'Base');
posCheck = tformCheck(1:3, 4);
disp('Verified end-effector position:');
disp(posCheck);

t = 0:0.01:2;
T = 2;
r = 0.2;
x0 = 0.5; 
y0 = 0.3; 
z0 = 1.2;

theta_traj = 2*pi*(t/T - sin(2*pi*t/T)/(2*pi));
x_d = x0 + r*cos(theta_traj);
y_d = y0 + r*sin(theta_traj);
z_d = z0 * ones(size(t));

q_traj = zeros(6, length(t));
for i = 1:length(t)
    targetTform = trvec2tform([x_d(i), y_d(i), z_d(i)]);
    [configSol, ~] = ik('Body6', targetTform, weights, initialGuess);
    for j = 1:6
        q_traj(j, i) = configSol(j).JointPosition;
    end
    initialGuess = configSol;
end

dq_traj = gradient(q_traj, 0.01);

figure('Name', 'Joint Positions');
for i = 1:6
    subplot(2, 3, i);
    plot(t, rad2deg(q_traj(i, :)), 'LineWidth', 1.5);
    title(['Joint ', num2str(i), ' Position']);
    xlabel('Time (s)'); 
    ylabel('Angle (deg)');
    grid on;
end

figure('Name', 'Joint Velocities');
for i = 1:6
    subplot(2, 3, i);
    plot(t, rad2deg(dq_traj(i, :)), 'LineWidth', 1.5);
    title(['Joint ', num2str(i), ' Velocity']);
    xlabel('Time (s)'); 
    ylabel('Velocity (deg/s)');
    grid on;
end

ddq_traj = gradient(dq_traj, 0.01);
disp('ddq_traj created successfully');

robot.DataFormat = 'column';

tau = zeros(6, length(t));

for i = 1:length(t)
    tau(:, i) = inverseDynamics(robot, q_traj(:,i), dq_traj(:,i), ddq_traj(:,i));
end

peak_torque = max(abs(tau), [], 2);
avg_torque = mean(abs(tau), 2);

disp('Peak Torque per Joint (Nm):');
for i = 1:6
    disp(['Joint ', num2str(i), ': Peak = ', num2str(peak_torque(i)), ' Nm, Avg = ', num2str(avg_torque(i)), ' Nm']);
end

