% Performance Evaulation of the Manipulator Class Methods


%% Import the CoreRobotics Library
addpath ../
import CoreRobotics.*

%% Settings
N = 10;     % number of links
M = 100;    % test points per 1:N

%% Set up manipulator
Robot = CoreRobotics.Manipulator;
freevar = {'pos_x','pos_y','pos_z','ang_a','ang_b','ang_g'};

%% Run test
rt1 = zeros(M,N);
rt2 = zeros(M,N);
for k = 1:N
    
    fprintf('Eval for N = %i joints\n',k);
    
    T = CoreRobotics.Frame3D();
    T.pos_x = rand;
    T.pos_y = rand;
    T.pos_z = rand;
    T.ang_a = rand;
    T.ang_b = rand;
    T.ang_g = rand;
    T.free_variables = freevar{randi(6)};
    Link = CoreRobotics.RigidBody('frame',T);
    Robot = Robot.AddLink(Link);
    
    % Forward Kinematics method
    for l = 1:M
        u = rand(k,1);
        tic
        FK = Robot.GetForwardKinematics(u);
        rt1(l,k) = toc;
    end
    
    % Jacobian method
    for l = 1:M
        tic
        J = Robot.GetJacobian(1);
        rt2(l,k) = toc;
    end
end

%% plot results
figure(1),clf
subplot(211)
hold on; grid on;
plot(1:N,rt1,'.k')
plot(1:N,mean(rt1),'-sr')
title('Forward Kinematics Method')
ylabel('Time [s]')
xlabel('N links')
subplot(212)
hold on; grid on;
plot(1:N,rt2,'.k')
plot(1:N,mean(rt2),'-sr')
title('Jacobian Method')
ylabel('Time [s]')
xlabel('N links')
