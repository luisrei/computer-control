%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%  Function script - Controller Design (Part II)                         %
%                                                                        %
%  Autores: nº 69933, João Prata                                         %
%           nº 78761, João Girão                                         %
%           nº 78486, Luís Rei                                           %
%                                                                        %
%                                                                        %
%  Versao: 1                                                             %
%                                                                        %
%  Data: 6/11/2017                                                       %
%                                                                        %                                                   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Clear everything before proceding
clear;
close all;

% Load sensor parameters Ke and Kp
load('sensor_param.mat', 'Ke', 'Kp');

% Load final matrices chosen in Part I
load('square_3231_ARMAX.mat', 'stateModelMatrices');
l = length(stateModelMatrices);
A = stateModelMatrices(1:l-1, 1:l-1);
B = stateModelMatrices(1:l-1, l);
C = stateModelMatrices(l, 1:l-1);
D = stateModelMatrices(l, l);
I = eye(length(C));

T = 10; % Simulation time
Ts = 0.02;

% Create disturbance signal
t = linspace(0, T, T/Ts); % Assumed sampling time of 20 ms
%signal = cat(2, ones(1, 2*T), zeros(1, 48*T));
signal = cat(2, ones(1, T/10*1/Ts), zeros(1, 9/10*T/Ts));
disturbance = [t; signal]';



%% Controller design & validation - Multiple R iterations

close all;

% Regulator design
Q = C'*C;
R = [0.0000001 0.1 1 10 50 100 100000000];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulate & plot controller response
figure;
p = zeros(1, length(R)+1); % Vector to contain all modeled responses' plots
subplot(2,1,1);
p(1) = plot(t, signal); % Plot the disturbance signal
hold on
for i = 1:length(R)
    [K,S,E] = dlqr(A,B,Q,R(i));

    % Simulate parameters
    sim('validate_controller');

    % Plot all responses in the same figure
    p(i+1) = plot(yout.time, yout.signals.values);    
end


% Title and Legend
title('Controller response with variation of R');
legend(p, 'Disturbance input signal',  'R close to zero',  ...
    ['R = ' num2str(R(2))], ['R = ' num2str(R(3))], ['R = ' num2str(R(4))], ...
    ['R = ' num2str(R(5))], ['R = ' num2str(R(6))],'R "close" to infinity', ...
    'Location', 'northeast');
xlabel('Time [s]');
ylabel('Volts');
axis([0 4 0 80]);

subplot(2,1,2);
p = zeros(1, length(R)); % Vector to contain all modeled responses' plots
p(1) = plot(t, signal); % Plot the disturbance signal
hold on
for i = 1:length(R)-1
    [K,S,E] = dlqr(A,B,Q,R(i));

    % Simulate parameters
    sim('validate_controller');

    % Plot all responses in the same figure
    p(i+1) = plot(yout.time, yout.signals.values);    
end


% Title and Legend
title('Controller response with variation of R');
legend(p, 'Disturbance input signal',  'R close to zero',  ...
    ['R = ' num2str(R(2))], ['R = ' num2str(R(3))], ['R = ' num2str(R(4))], ...
    ['R = ' num2str(R(5))], ['R = ' num2str(R(6))], ...
    'Location', 'southeast');
xlabel('Time [s]');
ylabel('Volts');
axis([0 4 -1 11]);




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulate & plot controller response (LOGARITMIC)
R = [0.000001 1 10 50 100 1000000]; % Redefine more sutable R's

t3 = (1:1:length(yout.signals.values));

figure
hold on
p = zeros(1, length(R)); % Vector to contain all modeled responses' plots
for i = 1:length(R)
    [K,S,E] = dlqr(A,B,Q,R(i));

    % Simulate parameters
    sim('validate_controller');

    % Plot all responses in the same figure
    p(i) = plot(t3, log10(abs(yout.signals.values))); 
end

% Title and Legend
title('Logaritmic controller response with variation of R');
legend(p, 'R close to zero', ['R = ' num2str(R(2))], ...
    ['R = ' num2str(R(3))], ['R = ' num2str(R(4))],  ...
    ['R = ' num2str(R(5))],'R "close" to infinity' ,'Location', 'southwest');
xlabel('Samples K');
axis([0 500 -50 5]);




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Loop transfer function frequency response
R = [1 10 50 100]; % Redefine more sutable R's

figure
hold on

% R = 1
[K,S,E] = dlqr(A,B,Q,R(1));
T_lqr = ss(A,B,K,0,Ts); 
bode(T_lqr);

% R = 10
[K,S,E] = dlqr(A,B,Q,R(2));
T_lqr = ss(A,B,K,0,Ts); 
bode(T_lqr);

% R = 50
[K,S,E] = dlqr(A,B,Q,R(3));
T_lqr = ss(A,B,K,0,Ts); 
bode(T_lqr);

% R = 100
[K,S,E] = dlqr(A,B,Q,R(4));
T_lqr = ss(A,B,K,0,Ts);
T_lqr_100 = T_lqr;
bode(T_lqr);

% Title and Legend
title('Loop transfer function frequency response with variation of R');
legend(['R = ' num2str(R(1))], ['R = ' num2str(R(2))], ...
    ['R = ' num2str(R(3))],['R = ' num2str(R(4))], 'Location', 'southwest');



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot the decay of the controller response in accordance to its
% eigenvalues for R = 100
t2 = 0:0.02:35;

figure
[K,S,E] = dlqr(A,B,Q,100);
for j = 1:length(E)
    if j==1
        plot(t2, abs(E(j)).^t2);
    elseif j ==2
        plot(t2, -abs(E(j)).^t2);
    elseif j == 3
        plot(t2, abs(E(j)).^t2);
    else
        plot(t2, -abs(E(j)).^t2, '--');
    end
    hold on 
end
plot(t2, (E(1).^t2 + E(2).^t2)/2, '-.');

% Title and Legend
title('Decay state-feedback controller');
legend(['Pole in = ' num2str(E(1))], ['Pole in = ' num2str(E(2))], ...
    ['Pole in = ' num2str(E(3))], ['Pole in = ' num2str(E(4))],...
    'Location', 'northeast');
xlabel('Time [s]');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot the decay of the controller response in accordance to its
% eigenvalues for R = 0.0000001
t2 = 0:0.02:35;
    
figure
hold on

[K,S,E] = dlqr(A,B,Q,0.0000001);
for j = 1:length(E)
    if j==1
        plot(t2, abs(E(j)).^t2);
    elseif j ==2
        plot(t2, -abs(E(j)).^t2);
    elseif j == 3
        plot(t2, abs(E(j)).^t2);
    else
        plot(t2, abs(E(j)).^t2);
    end
    hold on 
end
[m, i] = max(abs(E));
plot(t2, (E(i)).^t2, '-.');
    
% Title and Legend
title('Decay state-feedback controller');
legend(['Pole in = ' num2str(E(1))], ['Pole in = ' num2str(E(2))], ...
    ['Pole in = ' num2str(E(3))], ['Pole in = ' num2str(E(4))],...
    'Location', 'northeast');
xlabel('Time [s]');



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot slope of tangent line on the controller response

[K,S,E] = dlqr(A,B,Q,100);
sim('validate_controller');

t3 = (1:1:length(yout.signals.values));
[v, i] = max(abs(E));

figure
hold on
plot(t3, log10(abs(yout.signals.values)));
plot(t3, log10(max(abs(E)))*t3 + (10^v+log10(max(abs(E)))) );
xlabel('Samples K');
legend('log(y(k))', ['log(' num2str(v) ')']);
title('Logaritmic controller response and dominant pole slope for R = 100');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Closed-loop transfer function frequency response

[K,S,E] = dlqr(A,B,Q,100);

% External input
N = inv([A-eye(size(A)), B; C,0])*[zeros(size(A,1),1);1];
Nx = N(1:end-1,:);
Nu = N(end,:);
Nbar = Nu+K*Nx;

figure
hold on

% R = 1
[K,S,E] = dlqr(A,B,Q,R(1));
C_lqr = ss(A-B*K,B*Nbar,C,0,Ts);
bode(C_lqr)

% R = 10
[K,S,E] = dlqr(A,B,Q,R(2));
C_lqr = ss(A-B*K,B*Nbar,C,0,Ts);
bode(C_lqr)

% R = 50
[K,S,E] = dlqr(A,B,Q,R(3));
C_lqr = ss(A-B*K,B*Nbar,C,0,Ts);
bode(C_lqr)

% R = 100
[K,S,E] = dlqr(A,B,Q,R(4));
C_lqr = ss(A-B*K,B*Nbar,C,0,Ts);
C_lqr_100 = C_lqr;
bode(C_lqr)

% Title and Legend
title('Closed-loop transfer function frequency response with variation of R');
legend(['R = ' num2str(R(1))], ['R = ' num2str(R(2))], ...
    ['R = ' num2str(R(3))],['R = ' num2str(R(4))], 'Location', 'southwest');





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Closed-loop poles comparison with linear state feedback
[K,S,E] = dlqr(A,B,Q,100);
C_lqr = ss(A-B*K,B*Nbar,C,0,Ts);

figure
hold on

[NC_lqr,DC_lqr] = tfdata(C_lqr,'v');
subplot(1,2,1);
zplane(NC_lqr,DC_lqr);
ax = axis;
title('Closed-loop poles')

subplot(1,2,2);
zplane([],E);
axis(ax);
title('Open-loop poles')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot poles of the closed-loop for the values of the tuning parameters
figure
R = [0.0000001 0.1 1 10 50 100 100000000]; % Original considered R values
p = zeros(1, length(R)); % Vector to contain all closed-loop poles' plots
for i = 1:length(R)
    [K,S,E] = dlqr(A,B,Q,R(i));
    p(i) = plot(E, 'x');
    hold on  
end

% Draw discrete critical stability circle
zplane([],[]);
axis([-1.1 1.1 -1.1 1.1]);

% Title and Legend
title('Closed-loop poles of the state-feedback controller');
legend(p, 'R close to zero',  ['R = ' num2str(R(2))], ...
    ['R = ' num2str(R(3))], ['R = ' num2str(R(4))], ...
    ['R = ' num2str(R(5))], ['R = ' num2str(R(6))],'R "close" to infinity', ...
    'Location', 'southwest');



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Root-locus open-loop system
R = 100;

G = ss(A,B,C,D);
[NG,DG] = tfdata(G,'v');
[NG,DG] = eqtflength(NG,DG);
SRL = tf(conv(NG,fliplr(NG)),conv(DG,fliplr(DG)));

p_srl = rlocus(SRL,1/R);

figure;
subplot(1,2,2)
zplane([],p_srl)
title(['SRL poles for \rho = 1/R = ' num2str(1/R)]);

subplot(1,2,1)
rlocus(SRL)
hold on; zplane([],[]); hold off
axis([-1 2.5 -2.5 2.5])





%% Observer design & validation - Test same ratio QE/RE

%close all;

% Create disturbance signal
t = linspace(0, T, T/Ts); % Assumed sampling time of 20 ms
%signal = cat(2, ones(1, 2*T), zeros(1, 48*T));
signal = cat(2, ones(1, T/10*1/Ts), zeros(1, 9/10*T/Ts));
disturbance = [t; signal]';

% Current estimator design
RE = [0.01 0.1 1 10]; % Observation noise
G = eye(size(A));

ratio = 1000;

% Plot results
figure;
plot(t,signal);
hold on

for i = 1:length(RE)
    % Estimator design
    QE = RE(i)*ratio; % Process noise
    [M,P,Z,EE] = dlqe(A,B,C,QE,RE(i));
    
    % Estimator implementation
    PHIE = A-M*C*A;
    GAMMAE = B-M*C*B;
    CE = eye(size(PHIE));
    DE = zeros(size(CE,1),size(GAMMAE,2));

    % Simulate parameters
    sim('validate_observer');
    
    % Plot all responses in the same figure    
    plot(xout.time, xout.signals.values(:,1));
    plot(xout.time, xout.signals.values(:,2));
    plot(xout.time, xout.signals.values(:,3));
    plot(xout.time, xout.signals.values(:,4));
end

title(['Observer response with the w2/RE ratio of ' num2str(ratio)]);
legend('Disturbance input signal', ...
        ['QE = ' num2str(ratio*RE(1)) ', Rv = '  num2str(RE(1)) ''], ...
        ['QE = ' num2str(ratio*RE(2)) ', Rv = '  num2str(RE(2)) ''], ...
        ['QE = ' num2str(ratio*RE(3)) ', Rv = '  num2str(RE(3)) ''], ...
        ['QE = ' num2str(ratio*RE(4)) ', Rv = '  num2str(RE(4)) ''], ...
         'Location', 'southeast');
xlabel('Time [s]');
ylabel('Output');
axis([0 3 0 16])



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot observer's poles
p = zeros(1, length(RE));
figure
hold on

for i = 1:length(RE)
    % Estimator design
    QE = ratio*RE(i); % Process noise
    [M,P,Z,EE] = dlqe(A,B,C,QE,RE(i));

    % Estimator implementation
    PHIE = A-M*C*A;
    GAMMAE = B-M*C*B;
    CE = eye(size(PHIE));
    DE = zeros(size(CE,1),size(GAMMAE,2));
    
    % Plot all responses in the same figure
    p(i) = plot(EE, 'x');    
end

% Draw discrete critical stability circle
zplane([],[]);
axis([-1.1 1.1 -1.1 1.1]);

% Title and Legend
title(['Closed-loop poles of the observer for QE/Rv = ' num2str(ratio)]);
legend(p,['QE = ' num2str(ratio*RE(1)) ', Rv = '  num2str(RE(1)) ''], ...
        ['QE = ' num2str(ratio*RE(2)) ', Rv = '  num2str(RE(2)) ''], ...
        ['QE = ' num2str(ratio*RE(3)) ', Rv = '  num2str(RE(3)) ''], ...
        ['QE = ' num2str(ratio*RE(4)) ', Rv = '  num2str(RE(4)) ''], ...
         'Location', 'northwest');



%% Observer design & validation - Test other QE/RE ratio

%close all;

% Current estimator design
RE = [0.0001 0.001 0.01];
G = eye(size(A));
QE = 1;

p = zeros(1, length(RE)); % Vector to contain all modeled responses' plots

% Plot results
figure;
plot(t, signal); % Plot the disturbance signal
hold on

for j = 1:length(RE)
    [M,P,Z,EE] = dlqe(A,B,C,QE,RE(j));

    % Estimator implementation
    PHIE = A-M*C*A;
    GAMMAE = B-M*C*B;
    CE = eye(size(PHIE));
    DE = zeros(size(CE,1),size(GAMMAE,2));

    % Simulate parameters
    sim('validate_observer'); 

    % Plot all responses in the same figure
    plot(xout.time, xout.signals.values(:,1));
    plot(xout.time, xout.signals.values(:,2));
    %plot(xout.time, xout.signals.values(:,3));
    %plot(xout.time, xout.signals.values(:,4));
end

title('Observer response with several different QE/RE ratio');
xlabel('Time [s]');
ylabel('State');
axis([0 3 0 21])    



%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot consistency of the closed-loop poles
   
% Estimator design
QE = 1;
RE = [0.0001 0.001 0.01 0.1 1];

figure        
ax1 = subplot(1,2,1);
for j = 1:length(RE)
    [M,P,Z,EE] = dlqe(A,B,C,QE,RE(j));
    plot(ax1, EE, 'x');  
    hold on
end

% Draw discrete critical stability circle
zplane([],[]);
%axis([-1.1 1.1 -1.1 1.1]);

ax2 = subplot(1,2,2);
for j = 1:length(RE)
    [M,P,Z,EE] = dlqe(A,B,C,QE,RE(j));

    % Estimator implementation
    PHIE = A-M*C*A;

    plot(ax2, eig(PHIE), 'x');  
    hold on
end

% Draw discrete critical stability circle
zplane([],[]);
%axis([-1.1 2 -1.1 2]);

% Title and Legend
title('Closed-loop poles of the observer');
legend( '\rho_v = 10000', ...
        '\rho_v = 1000', '\rho_v = 100', ...
        '\rho_v = 10','\rho_v = 1','Location', 'northwest');



%% Join & validate control system

%close all;

% Optimal parameters
R = 100;
RE = 0.001;
QE = 1;

[K,S,E] = dlqr(A,B,Q,R);
[M,P,Z,EE] = dlqe(A,B,C,QE,RE);

PHIE = A-M*C*A;
GAMMAE = B-M*C*B;
CE = eye(size(PHIE));
DE = zeros(size(CE,1),size(GAMMAE,2));

disp('Designed regulator K = ');
K
disp('Current estimator L = ');
L = M

% External reference input
N = inv([A-eye(size(A)), B; C,0])*[zeros(size(A,1),1);1];
Nx = N(1:end-1,:);
Nu = N(end,:);
Nbar = Nu+K*Nx;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot loop transfer function
figure
hold on

T_lqg = ss([A zeros(size(A)); M*C*A PHIE-GAMMAE*K], ...
            [B; M*C*B],[zeros(size(K)) K],0,Ts);
bode(T_lqr_100, T_lqg)
legend('TF (LQR)', 'TF (LQG)')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot closed-loop system
figure
hold on

C_lqg = ss([A -B*K; M*C*A PHIE-GAMMAE*K-M*C*B*K], ...
	[B; M*C*B+GAMMAE]*Nbar,[C zeros(size(C))],0,Ts);
bode(C_lqr_100, C_lqg)
legend('C (LQR)','C (LQG)')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Closed-loop poles comparison with linear state feedback
figure 
hold on

[NC_lqg,DC_lqg] = tfdata(C_lqg,'v');
subplot(1,2,1)
zplane(NC_lqg,DC_lqg)
ax = axis;

subplot(1,2,2)
zplane([],[E; EE])
axis(ax)


%%%%%%%%%%%%%%%%
% Root-locus of the lqg

figure
hold on

[NC_lqg,DC_lqg] = tfdata(C_lqg,'v');
[NG,DG] = eqtflength(NC_lqg,DC_lqg);
SRL = tf(conv(NG,fliplr(NG)),conv(DG,fliplr(DG)));

p_srl = rlocus(SRL,1/RE);

subplot(1,2,2)
zplane([],p_srl)
title(['SRL poles for \rho = ' num2str(1/R) ', \rho_v = ' num2str(1/RE)]);
axis([-1 2.5 -3.5 3.5])

subplot(1,2,1)
rlocus(SRL)
hold on; zplane([],[]); hold off
axis([-1 2.5 -3.5 3.5])





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compare with closed-loop of the lqr
figure 
hold on

[NC_lqg,DC_lqg] = tfdata(C_lqg,'v');
subplot(1,2,1)
zplane(NC_lqg,DC_lqg)
axis([-1 2.5 -3.5 3.5])

[NC_lqr,DC_lqr] = tfdata(C_lqr,'v');
subplot(1,2,2);
zplane(NC_lqr,DC_lqr);
axis([-1 2.5 -3.5 3.5])



