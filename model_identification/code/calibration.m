%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%  Function script - Calibration step                                    %
%                                                                        %
%  Autores: nº 69633, João Prata                                         %
%           nº 78486, Luís Rei                                           %
%           nº 78761, João Girão                                         %
%                                                                        %
%  Versao: 1                                                             %
%  Data: 12/10/2017                                                      %
%                                                                        %                                                   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% y = Kp*theta_e + Ke*alpha_e

function calibration
    %% Potentiometer scalar Kp
    clear;
    close all;
    load potenciometro.mat;

    % Max and min values to calculate delta_V expected to be aprox. 10 V
    Vmax = max(tensao_pot.signals.values);
    Vmin = min(tensao_pot.signals.values);

    % Plot \theta_e. Plot auxiliary lines to calculate delta_V.
    figure;
    plot(tensao_pot.time, tensao_pot.signals.values);
    hold on;
    p1 = plot([tensao_pot.time(1),tensao_pot.time(length(tensao_pot.time))],[Vmax,Vmax], '--');
    p2 = plot([tensao_pot.time(1),tensao_pot.time(length(tensao_pot.time))],[Vmin,Vmin], '--');
    axis([0 9.6 -6 6]);

    % Legend
    title('Electrical tension yielded by the sensor of \theta over time');
    xlabel('Time [s]');
    ylabel('\theta_e [V]');
    legend([p1, p2], ['y = ' num2str(Vmax)], ['y = ' num2str(Vmin)]);

    % Calculate desired parameter
    delta_V = abs(Vmax - Vmin); % in [Volts]
    delta_theta = 360; % in [degree]

    Kp = delta_theta/delta_V; % in [degree/V]

    %% Strain gauge scalar Ke
    load extensometro.mat;

    % Plot "stair" like figure with the yielded voltages at different comb
    % positions
    figure;
    plot(tensao_ext.time(), tensao_ext.signals.values);
    axis([0 121.5 -1.5 2.5]);

    % Legend
    title('Electrical tension yielded by the sensor of \alpha over time');
    xlabel('Time [s]');
    ylabel('\alpha_e [V]');

    % Select with the cursor an initial and final point at each level to
    % determine the mean value of alpha_e in that interval
    % IF YOU CHOOSE TO DO IT THIS WAY UNCOMMENT THIS, AND COMMENT THE ARRAYS 
    % ALPHA AND ALPHA_E INSTANTIATED BELOW

    % [xi, yi] = ginput;
    % a = 1; b = 2;
    % alpha_e = zeros(1, 15);
    % for i = 1:15 
    %    alpha_e(i) = mean(tensao_ext.signals.values(round(xi(a)*1000):round(xi(b)*1000)));
    %    a = a+2;
    %    b = b+2;
    % end

    % Array with yielded alpha_e [V] at the 15 different comb positions
    alpha_e = [-1.32, -1.05, -0.79, -0.54, -0.28, -0.02, 0.16, 0.47, 0.81, 1.07, 1.31, 1.57, 1.83, 2.09, 2.34];

    % Array with real values of alpha at the 15 different comb positions
    alpha = [4.16, 3.27, 2.23, 1.34, 0.45, -0.15, -1.04, -2.23, -3.42, -4.30, -5.19, -6.22, -7.10, -8.12, -9.00];

    % Desired parameter Ke. Polynomial 1st order.
    p = polyfit(alpha_e, alpha, 1);

    % Plot the best fit (in a least-squares sense) for the data gathered
    figure;
    plot(alpha_e, alpha, '-o');
    hold on;
    plot([-1.5 2.5], [p(1)*-1.5+p(2) p(1)*2.5+p(2)], '-*'); % Regression line

    % Legend
    title(['Correlation between \alpha and \alpha_e, K_e = ' num2str(p(1))]);
    xlabel('\alpha_e [V]');
    ylabel('\alpha [degree]');
    legend('Experimental points', 'Linear regression');

    % Print K values obtained
    Kp
    Ke = p(1)

    % Save sensor parameters data
    sensor_param = [Kp, Ke];
    save('sensor_param.mat', 'sensor_param');

end

