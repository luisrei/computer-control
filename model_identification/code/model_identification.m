%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%  Function script - Model Identification                                %
%                                                                        %
%  Data processing, ARMAX model identification from plant data, and      %
%  conversion to state model is performed.                               %
%                                                                        %
%  Output: matrices that define the state model                          %
%                                                                        %
%  Autores: nº 69933, João Prata                                         %
%           nº 78486, Luís Rei                                           %
%           nº 78761, João Girão                                         %
%                                                                        %
%  Versao: 1.1                                                           %
%  Data: 12/10/2017                                                      %
%                                                                        %                                                   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [A, B, C, D] = model_identification(waveChoice)

    close all;

    %% Defining variables
    % Read sensor parameters.
    load('sensor_param.mat');
    Kp = sensor_param(1);
    Ke = sensor_param(2);
    
    % Read stored plant data. 
    % Cut out first 10 seconds to skip transient state
    if waveChoice == 1
    	load('scope_data_square.mat'); 
        t = ScopeData.time(501:end, :);
        sigs = ScopeData.signals.values(501:end, :);
    elseif waveChoice == 2
    	load('scope_data_PRBS.mat');
        t = ScopeData.time(501:end, :);
        sigs = ScopeData.signals.values(501:end, :);
    elseif waveChoice == 3
    	load('scope_data_saw.mat');
        t = ScopeData.time(501:end, :);
        sigs = ScopeData.signals.values(501:end, :);
        sigs(445:469, 3) = 9.9951;
    else
        'Invalid input argument.'
        return;
    end
    
    utrend = sigs(:,1); % Input signal
    thetae = sigs(:,2); % Potentiometer signal
    alphae = sigs(:,3); % Strain gauge signal

    %% Computing results
    % Computation of total bar angle
    ytrend = thetae*Kp + alphae*Ke;
    

    % Plot unfiltered data
    % Plot input signal
    figure;
    plot(t, utrend);
    xlabel('Time [s]');
    ylabel('U [V]');
    title('Input signal u');
    
    % Plot motor angle
    figure;
    plot(t,thetae);
    xlabel('Time [s]', 'fontsize', 20);
    ylabel('Y = \theta [V]', 'fontsize', 20);
    title('Motor angle', 'fontsize', 20);
    
    % Plot strain gauge signal
    figure;
    plot(t,alphae);
    xlabel('Time [s]', 'fontsize', 20);
    ylabel('\alpha [V]', 'fontsize', 20);
    title('Strain gauge signal', 'fontsize', 20);
    
    % Plot total angle
    figure;
    plot(t,ytrend);
    xlabel('Time [s]');
    ylabel('y [º]');
    title('Total bar angle');
    
    
    %% Handling results
    % Remove the integral effect from the motor shaft angle data
    af = 0.8;
    Afilt = [1 -af];
    Bfilt = (1-af)*[1 -1];
    
    % Filtering and detrending
    yf = filter(Bfilt,Afilt,ytrend);

    % Remove the tref (average value) of the input signal
    %u = detrend(utrend, 'constant'); % detrend the square wave
    u = utrend;
    
    figure
    plot(t, u);
    hold on
    plot(t,yf);
    
    %% Model identification
    % Variation of parameters to test in armax
    N = [
        3 2 3 1; 
        4 2 3 1;
        4 2 4 1;
        4 3 4 1;
        8 8 8 1;
        ]; 
    
    ytrend1 = ytrend - ytrend(1)*ones(length(ytrend),1);
    for i = 1:1:length(N(:,1))
        % Identify the model that relates motor electrical excitation with the motor
        % shaft angle
        z = [yf u];
        na = N(i,1); % AR part | sug: 4
        nb = N(i,2); % X part | sug: 2,3
        nc = N(i,3); % MA part
        nk = N(i,4); % Pure delay | sug: 1
        nn = [na nb nc nk];
        M = armax(z,nn) % th is a structure in identification toolbox format

        % Extract from th the vectors of the coefficients of the numerator and
        % denominator polynomials of the model
        [den1,num1] = polydata(M);

        % Comparing the output response of the model to the input signal with 
        % the differentiated and filtered data of the real system
        yfsim = filter(num1, den1, u);

        figure;
        plot(t, u);
        hold on;        
        plot(t, yfsim);

        % Legend
        xlabel('Time [s]', 'fontsize', 20);
        ylabel('Signal voltage [V]', 'fontsize', 20);
        title('Comparing the response of the model to the input signal', 'fontsize', 20);
        lgd = legend('Input signal', 'Model response to filtered');
        lgd.FontSize = 18;
        
        figure;
        zplane(num1,den1); % see non-miminum phase zero
        xlabel('Real part', 'fontsize', 20);
        ylabel('Imaginary part', 'fontsize', 20);
        title('zero-pole plot of the response of the model', 'fontsize', 20);

        % num and den have the coefficients of the numerator and denominator 
        % polynomials of the transfer function of the model identified for the
        % relation between the electrical excitation of the motor and the 
        % bar tip.
        [num, den] = eqtflength(num1,conv(den1,[1 -1]));

        % Conversion to the state model parametrized
        [A,B,C,D] = tf2ss(num, den);
        stateModelMatrices = [A , B; C, D];
        yhat = dlsim(A, B, C, D, u);
        yhat = dlsim(A, B, C, D, u-mean(u)); % adjust signal tendency

        % Plot predicted output signal from the model matrices
        figure;
        plot(t, yhat);
        hold on;
        plot(t, ytrend1);

        % Legend
        xlabel('Time [s]', 'fontsize', 20);
        ylabel('y [º]', 'fontsize', 20);
        title('Predicted output signal from the model matrices', 'fontsize', 20);
        legend('Predicted signal', 'Real output');
 
        %% Save matrices to utilize in posterior phases of work
        if waveChoice == 1
            save(['square_state_model_matrices_N' num2str(i) '.mat'], 'stateModelMatrices');
            save(['square_M_N' num2str(i) '.mat'],'M');
        elseif waveChoice == 2
            save(['PRBS_state_model_matrices_N' num2str(i) '.mat'], 'stateModelMatrices');
            save(['PRBS_M_N' num2str(i) '.mat'],'M');
        elseif waveChoice == 3
            save(['saw_state_model_matrices_N' num2str(i) '.mat'], 'stateModelMatrices');
            save(['saw_M_N' num2str(i) '.mat'],'M');
        end
         
    end
     
end












