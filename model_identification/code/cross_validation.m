%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%  Function script - Cross validation                                    %
%                                                                        %
%  Autores: nº 69933, João Prata                                         %
%           nº 78486, Luís Rei                                           %
%           nº 78761, João Girão                                         %
%                                                                        %
%  Versao: 1.1                                                           %
%  Data: 12/10/2017                                                      %
%                                                                        %                                                   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
clc;
clear;

%Matrices Preparation

for i = 1:5
    load(['square_state_model_matrices_N' num2str(i) '.mat'], 'stateModelMatrices');
    l = length(stateModelMatrices);
    A = stateModelMatrices(1:l-1, 1:l-1);
    B = stateModelMatrices(1:l-1, l);
    C = stateModelMatrices(l, 1:l-1);
    D = stateModelMatrices(l,l);
    save(['A_square_M' num2str(i) '.mat'], 'A'); 
    save(['B_square_M' num2str(i) '.mat'], 'B'); 
    save(['C_square_M' num2str(i) '.mat'], 'C'); 
    save(['D_square_M' num2str(i) '.mat'], 'D'); 
end

for i = 1:5
    load(['PRBS_state_model_matrices_N' num2str(i) '.mat'], 'stateModelMatrices');
    l = length(stateModelMatrices);
    A = stateModelMatrices(1:l-1, 1:l-1);
    B = stateModelMatrices(1:l-1, l);
    C = stateModelMatrices(l, 1:l-1);
    D = stateModelMatrices(l,l);
    save(['A_PRBS_M' num2str(i) '.mat'], 'A'); 
    save(['B_PRBS_M' num2str(i) '.mat'], 'B'); 
    save(['C_PRBS_M' num2str(i) '.mat'], 'C'); 
    save(['D_PRBS_M' num2str(i) '.mat'], 'D'); 
end

%% Cross-Validation

close all
clear
clc

load('sensor_param.mat');
Kp = sensor_param(1);
Ke = sensor_param(2);

for model = 1:2;
    for waveChoice = 1:3;
        
        % Load desired scope data
        if waveChoice == 1
            load('scope_data_square.mat');
        elseif waveChoice == 2
            load('scope_data_PRBS.mat');
        elseif waveChoice == 3
            load('scope_data_saw.mat');
        end
        
        % Assigning variable values
        t = ScopeData.time(501:end, :);
        sigs = ScopeData.signals.values(501:end, :);
        
        if waveChoice == 3
            sigs(445:469, 3) = 9.9951; %fixing sawtooth signal
        end
        
        
        
        utrend = sigs(:,1); % Input signal
        thetae = sigs(:,2); % Potentiometer signal
        alphae = sigs(:,3); % Strain gauge signal
        
        % Computing output signal
        ytrend = thetae*Kp + alphae*Ke;
        
        % Loading every model's matrices
        for i=1:5
            
            if model == 1
                load(['A_square_M' num2str(i)]);  
                load(['B_square_M' num2str(i)]);  
                load(['C_square_M' num2str(i)]);
                load(['D_square_M' num2str(i)]);
            end
            
            if model == 2
                load(['A_PRBS_M' num2str(i)]);  
                load(['B_PRBS_M' num2str(i)]);  
                load(['C_PRBS_M' num2str(i)]);
                load(['D_PRBS_M' num2str(i)]);
            end
            
            % Creating dynamic system with 20ms sampling time
            sys =ss(A,B,C,D,0.02);
            
            % Subplotting the Pole-zero maps
            figure(waveChoice+50*model)
            hold all;
            p=i;
            if i~=2
                if i>2
                    p=i-1;
                end
                subplot(2,2,p)
                pzmap(sys);
            end
            
            subplot(2,2,p)
            pzmap(sys);
            
            % Simulating predicted output
            yhat = dlsim(A, B, C, D, (utrend));
            
            if waveChoice == 1
                yhat = dlsim(A, B, C, D, (utrend-mean(utrend))); % Making mean zero on square wave
            end
            
            
            
            % Plotting results
            figure(10*(waveChoice-1)+i+(model-1)*5)
            plot(t,ytrend-(ytrend(1)));
            hold all;
            plot(t,yhat);
            
            xlabel('Time [s]');
            ylabel('y [º]');
            title('Predicted output signal from the model matrices');
            legend('Real output', 'Model predicted signal');
            
            
            % Error metric
            %square_error(i) = mean((ytrend-yhat).^2); %mean squared error
            RMSE(i) =sqrt( mean((ytrend-(yhat+ytrend(1))).^2)); %root mean squared error
            
            
            
        end
        
        % Displaying RMSE
        RMSE
        
        [minimum, index] = min(RMSE);
        
        % Displaying best RMSE
        disp(['Minimum error is: ' num2str(minimum)]);
        disp(['Best model is: ' num2str(index)]);
    end
end
