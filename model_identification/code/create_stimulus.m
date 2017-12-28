%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%  Function script - Generates stimulus to be applied to the system      %
%                                                                        %
%  Autores: nº 69933, João Prata                                         %
%           nº 78486, Luís Rei                                           %
%           nº 78761, João Girão                                         %
%                                                                        %
%  Input:                                                                %
%    - 1st param: choose stimulus signal type                            %
%    - 2nd param: stimulus amplitude                                     %
%    - 3rd param: stimulus frequency                                     %
%                                                                        %
%  Versao: 1.2                                                           %
%  Data: 12/10/2017                                                      %
%                                                                        %                                                   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function create_stimulus(sigChoice, amp, freq)
    %% Generate wave structure
    % Stimulus wave
    tend = 60;
    tstep = 0.001;
    stimulus = zeros(2,(tend/tstep));
    stimulus(1,:) = 0:tstep:tend-tstep;
    
    % Wave choice
    if(sigChoice == 1)
        stimulus(2,:) = amp*square(2*pi*freq*stimulus(1,:));              
    elseif(sigChoice == 2)
        stimulus(2,:) = idinput(length(stimulus), 'PRBS', [0 freq]);
    elseif(sigChoice == 3)
        stimulus(2,:) = amp*sawtooth(2 * pi * freq * stimulus(1,:));
    else
        'Invalid input argument.'
        return;
    end
    
    % Save input wave data
    save('stimulus.mat', 'stimulus');
    
    % Now we apply stimulus and record the results using 
    % 'apply_stimulus' simulink file.
   
    % Save simulink results to workspace and use it to identify the model

end


