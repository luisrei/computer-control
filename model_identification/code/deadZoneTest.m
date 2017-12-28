%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%  Function script - Generates stimulus to determine dead zone           %
%                                                                        %
%  Autores: n� 69933, Jo�o Prata                                         %
%           n� 78486, Lu�s Rei                                           %
%           n� 78761, Jo�o Gir�o                                         %
%                                                                        %
%  Versao: 1.2                                                           %
%  Data: 25/10/2017                                                      %
%                                                                        %                                                   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
%% Positive dead zone 
i = 0:0.05:2;
tend = 60;
tstep = 0.001;
l = (tend/tstep);
stimulus = zeros(2,l);
stimulus(1,:) = 0:tstep:tend-tstep;
amp = ones(1,l);
 
step = floor(length(stimulus)/length(i));
for j = 1:length(stimulus)
    if floor(j/step)+1 > 41
        k = 41;
    else
        k = floor(j/step)+1;
    end
    stimulus(2,j) = i(k)*amp(j);
end
 
% Save input wave data
save('stimulus.mat', 'stimulus');


%% Negative dead zone
i = 0:0.05:2;
tend = 60;
tstep = 0.001;
l = (tend/tstep);
stimulus = zeros(2,l);
stimulus(1,:) = 0:tstep:tend-tstep;
amp = ones(1,l);
 
step = floor(length(stimulus)/length(i));
for j = 1:length(stimulus)
    if floor(j/step)+1 > 41
        k = 41;
    else
        k = floor(j/step)+1;
    end
    stimulus(2,j) = -i(k)*amp(j);
end
 
% Save input wave data
save('stimulus.mat', 'stimulus');