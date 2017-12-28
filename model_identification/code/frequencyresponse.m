%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%  Function script - Frequency Response                                  %
%                                                                        %
%  Autores: n� 69933, Jo�o Prata                                         %
%           n� 78486, Lu�s Rei                                           %
%           n� 78761, Jo�o Gir�o                                         %
%                                                                        %
%  Versao: 1.1                                                           %
%  Data: 12/10/2017                                                      %
%                                                                        %                                                   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


i=1;
load(['A_square_M' num2str(i)]);
load(['B_square_M' num2str(i)]);
load(['C_square_M' num2str(i)]);
load(['D_square_M' num2str(i)]);

sys =ss(A,B,C,D,0.02);
bode(sys);
figure;
pzmap(sys);