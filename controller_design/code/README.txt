%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                            %
%  Parte II - CComp Project                  %
%                                            %
%  Autores: nº 69633, João Prata             %
%           nº 78761, João Girão             %
%           nº 78486, Luís Rei               %
%                                            %                                                   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Some graphs in the main matlab scripts are not properly titled to
avoid repetition of the legend in the final report delivered. In case
there is any doubt regarding the figures presented please contact the
authors.

%%
NOTES (observer):
- controller_design_1 assumes GAMMA1 = I, QE = qe*I;
- controller_design_2 assumes GAMMA1 = GAMMA = B, Qw = 1, Rv = 1/\rho;

Both programs design the controller from the state model matrices 
(square_3231_ARMAX.mat) found in part I of the project.
  
Sensor parameters calibrated in part I are in sensor_param.mat.

Eight simulinks are sent:
- 'sanity_check' for obvious reasons;
- 'validate_controller' to determine R and the loop-gain K;
- 'validate_observer' to determine Qw/Rv and the loop-gain L;
- 'joint_control_system' to simulate the joint system/controller 
  dynamics;
- 'plant_tests' to apply our design in the real plant;
- 'plant_tests_dz' to compensate the dead zone saturation with an
  integrator with reset level;
- 'plant_tests_dz_noreset' to compensate the dead zone saturation with 
  an integrator without reset level;
- 'plant_tests_dz_robust' to compensate the dead zone saturation with reset
  selector;

All other .mat files are structures representing the signal variation of 
the plant when we applied an input signal to a system modelled and
designed with certain parameters expressed in the title of said files.

'tau-*' structures represent the system's response when changing
the constant parameter present in the error integrator.