function [kneeTrackingParamSolution]=MSDParameterOptimazation(StateTrackTable,ControlTrackTable,model,time,MaxSLM1)

import org.opensim.modeling.*
Stime=time(1);
Etime=time(2);
StateWeight = 10.0/model.getNumCoordinates();
ControlWight=1;
Solverinterval=10;
track = MocoTrack();
track.setName('MSDstateTracking');
stateTrackingWeight = 1;

% tableProcessor2=TableProcessor(StateTrackTable);

modelProcessor = ModelProcessor(model);
track.setModel(modelProcessor);
track.setStatesReference(StateTrackTable);
track.set_states_global_tracking_weight(stateTrackingWeight);
track.set_allow_unused_references(true);
track.set_track_reference_position_derivatives(true);
track.set_apply_tracked_states_to_guess(true);
track.set_initial_time(Stime);
track.set_final_time(Etime);
track.set_minimize_control_effort(false);
stateWeights = MocoWeightSet();
% track.setReferenceLabel

stateWeights.cloneAndAppend(MocoWeight('/jointset/blockToGround/Block_tilt/value',StateWeight));
stateWeights.cloneAndAppend(MocoWeight('/jointset/blockToGround/Block_list/value',StateWeight));
stateWeights.cloneAndAppend(MocoWeight('/jointset/blockToGround/Block_rotation/value',StateWeight));
stateWeights.cloneAndAppend(MocoWeight('/jointset/blockToGround/Block_tx/value',StateWeight));
stateWeights.cloneAndAppend(MocoWeight('/jointset/blockToGround/Block_ty/value',StateWeight));
stateWeights.cloneAndAppend(MocoWeight('/jointset/blockToGround/Block_tz/value',StateWeight));


% stateWeights.cloneAndAppend(MocoWeight('/jointset/blockToGround/.*/speed',StateWeight*0.1));
track.set_states_weight_set(stateWeights);
study = track.initialize();
problem = study.updProblem();

% param = MocoParameter('tendon_slack_muscle1','/forceset/muscle1','tendon_slack_length', MocoBounds(0.2*MaxSLM1,MaxSLM1));
param1 = MocoParameter('max_iso_muscle1','/forceset/muscle1','max_isometric_force', MocoBounds(300,5000));
param2 = MocoParameter('optimal_fiber_muscle1','/forceset/muscle1','optimal_fiber_length', MocoBounds(.1,0.4));
param3 = MocoParameter('pennation_angle_muscle1','/forceset/muscle1','pennation_angle_at_optimal', MocoBounds(0,1.5));
param4 = MocoParameter('passive_fiber_muscle1','/forceset/muscle1','passive_fiber_strain_at_one_norm_force', MocoBounds(0.1,0.9));
param5 = MocoParameter('tendon_strain_muscle1','/forceset/muscle1','tendon_strain_at_one_norm_force', MocoBounds(0.02,0.06));
problem.addParameter(param1)
% problem.addParameter(param1)
% problem.addParameter(param2)
% problem.addParameter(param3)
% problem.addParameter(param4)
% problem.addParameter(param5)

% for i=0:1:osimmodel.getMuscles().getSize()
%     Musname = osimmodel.updMuscles().get(i).getName();
%     MusPath=append('/forceset/',char(Musname));
%     MaxTendonSlack=MinMTCLength(i+1);
%     param = MocoParameter(append('tendon_slack_',char(Musname)),MusPath,'tendon_slack_length', MocoBounds(0.2*MaxTendonSlack,MaxTendonSlack));
%     param1= MocoParameter(append('passive_fiber_',char(Musname)),MusPath,'passive_fiber_strain_at_one_norm_force', MocoBounds(0.2,0.8));
%     if sum(strcmp(char(Musname), ComplianacMusclename))
%         param2= MocoParameter(append('tendon_strain_',char(Musname)),MusPath,'tendon_strain_at_one_norm_force', MocoBounds(0.01,0.1));
%         problem.addParameter(param2);
%     else
%         problem.addParameter(param1);
%     end
%         problem.addParameter(param);
% end
ContTracking = MocoControlTrackingGoal('MuscleControlTracking');
% ContTracking.setWeight(w);
% controlsRef = TableProcessor('Kneeflexion_solution.sto');
ContTracking.setReference(ControlTrackTable);
ContTracking.setReferenceLabel('/forceset/muscle1','muscle1');
% ContTracking.setReferenceLabel('/forceset/muscle2','muscle2');
ContTracking.setWeightForControl('/forceset/muscle1',ControlWight);
% ContTracking.setWeightForControl('/forceset/muscle2',ControlWight);
problem.addGoal(ContTracking)
ContTracking.setWeightForControl('/forceset/muscle1',ControlWight);
%  ContTracking.setWeightForControl('/forceset/muscle2',ControlWight);
model = modelProcessor.process();
model.initSystem();
%% add cost function
% effort = MocoControlGoal.safeDownCast(problem.updGoal('control_effort'));
% effort.setWeight(0.001);
% effort.setExponent(2);
% effort.setDivideByDisplacement(false);
%% define parameter
% problem.setStateInfo('/jointset/blockToGround/Block_tx/value',[-1, 1]);
% problem.setStateInfo('/jointset/blockToGround/Block_ty/value',[-1, 1]);
% problem.setStateInfo('/jointset/blockToGround/Block_tz/value',[-1, 1]);
% problem.setStateInfo('/jointset/blockToGround/Block_tilt/value',[-pi(),pi()]);
% problem.setStateInfo('/jointset/blockToGround/Block_rotation/value',[-pi(), pi()]);
% problem.setStateInfo('/jointset/blockToGround/Block_list/value',[-pi(), pi()]);

%% optimal_fiber_length
solver = study.initCasADiSolver();
%% define solver
solver.set_num_mesh_intervals(Solverinterval);
solver.set_verbosity(2);
solver.set_optim_solver('ipopt');
solver.set_optim_convergence_tolerance(1e-2);
solver.set_optim_constraint_tolerance(1e0);
solver.set_optim_max_iterations(4000);
solver.set_implicit_auxiliary_derivatives_weight(0.00001)
solver.set_parameters_require_initsystem(false);
solver.resetProblem(problem);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
kneeTrackingParamSolution = study.solve();

end

%% Use the provided plotting function to plot some results.
% if plotResults
%     PlotOpenSimData;
% end