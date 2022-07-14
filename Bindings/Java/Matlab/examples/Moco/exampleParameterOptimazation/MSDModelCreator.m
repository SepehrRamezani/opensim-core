
clear all
import org.opensim.modeling.*
myLog = JavaLogSink();
Logger.addSink(myLog)
visualize=1;
plotResults=1;
Stime=0;
Etime=5;
%% Instantiate an empty model
model = Model();
model.setName('MSD_Model')
gravityVec3 = Vec3(0,-9.8,0);
model.setGravity(gravityVec3);
zeroVec3=Vec3(0);
anchorpos=[0.2,0.5,0];

blockheight=0.2;
blockmass=50;
%% Define Bodies and Joints in the Model
% Get a reference to the model's ground body
ground = model.getGround();

%% Attach Anchor geometry to the Ground
% Add offset frames so we can position the geometry
anchor1Offset = PhysicalOffsetFrame('anchor1', ground, Transform(Vec3(anchorpos(1),anchorpos(2),anchorpos(3))));
anchor1Offset.attachGeometry(Brick(Vec3(0.05,0.05,0.05)));
anchor2Offset = PhysicalOffsetFrame('anchor2', ground, Transform(Vec3(-anchorpos(1),anchorpos(2),-anchorpos(3))));
anchor2Offset.attachGeometry(Brick(Vec3(0.05,0.05,0.05)));
% Add the frames and Geometry
model.addComponent(anchor1Offset)
model.addComponent(anchor2Offset)

% Instantiate a Body with mass, inertia, and a display geometry
block = Body();
block.setName('Block');
block.setMass(blockmass);
block.setMassCenter(zeroVec3);
block.setInertia(Inertia(0.133,0.133,0.133,0,0,0));
% Add display geometry for the block
block.attachGeometry(Brick(Vec3(0.05)));


TransformAxis=SpatialTransform ();
% Instantiate a Free Joint (6 DoF) that connects the block and ground.
blockSideLength      = 0.1;
locationInParentVec3 = Vec3(0, blockSideLength/2, 0);
% cord1=Coordinate();
% cord1.setNarme('Block_tilt');
sptransform=SpatialTransform();
% coord=Coordinates();
% nameee=ArrayString('wee');

q1=sptransform.updTransformAxis(0);
q1.setName('rotation1');
q1.set_coordinates(0,'Block_tilt');
q1.set_axis(Vec3(0,0,1));
q1.set_function(LinearFunction(1,0));
cord1=Coordinate();
cord1.setRange([-pi, pi]);
cord1.setName('Block_tilt');
% Coordinate('Block_tilt',...
%      'Rotational',...
%     0,...
%     -pi(),...
%     pi());

q2=sptransform.updTransformAxis(1);
q2.setName('rotation2');
q2.set_coordinates(0,'Block_list');
q2.set_axis(Vec3(1,0,0));
q2.set_function(LinearFunction(1,0));
cord2=Coordinate();
cord2.setRange([-pi, pi]);
cord2.setName('Block_list')

q3=sptransform.updTransformAxis(2);
q3.setName('rotation3');
q3.set_coordinates(0,'Block_rotation');
q3.set_axis(Vec3(0,1,0));
q3.set_function(LinearFunction(1,0));
cord3=Coordinate();
cord3.setRange([-pi, pi]);
cord3.setName('Block_rotation')


q4=sptransform.updTransformAxis(3);
q4.setName('translation1');
q4.set_coordinates(0,'Block_tx');
q4.set_axis(Vec3(1,0,0));
q4.set_function(LinearFunction(1,0));
cord4=Coordinate();
cord4.setRange([-1, 1]);
cord4.setName('Block_tx')

q5=sptransform.updTransformAxis(4);
q5.setName('translation2');
q5.set_coordinates(0,'Block_ty');
q5.set_axis(Vec3(0,1,0));
q5.set_function(LinearFunction(1,0));
cord5=Coordinate();
cord5.setRange([-1, 1]);
cord5.setName('Block_ty')

q6=sptransform.updTransformAxis(5);
q6.setName('translation3');
q6.set_coordinates(0,'Block_tz');
q6.set_axis(Vec3(0,0,1));
q6.set_function(LinearFunction(1,0));
cord6=Coordinate();
cord6.setRange([-1, 1]);
cord6.setName('Block_tz')

% q.
%  blockToGround        = CustomJoint('blockToGround', ...
%                              ground, locationInParentVec3, zeroVec3, ...
%                              block, Vec3(0,0,0), Vec3(0,0,0),...
%                              sptransform);
% blockToGround        = PlanarJoint('blockToGround', ...
%                              ground, locationInParentVec3, zeroVec3, ...
%                             block, Vec3(0,0,0), Vec3(0,0,0));
blockToGround = PlanarJoint('PelvisToPlatform', ground, block);
Pelvis_rz = blockToGround.upd_coordinates(0); % Rotation about z
Pelvis_rz.setRange([-pi, pi]);
Pelvis_rz.setName('Block_rotation');
Pelvis_rz.setDefaultValue(0);

Pelvis_tx = blockToGround.upd_coordinates(1); % T about x
Pelvis_tx.setRange([-1, 1]);
Pelvis_tx.setName('Block_tx');
Pelvis_tx.setDefaultValue(-0.15);
Pelvis_tx.setDefaultSpeedValue(0)

Pelvis_ty = blockToGround.upd_coordinates(2); % Translation about y
Pelvis_ty.setRange([-1,1]);
Pelvis_ty.setName('Block_ty');
Pelvis_ty.setDefaultValue(blockheight);
Pelvis_ty.setDefaultSpeedValue(0)
Pelvis_ty.set_locked(true);
% Set bounds on the 6 coordinates of the Free Joint.
% angleRange 	  = [-pi, pi];
% positionRange = [-1, 1];
% for i=0:2, blockToGround.upd_coordinates(i).setRange(angleRange); end
% for i=3:5, blockToGround.upd_coordinates(i).setRange(positionRange); end
% % blockToGround.set_coordinates(0,cord1);
% % blockToGround.set_coordinates(1,cord2);
% % blockToGround.set_coordinates(2,cord3);
% % blockToGround.set_coordinates(3,cord4);
% % blockToGround.set_coordinates(4,cord5);
% % blockToGround.set_coordinates(5,cord6);




% % blockToGround.upd_coordinates(4).set_default_value (blockheight)
% Add the block body and joint to the model
model.addBody(block);
model.addJoint(blockToGround);

%% Define Muscles in the Model
% Define parameters for a Muscle
maxIsometricForce  = 4000.0;
optimalFiberLength = 0.25;
tendonSlackLength  = 0.1;
pennationAngle 	   = 10/180.*pi();
tendonstrain=0.04;
passivefiberstrain=0.55;
% Instantiate a Muscle
muscle1 = DeGrooteFregly2016Muscle();
muscle1.setName('muscle1')
muscle1.setMaxIsometricForce(maxIsometricForce)
muscle1.setOptimalFiberLength(optimalFiberLength)
muscle1.setTendonSlackLength(tendonSlackLength);
muscle1.setPennationAngleAtOptimalFiberLength(pennationAngle);
% muscle1.set_tendon_compliance_dynamics_mode('implicit');
muscle1.set_passive_fiber_strain_at_one_norm_force(passivefiberstrain);
muscle1.set_tendon_strain_at_one_norm_force(tendonstrain);
muscle1.set_max_contraction_velocity(100)

% Add Path points to muscle 1
muscle1.addNewPathPoint('muscle1-point1', ground, Vec3(-anchorpos(1),anchorpos(2),-anchorpos(3)))
muscle1.addNewPathPoint('muscle1-point2', block, Vec3(0,0.05,0))

% Instantiate a second Muscle
muscle2 = DeGrooteFregly2016Muscle();
muscle2.setName('muscle2');
muscle2.setMaxIsometricForce(maxIsometricForce)
muscle2.setOptimalFiberLength(optimalFiberLength)
muscle2.setTendonSlackLength(tendonSlackLength)
muscle2.setPennationAngleAtOptimalFiberLength(pennationAngle)
% muscle2.set_tendon_compliance_dynamics_mode('implicit');
muscle2.set_passive_fiber_strain_at_one_norm_force(passivefiberstrain);
muscle2.set_tendon_strain_at_one_norm_force(tendonstrain);
muscle2.set_max_contraction_velocity(100)

% Add Path points to  muscle 2
muscle2.addNewPathPoint('muscle2-point1', ground, Vec3(anchorpos(1),anchorpos(2),anchorpos(3)))
muscle2.addNewPathPoint('muscle2-point2', block, Vec3(0.05,0.05,0))

% Add the two muscles (as forces) to the model
model.addForce(muscle1)
% model.addForce(muscle2);

%% Define a Controller to the Model
muscleController = PrescribedController();
muscleController.setName('LinearRamp_Controller')
muscleController.setActuators(model.updActuators())

% Define Piecewise functions for the control values for the two muscles
p = inputParser();
defaultControl1 = [0.0 1 2 3 4 5.0 7;
    0.8 .2 1 0.5 0.5 0.4 0.4];
defaultControl2 = [0.0 1 2 3 4 5 7;
    0.8 .1 .5 0 1 0.2 0.2];
controlFunction1 = PiecewiseLinearFunction();
controlFunction2 = PiecewiseLinearFunction();
for i = 1:size(defaultControl1,2)
    controlFunction1.addPoint(defaultControl1(1,i), defaultControl1(2,i));
    controlFunction2.addPoint(defaultControl2(1,i), defaultControl2(2,i));
end
% Set the indiviudal muscle control functions for the prescribed muscle controller
muscleController.prescribeControlForActuator('muscle1', controlFunction1);
% muscleController.prescribeControlForActuator('muscle2', controlFunction2);

% Add the controller to the model
model.addController(muscleController);

%% Finalize connections so that sockets connectees are correctly saved
model.finalizeConnections();

%  model.setUseVisualizer(visualize)

state = model.initSystem();

%  osimSimulate(model, state, Etime);

initState = State(state);
% state.setTime(0.0);
% model.getCoordinateSet().get(0).setValue(state, 0.1*i);
manager = Manager(model);
% state.setTime(i*dTime);
manager.initialize(initState);
TimeCounter=0;
for i = 0:0.1:Etime       
    TimeCounter=TimeCounter+1;
    state = manager.integrate(i);
    Muscle1=model.getMuscles().get(0);
%     Muscle2=model.getMuscles().get(1);
    muscle1length(TimeCounter)=Muscle1.getLength(state);
%     muscle2length(TimeCounter)=Muscle2.getLength(state);
end	
MaxSLM1=min(muscle1length);
% MaxSLM2=min(muscle2length);
sTable=manager.getStatesTable();
forward=ForwardTool(); 
forward.setModel(model);
forward.setName('MSD_FD');
forward.print('FD_setup.xml')
forward.setFinalTime(Etime);
results=forward.run();

%% Print the model to a XML file (.osim)

model.print('MSD.osim');
% disp(['Model has been written to file: ' modelPath]);
% stofiles = STOFileAdapter();
% stofiles.write(sTable, 'simulation_states.sto');
model.removeController(muscleController);
model.finalizeConnections();
model.initSystem();
model.print('MSD_2.osim');
% 
Refmmodel = Model('MSD_2.osim');
statetable=TableProcessor('MSD_FD_states.sto');
controlltable=TableProcessor('MSD_FD_controls.sto');
kneeTrackingParamSolution=MSDParameterOptimazation(statetable,controlltable,model,[Stime Etime],MaxSLM1);

% Sptable=TableProcessor(sTable);
% kneeTrackingParamSolution=MSDParameterOptimazation(Sptable,Sptable,model,[Stime Etime],MaxSLM1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Logger.removeSink(myLog) 

