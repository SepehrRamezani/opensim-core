
clear all
import org.opensim.modeling.*
visualize=1;
plotResults=1;
Stime=0;
Etime=10;
%% Instantiate an empty model
model = Model();
model.setName('MSD_Model')
gravityVec3 = Vec3(0,-9.8,0);
model.setGravity(gravityVec3);
zeroVec3=Vec3(0);
anchorpos=[0.1,0.5,0.2];

blockheight=0.1;
blockmass=50;
%% Define Bodies and Joints in the Model
% Get a reference to the model's ground body
ground = model.getGround();

%% Attach Anchor geometry to the Ground
% Add offset frames so we can position the geometry
anchor1Offset = PhysicalOffsetFrame('anchor1', ground, Transform(Vec3(anchorpos(1),anchorpos(2),anchorpos(3))));
anchor1Offset.attachGeometry(Brick(Vec3(0.1,0.05,0.05)));
anchor2Offset = PhysicalOffsetFrame('anchor2', ground, Transform(Vec3(-anchorpos(1),anchorpos(2),-anchorpos(3))));
anchor2Offset.attachGeometry(Brick(Vec3(0.1,0.05,0.05)));
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


% Instantiate a Free Joint (6 DoF) that connects the block and ground.
blockSideLength      = 0.1;
locationInParentVec3 = Vec3(0, blockSideLength/2, 0);
blockToGround        = FreeJoint('blockToGround', ...
                            ground, locationInParentVec3, zeroVec3, ...
                            block, Vec3(0,0,0), Vec3(0,0,0));

% Set bounds on the 6 coordinates of the Free Joint.
angleRange 	  = [-pi, pi];
positionRange = [-1, 1];
for i=0:2, blockToGround.upd_coordinates(i).setRange(angleRange); end
for i=3:5, blockToGround.upd_coordinates(i).setRange(positionRange); end

blockToGround.upd_coordinates(4).set_default_value (blockheight)
% Add the block body and joint to the model
model.addBody(block);
model.addJoint(blockToGround);

%% Define Muscles in the Model
% Define parameters for a Muscle
maxIsometricForce  = 1000.0;
optimalFiberLength = 0.25;
tendonSlackLength  = 0.1;
pennationAngle 	   = 0./180.*pi();
tendonstrain=0.049;
passivefiberstrain=0.6;
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

% Add Path points to muscle 1
muscle1.addNewPathPoint('muscle1-point1', ground, Vec3(-anchorpos(1),anchorpos(2),-anchorpos(3)))
muscle1.addNewPathPoint('muscle1-point2', block, Vec3(0.0,0.05,-0.05))

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

% Add Path points to  muscle 2
muscle2.addNewPathPoint('muscle2-point1', ground, Vec3(anchorpos(1),anchorpos(2),anchorpos(3)))
muscle2.addNewPathPoint('muscle2-point2', block, Vec3(0.0,0.05,0.05))

% Add the two muscles (as forces) to the model
model.addForce(muscle1)
model.addForce(muscle2);

%% Define a Controller to the Model
muscleController = PrescribedController();
muscleController.setName('LinearRamp_Controller')
muscleController.setActuators(model.updActuators())

% Define Piecewise functions for the control values for the two muscles
p = inputParser();
defaultControl1 = [0.0 1 2 3 4 5.0 7;
    0.0 1 1 0.5 0.5 0.2 0.2];
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
muscleController.prescribeControlForActuator('muscle2', controlFunction2);

% Add the controller to the model
model.addController(muscleController);

%% Finalize connections so that sockets connectees are correctly saved
model.finalizeConnections();
model.setUseVisualizer(visualize)
state = model.initSystem();
% osimSimulate(model, state, Etime);

initState = State(state);
manager = Manager(model);
manager.initialize(initState);
state = manager.integrate(finalTime);
simulatedAtLeastOnce = true;

sTable = manager.getStatesTable();
stofiles = STOFileAdapter();
if ~isdir('ResultsFWD')
    mkdir ResultsFWD
end
stofiles.write(sTable, 'ResultsFWD/simulation_states.sto');

%% Use the provided plotting function to plot some results.
if plotResults
    PlotOpenSimData;
end
%% Print the model to a XML file (.osim)
modelPath = fullfile(cd, 'tug_of_war_muscles_controller.osim');
model.print(modelPath);
disp(['Model has been written to file: ' modelPath]);
