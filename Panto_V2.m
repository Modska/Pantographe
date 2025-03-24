mdl = "FiveBar";
load_system(mdl);
fk = simscape.multibody.KinematicsSolver(mdl);
base = mdl + "/World Frame/W";
follower = mdl + "/Five Bar Robot/Pen/Pen/Tip";
addFrameVariables(fk,"Pen","Translation", base, follower);

% Specify the center and radius of the circle
center_x = 0; % m
center_y = 2.5; % m
radius = 0.15; % m
th = 0:pi/50:2*pi; % radians

% Save the x and y coordinates of the circle for the IK analyses
coordinates_x = radius * cos(th) + center_x;
coordinates_y = radius * sin(th) + center_y;

ik = simscape.multibody.KinematicsSolver(mdl);

addFrameVariables(ik, "Pen","Translation", base, follower);

frameVariables(ik);

targetIds = ["Pen.Translation.x";"Pen.Translation.y"];
addTargetVariables(ik,targetIds);

jointPositionVariables(ik);

outputIds = ["j5.Rz.q";"j4.Rz.q";"j1.Rz.q";"j2.Rz.q"];
addOutputVariables(ik,outputIds);

guessIds = ["j1.Rz.q";"j2.Rz.q";"j5.Rz.q";"j4.Rz.q"];
addInitialGuessVariables(ik,guessIds);

guesses = [50 -50 45 135]; % Assign initial guesses for passive and active joints
N = length(coordinates_x);
index_in_ik = 1;
ik_data = zeros(N,length(outputIds)); % Allocate memory for the variable
for ind = 1:N
    targets = [coordinates_x(ind), coordinates_y(ind)]; % Use the x and y coordinates of the prescribed circle for the inverse kinematics problem
    [outputVec_ik,statusFlag_ik] = solve(ik,targets, guesses);
    if statusFlag_ik == 1 
        viewSolution(ik) % View the solution in the Kinematics Solver Viewer
        ik_data(index_in_ik,1:length(outputIds)) = outputVec_ik % Save the rotations of the motors
        index_in_ik=index_in_ik+1;
        guesses = [outputVec_ik(3) outputVec_ik(4) outputVec_ik(1) outputVec_ik(2)]; % Update next guess with last solution
    else
        error("Did not hit targets");
    end
end


