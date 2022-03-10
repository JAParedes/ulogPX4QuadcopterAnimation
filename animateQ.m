% Move and rotate quadcopter using the model handle, a translation vector
% and a quaternion to represent rotation
% Modified from https://github.com/gibiansky/experiments/tree/master/quadcopter/matlab
function animateQ(model, x, q)
        % Compute translation to correct linear position coordinates.
        dx = x;
        move = makehgtform('translate', dx);
        % Compute rotation to correct angles. Then, turn this rotation
        % into a 4x4 matrix represting the affine transformation.
        rotate = quat2rotm(q);
        rotate = [rotate zeros(3, 1); zeros(1, 3) 1];

        % Move the quadcopter to the right place, after putting it in the correct orientation.
        set(model,'Matrix', move * rotate);
end