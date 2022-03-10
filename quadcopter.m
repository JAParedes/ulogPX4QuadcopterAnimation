% Draw a quadcopter in x configuration, as well as arrows aligned with
% body-fixed frame. 
% Return a handle to the quadcopter object.
% Modified from https://github.com/gibiansky/experiments/tree/master/quadcopter/matlab
function h = quadcopter(sc)
    
    ll = 10*sc;
    hh = 0.5*sc;
    
    % Create arms.
    h(1) = prism_1(ll, hh);
    h(2) = prism_2(ll, hh);
    
    ll2 = ll*cos(pi/4)/2;
    
    % Create propellers (cylinder + 2 circular bases)
    [x,y,z] = cylinder;
    x = sc * 2 * x;
    y = sc * 2 * y;
    z = sc * 0.25 * z;
    h(3) = surf(x + ll2, y + ll2, z+0.25*sc, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(4) = surf(x + ll2, y - ll2, z+0.25*sc, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(5) = surf(x - ll2, y + ll2, z+0.25*sc, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(6) = surf(x - ll2, y - ll2, z+0.25*sc, 'EdgeColor', 'none', 'FaceColor', 'b');
    
    [X,Y,Z] = ellipsoid(0,0,0,1,1,0.001) ;
    X = sc * 2 * X;
    Y = sc * 2 * Y;
    
    h(7) = surf(X + ll2, Y + ll2, Z+0.25*sc, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(8) = surf(X + ll2, Y - ll2, Z+0.25*sc, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(9) = surf(X - ll2, Y + ll2, Z+0.25*sc, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(10) = surf(X - ll2, Y - ll2, Z+0.25*sc, 'EdgeColor', 'none', 'FaceColor', 'b');
    
    h(11) = surf(X + ll2, Y + ll2, Z+0.25*sc+sc * 0.25, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(12) = surf(X + ll2, Y - ll2, Z+0.25*sc+sc * 0.25, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(13) = surf(X - ll2, Y + ll2, Z+0.25*sc+sc * 0.25, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(14) = surf(X - ll2, Y - ll2, Z+0.25*sc+sc * 0.25, 'EdgeColor', 'none', 'FaceColor', 'b');

    % Create arrows aligned with body-fixed frame
    h(15) =arrow3d([0 ll2],[0 0],[hh hh],.8,.02,.04,'r');
    h(16) = arrow3d([0 0],[0 ll2],[hh hh],.8,.02,.04,'g');
    h(17) = arrow3d([0 0],[0 0],[hh hh+ll2],.8,.02,.04,'b');
    
    % Conjoin all quadcopter parts into one object.
    t = hgtransform;
    set(h, 'Parent', t);
    h = t;
end


% Prism for arm 1
function h = prism_1(l, h)
    [X,Y,Z] = arm_prism_faces_1(l, h);

    faces(1, :) = [4 2 1 3];
    faces(2, :) = [4 2 1 3] + 4;
    faces(3, :) = [4 2 6 8];
    faces(4, :) = [4 2 6 8] - 1;
    faces(5, :) = [1 2 6 5];
    faces(6, :) = [1 2 6 5] + 2;

    for i = 1:size(faces, 1)
        h(i) = fill3(X(faces(i, :)), Y(faces(i, :)), Z(faces(i, :)), 'r'); hold on;
    end

    % Conjoin all prism faces into one object.
    t = hgtransform;
    set(h, 'Parent', t);
    h = t;
end

% Prism for arm 2
function h = prism_2(l, h)
    [X,Y,Z] = arm_prism_faces_2(l, h);

    faces(1, :) = [4 2 1 3];
    faces(2, :) = [4 2 1 3] + 4;
    faces(3, :) = [4 2 6 8];
    faces(4, :) = [4 2 6 8] - 1;
    faces(5, :) = [1 2 6 5];
    faces(6, :) = [1 2 6 5] + 2;

    for i = 1:size(faces, 1)
        h(i) = fill3(X(faces(i, :)), Y(faces(i, :)), Z(faces(i, :)), 'r'); hold on;
    end

    % Conjoin all prism faces into one object.
    t = hgtransform;
    set(h, 'Parent', t);
    h = t;
end

% Prism points for arm 1
function [X,Y,Z] = arm_prism_faces_1(l, h)
    ll = l/2;
    hh = h/2;
    alph = cos(pi/4);
    X = [ll*alph-hh*alph ll*alph+hh*alph...
        ll*alph-hh*alph ll*alph+hh*alph...
        -ll*alph-hh*alph -ll*alph+hh*alph...
        -ll*alph-hh*alph -ll*alph+hh*alph];
    Y = [ll*alph+hh*alph ll*alph-hh*alph...
        ll*alph+hh*alph ll*alph-hh*alph...
        -ll*alph+hh*alph -ll*alph-hh*alph...
        -ll*alph+hh*alph -ll*alph-hh*alph];
    Z = [-hh -hh hh hh -hh -hh hh hh];
end

% Prism points for arm 2
function [X,Y,Z] = arm_prism_faces_2(l, h)
    ll = l/2;
    hh = h/2;
    alph = cos(pi/4);
    X = [-ll*alph+hh*alph -ll*alph-hh*alph...
        -ll*alph+hh*alph -ll*alph-hh*alph...
        ll*alph+hh*alph ll*alph-hh*alph...
        ll*alph+hh*alph ll*alph-hh*alph];
    Y = [ll*alph+hh*alph ll*alph-hh*alph...
        ll*alph+hh*alph ll*alph-hh*alph...
        -ll*alph+hh*alph -ll*alph-hh*alph...
        -ll*alph+hh*alph -ll*alph-hh*alph];
    Z = [-hh -hh hh hh -hh -hh hh hh];
end