function main()
    close all;
    figure;
    axis equal;
    xlim([-50 50]);
    ylim([-50 50]);
    hold on;

    x_rb = 15; 
    y_rb = -8; 
    theta_rb = -3*pi/4; 
    radius_rb = 3;
    
    n = 30;
    X_goal = randi([-50 50], 1, n);
    Y_goal = randi([-50 50], 1, n);

    Kp = 1.4;
    Ki = 0.1;
    Kd = 0.2;
    dt = 0.02;
    steps = 3000;

    t = linspace(0,2*pi,100);
    circleX0 = radius_rb*cos(t);
    circleY0 = radius_rb*sin(t);

    hCircle = plot(circleX0 + x_rb, circleY0 + y_rb, '-', ...
                   'Color',[0.6 0.6 0.6], 'LineWidth',1.5);
    hArrow = quiver(x_rb, y_rb, radius_rb*cos(theta_rb), radius_rb*sin(theta_rb), ...
                    0,'Color',[0 0.7 0],'LineWidth',2,'MaxHeadSize',0.5);

    pathX = x_rb;
    pathY = y_rb;
    hPath = plot(pathX, pathY, 'b-', 'LineWidth',1.5);

    for i = 1:n
        x_goal = X_goal(i);
        y_goal = Y_goal(i);
        plot(x_goal, y_goal, 'p','MarkerSize',12,'MarkerFaceColor','r');
        for k = 1:steps
            beta  = atan2(y_goal-y_rb, x_goal-x_rb);
            error = pipi(beta, theta_rb);
            omega = Kp*error + Ki*error*dt + Kd*error/dt;
            V = 30;
            if (norm([y_goal-y_rb x_goal-x_rb]) < 0.4)
                break;
            end
            disp(V);

            dx = V * cos(theta_rb);
            dy = V * sin(theta_rb);
            dtheta = omega;

            x_rb = x_rb + dx * dt;
            y_rb = y_rb + dy * dt;
            
            theta_rb = theta_rb + dtheta * dt;

            X = circleX0 * cos(theta_rb) - circleY0 * sin(theta_rb) + x_rb;
            Y = circleX0 * sin(theta_rb) + circleY0 * cos(theta_rb) + y_rb;
            set(hCircle, 'XData', X, 'YData', Y);
    
            set(hArrow, 'XData', x_rb, ...
                        'YData', y_rb, ...
                        'UData', radius_rb*cos(theta_rb), ...
                        'VData', radius_rb*sin(theta_rb));
    
            pause(dt);

            pathX(end+1) = x_rb;
            pathY(end+1) = y_rb;
            set(hPath, 'XData', pathX, 'YData', pathY);
        end
    end
end

function d = pipi(a, b)
    d = a - b;
    d = atan2(sin(d), cos(d));
end
