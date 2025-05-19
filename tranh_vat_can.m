function main()
    close all;
    figure;
    axis equal;
    xlim([-150 150]);
    ylim([-150 150]);
    hold on;

    x_rb = 10;
    y_rb = -40;
    theta_rb = -3*pi/4;
    radius_rb = 3;
    
    n = 1;

    ob = polyshape([0 0 15 15],[15 0 0 15]);
    ob_zone = 7.5*sqrt(2);

    Kp = 1.4;
    Ki = 0.1;
    Kd = 0.2;
    dt = 0.02;
    steps = 1000;

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
        x_goal = 0;
        y_goal = 40;
        plot(x_goal, y_goal, 'p','MarkerSize',12,'MarkerFaceColor','r');
        x_ob = 0;
        y_ob = 0;
        x_ob_center = x_ob + 7.5;
        y_ob_center = y_ob + 7.5;
        ob = translate(ob, [x_ob, y_ob]);
        plot(ob);
        for k = 1:steps
            % --- Vector hướng về đích ---
            u_goal = x_goal - x_rb;
            v_goal = y_goal - y_rb;
            norm_goal = norm([u_goal, v_goal]);
            u_goal = u_goal / norm_goal;
            v_goal = v_goal / norm_goal;
        
            % --- Vector tránh vật cản (chỉ khi gần vật cản) ---
            u_avoid = x_rb - x_ob_center;
            v_avoid = y_rb - y_ob_center;
            norm_avoid = norm([u_avoid, v_avoid]);
            if norm_avoid < ob_zone + 10 % chỉ tránh khi gần vật cản
                u_avoid = u_avoid / norm_avoid;
                v_avoid = v_avoid / norm_avoid;
            else
                u_avoid = 0;
                v_avoid = 0;
            end
        
            % --- Tổng hướng đi ---
            alpha = 0.5; % trọng số hướng về đích
            beta = 0.5;  % trọng số né vật cản
            u_total = alpha*u_goal + beta*u_avoid;
            v_total = alpha*v_goal + beta*v_avoid;
        
            % Chuẩn hóa vector tổng
            norm_total = norm([u_total, v_total]);
            u_total = u_total / norm_total;
            v_total = v_total / norm_total;
        
            % Vẽ các vector (chỉ để minh họa, có thể bỏ hoặc giữ lại)
            %quiver(x_rb, y_rb, u_goal*20, v_goal*20, 0, 'Color', 'b'); % hướng về đích
            %quiver(x_rb, y_rb, u_avoid*20, v_avoid*20, 0, 'Color', 'r'); % hướng tránh vật cản
            %quiver(x_rb, y_rb, u_total*20, v_total*20, 0, 'Color', 'g'); % hướng tổng hợp
        
            % Điều chỉnh hướng robot theo vector tổng
            gamma = atan2(v_total, u_total);
            error = pipi(gamma, theta_rb);
            omega = Kp*error + Ki*error*dt + Kd*error/dt;
            V = 30;
        
            % Dừng nếu gần đích
            if (norm([y_goal-y_rb, x_goal-x_rb]) < 0.4)
                break;
            end
        
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
