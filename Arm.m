%% Human Arm support calculations
% 2R - planar arm
% origin: o
% shoulder: 1
% elbow: 2
% hand: 3

classdef Arm < handle
    properties
        theta = zeros(2,1); % anatomical angles (1: shoulder, 2: elbow)
        len = zeros(2,1) % length of arm segments
        p = zeros(2,1); % location of center of mass of arm as a percentile
        mass = zeros(2,1);
        g = 9.81; 
    end

    methods
        function obj = Arm(theta, len, p_COM, mass)
            obj.theta = theta;
            obj.theta(1) = obj.theta(1)-pi/2;
            obj.len = len;
            obj.p = p_COM;
            obj.mass = mass;
            
        end
        % origin to shoulder
        function T_o1 = origin_shoulder(obj)
            T_o1 = [cos(obj.theta(1)), -sin(obj.theta(1)), 0;
                sin(obj.theta(1)),  cos(obj.theta(1)), 0;
                0, 0, 1];
        end
        % shoulder to elbow
        function T_12 = shoulder_elbow(obj)
            T_12 = [cos(obj.theta(2)), -sin(obj.theta(2)), obj.len(1);  ...
                sin(obj.theta(2)),  cos(obj.theta(2)), 0;  0, 0, 1];
        end

        function T_23 = elbow_hand(obj)
            % elbow to hand
            T_23 = [eye(2), [obj.len(2);0];  0, 0, 1];
        end

        % shoulder to hand
        function T_13 = shoulder_hand(obj)
            T_13 = obj.shoulder_elbow(obj.theta(1))*obj.elbow_hand(obj.theta(2));
        end

        % shoulder to upper arm CoM
        function T_1x1 = shoulder_uCOM(obj)
            x1 = obj.len(1)*obj.p(1); % distance of CoM from shoulder
            T_1x1 = eye(3);
            T_1x1(1:2,3) = [x1;0];
        end

        % elbow to fore arm CoM
        function T_2x2 = elbow_fCOM(obj)
            x2 = obj.len(2)*obj.p(2); % distance of CoM from shoulder
            T_2x2 = eye(3);
            T_2x2(1:2,3) = [x2;0];
        end

        % gravity forces of upper arm and fore arm
        function Fg = get_Fg(obj)
            Fg = obj.g*[obj.mass(1), obj.mass(2)];
        end
        % torques of upper arm and fore arm
        function Tq = get_Tq(obj)
            Fg = obj.get_Fg();
            T_1x1 = obj.origin_shoulder()*obj.shoulder_uCOM();
            T_1x2 = obj.origin_shoulder()*obj.shoulder_elbow()*obj.elbow_fCOM();
            g1 = Arm.get_config(T_1x1);
            g2 = Arm.get_config(T_1x2);
            d = [norm(g1(1:2)), norm(g2(1:2))];
            phi = [atan2(g1(2),g1(1)), atan2(g2(2),g2(1))];
            Tq = Fg.*d.*cos(phi);
        end

        function T_elbow_support = elbow_support(~, c)
            T_elbow_support = eye(3);
            T_elbow_support(1:2,3) = [c;0];
        end
        % supporting force c meters away from elbow
        function opt_f = get_support(obj, c)
            T_elbow_support = obj.elbow_support(c);
            T_origin_support = obj.origin_shoulder()*obj.shoulder_elbow()*T_elbow_support;
            g_support = Arm.get_config(T_origin_support);
            r = norm(g_support(1:2));
            phi = atan2(g_support(2), g_support(1));

            net_Torque = sum(obj.get_Tq());
            F_support_torque = net_Torque/r;
            a_support_torque = phi+pi/2;

            F_support_y = sum(obj.get_Fg());

            F_support = F_support_torque*[cos(a_support_torque); sin(a_support_torque); 0];

            r_vec = [g_support(1); g_support(2); 0];

            fun = @(f)norm(sum(cross(r_vec, f))-net_Torque)^2;
            Aeq = [0,0,1;
                   0,1,0];
            beq = [0;
                   F_support_y];

            opt_f = fmincon(fun, F_support, [],[], Aeq, beq);
            T = cross(r_vec,opt_f);

        end
        function show(arm, support_distance)
            s_T = 0.02 ; % scale torque dimension
            s_F = 0.01; % scale gravity force dimension
            s_S = 0.005; % scale supporting force dimension
            g_uCOM = Arm.get_config(arm.origin_shoulder()*arm.shoulder_uCOM());
            g_fCOM = Arm.get_config(arm.origin_shoulder()*arm.shoulder_elbow()*arm.elbow_fCOM());
            g_support = Arm.get_config(arm.origin_shoulder()*arm.shoulder_elbow()*arm.elbow_support(support_distance));
            
            Tq = arm.get_Tq();
            Fg = arm.get_Fg();
            F_support = arm.get_support(support_distance);
            
            a_u = atan2(g_uCOM(2),g_uCOM(1));
            a_f = atan2(g_fCOM(2),g_fCOM(1));
            
            planarR2_display(arm.theta, arm.len); hold on
            plot(g_uCOM(1),g_uCOM(2),'*r')
            plot(g_fCOM(1),g_fCOM(2),'*r')
            plot(g_support(1),g_support(2),'*g')
            
            quiver(g_uCOM(1), g_uCOM(2), s_T*Tq(1)*sin(a_u), s_T*-Tq(1)*cos(a_u), 'Color', [1,0,0]);
            quiver(g_fCOM(1), g_fCOM(2), s_T*Tq(2)*sin(a_f), s_T*-Tq(2)*cos(a_f), 'Color', [1,0,0]);
            quiver(g_uCOM(1), g_uCOM(2), 0, -s_F*Fg(1), 'Color', [0,0,1]);
            quiver(g_fCOM(1), g_fCOM(2), 0, -s_F*Fg(2), 'Color', [0,0,1]);
            
            dir_support = s_S*F_support;
            quiver(g_support(1), g_support(2), dir_support(1),dir_support(2), 'Color', [0,1,0]);
            
            title(['Net Force: ' num2str(sum(arm.get_Fg())) 'N, Net Torque: ' num2str(sum(arm.get_Tq())) 'Nm'])
        end
    end

    methods(Static)
        function g = get_config(T)
            g = [T(1:2,3)', atan2(T(2,1), T(1,1))];
        end
    end


end

