%% Human Arm support calculations
% 2R - planar arm
% origin: o
% shoulder: 1
% elbow: 2
% hand: 3

classdef Arm < handle
    properties
        theta = zeros(2,1); % anatomical angles (1: shoulder, 2: elbow)
        l1 = 0.4; % length of fore arm
        l2 = 0.25; % length of upper arm
        p1 = 0.5; % location of center of mass of upper arm as a percentile
        p2 = 0.5; % location of center of mass of fore arm as a percentile

        g = 9.81; 
        m1 = 3.2;
        m2 = 1.8;
    end

    methods
        function obj = Arm(theta)
            obj.theta = theta;
            obj.theta(1) = obj.theta(1)-pi/2;
        end
        % origin to shoulder
        function T_o1 = origin_shoulder(obj)
            theta = obj.theta(1); %-pi/2;
            T_o1 = [cos(theta), -sin(theta), 0;
                sin(theta),  cos(theta), 0;
                0, 0, 1];
        end
        % shoulder to elbow
        function T_12 = shoulder_elbow(obj)
            T_12 = [cos(obj.theta(2)), -sin(obj.theta(2)), obj.l1;  ...
                sin(obj.theta(2)),  cos(obj.theta(2)), 0;  0, 0, 1];
        end

        function T_23 = elbow_hand(obj)
            % elbow to hand
            T_23 = [eye(2), [obj.l2;0];  0, 0, 1];
        end

        % shoulder to hand
        function T_13 = shoulder_hand(obj)
            T_13 = obj.shoulder_elbow(obj.theta(1))*obj.elbow_hand(obj.theta(2));
        end

        % shoulder to upper arm CoM
        function T_1x1 = shoulder_uCOM(obj)
            x1 = obj.l1*obj.p1; % distance of CoM from shoulder
            T_1x1 = eye(3);
            T_1x1(1:2,3) = [x1;0];
        end

        % elbow to fore arm CoM
        function T_2x2 = elbow_fCOM(obj)
            x2 = obj.l2*obj.p2; % distance of CoM from shoulder
            T_2x2 = eye(3);
            T_2x2(1:2,3) = [x2;0];
        end

        % gravity forces of upper arm and fore arm
        function Fg = get_Fg(obj)
            Fg = obj.g*[obj.m1, obj.m2];
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
    end

    methods(Static)
        function g = get_config(T)
            g = [T(1:2,3)', atan2(T(2,1), T(1,1))];
        end
    end


end

