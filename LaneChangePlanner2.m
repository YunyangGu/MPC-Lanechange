classdef LaneChangePlanner2 < matlab.System
    % Untitled Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties
    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)
        m_traj = zeros(600,7);
        m_disptraj = zeros(600,3);
        xs = zeros(241,1);
        ys = zeros(241,1);
        phis = zeros(241,1);
        ss = zeros(241,1);
        m_trajs = cell(1,90);
        m_cnt = 1;
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            mat = load('roadpointdata.mat');
            obj.xs = mat.pointdata(:,1);
            obj.ys = mat.pointdata(:,2);
            obj.phis = mat.pointdata(:,3);
            obj.ss = mat.pointdata(:,6);
        end

        function [trajDisp,trajRet] = stepImpl(obj,replanflag,t0,lanenum)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            if(replanflag == 1)
                s0 = 0;
                actLats = 0;
                actLatv = 0.0;
                selectIdx = 1;
                if t0 > 0.1
                    for i = 1:size(obj.m_traj,1)
                        if abs(t0 - obj.m_traj(i,6)) <= 0.0001
                            selectIdx = i;
                            break;
                        end
                    end
                    s0 = obj.m_traj(selectIdx, 7);
                    tp_x = obj.m_traj(selectIdx, 1);
                    tp_y = obj.m_traj(selectIdx, 2);
                    tp_phi = obj.m_traj(selectIdx, 3);
                    tp_v = obj.m_traj(selectIdx, 5);
                    
%                     [pref_x,pref_y,pref_phi] = obj.GetXYPhiFromS(s0);
                     pref_x = interp1(obj.ss, obj.xs , s0);
             pref_y = interp1(obj.ss, obj.ys , s0);
             pref_phi = interp1(obj.ss, obj.phis , s0);
                    dx = tp_x - pref_x;
                    dy = tp_y - pref_y;
                    actLats = dy* cos(pref_phi)-dx*sin(pref_phi);
                    actLatv = tp_v* (sin(tp_phi)* cos(pref_phi)-cos(tp_phi)*sin(pref_phi));
                end
                tgtLatss = [4.0, 0.0, -4.0];
                tgtLats = tgtLatss(lanenum + 1);
                disFacTable_X = [-0.01 0.2, 0.3, 0.5, 1.0, 1.8, 2.0,4.0 4.01];
                disFacTable_Y = [0.0 0.0, 0.1, 0.4, 0.6, 0.8, 1.0,1.0 1.0];
                fac = interp1(disFacTable_X,disFacTable_Y,abs(tgtLats - actLats));
                [lon_pos, lon_spd, lat_pos, lat_spd] = GenTrajPoints(actLats, actLatv*fac, tgtLats);
%                 disp('***********LaneChangePlanner*************')
%                 disp(fac)
%                 disp(tgtLats)
%                 disp(actLats)
                t = (1:600)*0.01;
                s = s0 + lon_pos;
                v = sqrt(lon_spd.^2 + lat_spd.^2);
                phi = atan(lat_spd./lon_spd);
obj.m_disptraj(:,1) = s;
obj.m_disptraj(:,2) = lat_pos;

%                 [pref_x,pref_y,pref_phi] = obj.GetXYPhiFromS(s);
             pref_x = interp1(obj.ss, obj.xs , s);
             pref_y = interp1(obj.ss, obj.ys , s);
             pref_phi = interp1(obj.ss, obj.phis , s);
%                 pref_x = obj.GetXFromS(s);
%                 pref_y = obj.GetYFromS(s);
%                 pref_phi = obj.GetPhiFromS(s);
                x = pref_x + lat_pos.*cos(pref_phi + pi/2);
                y = pref_y + lat_pos.*sin(pref_phi + pi/2);
                obj.m_traj(:,1) = x;
                obj.m_traj(:,2) = y;
                disp('replan')
                disp(y);
                obj.m_traj(:,3) = phi+pref_phi;
                obj.m_traj(:,4) = phi*0;
                obj.m_traj(:,5) = v;
                obj.m_traj(:,6) = t + t0;
                obj.m_disptraj(:,3) = t + t0;
                obj.m_traj(:,7) = s;
                len = size(obj.m_traj,1);
                obj.m_traj(1,  4) = 0;
                obj.m_traj(len,4) = 0;
                for i = 2:(len-1)
                    a = obj.m_traj(i-1, 1:2);
                    b = obj.m_traj(i  , 1:2);
                    c = obj.m_traj(i+1, 1:2);
                    obj.m_traj(i,  4) = getCurvature(a,b,c);
                end
                obj.m_trajs{obj.m_cnt} = obj.m_traj;
                obj.m_cnt = obj.m_cnt + 1;
            end      
            trajRet = obj.m_traj;
            trajDisp = obj.m_disptraj;
        end
        function releaseImpl(obj)
            saveData.data = obj.m_trajs;
            saveData.cnt = obj.m_cnt;
%             saveData.m_LatsDiffs = obj.m_LatsDiffs;
            save('Trajs.mat', 'saveData');
        end
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        function [trajDisp,trajRet]  = getOutputSizeImpl(~)
            % Return size for each output port
            trajRet = [600,7];
            trajDisp = [600,3];
        end
        
        %------------------------------------------------------------------
        function [trajDisp,trajRet]  = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            trajRet    = 'double';
            trajDisp = 'double';
        end
        
        %------------------------------------------------------------------
        function [trajDisp,trajRet]  = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            trajRet        = false;
            trajDisp = false;
        end
        
        %------------------------------------------------------------------
        function [trajDisp,trajRet]  = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            trajRet        = true;
            trajDisp = true;
        end
    end
end
