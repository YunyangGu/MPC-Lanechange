classdef MPCControlB < matlab.System& matlab.system.mixin.Propagates
    % Untitled2 Add summary here
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
        m_steer = 0;
        m_acc = 0;
        sRef = 0;
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

         function [acc, steer, sRef] = ...
                  stepImpl(obj, controlFlag, traj_ref, veh_state, t)   
                  % 时间间隔
                  dT = 0.01;
                  % 输出参数
                  %  controlFlag 控制标志，为1得时候才会计算控制量，否则沿用上�?次得�?
                  %  veh_state 当前车辆的状�?
                  %  t 当前的时�?
                  %  traj_ref 轨迹，包含以下字�?
                  %       x
                  %       y
                  %       phi
                  %       curvature
                  %       v
                  %       t
                  %       s
                  disp('*************MPCControl******Loop***************');
                  disp(['time is ',num2str(t)])
                  disp(controlFlag)
                  % 如果controlFlag大于0 则重新计算控制量
                  if(controlFlag > 0)
                      % 车辆参数
                      VehConf.m  = 1217.2;
                      VehConf.Cf = 80000;
                      VehConf.Cr = 80000;
                      VehConf.Lf = 1.165;
                      VehConf.Lr = 1.165;
                      VehConf.Iz = 1020;
                      % 从规划的轨迹中按照时间戳比较，取出当前需要跟踪的轨迹�?
                      selectIdx = 1;
                      for i = 1:size(traj_ref,1)
                        if abs(t - traj_ref(i,6)) <= 0.0011
                            selectIdx = i;
                            break;
                        end
                      end
                      % selectIdx就是选取的轨迹点的索�?
                      % 得到待跟踪轨迹点的�?�度
                      vDes    = traj_ref(selectIdx,5);
                      % 得到待跟踪轨迹点的航向角
                      phiDes  = traj_ref(selectIdx,3);
                      % 得到待跟踪轨迹点的曲�?
                      kDes    = traj_ref(selectIdx,4);
                      obj.sRef = traj_ref(selectIdx,7);
                      dphiDes = vDes*kDes;
                      % 得到待跟踪点的x坐标
                      xTarget = traj_ref(selectIdx,1);
                      % 得到待跟踪点的y坐标
                      yTarget = traj_ref(selectIdx,2);
                      % 计算x y偏差
                      dxTarget = veh_state.xpos - xTarget;
                      dyTarget = veh_state.ypos - yTarget;
                      disp('*************MPCControl*******disp**************');
                      disp(t)
                      disp(selectIdx)
                      disp(traj_ref(1,6))
                      disp(traj_ref(2,6))
                      disp(traj_ref(end,6))
                      disp(traj_ref(selectIdx, 6))
                      disp(xTarget);
                      disp(yTarget);
                      disp(veh_state.xpos)
                      disp(veh_state.ypos)
                      % 计算航向角偏差和偏差�?
                      heading_error      = veh_state.yaw - phiDes;
                      heading_error_rate = veh_state.yawrate - dphiDes;
                      % 计算横向位置偏差和偏差率
                      lateral_error      = dyTarget*cos(phiDes) - dxTarget*sin(phiDes);
                      lateral_error_rate = veh_state.vact*sin(heading_error);
                      % 计算纵向速度偏差和位置偏�?
                      kParam = 1;
                      speed_error =  vDes - veh_state.vact*cos(heading_error)/kParam;
                      station_error = -(dxTarget*cos(phiDes) + dyTarget*sin(phiDes));
                      ErrState = [lateral_error;lateral_error_rate;heading_error;heading_error_rate; station_error; speed_error];
                      
%                       disp('***************MPCControl*****ErrState**************');
%                       disp(ErrState);
                      % 得到MPC控制中ABC矩阵的�??
                      [A, B, C] = GetMPCControlMatrix(VehConf, veh_state.vact);
                      % ABC矩阵的�?�离散化
                      Ad = eye(6) + dT*A;
                      Bd = dT*B;
                      Cd = dT*C*heading_error_rate;
                      % 求解MPC问题
                      u = solveMPC(Ad,Bd,Cd,ErrState,zeros(6,1));
                      % 得到的解保存起来
                      obj.m_steer = u(1);
                      obj.m_acc   = u(2);
                  end
                  steer = obj.m_steer;
                  acc = obj.m_acc;
                  sRef = obj.sRef;
                   
         end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        function [acc, steer,sRef] = getOutputSizeImpl(~)
            % Return size for each output port
            acc = 1;
            steer = 1;
            sRef = 1;
        end
        
        %------------------------------------------------------------------
        function [acc, steer,sRef] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            acc    = 'double';
            steer     = 'double';
            sRef = 'double';
        end
        
        %------------------------------------------------------------------
        function [acc, steer,sRef] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            acc        = false;
            steer    = false;
            sRef = false;
        end
        
        %------------------------------------------------------------------
        function [acc, steer,sRef] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            acc        = true;
            steer    = true;
            sRef = true;
        end
        
    end
end
