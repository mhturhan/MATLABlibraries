% A class for a serial link (One link attached to the next at it's endpoint).
% Author: Andrew Peekema

classdef SerialLink < handle
    properties
        f1   % Link start (SE3 element)
        f1f0 % Link start with respect to (w.r.t) f0 (SE3 element)
        g1f1 % Link midpoint w.r.t f1 (SE3 element)
        g1f0 % Link midpoint w.r.t f0 (SE3 element)
        h1g1 % Link endpoint w.r.t g1 (SE3 element)
        h1f0 % Link endpoint w.r.t f0 (SE3 element)
        h1f1 % Link endpoint w.r.t f1 (SE3 element)

        f0 % The origin
        qxyz % The rotation vector between f0 and f
        l % Link length
        r % Radius

        vis % CylinderClass object
    end

    methods
        % Constructor
        function obj = SerialLink(f0, qxyz, l, r)
            % Defaults if no arguments
            if nargin == 0
                f0 = SE3;       % The identity matrix
                qxyz = [0 0 0]; % No rotation
                l = 1;          % Unit length
                r = 0.1;        % Radius
            end

            % Link start
            obj.f1 = SE3([0 0 0 qxyz]);;
            obj.f1f0 = f0*obj.f1;

            % If this is a symbolic expression
            if isa(obj.f1f0.g,'sym')
                % Clean up notation
                obj.f1f0.g = simplify(obj.f1f0.g,'IgnoreAnalyticConstraints',true);
            end

            % Link midpoint
            obj.g1f1 = SE3([l/2 0 0]);
            obj.g1f0 = obj.f1f0*obj.g1f1;

            % Link end
            obj.h1g1 = SE3([l/2 0 0]);
            obj.h1f1 = obj.g1f1*obj.h1g1;
            obj.h1f0 = obj.g1f0*obj.h1g1;

            % Save the input variables
            obj.f0   = f0;
            obj.qxyz = qxyz;
            obj.l    = l;
            obj.r    = r;

            % Create a visualization object
            obj.vis = CylinderClass(r,l);
        end % serialLink

        function plot(obj)
            % Put the shape into a plot
            obj.vis.plot
        end % plot

        function gFun = plotFun(varargin)
            % Make the center link matrix a function
            % from: Variables (e.g. {'r1' 'r2'})
            % to: Numbers to substitute (e.g. {0.2 0.1})

            % The first argument is always the object
            obj = varargin{1};

            % If there are no arguments, use empty cells
            if length(varargin) == 3
                from = varargin{2};
                to   = varargin{3};
            else
                from = {};
                to   = {};
            end

            % Generate the SE3 matrix function for the center link
            gFun = matlabFunction(subs(obj.g1f0.g,from,to));
        end % gFun

        function plotObj(obj,g)
            % Plot the object at a position (g)

            % Rest the object frame
            obj.vis.resetFrame

            % Move to g
            obj.vis.globalMove(SE3(g))

            % Update the plot
            obj.vis.updatePlotData

            % Note: Don't forget to "drawnow" in the main loop, otherwise the
            % object might not show up
        end % plotObjPos

        function showCoord(obj,g)
            % Show the coordinate system of the object at a position (g)

            % Make an SE3 object of the center frame
            g = SE3(g);

            % Move in the x direction
            gx = g*SE3([3*obj.l/2 0 0]);
            % Move in the y direction
            gy = g*SE3([0 obj.r*2 0]);
            % Move in the z direction
            gz = g*SE3([0 0 obj.r*2]);

            % Plot a line in the x-direction
            plot3([g.x gx.x],[g.y gx.y],[g.z gx.z],'r')
            % Plot a line in the y-direction
            plot3([g.x gy.x],[g.y gy.y],[g.z gy.z],'g')
            % Plot a line in the z-direction
            plot3([g.x gz.x],[g.y gz.y],[g.z gz.z],'b')
        end % showCoord

        function traceEnd(obj,g,prop)
            % Trace the end of the link given
            %   g: the center frame
            %   prop: the plot properties

            % Make an SE3 object of the center frame
            g = SE3(g);

            % Move to the end of the link
            h = g*SE3([obj.l/2 0 0]);

            % Plot the point
            plot3(h.x,h.y,h.z,prop)
        end % traceEnd

    end % methods

end % classdef
