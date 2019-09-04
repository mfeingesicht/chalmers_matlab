classdef bike3DShow
    properties
        frame
        front
        rear
        handle
        bwheel
        ground
        fwheel
        SteerAngle
        TiltAngle
    end
    methods
        function obj = bike3DShow()
            obj.SteerAngle = 0;
            obj.TiltAngle = 0;
            
            obj.ground.f = [1 3 2 4];
            obj.ground.v = [
                -5 -15 0
                5 5 0
                5 -15 0
                -5 5 0
            ];
            obj.ground.g = hgtransform;
            
            
            [obj.frame.f, obj.frame.v] = createRectangle(1/4,5,1/4);
            [obj.front.f, obj.front.v] = createRectangle(1/4,4,1/4);
            [obj.rear.f, obj.rear.v] = createRectangle(1/4,3,1/4);
            [obj.handle.f, obj.handle.v] = createRectangle(1/4,4,1/4);
            [obj.fwheel.f1, obj.fwheel.f2, obj.fwheel.v] = createCylinder(3,0.25,20);
            [obj.bwheel.f1, obj.bwheel.f2, obj.bwheel.v] = createCylinder(3,0.25,20);
            
            
            obj.front.v = (rotx(90)*obj.front.v')' + [0 0 6.5];
            obj.rear.v = (rotx(90)*obj.rear.v')' + [0 -10  5.5];
            obj.fwheel.v = (roty(90)*obj.fwheel.v')' + [0 0 3];
            obj.bwheel.v = (roty(90)*obj.bwheel.v')' + [0 -10 3];
            obj.frame.v = obj.frame.v + [0 -5 8];
            obj.handle.v = (rotz(90)*obj.handle.v')' + [0 0 10];
            
            
            obj.fwheel.g = hgtransform;     
            obj.bwheel.g = hgtransform;     
            obj.front.g = hgtransform;
            obj.rear.g = hgtransform;
            obj.handle.g = hgtransform;
            obj.frame.g = hgtransform;
            
            %bigfig
            obj.draw();
            view(3)
            box on
            axis vis3d
            daspect([1 1 1])
        end
        
        function updateRotations(obj)
            
            obj.fwheel.g.Matrix = ...
                makehgtform('yrotate',obj.TiltAngle)*makehgtform('zrotate',obj.SteerAngle);
%                 makehgtform('translate',[0 5 -9.75])*...
%                 makehgtform('yrotate',pi/2)*...
%                 makehgtform('xrotate',obj.SteerAngle)*...
                
%                 makehgtform('translate',[0 0 10]);
            
            obj.bwheel.g.Matrix = ...
                makehgtform('yrotate',obj.TiltAngle);
%                 makehgtform('yrotate',pi/2)*...
%                 makehgtform('translate',[6.5 -5 0]);
%                 
%             
            obj.front.g.Matrix = ...
                makehgtform('yrotate',obj.TiltAngle)*makehgtform('zrotate',obj.SteerAngle);
%                 makehgtform('yrotate',-obj.SteerAngle)*...
%                 makehgtform('xrotate',pi/2)*...
%                 makehgtform('translate',[0 -2.5 -5]);
%                 
%             
            obj.rear.g.Matrix = ...
                makehgtform('yrotate',obj.TiltAngle);
%                 makehgtform('xrotate',pi/2)*...
%                 makehgtform('translate',[0 -2.5 5]);
%                 
%             
            obj.handle.g.Matrix = ...
                makehgtform('yrotate',obj.TiltAngle)*makehgtform('zrotate',obj.SteerAngle);
%                 makehgtform('xrotate',obj.TiltAngle)*...
%                 makehgtform('zrotate',pi/2)*...
%                 makehgtform('translate',[5 0 1.75]);
%                 
%             
            obj.frame.g.Matrix = ...
                makehgtform('yrotate',obj.TiltAngle);
            
        end
        
        function draw(obj)
            patch('Vertices',obj.fwheel.v,'Faces',obj.fwheel.f1,'FaceColor',[.3 .3 .3],'Parent',obj.fwheel.g)
            patch('Vertices',obj.fwheel.v,'Faces',obj.fwheel.f2,'FaceColor',[.1 .1 .1],'Parent',obj.fwheel.g)
            patch('Vertices',obj.bwheel.v,'Faces',obj.bwheel.f1,'FaceColor',[.3 .3 .3],'Parent',obj.bwheel.g)
            patch('Vertices',obj.bwheel.v,'Faces',obj.bwheel.f2,'FaceColor',[.1 .1 .1],'Parent',obj.bwheel.g)
            patch('Vertices',obj.frame.v,'Faces',obj.frame.f,'FaceColor',[1 0 0],'Parent',obj.frame.g)
            patch('Vertices',obj.front.v,'Faces',obj.front.f,'FaceColor',[1 0 0],'Parent',obj.front.g)
            patch('Vertices',obj.rear.v,'Faces',obj.rear.f,'FaceColor',[1 0 0],'Parent',obj.rear.g)
            patch('Vertices',obj.handle.v,'Faces',obj.handle.f,'FaceColor',[.75 .75 .75],'Parent',obj.handle.g)
            patch('Vertices',obj.ground.v,'Faces',obj.ground.f,'FaceColor',[.4 .1 .1],'Parent',obj.ground.g)
           
            drawnow
        end
        
        function fromData(obj, data)
            tic
            time = data.time;
            state = data.signals.values;
            index = 1;
           
            try
             obj.SteerAngle = -state(index,3);
             obj.TiltAngle = state(index,1);
             obj.updateRotations();
             obj.draw()
             s = sprintf("t = %.2f s    theta = %.2f [DEG]   v = %.2f m/s",time(index),180/pi*state(index,1),state(index,5));
             title(s);       
               
            catch
                clc
                close all
            end
        end
    end
end


