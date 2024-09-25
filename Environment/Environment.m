classdef Environment < handle

    properties(Constant)
        %% Initialise constant variables

        % Location of robotic arms
        origin = transl(0,0,0); 
        
        cartInitial = [[0.65, 0, 0]; ...    % self.cart.objectModel{1} UR3 robotic arm
                      [-0.65, 0, 0]];       % self.cart.objectModel{2} KUKA robotic arm

        platesInitial = [[0, 0.4, 0.325]];

        steps = 50;

    end

    properties
        %% Initialise changing variables
        % Robotic arm UR3
        ur3Origin
        ur3;
        ur3Gripper;
        ur3GripperOrigin;

        % Rbotic arm KUKA
        kukaOrigin;
        kuka;
        kukaGripper;
        kukaGripperOrigin;

        % % Origin point for robotic arms
        % ur3Origin;
        % kukaOrigin;

        % Holder for initial states for robotic arms
        initialStateUR3;
        initialStateKuka;

        % Objects
        plates;
        numPlates;
        dumplingTray;
        numDumplingTray;
        cart;
        numCart;
        
        % Testing
        testgripper

    end
    
    methods

        % Testing class comment out after use
        function self = Environment()
            self.loadEnvironment();
            drawnow();
        end

        function loadEnvironment(self)
            clf
            
            surf([-1,1; -1,1] ...
                ,[-1,-1; 1,1] ...
                ,[0,0; 0.01,0.01] ...
                ,'CData',imread('concrete.jpg') ... % Concrete from canvas
                ,'FaceColor','texturemap');
            axis equal;
            axis tight;
            view(3);
            hold on;
            
            % Round table (dia = 1.01167, r = 0.5 height = 0.3225) placed at [0, 0, 0.01]
            PlaceObject('table.ply',[0,0,0.01]);
            
            % Plate (Radius = 0.079)
            % PlaceObject('plate.ply', [0, 0.4, 0.325]);
            self.numPlates = 1;
            self.plates = Objects('plate', self.numPlates);
            for i = 1:self.numPlates
                self.plates.objectModel{i}.base = SE3(self.platesInitial(i,:)).T;
                self.plates.objectModel{i}.animate(0);
                drawnow();
                pause(0.01);
            end
            
            % Dumpling tray (Radius = 0.055m, height = 0.035m)
            PlaceObject('dumpling_tray.ply', [0, 0.2, 0.33]);
            
            % cart (lenght = 0.448, width = 0.225, height = 0.381)
            % height for arm is 0.344
            % height for plate/dishes is 0.075
            % PlaceObject('cart.ply', [0.65, 0, 0]);
            self.numCart = 2;
            self.cart = Objects('cart', self.numCart);
            for i = 1:self.numCart
                self.cart.objectModel{i}.base = SE3(self.cartInitial(i,:)).T;
                self.cart.objectModel{i}.animate(0);
                drawnow();
                pause(0.01);
            end

            self.ur3Origin = self.cart.objectModel{1}.base.T * transl(0,0,0.35);
            self.ur3 = UR3Modified(self.ur3Origin);

            self.kukaOrigin = self.cart.objectModel{2}.base.T * transl(0,0,0.35);
            self.kuka = UR3Modified(self.kukaOrigin);
            
            % testing gripper
            self.testgripper = LinearUR3eGripper(transl(0,0,0.5));
            


        end
    end

end