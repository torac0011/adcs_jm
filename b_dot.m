classdef b_dot < handle
    properties
        delta_t
        inertia
        altitude
        p
        linvel
        mf = [0; 0; 0]
        m = [0; 0; 0]
        control_torque = [0; 0; 0]
        theta_x = 0
        theta_y = 0
        theta_z = 0
        acceleration = [0; 0; 0]
        latitude
        counter = 0
        longitude
    end
    
    methods 
        function obj = b_dot(delta_t, inertia, altitude)
            obj.delta_t = delta_t;
            obj.inertia = diag(inertia);
            obj.altitude = altitude;
            Re = 6371.2e3; % Earth radius
            Rc = Re + altitude ; %  Distance from earth in metres
            obj.p = ((111e3 * Rc) / Re);
            G = 6.67428e-11; % Earth gravitational constant
            M = 5.972e24; % Earth mass
            obj.linvel = sqrt(G * M / Rc); % Linear velocity of the satellite
        end
        
        function get_bdot_params(self, omega)
            %represents the polar orbit
            self.latitude = (self.linvel * self.delta_t * self.counter)/self.p;% as its a latitude changes differently when its in space.
            self.latitude = 25 + self.latitude;
            % setting up the latitude for the law, latitude changes due to the change in the y coordinate of the system.
            self.longitude = 60 + ((7.29e-5 * self.delta_t * self.counter) * 180 / pi);
            %longitude change as the eath is rotating
            % Gives error on 90
            if self.latitude > 90
                self.latitude = 90 - mod(self.latitude, 90);
            elseif self.latitude < -90
                self.latitude = -90 - mod(self.latitude, -90);
            elseif self.latitude == 90
                self.latitude = 89.97;
            elseif self.latitude == -90
                self.latitude = -89.97;
            end
            if self.longitude > 180
                self.longitude = mod(self.longitude, 180) - 180;
            elseif self.longitude < -180
                self.longitude = 180 + mod(self.longitude, -180);
            end
            mag_field_nT = igrfmagm(self.altitude, self.latitude, self.longitude, decyear(2015,6,23),12);
            
            mag_field = mag_field_nT * 1e-9; % IMPORTANT TO NOTE THAT THE Z COMPONENT OBTAINED IS +VE DOWNWARDS.
            self.mf = mag_field.';
            self.counter = self.counter + 1;
            % mag_field_vector3=[mag_field_vector2(1),0,0;0,mag_field_vector2(2),0;0,0,mag_field_vector2(3)];
            
%             self.theta_x = self.theta_x + (omega(1) * self.delta_t)+(0.5* self.acceleration(1) * self.delta_t * self.delta_t);
%             self.theta_y = self.theta_y + (omega(2) * self.delta_t)+(0.5* self.acceleration(2) * self.delta_t * self.delta_t);
%             self.theta_z = self.theta_z + (omega(3) * self.delta_t)+(0.5* self.acceleration(3) * self.delta_t * self.delta_t);
%             %
%             
%             rx = [1, 0, 0; 0, cos(self.theta_x), -sin(self.theta_x); 0, sin(self.theta_x), cos(self.theta_x)];
%             ry = [cos(self.theta_y), 0, sin(self.theta_y); 0, 1, 0; -sin(self.theta_y), 0, cos(self.theta_y)];
%             rz = [cos(self.theta_z), -sin(self.theta_z), 0; sin(self.theta_z), cos(self.theta_z), 0; 0, 0, 1];
%             
%             self.mf = rx * ry * rz*mag_field;

            % as the magnetic field vecctor has the value in nano tesla
            % convert the obtained magnetic field in the frame of reference of the satellite.
            
            detb = -1/sqrt(dot((self.mf), (self.mf)));
%             gain = -1000;
            self.m = detb * cross(self.mf, omega); % magnetic moment
%             for i = 1 : length(self.m)
%                 if self.m(i) > .0016
%                     self.m(i) = .0016;
%                 elseif self.m(i) < -.0016
%                     self.m(i) = -.0016;
%                 end
%             end
            self.control_torque = cross(self.m, self.mf);
            self.acceleration = self.inertia \ self.control_torque;
        end
        
        function mf = get_mf(self)
            mf = self.mf;
        end
        
        function control_torque = get_torque(self)
            control_torque = self.control_torque;
        end
        
        function m = get_action_control(self)
            m = self.m;
        end
        
    end
end
    
