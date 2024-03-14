Vg=24; %Converter input voltage
L = 8e-3; %Converter input inductor
C = 4700e-6; %Converter output capacitor
Fsw = 25e3; %Switching frequency
io = 1; %Input current setpoint
Vc=48; %Converter output voltage

% State space model
Dp = Vg/Vc; %Duty cycle
D = 1-Dp; 
IL = io/(1-D); %Converter input current
Am = [0 -Dp/L; Dp/C 0];
Bm = [Vc/L 1/L 0; -IL/C 0 -1/C];
Cm = [1 0];
Dm = [0 0 0];
Sys = ss(Am,Bm,Cm,Dm);
G = tf(Sys);
Gid = G(1); %Inductor current with respect to D
GiVg = G(2); %Inductor current with respect to Vg
GiIo = G(3); %Inductor current with respect to Io
s = tf('s');
Gvir = Dp/(C*s); % Voltage with respect to I_reference
Gvio = -1/(C*s); % Voltage with respect to Io

%%%%%%Voltage control design

%Battery
V_ref = 48; %Reference voltage
V_B = 24; %Input voltage (Battery voltage)
wv1 = 2*pi*50; %Voltage control bandwidth
Co1 = 4700*10^-6; %Input capacitance
D1 = 1-(V_B/V_ref); %Duty cycle
eta = 1/10;

kvp1 = (wv1*Co1)/D1; %Proportional constant voltage control
kvi1 = kvp1*eta*wv1; %Integral constant voltage control

%Supercapacitor
V_C = 24; %Input voltage (Initial SC voltage)
wv2 = 2*pi*80; %Voltage control bandwidth
Co2 = 4700*1e-6; %Input capacitance
D2 = 1-(V_C/V_ref); %Duty cycle
eta = 1/10;

kvp2 = (wv2*Co2)/D2; %Proportional constant voltage control
kvi2 = kvp2*eta*wv2; %Integral constant voltage control

%%%%%%Current control
%Battery
wi1 = 2*pi*800; %Current control bandwidth
L1 = 0.008; %Input inductance
V_ref = 48; %Reference voltage

kip1 = (wi1*L1)/V_ref; %Proportional constant current control
kii1 = kip1*eta*wi1; %Integral constant current control

%SC
wi2 = 2*pi*800; %Current control bandwidth
L2 = 0.008; %Input inductance
V_ref = 48; %Reference voltage

kip2 = (wi2*L2)/V_ref; %Proportional constant current control
kii2 = kip2*eta*wi2; %Integral constant current control

%Battery control
Gcv = kvp1+(kvi1/s); %Battery voltage control
Gci = kip1+(kii1/s); %Battery current control

%SC control
Gcv2 = kvp2+(kvi2/s); %SC voltage control
Gci2 = kip2+(kii2/s); %SC current control

%Closed loop transfer function Battery
gclosevoltage = (Gvir*Gcv)/(1+Gvir*Gcv);
gclosecurrent = (Gid*Gci)/(1+Gid*Gci);

%Closed loop transfer function SC
gclosevoltage2 = (Gvir*Gcv2)/(1+Gvir*Gcv2);
gclosecurrent2 = (Gid*Gci2)/(1+Gid*Gci2);

%Controller design with saturation feedback
%For the battery
%Battery voltage control with saturation feedback
E1 = s+120; %Auxiliary function, obtained from the eigenvalues of the closed loop transfer function
c1anti = (kvp1*s + kvi1)/E1;
c2anti = (s-E1)/(E1);
%Battery current control with saturation feedback
E2 = s+280; %Auxiliary function
c1antic = (kip1*s + kii1)/E2;
c2antic = (s-E2)/(E2);

%For the SC
E1SC = s+200; %Auxiliary function
%SC voltage control with saturation feedback
c1antiSC = (kvp2*s + kvi2)/E1SC;
c2antiSC = (s-E1SC)/(E1SC);
%SC current control with saturation feedback
E2SC = s+280; %Auxiliary function
c1anticSC = (kip2*s + kii2)/E2SC;
c2anticSC = (s-E2SC)/(E2SC);

%Design of virtual filters
wc = 0.4*2*pi; %Cutoff frequency
xi = 0.8; %Damping ratio
RV = 1; %Virtual drop resistance
wn = wc/(((1+(2*(xi)^2))+sqrt((1+(2*(xi)^2))+1))^(1/2)); %Natural frequency
CV = 1/(2*RV*xi*wn); %Virtual drop capacitance
ki = wn^2*RV*CV; %Integral constant






