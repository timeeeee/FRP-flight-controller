function [fTHL_o,fEL_o,fAIL_o,fRDR_o,...
          fVT_o,falpha_o,fbeta_o,fphi_o,ftheta_o,fpsi_o,fP_o,fQ_o,fR_o,...
          falphab_o,fbetab_o,...
          accelX_o,accelY_o,accelZ_o...
          ]...
          =f_x_u_state_EKF_sailplane(...
                    alphadot_o,...
                    THL_o,EL_o,AIL_o,RDR_o,...
                    VT_o,alpha_o,beta_o,phi_o,theta_o,psi_o,P_o,Q_o,R_o,...
                    alphab_o,betab_o,...
                    THLcmd_o,ELcmd_o,AILcmd_o,RDRcmd_o,...
                    accelX_meas_o,accelY_meas_o,accelZ_meas_o,...
                    rho,...
                    S,Cbar,b,weight,g,IxxB,IyyB,IzzB,IxzB,...
                    CD0,CDu,CDa,CDq,CDadot,CDde,...
                    CD0_bar,...
                    Cyb,Cyp,Cyr,Cyda,Cydr,...
                    CL0,CLa,CLq,CLadot,CLu,CLde,...
                    Clb,Clp,Clr,Clda,Cldr,...
                    Cm0,Cma,Cmq,Cmadot,Cmu,Cmde,...
                    Cnb,Cnp,Cnr,Cnda,Cndr,...
                    xT0,xT1,xT2,...
                    Ptrim,Qtrim,Rtrim,Utrim,...
                    Athrottle,Bthrottle,...
                    Aelevator,Belevator,...
                    Aaileron,Baileron,...
                    Arudder,Brudder,...
                    ThrottleMax,ThrottleMin,...
                    ElevatorMax,ElevatorMin,...
                    AileronMax,AileronMin,...
                    RudderMax,RudderMin)

%mass and inertia coefficients
mass=weight/g;                                            %[lbf-s2/ft]
delta_I=IxxB*IzzB-IxzB^2;
c1=((IyyB-IzzB)*IzzB-IxzB^2)/delta_I;
c2=((IxxB-IyyB+IzzB)*IxzB)/delta_I;
c3=IzzB/delta_I;
c4=IxzB/delta_I;
c5=(IzzB-IxxB)/IyyB;
c6=IxzB/IyyB;
c7=1/IyyB;
c8=(IxxB*(IxxB-IyyB)+IxzB^2)/delta_I;
c9=IxxB/delta_I;
%dynamic pressure
Qbar_o=0.5*rho*VT_o^2;                                    %[lbf/ft2]
%body velocities
U_o=VT_o*cos(alpha_o)*cos(beta_o);                        %[ft/s]
V_o=VT_o*sin(beta_o);                                     %[ft/s]
W_o=VT_o*sin(alpha_o)*cos(beta_o);                        %[ft/s]
%aerodynamic coefficients
CL_o=CL0+CLa*alpha_o+CLq*(Q_o-Qtrim)*Cbar/2/Utrim+CLadot*alphadot_o*Cbar/2/Utrim+CLu*(U_o-Utrim)/Utrim+CLde*EL_o;
%CD_k=CD0+CDa*alpha_k+CDq*(Q_k-Qtrim)*Cbar/2/Utrim+CDadot*alphadot_k*Cbar/2/Utrim+CDu*(U_k-Utrim)/Utrim+CDde*EL_k; %lin
CD_o=CD0_bar+CL_o^2/(pi*b/Cbar*0.87)+CDq*(Q_o-Qtrim)*Cbar/2/Utrim+CDadot*alphadot_o*Cbar/2/Utrim+CDu*(U_o-Utrim)/Utrim+CDde*EL_o; %nonlin
CY_o=Cyb*beta_o+Cyp*(P_o-Ptrim)*b/2/Utrim+Cyr*(R_o-Rtrim)*b/2/Utrim+Cyda*AIL_o+Cydr*RDR_o;
Cls_o=Clb*beta_o+Clp*(P_o-Ptrim)*b/2/Utrim+Clr*(R_o-Rtrim)*b/2/Utrim+Clda*AIL_o+Cldr*RDR_o;
Cms_o=Cm0+Cma*alpha_o+Cmq*(Q_o-Qtrim)*Cbar/2/Utrim+Cmadot*alphadot_o*Cbar/2/Utrim+Cmu*(U_o-Utrim)/Utrim+Cmde*EL_o;
Cns_o=Cnb*beta_o+Cnp*(P_o-Ptrim)*b/2/Utrim+Cnr*(R_o-Rtrim)*b/2/Utrim+Cnda*AIL_o+Cndr*RDR_o;
CxA_o=-CD_o*cos(alpha_o)+CL_o*sin(alpha_o);
CyA_o=CY_o;
CzA_o=-CD_o*sin(alpha_o)-CL_o*cos(alpha_o);
Cl_o=Cls_o*cos(alpha_o)-Cns_o*sin(alpha_o);
Cm_o=Cms_o;
Cn_o=Cls_o*sin(alpha_o)+Cns_o*cos(alpha_o);
%engine thrust
T_o=xT2*(100*THL_o)^2+xT1*(100*THL_o)+xT0;                %[lbf]
%S&C-based accelerometer readings
accelX_scd_o=(Qbar_o*S*CxA_o+T_o)/mass;                   %[ft/s2]
accelY_scd_o=Qbar_o*S*CyA_o/mass;                         %[ft/s2]
accelZ_scd_o=Qbar_o*S*CzA_o/mass;                         %[ft/s2]
%accelerometer corresponding value (chose from scd-based or measurements)
accelX_o=accelX_scd_o;%accelX_meas_o;                     %[ft/s2]
accelY_o=accelY_scd_o;%accelY_meas_o;                     %[ft/s2]
accelZ_o=accelZ_scd_o;%accelZ_meas_o;                     %[ft/s2]
%body accelerations
Udot_o=R_o*V_o-Q_o*W_o-g*sin(theta_o)+accelX_o;           %[ft/s2]
Vdot_o=P_o*W_o-R_o*U_o+g*sin(phi_o)*cos(theta_o)+accelY_o;%[ft/s2]
Wdot_o=Q_o*U_o-P_o*V_o+g*cos(phi_o)*cos(theta_o)+accelZ_o;%[ft/s2]

%servo command saturation
if     THLcmd_o>ThrottleMax; THLcmd_o=ThrottleMax;
elseif THLcmd_o<ThrottleMin; THLcmd_o=ThrottleMin;
end;
if     ELcmd_o>ElevatorMax;  ELcmd_o=ElevatorMax;
elseif ELcmd_o<ElevatorMin;  ELcmd_o=ElevatorMin;
end;
if     AILcmd_o>AileronMax;  AILcmd_o=AileronMax;
elseif AILcmd_o<AileronMin;  AILcmd_o=AileronMin;
end;
if     RDRcmd_o>RudderMax;   RDRcmd_o=RudderMax;
elseif RDRcmd_o<RudderMin;   RDRcmd_o=RudderMin;
end;

%%f(x,u)
fTHL_o=Athrottle*THL_o+Bthrottle*THLcmd_o;
fEL_o=Aelevator*EL_o+Belevator*ELcmd_o;
fAIL_o=Aaileron*AIL_o+Baileron*AILcmd_o;
fRDR_o=Arudder*RDR_o+Brudder*RDRcmd_o;
fVT_o=(U_o*Udot_o+V_o*Vdot_o+W_o*Wdot_o)/VT_o;
falpha_o=(U_o*Wdot_o-W_o*Udot_o)/(U_o^2+W_o^2); %a alphadot state has to be incorporated later
fbeta_o=(VT_o*Vdot_o-V_o*fVT_o)/(U_o^2+W_o^2)*cos(beta_o);
fphi_o=P_o+(R_o*cos(phi_o)+Q_o*sin(phi_o))*tan(theta_o);
ftheta_o=Q_o*cos(phi_o)-R_o*sin(phi_o);
fpsi_o=(R_o*cos(phi_o)+Q_o*sin(phi_o))/cos(theta_o);
fP_o=(c2*P_o+c1*R_o)*Q_o+Qbar_o*S*b*(c3*Cl_o+c4*Cn_o);
fQ_o=c5*P_o*R_o-c6*(P_o^2-R_o^2)+Qbar_o*S*Cbar*c7*Cm_o;
fR_o=(c8*P_o-c2*R_o)*Q_o+Qbar_o*S*b*(c4*Cl_o+c9*Cn_o);
falphab_o=0;
fbetab_o=0;