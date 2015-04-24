module State_Functions
(k_function
,f_function
,h_function) where  

import Constants
import Matrix_Ops
import Common_Equations
import Data.List

-- k_function INPUT:
-- xs - previous stateObserver ouput - time t-1					(index address into list)
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)

-- ys - t-1 list of previous measurements			(index address into list)
-- 								vt, alpha, beta												(0,1,2)
-- body rates			p, q, r																(3,4,5)
-- magnetic flux 	hx_body, hy_body, hz_body							(6,7,8)
-- pilot commands throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd (9,10,11,12)
-- accelerometer 	axb, ayb, azb													(13,14,15)
-- gpsPosition    lat, long, alt 												(16,17,18)
-- gpsVelocity		vNorth, vEast, vDown									(19,20,21)
-- enable																								(22)
-- OUTPUT: 15 x 11 matrix
k_function :: [Float] -> [Float] -> [[Float]]
k_function xs ys = k_states


--f_function INPUT:
-- xs - previous stateObserver ouput - time t-1					(index address into list)
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)

-- ys - t-1 list of previous measurements			(index address into list)
-- pilot commands throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd (0,1,2,3)
-- accelerometer 	axb, ayb, azb													(4,5,6)
-- gpsPosition    lat, long, alt 												(7,8,9)
-- gpsVelocity		vNorth, vEast, vDown									(10,11,12)
-- enable																								(13)

-- OUTPUT:
--								fTHL_o,fEL_o,fAIL_o,fRDR_o, 					(0,1,2,3)
--								fVT_o,falpha_o,fbeta_o,								(4,5,6)
--								fphi_o,ftheta_o,fpsi_o,								(7,8,9)
--								fP_o,fQ_o,fR_o,												(10,11,12)
--								falphab_o,fbetab_o,										(13,14)
--								accelX_o,accelY_o,accelZ_o						(15,16,17)
--								alphaDot, alphaDot1,									(18,19)
f_function :: [Float] -> [Float] -> [Float]
f_function xs ys = [falpha_o xs]++ [falpha_o xs] ++ [fTHL_o xs ys] ++ [fEL_o xs ys] ++ [fAIL_o xs ys] ++ [fRDR_o xs ys] ++ [fVT_o xs]  ++ ys ++ [falpha_o xs] 
--   ++ [fbeta_o xs] ++ [fphi_o xs] ++ [ftheta_o xs] ++ [fpsi_o xs] ++ [fP_o xs] ++ [fQ_o xs] ++ [fR_o xs] ++ [falphab_o xs] ++ [fbetab_o xs]

fTHL_o :: [Float] -> [Float] -> Float
fTHL_o xs ys = (let (throttle,throttle_cmd) = ((xs !! 0), (ys !! 0))
								in aThrottle*throttle+bThrottle*throttle_cmd)
						 
fEL_o :: [Float] -> [Float] -> Float
fEL_o xs ys = (let (elevator,elevator_cmd) = ((xs !! 1), (ys !! 1))
							 in aElevator*elevator+bElevator*elevator_cmd)


fAIL_o :: [Float] -> [Float] -> Float
fAIL_o xs ys = (let (aileron,aileron_cmd) = ((xs !! 2), (ys !! 2))
							  in aAileron*aileron+bAileron*aileron_cmd)
							 
fRDR_o :: [Float] -> [Float] -> Float							
fRDR_o xs ys = (let (rudder, rudder_cmd) = ((xs !! 3), (ys !! 3))
								 in aRudder*rudder+bRudder*rudder_cmd)

fVT_o :: [Float] -> Float		
fVT_o xs = ((u xs)*(uDot xs)+(v xs)*(vDot xs)+(w xs)*(wDot xs)/(xs !! 4))

falpha_o :: [Float] -> Float		
falpha_o xs = (u xs)
--((u xs)*(wDot xs)-(w xs)*(uDot xs))/((u xs)^2+(w xs)^2)
-- ; %a alphadot state has to be incorporated later


--fbeta_o xs =(VT_o*Vdot_o-V_o*fVT_o)/(U_o^2+W_o^2)*cos(beta_o);
--fphi_o xs =P_o+(R_o*cos(phi_o)+Q_o*sin(phi_o))*tan(theta_o);
--ftheta_o xs =Q_o*cos(phi_o)-R_o*sin(phi_o);
--fpsi_o xs =(R_o*cos(phi_o)+Q_o*sin(phi_o))/cos(theta_o);
--fP_o xs =(c2*P_o+c1*R_o)*Q_o+Qbar_o*S*b*(c3*Cl_o+c4*Cn_o);
--fQ_o xs =c5*P_o*R_o-c6*(P_o^2-R_o^2)+Qbar_o*S*Cbar*c7*Cm_o;
--fR_o xs =(c8*P_o-c2*R_o)*Q_o+Qbar_o*S*b*(c4*Cl_o+c9*Cn_o);

falphab_o :: [Float] -> Float
falphab_o xs = 0.0

fbetab_o :: [Float] -> Float
fbetab_o xs = 0.0




--h_function INPUT:
--								fVT_o,falpha_o,fbeta_o,								(0,1,2)
--								fphi_o,ftheta_o,fpsi_o,								(3,4,5)
--								fP_o,fQ_o,fR_o,												(6,7,8)
--								falphab_o,fbetab_o,										(9,10)
--								accelX_o,accelY_o,accelZ_o						(11,12,13)
-- OUTPUT:
--								hVT_o,halpha_o,hbeta_o,								(0,1,2)
--								hP_o,hQ_o,hR_o,												(3,4,5)
--			          hHxbody_o,hHybody_o,hHzbody_o					(6,7,8)
h_function :: [Float] -> [Float]
h_function xs = [hVT_o xs] ++ [halpha_o xs] ++ [hbeta_o xs] ++ [hP_o xs] ++ [hQ_o xs] ++ [hR_o xs] ++ [hHxbody_o xs] ++ [hHybody_o xs] ++ [hHzbody_o xs]

hVT_o :: [Float] -> Float
hVT_o xs = (xs !! 0)

halpha_o :: [Float] -> Float
halpha_o xs =(xs !! 1) + (xs !! 9)

hbeta_o :: [Float] -> Float
hbeta_o xs = (xs !! 2) + (xs !! 10)

hP_o :: [Float] -> Float
hP_o xs = (xs !! 6)

hQ_o :: [Float] -> Float
hQ_o xs = (xs !! 7)

hR_o :: [Float] -> Float
hR_o xs = (xs !! 8)

hHxbody_o :: [Float] -> Float
hHxbody_o xs = (let (theta_o, psi_o) = ((xs !! 4),(xs !! 5))
								in (hN0noaa*cos(theta_o)*cos(psi_o))+(hE0noaa*cos(theta_o)*sin(psi_o))-(hD0noaa*sin(theta_o)))

hHybody_o :: [Float] -> Float								
hHybody_o xs = (let (theta_o, psi_o, phi_o) = ((xs !! 4),(xs !! 5),(xs !! 3))
								in (hN0noaa*(sin(phi_o)*sin(theta_o)*cos(psi_o)-cos(phi_o)*sin(psi_o)))+(hE0noaa*(sin(phi_o)*sin(theta_o)*sin(psi_o)+cos(phi_o)*cos(psi_o)))+(hD0noaa*sin(phi_o)*cos(theta_o)))

hHzbody_o :: [Float] -> Float
hHzbody_o xs = (let (theta_o, psi_o, phi_o) = ((xs !! 4),(xs !! 5),(xs !! 3))
								in (hN0noaa*(cos(phi_o)*sin(theta_o)*cos(psi_o)+sin(phi_o)*sin(psi_o)))+(hE0noaa*(cos(phi_o)*sin(theta_o)*sin(psi_o)-sin(phi_o)*cos(psi_o)))+(hD0noaa*cos(phi_o)*cos(theta_o)))
