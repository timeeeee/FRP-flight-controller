module State_Functions
(k_function
,f_function
,h_function
,rateLimit) where  

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

-- 

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
f_function xs ys = [fTHL_o xs ys] ++ [fEL_o xs ys] ++ [fAIL_o xs ys] ++ [fRDR_o xs ys] ++ 
									 [fVT_o xs]  ++ [falpha_o xs] ++ [fbeta_o xs] ++ 
									 [fphi_o xs] ++ [ftheta_o xs] ++ [fpsi_o xs] ++ 
									 [fP_o xs] ++ [fQ_o xs] ++ [fR_o xs] ++ 
									 [falphab_o xs] ++ [fbetab_o xs] ++ 
									 [accelX_o xs] ++ [accelY_o xs] ++ [accelZ_o xs] ++ 
									 [(xs !! 18)] ++ [falpha_o xs]

fTHL_o :: [Float] -> [Float] -> Float
fTHL_o xs ys = (let (throttle,throttle_cmd) = ((xs !! 0), (ys !! 0))
								in aThrottle*throttle+bThrottle*(limitThrottle throttle_cmd))
						 
fEL_o :: [Float] -> [Float] -> Float
fEL_o xs ys = (let (elevator,elevator_cmd) = ((xs !! 1), (ys !! 1))
							 in aElevator*elevator+bElevator*(limitElevator elevator_cmd))


fAIL_o :: [Float] -> [Float] -> Float
fAIL_o xs ys = (let (aileron,aileron_cmd) = ((xs !! 2), (ys !! 2))
							  in aAileron*aileron+bAileron*(limitAileron aileron_cmd))
							 
fRDR_o :: [Float] -> [Float] -> Float							
fRDR_o xs ys = (let (rudder, rudder_cmd) = ((xs !! 3), (ys !! 3))
								 in aRudder*rudder+bRudder*(limitRudder rudder_cmd))

								 						 
fVT_o :: [Float] -> Float		
fVT_o xs = (((u xs)*(uDot xs)+(v xs)*(vDot xs)+(w xs)*(wDot xs))/(xs !! 4))

falpha_o :: [Float] -> Float		
falpha_o xs = (((u xs)*(wDot xs) - (w xs) * (uDot xs))/((u xs)^2 + (w xs)^2))

fbeta_o :: [Float] -> Float		
fbeta_o xs = (((xs !! 4)*(vDot xs)-(v xs)*(fVT_o xs))/(((u xs)^2 + (w xs)^2)*(cos (xs !! 6))))

fphi_o :: [Float] -> Float		
fphi_o xs = ((xs !! 10)+((((xs !! 12))*(cos (xs !! 7))+((xs !! 11))*(sin (xs !! 7)))*(tan (xs !! 8))))

ftheta_o :: [Float] -> Float		
ftheta_o xs = ((((xs !! 11))*(cos (xs !! 7)))-(((xs !! 12))*(sin (xs !! 7))))

fpsi_o :: [Float] -> Float		
fpsi_o xs = ((((xs !! 12))*(cos (xs !! 7))+((xs !! 11))*(sin (xs !! 7)))/(cos (xs !! 8)))

fP_o :: [Float] -> Float		
fP_o xs = (((c8)*(xs !! 10)-(c2)*(xs !! 12))*(xs !! 11)+(qBar (xs !! 4))*s*b*((c4)*(cl_o xs)+(c9)*(cn_o xs)))

fQ_o :: [Float] -> Float		
fQ_o xs =((c5)*((xs !! 10))*((xs !! 12))-(c6)*(((xs !! 10))^2-((xs !! 12))^2)+(qBar (xs !! 4))*s*cBar*(c7)*(cm_o xs))

fR_o :: [Float] -> Float		
fR_o xs = (((c8)*((xs !! 10))-(c2)*((xs !! 12)))*((xs !! 11))+(qBar (xs !! 4))*s*b*((c4)*(cl_o xs)+(c9)*(cn_o xs)))


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


								
--rateLimit adjusts the previous stateObserver output to impose limits 
-- I am not sure why we are doing this at input over output...?
rateLimit:: [Float] -> [Float]
rateLimit xs = [limitThrottle (xs !! 0)] ++ [limitElevator (xs !! 1)] ++ [limitAileron (xs !! 2)] ++ [limitRudder (xs !! 3)] ++
							 [limitVT (xs !! 4)] ++ [limitAlpha (xs !! 5)] ++ [limitBeta (xs !! 6)] ++ 
							 [limitPhi (xs !! 7)] ++ [limitTheta (xs !! 8)] ++ [limitPsi (xs !! 9)] ++ 
							 [limitP (xs !! 10)] ++ [limitQ (xs !! 11)] ++ [limitR (xs !! 12)] ++ 
							 [limitBias1 (xs !! 13)] ++ [limitBias2 (xs !! 14)] ++ (drop 15 xs) 
							 
limitThrottle :: Float -> Float
limitThrottle x
   | x > throttleMax = throttleMax
   | x < throttleMin = throttleMin
   | otherwise       = x
	 
limitElevator :: Float -> Float
limitElevator x
   | x > elevatorMax = elevatorMax
   | x < elevatorMin = elevatorMin
   | otherwise       = x
	 
limitAileron :: Float -> Float
limitAileron x
   | x > aileronMax = aileronMax
   | x < aileronMin = aileronMin
   | otherwise      = x
	 
limitRudder :: Float -> Float
limitRudder x
   | x > rudderMax = rudderMax
   | x < rudderMin = rudderMin
   | otherwise     = x
	 

limitVT	:: Float -> Float
limitVT x
   | x > 200 				= 200
   | x < (10*eps) 	= (10*eps)
   | otherwise     	= x
	 
limitAlpha :: Float -> Float
limitAlpha x
   | x > (90*pi/180) 	= (90*pi/180)
   | x < (-90*pi/180) = (-90*pi/180)
   | otherwise     		= x

limitBeta :: Float -> Float
limitBeta x
   | x > (90*pi/180) 	= (90*pi/180)
   | x < (-90*pi/180) = (-90*pi/180)
   | otherwise     		= x
	 
limitPhi :: Float -> Float
limitPhi x
   | x > pi 		= pi
   | x < (-90) 	= (-pi)
   | otherwise  = x
	 
limitTheta :: Float -> Float
limitTheta x
   | x > pi 		= pi
   | x < (-90) 	= (-pi)
   | otherwise  = x

limitPsi :: Float -> Float
limitPsi x
   | x > (90*pi/180) 	= (90*pi/180)
   | x < (-90*pi/180) = (-90*pi/180)
   | otherwise     		= x	 
	 
limitP :: Float -> Float
limitP x
   | x > (900*pi/180) 	= (900*pi/180)
   | x < (-900*pi/180) 	= (-900*pi/180)
   | otherwise     			= x	 
	 
limitQ :: Float -> Float
limitQ x
   | x > (600*pi/180) 	= (600*pi/180)
   | x < (-600*pi/180) 	= (-600*pi/180)
   | otherwise  	   		= x	 
	 
limitR :: Float -> Float
limitR x
   | x > 200 		= 200
   | x < (-200) = (-200)
   | otherwise  = x	 

limitBias1 :: Float -> Float
limitBias1 x
   | x > (30*pi/180) 	= (30*pi/180)
   | x < (-30*pi/180) 	= (-30*pi/180)
   | otherwise  	   		= x	 
	 
limitBias2 :: Float -> Float
limitBias2 x
   | x > (30*pi/180) 	= (30*pi/180)
   | x < (-30*pi/180) 	= (-30*pi/180)
   | otherwise  	   		= x	 
	 

