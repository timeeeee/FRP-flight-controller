module State_Estimation  
(stateOutput) where  

import Constants
import Matrix_Ops
import Data.List

--import Common_Equations

-- DONE
-- stateOutput INPUT:
--							xs -	List of Current Measurements
--							ys -  List of Lists (3)
--										1 - previous output of stateObserver
--										2 - previous output of navigationObserver
--										3 - previous list of measurements
--						OUTPUT:
--							List of Lists (3)
--										1 - output of stateObserver
--										2 - output of navigationObserver
--										3 - list of measurements

stateOutput :: [Float] -> [[Float]] -> [[Float]]
stateOutput xs [] = [stateObserver xs [] [],	navigationObserver xs [] [], xs]
stateOutput xs ys = [stateObserver xs (ys !! 0) (ys !! 2),	navigationObserver xs (ys !! 1) (ys !! 2), xs ]



-- PART ONE - STATE OBSERVER

-- DONE stateObserver INPUT xs - current time t: 						(index address into list)
-- 								(vt, alpha, beta)											(0,1,2)
-- body rates			(p, q, r)															(3,4,5)
-- magnetic flux 	(hx_body, hy_body, hz_body)						(6,7,8)
-- pilot commands (throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd) (9,10,11,12)
-- accelerometer 	(axb, ayb, azb)												(13,14,15)
-- gpsPosition    (lat, long, alt) 											(16,17,18)
-- gpsVelocity		(vNorth, vEast, vDown)								(19,20,21)


-- ys - previous stateObserver ouput - time t-1					(index address into list)
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)
--								p_est																	(20)


-- zs - t-1 list of previous measurements								(index address into list)
-- 								(vt, alpha, beta)											(0,1,2)
-- body rates			(p, q, r)															(3,4,5)
-- magnetic flux 	(hx_body, hy_body, hz_body)						(6,7,8)
-- servo commands (throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd) (9,10,11,12)
-- accelerometer 	(axb, ayb, azb)												(13,14,15)
-- gpsPosition    (lat, long, alt) 											(16,17,18)
-- gpsVelocity		(vNorth, vEast, vDown)								(19,20,21)


-- stateObserver OUTPUT:																(index address into list)
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)
--								p_est																	(20)



-- Summary Note: xs is the current measurement,  ys is previous observer output, zs is the k-1 measurement
stateObserver :: [Float] -> [Float] -> [Float] -> [Float]
stateObserver xs [] []  = -- gforce to fps, then into ekf
													gainBias(ekf (gainAcc xs) [] [])
stateObserver xs ys zs = 	gainBias(ekf (gainAcc xs) ys zs)
												
												
-- helper functions for stateObserver 
-- DONE ekf INPUT xs - current time t: 						(index address into list)
-- 								(vt, alpha, beta)											(0,1,2)
-- body rates			(p, q, r)															(3,4,5)
-- magnetic flux 	(hx_body, hy_body, hz_body)						(6,7,8)
-- pilot commands (throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd) (9,10,11,12)
-- accelerometer 	(axb, ayb, azb)												(13,14,15)
-- gpsPosition    (lat, long, alt) 											(16,17,18)
-- gpsVelocity		(vNorth, vEast, vDown)								(19,20,21)


-- ys - previous stateObserver ouput - time t-1					(index address into list)
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)
--								p_est																	(20)


-- zs - t-1 list of previous measurements								(index address into list)
-- 								(vt, alpha, beta)											(0,1,2)
-- body rates			(p, q, r)															(3,4,5)
-- magnetic flux 	(hx_body, hy_body, hz_body)						(6,7,8)
-- servo commands (throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd) (9,10,11,12)
-- accelerometer 	(axb, ayb, azb)												(13,14,15)
-- gpsPosition    (lat, long, alt) 											(16,17,18)
-- gpsVelocity		(vNorth, vEast, vDown)								(19,20,21)


-- ekf OUTPUT:																					(index address into list)
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)
--								p_est																	(20)

ekf :: [Float] -> [Float] -> [Float] -> [Float]	
ekf xs [] [] = (let (beginX, endX) = (splitAt 8 xs)
								in (zeroVector 21))
ekf xs ys zs = (let ((beginX, endX),(beginZ, endZ)) = ((splitAt 9 xs),(splitAt 9 zs))
								in (decomposeEkf beginX (take 7 endZ) ys))
								

-- DONE (** I think)decomposeEkf INPUTS:xs - current time t: 						(index address into list)
-- 								(vt, alpha, beta)											(0,1,2)
-- body rates			(p, q, r)															(3,4,5)
-- magnetic flux 	(hx_body, hy_body, hz_body)						(6,7,8)

-- ys - t-1 list of previous measurements								(index address into list)
-- servo commands (throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd) (0,1,2,3)
-- accelerometer 	(axb, ayb, azb)												(4,5,6)

-- zs - previous stateObserver ouput - time t-1					(index address into list)
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)
--								p_est																	(20)
				
decomposeEkf :: [Float] -> [Float] -> [Float] -> [Float]	
decomposeEkf xs ys zs = (let (one, two, three) = ((y_ xs),(x_est_ ys zs),(k_function ys zs))
												 in zs) 
												 --(map (add (fromList (map fromList (matrixProduct (map toList three) (transpose (map toList (part2 xs (part1 two)))))))) (part1 two )) ++ (drop 15 two))
												 
toList :: Float -> [Float]
toList f = [f]
												 
fromList :: [Float] -> Float
fromList xs = (xs !! 0)							
					 
part1 :: [Float] -> [Float]
part1 xs = (drop 4 (take 18 xs))

part2 :: [Float] -> [Float] -> [Float]
part2 xs ys = vectorMinus xs (y_est_ ys)
										 
-- DONE adding gain to acceleration data to transform to g-force to fps2
gainAcc :: [Float] -> [Float]
gainAcc xs = --split at acc and rebuild
							let (begin,end) = splitAt 12 xs   in begin ++ [(g + (end !! 0))] ++ [g + (end !! 1)] ++ [g + (end !! 2)] ++ (drop 3 end)

-- DONE adding gain to bias elements -- this is done to the final output of ekf
gainBias :: [Float] -> [Float]
gainBias xs = --split at bias and rebuild
							let (begin,end) = splitAt 12 xs   in begin ++ [1 + (end !! 0)] ++ [1 + (end !! 1)] ++ (drop 2 end)
							

-- DONE _y
-- 		INPUT: xs - current time t: 						(index address into list)
-- 								(vt, alpha, beta)											(0,1,2)
-- body rates			(p, q, r)															(3,4,5)
-- magnetic flux 	(hx_body, hy_body, hz_body)						(6,7,8)
-- 		OUTPUT: 										 						(index address into list)
-- 								(vt, alpha, beta)											(0,1,2)
-- body rates			(p, q, r)															(3,4,5)
-- magnetic flux 	(hx_body, hy_body, hz_body)						(6,7,8)
-- this function is just to stay consistant with the matching block in Simulink
y_ :: [Float] -> [Float]
y_ xs = xs 

-- DONE y_est_ 
-- INPUT: output from e_est_
-- OUTPUT: output from h_function
y_est_ :: [Float] -> [Float]
y_est_ xs = (h_function xs)


-- DONE x_est_
-- INPUT: xs - t-1 list of previous measurements		(index address into list)
-- servo commands (throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd) (0,1,2,3)
-- accelerometer 	(axb, ayb, azb)												(4,5,6)

-- ys - previous stateObserver ouput - time t-1					(index address into list)
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)
--								p_est																	(20)

-- OUTPUT:
--fTHL_o,fEL_o,fAIL_o,fRDR_o, (0,1,2,3)
--fVT_o,falpha_o,fbeta_o,			(4,5,6)
--fphi_o,ftheta_o,fpsi_o,			(7,8,9)
--fP_o,fQ_o,fR_o,							(10,11,12)
--falphab_o,fbetab_o,					(13,14)
--accelX_o,accelY_o,accelZ_o	(15,16,17)
--alphaDot, alphadot_o_1,			(18,19)
--p_est												(20)
		
x_est_ :: [Float] -> [Float] -> [Float]
x_est_ xs ys = (gainDt (f_function xs ys) xs)
--(f_function xs ys)
--

-- ** NOT DONE gainDt - this adds dt to the f_x subset of xs and then vector adds the result to xs
--INPUT: 				xs
--fTHL_o,fEL_o,fAIL_o,fRDR_o, (0,1,2,3)
--fVT_o,falpha_o,fbeta_o,			(4,5,6)
--fphi_o,ftheta_o,fpsi_o,			(7,8,9)
--fP_o,fQ_o,fR_o,							(10,11,12)
--falphab_o,fbetab_o,					(13,14)
--accelX_o,accelY_o,accelZ_o	(15,16,17)
--alphaDot, alphadot_o_1,			(18,19)
--p_est												(20)
--							ys
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)
--								p_est																	(20)
--OUTPUT: 
gainDt :: [Float] -> [Float] -> [Float] 
gainDt xs ys = --split at bias and rebuild
							(let ((beginX, endX),(beginY, endY)) = ((splitAt 14 xs),(splitAt 14 ys))
  						 in (vectorSum beginX beginY) ++ endX)

-- k_function
-- **NOT COMPLETE
-- INPUT:	xs - t-1 list of previous measurements								(index address into list)
-- servo commands (throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd) (0,1,2,3)
-- accelerometer 	(axb, ayb, azb)												(4,5,6)

-- ys - previous stateObserver ouput - time t-1					(index address into list)
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)
--								p_est																	(20)
-- OUTPUT: size like: -- OUTPUTS
-- hVT_o,halpha_o,hbeta_o,				(0,1,2)
-- hP_o,hQ_o,hR_o,								(3,4,5)
-- hHxbody_o,hHybody_o,hHzbody_o 	(6,7,8)

k_function :: [Float] -> [Float] -> [Float]

k_function xs [] = xs
k_function xs ys = (zeroVector 9)

-- f_function 
-- *NOTE that inputs for f_function are the same as inputs to k_function
--				these should be computed only once if possible (in ekf?)
-- **NOT COMPLETE

-- INPUT: xs - t-1 list of previous measurements		(index address into list)
-- servo commands (throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd) (0,1,2,3)
-- accelerometer 	(axb, ayb, azb)												(4,5,6)

-- ys - previous stateObserver ouput - time t-1					(index address into list)
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)
--								p_est																	(20)

-- PARAMETERS                  
--				constant rho,...
--				constant S,Cbar,b,weight,g,IxxB,IyyB,IzzB,IxzB,...
--				constant CD0,CDu,CDa,CDq,CDadot,CDde,...
--				constant CD0_bar,...
--				constant Cyb,Cyp,Cyr,Cyda,Cydr,...
--				constant CL0,CLa,CLq,CLadot,CLu,CLde,...
--				constant Clb,Clp,Clr,Clda,Cldr,...
--				constant Cm0,Cma,Cmq,Cmadot,Cmu,Cmde,...
--				constant Cnb,Cnp,Cnr,Cnda,Cndr,...
--				constant xT0,xT1,xT2,...
--				constant Ptrim,Qtrim,Rtrim,Utrim,...
--				constant Athrottle,Bthrottle,...
--				constant Aelevator,Belevator,...
--				constant Aaileron,Baileron,...
--				constant Arudder,Brudder,...
--				constant ThrottleMax,ThrottleMin,...
--				constant ElevatorMax,ElevatorMin,...
--				constant AileronMax,AileronMin,...
--				constant RudderMax,RudderMin)

-- OUTPUT:
--fTHL_o,fEL_o,fAIL_o,fRDR_o, (0,1,2,3)
--fVT_o,falpha_o,fbeta_o,			(4,5,6)
--fphi_o,ftheta_o,fpsi_o,			(7,8,9)
--fP_o,fQ_o,fR_o,							(10,11,12)
--falphab_o,fbetab_o,					(13,14)
--accelX_o,accelY_o,accelZ_o	(15,16,17)
--alphaDot, alphadot_o_1,			(18,19)
--p_est												(20)

-- ** note: this is porting the correct size of output, taken from input values
-- needs common equations module added and used to build up output list composed of 
-- results of functions
f_function :: [Float] -> [Float] -> [Float]
f_function [] [] = []
f_function xs ys = ys

-- DONE h_function
-- INPUTS
--		fVT_o,falpha_o,fbeta_o,			(0,1,2)
--		fphi_o,ftheta_o,fpsi_o,			(3,4,5)
--		fP_o,fQ_o,fR_o,							(6,7,8)
--		falphab_o,fbetab_o,					(9,10)
--		accelX_o,accelY_o,accelZ_o	(11,12,13)

-- OUTPUTS
-- hVT_o,halpha_o,hbeta_o,				(0,1,2)
-- hP_o,hQ_o,hR_o,								(3,4,5)
-- hHxbody_o,hHybody_o,hHzbody_o 	(6,7,8)
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
hHxbody_o xs = (let (hN0noaa, hE0noaa, hD0noaa, theta_o, psi_o) = ((xs !! 6),(xs !! 7),(xs !! 8),(xs !! 4),(xs !! 5))
								in (hN0noaa*cos(theta_o)*cos(psi_o))+(hE0noaa*cos(theta_o)*sin(psi_o))-(hD0noaa*sin(theta_o)))

hHybody_o :: [Float] -> Float								
hHybody_o xs = (let (hN0noaa, hE0noaa, hD0noaa, theta_o, psi_o, phi_o) = ((xs !! 6),(xs !! 7),(xs !! 8),(xs !! 4),(xs !! 5),(xs !! 3))
								in (hN0noaa*(sin(phi_o)*sin(theta_o)*cos(psi_o)-cos(phi_o)*sin(psi_o)))+(hE0noaa*(sin(phi_o)*sin(theta_o)*sin(psi_o)+cos(phi_o)*cos(psi_o)))+(hD0noaa*sin(phi_o)*cos(theta_o)))

hHzbody_o :: [Float] -> Float
hHzbody_o xs = (let (hN0noaa, hE0noaa, hD0noaa, theta_o, psi_o, phi_o) = ((xs !! 6),(xs !! 7),(xs !! 8),(xs !! 4),(xs !! 5),(xs !! 3))
								in (hN0noaa*(cos(phi_o)*sin(theta_o)*cos(psi_o)+sin(phi_o)*sin(psi_o)))+(hE0noaa*(cos(phi_o)*sin(theta_o)*sin(psi_o)-sin(phi_o)*cos(psi_o)))+(hD0noaa*cos(phi_o)*cos(theta_o)))


							
-- PART TWO - NAVIGATION OBSERVER		
-- navigationObserver INPUT xs - current time t: 				(index address into list)
-- 								(vt, alpha, beta)											(0,1,2)
-- body rates			(p, q, r)															(3,4,5)
-- magnetic flux 	(hx_body, hy_body, hz_body)						(6,7,8)
-- pilot commands (throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd) (9,10,11,12)
-- accelerometer 	(axb, ayb, azb)												(13,14,15)
-- gpsPosition    (lat, long, alt) 											(16,17,18)
-- gpsVelocity		(vNorth, vEast, vDown)								(19,20,21)


-- ys - previous stateObserver ouput - time t-1					(index address into list)
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)


-- zs - t-1 list of previous measurements								(index address into list)
-- 								(vt, alpha, beta)											(0,1,2)
-- body rates			(p, q, r)															(3,4,5)
-- magnetic flux 	(hx_body, hy_body, hz_body)						(6,7,8)
-- servo commands (throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd) (9,10,11,12)
-- accelerometer 	(axb, ayb, azb)												(13,14,15)
-- gpsPosition    (lat, long, alt) 											(16,17,18)
-- gpsVelocity		(vNorth, vEast, vDown)								(19,20,21)


-- navigationObserver OUTPUT:
-- lla_est 				(lat_est, long_est, alt_est)					(0,1,2)
-- local_speed_est(vNorth_est,vEast_est,vDown_est)			(3,4,5)
-- wind_est				(wind0,wind1,wind2)										(6,7,8)

					
navigationObserver :: [Float] -> [Float] -> [Float]	-> [Float]						
navigationObserver xs [] [] = (zeroVector 9)
navigationObserver xs ys zs = (zeroVector 9)

