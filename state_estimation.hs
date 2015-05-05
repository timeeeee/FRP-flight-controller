--new state est 
module State_Estimation  
(stateOutput) where  

import Constants
import Matrix_Ops
import State_Functions
import Nav_Functions
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
stateOutput xs ys = [stateObserver xs (rateLimit(ys !! 0)) (ys !! 2),	navigationObserver xs (ys !! 1) (ys !! 2), xs ]


----------------------------------------------------------------------------------
-- PART ONE - STATE OBSERVER

-- ** DONE stateObserver INPUT xs - current time t:(index address into list)
-- 								vt, alpha, beta												(0,1,2)
-- body rates			p, q, r																(3,4,5)
-- magnetic flux 	hx_body, hy_body, hz_body							(6,7,8)
-- pilot commands throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd (9,10,11,12)
-- accelerometer 	axb, ayb, azb													(13,14,15)
-- gpsPosition    lat, long, alt 												(16,17,18)
-- gpsVelocity		vNorth, vEast, vDown									(19,20,21)
-- enable																								(22)

-- ys - previous stateObserver ouput - time t-1					(index address into list)
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)

-- zs - t-1 list of previous measurements			(index address into list)
-- 								vt, alpha, beta												(0,1,2)
-- body rates			p, q, r																(3,4,5)
-- magnetic flux 	hx_body, hy_body, hz_body							(6,7,8)
-- pilot commands throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd (9,10,11,12)
-- accelerometer 	axb, ayb, azb													(13,14,15)
-- gpsPosition    lat, long, alt 												(16,17,18)
-- gpsVelocity		vNorth, vEast, vDown									(19,20,21)
-- enable																								(22)

-- ekf OUTPUT:																		(index address into list)
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)
--[[-4.5309113e11,6.967565e11,5.4941586e17,5.2945036e18,-8.0631745e13,2.5494173e14,-2.6060903e20,8.162134e17,5.1490807e12,-7.6692084e17,-2.5837898e12,6.83236e18,-2.005717e14,-6.831078e13,6.783386e19,1.4865645,-2.5652207e-31,-7.292938e-5,0.0,1.4489848e16],
--[39.253124,-96.93884,529.4541,6.1339475e-2,-0.28544784,0.5771742,0.0,0.0,0.0],
--[42.70626,0.0,0.0,0.201245,-0.9365086,-1.8936161,22.775213,0.5654125,47.41833,0.4974103,1.9278824e-2,3.811761e-3,-2.505436e-3,1.4711692e-2,7.4e-5,-0.98087144,39.253124,-96.93884,529.4541,12.953088,-1.1230818,0.62852925,1.0]]

-- Summary Note: 
-- xs is the current measurement,  
-- ys is previous observer output, 
-- zs is the k-1 measurement
stateObserver :: [Float] -> [Float] -> [Float] -> [Float]
stateObserver xs [] []  = (ekf (gainAcc xs) [] [])
stateObserver xs ys zs = (ekf (gainAcc xs) ys zs)



-- ** DONE adding gain to acceleration data to transform to g-force to fps2
--split at acc and rebuild
gainAcc :: [Float] -> [Float]
gainAcc xs = (take 12 xs) ++ (accelerate (take 3 (drop 12 xs))) ++ (drop 15 xs)
--let (begin,end) = splitAt 13 xs   in begin ++ [(g * (end !! 0))] ++ [g * (end !! 1)] ++ [g * (end !! 2)] ++ (drop 3 end)
accelerate :: [Float] -> [Float]
accelerate xs = [(g * (xs !! 0))] ++ [g * (xs !! 1)] ++ [g * (xs !! 2)]

-- ** DONE adding gain to bias elements -- this is done to the final output of ekf
--split at bias and rebuild
gainBias :: [Float] -> [Float]
gainBias xs = let (begin,end) = splitAt 13 xs   in begin ++ [1 * (end !! 0)] ++ [1 * (end !! 1)] ++ (drop 2 end)



-- ekf OUTPUT:																		(index address into list)
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)
ekf :: [Float] -> [Float] -> [Float] -> [Float]	
-- initial output of ekf for first iteration: NOTE** may want biases and alphadots intitialized to something other than zero **
ekf xs [] [] = (let (vt, alpha, beta, p, q, r, hx_body, hy_body, hz_body, throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd, axb, ayb, azb) = ((xs !! 0), (xs !! 1),(xs !! 2),(xs !! 3),(xs !! 4),(xs !! 5),(xs !! 6),(xs !! 7),(xs !! 8),(xs !! 9),(xs !! 10),(xs !! 11),(xs !! 12),(xs !! 13),(xs !! 14),(xs !! 15))
								in [throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd,vt, alpha, beta,hx_body, hy_body, hz_body,p, q, r,0.0,0.0,axb, ayb, azb,0.0,0.0])
ekf xs ys zs = (let (one, two, three) = ((y_ xs), (x_est_ ys zs), (k_function ys zs))
								in ((vectorSum (reshape(matrixProduct (three) (transpose [(vectorMinus one (y_est_ two))]))) (take 15 two)) ++ (drop 15 two)))


-- y_ INPUT:
-- xs - current time t:(index address into list)
-- 								vt, alpha, beta												(0,1,2)
-- body rates			p, q, r																(3,4,5)
-- magnetic flux 	hx_body, hy_body, hz_body							(6,7,8)
-- pilot commands throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd (9,10,11,12)
-- accelerometer 	axb, ayb, azb													(13,14,15)
-- gpsPosition    lat, long, alt 												(16,17,18)
-- gpsVelocity		vNorth, vEast, vDown									(19,20,21)
-- enable																								(22)								

-- y_ OUTPUT:
-- 								vt, alpha, beta												(0,1,2)
-- body rates			p, q, r																(3,4,5)
-- magnetic flux 	hx_body, hy_body, hz_body							(6,7,8)								
y_ :: [Float] -> [Float]
y_ xs = (take 9 xs)

-- x_est_ INPUT
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

-- OUTPUT:
--								fTHL_o,fEL_o,fAIL_o,fRDR_o, 					(0,1,2,3)
--								fVT_o,falpha_o,fbeta_o,								(4,5,6)
--								fphi_o,ftheta_o,fpsi_o,								(7,8,9)
--								fP_o,fQ_o,fR_o,												(10,11,12)
--								falphab_o,fbetab_o,										(13,14)
--								accelX_o,accelY_o,accelZ_o						(15,16,17)
--								alphaDot, alphaDot1,									(18,19)
x_est_ :: [Float] -> [Float] -> [Float]
x_est_ xs ys = (let (f_out) = (f_function xs (drop 9 ys))
								in (vectorSum (take 9 xs) (gainDt (take 9 f_out))) ++ (drop 9 f_out))

-- multiply the vector with the constant value of dt								
gainDt :: [Float] -> [Float]
gainDt xs = (vectorScalarProduct dt xs)


-- y_est_ INPUT:															
-- 									xs: (output of x_est_)
--								fTHL_o,fEL_o,fAIL_o,fRDR_o, 					(0,1,2,3)
--								fVT_o,falpha_o,fbeta_o,								(4,5,6)
--								fphi_o,ftheta_o,fpsi_o,								(7,8,9)
--								fP_o,fQ_o,fR_o,												(10,11,12)
--								falphab_o,fbetab_o,										(13,14)
--								accelX_o,accelY_o,accelZ_o						(15,16,17)
--								alphaDot, alphaDot1,									(18,19)

-- OUTPUT:
--								hVT_o,halpha_o,hbeta_o,								(0,1,2)
--								hP_o,hQ_o,hR_o,												(3,4,5)
--			          hHxbody_o,hHybody_o,hHzbody_o					(6,7,8)
y_est_ :: [Float] -> [Float]
y_est_ xs = h_function (take 14 (drop 4 xs))

----------------------------------------------------------------------------------							
-- PART TWO - NAVIGATION OBSERVER		
-- navigationObserver INPUT xs - current time t: 				(index address into list)
-- 								vt, alpha, beta												(0,1,2)
-- body rates			p, q, r																(3,4,5)
-- magnetic flux 	hx_body, hy_body, hz_body							(6,7,8)
-- pilot commands throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd (9,10,11,12)
-- accelerometer 	axb, ayb, azb													(13,14,15)
-- gpsPosition    lat, long, alt 												(16,17,18)
-- gpsVelocity		vNorth, vEast, vDown									(19,20,21)
-- enable																								(22)


-- ys - previous navigationObserver OUTPUT:
-- lla_est 				(lat_est, long_est, alt_est)					(0,1,2)
-- local_speed_est(vNorth_est,vEast_est,vDown_est)			(3,4,5)
-- wind_est				(wind0,wind1,wind2)										(6,7,8)


-- zs - t-1 list of previous measurements								(index address into list)
-- 								vt, alpha, beta												(0,1,2)
-- body rates			p, q, r																(3,4,5)
-- magnetic flux 	hx_body, hy_body, hz_body							(6,7,8)
-- pilot commands throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd (9,10,11,12)
-- accelerometer 	axb, ayb, azb													(13,14,15)
-- gpsPosition    lat, long, alt 												(16,17,18)
-- gpsVelocity		vNorth, vEast, vDown									(19,20,21)
-- enable																								(22)


-- navigationObserver OUTPUT:
-- lla_est 				(lat_est, long_est, alt_est)					(0,1,2)
-- local_speed_est(vNorth_est,vEast_est,vDown_est)			(3,4,5)
-- wind_est				(wind0,wind1,wind2)										(6,7,8)

					
navigationObserver :: [Float] -> [Float] -> [Float]	-> [Float]						
navigationObserver xs [] [] = (zeroVector 9)
navigationObserver xs ys zs = (let (one) = (pre_ekf xs)
															 in (post_ekf one (nav_ekf one ys)))






-- pre_ekf INPUT xs - current time t: 				(index address into list)
-- 								vt, alpha, beta												(0,1,2)
-- body rates			p, q, r																(3,4,5)
-- magnetic flux 	hx_body, hy_body, hz_body							(6,7,8)
-- pilot commands throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd (9,10,11,12)
-- accelerometer 	axb, ayb, azb													(13,14,15)
-- gpsPosition    lat, long, alt 												(16,17,18)
-- gpsVelocity		vNorth, vEast, vDown									(19,20,21)
-- enable																								(22)

--OUPUT:
-- 								vt, alpha, beta												(0,1,2)
-- body rates			p, q, r																(3,4,5)
-- magnetic flux 	hx_body, hy_body, hz_body							(6,7,8)
-- pilot commands throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd (9,10,11,12)
-- accelerometer 	(adj axb, ayb, azb)										(13,14,15)
-- localPosition  North, East, Height 												(16,17,18)
-- gpsVelocity		vNorth, vEast, vDown									(19,20,21)
-- enable																								(22)
pre_ekf :: [Float] -> [Float]
pre_ekf xs = (nav_gainAcc (gps_to_local xs))
-- -8.438798e-2,3.827853e-3,-1.221665,39.25282 ,-96.93883,527.7165,12.921134,-3.9597724e-2,-1.4012743,1.0 (measured)
-- -2.7150989  ,0.12315734 ,-39.30585,123006.78,-459844.5,893.3533,12.921134,-3.9597724e-2,-1.4012743,1.0 (after pre_ekf)





-- nav_ekf INPUT: xs 
-- 								vt, alpha, beta												(0,1,2)
-- body rates			p, q, r																(3,4,5)
-- magnetic flux 	hx_body, hy_body, hz_body							(6,7,8)
-- pilot commands throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd (9,10,11,12)
-- accelerometer 	(adj axb, ayb, azb)										(13,14,15)
-- localPosition  North, East, Height 												(16,17,18)
-- gpsVelocity		vNorth, vEast, vDown									(19,20,21)
-- enable																								(22)

-- INPUT: ys (previous nav input)
-- lla_est 				(lat_est, long_est, alt_est)					(0,1,2)
-- local_speed_est(vNorth_est,vEast_est,vDown_est)			(3,4,5)
-- wind_est				(wind0,wind1,wind2)										(6,7,8)

-- OUTPUT: 
-- lla_est 				(lat_est, long_est, alt_est)					(0,1,2)
-- local_speed_est(vNorth_est,vEast_est,vDown_est)			(3,4,5)
-- wind_est				(wind0,wind1,wind2)										(6,7,8)

nav_ekf :: [Float] -> [Float] -> [Float]
nav_ekf xs ys = (let (one, two, three) = ((y_nav (take 6 (drop 16 xs))),(x_est_nav xs ys),(k_nav_funct xs ys))
								 in ((vectorSum (reshape6 (matrixProduct three (transpose [(gps_rate_correction one (vectorMinus one (y_est_nav two)) ys)]))) two) ++ (drop 6 ys)))





-- nav_ekf INPUT: (output from pre_ekf) 
-- 								vt, alpha, beta												(0,1,2)
-- body rates			p, q, r																(3,4,5)
-- magnetic flux 	hx_body, hy_body, hz_body							(6,7,8)
-- pilot commands throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd (9,10,11,12)
-- accelerometer 	(adj axb, ayb, azb)										(13,14,15)
-- localPosition  North, East, Height 												(16,17,18)
-- gpsVelocity		vNorth, vEast, vDown									(19,20,21)
-- enable																								(22)

-- INPUT: ys (output from ekf)
-- lla_est 				(lat_est, long_est, alt_est)					(0,1,2)
-- local_speed_est(vNorth_est,vEast_est,vDown_est)			(3,4,5)
-- wind_est				(wind0,wind1,wind2)										(6,7,8)

-- OUTPUT: 
-- lla_est 				(lat_est, long_est, alt_est)					(0,1,2)
-- local_speed_est(vNorth_est,vEast_est,vDown_est)			(3,4,5)
-- wind_est				(wind0,wind1,wind2)										(6,7,8)

post_ekf :: [Float] -> [Float] -> [Float]
post_ekf xs ys = (local_to_gps xs) ++ (local_speed ys) ++ (true_wind xs ys)

