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
<<<<<<< HEAD
stateOutput xs ys = [stateObserver xs (rateLimit(ys !! 0)) (ys !! 2),	
										 navigationObserver xs (ys !! 1) (ys !! 2), 
										 xs]
=======
stateOutput xs ys = [stateObserver xs (ys !! 0) (ys !! 2),	navigationObserver xs (ys !! 1) (ys !! 2), xs ]
>>>>>>> parent of bd026c8... state_observer is finished


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


-- Summary Note: 
-- xs is the current measurement,  
-- ys is previous observer output, 
-- zs is the k-1 measurement
stateObserver :: [Float] -> [Float] -> [Float] -> [Float]
<<<<<<< HEAD
stateObserver xs [] []  = (ekf (gainAcc xs) [] [])
stateObserver xs ys zs = (ekf (gainAcc xs) ys zs)

=======
stateObserver xs [] []  = gainBias(ekf (gainAcc xs) [] [])
stateObserver xs ys zs = gainBias(ekf (gainAcc xs) ys zs)
>>>>>>> parent of bd026c8... state_observer is finished

-- ** DONE adding gain to acceleration data to transform to g-force to fps2
--split at acc and rebuild
gainAcc :: [Float] -> [Float]
gainAcc xs = (take 13 xs) ++ (accelerate (take 3 (drop 13 xs))) ++ (drop 16 xs)

<<<<<<< HEAD
accelerate :: [Float] -> [Float]
accelerate xs = [(g * (xs !! 0))] ++ [g * (xs !! 1)] ++ [g * (xs !! 2)]

=======
>>>>>>> parent of bd026c8... state_observer is finished
-- ** DONE adding gain to bias elements -- this is done to the final output of ekf
--split at bias and rebuild
gainBias :: [Float] -> [Float]
gainBias xs = (let (begin,end) = splitAt 13 xs   
							 in begin ++ [1 * (end !! 0)] ++ [1 * (end !! 1)] ++ (drop 2 end))



-- ekf OUTPUT:																		(index address into list)
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)
ekf :: [Float] -> [Float] -> [Float] -> [Float]	
ekf xs [] [] = (let (vt, alpha, beta, p, q, r, hx_body, hy_body, hz_body, throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd, axb, ayb, azb) = ((xs !! 0), (xs !! 1),(xs !! 2),(xs !! 3),(xs !! 4),(xs !! 5),(xs !! 6),(xs !! 7),(xs !! 8),(xs !! 9),(xs !! 10),(xs !! 11),(xs !! 12),(xs !! 13),(xs !! 14),(xs !! 15))
<<<<<<< HEAD
								in [throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd,
										vt, alpha, beta,
										hx_body, hy_body, hz_body,
										p, q, r,
										0.0,0.0,
										axb, ayb, azb,
										0.0,0.0])
ekf xs ys zs = (let (one, two, three) = ((y_ xs), (x_est_ ys zs), (k_function ys zs))
								in ((rateLimit two) ++ (rateLimit (vectorSum (reshape (matrixProduct three (transpose [vectorMinus (take 9 xs) (y_est_ two)]))) (take 15 two) ) ++ (drop 15 two))))
								--vectorSum (reshape(matrixProduct (three) (transpose [(vectorMinus one (y_est_ two))]))) (take 15 two)) ++ (drop 15 two) ++ one)
--sample of two
--  0.5182963,1.931924e-2,	3.8875504e-3,-3.188739e-3,466.1799, -0.9963662,1.1359025e8,-7.5850997, 6.5481033,-45.30768, -31717.617, 2511.1797,-31717.617,0.0,      0.0,         97.349396,2113.705, 1373.538,0.0, -43.90859
--[0.21582651,1.3289034e-2,-1.0241674e-2,4.8055653e-3,200.0,1.064609,1.5707964,3.1415927,-11.871361,-1.5707964,15.707963,-10.471976,200.0,0.0,0.0,3768.0417,2853.4448,-6984.7983,0.0,24.425413

--sample of combo
-- -6.0162454,2.0107422,	 -210980.88,   -2045073.5,   2369.7786,-10.876247,1.0066524e8,-305915.88,-646.99725, 323598.28,-31746.654,-2636696.3,-31865.545,10.358159,-2.6202062e7,97.349396,2113.705, 1373.538,0.0, -43.90859
-- sample of rateLimited
-- 0.15,			0.2618,				-0.3491,		-0.1745,200.0,-1.5707964,1.5707964,-3.1415927,-3.1415927,1.5707964,-15.707963,-10.471976,-200.0,0.5235988,-0.5235988,8310.7705,2852.8403,7253.87,0.0,32.1303

=======
								in [throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd,vt, alpha, beta,hx_body, hy_body, hz_body,p, q, r,0.0,0.0,axb, ayb, azb,0.0,0.0])
ekf xs ys zs = (let (one, two, three) = ((y_ xs),(x_est_ ys zs),(k_function ys zs))
								in (one ++ two ++ (reshape three) ++ (matrixSize [one]) ++ (matrixSize [two]) ++ (matrixSize (matrixProduct (three) (transpose [vectorMinus one (y_est_ (take 15 two))]) ) ) )) 
								
								--( (reshape (matrixSum (matrixProduct (three) (transpose [vectorMinus one (y_est_ (take 9 two))]) ) (transpose [take 9 two]) )) ++  xs ) )
								
								
								-- scrap ((reshape(transpose [one])) ++ two ++ (reshape three ) ++ one ++ (matrixSize (transpose [one])) ++ (matrixSize [two]) ++ (matrixSize three)))
								--(
								--note: [one] is 1x9, [two] is 1x22, three is 15x11
>>>>>>> parent of bd026c8... state_observer is finished

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
gainDt xs = vectorScalarProduct dt xs


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
nav_ekf xs ys = xs





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

