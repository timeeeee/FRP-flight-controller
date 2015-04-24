--new state est 
module State_Estimation  
(stateOutput) where  

import Constants
import Matrix_Ops
import State_Functions
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
stateObserver xs [] []  = gainBias(ekf (gainAcc xs) [] [])
stateObserver xs ys zs = gainBias(ekf (gainAcc xs) ys zs)

-- ** DONE adding gain to acceleration data to transform to g-force to fps2
--split at acc and rebuild
gainAcc :: [Float] -> [Float]
gainAcc xs = let (begin,end) = splitAt 12 xs   in begin ++ [(g * (end !! 0))] ++ [g * (end !! 1)] ++ [g * (end !! 2)] ++ (drop 3 end)

-- ** DONE adding gain to bias elements -- this is done to the final output of ekf
--split at bias and rebuild
gainBias :: [Float] -> [Float]
gainBias xs = let (begin,end) = splitAt 12 xs   in begin ++ [1 * (end !! 0)] ++ [1 * (end !! 1)] ++ (drop 2 end)



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
								in [throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd,vt, alpha, beta,hx_body, hy_body, hz_body,p, q, r,0.0,0.0,axb, ayb, azb,0.0,0.0])
ekf xs ys zs = (let (one, two, three) = ((y_ xs),(x_est_ ys zs),(k_function ys zs))
								in (reshape(two ++ [111111.0] ++ two  ++ [0.0] ++ (three !! 0))
								--decomposeEkf one two three (xs !! 22))

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


-- decomposeEkf INPUT xs (output of y_):
-- 								vt, alpha, beta												(0,1,2)
-- body rates			p, q, r																(3,4,5)
-- magnetic flux 	hx_body, hy_body, hz_body							(6,7,8)	
	
-- 										ys: (output of x_est_)
--								fTHL_o,fEL_o,fAIL_o,fRDR_o, 					(0,1,2,3)
--								fVT_o,falpha_o,fbeta_o,								(4,5,6)
--								fphi_o,ftheta_o,fpsi_o,								(7,8,9)
--								fP_o,fQ_o,fR_o,												(10,11,12)
--								falphab_o,fbetab_o,										(13,14)
--								accelX_o,accelY_o,accelZ_o						(15,16,17)
--								alphaDot, alphaDot1,									(18,19)

--										zs: (output of k_function) - 15 x 11 matrix
-- enable value
-- decomposeEkf OUTPUT:																		(index address into list)
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)
decomposeEkf :: [Float] -> [Float] -> [[Float]] -> Float -> [Float]
decomposeEkf xs ys zs enab = (let (one,two) = ((matrixScalarProduct enab zs),(vectorMinus xs (y_est_ ys)))
															in  decompose2ekf one two ys)
															
-- decompose2ekf OUTPUT:																		(index address into list)
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)															
decompose2ekf :: [[Float]] -> [Float] -> [Float] -> [Float]														
decompose2ekf xs ys zs = zs
--toVector [xs] ++ toVector (matrixProduct [xs] (transpose [ys])) ++ toVector [zs] ++ zs ++ ys
															-- **COMPLETE AFTER K_FUNCTION IS KNOWN toVector(matrixSum (matrixProduct [one] (transpose [two])) (transpose [(take 9 ys)] )) ++ (drop 9 ys) )
															

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

								
								