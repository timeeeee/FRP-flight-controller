module State_Estimation  
(stateOutput) where  

import Constants
import Matrix_Ops

--import Common_Equations

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
-- stateObserver INPUT xs - current time t: 
-- 								(vt, alpha, beta)											(0,1,2)
-- body rates			(p, q, r)															(3,4,5)
-- magnetic flux 	(hx_body, hy_body, hz_body)						(6,7,8)
-- pilot commands (throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd) (9,10,11,12)
-- accelerometer 	(axb, ayb, azb)												(13,14,15)
-- gpsPosition    (lat, long, alt) 											(16,17,18)
-- gpsVelocity		(vNorth, vEast, vDown)								(19,20,21)
-- waypoints			size?

-- ys - previous stateObserver ouput - time t-1
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)


-- zs - t-1 list of previous measurements
-- 								(vt, alpha, beta)											(0,1,2)
-- body rates			(p, q, r)															(3,4,5)
-- magnetic flux 	(hx_body, hy_body, hz_body)						(6,7,8)
-- servo commands (throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd) (9,10,11,12)
-- accelerometer 	(axb, ayb, azb)												(13,14,15)
-- gpsPosition    (lat, long, alt) 											(16,17,18)
-- gpsVelocity		(vNorth, vEast, vDown)								(19,20,21)
-- waypoints			size?

-- stateObserver OUTPUT:
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)


-- DONE
-- Summary Note: xs is the current measurement,  ys is previous observer output, zs is the k-1 measurement
stateObserver :: [Float] -> [Float] -> [Float] -> [Float]
stateObserver xs [] []  = -- gforce to fps, then into ekf
											gainBias(ekf (gainAcc xs) [] [])
stateObserver xs ys zs = gainBias(ekf (gainAcc xs) ys zs)
												
												
-- helper functions for stateObserver 
-- ekf
-- **NOT COMPLETE
ekf :: [Float] -> [Float] -> [Float] -> [Float]	
ekf xs [] [] = (let (beginX, endX) = (splitAt 9 xs)
								in (subElem (y_ beginX) (y_est_ (x_est_ endX []))) ++ (k_function endX []))
ekf xs ys zs = let ((beginX, endX),(beginZ, endZ)) = ((splitAt 9 xs),(splitAt 9 zs))
								in (subElem (y_ beginX) (y_est_ (x_est_ endZ ys))) ++ (k_function endZ ys)
												
-- DONE adding gain to acceleration data to transform to g-force to fps2
gainAcc :: [Float] -> [Float]
gainAcc xs = --split at acc and rebuild
							let (begin,end) = splitAt 12 xs   in begin ++ [(g + (end !! 0))] ++ [g + (end !! 1)] ++ [g + (end !! 2)] ++ (drop 3 end)

-- DONE adding gain to bias elements
gainBias :: [Float] -> [Float]
gainBias xs = --split at bias and rebuild
							let (begin,end) = splitAt 13 xs   in begin ++ [1 + (end !! 0)] ++ [1 + (end !! 1)] ++ (drop 2 end)
							

-- DONE for function y_ -- This list is already in the order and format that is needed
-- the function y_ is just to stay consistant with the matching block in Simulink
y_ :: [Float] -> [Float]
y_ xs = xs 

-- DONE y_est_ 
-- INPUT: output from e_est_
-- OUTPUT: output from h_function
y_est_ :: [Float] -> [Float]
y_est_ xs = (h_function xs)

-- x_est_
-- **NOT COMPLETE
-- INPUT: xs - (t-1 state observer ouput) and 
--				ys - (t-1 measurements)
--OUTPUT: adjusted return value of f_function
x_est_ :: [Float] -> [Float] -> [Float]
x_est_ xs [] = xs
x_est_ xs ys = (f_function xs ys)

-- k_function
-- **NOT COMPLETE
-- INPUT: xs - return value of x_est_ of t-1
--				ys - return value of k_function of t-1
-- OUTPUT: 
k_function :: [Float] -> [Float] -> [Float]
k_function xs [] = xs
k_function xs ys = xs ++ ys

-- f_function 
-- *NOTE that inputs for f_function are the same as inputs to k_function
--				these should be computed only once if possible (in ekf?)
-- **NOT COMPLETE
-- INPUT:	xs - return value of x_est_ of t-1
--				ys - return value of k_function of t-1
-- OUTPUT:
f_function :: [Float] -> [Float] -> [Float]
f_function [] [] = []
f_function xs ys = xs ++ ys

-- **NOT COMPLETE
h_function :: [Float] -> [Float]
h_function xs = xs

--helper functions for ekf
-- DONE
subElem :: [Float] -> [Float] -> [Float]
subElem xs ys = (map sub1 (zip xs ys))

-- DONE
sub1 :: (Float,Float) -> Float
sub1 x = (fst x) - (snd x)

-- DONE
addElem :: [Float] -> [Float] -> [Float]
addElem xs ys = (map sub1 (zip xs ys))

-- DONE
add1 :: (Float,Float) -> Float
add1 x = (fst x) + (snd x)

							
-- PART TWO - NAVIGATION OBSERVER		
-- navigationObserver INPUT xs - current time t: 
-- 								(vt, alpha, beta)											(0,1,2)
-- body rates			(p, q, r)															(3,4,5)
-- magnetic flux 	(hx_body, hy_body, hz_body)						(6,7,8)
-- pilot commands (throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd) (9,10,11,12)
-- accelerometer 	(axb, ayb, azb)												(13,14,15)
-- gpsPosition    (lat, long, alt) 											(16,17,18)
-- gpsVelocity		(vNorth, vEast, vDown)								(19,20,21)
-- waypoints			size?

-- ys - navigationObserver ouput - time t-1
-- lla_est 				(lat_est, long_est, alt_est)					(0,1,2)
-- local_speed_est(vNorth_est,vEast_est,vDown_est)			(3,4,5)
-- wind_est				(wind0,wind1,wind2)										(6,7,8)

-- zs - measured values time t-1: 
-- 								(vt, alpha, beta)											(0,1,2)
-- body rates			(p, q, r)															(3,4,5)
-- magnetic flux 	(hx_body, hy_body, hz_body)						(6,7,8)
-- pilot commands (throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd) (9,10,11,12)
-- accelerometer 	(axb, ayb, azb)												(13,14,15)
-- gpsPosition    (lat, long, alt) 											(16,17,18)
-- gpsVelocity		(vNorth, vEast, vDown)								(19,20,21)
-- waypoints			size?


-- navigationObserver OUTPUT:
-- lla_est 				(lat_est, long_est, alt_est)					(0,1,2)
-- local_speed_est(vNorth_est,vEast_est,vDown_est)			(3,4,5)
-- wind_est				(wind0,wind1,wind2)										(6,7,8)

					
navigationObserver :: [Float] -> [Float] -> [Float]	-> [Float]						
navigationObserver xs [] [] = ekf xs [] []
navigationObserver xs ys zs = ekf xs ys zs

