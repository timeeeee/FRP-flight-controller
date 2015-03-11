module State_Estimation  
(stateOutput) where  

import Constants
--import Common_Equations


stateOutput :: [Float] -> [[Float]] -> [[Float]]
stateOutput xs [] = [stateObserver xs [],	navigationObserver xs []]
stateOutput xs ys = [stateObserver xs (ys !! 0),	navigationObserver xs (ys !! 1)]


-- stateObserver INPUT: 
-- 								(vt, alpha, beta)
-- body rates			(p, q, r)
-- magnetic flux 	(hx_body, hy_body, hz_body)
-- servo commands (throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd)
-- accelerometer 	(axb, ayb, azb)

-- stateObserver OUTPUT:
-- (p_est, q_est, r_est, phi_est, theta_est, psi_est,
--					vt_est, alpha_est, beta_est, axb, ayb, azb, 
--					throttle, elevator, aileron, rudder NOTE:aka deflection estimate
--					biases_est NOTE:last TWO items in list)


-- PART ONE - STATE OBSERVER
stateObserver :: [Float] -> [Float] -> [Float]
stateObserver xs [] = -- gforce to fps, then into ekf
											gainBias(ekf (gainAcc xs) [])
stateObserver xs ys = gainBias(ekf (gainAcc xs) ys)
												
												
-- helper functions for stateObserver
ekf :: [Float] -> [Float] -> [Float]	
ekf xs [] = xs	
ekf xs ys = xs

gainAcc :: [Float] -> [Float]
gainAcc xs = --split at acc and rebuild
							let (begin,end) = splitAt 9 xs   in begin ++ [(g + (end !! 0))] ++ [g + (end !! 1)] ++ [g + (end !! 2)] ++ (drop 3 end)

gainBias :: [Float] -> [Float]
gainBias xs = --split at acc and rebuild
							let (begin,end) = splitAt 17 xs   in begin ++ [1 + (end !! 0)] ++ [1 + (end !! 1)]
							

							
-- PART TWO - NAVIGATION OBSERVER							
navigationObserver :: [Float] -> [Float] -> [Float]								
navigationObserver xs [] = ekf xs []
navigationObserver xs ys = ekf xs ys



--((((h (f xs ys)) + (y_ xs)) * k_states) + (f xs ys))

--add result of matrix multiplication to ouput of f for final result
--matrix multiply result of addition to k_states					
--output of h is added to the output of y_ (element wise)
--input of h is output of f

--
y_ xs = xs 

f xs ys = xs

h xs = xs