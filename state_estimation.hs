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
-- 								(vt, alpha, beta)
-- body rates			(p, q, r)
-- magnetic flux 	(hx_body, hy_body, hz_body)
-- servo commands (throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd)
-- accelerometer 	(axb, ayb, azb)

-- ys - previous stateObserver ouput - time t-1
-- (p_est, q_est, r_est, phi_est, theta_est, psi_est,
--					vt_est, alpha_est, beta_est, axb, ayb, azb, 
--					throttle, elevator, aileron, rudder NOTE:aka deflection estimate
--					biases_est are the last TWO items in list)

-- zs - t-1 list of previous measurements
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


-- Note: xs is the current measurement,  ys is previous observer output, zs is the k-1 measurement
stateObserver :: [Float] -> [Float] -> [Float] -> [Float]
stateObserver xs [] []  = -- gforce to fps, then into ekf
											gainBias(ekf (gainAcc xs) [] [])
stateObserver xs ys zs = gainBias(ekf (gainAcc xs) ys zs)
												
												
-- helper functions for stateObserver
ekf :: [Float] -> [Float] -> [Float] -> [Float]	
ekf xs [] [] = (let (beginX, endX) = (splitAt 9 xs)
								in (subElem (y_ beginX) (y_est_ (x_est_ endX []))) ++ (k_function endX []))
ekf xs ys zs = let ((beginX, endX),(beginZ, endZ)) = ((splitAt 9 xs),(splitAt 9 zs))
								in (subElem (y_ beginX) (y_est_ (x_est_ endZ ys))) ++ (k_function endZ ys)
												
-- adding gain to acceleration data to transform to g-force to fps2
gainAcc :: [Float] -> [Float]
gainAcc xs = --split at acc and rebuild
							let (begin,end) = splitAt 9 xs   in begin ++ [(g + (end !! 0))] ++ [g + (end !! 1)] ++ [g + (end !! 2)] ++ (drop 3 end)

-- adding gain to bias elements
gainBias :: [Float] -> [Float]
gainBias xs = --split at bias and rebuild
							let (begin,end) = splitAt 17 xs   in begin ++ [1 + (end !! 0)] ++ [1 + (end !! 1)]
							

-- for function y_ -- This list is already in the order and format that is needed
-- the function y_ is just to stay consistant with the matching block in Simulink
y_ :: [Float] -> [Float]
y_ xs = xs 

y_est_ :: [Float] -> [Float]
y_est_ xs = xs

x_est_ :: [Float] -> [Float] -> [Float]
x_est_ xs [] = xs
x_est_ xs ys = xs

k_function :: [Float] -> [Float] -> [Float]
k_function xs [] = xs
k_function xs ys = xs

--helper functions for ekf
subElem :: [Float] -> [Float] -> [Float]
subElem xs ys = (map sub1 (zip xs ys))

sub1 :: (Float,Float) -> Float
sub1 x = (fst x) - (snd x)


							
-- PART TWO - NAVIGATION OBSERVER		INPUT xs - current time t: 
-- 								(vt, alpha, beta)
-- body rates			(p, q, r)
-- magnetic flux 	(hx_body, hy_body, hz_body)
-- servo commands (throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd)
-- accelerometer 	(axb, ayb, azb)

-- ys - previous navigationObserver ouput - time t-1


-- zs - t-1 list of previous measurements
-- 								(vt, alpha, beta)
-- body rates			(p, q, r)
-- magnetic flux 	(hx_body, hy_body, hz_body)
-- servo commands (throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd)
-- accelerometer 	(axb, ayb, azb)


-- navigationObserver OUTPUT:


					
navigationObserver :: [Float] -> [Float] -> [Float]	-> [Float]						
navigationObserver xs [] [] = ekf xs [] []
navigationObserver xs ys zs = ekf xs ys zs



--((((h (f xs ys)) + (y_ xs)) * k_states) + (f xs ys))

--add result of matrix multiplication to ouput of f for final result
--matrix multiply result of addition to k_states					
--output of h is added to the output of y_ (element wise)
--input of h is output of f

--

