module State_Estimation  
(stateOutput) where  

import Constants
--import Common_Equations


stateOutput :: [Float] -> [[Float]] -> [[Float]]
stateOutput xs [] = [stateObserver xs [] [],	navigationObserver xs [] [], xs]
stateOutput xs ys = [stateObserver xs (ys !! 0) (ys !! 2),	navigationObserver xs (ys !! 1) (ys !! 2), xs ]


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
-- Note: xs is the current measurement,  ys is previous observer output, zs is the k-1 measurement
stateObserver :: [Float] -> [Float] -> [Float] -> [Float]
stateObserver xs [] []  = -- gforce to fps, then into ekf
											gainBias(ekf (gainAcc xs) [] [])
stateObserver xs ys zs = gainBias(ekf (gainAcc xs) ys zs)
												
												
-- helper functions for stateObserver
-- helper functions for stateObserver
ekf :: [Float] -> [Float] -> [Float] -> [Float]	
ekf xs [] [] = (let (beginX, endX) = (splitAt 9 xs)
								in (y_ beginX) ++ (x_est_ endX []) ++ (k_function endX []))
ekf xs ys zs = let ((beginX, endX),(beginZ, endZ)) = ((splitAt 9 xs),(splitAt 9 zs))
								in (subElem (y_ beginX) (y_est_ (x_est_ endZ ys))) ++ (k_function endZ ys)
						


subElem :: [Float] -> [Float] -> [Float]
subElem xs ys = (map sub1 (zip xs ys))

sub1 :: (Float,Float) -> Float
sub1 x = (fst x) - (snd x)
						
gainAcc :: [Float] -> [Float]
gainAcc xs = --split at acc and rebuild
							let (begin,end) = splitAt 9 xs   in begin ++ [(g + (end !! 0))] ++ [g + (end !! 1)] ++ [g + (end !! 2)] ++ (drop 3 end)

gainBias :: [Float] -> [Float]
gainBias xs = --split at bias and rebuild
							let (begin,end) = splitAt 17 xs   in begin ++ [1 + (end !! 0)] ++ [1 + (end !! 1)]
							
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
							
-- PART TWO - NAVIGATION OBSERVER							
navigationObserver :: [Float] -> [Float] -> [Float]	-> [Float]						
navigationObserver xs [] [] = ekf xs [] []
navigationObserver xs ys zs = ekf xs ys zs



--((((h (f xs ys)) + (y_ xs)) * k_states) + (f xs ys))

--add result of matrix multiplication to ouput of f for final result
--matrix multiply result of addition to k_states					
--output of h is added to the output of y_ (element wise)
--input of h is output of f

--

f :: [Float] -> [Float] -> Float -> [Float]
f xs ys alphaDot_1 = xs

h xs = xs