module State_Estimation  
(stateOutput) where  

import Constants
--import Common_Equations


stateOutput :: [Float] -> [Float] -> [[Float]]
stateOutput xs [] = [stateObserver xs [],	navigationObserver xs []]
stateOutput xs ys = [stateObserver xs ys,	navigationObserver xs ys]

stateObserver :: [Float] -> [Float] -> [Float]
stateObserver xs [] = ekf xs []
stateObserver xs ys = ekf xs ys
												
navigationObserver :: [Float] -> [Float] -> [Float]								
navigationObserver xs [] = ekf xs []
navigationObserver xs ys = ekf xs ys

ekf :: [Float] -> [Float] -> [Float]	
ekf xs [] = xs	
ekf xs ys = xs

--((((h (f xs ys)) + (y_ xs)) * k_states) + (f xs ys))

--add result of matrix multiplication to ouput of f for final result
--matrix multiply result of addition to k_states					
--output of h is added to the output of y_ (element wise)
--input of h is output of f

--
y_ xs = xs 

f xs ys = xs

h xs = xs