module State_Estimation  
(stateOutput) where  



stateOutput :: [Float] -> [Float] -> [[Float]]
stateOutput xs [] = [stateObserver xs [],	navigationObserver xs []]
stateOutput xs ys = [stateObserver xs ys,	navigationObserver xs ys]
											 
stateObserver xs ys = ekf xs ys
												
navigationObserver xs ys = ekf xs ys

ekf xs ys = xs			

--add result of matrix multiplication to ouput of f for final result
--matrix multiply result of addition to k_states					
--output of h is added to the output of y_ (element wise)
--input of h is output of f



f xs ys = xs

h xs ys = xs