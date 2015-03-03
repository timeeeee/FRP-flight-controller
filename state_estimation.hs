module State_Estimation  
(stateOutput) where  

import Common_Equations



--Engine Constants
stateOutput :: [Float] -> [Float] -> [Float]
stateOutput xs [] = []
stateOutput xs ys = ys
										
