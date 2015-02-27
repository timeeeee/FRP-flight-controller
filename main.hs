module Main (main) where

import System.IO
import Control.Exception
import Data.List
import Constants
import State_Estimation

main :: IO ()
main = do 
       inh <- openFile "input.csv" ReadMode
       outh <- openFile "output.txt" WriteMode
       mainloop inh outh
       hClose inh
       hClose outh

mainloop :: Handle -> Handle -> IO ()
mainloop inh outh = 
    do ineof <- hIsEOF inh
       if ineof
           then return ()
           else do 
								inp <- hGetLine inh
								print (getState (state inp))
								mainloop inh outh


									 
									 


--helper functions to parse string from file in mainloop											 
getState :: [String] -> [Float]
getState xs = let [phi,theta, psi, p, q, r, lat, lon, alt, vnorth, veast, vdown, rc_throttle, rc_ele, rc_ail, rc_rudder, vt, alpha, beta] = map read xs
	    in stateOutput [phi,theta, psi, p, q, r, lat, lon, alt, vnorth, veast, vdown, rc_throttle, rc_ele, rc_ail, rc_rudder, vt, alpha, beta]

state :: String -> [String]
state xs = (take 19  (sample xs))
sample xs = 	split ',' xs
						
split :: Eq a => a -> [a] -> [[a]]
split d [] = []
split d s = x : split d (drop 1 y) where (x,y) = span (/= d) s