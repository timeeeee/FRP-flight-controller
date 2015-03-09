module Main (main) where

import System.IO
import Control.Exception
import Data.List
import Constants
import State_Estimation
import Common_Equations

main :: IO ()
main = do 
       inh <- openFile "input.csv" ReadMode
       outh <- openFile "output.txt" WriteMode
       mainloop inh outh []
       hClose inh
       hClose outh


mainloop :: Handle -> Handle -> [Float] -> IO ()
mainloop inh outh [] = 
    do ineof <- hIsEOF inh
       if ineof
           then return ()
           else do 
								--get samples from file
								inp <- hGetLine inh
								--print to screen the result of getState
								mainloop inh outh (getState (state inp))
mainloop inh outh prev = 
    do ineof <- hIsEOF inh
       if ineof
           then return ()
           else do 
								--get samples from file
								inp <- hGetLine inh
								--print to screen the result of getState
								print prev
								mainloop inh outh (getState (state inp))


--helper functions to parse string from file in mainloop			

--turn the string into a list of floats								 
getState :: [String] -> [Float]
getState xs = map read xs
	    
--turn the string into a list of string items
state :: String -> [String]
state xs = (take 19  (sample xs))
sample xs = 	split ',' xs
						
--split the string						
split :: Eq a => a -> [a] -> [[a]]
split d [] = []
split d s = x : split d (drop 1 y) where (x,y) = span (/= d) s