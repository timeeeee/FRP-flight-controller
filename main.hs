module Main (main) where

import System.IO
import Control.Exception
import Data.List
import Constants
import State_Estimation
--import Common_Equations

main :: IO ()
main = do 
       inh <- openFile "input.csv" ReadMode
       outh <- openFile "output.txt" WriteMode
       mainloop inh outh []
       hClose inh
       hClose outh


mainloop :: Handle -> Handle -> [[Float]] -> IO ()
mainloop inh outh [] = 
    do ineof <- hIsEOF inh
       if ineof
           then return ()
           else do 
								--get samples from file
								inp <- hGetLine inh
								mainloop inh outh (getState (state inp) [])
mainloop inh outh prev = 
    do ineof <- hIsEOF inh
       if ineof
           then return ()
           else do 
								--get samples from file
								inp <- hGetLine inh
								--print to screen (and file) the result of t-1 getState
								print prev
								hPrint outh prev
								mainloop inh outh (getState (state inp) prev)


--helper functions to parse string from file in mainloop			

--turn the string into a list of floats								 
getState :: [String] -> [[Float]] -> [[Float]]
getState xs [] = stateOutput (map read xs) []
getState xs ys = stateOutput (map read xs) ys
	    
--turn the string into a list of string items
state :: String -> [String]
state xs = (take 22  (sample xs))

--splitting the string at comma
sample :: String -> [String]
sample xs = 	split ',' xs
						
--split the string						
split :: Eq a => a -> [a] -> [[a]]
split d [] = []
split d s = x : split d (drop 1 y) where (x,y) = span (/= d) s