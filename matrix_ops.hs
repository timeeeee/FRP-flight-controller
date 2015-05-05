module Matrix_Ops
(zeroVector
,vectorScalarProduct
,matrixScalarProduct
,vectorSum
,matrixSum
,vectorMinus
,matrixMinus
,matrixProduct
,matrixSize
,reshape
,reshape6
) where 
-- source: http://www2.math.ou.edu/~dmccullough/teaching/f06-6833/haskell/matrix.pdf
import Data.List
import GHC.Float
type Vector = [Float]
type Matrix = [[Float]] 

--helper functions for ekf

numRows :: Matrix -> Int
numRows = length

numColumns :: Matrix -> Int
numColumns = length . head

zeroVector :: Int -> Vector
zeroVector n = replicate n 0.0

vectorScalarProduct :: Float -> Vector -> Vector
vectorScalarProduct n vec = [ n * x | x <- vec ]

matrixScalarProduct :: Float -> Matrix -> Matrix
matrixScalarProduct n m = [ vectorScalarProduct n row | row <- m ]

vectorSum :: Vector -> Vector -> Vector
vectorSum = zipWith (+)

matrixSum :: Matrix -> Matrix -> Matrix
matrixSum = zipWith vectorSum

vectorMinus :: Vector -> Vector -> Vector
vectorMinus = zipWith (-)

matrixMinus :: Matrix -> Matrix -> Matrix
matrixMinus = zipWith vectorMinus

dotProduct :: Vector -> Vector -> Float
dotProduct v w = sum ( zipWith (*) v w )

matrixProduct :: Matrix -> Matrix -> Matrix
matrixProduct m n = [ map (dotProduct row) (transpose n) | row <- m ]

matrixSize :: Matrix -> Vector
matrixSize m = [int2Float (numRows m)] ++ [int2Float (numColumns m)]

reshape :: Matrix -> Vector
reshape m = (m !! 0) ++ (m !! 1) ++ (m !! 2) ++ (m !! 3) ++ (m !! 4) ++ (m !! 5) ++ (m !! 6) ++ (m !! 7) ++ (m !! 8) ++ (m !! 9) ++ (m !! 10) ++ (m !! 11) ++ (m !! 12) ++ (m !! 13) ++ (m !! 14)

reshape6 :: Matrix -> Vector
reshape6 m = (m !! 0) ++ (m !! 1) ++ (m !! 2) ++ (m !! 3) ++ (m !! 4) ++ (m !! 5)