module Matrix_Ops
(Vector
,Matrix
,numRows
,numColumns
,vectorScalarProduct
,matrixScalarProduct
,vectorSum
,matrixSum
,vectorMinus
,matrixMinus
,dotProduct
,matrixProduct
) where 
-- source: http://www2.math.ou.edu/~dmccullough/teaching/f06-6833/haskell/matrix.pdf
import Data.List

type Vector = [Float]
type Matrix = [[Float]] 

--helper functions for ekf

numRows :: Matrix -> Int
numRows = length

numColumns :: Matrix -> Int
numColumns = length . head

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
