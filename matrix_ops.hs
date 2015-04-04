module Matrix_Ops
(Vector
,Matrix
,subElem
,addElem
) where 
-- source: http://www2.math.ou.edu/~dmccullough/teaching/f06-6833/haskell/matrix.pdf

type Vector = [Float]
type Matrix = [[Float]] 

--helper functions for ekf
-- DONE
subElem :: Vector -> Vector -> Vector
subElem xs ys = (map sub1 (zip xs ys))

-- DONE
sub1 :: (Float,Float) -> Float
sub1 x = (fst x) - (snd x)

-- DONE
addElem :: Vector -> Vector -> Vector
addElem xs ys = (map add1 (zip xs ys))

-- DONE
add1 :: (Float,Float) -> Float
add1 x = (fst x) + (snd x)



vectorScalarMultiply :: Vector -> Vector 

matrixScalarMultiply :: Matrix -> Matrix

matrixMultiply :: Matrix -> Matrix
