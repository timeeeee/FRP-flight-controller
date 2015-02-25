-- FRP System Identification
-- EECS_582 2015
-- Angie Wright & Tim Clark

-- State Observer
-----------------INPUT:
--vt, alpha, beta(from pressure sensor), 
--euler angles ins(from vector nav), 
--body rates ins(from vector nav), 
--gyroscope meas(from vector nav), 
--pilot commands (from rc), 
--accelerometer meas(from vector nav), 
--enable/Disable (pic/cic from rc)

-----------------OUTPUT:
--body rages est
--euler angles est
--vt, alpha beta est
--accelerometer est
--se term

------------------NOTE:
-- if the constant value name is different than the new_vars name 
-- the new vars name is commented, these need verified that they are correct
-- matchings

module Main where
 
import System.IO
import Control.Exception
import Data.List


--Engine Constants
xT0 = 0
xT1 = 0.0142
xT2	= 0

--moment constants
--IyyB
iyyB = 0.1206
--IxxB
ixxB = 0.0301
--IzzB
izzB = 0.0725
--IxzB
ixzB = 0.0

-- ARGUMENT FOR THE FORCE COEFFICIENTS 
--CDO
cDo = 0.0445
--CDA
cDalpha = 0.0073
--CDde
cDdeltaE = 0.0085
--CDadot
cDalphaDot = 0
--CDq
cDq = 0
--CDu
cDu = 0

--CLO
cLo = 0.2793
--CLa
cLalpha = 5.0137
--CLde
cLdeltaE = 0.4409
--CLadot
cLalphaDot = 0.7680
--CLq
cLq = 3.9215
--CLu
cLu = 4.2900

--CmO
cmo = 0.0276
--Cma
cmalpha = -0.3528
--Cmde
cmdeltaE = -1.4810
--Cmadot
cmalphaDot = -2.5798
--Cmq
cmq = -10.3561
--Cmu
cmu = 0

--Cyb
cYbeta = -0.0985
--Cyda
cYdeltaA = 0
--Cydr
cYdeltaR = 0.0797
--cYbetaDot ??????????
--Cyp
cYp = 5.8400

--Clb
clbeta = 0.0181
--Clda
cldeltaA = 0.1368
--Cldr
cldeltaR = 0.0017
--clbetaDot ??????????????
--Clp
clp = -0.4384

--Cnb
cnbeta = 0.0213
--Cnda
cndeltaA = -0.0057
--Cndr
cndeltaR = -0.0223
--cnbetaDot ???????????????
--Cnp
cnp = -0.0459

-- WING REFERENCE AREA
--S
s = 7.4300

-- WING SPAN
b = 10.7300

-- MEAN GEOMETRIC CORD
--Cbar
cBar = 0.7900

-- TRIMMED ANGLE OF ATTACK
alphaTrim = -0.0060
betaTrim = 0
--deltaETrim ???????????
--deltaATrim ??????????
--deltaRTrim ????????????
--uTrim      ????????????

-- WEIGHT
weight = 4.6100

-- GRAVITY APPROXIMATION
g = 32.1740

-- DYNAMIC PRESSURE 
rho = 0.0023

-- CONSTANT STATES KALMAN GAIN 
k_states = []


--this is the storage for EKF function
so_prev_meas = []
so_prev_x_est_ = []

no_prev_meas = []
no_prev_x_est_ = []

--Engine Function
xt :: Float -> Float
xt throttle = ((((throttle * 100)*(throttle * 100)) * xT2) + (xT1 * (throttle*100)) + (xT0))




--moment functions
c1 :: Float -> Float
c1 delta = ((((iyyB + izzB) * izzB) + (ixzB * ixzB)) / delta)

c2 :: Float -> Float
c2 delta = ((((ixxB - iyyB) + izzB) * ixzB) / delta)

c3 :: Float -> Float
c3 delta = (izzB / delta)

c4 :: Float -> Float
c4 delta = (ixzB / delta)

c5 :: Float
c5 = ((izzB - ixxB) / iyyB)

c6 :: Float
c6 = (ixzB / iyyB)

c7 :: Float
c7 = (1 / iyyB)

c8 :: Float -> Float
c8 delta = ((((ixxB - iyyB) * ixxB) + (ixzB * ixzB)) / delta)

c9 :: Float
c9 = (ixxB / delta)

delta :: Float
delta = ((ixxB * izzB) - (ixzB * ixzB))

momentConstants = [(c1 delta) , (c2 delta) , (c3 delta) , (c4 delta) , c5 , c6 , c7 , (c8 delta) , c9 , delta]


-- BODY VELOCITY COMPONENTS 
-- these require vt alpha and beta
u vt alpha beta = (vt * (cos alpha) * (cos beta))
v vt beta = (vt * (sin beta))
w vt alpha beta = (vt * (sin alpha) * (cos beta))

-- BODY ACCELERATION COMPONENTS
-- these require u v w, p q r, gravity, phi theta, xt(engine thrust) xa ya za(aerodynamic forces)


--mass
m weight g = (weight/g)

uDot v w q r g theta xt xa m = (((r*v) - (q*w) - (g * (sin theta)) + (xt + xa))/m)
vDot u w p r g phi theta ya m = ((-(r*u) + (p*w) + (g*(sin phi)*(cos theta)) + ya)/m)
wDot u v p q g phi theta za m = (((q*u) - (p*v) + (g*(cos phi)*(cos theta)) + za)/m)




-- if alpha is > alphaTrim delta is positive, else delta is neg (except 0 or course)
deltaAlpha alpha alphaTrim = (alpha - alphaTrim)
deltaBeta beta betaTrim = (beta - betaTrim)
deltaDeltaE deltaE deltaETrim = (deltaE - deltaETrim)
deltaDeltaA deltaA deltaATrim = (deltaA - deltaATrim)
deltaDeltaR deltaR deltaRTrim = (deltaR - deltaRTrim)

-- deltaAlphaDot = (AlphaDot - alphaTrim)
-- deltaBetaDot = (BetaDot - betaTrim)
-- deltap
-- deltaq
-- deltar
-- deltau





-- DRAG FORCE COEFFICIENT
cSBarD alpha deltaE deltaAlphaDot deltaq deltau uTrim =
	(cDo + (cDalpha * alpha) + (cDdeltaE * deltaE) + (cDalphaDot * ((deltaAlphaDot * cBar)/(2*uTrim))) + (cDq * ((deltaq * cBar)/(2*uTrim))) + (cDu * ((deltau * cBar)/(2*uTrim))))

-- SIDE FORCE COEFFICIENT 
cSBarY beta deltaA deltaR deltaBetaDot deltap deltar uTrim =
	((cYbeta * beta) + (cYdeltaA * deltaA) + (cYdeltaR * deltaR) + (cYbetaDot * ((deltaBetaDot * b)/(2*uTrim))) + (cYp * ((deltap * b)/(2*uTrim))) + (cYr * ((deltar * b)/(2*uTrim))))

-- LIFT FORCE COEFFICIENT
cSBarL alpha deltaE deltaAlphaDot deltaq deltau uTrim =
	(cLo + (cLalpha * alpha) + (cLdeltaE * deltaE) + (cLalphaDot * ((deltaAlphaDot * cBar)/(2*uTrim))) + (cLq * ((deltaq * cBar)/(2*uTrim))) + (cLu * ((deltau * cBar)/(2*uTrim))))

-- ROLLING MOMENT COEFFICIENT
cSBarl beta deltaA deltaR deltaBetaDot deltap deltar uTrim =
	((clbeta * beta) + (cldeltaA * deltaA) + (cldeltaR * deltaR) + (clbetaDot * ((deltaBetaDot * b)/(2*uTrim))) + (clp * ((deltap * b)/(2*uTrim))) + (clr * ((deltar * b)/(2*uTrim))))

-- PITCHING MOMENT COEFFICIENT
cSBarm alpha deltaE deltaAlphaDot deltaq deltau uTrim =
	(cmo + (cmalpha * alpha) + (cmdeltaE * deltaE) + (cmalphaDot * ((deltaAlphaDot * cBar)/(2*uTrim))) + (cmq * ((deltaq * cBar)/(2*uTrim))) + (cmu * ((deltau * cBar)/(2*uTrim))))

-- YAWING MOMENT COEFFICIENT
cSBarn beta deltaA deltaR deltaBetaDot deltap deltar uTrim =
	((cnbeta * beta) + (cndeltaA * deltaA) + (cndeltaR * deltaR) + (cnbetaDot * ((deltaBetaDot * b)/(2*uTrim))) + (cnp * ((deltap * b)/(2*uTrim))) + (cnr * ((deltar * b)/(2*uTrim))))
  
cx cSBarD cSBarL = (((- (cSBarD)) * (cos alphaTrim)) + (cSBarL * (sin alphaTrim)))
cy cSBarY = (cSBarY)
cz cSbarD cSBarL = (((- (cSBarD)) * (sin alphaTrim)) + (cSBarL * (cos alphaTrim)))

cl cSBarl cSbarn = (((cSBarl) * (cos alphaTrim)) - (cSBarn * (sin alphaTrim)))
cm cSBarm = (cSBarm)
cn cSBarl cSbarn = (((cSBarl) * (sin alphaTrim)) + (cSBarn * (cos alphaTrim)))


qBar vt = ((1/2) * rho * (vt*vt))
-- AERODYNAMIC FORCES (xa ya za) 
xa qBar cx = (qBar * s * cx)
ya qBar cy = (qBar * s * cy)
za qBar cz = (qBar * s * cz)

-- AERODYNAMIC MOMENTS (la, ma, na)
la qBar cl = (qBar * s * b * cl)
ma qBar cm = (qBar * s * b * cm)
na qBar cn = (qBar * s * b * cn)

--------
-- NEWTON-EULER EQUATIONS (PHYSICS BASED MODELING APPROACH)

-- AIR SPEED, ANGLE OF ATTACK, SIDESLIP ANGLE
vtDot u v w uDot vDot wDot vt = (((u*uDot) + (v*vDot) + (w*wDot)) / vt)
alphaDot u w uDot wDot = (((u*wDot) - (w*uDot)) / ((u*u) + (w*w)))
betaDot vt beta v vtDot vDot = (((vt*vDot) - (v*vtDot)) * ((vt*vt)*(cos beta)))
			
-- EULER ANGLES (AIRCRAFT ATTITUDE)
phiDot p q r phi theta = (p + ((tan theta) * ((q * (sin phi)) + (r * (cos phi)))))
thetaDot q r phi = ((q * (cos phi)) - (r * (sin phi)))
psiDot q r phi theta = (((q * (sin phi)) + (r * (cos phi))) * (cos theta))
			
-- BODY ANGULAR RATES
pDot p q r jxz jx jy jz la na = 
	(((jxz * (jx - jy + jz) * p * q) - (((jz * (jz - jy) ) + (jxz*jxz)) * q * r) + (jz*la) + (jxz*na)) / ((jx*jz) - (jxz*jxz)))
qDot p r jxz jx jz ma = 
	((((jz - jx) * p * r) + (jxz * ((p*p) - (r*r))) + ma) / ((jx*jz) - (jxz*jxz)))
rDot p q r jxz jx jy jz la na = 
	((((((jx - jy) * jx) + (jxz*jxz)) * p * q) - (jxz * (jx - jy + jz) * q * r) + (jxz*la) + (jx*na)) / ((jx*jz) - (jxz*jxz)))
			
-- SERVO DEFLECTIONS (THROTTLE, ELEVATOR, AILERON, RUDDER)
deltaTDot deltaT deltaTcmd tauT = ((deltaT / tauT) + (deltaTcmd / tauT))
deltaEDot deltaE deltaEcmd tauE = ((deltaE / tauE) + (deltaEcmd / tauE))
deltaADot deltaA deltaAcmd tauA = ((deltaA / tauA) + (deltaAcmd / tauA))
deltaRDot deltaR deltaRcmd tauR = ((deltaR / tauR) + (deltaRcmd / tauR))

-- INTERTIAL POSITION (POSITIONS NORTH, EAST, DOWN)
pNDot u v w phi theta psi wN = 
	((u * (cos theta) * (cos psi)) + (v * (((-(cos phi)) * (sin psi)) + ((sin phi) * (sin theta) * (cos psi)))) + (w * (((sin theta) * (sin phi)) + ((cos phi) * (sin theta) * (cos psi)))) - wN)
pEDot u v w phi theta psi wE = 
	((u * (cos theta) * (sin psi)) + (v * (((cos phi) * (cos psi)) + ((sin phi) * (sin theta) * (sin psi)))) + (w * (((-(sin phi)) * (cos psi)) + ((cos phi) * (sin theta) * (sin psi)))) - wE)
pDDot u v w phi theta wH = 
	((u * (sin theta)) - (v * (sin phi) * (cos theta)) - (w * (cos phi) * (cos theta)) - wH)


--------END NEWTON-EULER EQUATIONS

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
           else do inpStr <- hGetLine inh
									 print (estOut(state inpStr))
									 mainloop inh outh

									 
									 
--input the measured states, receive the final output
-- this is the "main" for the math
estOut :: [String] -> [Float]
estOut xs = let [phi,theta, psi, p, q, r, lat, lon, alt, vnorth, veast, vdown, rc_throttle, rc_ele, rc_ail, rc_rudder, vt, alpha, beta] = map read xs
	    in stateEstimation [phi,theta, psi, p, q, r, lat, lon, alt, vnorth, veast, vdown, rc_throttle, rc_ele, rc_ail, rc_rudder, vt, alpha, beta]


stateEstimation :: [Float] -> [Float]
stateEstimation xs = stateObserver xs


-- STATE OBSERVER returns the estimated values after 
-- processing from EXTENDED KALMAN FILTER in estOut
stateObserver :: [Float] -> [Float]
stateObserver xs = so_ekf xs 

--xs is the new measurements store is the k-1 storage
so_ekf :: [Float] -> [Float]
so_ekf xs = ((so_matrix_mult ((so_y_ xs) - (so_y_est so_x_ext_))) + so_x_est_)

--in simulink model this combines data from multiple lists to a single list
--not needed in this implementation
so_y_ :: [Float] -> [Float]
so_y_ xs = xs

--function of: so_prev_meas and so_prev_x_est_
so_x_est_ :: [Float]
so_x_est_ = 


so_y_est_ :: [Float] -> [Float]
so_y_est_ xs = so_h xs

so_h :: [Float] -> [Float]
so_h xs = 

so_f :: [Float] -> [Float]
so_f xs = 


so_matrix_mult :: [Float] -> [Float]
so_matrix_mult xs = k_sates * xs


--NAVIGATION OBSERVER 

navigationObserver :: [Float] -> [Float]
navigationObserver xs = no_ekf xs 

no_ekf :: [Float] -> [Float]
no_ekf xs = ((no_matrix_mult ((no_y_ xs) - (no_y_est no_x_ext_))) + no_x_est_)

--in simulink model this combines data from multiple lists to a single list
--not needed in this implementation
no_y_ :: [Float] -> [Float]
no_y_ xs = xs

--function of: so_prev_meas and so_prev_x_est_
no_x_est_ :: [Float]
no_x_est_ = 


no_y_est_ :: [Float] -> [Float]
no_y_est_ xs = no_h xs

no_h :: [Float] -> [Float]
no_h xs = 

no_f :: [Float] -> [Float]
no_f xs = 


no_matrix_mult :: [Float] -> [Float]
no_matrix_mult xs = k_sates * xs






--helper functions to parse string from file in mainloop		
state :: String -> [String]
state xs = (take 19  (sample xs))
sample xs = 	split ',' xs
						
split :: Eq a => a -> [a] -> [[a]]
split d [] = []
split d s = x : split d (drop 1 y) where (x,y) = span (/= d) s
