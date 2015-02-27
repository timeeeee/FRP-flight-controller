-- FRP System Identification
-- EECS_582 2015
-- Angie Wright & Tim Clark


module Main where
 
--------
-- MEASURED STATES (vt alpha beta phi theta psi p q r deltaT deltaE deltaA deltaR pN pE pD)
--------

-- AIR SPEED, ANGLE OF ATTACK, SIDESLIP ANGLE
-- vt 
-- alpha 
-- beta 
			
-- EULER ANGLES (AIRCRAFT ATTITUDE)
-- phi 
-- theta 
-- psi 
			
-- BODY ANGULAR RATES
-- p 
-- q 
-- r 
			
-- SERVO DEFLECTIONS (THROTTLE, ELEVATOR, AILERON, RUDDER)
-- deltaT 
-- deltaE 
-- deltaA 
-- deltaR 

-- INTERTIAL POSITION (POSITIONS NORTH, EAST, DOWN)
-- pN 
-- pE 
-- pD

--------END MEASURED STATES

-- INPUT FROM CONTROLLER (controller output)
-- deltaTcmd
-- deltaEcmd
-- deltaAcmd
-- deltaRcmd

-- WIND INPUT 
-- wN
-- wE
-- wH

--------
-- CONSTANTS?		
--------

-- MASS (m) 
-- m

-- INTERTIA Jxz
-- jxz

-- MOMENTS OF INERTIA (jx jy jz) -- product of inertia Jxz, mass, gravity acc (assumed known and constant)
-- jx
-- jy
-- jz

-- GRAVETY (g)
-- g

-- ENGINE THRUST (xt) -- function of maximum thrust and throttle deflection (deltaT)
-- xt

-- TIME CONSTANTS for THROTTLE, ELEVATOR, AILERON, RUDDER
-- tauT
-- tauE
-- tauA
-- tauR

-- AIR DENSITY
-- rho

-- WING REFERENCE AREA
-- s

-- WING SPAN
-- b

-- MEAN GEOMETRIC CORD
-- cBar

-- TRIMMED ANGLE OF ATTACK
--alphaTrim
--betaTrim
--deltaETrim
--deltaATrim
--deltaRTrim
--uTrim


--------END CONSTANTS?



-- BODY VELOCITY COMPONENTS 
-- these require vt alpha and beta
u vt alpha beta = (vt * (cos alpha) * (cos beta))
v vt beta = (vt * (sin beta))
w vt alpha beta = (vt * (sin alpha) * (cos beta))

-- BODY ACCELERATION COMPONENTS
-- these require u v w, p q r, gravity, phi theta, xt(engine thrust) xa ya za(aerodynamic forces) m(mass)
uDot v w q r g theta xt xa m = (((r*v) - (q*w) - (g * (sin theta)) + (xt + xa))/m)
vDot u w p r g phi theta ya m = ((-(r*u) + (p*w) + (g*(sin phi)*(cos theta)) + ya)/m)
wDot u v p q g phi theta za m = (((q*u) - (p*v) + (g*(cos phi)*(cos theta)) + za)/m)


-- DYNAMIC PRESSURE  not sure how to represent 
qBar rho vt = ((1/2) * rho * (vt*vt))

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

-- ARGUMENT FOR THE FORCE COEFFICIENTS 
--cDo
--cDalpha
--cDdeltaE
--cDalphaDot
--cDq
--cDu

--cLo
--cLalpha
--cLdeltaE
--cLalphaDot
--cLq
--cLu

--cmo
--cmalpha
--cmdeltaE
--cmalphaDot
--cmq
--cmu

--cYbeta
--cYdeltaA
--cYdeltaR
--cYbetaDot
--cYp

--clbeta
--cldeltaA
--cldeltaR
--clbetaDot
--clp

--cnbeta
--cndeltaA
--cndeltaR
--cnbetaDot
--cnp


-- DRAG FORCE COEFFICIENT
cSBarD cDo cDalpha cDdeltaE cDalphaDot cDq cDu alpha deltaE deltaAlphaDot cBar deltaq deltau uTrim =
	(cDo + (cDalpha * alpha) + (cDdeltaE * deltaE) + (cDalphaDot * ((deltaAlphaDot * cBar)/(2*uTrim))) + (cDq * ((deltaq * cBar)/(2*uTrim))) + (cDu * ((deltau * cBar)/(2*uTrim))))

-- SIDE FORCE COEFFICIENT 
cSBarY cYbeta cYdeltaA cYdeltaR cYbetaDot cYp cYr beta deltaA deltaR deltaBetaDot b deltap deltar uTrim =
	((cYbeta * beta) + (cYdeltaA * deltaA) + (cYdeltaR * deltaR) + (cYbetaDot * ((deltaBetaDot * b)/(2*uTrim))) + (cYp * ((deltap * b)/(2*uTrim))) + (cYr * ((deltar * b)/(2*uTrim))))

-- LIFT FORCE COEFFICIENT
cSBarL cLo cLalpha cLdeltaE cLalphaDot cLq cLu alpha deltaE deltaAlphaDot cBar deltaq deltau uTrim =
	(cLo + (cLalpha * alpha) + (cLdeltaE * deltaE) + (cLalphaDot * ((deltaAlphaDot * cBar)/(2*uTrim))) + (cLq * ((deltaq * cBar)/(2*uTrim))) + (cLu * ((deltau * cBar)/(2*uTrim))))

-- ROLLING MOMENT COEFFICIENT
cSBarl clbeta cldeltaA cldeltaR clbetaDot clp clr beta deltaA deltaR deltaBetaDot b deltap deltar uTrim =
	((clbeta * beta) + (cldeltaA * deltaA) + (cldeltaR * deltaR) + (clbetaDot * ((deltaBetaDot * b)/(2*uTrim))) + (clp * ((deltap * b)/(2*uTrim))) + (clr * ((deltar * b)/(2*uTrim))))

-- PITCHING MOMENT COEFFICIENT
cSBarm cmo cmalpha cmdeltaE cmalphaDot cmq cmu alpha deltaE deltaAlphaDot cBar deltaq deltau uTrim =
	(cmo + (cmalpha * alpha) + (cmdeltaE * deltaE) + (cmalphaDot * ((deltaAlphaDot * cBar)/(2*uTrim))) + (cmq * ((deltaq * cBar)/(2*uTrim))) + (cmu * ((deltau * cBar)/(2*uTrim))))

-- YAWING MOMENT COEFFICIENT
cSBarn cnbeta cndeltaA cndeltaR cnbetaDot cnp cnr beta deltaA deltaR deltaBetaDot b deltap deltar uTrim =
	((cnbeta * beta) + (cndeltaA * deltaA) + (cndeltaR * deltaR) + (cnbetaDot * ((deltaBetaDot * b)/(2*uTrim))) + (cnp * ((deltap * b)/(2*uTrim))) + (cnr * ((deltar * b)/(2*uTrim))))
  
cx cSBarD cSBarL alphaTrim = (((- (cSBarD)) * (cos alphaTrim)) + (cSBarL * (sin alphaTrim)))
cy cSBarY = (cSBarY)
cz cSbarD cSBarL alphaTrim = (((- (cSBarD)) * (sin alphaTrim)) + (cSBarL * (cos alphaTrim)))

cl cSBarl cSbarn alphaTrim = (((cSBarl) * (cos alphaTrim)) - (cSBarn * (sin alphaTrim)))
cm cSBarm = (cSBarm)
cn cSBarl cSbarn alphaTrim = (((cSBarl) * (sin alphaTrim)) + (cSBarn * (cos alphaTrim)))

-- AERODYNAMIC FORCES (xa ya za) 
xa qBar s cx = (qBar * s * cx)
ya qBar s cy = (qBar * s * cy)
za qBar s cz = (qBar * s * cz)

-- AERODYNAMIC MOMENTS (la, ma, na)
la qBar s b cl = (qBar * s * b * cl)
ma qBar s b cm = (qBar * s * b * cm)
na qBar s b cn = (qBar * s * b * cn)

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
	putStrLn "Hello."
	putStrLn "Goodbye."

