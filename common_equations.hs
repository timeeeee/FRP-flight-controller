module Common_Equations  
(xt
,c1
,c2
,c3
,c4
,c5
,c6
,c7
,c8
,c9
,delta
,momentConstants
,u
,v
,w
,m
,uDot
,vDot
,wDot
,deltaAlpha
,deltaBeta
,deltaDeltaE
,deltaDeltaA
,deltaDeltaR
,cSBarD
,cSBarY
,cSBarL
,cSBarl
,cSBarm
,cSBarn
,qBar
,xa
,ya
,za
,la
,ma
,na
,vtDot
,alphaDot
,betaDot
,phiDot
,thetaDot
,psiDot
,pDot
,qDot
,rDot
,deltaTDot
,deltaEDot
,deltaADot
,deltaRDot
,pNDot
,pEDot
,pDDot) where  

import Constants

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
m = (weight/g)

uDot v w q r g theta xt xa = (((r*v) - (q*w) - (g * (sin theta)) + (xt + xa))/m)
vDot u w p r g phi theta ya = ((-(r*u) + (p*w) + (g*(sin phi)*(cos theta)) + ya)/m)
wDot u v p q g phi theta za = (((q*u) - (p*v) + (g*(cos phi)*(cos theta)) + za)/m)




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
  
--cx cSBarD cSBarL = (((- (cSBarD)) * (cos alphaTrim)) + (cSBarL * (sin alphaTrim)))
--cy cSBarY = (cSBarY)
--cz cSbarD cSBarL = (((- (cSBarD)) * (sin alphaTrim)) + (cSBarL * (cos alphaTrim)))

--cl cSBarl cSbarn = (((cSBarl) * (cos alphaTrim)) - (cSBarn * (sin alphaTrim)))
--cm cSBarm = (cSBarm)
--cn cSBarl cSbarn = (((cSBarl) * (sin alphaTrim)) + (cSBarn * (cos alphaTrim)))


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
