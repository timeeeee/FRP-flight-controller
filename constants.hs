module Constants  
( xT0 
,	xT1
,	xT2
, iyyB
, ixxB
, izzB
, ixzB
, cDo
, cDalpha
, cDdeltaE
, cDalphaDot
, cDq
, cDu
, cLo
, cLalpha
, cLdeltaE
, cLalphaDot
, cLq
, cLu
, cmo
, cmalpha
, cmdeltaE
, cmalphaDot
, cmq
, cmu
, cYbeta
, cYdeltaA
, cYdeltaR
, cYbetaDot
, cYp
, cYr
, clbeta
, cldeltaA
, cldeltaR
, clbetaDot
, clp
, clr
, cnbeta
, cndeltaA
, cndeltaR
, cnbetaDot
, cnp
, cnr
, s
, b
, cBar
, alphaTrim
, betaTrim
, weight
, g
, rho
, k_states) where  


--these values are from newvars_1_60 for the yak_54 33%

--Engine Constants
xT0 :: Float
xT0 = 0.00

xT1 :: Float
xT1 = 0.0142

xT2 :: Float
xT2	= 0.00

--moment constants
--IyyB
iyyB :: Float
iyyB = 0.1206

--IxxB
ixxB :: Float
ixxB = 0.0301

--IzzB
izzB :: Float
izzB = 0.0725

--IxzB
ixzB :: Float
ixzB = 0.00

-- ARGUMENT FOR THE FORCE COEFFICIENTS 
--CDO
cDo :: Float
cDo = 0.0445

--CDA
cDalpha :: Float
cDalpha = 0.0073

--CDde
cDdeltaE :: Float
cDdeltaE = 0.0085

--CDadot
cDalphaDot :: Float
cDalphaDot = 0.00

--CDq
cDq :: Float
cDq = 0.00

--CDu
cDu :: Float
cDu = 0.00

--CLO
cLo :: Float
cLo = 0.2793

--CLa
cLalpha :: Float
cLalpha = 5.0137

--CLde
cLdeltaE :: Float
cLdeltaE = 0.4409

--CLadot
cLalphaDot :: Float
cLalphaDot = 0.7680

--CLq
cLq:: Float
cLq = 3.9215

--CLu
cLu:: Float
cLu = 4.2900

--CmO
cmo:: Float
cmo = 0.0276

--Cma
cmalpha:: Float
cmalpha = -0.3528

--Cmde
cmdeltaE :: Float
cmdeltaE = -1.4810

--Cmadot
cmalphaDot :: Float
cmalphaDot = -2.5798

--Cmq
cmq :: Float
cmq = -10.3561

--Cmu
cmu :: Float
cmu = 0.00

--Cyb
cYbeta :: Float
cYbeta = -0.0985

--Cyda
cYdeltaA :: Float
cYdeltaA = 0.00

--Cydr
cYdeltaR :: Float
cYdeltaR = 0.0797

--cYbetaDot ?????????? 0 for now
cYbetaDot :: Float
cYbetaDot = 0.00

--Cyp
cYp :: Float
cYp = 5.8400

--Cyr ----missing! ????
cYr :: Float
cYr = 0.00

--Clb
clbeta :: Float
clbeta = 0.0181

--Clda
cldeltaA :: Float
cldeltaA = 0.1368

--Cldr
cldeltaR :: Float
cldeltaR = 0.0017

--clbetaDot ??????????????
clbetaDot :: Float
clbetaDot = 0

--Clp
clp :: Float
clp = -0.4384

--Clr ----missing! ????
clr :: Float
clr = 0

--Cnb
cnbeta :: Float
cnbeta = 0.0213

--Cnda
cndeltaA :: Float
cndeltaA = -0.0057

--Cndr
cndeltaR :: Float
cndeltaR = -0.0223

--cnbetaDot ???????????????
cnbetaDot :: Float
cnbetaDot = 0.00

--Cnp
cnp :: Float
cnp = -0.0459

--cnr ----missing! ????
cnr :: Float
cnr = 0.00



-- WING REFERENCE AREA
--S
s :: Float
s = 7.4300

-- WING SPAN
b  :: Float
b = 10.7300

-- MEAN GEOMETRIC CORD
--Cbar
cBar :: Float
cBar = 0.7900

-- TRIMMED ANGLE OF ATTACK
alphaTrim :: Float
alphaTrim = -0.0060

betaTrim :: Float
betaTrim = 0

--deltaETrim ???????????
--deltaATrim ??????????
--deltaRTrim ????????????
--uTrim      ????????????

-- WEIGHT
weight :: Float
weight = 4.6100

-- GRAVITY APPROXIMATION
g :: Float
g = 32.1740

-- DYNAMIC PRESSURE 
rho :: Float
rho = 0.0023

k_states :: [Float]
k_states = []