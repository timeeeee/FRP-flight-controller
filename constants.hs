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
--cYbetaDot ?????????? 0 for now
cYbetaDot = 0
--Cyp
cYp = 5.8400
--Cyr ----missing! ????
cYr = 0

--Clb
clbeta = 0.0181
--Clda
cldeltaA = 0.1368
--Cldr
cldeltaR = 0.0017
--clbetaDot ??????????????
clbetaDot = 0
--Clp
clp = -0.4384
--Clr ----missing! ????
clr = 0

--Cnb
cnbeta = 0.0213
--Cnda
cndeltaA = -0.0057
--Cndr
cndeltaR = -0.0223
--cnbetaDot ???????????????
cnbetaDot = 0
--Cnp
cnp = -0.0459
--cnr ----missing! ????
cnr = 0

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

k_states = []