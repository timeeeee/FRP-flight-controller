module Constants  
( --a_W1 
--a_W2 
--a_W3
aAileron 	
,aElevator	
,aileron0_o
,aileronMax 
,aileronMin
,aRudder	
,aThrottle
--b_W1
--b_W2
--b_W3
,bAileron
,bElevator
,bRudder	
,bThrottle
,cD0	
,cD0_bar
,cD1
,cDa	
,cDaDot
,cDde
,cDq	
,cDu	
,cL0	
,cL1	
,cLa
,cLaDot
,cLde
,cLq
,cLu	
--c_W1 
--c_W2 
--c_W3
,cAileron
,cBar
,cElevator
--cH_tilde
,clb	
,clda
,cldr
,clp	
,clr
,cm0
,cm1	
,cma
,cmaDot
,cmde
,cmq
,cmu	
,cnb
,cnda
,cndr
,cnp
,cnr
,cRudder
,cThrottle
,cyb	
,cyda
,cydr
,cyp
,cyr	
--d_W1
--d_W2
--d_W3
,dAileron
,dElevator
,dRudder
,dThrottle
,elevator0_o
,elevatorMax
,elevatorMin
--f_second_hinf 
,gAM	
,hD0noaa	
,hE0noaa
,hN0noaa
,ixxB
,ixzB
,iyyB
,izzB
,kT
--k_nav 
--k_states 
--kd
--kd_inv 
,kiLat
,kiLon
,kpLat 
,kpLon 
,l
,nN_ann_X	
,nN_ann_Y 
,nN_ann_Z 
,n_nmpc
--p0_est_nav
--p0_est_sta
,p0_o	
,pA0	
,ptrim	
,q0_o
--q_obs_nav
--w_obs_sta
,qtrim
,r0_o
--r_obs_nav 
--r_obs_sta 
,rtrim 
,rudder0_o
,rudderMax	
,rudderMin
,s	
,tR
,throttle0_o
,throttleMax
,throttleMin
,utrim	
,vT0_o
,vTtrim
--w11d
--w12d
--w13d
--w14d
--w21d
--w22d
--w23d
--w24d
--w31d
--w32d
--w33d
--w34d
,x_maxXin
,x_maxXout
,x_maxYin
,x_maxYout
,x_minXin
,x_minXout
,x_minYin
,x_minYout
,y_maxXin
,y_maxXout
,y_maxYin
,y_maxYout
,y_minXin
,y_minXout
,y_minYin
,y_minYout
,z_maxXin
,z_maxXout
,z_maxYin
,z_maxYout
,z_minXin
,z_minXout
,z_minYin
,z_minYout
--a_ann_X_ic
--a_ann_Y_ic
--a_ann_Z_ic
,ailerontrim
,alpha0_o
,alphab0_o
,alphatrim
,ann_window_samples
,b
--b_ann_X_ic	(40x1 matrix)
--b_ann_Y_ic (40x1 matrix)
--b_ann_Z_ic (40x1 matrix)
,beta0_o
,betab0_o
,betatrim
,c_ann_X_ic
,c_ann_Y_ic
,c_ann_Z_ic
,dist2b
,dt
,e0_o
,elevatortrim
,g
,h0_o
,lambda_ann_X_ic
,lambda_ann_Y_ic
,lambda_ann_Z_ic
,n0_o
,n_ann_X
,n_ann_Y
,n_ann_Z
,ne
,nu
,nz
,p_ann_X
,p_ann_Y
,p_ann_Z
,phi0_o
,phitrim
,psi0_o
,psitrim
,rho
,rho_o
,robustness
,ruddertrim
,saturation_max_sta
,saturation_min_sta
,theta0_o
,thetatrim
,throttletrim
--u_IC
,uas
,ve0_o
,vh0_o
,vn0_o
--w_ann_X_ic
--w_ann_Y_ic
--w_ann_Z_ic
,weight
,x0_est_nav
,x0_est_sta
,xT0
,xT1
,xT2
,xw110_sim
,xw120_sim
,xw130_sim
,xw140_sim
,xw210_sim
,xw220_sim
,xw230_sim
,xw240_sim
,xw310_sim
,xw320_sim
,xw330_sim	
,xw340_sim) where  

--import Data.Number.Transfinite
infinity :: Float
infinity = 1.0 / 0.0

negativeInfinity :: Float
negativeInfinity = (-1.0) / 0.0

--these values are from newvars_4_25 for the Bird of Time Sailplane
--a_W1 (4x4 matrix)
--a_W2 (4x4 matrix)
--a_W3 (4x4 matrix)

aAileron :: Float
aAileron 		= -14.2800

aElevator :: Float
aElevator		= -14.2800

aileron0_o :: Float
aileron0_o	= 0.0

aileronMax :: Float
aileronMax 	= 0.3491

aileronMin :: Float
aileronMin	= -0.3491

aRudder :: Float
aRudder		 	= -5.0

aThrottle :: Float
aThrottle		= -10

--b_W1 (4x4 matrix)
--b_W2 (4x4 matrix)
--b_W3 (4x4 matrix)

bAileron :: Float
bAileron		= 14.2800

bElevator :: Float
bElevator		= 14.2800

bRudder :: Float
bRudder			= 5.0

bThrottle :: Float
bThrottle		= 10.0

cD0 :: Float
cD0					= 0.0445

cD0_bar :: Float
cD0_bar			= 0.0407

cD1 :: Float
cD1					= 0.0851

cDa :: Float
cDa					= 0.0073

cDaDot :: Float
cDaDot			= 0.0

cDde :: Float
cDde				= 0.0085

cDq :: Float
cDq					= 0.0

cDu :: Float
cDu					= 0.0

cL0 :: Float
cL0					= 0.2793

cL1 :: Float
cL1					= 0.5431

cLa :: Float
cLa					= 5.0137

cLaDot :: Float
cLaDot			= 0.7680

cLde :: Float
cLde				= 0.4409

cLq :: Float
cLq					= 3.9215

cLu :: Float
cLu					= 0.00042900 --4.2900e-04

--c_W1 (4x4 matrix)
--c_W2 (4x4 matrix)
--c_W3 (4x4 matrix)

cAileron :: Float
cAileron		= 1.0

cBar :: Float
cBar				= 0.7900

cElevator :: Float
cElevator		= 1.0

--cH_tilde (4x12 matrix)

clb :: Float
clb					= 0.0181

clda :: Float
clda				= 0.1368

cldr :: Float
cldr				= 0.0017

clp :: Float
clp					= -0.4384

clr :: Float
clr					= 0.0742

cm0 :: Float
cm0					= 0.0276

cm1 :: Float
cm1					= -0.00070000 ---7.0000e-04

cma :: Float
cma					= -0.3528

cmaDot :: Float
cmaDot			= -2.5798

cmde :: Float
cmde				= -1.4810

cmq :: Float
cmq					= -10.3561

cmu :: Float
cmu					= 0.0

cnb :: Float
cnb					= 0.0213

cnda :: Float
cnda				= -0.0057

cndr :: Float
cndr				= -0.0223

cnp :: Float
cnp					= -0.0459

cnr :: Float
cnr					= -0.0184

cRudder :: Float
cRudder			= 1.0

cThrottle :: Float
cThrottle		= 1.0

cyb :: Float
cyb					= -0.0985

cyda :: Float
cyda				= 0.0

cydr :: Float
cydr				= 0.0797

cyp :: Float
cyp					= 0.00058400 --5.8400e-04

cyr :: Float
cyr					= 0.0444

--d_W1 (4x4 matrix)
--d_W2 (4x4 matrix)
--d_W3 (4x4 matrix)

dAileron :: Float
dAileron		= 0.0

dElevator :: Float
dElevator		= 0.0

dRudder :: Float
dRudder			= 0.0

dThrottle :: Float
dThrottle		= 0.0

elevator0_o :: Float
elevator0_o	= 0.0201

elevatorMax :: Float
elevatorMax	= 0.2618

elevatorMin :: Float
elevatorMin	= -0.2618

--f_second_hinf (4x20 matrix)

gAM :: Float
gAM					= 1.4079

hD0noaa :: Float
hD0noaa			= 48.3774

hE0noaa :: Float
hE0noaa			= 0.8962

hN0noaa :: Float
hN0noaa			= 20.6483


--moment constants
--IxxB
ixxB :: Float
ixxB = 0.0301

--IxzB
ixzB :: Float
ixzB = 0.00

--IyyB
iyyB :: Float
iyyB = 0.1206

--IzzB
izzB :: Float
izzB = 0.0725

kT :: Float
kT	= 25.0

--k_nav (6x6 matrix)
--k_states (15x11 matrix)
--kd (4x4 ss)
--kd_inv (4x4 ss)

kiLat :: Float
kiLat	= 0.0

kiLon :: Float
kiLon = 0.00093750 --9.3750e-04

kpLat :: Float
kpLat = 0.0156

kpLon :: Float
kpLon = 0.0063

l :: Float
l = 300.0

nN_ann_X :: Float
nN_ann_X	= 40.0

nN_ann_Y :: Float
nN_ann_Y = 40.0

nN_ann_Z :: Float
nN_ann_Z = 40.0

n_nmpc :: Float
n_nmpc	= 20.0

--p0_est_nav (6x6 matrix)
--p0_est_sta (15x15 matrix)

p0_o :: Float
p0_o		= 0.0

pA0 :: Float
pA0			= 37138.0 --3.7138e+04

ptrim :: Float
ptrim		= 0.0

q0_o :: Float
q0_o		= -0.000000063643 -- -6.3643e-08

--q_obs_nav (6x6 matrix)
--w_obs_sta (15x15 matrix)

qtrim :: Float
qtrim		= -0.000000063643 -- -6.3643e-08

r0_o :: Float
r0_o		= 0.0

--r_obs_nav (6x6 matrix)
--r_obs_sta (9x9 matrix)

rtrim :: Float
rtrim 		= 0.0

rudder0_o :: Float
rudder0_o	= 0.0

rudderMax :: Float
rudderMax	= 0.1745

rudderMin :: Float
rudderMin	= -0.1745

s :: Float
s					= 7.4300

tR :: Float
tR				= 0.0

throttle0_o :: Float
throttle0_o	= 0.5500

throttleMax :: Float
throttleMax = 1.0

throttleMin :: Float
throttleMin	= 0.1500

utrim :: Float
utrim				= 45.8074

vT0_o :: Float
vT0_o				= 45.8083

vTtrim :: Float
vTtrim			= 45.8083

--w11d
--w12d
--w13d
--w14d
--w21d
--w22d
--w23d
--w24d
--w31d
--w32d
--w33d
--w34d

--NOTE: THIS WAS A SET OF ROWS IN NEWVARS!
x_maxXin :: [Float]
x_maxXin	= [168,0.2000,0.0100,1.0,0.5000,1.0,0.2000]

x_maxXout :: Float
x_maxXout	= 0.2000

x_maxYin :: Float
x_maxYin	= 1.0

x_maxYout :: Float
x_maxYout	= 1.0

--NOTE: THIS WAS A SET OF ROWS IN NEWVARS!
x_minXin :: [Float]
x_minXin	= [162,-0.2000,-0.0100,-1.0,-0.5000,0.1000,-0.2000]

x_minXout :: Float
x_minXout	= 0.0100

x_minYin :: Float
x_minYin	= -1.0

x_minYout :: Float
x_minYout	= -1.0

--NOTE: THIS WAS A SET OF ROWS IN NEWVARS!
y_maxXin :: [Float]
y_maxXin	= [168,0.0100,2.0,1.0,0.2000,0.6000,0.5000]

y_maxXout :: Float
y_maxXout	= 0.2000

y_maxYin :: Float
y_maxYin	= 1.0

y_maxYout :: Float
y_maxYout	= 1.0

--NOTE: THIS WAS A SET OF ROWS IN NEWVARS!
y_minXin :: [Float]
y_minXin	= [162,-0.0100,-2.0,-1.0,-0.2000,-0.6000,-0.5000]

y_minXout :: Float
y_minXout	= -0.2000

y_minYin :: Float
y_minYin	= -1.0

y_minYout :: Float
y_minYout	= -1.0

--NOTE: THIS WAS A SET OF ROWS IN NEWVARS!
z_maxXin :: [Float]
z_maxXin	= [168,0.200000000000000,0.0100000000000000,1,0.500000000000000,1,0.200000000000000]

z_maxXout :: Float
z_maxXout	= -0.5000

z_maxYin :: Float
z_maxYin	= 1.0

z_maxYout :: Float
z_maxYout	= 1.0

--NOTE: THIS WAS A SET OF ROWS IN NEWVARS!
z_minXin :: [Float]
z_minXin	= [162,-0.200000000000000,-0.0100000000000000,-1,-0.500000000000000,0.100000000000000,-0.200000000000000]

z_minXout :: Float
z_minXout	= -3.0

z_minYin :: Float
z_minYin	= -1.0

z_minYout :: Float
z_minYout	= -1.0


--a_ann_X_ic (1x40 matrix)
--a_ann_Y_ic (1x40 matrix)
--a_ann_Z_ic (1x40 matrix)

ailerontrim :: Float
ailerontrim = 0.0

alpha0_o :: Float
alpha0_o		= -0.0060

alphab0_o :: Float
alphab0_o		= 0.0

alphatrim :: Float
alphatrim		= -0.0060

ann_window_samples :: Float
ann_window_samples = 7.0

b :: Float
b						= 10.7300

--b_ann_X_ic	(40x1 matrix)
--b_ann_Y_ic (40x1 matrix)
--b_ann_Z_ic (40x1 matrix)

beta0_o :: Float
beta0_o	= 0.0

betab0_o :: Float
betab0_o	= 0.0

betatrim :: Float
betatrim	= 0.0

c_ann_X_ic :: Float
c_ann_X_ic	= 0.0926

c_ann_Y_ic :: Float
c_ann_Y_ic 	= 0.5791

c_ann_Z_ic :: Float
c_ann_Z_ic 	= 1.3843

dist2b :: Float
dist2b			= 320.0

dt :: Float
dt					= 0.0192

e0_o :: Float
e0_o				= 0.0

elevatortrim :: Float
elevatortrim	= 0.0201

g :: Float
g		= 32.17400

h0_o :: Float
h0_o	= 200.00

lambda_ann_X_ic :: Float
lambda_ann_X_ic	= 0.00001 --1.0000e-05

lambda_ann_Y_ic :: Float
lambda_ann_Y_ic	= 0.00001 --1.0000e-05

lambda_ann_Z_ic :: Float
lambda_ann_Z_ic	= 0.00001 --1.0000e-05

n0_o :: Float
n0_o	= 0.0

n_ann_X :: Float
n_ann_X	= 7.0

n_ann_Y :: Float
n_ann_Y = 7.0

n_ann_Z :: Float
n_ann_Z = 7.0

ne :: Float
ne	= 4.0

nu :: Float
nu	= 4.0

nz :: Float
nz	= 12.0
 
p_ann_X :: Float
p_ann_X	= 1.0

p_ann_Y :: Float
p_ann_Y	= 1.0

p_ann_Z :: Float
p_ann_Z	= 1.0

phi0_o :: Float
phi0_o	= 0.0

phitrim :: Float
phitrim	= 0.0

psi0_o :: Float
psi0_o	= 0.0

psitrim :: Float
psitrim	= 0.0

rho :: Float
rho	= 0.0023

rho_o :: Float
rho_o	= 0.0023

robustness :: Float
robustness	= 2.0

ruddertrim :: Float
ruddertrim	= 0.0

saturation_max_sta :: [Float]
saturation_max_sta = [1,	0.261799387799149,	0.349065850398866,	0.174532925199433,	200,	1.57079632679490,	1.57079632679490,	3.14159265358979,	3.14159265358979,	infinity,	15.7079632679490,	15.7079632679490,	10.4719755119660,	0.523598775598299,	0.523598775598299]

saturation_min_sta :: [Float]
saturation_min_sta = [0.150000000000000,	-0.261799387799149,	-0.349065850398866,	-0.174532925199433,	2.22044604925031e-15,	-1.57079632679490,	-1.57079632679490,	-3.14159265358979,	-3.14159265358979,	negativeInfinity,	-15.7079632679490,	-15.7079632679490,	-10.4719755119660,	-0.523598775598299,	-0.523598775598299]

theta0_o :: Float
theta0_o	= -0.0023

thetatrim :: Float
thetatrim	= -0.0023

throttletrim :: Float
throttletrim	= 0.5500

--u_IC	= (4x21 matrix)

uas :: Float
uas	= 4.0

ve0_o :: Float
ve0_o	= 0.0

vh0_o :: Float
vh0_o	= 0.0

vn0_o :: Float
vn0_o	= 0.0

--w_ann_X_ic (40x7 matrix)
--w_ann_Y_ic (40x7 matrix)
--w_ann_Z_ic (40x7 matrix)

weight :: Float
weight	= 4.6100

x0_est_nav :: [Float]
x0_est_nav = [0.0,0.0,200.0,0.0,0.0,0.0]

x0_est_sta :: [Float]
x0_est_sta	= [0.5500,0.0201,0.0,0.0,45.8083,-0.0060,0.0,0.0,-0.0023,0.0,0.0,-0.000000063643,0.0,0.0,0.0]


--Engine Constants
xT0 :: Float
xT0 = 0.00

xT1 :: Float
xT1 = 0.0142

xT2 :: Float
xT2	= 0.00

--Simulation Constants??
xw110_sim :: Float
xw110_sim	= -2.5000

xw120_sim :: Float
xw120_sim	= 0.0

xw130_sim :: Float
xw130_sim	= 0.0050

xw140_sim :: Float
xw140_sim	= -0.0490

xw210_sim :: Float
xw210_sim	= 0.0

xw220_sim :: Float
xw220_sim	= 0.0

xw230_sim :: Float
xw230_sim	= 0.0

xw240_sim :: Float
xw240_sim	= 2.0

xw310_sim :: Float
xw310_sim	= -2.3000

xw320_sim :: Float
xw320_sim	= 0.0

xw330_sim :: Float
xw330_sim	= 0.0

xw340_sim :: Float
xw340_sim	= 0.0

