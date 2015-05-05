module Common_Equations  
(c1
,c2
,c3
,c4
,c5
,c6
,c7
,c8
,c9
,u
,v
,w
,uDot
,vDot
,wDot
,accelX_o
,accelY_o
,accelZ_o
,qBar
,cl_o
,cn_o
,cm_o
) where  

import Constants

--moment functions (used above in f_function)

delta :: Float
delta = ((ixxB * izzB) - (ixzB * ixzB))


c1 :: Float
c1 = ((((iyyB - izzB) * izzB) - (ixzB * ixzB)) / delta)

c2 :: Float
c2 = ((((ixxB - iyyB) + izzB) * ixzB) / delta)

c3 :: Float
c3 = (izzB / delta)

c4 :: Float
c4 = (ixzB / delta)

c5 :: Float
c5 = ((izzB - ixxB) / iyyB)

c6 :: Float
c6 = (ixzB / iyyB)

c7 :: Float
c7 = (1 / iyyB)

c8 :: Float
c8 = ((((ixxB - iyyB) * ixxB) + (ixzB * ixzB)) / delta)

c9 :: Float
c9 = (ixxB / delta)



-- BODY VELOCITY COMPONENTS 
-- these require vt alpha and beta
u :: [Float] -> Float
u xs = (let (vt, alpha, beta) = ((xs !! 4),(xs !! 5),(xs !! 6))
				in vt) --(vt * (cos alpha) * (cos beta)))

v :: [Float] -> Float
v xs = (let (vt, alpha, beta) = ((xs !! 4),(xs !! 5),(xs !! 6))
				in (vt * (sin beta)))

w :: [Float] -> Float
w xs = (let (vt, alpha, beta) = ((xs !! 4),(xs !! 5),(xs !! 6))
				in (vt * (sin alpha) * (cos beta)))

-- %aerodynamic coefficients
cL_o :: [Float] -> Float
cL_o xs = (let (throttle, elevator, aileron, rudder, vt, alpha, beta, phi, theta, psi, p, q, r, bias1, bias2, ax, ay, az, alphaDot) = ((xs !! 0), (xs !! 1),(xs !! 2),(xs !! 3),(xs !! 4),(xs !! 5),(xs !! 6),(xs !! 7),(xs !! 8),(xs !! 9),(xs !! 10),(xs !! 11),(xs !! 12),(xs !! 13),(xs !! 14),(xs !! 15),(xs !! 16),(xs !! 17),(xs !! 19))
					 in  (cL0 + cLa * alpha + cLq * (q - qtrim) * cBar/2/utrim + cLaDot * alphaDot * cBar/2/utrim + cLu * ((u xs) - utrim) / utrim + cLde * elevator))

-- not used in model: %CD_k=CD0+CDa*alpha_k+CDq*(Q_k-Qtrim)*Cbar/2/Utrim+CDadot*alphadot_k*Cbar/2/Utrim+CDu*(U_k-Utrim)/Utrim+CDde*EL_k; %lin

cD_o :: [Float] -> Float
cD_o xs = (let (throttle, elevator, aileron, rudder, vt, alpha, beta, phi, theta, psi, p, q, r, bias1, bias2, ax, ay, az, alphaDot) = ((xs !! 0), (xs !! 1),(xs !! 2),(xs !! 3),(xs !! 4),(xs !! 5),(xs !! 6),(xs !! 7),(xs !! 8),(xs !! 9),(xs !! 10),(xs !! 11),(xs !! 12),(xs !! 13),(xs !! 14),(xs !! 15),(xs !! 16),(xs !! 17),(xs !! 19))
					 in (cD0_bar + (cL_o xs)^2 / (pi * b / cBar * 0.87) + cDq * (q - qtrim) * cBar /2/ utrim + cDaDot * alphaDot * cBar/2/utrim + cDu * ((u xs)-utrim)/utrim + cDde * elevator))

cY_o :: [Float] -> Float
cY_o xs = (let (throttle, elevator, aileron, rudder, vt, alpha, beta, phi, theta, psi, p, q, r, bias1, bias2, ax, ay, az, alphaDot) = ((xs !! 0), (xs !! 1),(xs !! 2),(xs !! 3),(xs !! 4),(xs !! 5),(xs !! 6),(xs !! 7),(xs !! 8),(xs !! 9),(xs !! 10),(xs !! 11),(xs !! 12),(xs !! 13),(xs !! 14),(xs !! 15),(xs !! 16),(xs !! 17),(xs !! 19))
					 in (cyb * beta + cyp * (p - ptrim) * b/2/utrim + cyr * (r - rtrim) * b/2/utrim + cyda * aileron + cydr * rudder))

cls_o :: [Float] -> Float
cls_o xs = (let (throttle, elevator, aileron, rudder, vt, alpha, beta, phi, theta, psi, p, q, r, bias1, bias2, ax, ay, az, alphaDot) = ((xs !! 0), (xs !! 1),(xs !! 2),(xs !! 3),(xs !! 4),(xs !! 5),(xs !! 6),(xs !! 7),(xs !! 8),(xs !! 9),(xs !! 10),(xs !! 11),(xs !! 12),(xs !! 13),(xs !! 14),(xs !! 15),(xs !! 16),(xs !! 17),(xs !! 19))
					 in (clb * beta + clp * (p - ptrim) * b/2/utrim + clr * (r - rtrim) * b/2/utrim + clda * aileron + cldr * rudder))

cms_o :: [Float] -> Float
cms_o xs = (let (throttle, elevator, aileron, rudder, vt, alpha, beta, phi, theta, psi, p, q, r, bias1, bias2, ax, ay, az, alphaDot) = ((xs !! 0), (xs !! 1),(xs !! 2),(xs !! 3),(xs !! 4),(xs !! 5),(xs !! 6),(xs !! 7),(xs !! 8),(xs !! 9),(xs !! 10),(xs !! 11),(xs !! 12),(xs !! 13),(xs !! 14),(xs !! 15),(xs !! 16),(xs !! 17),(xs !! 19))
					 in (cm0 + cma * alpha + cmq * (q - qtrim) * cBar/2/utrim + cmaDot * alphaDot * cBar/2/utrim + cmu * ((u xs) - utrim) / utrim + cmde * elevator))

cns_o :: [Float] -> Float
cns_o xs = (let (throttle, elevator, aileron, rudder, vt, alpha, beta, phi, theta, psi, p, q, r, bias1, bias2, ax, ay, az, alphaDot) = ((xs !! 0), (xs !! 1),(xs !! 2),(xs !! 3),(xs !! 4),(xs !! 5),(xs !! 6),(xs !! 7),(xs !! 8),(xs !! 9),(xs !! 10),(xs !! 11),(xs !! 12),(xs !! 13),(xs !! 14),(xs !! 15),(xs !! 16),(xs !! 17),(xs !! 19))
					 in (cnb * beta + cnp * (p - ptrim) * b/2/utrim + cnr * (r - rtrim) * b/2/utrim + cnda * aileron + cndr * rudder))

cxA_o :: [Float] -> Float
cxA_o xs = (let (throttle, elevator, aileron, rudder, vt, alpha, beta, phi, theta, psi, p, q, r, bias1, bias2, ax, ay, az, alphaDot) = ((xs !! 0), (xs !! 1),(xs !! 2),(xs !! 3),(xs !! 4),(xs !! 5),(xs !! 6),(xs !! 7),(xs !! 8),(xs !! 9),(xs !! 10),(xs !! 11),(xs !! 12),(xs !! 13),(xs !! 14),(xs !! 15),(xs !! 16),(xs !! 17),(xs !! 19))
					 in (-(cD_o xs) * (cos alpha) + (cL_o xs) * (sin alpha)))

cyA_o :: [Float] -> Float
cyA_o xs = (let (throttle, elevator, aileron, rudder, vt, alpha, beta, phi, theta, psi, p, q, r, bias1, bias2, ax, ay, az, alphaDot) = ((xs !! 0), (xs !! 1),(xs !! 2),(xs !! 3),(xs !! 4),(xs !! 5),(xs !! 6),(xs !! 7),(xs !! 8),(xs !! 9),(xs !! 10),(xs !! 11),(xs !! 12),(xs !! 13),(xs !! 14),(xs !! 15),(xs !! 16),(xs !! 17),(xs !! 19))
					 in (cY_o xs))

czA_o :: [Float] -> Float
czA_o xs = (let (throttle, elevator, aileron, rudder, vt, alpha, beta, phi, theta, psi, p, q, r, bias1, bias2, ax, ay, az, alphaDot) = ((xs !! 0), (xs !! 1),(xs !! 2),(xs !! 3),(xs !! 4),(xs !! 5),(xs !! 6),(xs !! 7),(xs !! 8),(xs !! 9),(xs !! 10),(xs !! 11),(xs !! 12),(xs !! 13),(xs !! 14),(xs !! 15),(xs !! 16),(xs !! 17),(xs !! 19))
					 in (-(cD_o xs) * (sin alpha) - (cL_o xs) * (cos alpha)))

cl_o :: [Float] -> Float
cl_o xs = (let (throttle, elevator, aileron, rudder, vt, alpha, beta, phi, theta, psi, p, q, r, bias1, bias2, ax, ay, az, alphaDot) = ((xs !! 0), (xs !! 1),(xs !! 2),(xs !! 3),(xs !! 4),(xs !! 5),(xs !! 6),(xs !! 7),(xs !! 8),(xs !! 9),(xs !! 10),(xs !! 11),(xs !! 12),(xs !! 13),(xs !! 14),(xs !! 15),(xs !! 16),(xs !! 17),(xs !! 19))
					 in ((cls_o xs) * (cos alpha) - (cns_o xs) * (sin alpha)))

cm_o :: [Float] -> Float
cm_o xs = (let (throttle, elevator, aileron, rudder, vt, alpha, beta, phi, theta, psi, p, q, r, bias1, bias2, ax, ay, az, alphaDot) = ((xs !! 0), (xs !! 1),(xs !! 2),(xs !! 3),(xs !! 4),(xs !! 5),(xs !! 6),(xs !! 7),(xs !! 8),(xs !! 9),(xs !! 10),(xs !! 11),(xs !! 12),(xs !! 13),(xs !! 14),(xs !! 15),(xs !! 16),(xs !! 17),(xs !! 19))
					 in (cms_o xs))

cn_o :: [Float] -> Float
cn_o xs = (let (throttle, elevator, aileron, rudder, vt, alpha, beta, phi, theta, psi, p, q, r, bias1, bias2, ax, ay, az, alphaDot) = ((xs !! 0), (xs !! 1),(xs !! 2),(xs !! 3),(xs !! 4),(xs !! 5),(xs !! 6),(xs !! 7),(xs !! 8),(xs !! 9),(xs !! 10),(xs !! 11),(xs !! 12),(xs !! 13),(xs !! 14),(xs !! 15),(xs !! 16),(xs !! 17),(xs !! 19))
					 in ((cls_o xs) * (sin alpha) + (cns_o xs) * (cos alpha)))

--Engine Function
xt :: Float -> Float
xt throttle = ((((throttle * 100)*(throttle * 100)) * xT2) + (xT1 * (throttle*100)) + (xT0))

-- AERODYNAMIC FORCES 
-- %S&C-based accelerometer readings
accelX_scd_o :: [Float] -> Float
accelX_scd_o xs = (let (throttle, elevator, aileron, rudder, vt, alpha, beta, phi, theta, psi, p, q, r, bias1, bias2, ax, ay, az, alphaDot) = ((xs !! 0), (xs !! 1),(xs !! 2),(xs !! 3),(xs !! 4),(xs !! 5),(xs !! 6),(xs !! 7),(xs !! 8),(xs !! 9),(xs !! 10),(xs !! 11),(xs !! 12),(xs !! 13),(xs !! 14),(xs !! 15),(xs !! 16),(xs !! 17),(xs !! 19))
									 in (((qBar vt) * s * (cxA_o xs) + (xt throttle)) / m))

accelY_scd_o :: [Float] -> Float
accelY_scd_o xs = (let (throttle, elevator, aileron, rudder, vt, alpha, beta, phi, theta, psi, p, q, r, bias1, bias2, ax, ay, az, alphaDot) = ((xs !! 0), (xs !! 1),(xs !! 2),(xs !! 3),(xs !! 4),(xs !! 5),(xs !! 6),(xs !! 7),(xs !! 8),(xs !! 9),(xs !! 10),(xs !! 11),(xs !! 12),(xs !! 13),(xs !! 14),(xs !! 15),(xs !! 16),(xs !! 17),(xs !! 19))
									 in ((qBar vt) * s * (cyA_o xs) / m))

accelZ_scd_o :: [Float] -> Float
accelZ_scd_o xs = (let (throttle, elevator, aileron, rudder, vt, alpha, beta, phi, theta, psi, p, q, r, bias1, bias2, ax, ay, az, alphaDot) = ((xs !! 0), (xs !! 1),(xs !! 2),(xs !! 3),(xs !! 4),(xs !! 5),(xs !! 6),(xs !! 7),(xs !! 8),(xs !! 9),(xs !! 10),(xs !! 11),(xs !! 12),(xs !! 13),(xs !! 14),(xs !! 15),(xs !! 16),(xs !! 17),(xs !! 19))
									 in ((qBar vt) * s * (czA_o xs) / m))


-- %accelerometer corresponding value (chose from scd-based or measurements)
accelX_o :: [Float] -> Float
accelX_o xs = (let (throttle, elevator, aileron, rudder, vt, alpha, beta, phi, theta, psi, p, q, r, bias1, bias2, ax, ay, az, alphaDot) = ((xs !! 0), (xs !! 1),(xs !! 2),(xs !! 3),(xs !! 4),(xs !! 5),(xs !! 6),(xs !! 7),(xs !! 8),(xs !! 9),(xs !! 10),(xs !! 11),(xs !! 12),(xs !! 13),(xs !! 14),(xs !! 15),(xs !! 16),(xs !! 17),(xs !! 19))
							 in (accelX_scd_o xs))
-- ;%accelX_meas_o;                     %[ft/s2]

accelY_o :: [Float] -> Float
accelY_o xs = (let (throttle, elevator, aileron, rudder, vt, alpha, beta, phi, theta, psi, p, q, r, bias1, bias2, ax, ay, az, alphaDot) = ((xs !! 0), (xs !! 1),(xs !! 2),(xs !! 3),(xs !! 4),(xs !! 5),(xs !! 6),(xs !! 7),(xs !! 8),(xs !! 9),(xs !! 10),(xs !! 11),(xs !! 12),(xs !! 13),(xs !! 14),(xs !! 15),(xs !! 16),(xs !! 17),(xs !! 19))
							 in (accelY_scd_o xs ))
-- ;%accelY_meas_o;                     %[ft/s2]

accelZ_o :: [Float] -> Float
accelZ_o xs = (let (throttle, elevator, aileron, rudder, vt, alpha, beta, phi, theta, psi, p, q, r, bias1, bias2, ax, ay, az, alphaDot) = ((xs !! 0), (xs !! 1),(xs !! 2),(xs !! 3),(xs !! 4),(xs !! 5),(xs !! 6),(xs !! 7),(xs !! 8),(xs !! 9),(xs !! 10),(xs !! 11),(xs !! 12),(xs !! 13),(xs !! 14),(xs !! 15),(xs !! 16),(xs !! 17),(xs !! 19))
							 in (accelZ_scd_o xs))
-- ;%accelZ_meas_o;                     %[ft/s2]

--mass
m = (weight/g)

-- BODY ACCELERATION COMPONENTS [ft/s2]
uDot :: [Float] -> Float
uDot xs = (let (throttle, elevator, aileron, rudder, vt, alpha, beta, phi, theta, psi, p, q, r, bias1, bias2, ax, ay, az, alphaDot) = ((xs !! 0), (xs !! 1),(xs !! 2),(xs !! 3),(xs !! 4),(xs !! 5),(xs !! 6),(xs !! 7),(xs !! 8),(xs !! 9),(xs !! 10),(xs !! 11),(xs !! 12),(xs !! 13),(xs !! 14),(xs !! 15),(xs !! 16),(xs !! 17),(xs !! 19))
					 in ((r*(v xs)) - (q*(w xs)) - (g * (sin theta)) + ax))
					 --(accelX_o vt alpha beta q alphaDot elevator throttle)))

vDot :: [Float] -> Float
vDot xs = (let (throttle, elevator, aileron, rudder, vt, alpha, beta, phi, theta, psi, p, q, r, bias1, bias2, ax, ay, az, alphaDot) = ((xs !! 0), (xs !! 1),(xs !! 2),(xs !! 3),(xs !! 4),(xs !! 5),(xs !! 6),(xs !! 7),(xs !! 8),(xs !! 9),(xs !! 10),(xs !! 11),(xs !! 12),(xs !! 13),(xs !! 14),(xs !! 15),(xs !! 16),(xs !! 17),(xs !! 19))
					 in ((p*(w xs)) - (r*(u xs)) + (g*(sin phi)*(cos theta)) + ay))
					 --(accelY_o vt beta p r aileron rudder))

wDot :: [Float] -> Float
wDot xs = (let (throttle, elevator, aileron, rudder, vt, alpha, beta, phi, theta, psi, p, q, r, bias1, bias2, ax, ay, az, alphaDot) = ((xs !! 0), (xs !! 1),(xs !! 2),(xs !! 3),(xs !! 4),(xs !! 5),(xs !! 6),(xs !! 7),(xs !! 8),(xs !! 9),(xs !! 10),(xs !! 11),(xs !! 12),(xs !! 13),(xs !! 14),(xs !! 15),(xs !! 16),(xs !! 17),(xs !! 19))
					 in ((q*(u xs)) - (p*(v xs)) + (g*(cos phi)*(cos theta)) + az))
					 --(accelZ_o vt alpha beta q alphaDot elevator))

qBar :: Float -> Float
qBar vt = ((1/2) * rho * (vt*vt))
