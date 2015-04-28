next_f :: [Float] -> [Float]
next_f xs = uvw (xs !! 4) (xs !! 5) (xs !! 6) (xs !! 7) (xs !! 8) (xs !! 9)(xs !! 10) (xs !! 11) (xs !!12) (xs !! 0)

uvw :: Float -> Float -> Float -> Float -> Float -> Float -> Float -> Float -> Float -> [Float]
uvw vt alpha beta phi theta psi p q r throttle = (let (u_o, v_o, w_o) = ((u vt alpha beta), (v vt beta), (w vt alpha beta))
																									in uvw_dot u_o v_o w_o phi theta psi p q r throttle)
																				 
uvw_dot :: Float -> Float -> Float -> Float -> Float -> Float -> Float -> Float -> Float -> [Float]																			 
uvw_dot u_o v_o w_o phi theta psi p q r throttle = (let (u_dot, v_dot, w_dot) = ((uDot v_o w_o q r theta throttle xa),(vDot),(wDot))
																										in 




constants:
--a_W1 
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
,xw340_sim


												 --(map (add (fromList (map fromList (matrixProduct (map toList three) (transpose (map toList (part2 xs (part1 two)))))))) (part1 two )) ++ (drop 15 two))

												 
toList :: Float -> [Float]
toList f = [f]
												 
fromList :: [Float] -> Float
fromList xs = (xs !! 0)							
					 
part1 :: [Float] -> [Float]
part1 xs = (drop 4 (take 18 xs))

-- xs
-- 								(vt, alpha, beta)											(0,1,2)
-- body rates			(p, q, r)															(3,4,5)
-- magnetic flux 	(hx_body, hy_body, hz_body)						(6,7,8)

-- ys
--fTHL_o,fEL_o,fAIL_o,fRDR_o, (0,1,2,3)
--fVT_o,falpha_o,fbeta_o,			(4,5,6)
--fphi_o,ftheta_o,fpsi_o,			(7,8,9)
--fP_o,fQ_o,fR_o,							(10,11,12)
--falphab_o,fbetab_o,					(13,14)
--accelX_o,accelY_o,accelZ_o	(15,16,17)
--alphaDot, alphadot_o_1,			(18,19)
--p_est,enable								(20,21)

-- to vectorMinus-
-- 		hVT_o,halpha_o,hbeta_o,					(0,1,2)
-- 		hP_o,hQ_o,hR_o,									(3,4,5)
-- 		hHxbody_o,hHybody_o,hHzbody_o 	(6,7,8)
subtractAdjusted :: [Float] -> [Float] -> [Float]
subtractAdjusted xs ys = vectorMinus xs (y_est_ ys) two




										 
							

							
							





-- DONE y_est_ 
-- INPUT: output from e_est_
--		fVT_o,falpha_o,fbeta_o,					(0,1,2)
--		fphi_o,ftheta_o,fpsi_o,					(3,4,5)
--		fP_o,fQ_o,fR_o,									(6,7,8)
--		falphab_o,fbetab_o,							(9,10)
--		accelX_o,accelY_o,accelZ_o			(11,12,13)
-- OUTPUT: output from h_function
-- 		hVT_o,halpha_o,hbeta_o,					(0,1,2)
-- 		hP_o,hQ_o,hR_o,									(3,4,5)
-- 		hHxbody_o,hHybody_o,hHzbody_o 	(6,7,8)
y_est_ :: [Float] -> [Float]
y_est_ xs = (h_function xs)












-

							 
							 
							 












-- DONE h_function:
-- INPUTS:
--		fVT_o,falpha_o,fbeta_o,					(0,1,2)
--		fphi_o,ftheta_o,fpsi_o,					(3,4,5)
--		fP_o,fQ_o,fR_o,									(6,7,8)
--		falphab_o,fbetab_o,							(9,10)
--		accelX_o,accelY_o,accelZ_o			(11,12,13)

-- OUTPUTS:
-- 		hVT_o,halpha_o,hbeta_o,					(0,1,2)
-- 		hP_o,hQ_o,hR_o,									(3,4,5)
-- 		hHxbody_o,hHybody_o,hHzbody_o 	(6,7,8)
h_function :: [Float] -> [Float]
h_function xs = [hVT_o xs] ++ [halpha_o xs] ++ [hbeta_o xs] ++ [hP_o xs] ++ [hQ_o xs] ++ [hR_o xs] ++ [hHxbody_o xs] ++ [hHybody_o xs] ++ [hHzbody_o xs]

hVT_o :: [Float] -> Float
hVT_o xs = (xs !! 0)

halpha_o :: [Float] -> Float
halpha_o xs =(xs !! 1) + (xs !! 9)

hbeta_o :: [Float] -> Float
hbeta_o xs = (xs !! 2) + (xs !! 10)

hP_o :: [Float] -> Float
hP_o xs = (xs !! 6)

hQ_o :: [Float] -> Float
hQ_o xs = (xs !! 7)

hR_o :: [Float] -> Float
hR_o xs = (xs !! 8)

hHxbody_o :: [Float] -> Float
hHxbody_o xs = (let (hN0noaa, hE0noaa, hD0noaa, theta_o, psi_o) = ((xs !! 6),(xs !! 7),(xs !! 8),(xs !! 4),(xs !! 5))
								in (hN0noaa*cos(theta_o)*cos(psi_o))+(hE0noaa*cos(theta_o)*sin(psi_o))-(hD0noaa*sin(theta_o)))

hHybody_o :: [Float] -> Float								
hHybody_o xs = (let (hN0noaa, hE0noaa, hD0noaa, theta_o, psi_o, phi_o) = ((xs !! 6),(xs !! 7),(xs !! 8),(xs !! 4),(xs !! 5),(xs !! 3))
								in (hN0noaa*(sin(phi_o)*sin(theta_o)*cos(psi_o)-cos(phi_o)*sin(psi_o)))+(hE0noaa*(sin(phi_o)*sin(theta_o)*sin(psi_o)+cos(phi_o)*cos(psi_o)))+(hD0noaa*sin(phi_o)*cos(theta_o)))

hHzbody_o :: [Float] -> Float
hHzbody_o xs = (let (hN0noaa, hE0noaa, hD0noaa, theta_o, psi_o, phi_o) = ((xs !! 6),(xs !! 7),(xs !! 8),(xs !! 4),(xs !! 5),(xs !! 3))
								in (hN0noaa*(cos(phi_o)*sin(theta_o)*cos(psi_o)+sin(phi_o)*sin(psi_o)))+(hE0noaa*(cos(phi_o)*sin(theta_o)*sin(psi_o)-sin(phi_o)*cos(psi_o)))+(hD0noaa*cos(phi_o)*cos(theta_o)))
