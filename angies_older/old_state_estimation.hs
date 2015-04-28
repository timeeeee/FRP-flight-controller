

-- helper functions for stateObserver 
-- ** DONE ekf INPUT xs - current time t: 						(index address into list)
-- 								vt, alpha, beta												(0,1,2)
-- body rates			p, q, r																(3,4,5)
-- magnetic flux 	hx_body, hy_body, hz_body							(6,7,8)
-- pilot commands throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd (9,10,11,12)
-- accelerometer 	axb, ayb, azb													(13,14,15)
-- gpsPosition    lat, long, alt 												(16,17,18)
-- gpsVelocity		vNorth, vEast, vDown									(19,20,21)


-- ys - previous stateObserver ouput - time t-1					(index address into list)
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)
--								p_est, enable													(20,21)


-- zs - t-1 list of previous measurements			(index address into list)
-- 								vt, alpha, beta												(0,1,2)
-- body rates			p, q, r																(3,4,5)
-- magnetic flux 	hx_body, hy_body, hz_body							(6,7,8)
-- servo commands throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd (9,10,11,12)
-- accelerometer 	axb, ayb, azb													(13,14,15)
-- gpsPosition    lat, long, alt 												(16,17,18)
-- gpsVelocity		vNorth, vEast, vDown									(19,20,21)


-- ekf OUTPUT:																		(index address into list)
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)
--								p_est, enable													(20,21)

ekf :: [Float] -> [Float] -> [Float] -> [Float]	
ekf xs [] [] = (zeroVector 22)
ekf xs ys zs = (let (one, two, three) = ((y_ xs),(x_est_ ys zs),(k_function ys zs))
								in decompose_ekf one two three)
								
--	(let ((beginX, endX),(beginZ, endZ)) = ((splitAt 8 xs),(splitAt 8 zs))
--								in (decomposeEkf beginX (take 7 endZ) ys))
								
								
								

												

								

								
	
	.
	.
	.
	.
	.
	.
	.
	.
	.
	.
	.
	.
	.
	.
	.
	.
	.
	.
	.
	.
-- ** DONE decomposeEkf INPUTS:xs - current time t: 						(index address into list)
-- 								vt, alpha, beta												(0,1,2)
-- body rates			p, q, r																(3,4,5)
-- magnetic flux 	hx_body, hy_body, hz_body							(6,7,8)

-- ys - t-1 list of previous measurements								(index address into list)
-- servo commands throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd (0,1,2,3)
-- accelerometer 	axb, ayb, azb													(4,5,6)

-- zs - previous stateObserver ouput - time t-1					(index address into list)
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)
--								p_est, enable													(20,21)

-- decomposeEkf OUTPUT:																		(index address into list)
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)
--								p_est, enable													(20,21)
				
decomposeEkf :: [Float] -> [Float] -> [Float] -> [Float]	
decomposeEkf xs ys zs = zs
--(let (one, two, three) = ((y_ xs),(x_est_ ys zs),(k_function ys zs))
--												 in decompose2 one two three ) 
	
----------------------------
-- DECOMPOSE EKF REDUCTION FUNCITONS
	
-- ** DONE _y
-- 		INPUT: xs - current time t: 						(index address into list)
-- 								vt, alpha, beta												(0,1,2)
-- body rates			p, q, r																(3,4,5)
-- magnetic flux 	hx_body, hy_body, hz_body							(6,7,8)

-- 		OUTPUT: 										 						(index address into list)
-- 								vt, alpha, beta												(0,1,2)
-- body rates			p, q, r																(3,4,5)
-- magnetic flux 	hx_body, hy_body, hz_body							(6,7,8)
-- this function is just to stay consistant with the matching block in Simulink
y_ :: [Float] -> [Float]
y_ xs = xs 



-- ** DONE x_est_
-- INPUT: xs - t-1 list of previous measurements		(index address into list)
-- servo commands (throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd) (0,1,2,3)
-- accelerometer 	(axb, ayb, azb)												(4,5,6)

-- ys - previous stateObserver ouput - time t-1					(index address into list)
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)
--								p_est, enable													(20,21)

-- OUTPUT:
--								fTHL_o,fEL_o,fAIL_o,fRDR_o, 					(0,1,2,3)
--								fVT_o,falpha_o,fbeta_o,								(4,5,6)
--								fphi_o,ftheta_o,fpsi_o,								(7,8,9)
--								fP_o,fQ_o,fR_o,												(10,11,12)
--								falphab_o,fbetab_o,										(13,14)
--								accelX_o,accelY_o,accelZ_o						(15,16,17)
--								alphaDot, alphadot_o_1,								(18,19)
--								p_est,enable													(20,21)
		
x_est_ :: [Float] -> [Float] -> [Float]
x_est_ xs ys = ys
--(gainDt (f_function xs ys) xs)

---------
-- COMPONENTS OF X_EST_

-- ** DONE gainDt - this adds dt to the f_x subset of xs and then vector adds the result to xs
--INPUT: 				xs
--								fTHL_o,fEL_o,fAIL_o,fRDR_o, 					(0,1,2,3)
--								fVT_o,falpha_o,fbeta_o,								(4,5,6)
--								fphi_o,ftheta_o,fpsi_o,								(7,8,9)
--								fP_o,fQ_o,fR_o,												(10,11,12)
--								falphab_o,fbetab_o,										(13,14)
--								accelX_o,accelY_o,accelZ_o						(15,16,17)
--								alphaDot, alphadot_o_1,								(18,19)
--								p_est,enable													(20,21)
--							ys
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)
--								p_est, enable													(20,21)
--OUTPUT: 
gainDt :: [Float] -> [Float] -> [Float] 
gainDt xs ys = --split at bias and rebuild
							(let ((beginX, endX),(beginY, endY)) = ((splitAt 14 xs),(splitAt 14 ys))
  						 in (vectorSum beginX beginY) ++ endX)

-- f_function 
-- **NOT COMPLETE

-- INPUT: xs - t-1 list of previous measurements		(index address into list)
-- servo commands (throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd) (0,1,2,3)
-- accelerometer 	(axb, ayb, azb)												(4,5,6)

-- ys - previous stateObserver ouput - time t-1					(index address into list)
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)
--								p_est, enable													(20,21)

-- PARAMETERS                  
--				constant rho,...
--				constant S,Cbar,b,weight,g,IxxB,IyyB,IzzB,IxzB,...
--				constant CD0,CDu,CDa,CDq,CDadot,CDde,...
--				constant CD0_bar,...
--				constant Cyb,Cyp,Cyr,Cyda,Cydr,...
--				constant CL0,CLa,CLq,CLadot,CLu,CLde,...
--				constant Clb,Clp,Clr,Clda,Cldr,...
--				constant Cm0,Cma,Cmq,Cmadot,Cmu,Cmde,...
--				constant Cnb,Cnp,Cnr,Cnda,Cndr,...
--				constant xT0,xT1,xT2,...
--				constant Ptrim,Qtrim,Rtrim,Utrim,...
--				constant Athrottle,Bthrottle,...
--				constant Aelevator,Belevator,...
--				constant Aaileron,Baileron,...
--				constant Arudder,Brudder,...
--				constant ThrottleMax,ThrottleMin,...
--				constant ElevatorMax,ElevatorMin,...
--				constant AileronMax,AileronMin,...
--				constant RudderMax,RudderMin)

-- OUTPUT:
--						fTHL_o,fEL_o,fAIL_o,fRDR_o, 						(0,1,2,3)
--						fVT_o,falpha_o,fbeta_o,									(4,5,6)
--						fphi_o,ftheta_o,fpsi_o,									(7,8,9)
--						fP_o,fQ_o,fR_o,													(10,11,12)
--						falphab_o,fbetab_o,											(13,14)
--						accelX_o,accelY_o,accelZ_o							(15,16,17)
--						alphaDot, alphadot_o_1,									(18,19)
--						p_est, enable														(20,21)

-- ** note: this is porting the correct size of output, taken from input values
-- needs common equations module added and used to build up output list composed of 
-- results of functions
f_function :: [Float] -> [Float] -> [Float]
f_function [] [] = []
f_function xs ys = ys							 
							 
							 
-- END COMPONENTS X_EST_							 
---------

-- k_function
-- **NOT COMPLETE
-- INPUT:	xs - t-1 list of previous measurements								(index address into list)
-- servo commands (throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd) (0,1,2,3)
-- accelerometer 	(axb, ayb, azb)												(4,5,6)

-- ys - previous stateObserver ouput - time t-1					(index address into list)
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)
--								p_est, enable													(20,21)

-- OUTPUT: size like: -- OUTPUTS
-- 								hVT_o,halpha_o,hbeta_o,								(0,1,2)
-- 								hP_o,hQ_o,hR_o,												(3,4,5)
-- 								hHxbody_o,hHybody_o,hHzbody_o 				(6,7,8)

k_function :: [Float] -> [Float] -> [Float]

k_function xs [] = xs
k_function xs ys = (zeroVector 9)


------------------------------------------------------------------------------


-- ** not DONE decompose2 INPUT
--							xs-output of y_
-- 								vt, alpha, beta												(0,1,2)
-- body rates			p, q, r																(3,4,5)
-- magnetic flux 	hx_body, hy_body, hz_body							(6,7,8)

--							ys-output of x_est_
--								fTHL_o,fEL_o,fAIL_o,fRDR_o, 					(0,1,2,3)
--								fVT_o,falpha_o,fbeta_o,								(4,5,6)
--								fphi_o,ftheta_o,fpsi_o,								(7,8,9)
--								fP_o,fQ_o,fR_o,												(10,11,12)
--								falphab_o,fbetab_o,										(13,14)
--								accelX_o,accelY_o,accelZ_o						(15,16,17)
--								alphaDot, alphadot_o_1,								(18,19)
--								p_est,enable													(20,21)

--							zs-output of k_function
-- 								hVT_o,halpha_o,hbeta_o,								(0,1,2)
-- 								hP_o,hQ_o,hR_o,												(3,4,5)
-- 								hHxbody_o,hHybody_o,hHzbody_o 				(6,7,8)

												 
-- decompose2 OUTPUT:																		(index address into list)
-- 								throttle, elevator, aileron, rudder 	(0,1,2,3)
--								vt_est, alpha_est, beta_est, 					(4,5,6)
--  							phi_est, theta_est, psi_est, 					(7,8,9)
--								p_est, q_est, r_est, 									(10,11,12)
--								biases_est													  (13,14)
--								axb, ayb, azb, 												(15,16,17)
--								alphaDot, alphaDot1										(18,19)
--								p_est, enable													(20,21)												 
decompose2 :: [Float] -> [Float] -> [Float] -> [Float]	
decompose2 xs ys zs = ys

------------------------------------------------------
-- DECOMPOSE 2 REDUCTION FUNCTIONS



