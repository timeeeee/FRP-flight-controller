module Nav_Functions
(gps_to_local
,nav_gainAcc
,local_to_gps
,local_speed
,true_wind
,y_nav
,x_est_nav
,k_nav_funct
,gps_rate_correction
,y_est_nav) where  

import Constants
import Matrix_Ops
import Common_Equations
import Data.List

-------------------------------
-- used by pre_ekf

-- gps_to_local
-- INPUT: xs - 
-- 								vt, alpha, beta												(0,1,2)
-- body rates			p, q, r																(3,4,5)
-- magnetic flux 	hx_body, hy_body, hz_body							(6,7,8)
-- pilot commands throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd (9,10,11,12)
-- accelerometer 	axb, ayb, azb													(13,14,15)
-- gpsPosition    lat, long, alt 												(16,17,18)
-- gpsVelocity		vNorth, vEast, vDown									(19,20,21)
-- enable				

--OUTPUT:
-- 								vt, alpha, beta												(0,1,2)
-- body rates			p, q, r																(3,4,5)
-- magnetic flux 	hx_body, hy_body, hz_body							(6,7,8)
-- pilot commands throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd (9,10,11,12)
-- accelerometer 	axb, ayb, azb													(13,14,15)
-- localPosition  North, East, Height										(16,17,18)
-- gpsVelocity		vNorth, vEast, vDown									(19,20,21)
-- enable																								(22)
gps_to_local :: [Float] -> [Float]
gps_to_local xs = (let (begin,end) = splitAt 16 xs
									 in begin ++ (lla_to_fe (take 3 end)) ++ (nav_gains (take 3 (drop 3 end))) ++ (drop 6 end))
									 --[0.0] ++ [0.0] ++ [0.0] ++

-- INPUT: gpsVelocity		vNorth, vEast, vDown						(0,1,2)
-- OUTPUT: adjusted 	vNorth, vEast, vDown							(0,1,2)	
nav_gains :: [Float] -> [Float]
nav_gains xs = ([(xs !! 0) * (1/0.3048)] ++	[(xs !! 1) * (1/0.3048)] ++	[((xs !! 2) * (-1)) * (1/0.3048)])

-- INPUT:
-- 								vt, alpha, beta												(0,1,2)
-- body rates			p, q, r																(3,4,5)
-- magnetic flux 	hx_body, hy_body, hz_body							(6,7,8)
-- pilot commands throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd (9,10,11,12)
-- accelerometer 	axb, ayb, azb													(13,14,15)
-- localPosition  North, East, Height										(16,17,18)
-- gpsVelocity		vNorth, vEast, vDown									(19,20,21)
-- enable																								(22)
-- OUTPUT: (same list with adjusted accelerations)									 
nav_gainAcc :: [Float] -> [Float]
nav_gainAcc xs = (let (begin,end) = splitAt 13 xs   
									in begin ++ [(g * (end !! 0))] ++ [g * (end !! 1)] ++ [g * (end !! 2)] ++ (drop 3 end))

--used by gps_to_local
-- INPUT: Latitude, Longitude, Altitude
-- OUPUT: North, East, Height
lla_to_fe :: [Float] -> [Float]
lla_to_fe xs = [((xs !! 0) - ((waypoints !! 0) !! 0))*((waypoints !! 11) !! 0)] ++ 
							 [((xs !! 1) - ((waypoints !! 1) !! 0))*((waypoints !! 12) !! 0)] ++
							 [((xs !! 2) - ((waypoints !! 2) !! 0))*(1/0.3048)]

------------------------------------
-- used by ekf

--INPUT: local position, local speed
--OUTPUT: pn,pe,ph (0,1,2)
--				vn,ve,vh (3,4,5)
y_nav :: [Float] -> [Float]
y_nav xs = xs

-- nav_ekf INPUT: xs 
-- 								vt, alpha, beta												(0,1,2)
-- body rates			p, q, r																(3,4,5)
-- magnetic flux 	hx_body, hy_body, hz_body							(6,7,8)
-- pilot commands throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd (9,10,11,12)
-- accelerometer 	(adj axb, ayb, azb)										(13,14,15)
-- localPosition  North, East, Height 									(16,17,18)
-- gpsVelocity		vNorth, vEast, vDown									(19,20,21)
-- enable																								(22)

-- INPUT: ys (previous nav input)
-- lla_est 				(lat_est, long_est, alt_est)					(0,1,2)
-- local_speed_est(vNorth_est,vEast_est,vDown_est)			(3,4,5)
-- wind_est				(wind0,wind1,wind2)										(6,7,8)

--OUTPUT: pn,pe,ph (0,1,2)
--				vn,ve,vh (3,4,5)
x_est_nav :: [Float]-> [Float] -> [Float]
x_est_nav xs ys = (vectorSum ys (gainDtNav (f_nav xs ys)))

gainDtNav :: [Float] -> [Float]
gainDtNav xs = (vectorScalarProduct dt xs)

f_nav :: [Float] -> [Float] -> [Float]
f_nav xs ys = [(ys !! 3)] ++ [(ys !! 4)] ++ [(ys !! 5)] ++ (f_vn xs) ++ (f_ve xs) ++ (f_vh xs)

f_vn :: [Float] -> [Float]
f_vn xs = (let (ax, ay, az, phi, theta, psi) = ((xs !! 13),(xs !! 14),(xs !! 15),(xs !! 6),(xs !! 7),(xs !! 8))
					 in [(ax*(cos theta)*(cos psi))+(ay*(((sin theta)*(sin phi)*(cos psi))-((cos phi)*(sin psi))))+((((sin phi)*(sin psi))+((sin theta)*(cos phi)*(cos psi)))*az)])

f_ve :: [Float] -> [Float]
f_ve xs = (let (ax, ay, az, phi, theta, psi) = ((xs !! 13),(xs !! 14),(xs !! 15),(xs !! 6),(xs !! 7),(xs !! 8))
					 in [((ax)*(cos theta)*(sin psi))+((((cos phi)*(cos psi))+((sin theta)*(sin phi)*(sin psi)))*(ay))+((((sin theta)*(cos phi)*(sin psi))-((sin phi)*(cos psi)))*(az))])

f_vh :: [Float] -> [Float]
f_vh xs = (let (ax, ay, az, phi, theta, psi) = ((xs !! 13),(xs !! 14),(xs !! 15),(xs !! 6),(xs !! 7),(xs !! 8))
					 in [(((ax)*(sin theta))-((cos theta)*(sin phi)*(ay))-((cos phi)*(cos theta)*(az)))-g])
-- (cos phi) (sin phi) (cos theta) (sin theta) (cos psi) (sin psi)])

-- OUTPUT: 6 x 6 matrix
k_nav_funct :: [Float] -> [Float] -> [[Float]]
k_nav_funct xs ys = k_nav

--INPUT: local position, local speed
--OUTPUT: pn,pe,ph (0,1,2)
--				vn,ve,vh (3,4,5)
y_est_nav :: [Float] -> [Float]
y_est_nav xs = xs

--INPUT: output from y_nav and result of addition of y_nav +- y_est_nav
gps_rate_correction :: [Float] -> [Float] -> [Float] -> [Float]
gps_rate_correction xs ys zs = (if (((xs !! 0) - (zs !! 0)) == 0)
																then [0.0,0.0,0.0,0.0,0.0,0.0]
																else ys)






-----------------------------------
--used by post_ekf

-- local_to_gps INPUT: 
-- xs (output from pre_ekf) 
-- 								vt, alpha, beta												(0,1,2)
-- body rates			p, q, r																(3,4,5)
-- magnetic flux 	hx_body, hy_body, hz_body							(6,7,8)
-- pilot commands throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd (9,10,11,12)
-- accelerometer 	(adj axb, ayb, azb)										(13,14,15)
-- localPosition  North, East, Height 									(16,17,18)
-- gpsVelocity		vNorth, vEast, vDown									(19,20,21)
-- enable																								(22)

-- OUTPUT: 
-- lla_est 				(lat_est, long_est, alt_est)					(0,1,2)

local_to_gps :: [Float] -> [Float]
local_to_gps xs = (let (begin,end) = splitAt 16 xs
									 in (local_to_lla (take 3 end)))

--INPUT: localPosition  North, East, Height 					(0,1,2)
--OUTPUT: gps_pos_est  Latitude,Longitude, Altitiude  (0,1,2)
local_to_lla :: [Float] -> [Float]
local_to_lla xs = [(xs !! 0)/((waypoints !! 11) !! 0) + ((waypoints !! 0) !! 0)] ++
									[(xs !! 1)/((waypoints !! 12) !! 0) + ((waypoints !! 1) !! 0)] ++
									[(xs !! 2) * 0.3048 + ((waypoints !! 2) !! 0)]

									
-- INPUT: xs (output from ekf)
-- lla_est 				(lat_est, long_est, alt_est)					(0,1,2)
-- local_speed_est(vNorth_est,vEast_est,vDown_est)			(3,4,5)
-- wind_est				(wind0,wind1,wind2)										(6,7,8)									 

--OUTPUT: local_speed_est(vNorth_est,vEast_est,vDown_est)			(0,1,2)

local_speed :: [Float] -> [Float]
local_speed xs = [(xs !! 3) * 0.3048] ++ [(xs !! 4) * 0.3048] ++ [(xs !! 5) * (-0.3048)]


--true_wind
-- 								vt, alpha, beta												(0,1,2)
-- body rates			p, q, r																(3,4,5)
-- magnetic flux 	hx_body, hy_body, hz_body							(6,7,8)
-- pilot commands throttle_cmd, elevator_cmd, aileron_cmd, rudder_cmd (9,10,11,12)
-- accelerometer 	(adj axb, ayb, azb)										(13,14,15)
-- localPosition  North, East, Height 												(16,17,18)
-- gpsVelocity		vNorth, vEast, vDown									(19,20,21)
-- enable																								(22)

-- INPUT: ys (output from ekf)
-- lla_est 				(lat_est, long_est, alt_est)					(0,1,2)
-- local_speed_est(vNorth_est,vEast_est,vDown_est)			(3,4,5)
-- wind_est				(wind0,wind1,wind2)										(6,7,8)

								
-- OUPUT: wind_est				(wind0,wind1,wind2)						(0,1,2)
true_wind :: [Float] -> [Float] -> [Float]
true_wind xs ys = [0.0,0.0,0.0]