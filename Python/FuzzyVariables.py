################################# IMPORT PACKAGES #################################
from Memberships import MembershipFunction

#################################### VARIABLES ####################################
MemF = MembershipFunction()
midSlow, midMedium, midFast = MemF.GetMidPoints()
zero = 0

## Format: Lxxx is left wheel; Rxxx is right wheel.
##          xxx is the sensor readings which equate to the shapes defined above. C = close; I = ideal; F = far
## Example: a variable lcfi represents:
# 							l x x x --> left wheel
# 							x c x x --> object in close proximity to right sensor 
# 							x x f x --> object in far proximity to front sensors
# 							x x x i --> object in ideal proximity to left sensor 

# Define the rules for Right Edge - two sensors available on that side
lcc = midSlow; lci = midSlow; lcf = midSlow
lic = midSlow; lii = midSlow; lif = midSlow
lfc = midFast; lfi = midMedium; lff = midMedium
l_close_RE  = [lcc, lci, lcf]
l_ideal_RE  = [lic, lii, lif]
l_far_RE    = [lfc, lfi, lff]
l_RE = [l_close_RE, l_ideal_RE, l_far_RE]

rcc = midFast; rci = midFast; rcf = midFast
ric = midMedium; rii = midSlow; rif = midMedium
rfc = midMedium; rfi = midSlow; rff = midSlow
r_close_RE  = [rcc, rci, rcf]
r_ideal_RE  = [ric, rii, rif]
r_far_RE    = [rfc, rfi, rff]
r_RE = [r_close_RE, r_ideal_RE, r_far_RE]

# Define the rules for Obstacle Avoidance  - uses side and front sensors
lccc = zero; lcci = midSlow; lccf = midSlow
lcic = midMedium; lcii = midSlow; lcif = midSlow
lcfc = midMedium; lcfi = midSlow; lcff = midSlow
licc = midMedium; lici = midSlow; licf = midSlow
liic = midFast; liii = midSlow; liif = midSlow
lifc = midFast; lifi = midMedium; liff = midSlow
lfcc = midFast; lfci = midFast; lfcf = midSlow
lfic = midFast; lfii = midFast; lfif = midSlow
lffc = midFast; lffi = midFast; lfff = midMedium
l_close_OA = [lccc, lcci, lccf, lcic, lcii, lcif, lcfc, lcfi, lcff]
l_ideal_OA = [licc, lici, licf, liic, liii, liif, lifc, lifi, liff]
l_far_OA = [lfcc, lfci, lfcf, lfic, lfii, lfif, lffc, lffi, lfff]
l_OA = [l_close_OA, l_ideal_OA, l_far_OA]

rccc = midFast; rcci = midFast; rccf = midFast
rcic = midMedium; rcii = midFast; rcif = midFast
rcfc = midMedium; rcfi = midFast; rcff = midFast
ricc = midSlow; rici = midFast; ricf = midFast
riic = midSlow; riii = midSlow; riif = midFast
rifc = midSlow; rifi = midMedium; riff = midFast
rfcc = midSlow; rfci = midSlow; rfcf = midFast
rfic = midSlow; rfii = midMedium; rfif = midFast
rffc = midSlow; rffi = midSlow; rfff = midMedium
r_close_OA = [rccc, rcci, rccf, rcic, rcii, rcif, rcfc, rcfi, rcff]
r_ideal_OA = [ricc, rici, ricf, riic, riii, riif, rifc, rifi, riff]
r_far_OA = [rfcc, rfci, rfcf, rfic, rfii, rfif, rffc, rffi, rfff]
r_OA = [r_close_OA, r_ideal_OA, r_far_OA]