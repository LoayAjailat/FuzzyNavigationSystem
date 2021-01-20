class MembershipParameters():
	def __init__(self, a, b, c, d):
		self.a = a
		self.b = b
		self.c = c
		self.d = d

class MembershipFunction():
	def __init__(self):
		# Distance memberships
		self.close = 0
		self.ideal = 0
		self.far = 0
		# Objective memberships
		self.d_GS = 0
		self.d_OA = 0
		self.d_RE = 0 
		# Speed memberships
		self.slow = 0
		self.medium = 0
		self.fast = 0
		# Initialise shapes
		self.MembershipShapes()
		
	# Calculates the membership degree of the arg dist (distance from object to sensor)
	def CalcMembershipDegree(self, dist, MF):
		try:
			# Calculate at rising edge
			if (MF.a <= dist and dist < MF.b):
				return (dist - MF.a) / (MF.b - MF.a)
			# Return 1 if its a horizontal line
			elif (MF.b <= dist and dist <= MF.c):
				return 1.0
			# Calculate at falling edge
			elif (MF.c < dist and dist <= MF.d):
				return (MF.d - dist) / (MF.d - MF.c)
			else:
				return 0.0
		except Exception as e:
			print("Could not calculate membership degree due to input error. Please check input.\nDefaulting to 0.0")
			print(f"ERROR: {repr(e)}")
			return 0.0

	# Defines the shapes of the membership functions
	def MembershipShapes(self):
		# Shape for close distances
		self.close = MembershipParameters(0, 100, 275, 450)
		self.ideal = MembershipParameters(400, 575, 575, 750)
		self.far   = MembershipParameters(725, 1100, 1475, 5000)

		# Shape for obstacle avoidance
		self.d_OA = MembershipParameters(0, 0, 750, 2000)
		# First shape for Right Edge
		self.d_RE = MembershipParameters(0, 0, 800, 2500)
		# Second shape for Goal Seeking
		self.d_GS = MembershipParameters(1800, 3000, 5000, 5000)

		# Shape for slow speeds
		self.slow = MembershipParameters(0, 40, 60, 125)
		# Shape for medium speeds
		self.medium = MembershipParameters(80, 140, 160, 200)
		# Shape for fast speeeds
		self.fast = MembershipParameters(160, 200, 240, 280)
	
	# Calculates the mid-point between 2 values
	def CalcMidPoint(self, a, b):
		try:
			return (a + b) /2
		except:
			print("Error calculating the mid-point. Check your input value.\nDefaulting to 10")
			return 10.0

	# Gets the mid-points for the speed shapes
	def GetMidPoints(self):
		midSlow = self.CalcMidPoint(self.slow.b, self.slow.c)
		midMedium = self.CalcMidPoint(self.medium.b, self.medium.c)
		midFast = self.CalcMidPoint(self.fast.b, self.fast.c)
		return midSlow, midMedium, midFast