"""
[project] 		Fuzzy system controlled robot

[brief summary] The project aims to design a Fuzzy system capable of driving the Pioneer 3-DX intelligent mobile robot
				using the ARIA API. The robot's objectives are:
					1. Follow a right-edge wall (using sonar sensors on the right-hand side)
					2. Avoid any obstacles presented within the robot's FOV (using sonar sensors on the front)
					3. If no wall is detected on the right-side and no obstacles are present, then the robot 
					   would seek a right-edge wall (Goal seeking).
				
[notes]			The ARIA API requires manual installation and setup. It cannot be installed through pip.
				ARIA allows for Pioneer 3-DX simulation in MobileSim.
				
[author]        Loay Ajailat
"""
################################# IMPORT PACKAGES #################################
from Memberships import *
from FuzzyVariables import *
import sys
import imp
import logging
logging.basicConfig(filename=f'./ErrorLog.log', filemode='a', 
			format='%(asctime)s - %(name)s - %(levelname)s - %(message)s', level=logging.DEBUG)
# Try locating the ARIA API if installed
try:
	imp.find_module('AriaPy')
	found = True
except ImportError:
	found = False
	print("ARIA API not found.")
	logging.error("ARIA API not found.")
if found:
	# Try importing the library
	try:
		from AriaPy import *
	except:
		print("Error importing ARIA")
###################################### CLASS ######################################
class FuzzySystem():
	def __init__(self):
		self.robot = ""
		
	# Initialise connection to robot
	def initRobot(self):
		try:
			# Global library initialization
			Aria.init()
			argparser = ArArgumentParser(sys.argv)
			argparser.loadDefaultArguments()
			# Create a robot object
			self.robot = ArRobot()
			# Connect to robot
			conn = ArRobotConnector(argparser, self.robot)
			if (not conn.connectRobot(self.robot)):
				print ('Error connecting to robot')
				Aria.logOptions()
				print ('Could not connect to robot, exiting.')
				Aria.exit(1)
			print('Connection successful.')
			self.robot.addSensorInterpTask(self.PrintPose)
			# Add sonar device
			sonar = ArSonarDevice()
			self.robot.addRangeDevice(sonar)
			# Enable robot
			self.robot.runAsync(False)
			self.robot.lock()
			self.robot.enableMotors()
			self.robot.unlock()

			return True
		except Exception as e:
			print(f"ERROR: {repr(e)}")
			logging.error(repr(e))
			return False

	# Gets the robot's pose
	def getPose(self):
		return self.robot.getPose()

	# Prints the robot's pose to terminal
	def PrintPose(self):
		# print (f"Coordinates --> X: {self.robot.getX()}, Y: {self.robot.getY()}, Theta: {self.robot.getTh()}, Vel: {self.robot.getVel()}")
		print(self.getPose())

	# Fetches the sonar sensor readings
	def GetSonarReadings(self):
		sonarRange = [0,0,0,0,0,0,0]
		try:
			for i in range(len(sonarRange)):
				sonarSensor[i] = self.robot.getSonarReading(i)
				sonarRange[i] = sonarSensor[i].getRange()
		except Exception as e:
			print("Failed to fetch sensor readings.\nDefaulting to arbitrary values.")
			# logging.error("Failed to fetch sensor readings.\nDefaulting to arbitrary values.")
			# logging.error(repr(e))
			sonarRange = [2877, 1253, 1747, 425, 400, 1000, 1200, 100]
		return sonarRange

	# Truncates values by subtracting 1
	def TruncateValue(self, val, maxVal):
		if val == maxVal:
			val -= 1
			# Make sure value is not negative
			if val < 0:
				val = 0
		return val

	# Calculate the firing strength 
	def CalcFiringStrength(self, sensors, array):
		# Check if array is less than the correct size
		if len(array) < 3:
			print("Array size is incorrect. Defaulting value to 0.0")
			return 0.0

		if len(sensors) == 2:
			a = 0
			b = 0

			a_degClose = array[0]
			a_degIdeal = array[1]
			a_degFar   = array[2]
			d_Degrees = {'c':[a_degClose[0],a_degClose[1]], 
						'i':[a_degIdeal[0], a_degIdeal[1]], 
						'f':[a_degFar[0], a_degFar[1]]}

			a = d_Degrees[sensors[0]][0]
			b = d_Degrees[sensors[1]][1]
			try:
				f = a * b
				return f
			except Exception as e:
				print(f"ERROR: {repr(e)}")
				logging.error(repr(e))
				print("Defaulting to 0.0")
				return 0.0		

		elif len(sensors) == 3:
			a = 0
			b = 0
			c = 0
			
			a_degClose = array[0]
			a_degIdeal = array[1]
			a_degFar   = array[2]
			d_Degrees = {'c':[a_degClose[0],a_degClose[1], a_degClose[2]], 
						'i':[a_degIdeal[0], a_degIdeal[1], a_degIdeal[2]], 
						'f':[a_degFar[0], a_degFar[1], a_degFar[2]]}

			a = d_Degrees[sensors[0]][0]
			b = d_Degrees[sensors[1]][1]
			c = d_Degrees[sensors[2]][2]
			try:
				f = a * b * c
				return f
			except Exception as e:
				logging.error(repr(e))
				print(f"ERROR: {repr(e)}")
				print("Defaulting to 0.0")
				return 0.0

	# Defuzzify the given values
	def Defuzzify(self, rule, numSensors, firingStrength, speeds):
		v = speeds
		f = firingStrength
		ruleDict = {'close':0, 'ideal':1, 'far': 2}
		
		# Check if rule exists in the dictionary
		if rule in ruleDict:
			ruleIndex = ruleDict[rule]
			if numSensors == 2:
				result = 0
				for i in range(3):
					result += f[ruleIndex][i] * v[ruleIndex][i]
				return result
			elif numSensors == 3:
				result = 0
				for i in range(9):
					result += f[ruleIndex][i] * v[ruleIndex][i]
				return result
		else:
			print("Rule does not exist. Check your input arg.")
			return 0

	# Calculate the defuzzified speed
	def GetDefuzzifiedSpeed(self, numSens, firingStrength, rules):
		# Defuzzify the values
		close = self.Defuzzify('close', numSens, firingStrength, rules)
		ideal = self.Defuzzify('ideal', numSens, firingStrength, rules)
		far   = self.Defuzzify('far', numSens, firingStrength, rules)

		# Calculate the sum of the firing strengths
		sumMem = 0
		for i in range(len(firingStrength)):
			sumMem += sum(firingStrength[i])
		
		# Calculate the speed
		if sumMem != 0: # Check if not 0, to prevent division by 0
			speed = (close + ideal + far) / sumMem
			return speed
		else:
			return 0

	# Main loop
	def run(self):
		# Initialise robot
		## NOTE: if unable to connect to the robot, comment this section so that you 
		## 		can debug and preview the results of the below functions & calculations.
		b_init = self.initRobot()
		if not b_init:
			print("Failed to communicate with robot")
			return

		# Define membership object
		MemF = MembershipFunction()

		while (True):
			# Get sonar readings
			sonarRange = self.GetSonarReadings()

			sonar7 = sonarRange[7]
			sonar6 = sonarRange[6]
			sonar5 = sonarRange[5]
			sonar4 = sonarRange[4]
			sonar3 = sonarRange[3]
			sonar2 = sonarRange[2]
			sonar1 = sonarRange[1]
			sonar0 = sonarRange[0]
			# Get minimum value between front 2 sensors
			sonarFront = min(sonar3, sonar4)
			sonarList = [sonarFront, sonar2, sonar3, sonar4, sonar5, sonar6, sonar7]

			# Truncate values to 4999 to prevent zero division
			for s in range(len(sonarList)):
				sonarList[s] = self.TruncateValue(sonarList[s], 5000)
			
			# Membership Degree for Right Edge Following
			degreeClose6 = MemF.CalcMembershipDegree(sonar6, MemF.close)
			degreeClose7 = MemF.CalcMembershipDegree(sonar7, MemF.close)
			degreeIdeal6 = MemF.CalcMembershipDegree(sonar6, MemF.ideal)
			degreeIdeal7 = MemF.CalcMembershipDegree(sonar7, MemF.ideal)
			degreeFar6 	 = MemF.CalcMembershipDegree(sonar6, MemF.far)
			degreeFar7	 = MemF.CalcMembershipDegree(sonar7, MemF.far)

			a_degClose_RE = [degreeClose6, degreeClose7]
			a_degIdeal_RE = [degreeIdeal6, degreeIdeal7]
			a_degFar_RE   = [degreeFar6, degreeFar7]
			a_Combined_RE = [a_degClose_RE, a_degIdeal_RE, a_degFar_RE]

			# Firing Strengths for Right Edge Following - 9 rules (3x3)
			cc = self.CalcFiringStrength('cc', a_Combined_RE)
			ci = self.CalcFiringStrength('ci', a_Combined_RE)
			cf = self.CalcFiringStrength('cf', a_Combined_RE)
			ic = self.CalcFiringStrength('ic', a_Combined_RE)
			ii = self.CalcFiringStrength('ii', a_Combined_RE)
			If = self.CalcFiringStrength('if', a_Combined_RE)
			fc = self.CalcFiringStrength('fc', a_Combined_RE)
			fi = self.CalcFiringStrength('fi', a_Combined_RE)
			ff = self.CalcFiringStrength('ff', a_Combined_RE)

			f_close_RE = [cc, ci, cf]
			f_ideal_RE = [ic, ii, If]
			f_far_RE   = [fc, fi, ff]
			f_Combined_RE = [f_close_RE, f_ideal_RE, f_far_RE]

			# Membership Degrees for obstacle avoidance
			degreeClose_LFS = MemF.CalcMembershipDegree(sonar2, MemF.close)
			degreeClose_MFS = MemF.CalcMembershipDegree(sonarFront, MemF.close)
			degreeClose_RFS = MemF.CalcMembershipDegree(sonar5, MemF.close)
			degreeIdeal_LFS = MemF.CalcMembershipDegree(sonar2, MemF.ideal)
			degreeIdeal_MFS = MemF.CalcMembershipDegree(sonarFront, MemF.ideal)
			degreeIdeal_RFS = MemF.CalcMembershipDegree(sonar5, MemF.ideal)
			degreeFar_LFS 	= MemF.CalcMembershipDegree(sonar2, MemF.far)
			degreeFar_MFS 	= MemF.CalcMembershipDegree(sonarFront, MemF.far)
			degreeFar_RFS 	= MemF.CalcMembershipDegree(sonar5, MemF.far)

			a_degClose_OA = [degreeClose_RFS, degreeClose_MFS, degreeClose_LFS]
			a_degIdeal_OA = [degreeIdeal_RFS, degreeIdeal_MFS, degreeIdeal_LFS]
			a_degFar_OA   = [degreeFar_RFS, degreeFar_MFS, degreeFar_LFS]
			a_Combined_OA = [a_degClose_OA, a_degIdeal_OA, a_degFar_OA]

			# Firing Strengths for Obstacle Avoidance - 27 rules (3x3x3)
			ccc = self.CalcFiringStrength('ccc', a_Combined_OA)
			cci = self.CalcFiringStrength('cci', a_Combined_OA)
			ccf = self.CalcFiringStrength('ccf', a_Combined_OA)
			cic = self.CalcFiringStrength('cic', a_Combined_OA)
			cii = self.CalcFiringStrength('cii', a_Combined_OA)
			cif = self.CalcFiringStrength('cif', a_Combined_OA)
			cfc = self.CalcFiringStrength('cfc', a_Combined_OA)
			cfi = self.CalcFiringStrength('cfi', a_Combined_OA)
			cff = self.CalcFiringStrength('cff', a_Combined_OA)
			icc = self.CalcFiringStrength('icc', a_Combined_OA)
			ici = self.CalcFiringStrength('ici', a_Combined_OA)
			icf = self.CalcFiringStrength('icf', a_Combined_OA)
			iic = self.CalcFiringStrength('iic', a_Combined_OA)
			iii = self.CalcFiringStrength('iii', a_Combined_OA)
			iif = self.CalcFiringStrength('iif', a_Combined_OA)
			ifc = self.CalcFiringStrength('ifc', a_Combined_OA)
			ifi = self.CalcFiringStrength('ifi', a_Combined_OA)
			iff = self.CalcFiringStrength('iff', a_Combined_OA)
			fcc = self.CalcFiringStrength('fcc', a_Combined_OA)
			fci = self.CalcFiringStrength('fci', a_Combined_OA)
			fcf = self.CalcFiringStrength('fcf', a_Combined_OA)
			fic = self.CalcFiringStrength('fic', a_Combined_OA)
			fii = self.CalcFiringStrength('fii', a_Combined_OA)
			fif = self.CalcFiringStrength('fif', a_Combined_OA)
			ffc = self.CalcFiringStrength('ffc', a_Combined_OA)
			ffi = self.CalcFiringStrength('ffi', a_Combined_OA)
			fff = self.CalcFiringStrength('fff', a_Combined_OA)

			f_close_OA = [ccc, cci, ccf, cic, cii, cif, cfc, cfi, cff]
			f_ideal_OA = [icc, ici, icf, iic, iii, iif, ifc, ifi, iff]
			f_far_OA   = [fcc, fci, fcf, fic, fii, fif, ffc, ffi, fff]
			f_Combined_OA = [f_close_OA, f_ideal_OA, f_far_OA]

			## Defuzzification for Right Edge Following
			# Left wheel
			numSensors_RE = 2
			leftVel_RE = self.GetDefuzzifiedSpeed(numSensors_RE, f_Combined_RE, l_RE)
			# Right wheel
			rightVel_RE = self.GetDefuzzifiedSpeed(numSensors_RE, f_Combined_RE, r_RE)
			
			## Defuzzification for Obstacle Avoidance
			# Left wheel
			numSensors_OA = 3
			leftVel_OA = self.GetDefuzzifiedSpeed(numSensors_OA, f_Combined_OA, l_OA)
			# Right wheel
			rightVel_OA = self.GetDefuzzifiedSpeed(numSensors_OA, f_Combined_OA, r_OA)

			minSensor_OA = min(sonar5, sonarFront, sonar2)
			memDegree_OA = MemF.CalcMembershipDegree(minSensor_OA, MemF.d_OA)

			minSensor_RE = min(sonar6, sonar7)
			memDegree_RE = MemF.CalcMembershipDegree(minSensor_RE, MemF.d_RE)
			memDegree_GS = MemF.CalcMembershipDegree(minSensor_RE, MemF.d_GS)
			memDegree_RE = max(memDegree_RE, memDegree_GS) * 0.8

			leftVelFinal = (memDegree_OA * leftVel_OA + memDegree_RE * leftVel_RE) / (memDegree_OA + memDegree_RE)
			leftVelFinal = round(leftVelFinal, 2)
			rightVelFinal = (memDegree_OA * rightVel_OA + memDegree_RE * rightVel_RE) / (memDegree_OA + memDegree_RE)
			rightVelFinal = round(rightVelFinal, 2)
		
			print(f"Left wheel velocity: {leftVelFinal}")
			print(f"Right wheel velocity: {rightVelFinal}")

			self.robot.setVel2(leftVelFinal, rightVelFinal) # velocity is in mm/s

			ArUtil.sleep(100)
		# Termination
		print("Shutting down...")
		self.robot.lock()
		self.robot.stop()
		self.robot.unlock()

		Aria.exit()
		return 0

###################################### MAIN #######################################
if __name__ == "__main__":
	o_Fuzzy = FuzzySystem()
	try:
		o_Fuzzy.run()
	except Exception as e:
		print(f"ERROR: {repr(e)}")
		logging.error(repr(e))
