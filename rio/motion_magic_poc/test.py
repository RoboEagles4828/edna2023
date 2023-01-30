import math

# 90 degrees
testVal = math.pi / 4

# go down 180 degrees
modifyVal = math.pi

# Get remainder
result = math.fmod(testVal - modifyVal, 2 * math.pi)
print(result)


# theory
moveDown = modifyVal - testVal
result2 = (2 * math.pi) - moveDown
print(result2)

# Correction for negative values
result += (2 * math.pi)
print(result)

assert result == result2