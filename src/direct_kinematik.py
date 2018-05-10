import math


l1 = 350
l2 = 300
alpha = 40

beta = 89

x = l1 * math.cos(math.radians(alpha)) + l2 * math.cos(math.radians(alpha + beta))
y = l1 * math.sin(math.radians(alpha)) + l2 * math.sin(math.radians(alpha + beta))
print (x, y)