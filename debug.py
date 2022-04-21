cell = 954200

width = 640
height = 480

z = cell//(width * height)
cell = cell - (z * width * height)
y = cell//width
x = cell%width

print(x, y, z, sep=" ")