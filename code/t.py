# Importing the required modules
import matplotlib.pyplot as plt
import numpy as np

# Creating a new figure
plt.figure(dpi=100)

# Creating an array from the list
x= np.array([1,2,3,4,5])
y= np.array([6,8,6,1,2])

# Plotting Numpy array
plt.plot(x,y,linestyle='--')

# Adding details to the plot
plt.title('Line Plot of a NumPy Array')
plt.xlabel('x-axis')
plt.ylabel('y-axis')

# Displaying the plot
plt.show()
