#Code to analyse stationary data from RTK GPS

import pandas as pd
import matplotlib.pyplot as plt

to_print =[5,6,8,9]

filename = input("Enter the name of the csv file: ")
df = pd.read_csv(filename, skipinitialspace=True , usecols=to_print)

df.columns = ["Latitude", "Longitude", "UTM_easting","UTM_northing"]
df.head()

lat = df["Latitude"]
long = df["Longitude"]
utm_e = df['UTM_easting']
utm_n = df['UTM_northing']


plt.scatter(lat, long)
plt.xlabel("Latitude")
plt.ylabel("Longitude")
plt.title("Latitude vs Longitude")
plt.show()

plt.scatter(utm_e,utm_n)
plt.xlabel("UTM Easting")
plt.ylabel("UTM Northing")
plt.title("UTM Easting vs UTM Northing")
plt.show()


east_range = utm_e.max() - utm_e.min()
north_range = utm_n.max() - utm_n.min()

print("The error estimation for UTM Easting is",east_range ,"m") 
print("The error estimation for UTM Northing is", north_range ,"m")


dev_easting = utm_e.std()
dev_northing = utm_n.std()
print("The standard deviation for UTM Easting is", dev_easting)
print("The standard deviation for UTM Northing is", dev_northing)