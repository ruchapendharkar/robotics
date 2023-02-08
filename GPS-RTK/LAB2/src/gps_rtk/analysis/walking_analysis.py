#Code to analyse walking data from RTK GPS

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
