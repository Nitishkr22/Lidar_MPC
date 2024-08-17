import math as m

def latlon_to_XY(lat0, lon0, lat1, lon1):
	R_earth = 6371000 # meters
	delta_lat = m.radians(lat1 - lat0)
	delta_lon = m.radians(lon1 - lon0)

	lat_avg = 0.5 * ( m.radians(lat1) + m.radians(lat0) )
	X = R_earth * delta_lon * m.cos(lat_avg)
	Y = R_earth * delta_lat

	return X,Y




X, Y = latlon_to_XY(17.6020193184926, 78.1266609417371, 17.6020193147673, 78.1266609520535)
print(X)
print(Y)