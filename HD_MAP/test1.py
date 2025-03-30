from pyproj import Proj, transform

# Example UTM coordinates (easting, northing) and zone
easting = 611594.210007
northing = 1200306.526993
zone_number = 48 # Change as per your data
hemisphere = 'east'  # Use 'south' for the southern hemisphere

# Define UTM and WGS84 projections
utm_proj = Proj(proj="utm", zone=zone_number, hemisphere=hemisphere, ellps="WGS84")
wgs84_proj = Proj(proj="latlong", datum="WGS84")

# Transform UTM to lat/lon
lon, lat = transform(utm_proj, wgs84_proj, easting, northing)
print(f"Latitude: {lat}, Longitude: {lon}")
