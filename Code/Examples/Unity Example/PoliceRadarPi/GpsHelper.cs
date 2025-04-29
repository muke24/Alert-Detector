using System;

public static class GpsHelper
{
	public static float CalculateDistance(Location point1, Location point2)
	{
		float lon1 = point1.x * (float)(Math.PI / 180.0);
		float lat1 = point1.y * (float)(Math.PI / 180.0);
		float lon2 = point2.x * (float)(Math.PI / 180.0);
		float lat2 = point2.y * (float)(Math.PI / 180.0);

		float dLat = lat2 - lat1;
		float dLon = lon2 - lon1;

		float a = (float)(Math.Sin(dLat / 2) * Math.Sin(dLat / 2) +
						  Math.Cos(lat1) * Math.Cos(lat2) *
						  Math.Sin(dLon / 2) * Math.Sin(dLon / 2));
		float c = (float)(2 * Math.Atan2(Math.Sqrt(a), Math.Sqrt(1 - a)));
		float earthRadiusKm = 6371f;

		return earthRadiusKm * c;
	}
}