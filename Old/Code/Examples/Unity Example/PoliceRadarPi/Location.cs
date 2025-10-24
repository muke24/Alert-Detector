namespace WazeAlertPi;

public class Location
{
	public float x { get; set; } // Longitude
	public float y { get; set; } // Latitude

	public Location(float longitude, float latitude)
	{
		this.x = longitude;
		this.y = latitude;
	}
}