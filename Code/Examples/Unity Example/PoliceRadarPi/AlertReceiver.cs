using System;
using System.Net.Http;
using System.Text.Json;
using System.Threading;

namespace WazeAlertPi;

public class AlertReceiver
{
	private GpsReader gpsReader;
	private float maxDistanceKm;
	private float checkInterval;
	private float movementThreshold;
	private Location lastCheckedLocation;
	private DateTime lastCheckTime;
	private Alert[] currentAlerts;
	private readonly string baseUrl = "https://www.waze.com/live-map/api/georss";
	private readonly HttpClient httpClient;

	public Alert[] CurrentAlerts => currentAlerts;
	public Location CurrentLocation => gpsReader.GetCurrentLocation();
	public float MaxDistance => maxDistanceKm;

	public AlertReceiver(GpsReader gpsReader, float maxDistanceKm = 1f, float checkInterval = 60f, float movementThreshold = 0.2f)
	{
		this.gpsReader = gpsReader;
		this.maxDistanceKm = maxDistanceKm;
		this.checkInterval = checkInterval;
		this.movementThreshold = movementThreshold;
		this.lastCheckTime = DateTime.MinValue;
		this.httpClient = new HttpClient();
	}

	public void Start()
	{
		Console.WriteLine("Alert Receiver started.");
		while (true)
		{
			Location currentLocation = gpsReader.GetCurrentLocation();
			if (currentLocation == null)
			{
				Console.WriteLine("Waiting for GPS fix...");
				Thread.Sleep(1000);
				continue;
			}

			if (lastCheckedLocation == null)
			{
				lastCheckedLocation = currentLocation;
			}

			float timeSinceLastCheck = (float)(DateTime.Now - lastCheckTime).TotalSeconds;
			float distanceMoved = GpsHelper.CalculateDistance(lastCheckedLocation, currentLocation);

			if (timeSinceLastCheck >= checkInterval || distanceMoved > movementThreshold)
			{
				FetchPoliceData(currentLocation);
				lastCheckedLocation = currentLocation;
				lastCheckTime = DateTime.Now;
			}

			Thread.Sleep(1000); // Check every 1 second
		}
	}

	private void FetchPoliceData(Location location)
	{
		BoundingArea area = BoundingBox(location, maxDistanceKm);
		string url = $"{baseUrl}?top={area.top}&bottom={area.bottom}&left={area.left}&right={area.right}&env=row&types=alerts";

		try
		{
			var response = httpClient.GetAsync(url).Result;
			if (response.IsSuccessStatusCode)
			{
				string json = response.Content.ReadAsStringAsync().Result;
				WazeResponse data = JsonSerializer.Deserialize<WazeResponse>(json);
				if (data != null && data.alerts != null && data.alerts.Length > 0)
				{
					currentAlerts = data.alerts;
					Console.WriteLine($"Received {currentAlerts.Length} alerts at {DateTime.Now}:");
					foreach (var alert in currentAlerts)
					{
						Console.WriteLine($"- {alert.type} at ({alert.location.x}, {alert.location.y})");
					}
				}
				else
				{
					currentAlerts = null;
					Console.WriteLine("No alerts found.");
				}
			}
			else
			{
				Console.WriteLine($"HTTP Error: {response.StatusCode}");
			}
		}
		catch (Exception ex)
		{
			Console.WriteLine($"Fetch Error: {ex.Message}");
		}
	}

	private BoundingArea BoundingBox(Location location, float distanceInKm)
	{
		float latitude = location.y;
		float longitude = location.x;

		float latInRadians = latitude * (float)(Math.PI / 180.0);
		float deltaLatitude = distanceInKm / 111f;
		float deltaLongitude = distanceInKm / (111f * (float)Math.Cos(latInRadians));

		float left = longitude - deltaLongitude;
		float bottom = latitude - deltaLatitude;
		float right = longitude + deltaLongitude;
		float top = latitude + deltaLatitude;

		return new BoundingArea(left, bottom, right, top);
	}
}