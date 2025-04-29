using System;
using System.IO.Ports;
using System.Threading.Tasks;

namespace WazeAlertPi;

public class GpsReader : IDisposable
{
	private SerialPort serialPort;
	private Location currentLocation;
	private object lockObject = new object();
	private bool disposed = false;
	private bool isConnected = false;

	public GpsReader(string portName = "/dev/serial0", int baudRate = 9600)
	{
		try
		{
			serialPort = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One);
			serialPort.Open();
			Console.WriteLine("GPS serial port opened successfully.");

			// Check if the GPS module is connected and sending data
			isConnected = CheckConnection();
			if (isConnected)
			{
				Console.WriteLine("GPS module is connected and communicating.");
			}
			else
			{
				Console.WriteLine("GPS module is not responding. Check connections and power.");
			}

			Task.Run(() => ReadGpsData());
		}
		catch (Exception ex)
		{
			Console.WriteLine($"Failed to open GPS serial port: {ex.Message}");
			throw;
		}
	}

	// Property to expose connection status
	public bool IsConnected => isConnected;

	private bool CheckConnection()
	{
		try
		{
			// Attempt to read a few bytes from the serial port with a timeout
			serialPort.ReadTimeout = 5000; // 5 seconds timeout
			string testData = serialPort.ReadLine(); // Try to read one line of NMEA data
			if (!string.IsNullOrEmpty(testData))
			{
				Console.WriteLine($"Received initial data from GPS module: {testData}");
				return true;
			}
			return false;
		}
		catch (Exception ex)
		{
			Console.WriteLine($"Error checking GPS connection: {ex.Message}");
			return false;
		}
	}

	private void ReadGpsData()
	{
		while (!disposed)
		{
			try
			{
				string line = serialPort.ReadLine();
				if (line.StartsWith("$GPRMC"))
				{
					var parts = line.Split(',');
					if (parts.Length > 6 && parts[2] == "A") // Active status means valid fix
					{
						float lat = ParseCoordinate(parts[3], parts[4], true);
						float lon = ParseCoordinate(parts[5], parts[6], false);
						lock (lockObject)
						{
							currentLocation = new Location(lon, lat);
						}
					}
				}
			}
			catch (Exception ex)
			{
				if (!disposed)
				{
					Console.WriteLine($"GPS Read Error: {ex.Message}");
				}
			}
		}
	}

	public Location GetCurrentLocation()
	{
		lock (lockObject)
		{
			return currentLocation != null ? new Location(currentLocation.x, currentLocation.y) : null;
		}
	}

	private float ParseCoordinate(string coord, string hemisphere, bool isLatitude)
	{
		int degreeDigits = isLatitude ? 2 : 3;
		string degStr = coord.Substring(0, degreeDigits);
		string minStr = coord.Substring(degreeDigits);
		float degrees = float.Parse(degStr);
		float minutes = float.Parse(minStr) / 60f;
		float decimalDegrees = degrees + minutes;

		if ((isLatitude && hemisphere == "S") || (!isLatitude && hemisphere == "W"))
		{
			decimalDegrees = -decimalDegrees;
		}
		return decimalDegrees;
	}

	public void Dispose()
	{
		Dispose(true);
		GC.SuppressFinalize(this);
	}

	protected virtual void Dispose(bool disposing)
	{
		if (!disposed)
		{
			if (disposing)
			{
				if (serialPort != null && serialPort.IsOpen)
				{
					serialPort.Close();
					Console.WriteLine("GPS serial port closed.");
				}
			}
			disposed = true;
		}
	}

	~GpsReader()
	{
		Dispose(false);
	}
}