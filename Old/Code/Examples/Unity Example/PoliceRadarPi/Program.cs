using System;

class Program
{
	static void Main(string[] args)
	{
		try
		{
			using (GpsReader gpsReader = new GpsReader("/dev/serial0", 9600))
			{
				AlertReceiver alertReceiver = new AlertReceiver(gpsReader, 1f, 60f, 0.2f);
				alertReceiver.Start();
			}
		}
		catch (Exception ex)
		{
			Console.WriteLine($"Startup Error: {ex.Message}");
		}
	}
}