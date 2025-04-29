
public struct BoundingArea
{
	public float top;
	public float bottom;
	public float left;
	public float right;

	public BoundingArea(float left, float bottom, float right, float top)
	{
		this.left = left;
		this.bottom = bottom;
		this.right = right;
		this.top = top;
	}
}