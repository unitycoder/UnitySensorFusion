using UnityEngine;
using System.Collections;

public class SensorFusion : MonoBehaviour
{
	static Vector3 accelerometer = new Vector3();
	static Vector3 gyroscope = new Vector3();
	static Quaternion filterToWorldQ = new Quaternion();
	static Quaternion inverseWorldToScreenQ = new Quaternion();
	static Quaternion worldToScreenQ = new Quaternion();
	static Quaternion originalPoseAdjustQ = new Quaternion();
	static Quaternion resetQ = new Quaternion();
	static float previousTime;

	const float PREDICTION_TIME = 0.040f;

	static int windowOrientation = 0;
	static bool isLandscape = false;

	static ComplementaryFilter filter;
	static PosePredictor posePredictor;

	public static Quaternion GetOrientation()
	{
		Quaternion orientation = filter.GetOrientation();
	
		// Predict orientation.
		Quaternion predictedQ = posePredictor.GetPrediction(orientation, gyroscope, previousTime);

        Quaternion o = filterToWorldQ;
		o *= resetQ;
		o *= predictedQ;
		o *= worldToScreenQ;

		return o;
	}
	public static void Recenter()
	{
		// Reduce to inverted yaw-only.
		resetQ = filter.GetOrientation();
		resetQ.x = 0;
		resetQ.y = 0;
		resetQ.z *= -1;

		// Take into account extra transformations in landscape mode.
		if(isLandscape)
			resetQ *= inverseWorldToScreenQ;

		// Take into account original pose.
		resetQ *= originalPoseAdjustQ;
	}
	public static void SetScreenTransform()
	{
		worldToScreenQ = new Quaternion(0, 0, 0, 1);
		switch(windowOrientation)
		{
			case 0:
				break;
			case 90:
				worldToScreenQ = Quaternion.AngleAxis(-Mathf.PI / 2, new Vector3(0, 0, 1));
				break;
			case -90:
				worldToScreenQ = Quaternion.AngleAxis(Mathf.PI / 2, new Vector3(0, 0, 1));
				break;
			case 180:
				// TODO
				break;
		}
		inverseWorldToScreenQ = Quaternion.Inverse(worldToScreenQ);
	}

	void Awake()
	{
		if(SystemInfo.supportsGyroscope)
		{
			filter = new ComplementaryFilter(.98f, PREDICTION_TIME);
			Input.gyro.enabled = true;
		}
		else
		{
			filter = new ComplementaryFilter(.5f, PREDICTION_TIME);
		}
		posePredictor = new PosePredictor(PREDICTION_TIME);

		filterToWorldQ = Quaternion.AngleAxis(-Mathf.PI / 2, new Vector3(1, 0, 0));

		originalPoseAdjustQ = Quaternion.AngleAxis(-windowOrientation * Mathf.PI / 18, new Vector3(0, 0, 1));

		SetScreenTransform();
		if(isLandscape)
			filterToWorldQ *= inverseWorldToScreenQ;

		Recenter();
	}

	void LateUpdate()
	{
        Vector3 accGravity = Input.acceleration; // INCLUDING GRAVITY

        Vector3 rotRate = Input.gyro.rotationRate;

        float time = Time.time;

		float deltaS = time - previousTime;

		accelerometer = -accGravity;
		gyroscope = rotRate;

		filter.AddAccelerationSample(accelerometer, time);
		filter.AddGyroSample(gyroscope, time);

		previousTime = time;
	}

	public class ComplementaryFilter
	{
		/*
		An implementation of a simple complementary filter, which fuses gyroscope and accelerometer data

		Accelerometer data is very noisy, but stable over the long term.
		Gyroscope data is smooth, but tends to drift over the long term.

		This fusion is relatively simple:
		1. Get orientation estimates from accelerometer by applying a low-pass filter on that data.
		2. Get orientation estimates from gyroscope by integrating over time.
		3. Combine the two estimates, weighing (1) in the long term, but (2) for the short term.
		*/

		SensorSample currentAccelSample = new SensorSample();
		SensorSample currentGyroSample = new SensorSample();
		SensorSample previousGyroSample = new SensorSample();

		// Set the quaternion to be looking in the -z direction by default.
		Quaternion filterQ = new Quaternion(1, 0, 0, 1);
		Quaternion previousFilterQ = new Quaternion();
		// Orientation based on the accelerometer.
		Quaternion accelQ = new Quaternion();
		// Whether or not the orientation has been initialized.
		bool isOrientationInitialized = false;
		// Running estimate of gravity based on the current orientation.
		Vector3 estimatedGravity = new Vector3();
		// Measured gravity based on accelerometer.
		Vector3 measuredGravity = new Vector3();

		float accelGyroFactor;
		float predictionTime;

		public ComplementaryFilter(float accelGyroFactor, float predictionTime)
		{
			this.accelGyroFactor = accelGyroFactor;
			this.predictionTime = predictionTime;
		}

		public void AddAccelerationSample(Vector3 value, float time)
		{
			currentAccelSample.value = value;
			currentAccelSample.time = time;
		}
		public void AddGyroSample(Vector3 value, float time)
		{
			currentGyroSample.value = value;
			currentGyroSample.time = time;

			float delta = time - previousGyroSample.time;

			//

			if(!isOrientationInitialized)
			{
				accelQ = AccelToQuaternion(currentAccelSample.value);
				previousFilterQ = accelQ;
				isOrientationInitialized = true;
				return;
			}

			var deltaT = currentGyroSample.time -
				previousGyroSample.time;

			// Convert gyro rotation vector to a quaternion delta.
			Quaternion gyroDeltaQ = GyroToQuaternionDelta(currentGyroSample.value, deltaT);

			// filter_1 = K * (filter_0 + gyro * dT) + (1 - K) * accel.
			filterQ = previousFilterQ;
			filterQ *= gyroDeltaQ;

			// Calculate the delta between the current estimated gravity and the real
			// gravity vector from accelerometer.
			Quaternion invFilterQ = new Quaternion();
			invFilterQ = filterQ;
			invFilterQ = Quaternion.Inverse(invFilterQ);

			estimatedGravity = new Vector3(0, 0, -1);
			estimatedGravity = ApplyQuaternion(invFilterQ, estimatedGravity);
			estimatedGravity.Normalize();

			measuredGravity = currentAccelSample.value;
			measuredGravity.Normalize();

			// Compare estimated gravity with measured gravity, get the delta quaternion
			// between the two.
			Quaternion deltaQ = new Quaternion();
			deltaQ.SetFromToRotation(estimatedGravity, measuredGravity);
			deltaQ = Quaternion.Inverse(deltaQ);

			// Calculate the SLERP target: current orientation plus the measured-estimated
			// quaternion delta.
			Quaternion targetQ = new Quaternion();
			targetQ = filterQ;
			targetQ *= deltaQ;

			// SLERP factor: 0 is pure gyro, 1 is pure accel.
			filterQ = Quaternion.Slerp(filterQ, targetQ, 1 - accelGyroFactor);
			previousFilterQ = filterQ;

			//

			previousGyroSample = currentGyroSample;
		}

		public Quaternion GetOrientation()
		{
			return this.filterQ;
		}
		public Quaternion AccelToQuaternion(Vector3 accel)
		{
			Vector3 normAccel = accel;
			normAccel.Normalize();
			Quaternion quat = new Quaternion();
			quat.SetFromToRotation(new Vector3(0, 0, -1), normAccel);
			quat = Quaternion.Inverse(quat);
			return quat;
		}
		public Quaternion GyroToQuaternionDelta(Vector3 gyro, float dt)
		{
			// Extract axis and angle from the gyroscope data.
			Quaternion quat = new Quaternion();
			Vector3 axis = gyro;
			axis.Normalize();
			quat = Quaternion.AngleAxis(gyro.magnitude * dt, axis);
			return quat;
		}
		public Vector3 ApplyQuaternion(Quaternion q, Vector3 v)
		{
			Vector3 o = new Vector3();

			// calculate quat * vector
			var ix = q.w * v.x + q.y * v.z - q.z * v.y;
			var iy = q.w * v.y + q.z * v.x - q.x * v.z;
			var iz = q.w * v.z + q.x * v.y - q.y * v.x;
			var iw = -q.x * v.x - q.y * v.y - q.z * v.z;

			// calculate result * inverse quat
			o.x = ix * q.w + iw * -q.x + iy * -q.z - iz * -q.y;
			o.y = iy * q.w + iw * -q.y + iz * -q.x - ix * -q.z;
			o.z = iz * q.w + iw * -q.z + ix * -q.y - iy * -q.x;

			return o;
		}
	}

	public class PosePredictor
	{
		/*
		Given an orientation and the gyroscope data, predicts the future orientation. This makes rendering appear faster.
		
		Also see: http://msl.cs.uiuc.edu/~lavalle/papers/LavYerKatAnt14.pdf
		*/

		float predictionTime;
		Quaternion previousQ = new Quaternion();
		float previousTime;
		Quaternion deltaQ = new Quaternion();
		Quaternion outQ = new Quaternion();

		public PosePredictor(float predictionTime)
		{
			this.predictionTime = predictionTime;
		}

		public Quaternion GetPrediction(Quaternion currentQ, Vector3 gyro, float time)
		{
            Debug.Log(currentQ);

            if(previousTime == 0)
			{
				previousQ = currentQ;
				previousTime = time;
				return currentQ;
			}

			// Calculate axis and angle based on gyroscope rotation rate data.
			Vector3 axis = new Vector3();
			axis = gyro;
			axis.Normalize();

			float angularSpeed = gyro.magnitude;

			// If we're rotating slowly, don't do prediction.
			if(angularSpeed < Mathf.Deg2Rad * 20)
			{
				outQ = currentQ;
				previousQ = currentQ;
				return outQ;
			}

			// Get the predicted angle based on the time delta and latency.
			float predictAngle = predictionTime;

			deltaQ = Quaternion.AngleAxis(predictAngle, axis);
			outQ = previousQ;
			outQ *= deltaQ;

			previousQ = currentQ;

			return outQ;
		}
	}

	public struct SensorSample
	{
		public Vector3 value;
		public float time;
	}
}