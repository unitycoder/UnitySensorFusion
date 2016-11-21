using UnityEngine;
using System.Collections;

public class Test : MonoBehaviour {

	// Use this for initialization
	void Start () {
        SensorFusion.Recenter();
    }
	
	// Update is called once per frame
	void Update () {
        transform.rotation = SensorFusion.GetOrientation();
	}
}
