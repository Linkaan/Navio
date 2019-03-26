using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using MLAgents;

public class DroneAgent : Agent
{
    public Transform spawnPoint;
    public Transform graphics;

	public GameObject level0;
	public GameObject level1;

    public LayerMask layerMask;

    public GameObject iwashere;

    Rigidbody rBody;
    void Start()
    {
        rBody = GetComponent<Rigidbody>();
    }

    public float speed = 10;
    public float rotationSpeed = 2;

    public float windForce = 0;
    public float windFreq = 0;

	public int level;

    RollerAcademy academy;

    private float offsetDistance;

    private bool resetAgent = false;

	private bool reachedTarget = false;

	private bool movedTarget = false;

	float lastDistance;

    float lastTime;

    public override void InitializeAgent()
    {
        academy = FindObjectOfType<RollerAcademy>();
    }

    public override void AgentAction(float[] vectorAction, string textAction)
    {
        // Rewards
        float distanceToTarget = Vector3.Distance(this.transform.position,
                                                  Target.position);

        // Reached target
        /*if (distanceToTarget < 2f)
        {
            AddReward(1.0f);
            MoveTarget();
            //Done();
        }*/
        /*
		if (reachedTarget) {
			AddReward(1.0f);
			MoveTarget();
			reachedTarget = false;
		}*/

		/*
		float movingTowardsDot = Vector3.Dot(rBody.velocity.normalized, (Target.position - transform.position).normalized);

		Debug.DrawRay(this.transform.position, transform.forward*5, Color.red, 1);
*/
		float facingDot = Vector3.Dot(transform.forward.normalized, (Target.position - transform.position).normalized);
/*		Debug.DrawRay(this.transform.position, (Target.position - transform.position).normalized*5, Color.red, 1);
		Debug.Log (facingDot);*/

        // Time penalty
        AddReward(-0.01f);

        RaycastHit hitDown;
        if (rBody.SweepTest(-transform.up, out hitDown, 200))
        {
            if (hitDown.distance > 4 && hitDown.distance < 10) AddReward(0.005f);
            else if (hitDown.distance > 12 || hitDown.distance < 2) AddReward(-0.0075f);
        }

        Vector3 localVelocity = transform.InverseTransformDirection(rBody.velocity);

        if (!Physics.CheckSphere(transform.position, 10, layerMask)) AddReward(0.011f * Mathf.Max(0.1f, 2f * localVelocity.normalized.z - Mathf.Abs(localVelocity.normalized.x)));
        else if (Physics.OverlapSphere(transform.position, 10, layerMask).Length > 10)
        {
            resetAgent = true;
        }

        // Collided
        if (resetAgent || transform.position.y > 200 || transform.position.y < -10)
        {
			resetAgent = true;
            AddReward(-1.0f);
            Done();
        }

        if (distanceToTarget > 2000)
        {
            spawnPoint.position = RandomNavmeshLocation(1850f);
            this.transform.position = new Vector3(spawnPoint.position.x, 3 + spawnPoint.position.y, spawnPoint.position.z);
        }

        // Actions, size = 2
        /*Vector3 controlSignal = Vector3.zero;
        controlSignal.x = vectorAction[0];
        controlSignal.y = vectorAction[1];
        controlSignal.z = vectorAction[2];

        Vector3 force = (this.transform.forward * ) +
                        (this.transform.up      * vectorAction[1]) +
                        (this.transform.right   * vectorAction[2]);
        rBody.AddForce(force * speed);*/
        rBody.AddRelativeForce(Vector3.forward * vectorAction[2] * speed);
        rBody.AddRelativeForce(Vector3.up * vectorAction[1] * speed);
        rBody.AddRelativeForce(Vector3.right * vectorAction[0] * speed);

        //transform.Rotate(new Vector3(0, , 0));
        rBody.AddTorque(transform.up * vectorAction[3] * rotationSpeed);
        /*
        Vector3 forward = new Vector3(controlSignal.x, graphics.position.y, controlSignal.z);
        graphics.rotation = Quaternion.LookRotation(forward, Vector3.up);

        Vector3 euler = graphics.eulerAngles;
        euler.x = 0;
        graphics.eulerAngles = euler;*/
        /*
		if (distanceToTarget < lastDistance && movedTarget) {
			if (facingDot > 0.7f) {
				AddReward (0.06f);
			} else {
				AddReward (0.02f);
			}
			lastDistance = distanceToTarget;
		} else {
			movedTarget = true;
		}*/
    }

    Vector3[] windDirections = new Vector3[]{
        Vector3.forward,
        Vector3.up,
        Vector3.right
    };
    void FixedUpdate() {
        if (Random.value < windFreq) {
            rBody.AddTorque(transform.up * Random.Range(windForce, windForce));
        }

        if (Random.value < windFreq / 2f) {
            Vector3 randVector = windDirections[Random.Range(0, windDirections.Length)];
            rBody.AddForce(randVector * Random.Range(windForce * -10f, windForce * 10f));
        }
    }

    void Update() {
        float velocityForward = Vector3.Dot(rBody.velocity, -this.transform.right);
        float velocityRight = -Vector3.Dot(rBody.velocity, this.transform.forward);

        //graphics.rotation = Quaternion.LookRotation(rBody.velocity, Vector3.up);
        Vector3 euler = graphics.eulerAngles;
        euler.x = Mathf.Min(velocityForward * 5f, 25f);
        euler.z = Mathf.Min(velocityRight * 5f, 25f);
        graphics.eulerAngles = euler;

        if (Time.time - lastTime > 10)
        {
            lastTime = Time.time;
            Transform ts = Instantiate(iwashere).transform;
            ts.position = transform.position;
        }
    }

    public void MoveTarget() {
		do {
			RaycastHit hit;

			Target.position = new Vector3 (Random.value * 18 - 9 + spawnPoint.position.x,
				10f + +spawnPoint.position.y,
				Random.value * 18 - 9 + spawnPoint.position.z);

			if (Physics.Raycast (Target.position, -Vector3.up, out hit)) {
				float offset = level == 0 ? 4.5f : level == 1 ? Random.value * 4.5f : 0.5f;
				Target.position = new Vector3 (Target.position.x, hit.point.y + offset, Target.position.z);
			}				
		} while(Vector3.Distance (this.transform.position, Target.position) < 4f);

		lastDistance = 1000000;
		movedTarget = false;
    }

    public Transform Target;
    public override void AgentReset()
    {
        windForce = academy.resetParameters["wind_force"];
        windFreq = academy.resetParameters["wind_freq"];
		level = (int) academy.resetParameters["level"];
        /*
		if (level == 0) {
			Vector3 size = Target.gameObject.GetComponent<BoxCollider> ().size;
			size.y = 10;
			Target.gameObject.GetComponent<BoxCollider> ().size = size;
		} else {
			Vector3 size = Target.gameObject.GetComponent<BoxCollider> ().size;
			size.y = 4;
			Target.gameObject.GetComponent<BoxCollider> ().size = size;
		}

		if (level <= 1) {
			level0.SetActive (true);
			level1.SetActive (false);
		} else if (level == 2) {
			level0.SetActive (false);
			level1.SetActive (true);
		}*/

        if (resetAgent || transform.position.y > 200)
        {
            // The Agent fell
            spawnPoint.position = RandomNavmeshLocation(1850f);
            this.transform.position = new Vector3(spawnPoint.position.x, 40 + spawnPoint.position.y, spawnPoint.position.z);
            this.transform.rotation = Quaternion.identity;
            this.rBody.angularVelocity = Vector3.zero;
            this.rBody.velocity = Vector3.zero;
            resetAgent = false;
			lastDistance = 1000000;

            foreach(deletemesoon obj in FindObjectsOfType<deletemesoon>()) Destroy(obj.gameObject);
        }
        else
        {
            // Move the target to a new spot
            //MoveTarget();
        }
    }

    public Vector3 RandomNavmeshLocation(float radius)
    {
        Vector3 finalPosition = Vector3.zero;
        do
        {
            Vector3 randomDirection = Random.insideUnitSphere * radius;
            randomDirection += Target.position;
            NavMeshHit hit;
            if (NavMesh.SamplePosition(randomDirection, out hit, radius, 1))
            {
                finalPosition = hit.position;
            }
        } while (finalPosition.y > 7.75);

        return finalPosition;
    }

    /*
	void OnTriggerEnter(Collider col) {
		reachedTarget = true;
	}*/

    void OnCollisionEnter(Collision col) {
        if (!col.gameObject.CompareTag("iwashere")) resetAgent = true;
    }

    public override void CollectObservations()
    {
        RaycastHit hit;
        // Calculate relative position
        //Vector3 relativePosition = Target.position - this.transform.position;

        //AddVectorObs(relativePosition.normalized);

        // Relative position
        /*
        AddVectorObs(relativePosition.normalized.x);

        if (Physics.Raycast(transform.position, -Vector3.up, out hit))
        {
            offsetDistance = hit.distance;
        }

        AddVectorObs(offsetDistance);

        AddVectorObs(relativePosition.normalized.z);*/

        // Distance to edges of platform
        /*
        AddVectorObs((this.transform.position.x + 5) / 5);
        AddVectorObs((this.transform.position.x - 5) / 5);
        AddVectorObs((this.transform.position.z + 5) / 5);
        AddVectorObs((this.transform.position.z - 5) / 5);*/

        RaycastHit hitDown;

        for (int i = 0; i < 6; i++)
        {
            RaycastHit hitCircle;

            Vector3 rotatedVector = Quaternion.AngleAxis((180f / 5) * 2 + 90f, -transform.right) * transform.forward;
            rotatedVector = Quaternion.AngleAxis((180f / 5) * i + 90f, transform.up) * rotatedVector;
            Debug.DrawLine(transform.position + rotatedVector, transform.position + rotatedVector * 10f, Color.magenta, 1f);
            if (rBody.SweepTest(rotatedVector, out hitCircle, 200))
            {
                AddVectorObs(hitCircle.distance);
            }
            else AddVectorObs(200);
        }

        for (int i = 0; i < 6; i++)
        {
            RaycastHit hitCircle;

            Vector3 rotatedVector = Quaternion.AngleAxis((180f / 5) * i + 90f, transform.up) * -transform.forward;
            Debug.DrawLine(transform.position + rotatedVector, transform.position + rotatedVector * 10f, Color.red, 1f);
            if (rBody.SweepTest(rotatedVector, out hitCircle, 200))
            {
                AddVectorObs(hitCircle.distance);
            }
            else AddVectorObs(200);
        }

        for (int i = 0; i < 6; i++)
        {
            RaycastHit hitCircle;

            Vector3 rotatedVector = Quaternion.AngleAxis((180f / 5) * 3 + 90f, -transform.right) * transform.forward;
            rotatedVector = Quaternion.AngleAxis((180f / 5) * i + 90f, transform.up) * rotatedVector;
            Debug.DrawLine(transform.position + rotatedVector, transform.position + rotatedVector * 10f, Color.green, 1f);
            if (rBody.SweepTest(rotatedVector, out hitCircle, 200))
            {
                AddVectorObs(hitCircle.distance);
            }
            else AddVectorObs(200);
        }

        for (int i = 0; i < 6; i++)
        {
            RaycastHit hitCircle;

            Vector3 rotatedVector = Quaternion.AngleAxis((180f / 5) * 4 + 90f, -transform.right) * transform.forward;
            rotatedVector = Quaternion.AngleAxis((180f / 5) * i + 90f, transform.up) * rotatedVector;
            Debug.DrawLine(transform.position + rotatedVector, transform.position + rotatedVector * 10f, Color.blue, 1f);
            if (rBody.SweepTest(rotatedVector, out hitCircle, 200))
            {
                AddVectorObs(hitCircle.distance);
            }
            else AddVectorObs(200);
        }

        if (rBody.SweepTest(-transform.up, out hitDown, 200))
        {
            AddVectorObs(hitDown.distance);
        }
        else AddVectorObs(200);

        // Agent velocity
        AddVectorObs(rBody.velocity);

        // Agent gyro
		//AddVectorObs(Vector3.Dot(transform.forward.normalized, (Target.position - transform.position).normalized));
        //AddVectorObs(transform.up);
    }
}