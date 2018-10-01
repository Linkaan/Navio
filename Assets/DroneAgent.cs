using System.Collections.Generic;
using UnityEngine;
using MLAgents;

public class DroneAgent : Agent
{
    public Transform spawnPoint;
    public Transform graphics;

    Rigidbody rBody;
    void Start()
    {
        rBody = GetComponent<Rigidbody>();
    }

    public float speed = 10;
    public float rotationSpeed = 2;

    public float windForce = 0;
    public float windFreq = 0;

    RollerAcademy academy;

    private float offsetDistance;

    private bool resetAgent = false;

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
        if (distanceToTarget < 2f)
        {
            AddReward(1.0f);
            MoveTarget();
            //Done();
        }

        Vector3 pos = Target.position;
        pos.y = this.transform.position.y;
        //float movingTowardsDot = Vector3.Dot(rBody.velocity, (pos - transform.position).normalized);
        //AddReward(0.03f * movingTowardsDot);

        //Debug.DrawRay(this.transform.position, this.transform.forward, Color.red, 2);

        //float facingDot = Vector3.Dot(transform.forward, (pos - transform.position).normalized);
        //AddReward(0.01f * facingDot);
        //Debug.Log(facingDot);

        // Time penalty
        AddReward(-0.01f);

        // Collided
        if (resetAgent || transform.position.y > 8)
        {
            AddReward(-1.0f);
            Done();
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
    }

    public void MoveTarget() {
        RaycastHit hit;

        Target.position = new Vector3(Random.value * 18 - 9 + spawnPoint.position.x,
                                      10f + +spawnPoint.position.y,
                                      Random.value * 18 - 9 + spawnPoint.position.z);

        if (Physics.Raycast(Target.position, -Vector3.up, out hit))
        {
            Target.position = new Vector3(Target.position.x, hit.point.y + 0.5f, Target.position.z);
        }
    }

    public Transform Target;
    public override void AgentReset()
    {
        if (academy.resetParameters.ContainsKey("wind_force")) {
            windForce = academy.resetParameters["wind_force"];   
        }

        if (academy.resetParameters.ContainsKey("wind_freq"))
        {
            windFreq = academy.resetParameters["wind_freq"];
        }

        if (resetAgent || transform.position.y > 8)
        {
            // The Agent fell
            this.transform.position = new Vector3(spawnPoint.position.x, 3 + spawnPoint.position.y, spawnPoint.position.z);
            this.transform.rotation = Quaternion.identity;
            this.rBody.angularVelocity = Vector3.zero;
            this.rBody.velocity = Vector3.zero;
            resetAgent = false;
        }
        else
        {
            // Move the target to a new spot
            MoveTarget();
        }
    }

    void OnCollisionEnter(Collision col) {
        resetAgent = true;
    }

    public override void CollectObservations()
    {
        RaycastHit hit;
        // Calculate relative position
        Vector3 relativePosition = Target.position - this.transform.position;

        //AddVectorObs(relativePosition.normalized);

        // Relative position
        AddVectorObs(relativePosition.normalized.x);

        if (Physics.Raycast(transform.position, -Vector3.up, out hit))
        {
            offsetDistance = hit.distance;
        }

        AddVectorObs(offsetDistance);

        AddVectorObs(relativePosition.normalized.z);

        // Distance to edges of platform
        /*
        AddVectorObs((this.transform.position.x + 5) / 5);
        AddVectorObs((this.transform.position.x - 5) / 5);
        AddVectorObs((this.transform.position.z + 5) / 5);
        AddVectorObs((this.transform.position.z - 5) / 5);*/

        // Agent velocity
        AddVectorObs(rBody.velocity);

        // Agent gyro
        AddVectorObs(transform.forward);
        AddVectorObs(transform.up);
    }
}