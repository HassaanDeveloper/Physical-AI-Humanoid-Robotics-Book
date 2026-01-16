using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class NavigationChallenge : MonoBehaviour
{
    [Header("Navigation Points")]
    public List<Transform> navigationWaypoints = new List<Transform>();

    [Header("Challenge Settings")]
    public float waypointRadius = 1.0f;
    public float maxTimeLimit = 120f; // 2 minutes
    public int targetWaypointCount = 5;

    [Header("UI Elements")]
    public UnityEngine.UI.Text timerText;
    public UnityEngine.UI.Text waypointCounterText;
    public UnityEngine.UI.Button restartButton;

    [Header("Robot Reference")]
    public GameObject robot;

    private int currentWaypointIndex = 0;
    private float startTime;
    private bool challengeActive = false;
    private float currentTime;

    void Start()
    {
        InitializeChallenge();
        SetupUI();
    }

    void Update()
    {
        if (challengeActive)
        {
            UpdateTimer();
            CheckWaypointReached();
        }
    }

    private void InitializeChallenge()
    {
        if (navigationWaypoints.Count == 0)
        {
            // Create waypoints dynamically if none exist
            CreateDefaultWaypoints();
        }

        currentWaypointIndex = 0;
        startTime = Time.time;
        challengeActive = true;

        HighlightCurrentWaypoint();
    }

    private void CreateDefaultWaypoints()
    {
        // Create 5 default waypoints in a star pattern
        Vector3 center = transform.position;
        float radius = 5.0f;

        for (int i = 0; i < targetWaypointCount; i++)
        {
            float angle = (2 * Mathf.PI * i) / targetWaypointCount;
            Vector3 pos = center + new Vector3(Mathf.Cos(angle) * radius, 0, Mathf.Sin(angle) * radius);

            GameObject wpObj = new GameObject($"Waypoint_{i + 1}");
            wpObj.transform.position = pos;

            // Add visual indicator
            GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.SetParent(wpObj.transform);
            sphere.transform.localPosition = Vector3.zero;
            sphere.transform.localScale = Vector3.one * 0.5f;
            sphere.GetComponent<Renderer>().material.color = Color.green;

            navigationWaypoints.Add(wpObj.transform);
        }
    }

    private void SetupUI()
    {
        if (timerText != null)
        {
            timerText.text = FormatTime(maxTimeLimit);
        }

        if (waypointCounterText != null)
        {
            waypointCounterText.text = $"Waypoints: {currentWaypointIndex}/{targetWaypointCount}";
        }

        if (restartButton != null)
        {
            restartButton.onClick.AddListener(RestartChallenge);
        }
    }

    private void UpdateTimer()
    {
        currentTime = Time.time - startTime;
        float remainingTime = maxTimeLimit - currentTime;

        if (remainingTime <= 0)
        {
            ChallengeFailed("Time limit exceeded!");
            return;
        }

        if (timerText != null)
        {
            timerText.text = FormatTime(remainingTime);
        }
    }

    private void CheckWaypointReached()
    {
        if (currentWaypointIndex >= navigationWaypoints.Count)
        {
            ChallengeCompleted();
            return;
        }

        Transform currentWp = navigationWaypoints[currentWaypointIndex];
        float distance = Vector3.Distance(robot.transform.position, currentWp.position);

        if (distance <= waypointRadius)
        {
            // Waypoint reached
            MarkWaypointAsReached(currentWp);
            currentWaypointIndex++;

            if (waypointCounterText != null)
            {
                waypointCounterText.text = $"Waypoints: {currentWaypointIndex}/{targetWaypointCount}";
            }

            if (currentWaypointIndex >= targetWaypointCount)
            {
                ChallengeCompleted();
            }
            else
            {
                HighlightCurrentWaypoint();
            }
        }
    }

    private void HighlightCurrentWaypoint()
    {
        foreach (Transform wp in navigationWaypoints)
        {
            // Reset all waypoints to green
            Renderer[] renderers = wp.GetComponentsInChildren<Renderer>();
            foreach (Renderer r in renderers)
            {
                r.material.color = Color.green;
            }
        }

        if (currentWaypointIndex < navigationWaypoints.Count)
        {
            Transform currentWp = navigationWaypoints[currentWaypointIndex];
            Renderer[] renderers = currentWp.GetComponentsInChildren<Renderer>();
            foreach (Renderer r in renderers)
            {
                r.material.color = Color.red; // Current waypoint is red
            }
        }
    }

    private void MarkWaypointAsReached(Transform waypoint)
    {
        Renderer[] renderers = waypoint.GetComponentsInChildren<Renderer>();
        foreach (Renderer r in renderers)
        {
            r.material.color = Color.blue; // Reached waypoint is blue
        }
    }

    private void ChallengeCompleted()
    {
        challengeActive = false;
        Debug.Log($"Challenge completed! Time: {FormatTime(currentTime)}");

        // Show success message
        if (timerText != null)
        {
            timerText.text = "SUCCESS!";
            timerText.color = Color.green;
        }

        // Disable robot movement or freeze it
        if (robot != null)
        {
            var rb = robot.GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.constraints = RigidbodyConstraints.FreezeAll;
            }
        }
    }

    private void ChallengeFailed(string reason)
    {
        challengeActive = false;
        Debug.LogWarning($"Challenge failed: {reason}");

        // Show failure message
        if (timerText != null)
        {
            timerText.text = "FAILED!";
            timerText.color = Color.red;
        }
    }

    private string FormatTime(float seconds)
    {
        int minutes = Mathf.FloorToInt(seconds / 60);
        int secs = Mathf.FloorToInt(seconds % 60);
        return $"{minutes:00}:{secs:00}";
    }

    public void RestartChallenge()
    {
        // Destroy existing waypoints
        foreach (Transform wp in navigationWaypoints)
        {
            if (wp != null)
            {
                DestroyImmediate(wp.gameObject);
            }
        }
        navigationWaypoints.Clear();

        // Reset robot position
        if (robot != null)
        {
            robot.transform.position = new Vector3(0, 1, 0); // Starting position
            var rb = robot.GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.velocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
                rb.constraints = RigidbodyConstraints.None; // Unfreeze for new challenge
            }
        }

        InitializeChallenge();
        SetupUI();
    }

    void OnDrawGizmos()
    {
        if (navigationWaypoints != null)
        {
            for (int i = 0; i < navigationWaypoints.Count; i++)
            {
                if (navigationWaypoints[i] != null)
                {
                    Gizmos.color = (i == currentWaypointIndex) ? Color.red : Color.green;
                    Gizmos.DrawWireSphere(navigationWaypoints[i].position, waypointRadius);

                    if (i > 0)
                    {
                        Gizmos.DrawLine(navigationWaypoints[i - 1].position, navigationWaypoints[i].position);
                    }
                }
            }
        }
    }
}