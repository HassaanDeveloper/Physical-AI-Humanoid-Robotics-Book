using UnityEngine;
using System.Collections;

public class RobotInteraction : MonoBehaviour
{
    [Header("Movement Settings")]
    public float moveSpeed = 5.0f;
    public float rotationSpeed = 100.0f;

    [Header("Interaction Settings")]
    public float interactionDistance = 3.0f;
    public LayerMask interactionLayer;

    [Header("UI Elements")]
    public GameObject interactionPrompt;

    private Camera mainCamera;
    private Rigidbody rb;
    private bool isInteracting = false;

    void Start()
    {
        mainCamera = Camera.main;
        rb = GetComponent<Rigidbody>();

        if (interactionPrompt != null)
        {
            interactionPrompt.SetActive(false);
        }
    }

    void Update()
    {
        HandleMovement();
        HandleInteraction();
    }

    private void HandleMovement()
    {
        if (isInteracting) return;

        float horizontal = Input.GetAxis("Horizontal");
        float vertical = Input.GetAxis("Vertical");

        Vector3 movement = new Vector3(horizontal, 0, vertical) * moveSpeed * Time.deltaTime;
        transform.Translate(movement);

        // Rotation based on mouse movement
        if (Input.GetMouseButton(1)) // Right mouse button
        {
            float mouseX = Input.GetAxis("Mouse X");
            transform.Rotate(Vector3.up, mouseX * rotationSpeed * Time.deltaTime);
        }
    }

    private void HandleInteraction()
    {
        if (Input.GetKeyDown(KeyCode.E))
        {
            Ray ray = mainCamera.ScreenPointToRay(new Vector3(Screen.width / 2, Screen.height / 2));
            RaycastHit hit;

            if (Physics.Raycast(ray, out hit, interactionDistance, interactionLayer))
            {
                StartCoroutine(InteractWithObject(hit.collider.gameObject));
            }
        }
    }

    private IEnumerator InteractWithObject(GameObject targetObject)
    {
        isInteracting = true;

        if (interactionPrompt != null)
        {
            interactionPrompt.SetActive(true);
        }

        // Perform interaction animation or action
        Debug.Log("Interacting with: " + targetObject.name);

        // Example interaction: move towards the object
        Vector3 targetPosition = targetObject.transform.position;
        targetPosition.y = transform.position.y; // Keep same height

        float elapsedTime = 0;
        float duration = 2.0f;
        Vector3 startPosition = transform.position;

        while (elapsedTime < duration)
        {
            transform.position = Vector3.Lerp(startPosition, targetPosition, elapsedTime / duration);
            elapsedTime += Time.deltaTime;
            yield return new WaitForEndOfFrame();
        }

        if (interactionPrompt != null)
        {
            interactionPrompt.SetActive(false);
        }

        isInteracting = false;
    }

    void OnDrawGizmosSelected()
    {
        if (mainCamera != null)
        {
            Gizmos.color = Color.yellow;
            Ray ray = mainCamera.ScreenPointToRay(new Vector3(Screen.width / 2, Screen.height / 2));
            Gizmos.DrawRay(ray.origin, ray.direction * interactionDistance);
        }
    }
}