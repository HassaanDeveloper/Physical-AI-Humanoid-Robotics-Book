# Content API Contracts: VLA Module

**Created**: 2026-01-14
**Status**: Completed
**Purpose**: API contracts for Module 4 content access and interaction

## Content Access API

### Get Chapter Content

**Endpoint**: `GET /api/module4/chapters/{chapter_id}`

**Parameters**:
```json
{
  "chapter_id": "1-4",  // Chapter identifier
  "format": "markdown|html|json",  // Response format (default: markdown)
  "include_exercises": true|false  // Include practical exercises
}
```

**Response**:
```json
{
  "success": true,
  "data": {
    "id": "1",
    "title": "Vision-Language-Action Foundations",
    "description": "Introduction to VLA systems",
    "content": "# Chapter 1: VLA Foundations...",
    "sections": [
      {
        "id": "1.1",
        "title": "VLA Overview",
        "content": "## Overview..."
      }
    ],
    "exercises": [
      {
        "id": "EX-1-1",
        "title": "System Design Exercise"
      }
    ],
    "learning_outcomes": [
      "Understand VLA architecture",
      "Identify system components"
    ],
    "prerequisites": ["Module 1", "Module 2", "Module 3"]
  },
  "timestamp": "2026-01-14T10:00:00Z"
}
```

**Error Responses**:
```json
// 404 Not Found
{
  "success": false,
  "error": "Chapter not found",
  "code": 404,
  "details": "Chapter ID '5' does not exist"
}

// 400 Bad Request
{
  "success": false,
  "error": "Invalid parameters",
  "code": 400,
  "details": "Format must be one of: markdown, html, json"
}
```

### Get Section Content

**Endpoint**: `GET /api/module4/chapters/{chapter_id}/sections/{section_id}`

**Parameters**:
```json
{
  "chapter_id": "1-4",
  "section_id": "1.1-4.3",
  "include_code": true|false  // Include code examples
}
```

**Response**:
```json
{
  "success": true,
  "data": {
    "id": "1.1",
    "title": "VLA Overview",
    "content": "## VLA Overview Content...",
    "key_concepts": ["Perception", "Cognition", "Action"],
    "code_examples": [
      {
        "id": "CODE-1-1",
        "title": "Basic VLA Pipeline",
        "language": "python",
        "code": "import vla..."
      }
    ],
    "diagrams": [
      {
        "id": "DIAG-1-1",
        "title": "VLA Architecture",
        "url": "/assets/module4/diagrams/vla-architecture.png"
      }
    ]
  }
}
```

### Get Practical Exercise

**Endpoint**: `GET /api/module4/exercises/{exercise_id}`

**Parameters**:
```json
{
  "exercise_id": "EX-1-1 to EX-4-2",
  "include_solution": true|false  // Include solution guide
}
```

**Response**:
```json
{
  "success": true,
  "data": {
    "id": "EX-2-1",
    "title": "Voice Command Processing",
    "chapter_id": "2",
    "description": "Implement voice command system...",
    "objectives": ["Integrate Whisper", "Process commands"],
    "prerequisites": ["ROS 2 installed", "Whisper setup"],
    "instructions": [
      "Create ROS 2 node",
      "Integrate Whisper",
      "Implement command parsing"
    ],
    "verification": [
      "90% command recognition",
      "<2s response time"
    ],
    "solution": "# Solution Guide...",  // Only if include_solution=true
    "difficulty": "intermediate",
    "estimated_time": 90  // minutes
  }
}
```

## Search API

### Search Module Content

**Endpoint**: `GET /api/module4/search`

**Parameters**:
```json
{
  "query": "voice recognition",  // Search term
  "limit": 10,                  // Max results (default: 10)
  "filters": {                  // Optional filters
    "chapter": ["2", "3"],
    "type": ["content", "exercise", "code"]
  }
}
```

**Response**:
```json
{
  "success": true,
  "data": {
    "results": [
      {
        "id": "2.1",
        "type": "content",
        "title": "Speech Recognition",
        "chapter": "2",
        "excerpt": "Whisper provides state-of-the-art speech recognition...",
        "score": 0.95,
        "url": "/module4/2-voice-to-action#speech-recognition"
      }
    ],
    "total": 5,
    "limit": 10,
    "offset": 0
  }
}
```

### Advanced Search

**Endpoint**: `POST /api/module4/search/advanced`

**Request Body**:
```json
{
  "query": {
    "text": "LLM planning",
    "fields": ["title", "content", "keywords"]
  },
  "filters": {
    "difficulty": ["intermediate", "advanced"],
    "chapter": ["3", "4"]
  },
  "sort": "relevance|date|popularity",
  "limit": 20
}
```

**Response**: Same as search response with additional filtering

## Assessment API

### Submit Exercise Solution

**Endpoint**: `POST /api/module4/assessments`

**Request Body**:
```json
{
  "student_id": "student123",
  "exercise_id": "EX-3-1",
  "solution": {
    "code": "def plan_task():...",
    "answers": ["answer1", "answer2"],
    "notes": "Implementation notes..."
  },
  "timestamp": "2026-01-14T12:00:00Z"
}
```

**Response**:
```json
{
  "success": true,
  "data": {
    "assessment_id": "ASSESS-12345",
    "exercise_id": "EX-3-1",
    "status": "submitted",
    "timestamp": "2026-01-14T12:00:01Z",
    "feedback": "Solution received, awaiting review"
  }
}
```

### Get Assessment Results

**Endpoint**: `GET /api/module4/assessments/{assessment_id}`

**Response**:
```json
{
  "success": true,
  "data": {
    "assessment_id": "ASSESS-12345",
    "exercise_id": "EX-3-1",
    "student_id": "student123",
    "status": "graded",
    "score": 85,
    "feedback": {
      "general": "Good implementation overall",
      "strengths": ["Clean code", "Good error handling"],
      "improvements": ["Add more comments", "Handle edge cases"],
      "rubric": {
        "correctness": 4,
        "completeness": 4,
        "code_quality": 3,
        "documentation": 3
      }
    },
    "graded_by": "instructor456",
    "graded_at": "2026-01-15T10:30:00Z"
  }
}
```

## Progress Tracking API

### Get Student Progress

**Endpoint**: `GET /api/module4/progress/{student_id}`

**Response**:
```json
{
  "success": true,
  "data": {
    "student_id": "student123",
    "module_id": "4",
    "progress": {
      "chapter_1": {
        "status": "completed",
        "completion_date": "2026-01-10",
        "score": 92
      },
      "chapter_2": {
        "status": "in_progress",
        "last_activity": "2026-01-12",
        "exercises_completed": 1
      }
    },
    "overall_completion": 45,
    "estimated_completion": "2026-01-20"
  }
}
```

### Update Progress

**Endpoint**: `POST /api/module4/progress`

**Request Body**:
```json
{
  "student_id": "student123",
  "chapter_id": "2",
  "status": "completed",
  "exercise_id": "EX-2-1",
  "score": 88,
  "timestamp": "2026-01-14T14:00:00Z"
}
```

## Error Handling

### Standard Error Format

```json
{
  "success": false,
  "error": "Error type",
  "code": HTTP_status_code,
  "message": "Human-readable error message",
  "details": "Technical details (optional)",
  "timestamp": "ISO_timestamp",
  "request_id": "unique_identifier"
}
```

### Common Error Codes

| Code | Error Type | Description |
|------|------------|-------------|
| 400 | Bad Request | Invalid parameters or malformed request |
| 401 | Unauthorized | Authentication required |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource does not exist |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Error | Server-side error |

## Rate Limiting

```json
// Rate limit response headers
{
  "X-RateLimit-Limit": 100,      // Max requests per window
  "X-RateLimit-Remaining": 95,   // Remaining requests
  "X-RateLimit-Reset": 3600      // Seconds until reset
}
```

## Authentication

### API Key Authentication

```http
GET /api/module4/chapters/1
Headers:
  Authorization: Bearer api_key_12345
  X-API-Version: 1.0
```

### Session Authentication

```http
GET /api/module4/progress/student123
Headers:
  Cookie: session_id=abc123
```

## Versioning

### API Versioning Strategy

- **URL Path**: `/v1/module4/...` (recommended)
- **Header**: `X-API-Version: 1.0`
- **Accept Header**: `Accept: application/vnd.module4.v1+json`

### Version Support

| Version | Status | Deprecation Date |
|---------|--------|------------------|
| 1.0 | Current | - |
| 0.9 | Deprecated | 2026-06-14 |

## Webhook Events

### Exercise Submission Event

```json
{
  "event": "exercise.submitted",
  "timestamp": "2026-01-14T12:00:00Z",
  "data": {
    "student_id": "student123",
    "exercise_id": "EX-2-1",
    "chapter_id": "2",
    "status": "submitted"
  }
}
```

### Assessment Completed Event

```json
{
  "event": "assessment.completed",
  "timestamp": "2026-01-15T10:30:00Z",
  "data": {
    "student_id": "student123",
    "exercise_id": "EX-2-1",
    "score": 88,
    "status": "graded"
  }
}
```

## API Contract Compliance

### Response Time SLAs

| Endpoint Type | SLA |
|---------------|-----|
| Content Access | <500ms |
| Search | <800ms |
| Assessment | <1000ms |
| Progress | <300ms |

### Availability SLAs

- **Production**: 99.9% uptime
- **Staging**: 99.5% uptime
- **Development**: Best effort

### Data Retention

- **Content**: Permanent (versioned)
- **Assessments**: 2 years
- **Progress Data**: 5 years
- **Logs**: 30 days

This API contract defines the interfaces for accessing and interacting with Module 4 content, ensuring consistent integration with learning management systems and student portals.