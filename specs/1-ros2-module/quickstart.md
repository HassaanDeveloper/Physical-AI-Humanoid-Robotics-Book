# Quickstart Guide: Setting up the ROS 2 Module

## Prerequisites

Before starting with the ROS 2 Module, ensure you have the following installed:

- Node.js (version 18 or higher)
- npm (usually comes with Node.js)
- Git
- A code editor (VS Code recommended)

## Step 1: Initialize Docusaurus Project

1. Create a new Docusaurus project:
```bash
npx create-docusaurus@latest website classic
```

2. Navigate to the project directory:
```bash
cd website
```

## Step 2: Install Additional Dependencies (if needed)

Most dependencies come with the classic template, but verify:
```bash
npm install
```

## Step 3: Create the ROS 2 Module Directory

1. Create the docs directory structure:
```bash
mkdir -p docs/ros2-module
```

## Step 4: Add Chapter Content

Create the four chapter files in `docs/ros2-module/`:

1. `chapter-1-ros2-embodied-intelligence.md`
2. `chapter-2-nodes-topics-services.md`
3. `chapter-3-python-agents-rclpy.md`
4. `chapter-4-humanoid-modeling-urdf.md`

## Step 5: Configure Navigation

Update the `sidebars.js` file to include the new module:

```javascript
module.exports = {
  docs: [
    {
      type: 'category',
      label: 'ROS 2 Module - The Robotic Nervous System',
      items: [
        'ros2-module/chapter-1-ros2-embodied-intelligence',
        'ros2-module/chapter-2-nodes-topics-services',
        'ros2-module/chapter-3-python-agents-rclpy',
        'ros2-module/chapter-4-humanoid-modeling-urdf',
      ],
    },
    // Other existing items...
  ],
};
```

## Step 6: Run Development Server

Start the local development server:
```bash
npm run start
```

Your site will be available at `http://localhost:3000`.

## Step 7: Verify Setup

1. Check that the navigation menu shows the ROS 2 Module
2. Verify that all four chapters are accessible
3. Test that links work correctly
4. Confirm the styling matches the rest of the site

## Next Steps

1. Add content to each chapter file following the Docusaurus markdown format
2. Include code examples with proper syntax highlighting
3. Add learning objectives and exercises to each chapter
4. Test the deployment locally before pushing to GitHub