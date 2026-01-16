// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1',
      items: [
        'ros2-module/index',
        'ros2-module/chapter-1-ros2-embodied-intelligence',
        'ros2-module/chapter-2-nodes-topics-services',
        'ros2-module/chapter-3-python-agents-rclpy',
        'ros2-module/chapter-4-humanoid-modeling-urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2',
      items: [
        'module2/digital-twins-fundamentals',
        'module2/physics-simulation-with-gazebo',
        'module2/unity-environment-interaction',
        'module2/sensor-simulation-humanoids',
        {
          type: 'category',
          label: 'Assessments',
          items: [
            'module2/assessment-digital-twins',
            'module2/assessment-physics',
            'module2/assessment-environment',
            'module2/assessment-sensors',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3',
      items: [
        'module3/nvidia-isaac-ai-driven-robotics',
        'module3/isaac-sim-synthetic-data-generation',
        'module3/isaac-ros-vslam-perception',
        {
          type: 'category',
          label: 'Assessments',
          items: [
            'module3/assessment-isaac-fundamentals',
            'module3/assessment-isaac-sim',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4',
      items: [
        'module4/vla-foundations',
        'module4/voice-to-action',
        'module4/llm-cognitive-planning',
        'module4/autonomous-humanoid-capstone',
      ],
    },
  ],
};

export default sidebars;
