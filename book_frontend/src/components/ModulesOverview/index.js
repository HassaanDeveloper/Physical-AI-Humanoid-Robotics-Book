import React from 'react';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

const modules = [
  {
    id: 'module1',
    title: 'ROS 2 - The Robotic Nervous System',
    description: 'Foundational concepts of ROS 2 for Physical AI',
    path: '/docs/ros2-module/chapter-1-ros2-embodied-intelligence',
    icon: '🤖',
  },
  {
    id: 'module2',
    title: 'Digital Twins & Simulation',
    description: 'Environment modeling and physics simulation',
    path: '/docs/module2/digital-twins-fundamentals',
    icon: '🌍',
  },
  {
    id: 'module3',
    title: 'NVIDIA Isaac & VSLAM',
    description: 'AI-driven robotics and visual perception',
    path: '/docs/module3/nvidia-isaac-ai-driven-robotics',
    icon: '👁️',
  },
  {
    id: 'module4',
    title: 'Autonomous Humanoid Capstone',
    description: 'Integrated systems and practical applications',
    path: '/docs/module4/autonomous-humanoid-capstone',
    icon: '🧠',
  },
];

export default function ModulesOverview() {
  return (
    <section className={styles.modulesSection}>
      <div className={styles.container}>
        <h2 className={styles.sectionTitle}>Learning Modules</h2>
        <p className={styles.sectionDescription}>
          Explore the comprehensive curriculum covering Physical AI and Humanoid Robotics
        </p>

        <div className={styles.modulesGrid}>
          {modules.map((module) => (
            <div key={module.id} className={styles.moduleCard}>
              <div className={styles.moduleIcon}>{module.icon}</div>
              <h3 className={styles.moduleTitle}>{module.title}</h3>
              <p className={styles.moduleDescription}>{module.description}</p>
              <Link
                to={module.path}
                className={styles.moduleLink}
              >
                Start Module →
              </Link>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}