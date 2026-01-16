import React from 'react';
import styles from './SimulationDemo.module.css';

const SimulationDemo = ({ title, description, children, type = 'default' }) => {
  const containerClass = `${styles.simulationDemo} ${styles[`simulationDemo${type.charAt(0).toUpperCase() + type.slice(1)}`]}`;

  return (
    <div className={containerClass}>
      <div className={styles.simulationDemoHeader}>
        <h3 className={styles.simulationDemoTitle}>{title}</h3>
      </div>
      <div className={styles.simulationDemoContent}>
        <p className={styles.simulationDemoDescription}>{description}</p>
        <div className={styles.simulationDemoBody}>
          {children}
        </div>
      </div>
    </div>
  );
};

export default SimulationDemo;