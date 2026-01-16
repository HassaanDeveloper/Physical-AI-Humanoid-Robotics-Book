import React from 'react';
import clsx from 'clsx';
import styles from './IsaacDemo.module.css';

const IsaacDemo = ({ title, description, children, type = 'default' }) => {
  const containerClass = clsx(
    styles.isaacDemo,
    styles[`isaacDemo${type.charAt(0).toUpperCase() + type.slice(1)}`]
  );

  return (
    <div className={containerClass}>
      <div className={styles.isaacDemoHeader}>
        <h3 className={styles.isaacDemoTitle}>
          <span className={styles.isaacPlatformIcon}>I</span>
          {title}
        </h3>
      </div>
      <div className={styles.isaacDemoContent}>
        <p className={styles.isaacDemoDescription}>{description}</p>
        <div className={styles.isaacDemoBody}>
          {children}
        </div>
      </div>
    </div>
  );
};

export default IsaacDemo;