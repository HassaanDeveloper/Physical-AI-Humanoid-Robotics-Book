import React from 'react';
import styles from './IsaacArchitectureDiagram.module.css';

const IsaacArchitectureDiagram = () => {
  const components = [
    {
      id: 'isaac-sim',
      title: 'Isaac Sim',
      subtitle: 'Simulation',
      position: { top: '20%', left: '5%' },
      color: '#4a90e2'
    },
    {
      id: 'isaac-ros',
      title: 'Isaac ROS',
      subtitle: 'Perception & Inference',
      position: { top: '20%', left: '35%' },
      color: '#4a90e2'
    },
    {
      id: 'isaac-apps',
      title: 'Isaac Apps',
      subtitle: 'Reference Apps',
      position: { top: '20%', left: '65%' },
      color: '#4a90e2'
    },
    {
      id: 'isaac-sdk',
      title: 'Isaac SDK',
      subtitle: 'Development Tools',
      position: { top: '40%', left: '20%' },
      color: '#4a90e2'
    },
    {
      id: 'jetson',
      title: 'Jetson Platform',
      subtitle: 'Hardware',
      position: { top: '40%', left: '60%' },
      color: '#4a90e2'
    },
    {
      id: 'robot',
      title: 'Robot',
      subtitle: 'Physical System',
      position: { top: '65%', left: '40%' },
      color: '#4a90e2'
    }
  ];

  const connections = [
    { from: 'isaac-sim', to: 'isaac-sdk', label: 'Simulation' },
    { from: 'isaac-ros', to: 'jetson', label: 'Perception' },
    { from: 'isaac-sdk', to: 'robot', label: 'Control' },
    { from: 'jetson', to: 'robot', label: 'Navigation' },
    { from: 'isaac-apps', to: 'robot', label: 'Applications' }
  ];

  return (
    <div className={styles.diagramContainer}>
      <h3 className={styles.diagramTitle}>NVIDIA Isaac Platform Architecture</h3>
      <div className={styles.architectureCanvas}>
        {components.map(component => (
          <div
            key={component.id}
            className={styles.component}
            style={{
              ...component.position,
              backgroundColor: component.color,
              '--component-color': component.color
            }}
          >
            <div className={styles.componentTitle}>{component.title}</div>
            <div className={styles.componentSubtitle}>{component.subtitle}</div>
          </div>
        ))}

        {connections.map((connection, index) => (
          <svg
            key={index}
            className={styles.connectionSvg}
            style={{ position: 'absolute', top: 0, left: 0, width: '100%', height: '100%', pointerEvents: 'none' }}
          >
            <line
              x1={getConnectionPosition(connection.from).x}
              y1={getConnectionPosition(connection.from).y}
              x2={getConnectionPosition(connection.to).x}
              y2={getConnectionPosition(connection.to).y}
              className={styles.connectionLine}
            />
            <text
              x={(getConnectionPosition(connection.from).x + getConnectionPosition(connection.to).x) / 2}
              y={(getConnectionPosition(connection.from).y + getConnectionPosition(connection.to).y) / 2 - 10}
              className={styles.connectionLabel}
            >
              {connection.label}
            </text>
          </svg>
        ))}
      </div>
    </div>
  );

  function getConnectionPosition(componentId) {
    const component = components.find(c => c.id === componentId);
    if (!component) return { x: 0, y: 0 };

    // Convert percentage positions to pixel values (assuming 800x600 canvas)
    const leftPercent = parseFloat(component.position.left);
    const topPercent = parseFloat(component.position.top);

    return {
      x: leftPercent * 8, // 800px width * percentage
      y: topPercent * 6  // 600px height * percentage (scaled down for easier calculation)
    };
  }
};

export default IsaacArchitectureDiagram;