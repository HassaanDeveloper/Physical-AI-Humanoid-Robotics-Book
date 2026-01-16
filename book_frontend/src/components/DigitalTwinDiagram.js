import React from 'react';
import styles from './DigitalTwinDiagram.module.css';

const DigitalTwinDiagram = () => {
  const layers = [
    {
      name: 'Physical Layer',
      description: 'The actual robot with sensors and actuators',
      color: '#FF6B6B'
    },
    {
      name: 'Data Layer',
      description: 'Real-time data streams from sensors',
      color: '#4ECDC4'
    },
    {
      name: 'Virtual Layer',
      description: 'Simulation models and physics engines',
      color: '#45B7D1'
    },
    {
      name: 'Analytics Layer',
      description: 'Processing and optimization algorithms',
      color: '#96CEB4'
    },
    {
      name: 'Interface Layer',
      description: 'Visualization and control systems',
      color: '#FFEAA7'
    }
  ];

  return (
    <div className={styles.diagramContainer}>
      <h3 className={styles.diagramTitle}>Digital Twin Architecture</h3>
      <div className={styles.layersContainer}>
        {layers.map((layer, index) => (
          <React.Fragment key={index}>
            <div
              className={styles.layer}
              style={{ borderColor: layer.color }}
            >
              <div
                className={styles.layerHeader}
                style={{ backgroundColor: layer.color }}
              >
                <h4 className={styles.layerName}>{layer.name}</h4>
              </div>
              <div className={styles.layerContent}>
                <p className={styles.layerDescription}>{layer.description}</p>
              </div>
            </div>
            {index < layers.length - 1 && (
              <div className={styles.connection}>
                <div className={styles.connectionArrow}>↓</div>
              </div>
            )}
          </React.Fragment>
        ))}
      </div>

      <div className={styles.bidirectionalConnection}>
        <div className={styles.arrowLeft}>← Bidirectional Data Flow →</div>
      </div>

      <div className={styles.legend}>
        <p><strong>Digital Twin Architecture:</strong> The 5-layer system that connects physical robots to their virtual counterparts.</p>
      </div>
    </div>
  );
};

export default DigitalTwinDiagram;